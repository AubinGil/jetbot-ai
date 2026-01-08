#!/usr/bin/env python3
"""
VILA Vision Processing Node (lightweight router)
- Keeps the same REST API (/analyze, /health)
- For inference, routes to remote Ollama (Mac) first, then falls back to local Ollama (Jetson)
  - Remote default: http://192.168.2.29:11434 model minicpm-v:8b
  - Local fallback: http://127.0.0.1:11434 model llava:latest
"""
import os
import base64
import time
from typing import Optional, Tuple

import requests
from requests.exceptions import RequestException
from flask import Flask, request, jsonify

app = Flask(__name__)

# Configuration via environment variables (with sane defaults)
# Priority: COSMOS (OpenAI-compatible API) -> REMOTE (Ollama) -> LOCAL (Ollama)
COSMOS_HOST = os.environ.get("COSMOS_HOST", "http://192.168.2.16:8001").rstrip("/")
COSMOS_MODEL = os.environ.get("COSMOS_MODEL", "nvidia/cosmos-reason1-7b")
REMOTE_HOST = os.environ.get("OLLAMA_REMOTE_HOST", "http://192.168.2.29:11434").rstrip("/")
REMOTE_MODEL = os.environ.get("OLLAMA_REMOTE_MODEL", "minicpm-v:8b")
LOCAL_HOST = os.environ.get("OLLAMA_LOCAL_HOST", "http://127.0.0.1:11434").rstrip("/")
LOCAL_MODEL = os.environ.get("OLLAMA_LOCAL_MODEL", "llava:latest")
HTTP_TIMEOUT = float(os.environ.get("OLLAMA_TIMEOUT", "30"))

ALLOWED_ACTIONS = {"FORWARD", "LEFT", "RIGHT", "STOP", "BACKWARD"}


def cosmos_openai_generate(host: str, model: str, prompt: str, image_b64: Optional[str]) -> Tuple[Optional[str], Optional[str]]:
    """Call OpenAI-compatible /v1/chat/completions endpoint (for Cosmos). Returns (response_text, error)."""
    url = f"{host}/v1/chat/completions"

    # Build messages in OpenAI format
    messages = []
    if image_b64:
        # Vision message with image
        messages.append({
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}}
            ]
        })
    else:
        # Text-only message
        messages.append({
            "role": "user",
            "content": prompt
        })

    payload = {
        "model": model,
        "messages": messages,
        "max_tokens": 512
    }

    try:
        r = requests.post(url, json=payload, timeout=HTTP_TIMEOUT)
        if r.status_code != 200:
            return None, f"HTTP {r.status_code}: {r.text[:200]}"
        data = r.json()
        # Extract response from OpenAI format
        resp = data.get("choices", [{}])[0].get("message", {}).get("content")
        if not resp:
            return None, f"No response in JSON: {list(data.keys())}"
        return resp, None
    except RequestException as e:
        return None, str(e)


def ollama_generate(host: str, model: str, prompt: str, image_b64: Optional[str]) -> Tuple[Optional[str], Optional[str]]:
    """Call Ollama generate API. Returns (response_text, error)."""
    url = f"{host}/api/generate"
    payload = {
        "model": model,
        "prompt": prompt,
        "stream": False,
    }
    if image_b64:
        payload["images"] = [image_b64]
    try:
        r = requests.post(url, json=payload, timeout=HTTP_TIMEOUT)
        if r.status_code != 200:
            return None, f"HTTP {r.status_code}: {r.text[:200]}"
        data = r.json()
        resp = data.get("response")
        return resp, None
    except RequestException as e:
        return None, str(e)


def choose_action(text: str) -> str:
    """Extract a valid action token from model text output."""
    if not text:
        return "STOP"
    up = text.strip().upper()
    # Fast path: exact match
    if up in ALLOWED_ACTIONS:
        return up
    # Heuristic: find first allowed token contained in text
    for token in ("FORWARD", "LEFT", "RIGHT", "BACKWARD", "STOP"):
        if token in up:
            return token
    return "STOP"


@app.route('/analyze', methods=['POST'])
def analyze_image():
    """Analyze an image and return a navigation command via Ollama.
    Body: {"image": <base64>, "custom_prompt": <optional>}
    Response: {"action": <FORWARD|LEFT|RIGHT|STOP|BACKWARD>, "response": <raw_model_text>, "backend": <remote|local>}
    """
    try:
        data = request.get_json(force=True)
        image_b64 = data.get('image')
        custom_prompt = data.get('custom_prompt')

        # Use custom prompt if provided, otherwise use navigation prompt
        if custom_prompt:
            prompt = custom_prompt
        else:
            prompt = (
                "Analyze this robot's view and give navigation advice.\n"
                "Respond ONLY with one of these actions:\n"
                "- FORWARD: if path is clear ahead\n"
                "- LEFT: if obstacle ahead, clear on left\n"
                "- RIGHT: if obstacle ahead, clear on right\n"
                "- STOP: if blocked or dangerous\n"
                "- BACKWARD: if stuck\n\n"
                "Just respond with the action word only."
            )

        # Try Cosmos first (if configured), then remote Ollama, then local Ollama
        text = None
        err = None
        backend = None

        if COSMOS_HOST:
            text, err = cosmos_openai_generate(COSMOS_HOST, COSMOS_MODEL, prompt, image_b64)
            backend = "cosmos"
            if text:
                print(f"Using Cosmos backend")

        if not text and (err or not COSMOS_HOST):
            if COSMOS_HOST:
                print(f"Cosmos failed ({err}); trying remote {REMOTE_MODEL}")
            text, err = ollama_generate(REMOTE_HOST, REMOTE_MODEL, prompt, image_b64)
            backend = "remote"

        if not text and err:
            print(f"Remote failed ({err}); falling back to local {LOCAL_MODEL}")
            text, err2 = ollama_generate(LOCAL_HOST, LOCAL_MODEL, prompt, image_b64)
            backend = "local"
            if err2 or not text:
                msg = f"All backends failed: cosmos={err if COSMOS_HOST else 'N/A'}, remote={err}, local={err2}"
                print(msg)
                return jsonify({'error': msg}), 502

        # Only extract action if not using custom prompt
        if custom_prompt:
            action = "NARRATION"  # Indicate this is a narration response
        else:
            action = choose_action(text)
            print(f"Decision ({backend}): {action}")

        return jsonify({'action': action, 'response': text, 'backend': backend})

    except Exception as e:
        print(f"Error: {e}")
        return jsonify({'error': str(e)}), 500


@app.route('/describe', methods=['POST'])
def describe_scene():
    """Describe what the robot sees (for narration/entertainment).
    Body: {"image": <base64>, "prompt": <description_prompt>}
    Response: {"description": <text>, "backend": <remote|local>}
    """
    try:
        data = request.get_json(force=True)
        image_b64 = data.get('image')
        prompt = data.get('prompt', "Describe what you see in this image in 1-2 entertaining sentences.")

        # Try Cosmos first (if configured), then remote Ollama, then local Ollama
        text = None
        err = None
        backend = None

        if COSMOS_HOST:
            text, err = cosmos_openai_generate(COSMOS_HOST, COSMOS_MODEL, prompt, image_b64)
            backend = "cosmos"

        if not text and (err or not COSMOS_HOST):
            if COSMOS_HOST:
                print(f"Cosmos failed ({err}); trying remote {REMOTE_MODEL}")
            text, err = ollama_generate(REMOTE_HOST, REMOTE_MODEL, prompt, image_b64)
            backend = "remote"

        if not text and err:
            print(f"Remote failed ({err}); falling back to local {LOCAL_MODEL}")
            text, err2 = ollama_generate(LOCAL_HOST, LOCAL_MODEL, prompt, image_b64)
            backend = "local"
            if err2 or not text:
                msg = f"All backends failed: cosmos={err if COSMOS_HOST else 'N/A'}, remote={err}, local={err2}"
                print(msg)
                return jsonify({'error': msg}), 502

        print(f"Narration ({backend}): {text[:100]}...")
        return jsonify({'description': text, 'response': text, 'backend': backend})

    except Exception as e:
        print(f"Error: {e}")
        return jsonify({'error': str(e)}), 500


def backend_health(host: str) -> bool:
    try:
        r = requests.get(f"{host}/api/tags", timeout=2)
        return r.status_code == 200
    except RequestException:
        return False


@app.route('/health', methods=['GET'])
def health():
    response = {'status': 'ok'}
    if COSMOS_HOST:
        # For Cosmos (OpenAI API), check /v1/models endpoint instead
        try:
            r = requests.get(f"{COSMOS_HOST}/v1/models", timeout=2)
            status_cosmos = (r.status_code == 200)
        except:
            status_cosmos = False
        response['cosmos'] = {'host': COSMOS_HOST, 'model': COSMOS_MODEL, 'reachable': status_cosmos}
    status_remote = backend_health(REMOTE_HOST)
    status_local = backend_health(LOCAL_HOST)
    response['remote'] = {'host': REMOTE_HOST, 'model': REMOTE_MODEL, 'reachable': status_remote}
    response['local'] = {'host': LOCAL_HOST, 'model': LOCAL_MODEL, 'reachable': status_local}
    return jsonify(response)


if __name__ == '__main__':
    print("Starting VILA Vision Router on port 5000...")
    if COSMOS_HOST:
        print(f"Primary: Cosmos at {COSMOS_HOST} model={COSMOS_MODEL}")
    print(f"Remote: {REMOTE_HOST} model={REMOTE_MODEL} | Local: {LOCAL_HOST} model={LOCAL_MODEL}")
    # Single-threaded to keep ordering predictable
    app.run(host='0.0.0.0', port=5000, threaded=False)
