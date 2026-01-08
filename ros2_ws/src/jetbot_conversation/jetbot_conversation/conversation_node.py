#!/usr/bin/env python3
"""
Smart Conversation Pipeline Node for JetBot
Integrates: ASR (Whisper) → LLM (Ollama) → Command Router → TTS (Piper)
With: Vision understanding (VILA/LLaVA), Context memory, Nav2 integration
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.timer import Timer
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
import json
import time
import re
import requests
import subprocess
import os
from typing import Dict, List, Optional
from jetbot_conversation.tools import ToolRegistry
import numpy as np

class ConversationNode(Node):
    def __init__(self):
        super().__init__('conversation_node')
        
        # Parameters
        # Primary/Fallback LLM configuration
        # Primary now points to Cosmos (OpenAI-compatible API)
        self.declare_parameter('primary_ollama_host', 'http://192.168.2.33:8001')
        self.declare_parameter('primary_ollama_model', 'nvidia/cosmos-reason1-7b')
        self.declare_parameter('primary_ollama_vision_model', 'nvidia/cosmos-reason1-7b')
        self.declare_parameter('primary_api_type', 'openai')  # 'ollama' or 'openai'
        # Optional API key for OpenAI-compatible endpoints (e.g., NIM gateways)
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('fallback_ollama_host', 'http://localhost:11434')
        self.declare_parameter('fallback_ollama_model', 'granite3.3')
        self.declare_parameter('fallback_ollama_vision_model', 'granite3.2-vision:latest')
        self.declare_parameter('fallback_api_type', 'ollama')  # 'ollama' or 'openai'
        self.declare_parameter('primary_timeout', 30.0)  # Timeout for primary endpoint

        # Legacy parameters (for backwards compatibility)
        self.declare_parameter('ollama_host', 'http://192.168.2.29:11434')
        self.declare_parameter('ollama_model', 'cosmos/reason')
        self.declare_parameter('ollama_vision_model', 'granite3.2-vision:latest')

        self.declare_parameter('context_memory_size', 5)
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('command_timeout', 5.0)
        self.declare_parameter('gesture_cooldown_sec', 8.0)
        self.declare_parameter('language', 'en')  # Language for responses ('en' or 'fr')

        # Load parameters
        self.primary_ollama_host = self.get_parameter('primary_ollama_host').value
        self.primary_ollama_model = self.get_parameter('primary_ollama_model').value
        self.primary_ollama_vision_model = self.get_parameter('primary_ollama_vision_model').value
        self.primary_api_type = self.get_parameter('primary_api_type').value
        self.openai_api_key = self.get_parameter('openai_api_key').value or os.environ.get('OPENAI_API_KEY', '')
        self.fallback_ollama_host = self.get_parameter('fallback_ollama_host').value
        self.fallback_ollama_model = self.get_parameter('fallback_ollama_model').value
        self.fallback_ollama_vision_model = self.get_parameter('fallback_ollama_vision_model').value
        self.fallback_api_type = self.get_parameter('fallback_api_type').value
        self.primary_timeout = self.get_parameter('primary_timeout').value

        # Legacy support
        self.ollama_host = self.get_parameter('ollama_host').value
        self.ollama_model = self.get_parameter('ollama_model').value
        self.ollama_vision_model = self.get_parameter('ollama_vision_model').value

        self.context_memory_size = self.get_parameter('context_memory_size').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.gesture_cooldown = self.get_parameter('gesture_cooldown_sec').value
        self.language = self.get_parameter('language').value

        # Optional secondary remote (for text-only) and controlled auto-detect
        self.declare_parameter('secondary_ollama_host', self.primary_ollama_host)
        self.declare_parameter('secondary_ollama_model', 'llama3.1:latest')
        self.declare_parameter('secondary_api_type', self.primary_api_type)  # 'ollama' or 'openai'
        self.secondary_ollama_host = self.get_parameter('secondary_ollama_host').value
        self.secondary_ollama_model = self.get_parameter('secondary_ollama_model').value
        self.secondary_api_type = self.get_parameter('secondary_api_type').value

        # Auto-detect only if requested by setting model to 'auto'
        detected_model = None
        want_auto_text = (self.primary_ollama_model == 'auto')
        want_auto_vision = (self.primary_ollama_vision_model == 'auto')
        if want_auto_text or want_auto_vision:
            detected_model = self._detect_running_model(self.primary_ollama_host)
            if detected_model:
                self.get_logger().info(f'✨ Auto-detected model: {detected_model}')
                if want_auto_text:
                    self.primary_ollama_model = detected_model
                if want_auto_vision:
                    self.primary_ollama_vision_model = detected_model
        if not detected_model:
            self.get_logger().info(f'📌 Using configured models — text: {self.primary_ollama_model}, vision: {self.primary_ollama_vision_model}')

        # State
        self.conversation_history: List[Dict] = []
        self.current_image: Optional[np.ndarray] = None
        self.last_scan = None
        self.robot_state = "idle"  # idle, moving, navigating, listening
        self.auto_stop_timer: Optional[Timer] = None
        # Keep the latest commanded twist and republish while moving
        self.current_twist = Twist()
        self.motion_timer = self.create_timer(0.1, self.motion_tick)
        self.last_gesture_time = 0.0
        self.just_woke = False
        # Track autonomous navigation process
        self.nav_process = None
        
        # Tool/API support
        self.tool_registry = ToolRegistry(logger=self.get_logger())
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tts_pub = self.create_publisher(String, 'tts/speak', 10)
        self.response_pub = self.create_publisher(String, 'conversation/response', 10)
        
        # Subscribers
        self.create_subscription(String, 'stt/text', self.handle_speech_input, 10)
        self.create_subscription(Image, 'camera/image_raw', self.handle_image, 10)
        self.create_subscription(String, 'conversation/command', self.handle_command, 10)
        self.create_subscription(Bool, 'stt/awake', self.handle_awake_event, 10)
        self.create_subscription(String, 'gesture/events', self.handle_gesture_event, 10)
        
        # Action clients
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Command patterns (English)
        self.command_patterns_en = {
            'move_forward': [r'move forward', r'go forward', r'go ahead', r'advance'],
            'move_backward': [r'move back', r'go back', r'reverse', r'back up'],
            'turn_left': [r'turn left', r'rotate left', r'spin left'],
            'turn_right': [r'turn right', r'rotate right', r'spin right'],
            'turn_around': [r'turn 180', r'turn around', r'rotate 180', r'u-?turn'],
            'stop': [r'stop', r'halt', r'freeze', r'wait', r'quit', r'end', r'cancel'],
            'follow_me': [r'follow me', r'follow'],
            'look': [r'what do you see', r'look at', r'describe', r'what\'s there', r'see'],
            'start_autonomous_nav': [r'start navigation', r'start autonomous', r'begin navigation', r'autonomous mode'],
            'navigate': [r'go to', r'navigate to', r'move to', r'travel to'],
            'status': [r'how are you', r'status', r'what\'s your status', r'report'],
            # Vision-grounded object commands
            'vision_navigate': [r'get closer to', r'go to the', r'move to the', r'approach the',
                               r'go around', r'avoid the', r'find the', r'go towards'],
        }

        # Command patterns (French)
        self.command_patterns_fr = {
            'move_forward': [r'avance', r'va devant', r'en avant'],
            'move_backward': [r'recule', r'va en arri[èe]re', r'marche arri[èe]re'],
            'turn_left': [r'tourne [àa] gauche', r'gauche', r'pivote [àa] gauche'],
            'turn_right': [r'tourne [àa] droite', r'droite', r'pivote [àa] droite'],
            'turn_around': [r'demi-tour', r'tourne-toi', r'fais demi-tour'],
            'stop': [r'arr[êe]te', r'stop', r'stoppe', r'reste l[àa]', r'quitte', r'termine', r'annule'],
            'follow_me': [r'suis-moi', r'suis moi'],
            'look': [r'que vois-tu', r'regarde', r'd[ée]cris', r'que vois tu'],
            'start_autonomous_nav': [r'commence navigation', r'd[ée]marre navigation', r'mode autonome', r'commence [àa] naviguer'],
            'navigate': [r'va [àa]', r'navigue vers', r'd[ée]place-toi vers'],
            'status': [r'comment vas-tu', r'statut', r'[ée]tat', r'rapport'],
            # Vision-grounded object commands
            'vision_navigate': [r'approche-toi', r'va vers le', r'va vers la', r'd[ée]place-toi vers le',
                               r'contourne le', r'[ée]vite le', r'trouve le', r'trouve la'],
        }

        # Select patterns based on language
        self.command_patterns = self.command_patterns_fr if self.language == 'fr' else self.command_patterns_en
        
        self.get_logger().info('🤖 Conversation Node initialized!')
        self.get_logger().info(f'   Language: {self.language}')
        self.get_logger().info(f'   Primary Ollama: {self.primary_ollama_host}')
        if detected_model:
            self.get_logger().info(f'   🔄 Auto-detected model: {self.primary_ollama_model}')
        else:
            self.get_logger().info(f'   Primary Text Model: {self.primary_ollama_model}')
            self.get_logger().info(f'   Primary Vision Model: {self.primary_ollama_vision_model}')
        self.get_logger().info(f'   Fallback Ollama: {self.fallback_ollama_host}')
        self.get_logger().info(f'   Fallback Text Model: {self.fallback_ollama_model}')
        self.get_logger().info(f'   Fallback Vision Model: {self.fallback_ollama_vision_model}')
        self.get_logger().info(f'   Secondary LLM: {self.secondary_ollama_model} @ {self.secondary_ollama_host} (API: {self.secondary_api_type})')
        self.get_logger().info('   Gesture listener ready on gesture/events')

    def handle_awake_event(self, msg: Bool):
        """Track wake/sleep state from audio node to shape greetings."""
        if bool(msg.data):
            self.just_woke = True
            self.get_logger().info('👋 Wake word active — will keep greeting brief unless asked for long form')
        else:
            self.just_woke = False

    def handle_gesture_event(self, msg: String):
        """React to gesture events (e.g., waving hand)."""
        if msg.data.lower() != 'wave':
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_gesture_time < self.gesture_cooldown:
            self.get_logger().info(f'👋 Wave detected but cooldown active ({int(now - self.last_gesture_time)}s ago)')
            return

        self.last_gesture_time = now
        self.get_logger().info(f'👋 Wave detected! Current state: {self.robot_state}')

        # Don't interrupt autonomous navigation
        if self.robot_state == "autonomous_navigation":
            self.get_logger().info('⚠️ Ignoring wave - in autonomous navigation mode')
            return

        prompt = "Salut! Je t'ai vu faire signe. J'arrive vers toi!" if self.language == 'fr' else "Hi there! I saw you waving. I'm heading your way!"
        self.publish_response(prompt)

        # Move forward toward the person who waved
        self.get_logger().info('🚶 Moving forward toward waving person')
        response = self.execute_motion('forward')
        self.get_logger().info(f'✓ Motion command executed: {response}')
    
    def handle_speech_input(self, msg: String):
        """Process speech-to-text input"""
        text = msg.data.strip().lower()
        if not text:
            return

        self.get_logger().info(f'🎤 Heard: "{text}"')

        # Brief greeting immediately after wake word, unless explicitly long-form
        if self.just_woke and not self.is_long_form_request(text):
            if self.is_simple_greeting(text):
                self.just_woke = False
                brief = "Salut! Prêt quand tu l'es." if self.language == 'fr' else "Hi! Ready when you are."
                self.publish_response(brief)
                self.add_to_history('assistant', brief)
                return

        # Add to conversation history
        self.add_to_history('user', text)
        
        # Process the input
        response = self.process_input(text)
        
        # Publish response
        self.publish_response(response)

        # Clear just_woke after first interaction
        self.just_woke = False
    
    def handle_command(self, msg: String):
        """Handle direct command input (from web UI, etc.)"""
        self.handle_speech_input(msg)
    
    def handle_image(self, msg: Image):
        """Store latest camera image"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
    
    def detect_tool_query(self, text: str) -> tuple:
        """Detect if query requires a tool/API call"""
        text_lower = text.lower()

        # Time queries
        time_patterns = [
            r'what time is it', r'current time', r"what's the time",
            r'tell me the time', r'what day is it', r"what's the date",
            r'what is the date', r'what date is it'
        ]
        for pattern in time_patterns:
            if re.search(pattern, text_lower):
                return ('time', {})

        # Weather queries
        weather_patterns = [
            r'weather in ([\w\s]+)',
            r"what's the weather (?:in|at) ([\w\s]+)",
            r'weather (?:for|at) ([\w\s]+)',
            r"how's the weather in ([\w\s]+)",
        ]
        for pattern in weather_patterns:
            match = re.search(pattern, text_lower)
            if match:
                location = match.group(1).strip()
                return ('weather', {'location': location})

        # Generic weather (no location)
        if re.search(r"(?:what's|how's|tell me) the weather", text_lower):
            # Default to a common location or ask
            return ('weather', {'location': 'current'})

        # Web search queries
        search_patterns = [
            r'search (?:for|about) (.+)',
            r'look up (.+)',
            r'find information about (.+)',
            r'what do you know about (.+)',
            r'tell me about (.+)',
        ]
        for pattern in search_patterns:
            match = re.search(pattern, text_lower)
            if match:
                query = match.group(1).strip()
                # Don't search for navigation/movement queries
                if any(word in query for word in ['forward', 'backward', 'left', 'right', 'turn', 'move']):
                    continue
                return ('search', {'query': query})

        # Location queries (where are we / what city)
        location_patterns = [
            r'where am i',
            r'where are we',
            r'what city am i in',
            r'what city are we in',
            r'what is our location',
            r"what's our location",
            r'what country are we in',
            r'current location'
        ]
        for pattern in location_patterns:
            if re.search(pattern, text_lower):
                return ('location', {})

        # Math/calculation queries
        calc_patterns = [
            r'calculate (.+)',
            r'what is ([\d\s+\-*/().]+)',
            r'compute (.+)',
            r'solve (.+)',
        ]
        for pattern in calc_patterns:
            match = re.search(pattern, text_lower)
            if match:
                expression = match.group(1).strip()
                # Simple check if it looks like math
                if re.search(r'[\d+\-*/()]', expression):
                    return ('calculate', {'expression': expression})

        # Random fact
        if re.search(r'tell me (?:a |an )?(?:interesting )?fact', text_lower):
            return ('fact', {})

        return (None, {})

    def process_with_tools(self, text: str) -> str:
        """Process query using tools if needed, then LLM for natural response"""
        # Check if this needs a tool
        long_form = self.is_long_form_request(text)
        tool_type, tool_args = self.detect_tool_query(text)

        if tool_type is None:
            # No tool needed, use regular LLM
            return self.query_llm(text)

        # Execute the tool
        self.get_logger().info(f'🔧 Using tool: {tool_type} with args: {tool_args}')

        if tool_type == 'time':
            tool_result = self.tool_registry.get_current_time()
        elif tool_type == 'weather':
            location = tool_args.get('location', 'current')
            if location == 'current':
                # Try to get current city via location tool
                loc_info = self.tool_registry.get_location()
                if isinstance(loc_info, str) and not loc_info.startswith('Error'):
                    # Expect format: "Current location: City, Region, Country (lat, lon)"
                    try:
                        after_colon = loc_info.split(':', 1)[1].strip()
                        city = after_colon.split(',')[0].strip()
                        if not city or city.lower() == 'unknown':
                            return "Which location would you like weather for?"
                        tool_result = self.tool_registry.get_weather(city)
                    except Exception:
                        return "Which location would you like weather for?"
                else:
                    return "Which location would you like weather for?"
            else:
                tool_result = self.tool_registry.get_weather(location)
        elif tool_type == 'search':
            tool_result = self.tool_registry.search_web(tool_args['query'])
        elif tool_type == 'calculate':
            tool_result = self.tool_registry.calculate(tool_args['expression'])
        elif tool_type == 'fact':
            tool_result = self.tool_registry.get_fact()
        elif tool_type == 'location':
            tool_result = self.tool_registry.get_location()
        else:
            tool_result = f"Unknown tool type: {tool_type}"

        # If tool returned error, return it directly
        if tool_result.startswith("Error"):
            return tool_result

        # Give result to LLM to formulate a natural response
        length_instruction = "Feel free to expand into a narrative of 4-6 sentences." if long_form else "Keep it brief (1-2 sentences)."
        prompt = f"The user asked: '{text}'\n\nI got this information: {tool_result}\n\nPlease respond naturally and conversationally, incorporating this information. {length_instruction}"

        response = self.query_llm(prompt)
        return response

    def process_input(self, text: str) -> str:
        """Smart processing pipeline"""

        # 1. Check for direct motion commands
        command_type = self.detect_command(text)

        if command_type == 'move_forward':
            return self.execute_motion('forward')
        elif command_type == 'move_backward':
            return self.execute_motion('backward')
        elif command_type == 'turn_left':
            return self.execute_motion('left')
        elif command_type == 'turn_right':
            return self.execute_motion('right')
        elif command_type == 'turn_around':
            return self.execute_turnaround()
        elif command_type == 'stop':
            return self.execute_motion('stop')

        # 2. Follow me command - brief acknowledgment only, no vision description
        elif command_type == 'follow_me':
            return "D'accord!" if self.language == 'fr' else "Okay!"

        # 3. Vision query
        elif command_type == 'look':
            return self.process_vision_query(text)

        # 4. Start autonomous navigation
        elif command_type == 'start_autonomous_nav':
            return self.start_autonomous_navigation()

        # 5. Navigation command (Nav2 location-based)
        elif command_type == 'navigate':
            return self.process_navigation_command(text)

        # 6. Status query
        elif command_type == 'status':
            return self.get_status_report()

        # 7. Vision-grounded navigation (e.g., "get closer to the shoes")
        elif command_type == 'vision_navigate':
            return self.handle_vision_grounded_command(text)

        # 8. General conversation - use tools if needed, then LLM
        else:
            return self.process_with_tools(text)

    def is_simple_greeting(self, text: str) -> bool:
        patterns = [
            r'^hi$', r'^hello$', r'^hey$', r'^yo$',
            r'^(hi|hello|hey)[\s!,\.]*$',
            r'^(good\s+(morning|afternoon|evening))$',
            r'^(hi|hello|hey) there$'
        ]
        return any(re.search(p, text, re.IGNORECASE) for p in patterns)

    def is_long_form_request(self, text: str) -> bool:
        triggers = [
            r'tell me (a )?(tale|story)',
            r'can we have a conversation',
            r'let\'s have a conversation',
            r'talk at length',
            r'explain in detail',
            r'give me (a )?long answer',
            r'let\'s chat'
        ]
        return any(re.search(p, text, re.IGNORECASE) for p in triggers)
    
    def detect_command(self, text: str) -> Optional[str]:
        """Detect command type from text"""
        for cmd_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text, re.IGNORECASE):
                    return cmd_type
        return None
    
    def execute_motion(self, direction: str) -> str:
        """Execute motion command"""
        twist = Twist()

        if self.language == 'fr':
            # French responses
            if direction == 'forward':
                twist.linear.x = self.linear_speed
                response = f"J'avance à {self.linear_speed} mètres par seconde"
            elif direction == 'backward':
                twist.linear.x = -self.linear_speed
                response = "Je recule"
            elif direction == 'left':
                twist.angular.z = self.angular_speed
                response = "Je tourne à gauche"
            elif direction == 'right':
                twist.angular.z = -self.angular_speed
                response = "Je tourne à droite"
            elif direction == 'stop':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                # Also stop autonomous navigation if running
                if self.nav_process is not None:
                    self._stop_autonomous_navigation()
                    response = "J'arrête la navigation autonome"
                else:
                    response = "Je m'arrête"
            else:
                return "Je ne comprends pas cette commande de mouvement"
        else:
            # English responses
            if direction == 'forward':
                twist.linear.x = self.linear_speed
                response = f"Moving forward at {self.linear_speed} meters per second"
            elif direction == 'backward':
                twist.linear.x = -self.linear_speed
                response = f"Moving backward"
            elif direction == 'left':
                twist.angular.z = self.angular_speed
                response = f"Turning left"
            elif direction == 'right':
                twist.angular.z = -self.angular_speed
                response = f"Turning right"
            elif direction == 'stop':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                # Also stop autonomous navigation if running
                if self.nav_process is not None:
                    self._stop_autonomous_navigation()
                    response = "Stopping autonomous navigation"
                else:
                    response = "Stopping"
            else:
                return "I don't understand that motion command"

        # Publish and store command; continue publishing while moving
        self.current_twist = twist
        self.cmd_vel_pub.publish(self.current_twist)
        self.robot_state = "moving" if direction != 'stop' else "idle"

        if direction != 'stop':
            self.schedule_auto_stop()
        else:
            self.cancel_auto_stop_timer()

        self.get_logger().info(f'🚗 {response}')
        return response

    def execute_turnaround(self) -> str:
        """Rotate approximately 180° based on angular_speed."""
        # Compute duration for ~180° turn
        if self.angular_speed <= 0:
            return "Je ne peux pas tourner car la vitesse angulaire est zéro." if self.language == 'fr' else "I cannot turn because angular speed is zero."
        duration = (np.pi) / float(self.angular_speed)
        # Command rotation (turn left by default)
        self.current_twist = Twist()
        self.current_twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(self.current_twist)
        self.robot_state = "moving"
        # Use a custom auto-stop matching the computed duration (with small margin)
        self.schedule_auto_stop(duration + 0.2)
        self.get_logger().info(f'🔄 Turning around (~180°) for {duration:.2f}s')
        return "Je fais demi-tour" if self.language == 'fr' else "Turning around"
    
    def auto_stop(self):
        """Safety auto-stop"""
        if self.robot_state == "moving":
            self.execute_motion('stop')
            self.get_logger().info('⏱️ Auto-stopped after timeout')

    def schedule_auto_stop(self, duration: Optional[float] = None):
        """Arm the auto-stop timer. If duration is provided, use it; otherwise default."""
        self.cancel_auto_stop_timer()

        def timer_callback():
            self.auto_stop_timer = None
            self.auto_stop()

        timeout = float(duration) if (duration is not None and duration > 0) else float(self.command_timeout)
        self.auto_stop_timer = self.create_timer(timeout, timer_callback)

    def cancel_auto_stop_timer(self):
        """Stop the auto-stop timer if one is running."""
        if self.auto_stop_timer is not None:
            self.auto_stop_timer.cancel()
            self.auto_stop_timer = None

    def motion_tick(self):
        """Continuously publish the last motion command while in moving state."""
        if self.robot_state == "moving":
            self.cmd_vel_pub.publish(self.current_twist)
    
    def _try_openai_request(self, host: str, model: str, prompt: str, image_b64: Optional[str], timeout: float, is_primary: bool = True) -> Optional[dict]:
        """Helper to try an OpenAI-compatible API request with error handling.
        Tries multiple common endpoint paths for compatibility (chat/completions, responses).
        Optionally includes Authorization header from param/env.
        """
        try:
            base = host.rstrip('/')

            # Build messages in OpenAI format
            messages = []
            if image_b64:
                messages.append({
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}}
                    ]
                })
            else:
                messages.append({
                    "role": "user",
                    "content": prompt
                })

            payload_chat = {
                "model": model,
                "messages": messages,
                "max_tokens": 512
            }

            headers = {"Content-Type": "application/json"}
            if getattr(self, 'openai_api_key', ''):
                headers["Authorization"] = f"Bearer {self.openai_api_key}"

            # Candidate endpoints in order of likelihood
            candidates = [
                f"{base}/v1/chat/completions",
                f"{base}/chat/completions",
                f"{base}/v1/responses",
                f"{base}/responses",
            ]

            for idx, url in enumerate(candidates):
                try:
                    # Adjust payload for the Responses API if used
                    if url.endswith('/responses'):
                        # Responses API: input is the same messages list under "input" with a user role
                        # Many OpenAI-compatible servers accept "input" as a string; we pass messages for safety
                        payload = {"model": model, "input": messages, "max_output_tokens": 512}
                    else:
                        payload = payload_chat

                    response = requests.post(url, json=payload, headers=headers, timeout=timeout)
                    if response.status_code == 200:
                        data = response.json()
                        # Normalize to {"response": text}
                        if 'choices' in data:
                            resp_text = data.get("choices", [{}])[0].get("message", {}).get("content")
                        elif 'output' in data:
                            # Some Responses API variants return {output: [{content: [{text: {...}}]}]}
                            out = data.get('output')
                            if isinstance(out, list) and out:
                                content = out[0].get('content') or []
                                # Grab first text segment
                                if content and isinstance(content, list):
                                    txt = content[0].get('text') or {}
                                    resp_text = txt.get('content') if isinstance(txt, dict) else txt
                                else:
                                    resp_text = None
                            else:
                                resp_text = None
                        else:
                            # Some servers return {message: {...}}
                            resp_text = data.get('message', {}).get('content')

                        if resp_text:
                            return {"response": resp_text}
                        # If empty, keep trying other endpoints
                    elif response.status_code in (404, 405):
                        # Try next candidate if not found or method not allowed
                        continue
                    else:
                        self.get_logger().warning(f'{"Primary" if is_primary else "Fallback"} OpenAI API returned status {response.status_code} at {url}')
                        return None
                except Exception as inner_e:
                    # Try the next candidate endpoint
                    continue

            # If all candidates failed
            self.get_logger().warning(f'{"Primary" if is_primary else "Fallback"} OpenAI-compatible endpoints not available at {base}')
            return None
        except Exception as e:
            self.get_logger().warning(f'{"Primary" if is_primary else "Fallback"} OpenAI API request failed: {e}')
            return None

    def _try_ollama_request(self, endpoint: str, payload: dict, timeout: float, is_primary: bool = True) -> Optional[dict]:
        """Helper to try an Ollama request with error handling"""
        try:
            response = requests.post(endpoint, json=payload, timeout=timeout)
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().warning(f'{"Primary" if is_primary else "Fallback"} Ollama returned status {response.status_code}')
                return None
        except Exception as e:
            self.get_logger().warning(f'{"Primary" if is_primary else "Fallback"} Ollama request failed: {e}')
            return None

    def _detect_running_model(self, host: str) -> Optional[str]:
        """Detect which model is currently running on the Ollama server"""
        try:
            response = requests.get(f'{host}/api/ps', timeout=5.0)
            if response.status_code == 200:
                data = response.json()
                # /api/ps returns {"models": [{"name": "model:tag", ...}, ...]}
                models = data.get('models', [])
                if models:
                    model_name = models[0].get('name', None)
                    self.get_logger().info(f'🔍 Detected running model: {model_name}')
                    return model_name
                else:
                    self.get_logger().info('No models currently running on server')
                    return None
            else:
                self.get_logger().warning(f'Failed to query running models: status {response.status_code}')
                return None
        except Exception as e:
            self.get_logger().warning(f'Failed to detect running models: {e}')
            return None

    def process_vision_query(self, text: str) -> str:
        """Process vision-related queries using VILA/LLaVA with primary/fallback"""
        if self.current_image is None:
            return "Je n'ai pas encore d'image de la caméra. Laisse-moi regarder d'abord." if self.language == 'fr' else "I don't have a camera image yet. Let me look around first."

        self.get_logger().info('👁️ Processing vision query...')

        # Encode image to base64
        import cv2
        import base64
        _, buffer = cv2.imencode('.jpg', self.current_image)
        img_base64 = base64.b64encode(buffer).decode('utf-8')

        # Try primary endpoint first
        self.get_logger().info(f'Trying primary vision model: {self.primary_ollama_vision_model} @ {self.primary_ollama_host} (API: {self.primary_api_type})')

        if self.primary_api_type == 'openai':
            primary_result = self._try_openai_request(
                self.primary_ollama_host,
                self.primary_ollama_vision_model,
                text,
                img_base64,
                timeout=self.primary_timeout,
                is_primary=True
            )
        else:
            primary_result = self._try_ollama_request(
                f'{self.primary_ollama_host}/api/generate',
                {
                    'model': self.primary_ollama_vision_model,
                    'prompt': text,
                    'images': [img_base64],
                    'stream': False
                },
                timeout=self.primary_timeout,
                is_primary=True
            )

        if primary_result:
            answer = primary_result.get('response', 'I see something but cannot describe it.')
            self.get_logger().info('✓ Primary vision model responded')
            self.add_to_history('assistant', answer)
            return answer

        # Fallback to local endpoint
        self.get_logger().info(f'Falling back to local vision model: {self.fallback_ollama_vision_model} @ {self.fallback_ollama_host} (API: {self.fallback_api_type})')

        if self.fallback_api_type == 'openai':
            fallback_result = self._try_openai_request(
                self.fallback_ollama_host,
                self.fallback_ollama_vision_model,
                text,
                img_base64,
                timeout=60,
                is_primary=False
            )
        else:
            fallback_result = self._try_ollama_request(
                f'{self.fallback_ollama_host}/api/generate',
                {
                    'model': self.fallback_ollama_vision_model,
                    'prompt': text,
                    'images': [img_base64],
                    'stream': False
                },
                timeout=60,
                is_primary=False
            )

        if fallback_result:
            answer = fallback_result.get('response', 'I see something but cannot describe it.')
            self.get_logger().info('✓ Fallback vision model responded')
            self.add_to_history('assistant', answer)
            return answer

        return "My vision system is not responding."
    
    def process_navigation_command(self, text: str) -> str:
        """Process navigation commands"""
        # Extract location/coordinates (simple pattern matching)
        # You can make this smarter with NLP or predefined locations
        
        # For now, simple hardcoded locations
        locations = {
            'kitchen': (2.0, 1.0, 0.0),
            'bedroom': (3.0, -1.5, 0.0),
            'living room': (1.0, 0.5, 0.0),
            'charging station': (0.0, 0.0, 0.0),
        }
        
        for location, coords in locations.items():
            if location in text.lower():
                return self.navigate_to_point(coords[0], coords[1], coords[2], location)
        
        return "I'm not sure where you want me to go. Try: kitchen, bedroom, living room, or charging station."
    
    def navigate_to_point(self, x: float, y: float, yaw: float, location: str) -> str:
        """Navigate to a specific point using Nav2"""
        if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
            return "Navigation system is not available."
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = np.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = np.cos(yaw / 2.0)
        
        self.robot_state = "navigating"
        self.nav_action_client.send_goal_async(goal_msg)
        
        self.get_logger().info(f'🗺️ Navigating to {location} at ({x}, {y})')
        return f"Navigating to {location}. I'll let you know when I arrive."

    def start_autonomous_navigation(self) -> str:
        """Start autonomous obstacle avoidance navigation"""
        try:
            # Stop any existing navigation first
            if self.nav_process is not None:
                self._stop_autonomous_navigation()

            script_path = os.path.expanduser('~/start_obstacle_nav_gpu.sh')

            if not os.path.exists(script_path):
                self.get_logger().error(f'Navigation script not found: {script_path}')
                return "Sorry, I can't find the navigation script. Please check the installation."

            self.get_logger().info('🚀 Starting autonomous obstacle avoidance navigation...')

            # Launch the script as a subprocess we can track and kill
            self.nav_process = subprocess.Popen(
                ['bash', script_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new process group for easy cleanup
            )

            self.robot_state = "autonomous_navigation"
            response = "D'accord, je démarre la navigation autonome avec évitement d'obstacles." if self.language == 'fr' else "Okay, starting autonomous obstacle avoidance navigation."
            self.get_logger().info(f'✓ Navigation script launched (PID: {self.nav_process.pid})')
            return response

        except Exception as e:
            self.get_logger().error(f'Failed to start autonomous navigation: {e}')
            return "Sorry, I encountered an error starting the navigation system."

    def _stop_autonomous_navigation(self):
        """Stop the autonomous navigation process"""
        if self.nav_process is None:
            return

        self.get_logger().info(f'🛑 Stopping autonomous navigation (PID: {self.nav_process.pid})')

        try:
            # Kill the process group to stop all child processes
            os.killpg(os.getpgid(self.nav_process.pid), signal.SIGTERM)

            # Wait for it to terminate
            try:
                self.nav_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # Force kill if it doesn't stop gracefully
                self.get_logger().warning('Navigation process did not stop gracefully, force killing...')
                os.killpg(os.getpgid(self.nav_process.pid), signal.SIGKILL)
                self.nav_process.wait()

            self.get_logger().info('✓ Autonomous navigation stopped')

        except ProcessLookupError:
            # Process already dead
            self.get_logger().info('Navigation process already terminated')
        except Exception as e:
            self.get_logger().error(f'Error stopping navigation: {e}')
        finally:
            self.nav_process = None
            self.robot_state = "idle"

    def get_status_report(self) -> str:
        """Generate status report"""
        status = f"I'm currently {self.robot_state}. "

        if self.current_image is not None:
            status += "My camera is working. "
        else:
            status += "I don't have camera feed yet. "

        status += f"I remember our last {len(self.conversation_history)} exchanges."

        return status

    def handle_vision_grounded_command(self, text: str) -> str:
        """Handle vision-grounded navigation commands like 'get closer to the shoes'"""
        if self.current_image is None:
            return "Je n'ai pas encore d'image de la caméra." if self.language == 'fr' else "I don't have a camera image yet."

        self.get_logger().info(f'🎯 Vision-grounded command: "{text}"')

        # Query Cosmos with structured prompt for spatial guidance
        if self.language == 'fr':
            prompt = f"""Commande utilisateur: {text}

Analyse l'image de la caméra et fournis des instructions de navigation:
1. Vois-tu l'objet cible?
2. Où est-il par rapport au robot? (gauche/droite/devant, distance estimée)
3. Que doit faire le robot? (tourner direction/angle, puis avancer distance)

Réponds dans ce format exact:
OBJET: [nom de l'objet]
VISIBLE: [oui/non]
POSITION: [description de la position]
ACTION: [commande précise comme "tourner à droite 20 degrés puis avancer 2 mètres"]
CONFIANCE: [haute/moyenne/faible]"""
        else:
            prompt = f"""User command: {text}

Analyze the camera view and provide navigation guidance:
1. Do you see the target object?
2. Where is it relative to the robot? (left/right/ahead, estimated distance)
3. What should the robot do? (turn direction/angle, then move distance)

Respond in this exact format:
TARGET: [object name]
VISIBLE: [yes/no]
POSITION: [position description]
ACTION: [specific command like "turn right 20 degrees then forward 2 meters"]
CONFIDENCE: [high/medium/low]"""

        # Get vision response from Cosmos
        try:
            response = self.process_vision_query(prompt)
            self.get_logger().info(f'📍 Cosmos guidance: {response}')

            # Parse the spatial guidance
            nav_plan = self.parse_spatial_guidance(response)

            if nav_plan is None:
                return "Je n'ai pas pu comprendre les instructions de navigation." if self.language == 'fr' else "I couldn't understand the navigation instructions."

            # Execute the navigation plan
            return self.execute_navigation_plan(nav_plan, response)

        except Exception as e:
            self.get_logger().error(f'Vision-grounded command failed: {e}')
            return "Désolé, j'ai eu un problème." if self.language == 'fr' else "Sorry, I had a problem."

    def parse_spatial_guidance(self, response: str) -> Optional[dict]:
        """Parse Cosmos's spatial guidance into actionable navigation plan"""
        try:
            lines = response.strip().split('\n')
            plan = {
                'target': None,
                'visible': False,
                'position': None,
                'action': None,
                'confidence': 'low',
                'actions': []
            }

            for line in lines:
                line = line.strip()
                if line.startswith('TARGET:') or line.startswith('OBJET:'):
                    plan['target'] = line.split(':', 1)[1].strip()
                elif line.startswith('VISIBLE:'):
                    visible_text = line.split(':', 1)[1].strip().lower()
                    plan['visible'] = ('yes' in visible_text or 'oui' in visible_text)
                elif line.startswith('POSITION:'):
                    plan['position'] = line.split(':', 1)[1].strip()
                elif line.startswith('ACTION:'):
                    plan['action'] = line.split(':', 1)[1].strip()
                elif line.startswith('CONFIDENCE:') or line.startswith('CONFIANCE:'):
                    conf_text = line.split(':', 1)[1].strip().lower()
                    if 'high' in conf_text or 'haute' in conf_text:
                        plan['confidence'] = 'high'
                    elif 'medium' in conf_text or 'moyenne' in conf_text:
                        plan['confidence'] = 'medium'

            # If no structured format, try to extract action from free text
            if not plan['action'] and not plan['visible']:
                # Look for navigation keywords in the response
                response_lower = response.lower()
                if 'turn' in response_lower or 'tourne' in response_lower:
                    plan['action'] = response
                    plan['visible'] = True  # Assume object is visible if giving directions

            return plan if plan['action'] else None

        except Exception as e:
            self.get_logger().error(f'Failed to parse spatial guidance: {e}')
            return None

    def execute_navigation_plan(self, plan: dict, full_response: str) -> str:
        """Execute the navigation plan from Cosmos"""
        if not plan['visible']:
            if self.language == 'fr':
                return f"Je ne vois pas {plan['target'] or 'la cible'}."
            else:
                return f"I don't see {plan['target'] or 'the target'}."

        # Extract turn and movement from action
        action = plan['action'].lower()
        self.get_logger().info(f'🎯 Executing action: {action}')

        # Simple execution: detect turn direction
        twist = Twist()

        if 'left' in action or 'gauche' in action:
            # Turn left
            twist.angular.z = self.angular_speed
            duration = 1.0  # Default 1 second turn

            # Try to extract angle
            import re
            angle_match = re.search(r'(\d+)\s*(?:degrees|degrés|°)', action)
            if angle_match:
                angle = float(angle_match.group(1))
                # Rough calculation: ~60 degrees per second at angular_speed
                duration = angle / 60.0

            self.cmd_vel_pub.publish(twist)
            self.robot_state = "moving"

            # Schedule stop
            if self.auto_stop_timer:
                self.auto_stop_timer.cancel()
            self.auto_stop_timer = self.create_timer(duration, self.auto_stop_motion)

            response = f"Je tourne à gauche vers {plan['target']}." if self.language == 'fr' else f"Turning left towards {plan['target']}."

        elif 'right' in action or 'droite' in action:
            # Turn right
            twist.angular.z = -self.angular_speed
            duration = 1.0

            import re
            angle_match = re.search(r'(\d+)\s*(?:degrees|degrés|°)', action)
            if angle_match:
                angle = float(angle_match.group(1))
                duration = angle / 60.0

            self.cmd_vel_pub.publish(twist)
            self.robot_state = "moving"

            if self.auto_stop_timer:
                self.auto_stop_timer.cancel()
            self.auto_stop_timer = self.create_timer(duration, self.auto_stop_motion)

            response = f"Je tourne à droite vers {plan['target']}." if self.language == 'fr' else f"Turning right towards {plan['target']}."

        elif 'forward' in action or 'avance' in action or 'ahead' in action:
            # Move forward
            twist.linear.x = self.linear_speed * 0.5  # Slower for safety
            duration = 2.0

            # Try to extract distance
            import re
            dist_match = re.search(r'(\d+(?:\.\d+)?)\s*(?:meters|mètres|m\b)', action)
            if dist_match:
                distance = float(dist_match.group(1))
                # Rough calculation: distance / speed
                duration = distance / (self.linear_speed * 0.5)

            self.cmd_vel_pub.publish(twist)
            self.robot_state = "moving"

            if self.auto_stop_timer:
                self.auto_stop_timer.cancel()
            self.auto_stop_timer = self.create_timer(duration, self.auto_stop_motion)

            response = f"J'avance vers {plan['target']}." if self.language == 'fr' else f"Moving towards {plan['target']}."

        else:
            # Couldn't parse specific action, provide feedback
            response = f"{plan['target']} est {plan['position']}." if self.language == 'fr' else f"{plan['target']} is {plan['position']}."

        return response

    def build_system_prompt(self, long_form: bool) -> str:
        """Construct the system prompt with optional long-form allowance."""
        if self.language == 'fr':
            # French system prompt
            instructions = [
                "Tu es un assistant robot utile nommé JetBot. Tu peux:",
                "- Te déplacer (avancer, reculer, tourner à gauche/droite)",
                "- Naviguer vers des endroits",
                "- Voir et décrire ton environnement",
                "- Avoir des conversations naturelles",
            ]

            if long_form:
                instructions.append("Sois amical et serviable; quand l'utilisateur demande explicitement une histoire ou une explication plus longue, réponds avec une réponse plus riche et narrative (4-6 phrases).")
            else:
                instructions.append("Sois concis, amical et serviable. Garde les réponses en moins de 3 phrases.")
        else:
            # English system prompt
            instructions = [
                "You are a helpful robot assistant named JetBot. You can:",
                "- Move around (forward, backward, turn left/right)",
                "- Navigate to locations",
                "- See and describe your environment",
                "- Have natural conversations",
            ]

            if long_form:
                instructions.append("Be friendly and helpful; when the user explicitly asks for a story or longer explanation, respond with a richer, more narrative answer (4-6 sentences).")
            else:
                instructions.append("Be concise, friendly, and helpful. Keep responses under 3 sentences.")

        return "\n".join(instructions)
    
    def query_llm(self, text: str) -> str:
        """Query Ollama LLM with conversation context and primary/fallback"""

        # Build context-aware prompt
        context = self.build_context()
        long_form_request = self.is_long_form_request(text)
        system_prompt = self.build_system_prompt(long_form_request)

        prompt = f"{system_prompt}\n\nConversation history:\n{context}\n\nUser: {text}\nJetBot:"

        # Try primary endpoint first
        self.get_logger().info(f'Trying primary LLM: {self.primary_ollama_model} @ {self.primary_ollama_host} (API: {self.primary_api_type})')

        if self.primary_api_type == 'openai':
            primary_result = self._try_openai_request(
                self.primary_ollama_host,
                self.primary_ollama_model,
                prompt,
                None,  # No image for text-only
                timeout=self.primary_timeout,
                is_primary=True
            )
        else:
            primary_result = self._try_ollama_request(
                f'{self.primary_ollama_host}/api/generate',
                {
                    'model': self.primary_ollama_model,
                    'prompt': prompt,
                    'stream': False,
                    'options': {
                        'temperature': 0.7,
                        'num_ctx': 1024,
                    }
                },
                timeout=self.primary_timeout,
                is_primary=True
            )

        if primary_result:
            answer = primary_result.get('response', 'I am not sure how to respond.').strip()
            self.get_logger().info('✓ Primary LLM responded')
            self.add_to_history('assistant', answer)
            return answer

        # Secondary remote endpoint (e.g., server: llama3.1:latest)
        self.get_logger().info(f'Trying secondary LLM: {self.secondary_ollama_model} @ {self.secondary_ollama_host} (API: {self.secondary_api_type})')

        if self.secondary_api_type == 'openai':
            secondary_result = self._try_openai_request(
                self.secondary_ollama_host,
                self.secondary_ollama_model,
                prompt,
                None,
                timeout=self.primary_timeout,
                is_primary=False
            )
        else:
            secondary_result = self._try_ollama_request(
                f'{self.secondary_ollama_host}/api/generate',
                {
                    'model': self.secondary_ollama_model,
                    'prompt': prompt,
                    'stream': False,
                    'options': {
                        'temperature': 0.7,
                        'num_ctx': 1024,
                    }
                },
                timeout=self.primary_timeout,
                is_primary=False
            )

        if secondary_result:
            answer = secondary_result.get('response', 'I am not sure how to respond.').strip()
            self.get_logger().info('✓ Secondary LLM responded')
            self.add_to_history('assistant', answer)
            return answer

        # Fallback to local endpoint
        self.get_logger().info(f'Falling back to local LLM: {self.fallback_ollama_model} @ {self.fallback_ollama_host} (API: {self.fallback_api_type})')

        if self.fallback_api_type == 'openai':
            fallback_result = self._try_openai_request(
                self.fallback_ollama_host,
                self.fallback_ollama_model,
                prompt,
                None,  # No image for text-only
                timeout=60,
                is_primary=False
            )
        else:
            fallback_result = self._try_ollama_request(
                f'{self.fallback_ollama_host}/api/generate',
                {
                    'model': self.fallback_ollama_model,
                    'prompt': prompt,
                    'stream': False,
                    'options': {
                        'temperature': 0.7,
                        'num_ctx': 1024,
                    }
                },
                timeout=60,
                is_primary=False
            )

        if fallback_result:
            answer = fallback_result.get('response', 'I am not sure how to respond.').strip()
            self.get_logger().info('✓ Local fallback LLM responded')
            self.add_to_history('assistant', answer)
            return answer

        return "My language processing is offline."
    
    def build_context(self) -> str:
        """Build conversation context from history"""
        context_lines = []
        for entry in self.conversation_history[-self.context_memory_size:]:
            role = "You" if entry['role'] == 'user' else "JetBot"
            context_lines.append(f"{role}: {entry['content']}")
        return "\n".join(context_lines)
    
    def add_to_history(self, role: str, content: str):
        """Add to conversation history"""
        self.conversation_history.append({
            'role': role,
            'content': content,
            'timestamp': time.time()
        })
        
        # Trim history to max size
        if len(self.conversation_history) > self.context_memory_size * 2:
            self.conversation_history = self.conversation_history[-self.context_memory_size * 2:]
    
    def publish_response(self, response: str):
        """Publish response to TTS and response topic"""
        msg = String()
        msg.data = response
        
        self.tts_pub.publish(msg)
        self.response_pub.publish(msg)
        
        self.get_logger().info(f'💬 Response: "{response}"')

def main(args=None):
    rclpy.init(args=args)
    node = ConversationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
