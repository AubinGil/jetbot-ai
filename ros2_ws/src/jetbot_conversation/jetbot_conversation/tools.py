#!/usr/bin/env python3
"""
Tool/API functions for JetBot conversation system
Provides: time/date, weather, web search, calculations, etc.
"""
import requests
import json
from datetime import datetime
from typing import Dict, List, Any, Optional
import math


class ToolRegistry:
    """Registry of available tools for the LLM"""

    def __init__(self, logger=None):
        self.logger = logger
        self.tools = self._register_tools()

    def _register_tools(self) -> Dict[str, Dict]:
        """Register all available tools with their schemas"""
        return {
            "get_current_time": {
                "type": "function",
                "function": {
                    "name": "get_current_time",
                    "description": "Get the current time and date",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "timezone": {
                                "type": "string",
                                "description": "Timezone (optional, defaults to local)",
                            }
                        },
                    }
                }
            },
            "get_location": {
                "type": "function",
                "function": {
                    "name": "get_location",
                    "description": "Get the current approximate location (city/region/country) based on IP",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            },
            "get_weather": {
                "type": "function",
                "function": {
                    "name": "get_weather",
                    "description": "Get current weather for a location",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "location": {
                                "type": "string",
                                "description": "City name or location (e.g., 'London', 'New York', 'Tokyo')",
                            }
                        },
                        "required": ["location"]
                    }
                }
            },
            "search_web": {
                "type": "function",
                "function": {
                    "name": "search_web",
                    "description": "Search the web for information using DuckDuckGo",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {
                                "type": "string",
                                "description": "The search query",
                            }
                        },
                        "required": ["query"]
                    }
                }
            },
            "calculate": {
                "type": "function",
                "function": {
                    "name": "calculate",
                    "description": "Perform mathematical calculations",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "expression": {
                                "type": "string",
                                "description": "Math expression to evaluate (e.g., '2+2', '5*8', 'sqrt(16)')",
                            }
                        },
                        "required": ["expression"]
                    }
                }
            },
            "get_fact": {
                "type": "function",
                "function": {
                    "name": "get_fact",
                    "description": "Get a random interesting fact",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            }
        }

    def get_tool_schemas(self) -> List[Dict]:
        """Get all tool schemas for LLM"""
        return list(self.tools.values())

    def execute_tool(self, tool_name: str, arguments: Dict) -> str:
        """Execute a tool and return result"""
        try:
            if tool_name == "get_current_time":
                return self.get_current_time(arguments.get("timezone"))
            elif tool_name == "get_location":
                return self.get_location()
            elif tool_name == "get_weather":
                return self.get_weather(arguments.get("location"))
            elif tool_name == "search_web":
                return self.search_web(arguments.get("query"))
            elif tool_name == "calculate":
                return self.calculate(arguments.get("expression"))
            elif tool_name == "get_fact":
                return self.get_fact()
            else:
                return f"Error: Unknown tool '{tool_name}'"
        except Exception as e:
            error_msg = f"Error executing {tool_name}: {str(e)}"
            if self.logger:
                self.logger.error(error_msg)
            return error_msg

    # =========================================================================
    # TOOL IMPLEMENTATIONS
    # =========================================================================

    def get_current_time(self, timezone: Optional[str] = None) -> str:
        """Get current time and date"""
        try:
            now = datetime.now()

            # Format output
            time_str = now.strftime("%I:%M %p")  # 12-hour format
            date_str = now.strftime("%A, %B %d, %Y")  # Full date

            result = f"Current time: {time_str}\nDate: {date_str}"

            if self.logger:
                self.logger.info(f"Time query: {result}")

            return result
        except Exception as e:
            return f"Error getting time: {str(e)}"

    def get_weather(self, location: str) -> str:
        """Get weather using wttr.in service"""
        try:
            if not location:
                return "Error: Location is required"

            # Use wttr.in API (free, no key required)
            url = f"https://wttr.in/{location}?format=j1"

            if self.logger:
                self.logger.info(f"Fetching weather for: {location}")

            response = requests.get(url, timeout=5)
            response.raise_for_status()

            data = response.json()

            # Extract current weather
            current = data['current_condition'][0]
            temp_c = current['temp_C']
            temp_f = current['temp_F']
            desc = current['weatherDesc'][0]['value']
            humidity = current['humidity']
            wind_mph = current['windspeedMiles']
            feels_like_c = current['FeelsLikeC']
            feels_like_f = current['FeelsLikeF']

            # Get location name
            nearest_area = data['nearest_area'][0]
            area_name = nearest_area.get('areaName', [{}])[0].get('value', location)

            result = (
                f"Weather in {area_name}:\n"
                f"Temperature: {temp_f}°F ({temp_c}°C)\n"
                f"Feels like: {feels_like_f}°F ({feels_like_c}°C)\n"
                f"Conditions: {desc}\n"
                f"Humidity: {humidity}%\n"
                f"Wind: {wind_mph} mph"
            )

            if self.logger:
                self.logger.info(f"Weather result: {result}")

            return result

        except requests.Timeout:
            return f"Error: Request timed out while fetching weather for {location}"
        except requests.RequestException as e:
            return f"Error: Could not fetch weather data ({str(e)})"
        except (KeyError, IndexError) as e:
            return f"Error: Invalid weather data format ({str(e)})"
        except Exception as e:
            return f"Error getting weather: {str(e)}"

    def get_location(self) -> str:
        """Get approximate current location via IP geolocation"""
        try:
            # Try ipapi.co first
            url1 = "https://ipapi.co/json/"
            if self.logger:
                self.logger.info("Fetching current location from ipapi.co...")
            r1 = requests.get(url1, timeout=5)
            if r1.ok:
                j = r1.json()
                city = j.get("city")
                region = j.get("region") or j.get("region_code")
                country = j.get("country_name") or j.get("country")
                lat = j.get("latitude")
                lon = j.get("longitude")
                parts = [p for p in [city, region, country] if p]
                where = ", ".join(parts) if parts else "Unknown"
                coords = f" (lat {lat}, lon {lon})" if lat is not None and lon is not None else ""
                return f"Current location: {where}{coords}"

            # Fallback to ipinfo.io
            if self.logger:
                self.logger.info("ipapi.co failed, trying ipinfo.io...")
            r2 = requests.get("https://ipinfo.io/json", timeout=5)
            r2.raise_for_status()
            j2 = r2.json()
            city = j2.get("city")
            region = j2.get("region")
            country = j2.get("country")
            loc = j2.get("loc")  # 'lat,lon'
            coords = f" ({loc})" if loc else ""
            parts = [p for p in [city, region, country] if p]
            where = ", ".join(parts) if parts else "Unknown"
            return f"Current location: {where}{coords}"
        except requests.Timeout:
            return "Error: Request timed out while fetching location"
        except requests.RequestException as e:
            return f"Error: Could not fetch location ({str(e)})"
        except Exception as e:
            return f"Error getting location: {str(e)}"

    def search_web(self, query: str) -> str:
        """Search web using DuckDuckGo Instant Answer API"""
        try:
            if not query:
                return "Error: Search query is required"

            # Use DuckDuckGo Instant Answer API (free, no key)
            url = "https://api.duckduckgo.com/"
            params = {
                'q': query,
                'format': 'json',
                'no_html': 1,
                'skip_disambig': 1
            }

            if self.logger:
                self.logger.info(f"Searching web for: {query}")

            response = requests.get(url, params=params, timeout=5)
            response.raise_for_status()

            data = response.json()

            # Try to get the best answer
            if data.get('AbstractText'):
                result = data['AbstractText']
            elif data.get('Answer'):
                result = data['Answer']
            elif data.get('Definition'):
                result = data['Definition']
            elif data.get('RelatedTopics') and len(data['RelatedTopics']) > 0:
                # Get first related topic
                topic = data['RelatedTopics'][0]
                if isinstance(topic, dict) and 'Text' in topic:
                    result = topic['Text']
                else:
                    result = "No clear answer found. Try asking more specifically."
            else:
                result = "No information found for that query."

            # Limit response length
            if len(result) > 300:
                result = result[:297] + "..."

            if self.logger:
                self.logger.info(f"Search result: {result[:100]}...")

            return result

        except Exception as e:
            return f"Error searching web: {str(e)}"

    def calculate(self, expression: str) -> str:
        """Safely evaluate mathematical expressions"""
        try:
            if not expression:
                return "Error: Expression is required"

            # Safe evaluation with limited scope
            # Only allow math operations and functions
            safe_dict = {
                'sqrt': math.sqrt,
                'pow': math.pow,
                'sin': math.sin,
                'cos': math.cos,
                'tan': math.tan,
                'pi': math.pi,
                'e': math.e,
                'abs': abs,
                'round': round,
                'min': min,
                'max': max,
            }

            if self.logger:
                self.logger.info(f"Calculating: {expression}")

            # Remove any potentially dangerous characters
            if any(c in expression for c in ['__', 'import', 'eval', 'exec', 'open', 'file']):
                return "Error: Invalid expression"

            # Evaluate
            result = eval(expression, {"__builtins__": {}}, safe_dict)

            # Format result
            if isinstance(result, float):
                if result.is_integer():
                    result = int(result)
                else:
                    result = round(result, 6)

            answer = f"{expression} = {result}"

            if self.logger:
                self.logger.info(f"Calculation result: {answer}")

            return answer

        except SyntaxError:
            return f"Error: Invalid mathematical expression: {expression}"
        except NameError as e:
            return f"Error: Unknown function or variable in expression"
        except Exception as e:
            return f"Error calculating: {str(e)}"

    def get_fact(self) -> str:
        """Get a random interesting fact"""
        try:
            # Use uselessfacts.jsph.pl API (free, no key)
            url = "https://uselessfacts.jsph.pl/random.json?language=en"

            if self.logger:
                self.logger.info("Fetching random fact...")

            response = requests.get(url, timeout=5)
            response.raise_for_status()

            data = response.json()
            fact = data.get('text', 'No fact available')

            if self.logger:
                self.logger.info(f"Fact: {fact[:100]}...")

            return f"Here's an interesting fact: {fact}"

        except Exception as e:
            return f"Error getting fact: {str(e)}"


# Example usage
if __name__ == "__main__":
    # Test tools
    registry = ToolRegistry()

    print("=== Testing Tools ===\n")

    print("1. Current Time:")
    print(registry.get_current_time())
    print()

    print("2. Weather:")
    print(registry.get_weather("London"))
    print()

    print("3. Calculator:")
    print(registry.calculate("2 + 2"))
    print(registry.calculate("sqrt(16) * 5"))
    print()

    print("4. Web Search:")
    print(registry.search_web("Python programming"))
    print()

    print("5. Random Fact:")
    print(registry.get_fact())
