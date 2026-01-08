#!/usr/bin/env python3
"""
ROS 2 Robot Face Display Node
Animated robot eyes with personality, controllable via ROS 2 topics and services

Topics:
  /robot_face/expression (std_msgs/String) - Set face expression
  /robot_face/gaze (std_msgs/String) - Control eye gaze direction

Controls (when terminal is focused):
  SPACE - Change mood/expression
  Arrow keys - Move eye position
  I - Toggle idle mode (auto eye movement)
  C - Toggle curiosity mode
  S - Toggle sweat drops
  L - Laugh animation
  B - Blink
  F - Toggle confused animation
  ESC - Exit
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import math
import random
import sys
import threading
from dataclasses import dataclass
from enum import Enum

# Initialize Pygame
pygame.init()

# Display settings
FULLSCREEN = True
if FULLSCREEN:
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    WIDTH, HEIGHT = screen.get_size()
else:
    WIDTH, HEIGHT = 1024, 600
    screen = pygame.display.set_mode((WIDTH, HEIGHT))

pygame.display.set_caption("JetBot RoboEyes - ROS 2")

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
CYAN = (0, 255, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 150, 255)
PURPLE = (150, 0, 255)

clock = pygame.time.Clock()
FPS = 60  # Higher FPS for smoother animations

class Mood(Enum):
    HAPPY = "happy"
    ANGRY = "angry"
    TIRED = "tired"
    SURPRISED = "surprised"
    DEFAULT = "default"
    SCANNING = "scanning"

class Position(Enum):
    CENTER = (0, 0)
    N = (0, -1)
    NE = (1, -1)
    E = (1, 0)
    SE = (1, 1)
    S = (0, 1)
    SW = (-1, 1)
    W = (-1, 0)
    NW = (-1, -1)

@dataclass
class EyeState:
    openness: float = 1.0  # 0.0 to 1.0
    width: float = 70.0
    height: float = 90.0
    position_x: float = 0.0
    position_y: float = 0.0
    pupil_x: float = 0.0
    pupil_y: float = 0.0
    border_radius: int = 15

@dataclass
class RobotState:
    mood: Mood = Mood.DEFAULT
    left_eye: EyeState = None
    right_eye: EyeState = None
    eye_spacing: int = 240
    auto_blink: bool = True
    idle_mode: bool = False
    curiosity: bool = False
    sweat: bool = False

    def __post_init__(self):
        if self.left_eye is None:
            self.left_eye = EyeState()
        if self.right_eye is None:
            self.right_eye = EyeState()

# Animation state
class AnimationState:
    def __init__(self):
        self.transition_progress = 1.0
        self.target_state = None
        self.blink_timer = 0
        self.auto_blink_timer = random.randint(120, 300)
        self.idle_timer = random.randint(90, 240)
        self.laugh_timer = 0
        self.confused_timer = 0
        self.sweat_drops = []


def lerp(start, end, t):
    """Linear interpolation for smooth transitions"""
    return start + (end - start) * t

def ease_in_out(t):
    """Smooth easing function"""
    return t * t * (3.0 - 2.0 * t)

def draw_rounded_rect(surface, color, rect, radius, width=0):
    """Draw a rounded rectangle"""
    x, y, w, h = rect
    pygame.draw.rect(surface, color, (x + radius, y, w - 2*radius, h), width)
    pygame.draw.rect(surface, color, (x, y + radius, w, h - 2*radius), width)
    pygame.draw.circle(surface, color, (x + radius, y + radius), radius, width)
    pygame.draw.circle(surface, color, (x + w - radius, y + radius), radius, width)
    pygame.draw.circle(surface, color, (x + radius, y + h - radius), radius, width)
    pygame.draw.circle(surface, color, (x + w - radius, y + h - radius), radius, width)

def draw_eye(center_x, center_y, eye_state, color=CYAN):
    """Draw a single animated eye"""
    w = eye_state.width
    h = eye_state.height * eye_state.openness

    if eye_state.openness < 0.1:
        # Fully closed - draw line
        pygame.draw.line(screen, color,
                        (int(center_x - w/2), int(center_y)),
                        (int(center_x + w/2), int(center_y)), 4)
        return

    # Eye position offset
    pos_x = center_x + eye_state.position_x
    pos_y = center_y + eye_state.position_y

    # Draw eye outline
    eye_rect = (int(pos_x - w/2), int(pos_y - h/2), int(w), int(h))
    draw_rounded_rect(screen, color, eye_rect, eye_state.border_radius, 3)

    # Draw pupil
    if eye_state.openness > 0.3:
        pupil_size = int(15 * eye_state.openness)
        pupil_x = int(pos_x + eye_state.pupil_x)
        pupil_y = int(pos_y + eye_state.pupil_y)

        # Pupil bounds
        max_offset = int((w/2 - pupil_size - 5) * eye_state.openness)
        pupil_x = max(pos_x - max_offset, min(pos_x + max_offset, pupil_x))
        pupil_y = max(pos_y - max_offset, min(pos_y + max_offset, pupil_y))

        # White of eye
        inner_rect = (int(pos_x - w/2 + 5), int(pos_y - h/2 + 5),
                     int(w - 10), int(h - 10))
        draw_rounded_rect(screen, color, inner_rect, eye_state.border_radius - 3, 0)

        # Pupil
        pygame.draw.circle(screen, BLACK, (pupil_x, pupil_y), pupil_size)

        # Highlight
        pygame.draw.circle(screen, WHITE, (pupil_x + 4, pupil_y - 4),
                          max(3, pupil_size // 3))

def apply_mood(robot, mood):
    """Apply mood settings to robot eyes"""
    if mood == Mood.HAPPY:
        robot.left_eye.height = 80
        robot.right_eye.height = 80
        robot.left_eye.width = 70
        robot.right_eye.width = 70
    elif mood == Mood.ANGRY:
        robot.left_eye.height = 60
        robot.right_eye.height = 60
        robot.left_eye.width = 80
        robot.right_eye.width = 80
    elif mood == Mood.TIRED:
        robot.left_eye.height = 50
        robot.right_eye.height = 50
        robot.left_eye.openness = 0.5
        robot.right_eye.openness = 0.5
    elif mood == Mood.SURPRISED:
        robot.left_eye.height = 110
        robot.right_eye.height = 110
        robot.left_eye.width = 80
        robot.right_eye.width = 80
    else:  # DEFAULT
        robot.left_eye.height = 90
        robot.right_eye.height = 90
        robot.left_eye.width = 70
        robot.right_eye.width = 70

def set_eye_position(robot, position):
    """Set eye gaze direction"""
    if isinstance(position, Position):
        offset_x, offset_y = position.value
    else:
        offset_x, offset_y = position
    robot.left_eye.pupil_x = offset_x * 15
    robot.left_eye.pupil_y = offset_y * 15
    robot.right_eye.pupil_x = offset_x * 15
    robot.right_eye.pupil_y = offset_y * 15

def draw_sweat_drop(x, y, size, alpha):
    """Draw animated sweat drop"""
    s = pygame.Surface((size*2, size*3), pygame.SRCALPHA)
    color = (*CYAN[:3], int(255 * alpha))
    pygame.draw.circle(s, color, (size, size), size)
    pygame.draw.polygon(s, color, [
        (size, size*2),
        (size - size//2, size),
        (size + size//2, size)
    ])
    screen.blit(s, (int(x - size), int(y - size)))

def draw_scanning_effect():
    """Draw scanning animation overlay"""
    angle = pygame.time.get_ticks() / 1000.0
    for i in range(8):
        a = angle + i * (math.pi / 4)
        end_x = int(WIDTH/2 + math.cos(a) * 400)
        end_y = int(HEIGHT/2 + math.sin(a) * 300)
        pygame.draw.line(screen, GREEN, (WIDTH//2, HEIGHT//2),
                        (end_x, end_y), 1)

def draw_mood_indicator(robot):
    """Draw mouth/mood indicator"""
    mouth_y = HEIGHT // 2 + 100

    if robot.mood == Mood.HAPPY:
        pygame.draw.arc(screen, CYAN, (WIDTH//2 - 80, mouth_y - 20, 160, 60),
                       -math.pi, 0, 4)
    elif robot.mood == Mood.ANGRY:
        pygame.draw.arc(screen, RED, (WIDTH//2 - 80, mouth_y - 40, 160, 60),
                       0, math.pi, 4)
    elif robot.mood == Mood.TIRED:
        pygame.draw.line(screen, CYAN, (WIDTH//2 - 60, mouth_y),
                        (WIDTH//2 + 60, mouth_y), 3)
    elif robot.mood == Mood.SURPRISED:
        pygame.draw.circle(screen, CYAN, (WIDTH//2, mouth_y), 25, 3)
    elif robot.mood == Mood.SCANNING:
        font = pygame.font.Font(None, 32)
        text = font.render("SCANNING...", True, GREEN)
        screen.blit(text, (WIDTH//2 - text.get_width()//2, mouth_y - 10))

def update_animations(robot, anim):
    """Update all animation states"""
    # Auto blink
    if robot.auto_blink:
        anim.auto_blink_timer -= 1
        if anim.auto_blink_timer <= 0:
            anim.blink_timer = 8
            anim.auto_blink_timer = random.randint(120, 300)

    # Blink animation
    if anim.blink_timer > 0:
        anim.blink_timer -= 1
        progress = anim.blink_timer / 8.0
        if progress > 0.5:
            openness = (progress - 0.5) * 2
        else:
            openness = (1 - progress) * 2
        robot.left_eye.openness = openness
        robot.right_eye.openness = openness
    else:
        if robot.mood != Mood.TIRED:
            robot.left_eye.openness = 1.0
            robot.right_eye.openness = 1.0
        else:
            robot.left_eye.openness = 0.5
            robot.right_eye.openness = 0.5

    # Idle mode - random eye movements
    if robot.idle_mode:
        anim.idle_timer -= 1
        if anim.idle_timer <= 0:
            positions = list(Position)
            new_pos = random.choice(positions)
            set_eye_position(robot, new_pos)
            anim.idle_timer = random.randint(90, 240)

    # Laugh animation
    if anim.laugh_timer > 0:
        anim.laugh_timer -= 1
        offset = math.sin(anim.laugh_timer * 0.5) * 10
        robot.left_eye.position_y = offset
        robot.right_eye.position_y = offset
    else:
        if anim.confused_timer == 0:
            robot.left_eye.position_y = 0
            robot.right_eye.position_y = 0

    # Confused animation
    if anim.confused_timer > 0:
        anim.confused_timer -= 1
        offset = math.sin(anim.confused_timer * 0.3) * 15
        robot.left_eye.position_x = offset
        robot.right_eye.position_x = -offset
    else:
        if anim.laugh_timer == 0:
            robot.left_eye.position_x = 0
            robot.right_eye.position_x = 0

    # Sweat drops
    if robot.sweat:
        if random.random() < 0.03:
            anim.sweat_drops.append({
                'x': random.randint(WIDTH//4, 3*WIDTH//4),
                'y': HEIGHT//4,
                'speed': random.uniform(2, 4),
                'alpha': 1.0
            })

        for drop in anim.sweat_drops[:]:
            drop['y'] += drop['speed']
            drop['alpha'] -= 0.01
            if drop['y'] > HEIGHT or drop['alpha'] <= 0:
                anim.sweat_drops.remove(drop)

    # Curiosity - eyes get wider when looking to sides
    if robot.curiosity:
        pupil_offset = abs(robot.left_eye.pupil_x)
        if pupil_offset > 10:
            extra_height = (pupil_offset - 10) * 2
            robot.left_eye.height = 90 + extra_height
            robot.right_eye.height = 90 + extra_height

def draw_frame(robot, anim):
    """Draw the complete frame"""
    screen.fill(BLACK)

    # Scanning effect
    if robot.mood == Mood.SCANNING:
        draw_scanning_effect()

    # Sweat drops
    for drop in anim.sweat_drops:
        draw_sweat_drop(drop['x'], drop['y'], 8, drop['alpha'])

    # Eyes
    left_x = WIDTH // 2 - robot.eye_spacing // 2
    right_x = WIDTH // 2 + robot.eye_spacing // 2
    eye_y = HEIGHT // 2 - 30

    color = GREEN if robot.mood == Mood.SCANNING else CYAN
    if robot.mood == Mood.ANGRY:
        color = RED

    draw_eye(left_x, eye_y, robot.left_eye, color)
    draw_eye(right_x, eye_y, robot.right_eye, color)

    # Mood indicator
    draw_mood_indicator(robot)

    # UI
    font_small = pygame.font.Font(None, 20)
    status = f"Mood: {robot.mood.value.upper()} | Idle: {'ON' if robot.idle_mode else 'OFF'} | Curiosity: {'ON' if robot.curiosity else 'OFF'}"
    status_text = font_small.render(status, True, CYAN)
    screen.blit(status_text, (10, 10))

    controls = "ROS2 Controlled | SPACE:Mood | I:Idle | C:Curiosity | S:Sweat | L:Laugh | F:Confused | ESC:Exit"
    hint_text = font_small.render(controls, True, CYAN)
    screen.blit(hint_text, (10, HEIGHT - 25))


class RobotFaceNode(Node):
    def __init__(self):
        super().__init__('robot_face_node')

        self.robot = RobotState()
        self.anim = AnimationState()

        # Apply initial mood
        apply_mood(self.robot, self.robot.mood)

        # Create subscriptions using String messages
        self.expression_sub = self.create_subscription(
            String,
            'robot_face/expression',
            self.expression_callback,
            10
        )

        self.gaze_sub = self.create_subscription(
            String,
            'robot_face/gaze',
            self.gaze_callback,
            10
        )

        self.get_logger().info('Robot Face Node initialized')
        self.get_logger().info('Subscribed to:')
        self.get_logger().info('  /robot_face/expression (std_msgs/String)')
        self.get_logger().info('  /robot_face/gaze (std_msgs/String)')

        # Start pygame loop in separate thread
        self.running = True
        self.pygame_thread = threading.Thread(target=self.pygame_loop)
        self.pygame_thread.daemon = True
        self.pygame_thread.start()

    def expression_callback(self, msg):
        """Handle expression changes from ROS topic"""
        expression = msg.data.lower()
        self.get_logger().info(f'Received expression: {expression}')

        # Map string to Mood enum
        mood_map = {
            'default': Mood.DEFAULT,
            'happy': Mood.HAPPY,
            'angry': Mood.ANGRY,
            'tired': Mood.TIRED,
            'surprised': Mood.SURPRISED,
            'scanning': Mood.SCANNING
        }

        if expression in mood_map:
            self.robot.mood = mood_map[expression]
            apply_mood(self.robot, self.robot.mood)
        else:
            self.get_logger().warning(f'Unknown expression: {expression}')

    def gaze_callback(self, msg):
        """Handle gaze direction changes from ROS topic"""
        gaze = msg.data.lower()
        self.get_logger().info(f'Received gaze: {gaze}')

        # Map string to Position enum
        position_map = {
            'center': Position.CENTER,
            'n': Position.N,
            'ne': Position.NE,
            'e': Position.E,
            'se': Position.SE,
            's': Position.S,
            'sw': Position.SW,
            'w': Position.W,
            'nw': Position.NW
        }

        if gaze in position_map:
            set_eye_position(self.robot, position_map[gaze])
            self.robot.idle_mode = False
        else:
            self.get_logger().warning(f'Unknown gaze position: {gaze}')

    def pygame_loop(self):
        """Main pygame rendering loop"""
        moods = [Mood.DEFAULT, Mood.HAPPY, Mood.SURPRISED, Mood.TIRED,
                 Mood.ANGRY, Mood.SCANNING]
        positions = [Position.CENTER, Position.N, Position.NE, Position.E,
                    Position.SE, Position.S, Position.SW, Position.W, Position.NW]

        current_mood_idx = 0

        while self.running and rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False

                    elif event.key == pygame.K_SPACE:
                        current_mood_idx = (current_mood_idx + 1) % len(moods)
                        self.robot.mood = moods[current_mood_idx]
                        apply_mood(self.robot, self.robot.mood)

                    elif event.key == pygame.K_i:
                        self.robot.idle_mode = not self.robot.idle_mode

                    elif event.key == pygame.K_c:
                        self.robot.curiosity = not self.robot.curiosity
                        if not self.robot.curiosity:
                            self.robot.left_eye.height = 90
                            self.robot.right_eye.height = 90

                    elif event.key == pygame.K_s:
                        self.robot.sweat = not self.robot.sweat
                        if not self.robot.sweat:
                            self.anim.sweat_drops.clear()

                    elif event.key == pygame.K_l:
                        self.anim.laugh_timer = 60

                    elif event.key == pygame.K_f:
                        self.anim.confused_timer = 60

                    elif event.key == pygame.K_b:
                        self.anim.blink_timer = 8

                    elif event.key == pygame.K_UP:
                        set_eye_position(self.robot, Position.N)
                        self.robot.idle_mode = False

                    elif event.key == pygame.K_DOWN:
                        set_eye_position(self.robot, Position.S)
                        self.robot.idle_mode = False

                    elif event.key == pygame.K_LEFT:
                        set_eye_position(self.robot, Position.W)
                        self.robot.idle_mode = False

                    elif event.key == pygame.K_RIGHT:
                        set_eye_position(self.robot, Position.E)
                        self.robot.idle_mode = False

            # Update animations
            update_animations(self.robot, self.anim)

            # Draw frame
            draw_frame(self.robot, self.anim)

            pygame.display.flip()
            clock.tick(FPS)

        pygame.quit()
        self.get_logger().info('Pygame loop ended')


def main(args=None):
    rclpy.init(args=args)

    node = RobotFaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
