"""Keyboard teleop for the hexapod: publishes /cmd_vel, /stance_height_command
and /gesture_command.

A terminal can't report key releases, so movement keys *latch*: the last
direction keeps being republished at `rate_hz` (well inside locomotion_node's
command_timeout_s deadman) until you press space or another direction.
Run it in its own terminal: `ros2 run hexapod_locomotion teleop_keyboard`.
"""
from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, String

HELP = """
Hexapod keyboard teleop
-----------------------
  w / s : forward / backward        (latched until space)
  a / d : strafe left / right
  q / e : turn left / right
  space : stop
  + / - : speed up / down (10%..100% of the limits)
  r / f : stance height up / down
  1 / 2 / 3 : gesture bow / wave / sway
  x or Ctrl-C : quit
"""

# key -> (linear x, linear y, angular z) as fractions of the speed limits
MOVES = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "q": (0.0, 0.0, 1.0),
    "e": (0.0, 0.0, -1.0),
    " ": (0.0, 0.0, 0.0),
}
GESTURES = {"1": "bow", "2": "wave", "3": "sway"}


class TeleopKeyboard(Node):
    def __init__(self) -> None:
        super().__init__("teleop_keyboard")
        self.declare_parameter("max_linear_x_m_s", 0.10)
        self.declare_parameter("max_linear_y_m_s", 0.10)
        self.declare_parameter("max_angular_z_rad_s", 0.40)
        self.declare_parameter("stance_height_m", 0.100)
        self.declare_parameter("stance_height_step_m", 0.005)
        self.declare_parameter("min_stance_height_m", 0.060)
        self.declare_parameter("max_stance_height_m", 0.135)
        self.declare_parameter("rate_hz", 20.0)

        self.max_x = float(self.get_parameter("max_linear_x_m_s").value)
        self.max_y = float(self.get_parameter("max_linear_y_m_s").value)
        self.max_z = float(self.get_parameter("max_angular_z_rad_s").value)
        self.height = float(self.get_parameter("stance_height_m").value)
        self.height_step = float(self.get_parameter("stance_height_step_m").value)
        self.min_height = float(self.get_parameter("min_stance_height_m").value)
        self.max_height = float(self.get_parameter("max_stance_height_m").value)

        self.direction = (0.0, 0.0, 0.0)
        self.speed = 0.5

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.height_pub = self.create_publisher(Float64, "/stance_height_command", 10)
        self.gesture_pub = self.create_publisher(String, "/gesture_command", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.publish_command)

    def publish_command(self) -> None:
        message = Twist()
        message.linear.x = self.direction[0] * self.speed * self.max_x
        message.linear.y = self.direction[1] * self.speed * self.max_y
        message.angular.z = self.direction[2] * self.speed * self.max_z
        self.cmd_pub.publish(message)

    def handle_key(self, key: str) -> bool:
        """Apply one keypress; returns False when the user asked to quit."""
        if key in ("x", "\x03"):
            return False
        if key in MOVES:
            self.direction = MOVES[key]
        elif key in ("+", "="):
            self.speed = min(1.0, self.speed + 0.1)
            print(f"\rspeed {self.speed:.0%}    ", end="", flush=True)
        elif key in ("-", "_"):
            self.speed = max(0.1, self.speed - 0.1)
            print(f"\rspeed {self.speed:.0%}    ", end="", flush=True)
        elif key in ("r", "f"):
            delta = self.height_step if key == "r" else -self.height_step
            self.height = min(self.max_height, max(self.min_height, self.height + delta))
            self.height_pub.publish(Float64(data=self.height))
            print(f"\rstance height {self.height * 100:.1f} cm    ", end="", flush=True)
        elif key in GESTURES:
            self.direction = MOVES[" "]
            self.gesture_pub.publish(String(data=GESTURES[key]))
        return True

    def stop(self) -> None:
        self.direction = MOVES[" "]
        self.publish_command()


def main() -> None:
    if not sys.stdin.isatty():
        sys.exit("teleop_keyboard needs an interactive terminal (stdin is not a TTY)")

    rclpy.init()
    node = TeleopKeyboard()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                running = node.handle_key(sys.stdin.read(1).lower())
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
