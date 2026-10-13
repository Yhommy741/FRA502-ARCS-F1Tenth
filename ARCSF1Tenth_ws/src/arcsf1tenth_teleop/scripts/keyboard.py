#!/usr/bin/python3

import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


HELP = """
ARCS F1Tenth keyboard teleop
----------------------------
   w : speed up          s : slow down / reverse
   a : steer left        d : steer right    (hold; release = straight)
   space : STOP (speed 0, steering center)
   q / Ctrl-C : quit
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Run by hand (not from the launch file), so it can't see the launch 'namespace'
        # argument: for another namespace use -p drive_topic:=/<namespace>/drive
        self.declare_parameter('drive_topic', '/arcsf1tenth/drive')
        self.declare_parameter('max_speed', 2.0)            # [m/s]
        self.declare_parameter('speed_step', 0.2)           # [m/s] per key press
        self.declare_parameter('max_steering_angle', 0.5236)  # [rad]
        self.declare_parameter('steering_rate', 0.25)        # [rad/s] turning toward full lock while a/d held
        self.declare_parameter('center_rate', 20.0)          # [rad/s] returning to straight after release
        # A terminal never sees key releases, only auto-repeat while a key is held.
        # Steering is released when the repeats stop:
        self.declare_parameter('hold_timeout_first', 0.6)   # [s] wait for auto-repeat to start (OS delay ~0.5 s)
        self.declare_parameter('hold_timeout', 0.1)        # [s] gap that means "released" once repeating
        self.declare_parameter('rate', 20.0)                # [Hz] publish rate

        self.max_speed = self.get_parameter('max_speed').value
        self.speed_step = self.get_parameter('speed_step').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.steer_rate = self.get_parameter('steering_rate').value
        self.center_rate = self.get_parameter('center_rate').value
        self.hold_timeout_first = self.get_parameter('hold_timeout_first').value
        self.hold_timeout = self.get_parameter('hold_timeout').value
        self.period = 1.0 / self.get_parameter('rate').value

        drive_topic = self.get_parameter('drive_topic').value
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.speed = 0.0
        self.steering_angle = 0.0

        self.steer_dir = 0          # +1 left, -1 right, 0 none (key held)
        self.steer_repeating = False
        self.last_steer_key = 0.0

    def handle_key(self, key, now):
        if key == 'w':
            self.speed += self.speed_step
        elif key == 's':
            self.speed -= self.speed_step
        elif key in ('a', 'd'):
            direction = 1 if key == 'a' else -1
            # Same key arriving again quickly = auto-repeat of a held key
            self.steer_repeating = (
                direction == self.steer_dir and now - self.last_steer_key < self.hold_timeout_first
            )
            self.steer_dir = direction
            self.last_steer_key = now
        elif key == ' ':
            self.speed = 0.0
            self.steering_angle = 0.0
            self.steer_dir = 0

        self.speed = round(clamp(self.speed, -self.max_speed, self.max_speed), 3)

    def update_steering(self, now, dt):
        """Steer toward full lock while a/d is held, back to straight when released."""
        timeout = self.hold_timeout if self.steer_repeating else self.hold_timeout_first
        if self.steer_dir != 0 and now - self.last_steer_key > timeout:
            self.steer_dir = 0

        target = self.steer_dir * self.max_steer
        # Moving toward straight (released, or switching sides) uses the faster center rate
        toward_center = abs(target) < abs(self.steering_angle) or target * self.steering_angle < 0
        step = (self.center_rate if toward_center else self.steer_rate) * dt
        self.steering_angle = clamp(target, self.steering_angle - step, self.steering_angle + step)

    def publish(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle
        self.drive_pub.publish(msg)

    def status(self):
        return f'\rspeed: {self.speed:+5.2f} m/s   steering: {self.steering_angle:+5.3f} rad   '


def get_keys(timeout):
    """Return all keys pressed since the last call ('' if none within timeout).

    Reads the raw file descriptor instead of sys.stdin, whose internal buffer
    would hide queued keys from select() (e.g. when a key is held down).
    """
    fd = sys.stdin.fileno()
    ready, _, _ = select.select([fd], [], [], timeout)
    return os.read(fd, 1024).decode(errors='ignore') if ready else ''


def clamp(value, low, high):
    return max(low, min(high, value))


def main(args=None):
    if not sys.stdin.isatty():
        print('keyboard.py needs an interactive terminal: run it with "ros2 run", not from a launch file.')
        return

    rclpy.init(args=args)
    node = KeyboardTeleop()

    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        last = time.monotonic()
        while rclpy.ok():
            keys = get_keys(node.period)
            if 'q' in keys or '\x03' in keys:
                break
            now = time.monotonic()
            for key in keys:
                node.handle_key(key, now)
            node.update_steering(now, now - last)
            last = now
            node.publish()
            sys.stdout.write(node.status())
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        # Always leave the car stopped and the terminal usable
        node.speed = 0.0
        node.steering_angle = 0.0
        node.publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
