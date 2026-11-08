#!/usr/bin/python3

import math
import subprocess
import threading

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

GZ_TIMEOUT_MS = 3000


# Build the SDF model string of one static, colliding obstacle.
def obstacle_sdf(name, shape, size, height):
    if shape == 'cylinder':
        geometry = f'<cylinder><radius>{size / 2}</radius><length>{height}</length></cylinder>'
    else:
        geometry = f'<box><size>{size} {size} {height}</size></box>'
    return (f'<sdf version="1.9"><model name="{name}"><static>true</static><link name="link">'
            f'<collision name="collision"><geometry>{geometry}</geometry></collision>'
            f'<visual name="visual"><geometry>{geometry}</geometry>'
            '<material><ambient>1 0.4 0 1</ambient><diffuse>1 0.4 0 1</diffuse></material>'
            '</visual></link></model></sdf>')


# Escape text as a protobuf string literal for the gz --req argument.
def gz_quote(text):
    return '"' + text.replace('\\', '\\\\').replace('"', '\\"') + '"'


class ObstacleSpawner(Node):
    # Set up the node: parameters, TF, the click subscriber and the clear service.
    def __init__(self):
        super().__init__('obstacle_spawner')

        # Parameters
        self.declare_parameter('world', 'empty')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('shape', 'box')
        self.declare_parameter('size', 0.4)
        self.declare_parameter('height', 0.4)
        self.declare_parameter('remove_radius', 0.5)
        self.world = self.get_parameter('world').value
        self.world_frame = self.get_parameter('world_frame').value

        # Variables
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.obstacles = {}  # name -> (x, y) in the world frame
        self.counter = 0
        self.lock = threading.Lock()

        # Publishers

        # Subscribers
        # RViz publishes clicked points on the global /clicked_point
        self.create_subscription(PointStamped, '/clicked_point', self.on_click, 10)

        # Service Servers
        self.create_service(Trigger, 'clear_obstacles', self.on_clear)

        # Service Clients

        # Action Servers

        # Action Clients

        # Timers

        # On Start
        self.get_logger().info(
            f'world "{self.world}": click with RViz "Publish Point" to add/remove obstacles')

    # Spawn an obstacle at the clicked point, or remove the obstacles near it.
    def on_click(self, msg):
        point = self.to_world(msg)
        if point is None:
            return
        x, y = point
        radius = self.get_parameter('remove_radius').value
        with self.lock:
            near = [name for name, (ox, oy) in self.obstacles.items()
                    if math.hypot(ox - x, oy - y) < radius]
        if near:
            for name in near:
                self.run_async(self.remove, name)
        else:
            with self.lock:
                self.counter += 1
                name = f'obstacle_{self.counter}'
            self.run_async(self.spawn, name, x, y)

    # Service callback: remove every spawned obstacle and report how many were removed.
    def on_clear(self, request, response):
        with self.lock:
            names = list(self.obstacles)
        failed = [name for name in names if not self.remove(name)]
        response.success = not failed
        response.message = (f'removed {len(names) - len(failed)} obstacle(s)'
                            + (f', failed: {failed}' if failed else ''))
        return response

    # Transform a clicked point from the RViz fixed frame to (x, y) in the Gazebo world frame.
    def to_world(self, msg):
        if msg.header.frame_id in ('', self.world_frame):
            return msg.point.x, msg.point.y
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, msg.header.frame_id, Time())
        except TransformException as error:
            self.get_logger().warn(f'cannot place obstacle: {error}')
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        px, py = msg.point.x, msg.point.y
        return (t.x + math.cos(yaw) * px - math.sin(yaw) * py,
                t.y + math.sin(yaw) * px + math.cos(yaw) * py)

    # Run a function in a background thread, because gz service calls can take a few 100 ms.
    def run_async(self, func, *args):
        threading.Thread(target=func, args=args, daemon=True).start()

    # Call a Gazebo world service through the gz CLI and return True if it succeeded.
    def gz_call(self, service, reqtype, request):
        cmd = ['gz', 'service', '-s', f'/world/{self.world}/{service}', '--reqtype', reqtype,
               '--reptype', 'gz.msgs.Boolean', '--timeout', str(GZ_TIMEOUT_MS), '--req', request]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=GZ_TIMEOUT_MS / 1000 + 2)
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().error(f'gz service {service}: {error}')
            return False
        if 'data: true' not in result.stdout:
            self.get_logger().error(f'gz service {service} failed: {(result.stdout + result.stderr).strip()}')
            return False
        return True

    # Spawn an obstacle model in Gazebo at (x, y) and remember it.
    def spawn(self, name, x, y):
        shape = self.get_parameter('shape').value
        size = float(self.get_parameter('size').value)
        height = float(self.get_parameter('height').value)
        request = (f'sdf: {gz_quote(obstacle_sdf(name, shape, size, height))}, '
                   f'pose: {{position: {{x: {x}, y: {y}, z: {height / 2}}}}}')
        if self.gz_call('create', 'gz.msgs.EntityFactory', request):
            with self.lock:
                self.obstacles[name] = (x, y)
            self.get_logger().info(f'spawned {name} ({shape} {size} m) at world x={x:.2f} y={y:.2f}')
            return True
        return False

    # Remove an obstacle model from Gazebo and forget it.
    def remove(self, name):
        if self.gz_call('remove', 'gz.msgs.Entity', f'name: "{name}", type: MODEL'):
            with self.lock:
                self.obstacles.pop(name, None)
            self.get_logger().info(f'removed {name}')
            return True
        return False


# Start the node and spin until Ctrl+C.
def main():
    rclpy.init()
    node = ObstacleSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
