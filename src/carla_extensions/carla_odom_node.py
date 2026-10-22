#!/usr/bin/env python3

import math

import carla
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class CarlaOdomNode(Node):
    def __init__(self):
        super().__init__('carla_odom_node')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 2000)
        self.declare_parameter('role_name', 'hero')
        self.declare_parameter('frequency', 20.0)

        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        self.role_name = self.get_parameter('role_name').value
        frequency = max(float(self.get_parameter('frequency').value), 1.0)

        self.client = carla.Client(host, port)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()

        self.publisher = self.create_publisher(
            PoseStamped,
            f'/carla/{self.role_name}/odom',
            10,
        )

        # The leaderboard owns synchronous ticks; this timer only reads state.
        self.timer = self.create_timer(1.0 / frequency, self.publish_pose)
        self.get_logger().info(
            f'Publishing map pose on /carla/{self.role_name}/odom'
        )

    def find_vehicle(self):
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == self.role_name:
                return actor
        return None

    @staticmethod
    def quaternion_from_carla_rotation(rotation):
        roll = math.radians(rotation.roll)
        pitch = -math.radians(rotation.pitch)
        yaw = -math.radians(rotation.yaw)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def publish_pose(self):
        vehicle = self.find_vehicle()
        if vehicle is None:
            return

        transform = vehicle.get_transform()
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = transform.location.x
        pose.pose.position.y = transform.location.y
        pose.pose.position.z = transform.location.z
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = self.quaternion_from_carla_rotation(transform.rotation)
        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()