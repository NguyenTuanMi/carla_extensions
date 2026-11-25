#!/usr/bin/env python3

import math

import carla
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CarlaExtensionNode(Node):
    def __init__(self):
        super().__init__('carla_extension_node')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 2000)
        self.declare_parameter('role_name', 'hero')
        self.declare_parameter('frequency', 20.0)
        self.declare_parameter(
            'camera_views', ['Center', 'Rearview', 'Right', 'Left']
        )

        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        self.role_name = self.get_parameter('role_name').value
        frequency = max(float(self.get_parameter('frequency').value), 1.0)
        camera_views = self.get_parameter('camera_views').value

        self.client = carla.Client(host, port)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            f'/carla/{self.role_name}/odom',
            10,
        )
        self._image_publishers = {}
        self._image_subscriptions = []
        self._warned_cameras = set()
        self._camera_callback_group = ReentrantCallbackGroup()

        for view in camera_views:
            input_topic = f'/carla/{self.role_name}/{view}/image'
            output_topic = f'/carla/{self.role_name}/{view}/image_opaque'
            publisher = self.create_publisher(
                Image, output_topic, qos_profile_sensor_data
            )
            subscription = self.create_subscription(
                Image,
                input_topic,
                lambda message, camera_view=view: self.republish_image(
                    message, camera_view
                ),
                qos_profile_sensor_data,
                callback_group=self._camera_callback_group,
            )
            self._image_publishers[view] = publisher
            self._image_subscriptions.append(subscription)

        # The leaderboard owns synchronous ticks; this timer only reads state.
        self.timer = self.create_timer(1.0 / frequency, self.publish_pose)
        self.get_logger().info(
            f'Publishing map pose on /carla/{self.role_name}/odom'
        )
        self.get_logger().info(
            'Correcting camera alpha on: ' + ', '.join(camera_views)
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
        self.pose_publisher.publish(pose)

    def republish_image(self, message, view):
        if message.encoding != 'bgra8':
            if view not in self._warned_cameras:
                self.get_logger().error(
                    f'{view}: expected bgra8, received {message.encoding}'
                )
                self._warned_cameras.add(view)
            return

        row_bytes = message.width * 4
        expected_bytes = message.height * message.step
        if message.step < row_bytes or len(message.data) < expected_bytes:
            if view not in self._warned_cameras:
                self.get_logger().error(
                    f'{view}: image dimensions, step, or data size are inconsistent'
                )
                self._warned_cameras.add(view)
            return

        pixels = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )
        pixels[:, 3:row_bytes:4] = 255
        self._image_publishers[view].publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaExtensionNode()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()