#!/usr/bin/env python3
"""Apply the supplied Astra Pro working range to depth and XYZ measurements."""
import argparse
from array import array

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2


class AstraDepthRange(Node):
    def __init__(self, minimum, maximum):
        super().__init__('astra_pro_depth_range')
        self.minimum, self.maximum = minimum, maximum
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.depth_publisher = self.create_publisher(Image, '/depth', qos)
        self.cloud_publisher = self.create_publisher(PointCloud2, '/depth/points', qos)
        self.create_subscription(Image, '/depth/raw', self.depth, qos)
        self.create_subscription(PointCloud2, '/depth/points_raw', self.cloud, qos)
        self.get_logger().info(f'Astra Pro: valid axial depth {minimum}--{maximum} m; '
                               'outside range is NaN; raw topics retained')

    def invalid(self, z):
        return ~np.isfinite(z) | (z < self.minimum) | (z > self.maximum)

    def depth(self, message):
        if message.encoding != '32FC1':
            self.get_logger().error(f'Expected 32FC1, got {message.encoding}')
            return
        data = bytearray(message.data)
        values = np.ndarray((message.height, message.width),
                            dtype=('>' if message.is_bigendian else '<') + 'f4',
                            buffer=data, strides=(message.step, 4))
        values[self.invalid(values)] = np.nan
        message.data = array('B', data)
        self.depth_publisher.publish(message)

    def cloud(self, message):
        fields = {field.name: field for field in message.fields}
        if not all(name in fields and fields[name].datatype == 7 for name in ('x', 'y', 'z')):
            self.get_logger().error('Expected float32 XYZ fields')
            return
        dtype = np.dtype({'names': ['x', 'y', 'z'],
                          'formats': [('>' if message.is_bigendian else '<') + 'f4'] * 3,
                          'offsets': [fields[name].offset for name in ('x', 'y', 'z')],
                          'itemsize': message.point_step})
        data = bytearray(message.data)
        values = np.ndarray((message.height, message.width), dtype=dtype,
                            buffer=data, strides=(message.row_step, message.point_step))
        invalid = self.invalid(values['z'])
        for name in ('x', 'y', 'z'):
            values[name][invalid] = np.nan
        message.data = array('B', data)
        message.is_dense = False
        self.cloud_publisher.publish(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--min-depth', type=float, default=0.6)
    parser.add_argument('--max-depth', type=float, default=8.0)
    options = parser.parse_args()
    if not 0 < options.min_depth < options.max_depth:
        parser.error('Require 0 < min-depth < max-depth')
    rclpy.init()
    node = AstraDepthRange(options.min_depth, options.max_depth)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
