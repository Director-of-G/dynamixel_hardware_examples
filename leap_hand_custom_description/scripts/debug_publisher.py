#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header

from urdf_parser_py.urdf import URDF


class DebugPublisherNode(Node):
    def __init__(self):
        super().__init__('debug_publisher')

        self.declare_parameter('robot_description', '')
        robot_description = self.get_parameter('robot_description').get_parameter_value().string_value
        self.robot_description = URDF.from_xml_string(robot_description)

        # 订阅 /joint_states 话题
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)

        # 发布 WrenchStamped 消息的话题
        self.wrench_publisher_list = []

    def initialize_wrench_publishers(self, num):
        self.wrench_publisher_list.clear()
        for i in range(num):
            _publisher = self.create_publisher(WrenchStamped, f'joint_wrench/joint_{i}', 10)
            self.wrench_publisher_list.append(_publisher)

    def joint_states_callback(self, msg):
        joint_names = msg.name

        if len(self.wrench_publisher_list) == 0:
            self.initialize_wrench_publishers(len(joint_names))

        # 假设joint_0到joint_15的effort都在-z方向，我们将其转换为WrenchStamped格式
        for i, joint in enumerate(joint_names):
            jid = int(joint.split("_")[-1])
            wrench_msg = WrenchStamped()

            # lookup child body frame
            wrench_frame = self.robot_description.joint_map.get(joint).child
            
            # 填充Header
            wrench_msg.header = Header()
            wrench_msg.header.stamp = self.get_clock().now().to_msg()
            wrench_msg.header.frame_id = wrench_frame

            wrench_msg.wrench.force.x = 0.0
            wrench_msg.wrench.force.y = 0.0
            wrench_msg.wrench.force.z = 0.0

            wrench_msg.wrench.torque.x = 0.0
            wrench_msg.wrench.torque.y = 0.0
            wrench_msg.wrench.torque.z = -msg.effort[i] / 100

            # 发布WrenchStamped消息
            self.wrench_publisher_list[jid].publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DebugPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()