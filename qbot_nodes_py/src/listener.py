#!/usr/bin/python3
#
# clone the repository        :~/ros2_ws/src$ git clone .......
# from ros2 workspace folder  :~/ros2_ws$ colcon build
# source the setup.bash file  :~/ros2_ws$ source .install/setup.bash
# start the node              :~/ros2_ws$ ros2 run qbot_nodes_py listener
#

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'pub_chatter', self.chatter_callback, 10)

    def chatter_callback(self, msg):
        self.get_logger().info('qbot listener heard: [%s]' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    node = Listener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
