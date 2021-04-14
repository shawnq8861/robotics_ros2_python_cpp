#!/usr/bin/python3
#
# clone the repository        :~/ros2_ws/src$ git clone .......
# from ros2 workspace folder  :~/ros2_ws$ colcon build
# source the setup.bash file  :~/ros2_ws$ source .install/setup.bash
# start the node              :~/ros2_ws$ ros2 run qbot_nodes_py talker
#

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.i = 0
        self.pub = self.create_publisher(String, 'pub_chatter', 10)
        timer_period = 1.65
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from qBot talker: {0}'.format(self.i)
        self.i += 1
        self.get_logger().info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    talker = Talker()

    rclpy.spin(talker)

    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
