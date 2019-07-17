#!/usr/bin/python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import sys

class SubCmdVel(Node):

    def __init__(self):
        super().__init__('sub_cmd_vel')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, vel_msg):
        self.get_logger().info('linear.x: [%s]' % str(vel_msg.linear.x))
        #
        # build the command string
        #
        cmd_str = "echo "
        vel_value = int(vel_msg.linear.x)
        #
        # handle the different cases
        #
        if vel_value == 0:
            cmd_str = cmd_str + "000"
        if 0 < vel_value < 10:
            cmd_str = cmd_str + "00"
            cmd_str = cmd_str + str(vel_value)
        if 10 <= vel_value < 100:
            cmd_str = cmd_str + "0"
            cmd_str = cmd_str + str(vel_value)
        if vel_value >= 100:
            cmd_str = cmd_str + "100"
        cmd_str = cmd_str + " > /dev/ttyACM0"
        print(cmd_str)
        subprocess.call(cmd_str, shell=True)


def main(args=None):
    rclpy.init(args=args)

    node = SubCmdVel()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
