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

#from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SubCmdVel(Node):

    def __init__(self):
        super().__init__('sub_cmd_vel')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        self.get_logger().info('linear: [%s]' % str(msg.data.linear))


def main(args=None):
    rclpy.init(args=args)

    node = SubCmdVel()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
