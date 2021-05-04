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
from qbot_nodes_py.srv import UpdateTwist


class PubCmdVel(Node):

    def __init__(self):
        super().__init__('pub_cmd_vel')
        self.i = 0
        self.vel_msg = Twist()
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = .75
        self.tmr = self.create_timer(timer_period, self.timer_callback)
        self.srv = self.create_service(UpdateTwist, 'update_position', self.update_position_callback)

    def timer_callback(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 1.1
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 5.0
        self.i += 1
        self.get_logger().info('Publishing: "{0}"'.format(str(self.vel_msg)))
        self.pub.publish(self.vel_msg)

    def update_position_callback(self, request, response):
        self.vel_msg = request.goal_twist
        response.result = 0
        return response.result


def main(args=None):
    rclpy.init(args=args)

    node = PubCmdVel()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
