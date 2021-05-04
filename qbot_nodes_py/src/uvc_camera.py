#!/usr/bin/python3
#
# clone the repository        :~/ros2_ws/src$ git clone .......
# from ros2 workspace folder  :~/ros2_ws$ colcon build
# source the setup.bash file  :~/ros2_ws$ source .install/setup.bash
# start the node              :~/ros2_ws$ ros2 run qbot_nodes_py uvc_camer
#

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import cv2
#import numpy as np

class UVCCamera(Node):

    def __init__(self):
        super().__init__('uvc_camera')
        self.i = 0
        self.pub = self.create_publisher(String, 'pub_chatter', 10)
        self.id = 2
        self.cap = cv2.VideoCapture()
        self.init_camera()
        timer_period = .5
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from qBot uvc_camera: {0}'.format(self.i)
        self.i += 1
        self.get_logger().info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)
        ret, self.frame = self.cap.read()
        h,  w = self.frame.shape[:2]
        print("cols = ", w)
        print("rows = ", h)

    def init_camera(self):
        self.cap.open(self.id)
        if self.cap.isOpened():
            print("opened camera...")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        else:
            print("could not open camera...")

def main(args=None):
    rclpy.init(args=args)

    uvc_camera = UVCCamera()

    rclpy.spin(uvc_camera)

    uvc_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
