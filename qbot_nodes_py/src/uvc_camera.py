#!/usr/bin/python3
#
# clone the repository        :~/ros2_ws/src$ git clone .......
# from ros2 workspace folder  :~/ros2_ws$ colcon build
# source the setup.bash file  :~/ros2_ws$ source .install/setup.bash
# start the node              :~/ros2_ws$ ros2 run qbot_nodes_py uvc_camera
#

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import cv2
import argparse
import time

from qbot_nodes_cpp.srv import CameraCommand

class UVCCamera(Node):

    def __init__(self, camera_idx, timer_period):
        super().__init__('uvc_camera')
        self.camera_command = 99
        self.i = 0
        self.srv = self.create_service(CameraCommand, 'uvc_camera/camera_command', self.camera_command_callback)
        self.pub = self.create_publisher(String, 'pub_chatter', 10)
        print("camera idx = ", camera_idx)
        self.idx = camera_idx
        self.cap = cv2.VideoCapture()
        self.init_camera()
        self.timer_period = timer_period
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
        curr_time = time.ctime()
        new_time = curr_time.replace(' ', '_')
        print(new_time)
        if self.camera_command == 1:
            cv2.imwrite(new_time + '.png', self.frame)

    def camera_command_callback(self, request, response):
        self.camera_command = request.command
        self.get_logger().info('Incoming request.command: %d' % (request.command))

        return response

    def init_camera(self):
        self.cap.open(self.idx)
        if self.cap.isOpened():
            print("opened camera...")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        else:
            print("could not open camera...")

def main(args=None):

    parser = argparse.ArgumentParser(description='Process an integer arg and a float arg')
    parser.add_argument('camera_idx', type=int)
    parser.add_argument('timer_period', type=float)
    args = parser.parse_args()

    rclpy.init()

    uvc_camera = UVCCamera(args.camera_idx, args.timer_period)

    rclpy.spin(uvc_camera)

    uvc_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
