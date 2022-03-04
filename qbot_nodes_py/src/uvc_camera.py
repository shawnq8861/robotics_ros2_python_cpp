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

#from qbot_nodes_cpp.srv import CameraCommand

class UVCCamera(Node):

    def __init__(self, camera_idx, fps):
        super().__init__('uvc_camera')
        self.camera_command = 99
        self.i = 0
        #self.srv = self.create_service(CameraCommand, 'uvc_camera/camera_command', self.camera_command_callback)
        self.pub = self.create_publisher(String, 'pub_chatter', 10)
        self.camera_idx = camera_idx
        self.fps = fps
        print("camera idx = ", self.camera_idx)
        self.cap = cv2.VideoCapture()
        #self.timer_period = self.init_camera()
        self.init_camera()
        print("fps = ", self.fps)
        #self.timer_period = timer_period
        self.timer_period = 1.0 / (1.05 * self.fps)
        self.tmr = self.create_timer(self.timer_period, self.timer_callback)

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
        print("fps = ", 1.0/self.timer_period)
        curr_time = time.ctime()
        new_time = curr_time.replace(' ', '_')
        print(new_time)
        #if self.camera_command == 1:
        #    cv2.imwrite(new_time + '.png', self.frame)
        #scale = 2
        #small_frame = cv2.resize(self.frame, (int(w/scale), int(h/scale)))
        #cv2.imshow("Live Image", small_frame)
        #cv2.waitKey(1)
        
    #def camera_command_callback(self, request, response):
    #    self.camera_command = request.command
    #    self.get_logger().info('Incoming request.command: %d' % (request.command))

    #    return response

    def init_camera(self):
        self.cap.open(self.camera_idx)
        if self.cap.isOpened():
            print("opened camera...")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            #self.cap.set(cv2.CAP_PROP_FPS, 15.0)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            #
            # retrieve frame rate, which is equal to 1/period
            #
            #fps = 10.0
            #fps = float(self.cap.get(cv2.CAP_PROP_FPS))
            #self.get_logger().info("frames per second = %f" % fps)
            width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            self.get_logger().info("width = %d" % width)
            height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.get_logger().info("height = %d" % height)
            #
            # set timer period to result in an update rate slightly faster 
            # than camera frame rate, that way the callback never misses the 
            # next frame
            #
            #self.cap.set(cv2.CAP_PROP_FPS, 30.0)
            fps = float(self.cap.get(cv2.CAP_PROP_FPS))
            print("fps = ", fps)
            #timer_period = 1.0 / (1.05 * fps)

        else:
            print("could not open camera...")

        #return fps

def main(args=None):

    parser = argparse.ArgumentParser(description='Process an integer arg and a float arg')
    parser.add_argument('camera_idx', type=int)
    #parser.add_argument('timer_period', type=float)
    parser.add_argument('fps', type=float)
    args = parser.parse_args()

    rclpy.init()

    #uvc_camera = UVCCamera(args.camera_idx, args.timer_period)
    uvc_camera = UVCCamera(args.camera_idx, args.fps)

    rclpy.spin(uvc_camera)
    cv2.destroyAllWindows()
    uvc_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
