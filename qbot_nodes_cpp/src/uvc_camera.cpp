#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "camera_interface_header.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class UVC_Camera : public rclcpp::Node
{
  public:
    UVC_Camera()
    : Node("uvc_camera"), count_(0)
    {
      command = Idle;
      img_publisher_ = this ->create_publisher<sensor_msgs::msg::Image>("uvc_camera/image", 1);
      timer_ = this->create_wall_timer(
      200ms, std::bind(&UVC_Camera::timer_callback, this));
      init_camera();
      //
      // advertise the save image service
      //
    }

  private:
    void timer_callback()
    {
      //
      // publish the image
      //
      cap >> frame;
      //ROS_INFO_STREAM("frame cols = " << frame.cols);
      std::cout << "frame cols = " << frame.cols << std::endl;
      //ROS_INFO_STREAM("frame rows = " << frame.rows);
      std::cout << "frame rows = " << frame.rows << std::endl;

      //img_msg_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      //img_publisher_->publish(*img_msg_ptr);

      switch (command)
      {
      case Idle:
        /* code */
        break;
      case Show:
        //
        // call show image
        //
        command = Idle;
        show_image();
        break;
      case Save:

      
      default:
        break;
      }
    }

    void init_camera()
    {
      //
      // set up video capture
      //
      //cv::VideoCapture cap;
      int deviceID = 2;             // 0 = open default camera
      int apiID = cv::CAP_GSTREAMER;      // 0 = autodetect default API
      //
      // open selected camera using selected API
      //
      cap.open(deviceID, apiID);
      if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
      }
      else {
        double api = cap.get(cv::CAP_PROP_BACKEND);
        //ROS_INFO_STREAM("backend = " << api);
        std::cout << "backend = " << api << std::endl;
        double frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        //ROS_INFO_STREAM("frame width = " << frame_width);
        double frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        //ROS_INFO_STREAM("frame height = " << frame_height);
        double frame_rate = cap.get(cv::CAP_PROP_FPS);
        //ROS_INFO_STREAM("frame rate = " << frame_rate);
        double fps = 5.0;
        cap.set(cv::CAP_PROP_FPS, fps);
        frame_rate = cap.get(cv::CAP_PROP_FPS);
        std::cout << "frame rate = " << frame_rate << std::endl;
        //ROS_INFO_STREAM("frame rate = " << frame_rate);
        frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        //ROS_INFO_STREAM("frame width = " << frame_width);
        std::cout << "frame width = " << frame_width << std::endl;
        frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        //ROS_INFO_STREAM("frame height = " << frame_height);
        std::cout << "frame height " << frame_height << std::endl;
        //cv::Mat frame;
        //ROS_INFO_STREAM("frame cols = " << frame.cols);
        //ROS_INFO_STREAM("frame rows = " << frame.rows);
      }
    }

    void show_image() {
      // named window
      // imshow
      // waitkey
    }

    void save_image() {
      // imwrite
    }

    cv::Mat frame;
    sensor_msgs::msg::Image::SharedPtr img_msg_ptr;
    cv::VideoCapture cap;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_; 
    size_t count_;
    int command;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UVC_Camera>());
  rclcpp::shutdown();
  return 0;
}
