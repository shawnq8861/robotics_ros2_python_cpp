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
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
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
      auto message = std_msgs::msg::String();
      message.data = "Camera node says hello... " + std::to_string(count_++);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
      //
      // publish the image
      //
      cap >> frame;
      //ROS_INFO_STREAM("frame cols = " << frame.cols);
      std::cout << "frame cols = " << frame.cols << std::endl;
      //ROS_INFO_STREAM("frame rows = " << frame.rows);
      std::cout << "frame rows = " << frame.rows << std::endl;
    }

    void init_camera()
    {
      //
      // set up video capture
      //
      //cv::VideoCapture cap;
      int deviceID = 1;             // 0 = open default camera
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
    cv::Mat frame;
    cv::VideoCapture cap;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UVC_Camera>());
  rclcpp::shutdown();
  return 0;
}
