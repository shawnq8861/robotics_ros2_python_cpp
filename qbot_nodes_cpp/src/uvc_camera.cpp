#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "camera_interface_header.hpp"

using namespace std::chrono_literals;

class UVC_Camera : public rclcpp::Node
{
public:
    UVC_Camera()
    : Node("uvc_camera")
    {
        command = Idle;
        img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("uvc_camera/image", 1);
        timer_ = this->create_wall_timer(
            200ms, std::bind(&UVC_Camera::timer_callback, this));
        sensor_msgs::msg::Image::UniquePtr img_ptr(new sensor_msgs::msg::Image());
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
        RCLCPP_INFO_STREAM(this->get_logger(), "frame cols = " << frame.cols);
        RCLCPP_INFO_STREAM(this->get_logger(), "frame rows = " << frame.rows);
        double frame_rate = cap.get(cv::CAP_PROP_FPS);
        RCLCPP_INFO_STREAM(this->get_logger(), "frame rate = " << frame_rate);
        auto stamp = now();
        //
        // the img_msg pointer is deleted automatically when the local 
        // variable goes out of scope
        //
        sensor_msgs::msg::Image::UniquePtr img_msg(new sensor_msgs::msg::Image());
        img_msg->header.stamp = stamp;
        img_msg->height = frame.rows;
        img_msg->width = frame.cols;
        img_msg->encoding = sensor_msgs::image_encodings::BGR8;
        img_msg->is_bigendian = false;
        img_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        img_msg->data.assign(frame.datastart, frame.dataend);
        img_publisher_->publish(std::move(img_msg));

        switch (command)
        {
        case Idle:
            
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
        int deviceID = 7;              // 0 = open default camera
        int apiID = cv::CAP_GSTREAMER; // 0 = autodetect default API
        //
        // open selected camera using selected API
        //
        cap.open(deviceID, apiID);
        if (!cap.isOpened())
        {
            std::cerr << "ERROR! Unable to open camera\n";
        }
        else
        {
            double api = cap.get(cv::CAP_PROP_BACKEND);
            RCLCPP_INFO_STREAM(this->get_logger(), "backend = " << api);
            double frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
            RCLCPP_INFO_STREAM(this->get_logger(), "frame width = " << frame_width);
            double frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
            RCLCPP_INFO_STREAM(this->get_logger(), "frame height = " << frame_height);
            double fps = 1.0;
            cap.set(cv::CAP_PROP_FPS, fps);
        }
    }

    void show_image()
    {
        // named window
        // imshow
        // waitkey
    }

    void save_image()
    {
        // imwrite
    }

    cv::Mat frame;
    cv::VideoCapture cap;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
    int command;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UVC_Camera>());
    rclcpp::shutdown();
    return 0;
}
