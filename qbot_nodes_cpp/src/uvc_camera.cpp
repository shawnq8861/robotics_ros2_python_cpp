//
// to run from commad line, use appropriate /dev/video* value for cam_idx:
// ros2 run qbot_nodes_cpp uvc_camera --ros-args -p cam_idx:=7
//

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
        this->declare_parameter<int>("cam_idx", 1);
        rclcpp::Parameter cam_idx_param = this->get_parameter("cam_idx");
        camera_idx_ = cam_idx_param.as_int();
        RCLCPP_INFO_STREAM(this->get_logger(), "camera idx = " << camera_idx_);
        command_ = Idle;
        img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("uvc_camera/image", 1);
        timer_ = this->create_wall_timer(
            200ms, std::bind(&UVC_Camera::timer_callback, this));
        init_camera();
        //
        // advertise the save image service
        //
        //
        // use lambda expression to handle service callback
        // to avoid using std::bind, which is messy for sevice callbacks
        //
        // command to test:  ros2 service call /camera_command 
        // qbot_nodes_cpp/srv/CameraCommand "{command: 1}"
        //
        auto handle_camera_command =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<qbot_nodes_cpp::srv::CameraCommand::Request> request,
                            std::shared_ptr<qbot_nodes_cpp::srv::CameraCommand::Response> response) -> void
        {
            (void)request_header;
            command_ = request->command;
            response = 0;
        };
        service_ = create_service<qbot_nodes_cpp::srv::CameraCommand>("camera_command", handle_camera_command);

    }

private:
    void timer_callback()
    {
        //
        // publish the image
        //
        cap_ >> frame_;
        RCLCPP_INFO_STREAM(this->get_logger(), "frame cols = " << frame_.cols);
        RCLCPP_INFO_STREAM(this->get_logger(), "frame rows = " << frame_.rows);
        double frame_rate = cap_.get(cv::CAP_PROP_FPS);
        RCLCPP_INFO_STREAM(this->get_logger(), "frame rate = " << frame_rate);
        auto stamp = now();
        //
        // the img_msg pointer is deleted automatically when the local 
        // variable goes out of scope
        //
        sensor_msgs::msg::Image::UniquePtr img_msg(new sensor_msgs::msg::Image());
        img_msg->header.stamp = stamp;
        img_msg->height = frame_.rows;
        img_msg->width = frame_.cols;
        img_msg->encoding = sensor_msgs::image_encodings::BGR8;
        img_msg->is_bigendian = false;
        img_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);
        img_msg->data.assign(frame_.datastart, frame_.dataend);
        img_publisher_->publish(std::move(img_msg));

        switch (command_)
        {
        case Idle:
            RCLCPP_INFO_STREAM(this->get_logger(), "Idle...");
            break;
        case Show:
            //
            // call show image
            //
            RCLCPP_INFO_STREAM(this->get_logger(), "Show...");
            command_ = Idle;
            show_image();
            break;
        case Save:
            //
            // call save image
            //
            RCLCPP_INFO_STREAM(this->get_logger(), "Save...");
            command_ = Idle;
            save_image();
            break;
        default:
            break;
        }

    }

    void init_camera()
    {
        //
        // set up video capture
        //
        int apiID = 0; // 0 = autodetect default API
        //
        // open selected camera using selected API
        //
        cap_.open(camera_idx_, apiID);
        if (!cap_.isOpened())
        {
            std::cerr << "ERROR! Unable to open camera\n";
        }
        else
        {
            double api = cap_.get(cv::CAP_PROP_BACKEND);
            RCLCPP_INFO_STREAM(this->get_logger(), "backend = " << api);
            double frame_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
            RCLCPP_INFO_STREAM(this->get_logger(), "frame width = " << frame_width);
            double frame_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
            RCLCPP_INFO_STREAM(this->get_logger(), "frame height = " << frame_height);
            double fps = 1.0;
            cap_.set(cv::CAP_PROP_FPS, fps);
        }
    }

    void show_image()
    {
        // named window
        // imshow
        // waitkey
        RCLCPP_INFO_STREAM(this->get_logger(), "show image...");
    }

    void save_image()
    {
        // imwrite
        RCLCPP_INFO_STREAM(this->get_logger(), "save image...");
    }

    cv::Mat frame_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
    rclcpp::Service<qbot_nodes_cpp::srv::CameraCommand>::SharedPtr service_;
    int command_;
    int camera_idx_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UVC_Camera>());
    rclcpp::shutdown();
    return 0;
}
