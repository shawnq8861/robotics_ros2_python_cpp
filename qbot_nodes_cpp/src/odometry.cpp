//
// to run from commad line:
// ros2 run qbot_nodes_cpp local_planner
//

#include <chrono>
#include <functional>
#include <memory>
#include <sched.h>
#include <sys/mman.h>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "qbot_nodes_cpp/msg/encoder_counts.hpp"
#include "robot_configuration.hpp"

using std::placeholders::_1;

class Odometry : public rclcpp::Node
{
public:
    Odometry()
    : Node("odometry"), count_(0)
    {
        curr_time = this->get_clock()->now();
        prev_time = curr_time;
        //
        // initialize the encoder counts subscriber
        //
        subscription_ = this->create_subscription<qbot_nodes_cpp::msg::EncoderCounts>(
        "enc_counts", 10, std::bind(&Odometry::subscriber_callback, this, _1));
        //
        // initialize the transform broadcaster
        //
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        //
        // initialize the odometry message publisher
        //
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    }

private:
    void subscriber_callback(const sensor_msgs::msg::Image::SharedPtr img_ptr)
    {
        curr_time = this->get_clock()->now();
        RCLCPP_INFO_STREAM(this->get_logger(), "\n\nsubscriber count_: " << count_ << "\n");
        ++count_;
        //
        // compute odometry in a typical way given the velocities of the robot
        //
        double x = 0.0;
        double y = 0.0;
        double th = 0.0;
        double vx = 0.1;
        double vy = -0.1;
        double vth = 0.1;
        rclcpp::Duration dt = curr_time - prev_time;
        double dt_sec = dt.nanoseconds() / 1e9;
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;
        
        x += delta_x;
        y += delta_y;
        theta += delta_th;
        //
        // since all odometry is 6DOF we'll need a quaternion created from yaw
        //
        tf2::Quaternion quat;

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, msg->theta);
        odom_trans.transform.rotation.x = quat.x();
        odom_trans.transform.rotation.y = quat.y();
        odom_trans.transform.rotation.z = quat.z();
        odom_trans.transform.rotation.w = quat.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(odom_trans);
        
        
        
        
        
        //
        // next, we'll publish the odometry message over ROS
        //
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        
        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        
        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        
        //publish the message
        odom_pub.publish(odom);

        prev_time = curr_time;
    }   
    rclcpp::Subscription<qbot_nodes_cpp::msg::EncoderCounts>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Time curr_time;
    rclcpp::Time prev_time;
    int count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Odometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
