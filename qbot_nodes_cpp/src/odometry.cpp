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
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "qbot_nodes_cpp/msg/encoder_counts.hpp"
#include "robot_configuration.hpp"

using std::placeholders::_1;

class Odometry : public rclcpp::Node
{
public:
    Odometry()
    : Node("odometry"), enc_m1_curr_(0), enc_m2_curr_(0), 
                        enc_m1_prev_(0), enc_m2_prev_(0)
    {
        vth_ = 0.0;
        vy_ = 0.0;
        vx_ = 0.0;
        x_ = 0.0;
        y_ = 0.0; 
        theta_ = 0.0;
        curr_time_ = this->get_clock()->now();
        prev_time_ = curr_time_;
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
    void subscriber_callback(const qbot_nodes_cpp::msg::EncoderCounts::SharedPtr msg)
    {
        //
        // first grab the time for time interval calculation
        //
        curr_time_ = this->get_clock()->now();
        rclcpp::Duration dt = curr_time_ - prev_time_;
        double dt_sec = dt.nanoseconds() / 1e9;
        //
        // compute odometry in a piecewise linear using small time slices
        //
        // use kinematic model to compute incremental distance change and velocity
        // changes using changes in encoder counts
        // distance = (delta encoder count) / (encoder counts per rev) * pi * wheel diameter
        //
        enc_m1_curr_ = msg->enc1_cnt;
        enc_m2_curr_ = msg->enc2_cnt;
        //RCLCPP_INFO_STREAM(this->get_logger(), "enc_m1_curr_: " << enc_m1_curr_ << ", enc_m2_curr_: " << enc_m2_curr_);
        double delta_left = ((pi * wheel_diameter) * ((double)(enc_m1_curr_ - enc_m1_prev_))) / ((double)(enc_counts_per_rev));
        double delta_right = ((pi * wheel_diameter) * ((double)(enc_m2_curr_ - enc_m2_prev_))) / ((double)(enc_counts_per_rev));
        //RCLCPP_INFO_STREAM(this->get_logger(), "delta_left: " << delta_left << ", delta_right: " << delta_right);
        double delta_dist = (delta_right + delta_left) / 2.0;
        double delta_th = (delta_right - delta_left) / wheel_base;
        //RCLCPP_INFO_STREAM(this->get_logger(), "delta_dist: " << delta_dist << ", delta_th: " << delta_th);
        double delta_x = delta_dist * cos(theta_ + (delta_th / 2.0));
        double delta_y = delta_dist * sin(theta_ + (delta_th / 2.0));
        theta_ += delta_th;
        x_ += delta_x;
        y_ += delta_y;
        vx_ = delta_x / dt_sec;
        vy_ = delta_y / dt_sec;
        vth_ = delta_th / dt_sec;
        RCLCPP_INFO_STREAM(this->get_logger(), "\nodometry x: " << x_ << ", y: " << y_ << ", theta: " << theta_);
        //
        // since all odometry is 6DOF we'll need a quaternion created from yaw
        //
        tf2::Quaternion tf_quat;
        //
        // set the header and translation members
        //
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = curr_time_;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        //
        // populate quaternion members
        //
        tf_quat.setRPY(0, 0, theta_);
        odom_trans.transform.rotation.x = tf_quat.x();
        odom_trans.transform.rotation.y = tf_quat.y();
        odom_trans.transform.rotation.z = tf_quat.z();
        odom_trans.transform.rotation.w = tf_quat.w();
        //
        // Send the transformation
        //
        tf_broadcaster_->sendTransform(odom_trans);
        //
        // setup and publish the odometry message over ROS
        //
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = curr_time_;
        odom.header.frame_id = "odom";
        geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(tf_quat);
        //
        // set the position
        //
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        //
        // set the velocity
        //
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx_;
        odom.twist.twist.linear.y = vy_;
        odom.twist.twist.angular.z = vth_;
        //
        // publish the message
        //
        publisher_->publish(odom);

        prev_time_ = curr_time_;
        enc_m1_prev_ = enc_m1_curr_;
        enc_m2_prev_ = enc_m2_curr_;
    }   
    rclcpp::Subscription<qbot_nodes_cpp::msg::EncoderCounts>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Time curr_time_;
    rclcpp::Time prev_time_;
    int32_t enc_m1_curr_;
    int32_t enc_m2_curr_;
    int32_t enc_m1_prev_;
    int32_t enc_m2_prev_;
    double x_;
    double y_;
    double theta_;
    double vx_;
    double vy_;
    double vth_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Odometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
