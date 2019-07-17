//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

void print_usage()
{
  printf("Usage for pub_cmd_vel app:\n");
  printf("pub_cmd_vel [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to publish. Defaults to chatter.\n");
}

// Create a PubCmdVel class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class PubCmdVel : public rclcpp::Node
{
public:
  explicit PubCmdVel(const std::string & topic_name)
  : Node("pub_cmd_vel")
  {
    // Create a function for when messages are to be sent.
    auto publish_message =
      [this]() -> void
      {
        //msg_ = std::make_unique<std_msgs::msg::String>();
        msg_ = std::make_unique<geometry_msgs::msg::Twist>();
        //msg_->data = "Hello World: " + std::to_string(count_++);
        msg_->linear.x = 0.0;
        msg_->linear.y = 0.0;
        msg_->linear.z = 0.0;
        msg_->angular.x = 0.0;
        msg_->angular.y = 0.0;
        msg_->angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", msg_->linear.x);

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(std::move(msg_));
      };

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    //pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, qos);
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name, qos);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  size_t count_ = 1;
  //std::unique_ptr<std_msgs::msg::String> msg_;
  std::unique_ptr<geometry_msgs::msg::Twist> msg_;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Parse the command line options.
  auto topic = std::string("cmd_vel");
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nullptr != cli_option) {
    topic = std::string(cli_option);
  }

  // Create a node.
  auto node = std::make_shared<PubCmdVel>(topic);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
