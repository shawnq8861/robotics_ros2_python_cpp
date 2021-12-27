#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "roboclaw.hpp"
#include <memory>
#include <sched.h>
#include <sys/mman.h>

using std::placeholders::_1;

static constexpr int node_priority = 97;
static constexpr int8_t max_retries = 5;

class BaseController : public rclcpp::Node
{
public:
    BaseController(double width, double diameter)
    : Node("base_controller"), wheel_diameter_(diameter), wheel_base_(width),
        port_("/dev/ttymxc2"), baudrate_(38400), address_(0x80)
    {
        RCLCPP_INFO(this->get_logger(), "set wheel base: [%f]", wheel_base_);
        RCLCPP_INFO(this->get_logger(), "set wheel diameter: [%f]", wheel_diameter_);
        robo_ = roboclaw_init(port_.c_str(), baudrate_);
        if (robo_ == nullptr) {
            RCLCPP_INFO_STREAM(this->get_logger(), "unable to instantiate roboclaw object...\n");
        }
        drive_wheels();
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&BaseController::cmd_vel_callback, this, _1));
    }

    ~BaseController()
    {
        roboclaw_close(robo_);
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard forward speed: [%f]", msg->linear.x);
        RCLCPP_INFO(this->get_logger(), "I heard angular speed: [%f]", msg->angular.z);
        //
        // need to read and publish encoder count
        //
        drive_wheels();
    }
    void drive_wheels() {
        //
        // use kinematic model to compute each wheel rotational velocity
        // output to the RoboClaw
        //
        int32_t enc_m1;
        int32_t enc_m2;
        //
        // read encoders (swiich to running in a ROS loop later on)
        //
        if (roboclaw_encoders(robo_, address_, &enc_m1, &enc_m2) != ROBOCLAW_OK) {
            RCLCPP_INFO_STREAM(this->get_logger(), "could not read encoder values...\n");
        }
        else {
            RCLCPP_INFO_STREAM(this->get_logger(), "encoder 1 count: " << enc_m1);
            RCLCPP_INFO_STREAM(this->get_logger(), "encoder 2 count: " << enc_m2);
        }
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    double wheel_base_;
    double wheel_diameter_;
    std::string port_;
    int baudrate_;
    uint8_t address_;
    struct roboclaw *robo_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    double width = 18.0;
    double diameter = 6.0;
    auto node = std::make_shared<BaseController>(width, diameter);
    //
    // use rt extensions
    //
    int rc = -1;
	struct sched_param my_params;
    //
	// Passing zero specifies caller’s (our) policy
    //
	my_params.sched_priority = node_priority;
    //
	// Passing zero specifies callers (our) pid
    // Set policy to SCHED_RR, no preemption with time slicing
    //
	rc = sched_setscheduler(0, SCHED_RR, &my_params);
    if ( rc == -1 ) {
        std::cout << "could not change scheduler policy" << std::endl;
    }
    else {
        std::cout << "changed scheduler policy" << std::endl;
    }
    //
    // lock memory to prevent paging after instantiations are complete
    //
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rclcpp::spin(node);
    //
    // unlock memory before teardown
    //
    munlockall();
    rclcpp::shutdown();
    
    return 0;
}
