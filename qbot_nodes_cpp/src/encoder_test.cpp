#include <stdio.h>
#include <memory>
#include <string>
#include <sched.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/string.hpp"
#include "roboclaw.hpp"

static constexpr int node_priority = 97;

using namespace std::chrono_literals;

class EncoderTest : public rclcpp::Node
{
public:
    EncoderTest()
    : Node("encoder_test"), count_(0)
    {
    }

private:
    size_t count_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    //
    // create the node
    //
    auto motor_encoder_test_node = std::make_shared<EncoderTest>();
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
    RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "encoder test... ");
    //
    // try to open a port
    //
    // port, baudrate, timeout in milliseconds
    //
    std::string port("/dev/ttymxc2");
    int baudrate = 38400;
    //
    //initialize at roboclaw object
    //
    struct roboclaw *robo;
	robo = roboclaw_init(port.c_str(), baudrate);
    if (robo == nullptr) {
        RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "unable to instantiate roboclaw object...\n");
    }
    else {
        uint8_t address = 0x80;
        int32_t enc_m1;
        int32_t enc_m2;
        //
        // lock memory to prevent paging after instantiations are complete
        //
        mlockall(MCL_CURRENT | MCL_FUTURE);
        //
        // read encoders (swiich to running in a ROS loop later on)
        //
        if (roboclaw_encoders(robo, address, &enc_m1, &enc_m2) != ROBOCLAW_OK) {
            RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "could not read encoder values...\n");
        }
        else {
            RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "encoder 1 count: " << enc_m1);
            RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "encoder 2 count: " << enc_m2);
        }
        //
        // unlock memory before teardown
        //
        munlockall();
        roboclaw_close(robo);
    }
    
    rclcpp::shutdown();
    return 0;
}
