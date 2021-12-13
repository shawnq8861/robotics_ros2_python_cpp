#include <stdio.h>
#include <memory>
#include <string>
#include <sched.h>
#include <sys/types.h>
#include <sys/mman.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

static constexpr int node_priority = 98;

class MotorEncoderTest : public rclcpp::Node
{
public:
    MotorEncoderTest()
    : Node("motor_encoder_tests"), count_(0)
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
    auto motor_encoder_test_node = std::make_shared<MotorEncoderTest>();
    //
    // use rt extensions
    //
    int rc = -1;
	struct sched_param my_params;
	// Passing zero specifies caller’s (our) policy
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
    // lock memory to prevent paging
    //
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // Do stuff here......
    RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "motor and encoder tests... ");

    //
    // unlock memory before teardown
    //
    munlockall();
    

    rclcpp::shutdown();
    return 0;
}
