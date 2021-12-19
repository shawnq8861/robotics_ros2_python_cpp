#include <stdio.h>
#include <memory>
#include <string>
#include <sstream>
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

class MotorEncoderTest : public rclcpp::Node
{
public:
    MotorEncoderTest()
    : Node("motor_duty_cycle_test"), count_(0)
    {
    }

private:
    size_t count_;

};

int main(int argc, char * argv[])
{
    //
    // get duty cycle
    //
    std::stringstream duty_ss;
    duty_ss << argv[1];
    int duty_cycle;
    duty_ss >> duty_cycle;
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
    RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "motor duty cycle tests... ");
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
        //
        // lock memory to prevent paging after instantiations are complete
        //
        mlockall(MCL_CURRENT | MCL_FUTURE);
        //
        // set the duty cycle
        //			
		if( duty_cycle > 25 ) {
			duty_cycle = 25;
        }	
		if( duty_cycle < -25 ) {
			duty_cycle = -25;
        }
		//	
		// 32767 is max duty cycle setpoint that roboclaw accepts
        //
        RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "duty cycle entered: " << duty_cycle);
		duty_cycle = (float)duty_cycle/100.0f * 32767;
        RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "duty cycle converted: " << duty_cycle);
        //
        // move the motors
        //       
		if(roboclaw_duty_m1m2(robo, address, duty_cycle, duty_cycle/2) != ROBOCLAW_OK ) {
			RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "could not set motor duty cycle...\n");		
		}
        else {
            RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "set motor duty cycle successfully...\n");
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
