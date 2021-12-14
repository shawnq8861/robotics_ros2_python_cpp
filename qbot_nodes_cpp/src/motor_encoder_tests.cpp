#include <stdio.h>
#include <memory>
#include <string>
#include <sched.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial/serial.h"

static constexpr int node_priority = 97;

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

void enumerate_ports()
{
	std::vector<serial::PortInfo> devices_found = serial::list_ports();

	std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while( iter != devices_found.end() )
	{
		serial::PortInfo device = *iter++;

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "port: " << device.port.c_str()
                            << " description: " << device.description.c_str()
                            << " hardware_id: " << device.hardware_id.c_str());
	}
}

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
    enumerate_ports();
    //
    // try to open a port
    //
    // port, baudrate, timeout in milliseconds
    std::string port("/dev/ttymxc2");
    std::cout << "port: " << port << std::endl;
    unsigned long baud = 38400;
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

    std::cout << "Is the serial port open?";
    if(my_serial.isOpen())
        std::cout << " Yes." << std::endl;
    else
        std::cout << " No." << std::endl;
    //
    // unlock memory before teardown
    //
    munlockall();
    

    rclcpp::shutdown();
    return 0;
}
