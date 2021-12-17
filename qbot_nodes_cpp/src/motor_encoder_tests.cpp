#include <stdio.h>
#include <memory>
#include <string>
#include <sched.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "roboclaw.hpp"

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
    RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "motor and encoder tests... ");
    //
    // try to open a port
    //
    // port, baudrate, timeout in milliseconds
    //
    std::string port("/dev/ttymxc2");
    //unsigned long baud = 38400;
    int baudrate = 38400;
    //serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1));

    //initialize at supplied terminal at specified baudrate
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
            //printf("could not read encoder values...\n");
            RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "could not read encoder values...\n");
        }
        else {
            //printf("enc m1: %d\n", enc_m1);
            //printf("enc m2: %d\n", enc_m2);
            RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "encoder 1 cout: " << enc_m1);
            RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "encoder 2 count: " << enc_m2);
        }
        //
        // unlock memory before teardown
        //
        munlockall();
    }
    


    //
    // check if the serial port is open
    //
    //RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "Is the serial port open?");
    //if(my_serial.isOpen()) {
    //    RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), " Yes. open, testing...\n\n");
    //    Roboclaw robo = Roboclaw(&my_serial);
    //    uint8_t address = 0x80;
    //    uint8_t status = 0;
    //    bool valid = false;
    //    my_serial.write(&address, 1);
    //    uint8_t command = 16;
    //    my_serial.write(&command, 1);
    //    uint32_t motor1_position = -1;
    //    auto ret_value = my_serial.read();
    //    RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "return value:  " << ret_value);
    //    motor1_position = robo.ReadEncM1(address, &status, &valid);
    //    RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), "motor1 position:  " << motor1_position);
    //    my_serial.close();
    //}
    //else {
    //    RCLCPP_INFO_STREAM(motor_encoder_test_node->get_logger(), " No.");
    //}



    
    rclcpp::shutdown();
    return 0;
}
