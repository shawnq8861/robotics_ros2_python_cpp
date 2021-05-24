#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <sched.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/select.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

static constexpr int node_priority = 98;
static constexpr suseconds_t wait_time_ms = 50;
static constexpr int del_val = 25000;	// 25 msec delay parameter
static constexpr int buff_size = 40000; // big enough to force paging
static constexpr int max_count = 30;

class RTTest : public rclcpp::Node
{
public:
    RTTest()
    : Node("realtime_tests"), count_(0)
    {
    }

    void run_jitter_test()
    {
        int result = 0;
	    int count = 0;
	    long min, max, average, current, interval;
        long min_sum, max_sum, ave_sum;
        long min_ave, max_ave, ave_ave;
	    struct timeval cur_time, last_time;
        fd_set inputs, testfds;
        //
        //  Initialize stuff
        //
        FD_ZERO (&inputs);      // select data structure
        FD_SET (0, &inputs);	
        max = 0;
        average = 0;
        min = 1000000;
        min_sum = 0;
        max_sum = 0;
        ave_sum = 0;
        min_ave = 0;
        max_ave = 0;
        ave_ave = 0;
        gettimeofday(&last_time, NULL);
        while(result == 0 && count < max_count) {
            ++count;
            //
            //  use select() to generate a sub-second timeout and also
            //  detect when the user wants to stop.  Note that both cur_time
            //  and testfds can be changed by select().
            //
            cur_time.tv_sec = 0;
            cur_time.tv_usec = wait_time_ms * 1000;
            testfds = inputs;
            result = select(FD_SETSIZE, &testfds, NULL, NULL, &cur_time);
            //
            //  Get the current time, compute the interval in microseconds from
            //  the last loop and compute the deviation from what we expect.
            //
            gettimeofday(&cur_time, NULL);
            interval = (cur_time.tv_sec - last_time.tv_sec) * 1000000 + (cur_time.tv_usec - last_time.tv_usec);
            current = interval - wait_time_ms * 1000;
            //
            //  Update the statistics and print them
            //
            if(abs (current) > max) {
                max = abs (current);
            }
            if(abs (current) < min) {
                min = abs (current);
            }
            if(average == 0) {
                average = abs (current);
            }
            else {
                average = (average + abs (current))/2;
            }
            last_time = cur_time;
            if(result == 0) {
                std::cout << "min " << min << ", max " << max << ", avg " << average << ", current " << current << std::endl;
            }
            last_time = cur_time;
            min_sum += min;
            max_sum += max;
            ave_sum += average;
        }
        min_ave = min_sum / count;
        max_ave = max_sum / count;
        ave_ave = ave_sum / count;
        std::cout << "min ave " << min_ave << ", max ave " << max_ave << ", ave " << ave_ave << std::endl;
	    std::cout << "End latency test process, iteration count = " << count << std::endl;
    }

private:
    size_t count_;
    int dummy_buff[buff_size];

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    //
    // create the node
    //
    auto rt_test_node = std::make_shared<RTTest>();
    //
    // check cl arg to see if running rt or not
    //
    std::string argv1 = argv[1];
    if (argv1 == "rt") {
        //
        // use rt extensions
        //
        //int rc, old_scheduler_policy;
        int rc = -1;
	    struct sched_param my_params;
	    // Passing zero specifies caller’s (our) policy
	    //old_scheduler_policy = sched_getscheduler(0);
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
    }
    //
    // run the jitter tests
    //
    rt_test_node->run_jitter_test();
    //
    // unlock memory before teardown
    //
    if (argv1 == "rt") {
        munlockall();
    }

    rclcpp::shutdown();
    return 0;
}
