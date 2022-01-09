//
// common robot hardware and software configuration parameters
//

#ifndef ROBOT_CONFIGURATION
#define ROBOT_CONFIGURATION

#include <chrono>

using namespace std::chrono_literals;

static constexpr int base_controller_priority = 97;
static constexpr int local_planner_priority = 95;
static constexpr int odometry_priority = 96;
static constexpr int8_t max_retries = 5;
static constexpr double wheel_base = 18.0;
static constexpr double wheel_diameter = 6.0;
static constexpr double rpm_max = 192.0;
static constexpr double pi = 3.1415926;
static constexpr int32_t enc_counts_per_rev = 750;
static constexpr std::chrono::milliseconds timer_period = 250ms;
//
// max_v_forward = (rpm_max / 60.0) * pi * wheel_diameter = 60.318 in/sec
//
static constexpr double max_v_forward = 60.0;
static constexpr double max_delta_v = 2.0;
//
// max_v_angular = max_v_forward / (wheel_base / 2.0) = 6.702
//
static constexpr double max_v_angular = pi / 16.0;

#endif // ROBOT_CONFIGURATION