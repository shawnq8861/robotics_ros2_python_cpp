#ifndef CAMERA_HEADER_HPP
#define CAMERA_HEADER_HPP
//
// camera interface header
//
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include "qbot_nodes_cpp/srv/camera_command.hpp"

enum CameraCommand {Idle, Show, Save, Quit};

#endif
