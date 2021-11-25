#ifndef CAMERA_HEADER_HPP
#define CAMERA_HEADER_HPP
//
// camera interface header
//
#include <sensor_msgs/msg/image.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>
#include "qbot_nodes_cpp/srv/camera_command.hpp"

enum CameraCommand {Idle, Show, Save, Quit};

class CameraControl
{
public:
    CameraControl()
    {control_value = Idle;};
    ~CameraControl(){};
    int get_control_value()
    {
        return control_value;
    }
    void set_control_value(int value)
    {
        control_value = value;
    }
    //bool callback(sony_camera_node::CameraCommandRequest& request, 
    //              sony_camera_node::CameraCommandResponse& response)
    //{
    //    if (request.command > -1 && request.command <= idle) {
    //        control_value = request.command;
    //        ROS_INFO_STREAM("service called, command value = " << (int)request.command);
    //        return true;
    //    }
    //    else {
    //        return false;
    //    }
    //}
private:
    int control_value;
};

#endif
