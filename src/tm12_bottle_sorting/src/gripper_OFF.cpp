#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/SetIO.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_set_io");
    ros::NodeHandle nh_demo;

    ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SetIO>("tm_driver/set_io");

    tm_msgs::SetIO srv1;
    tm_msgs::SetIO srv2;

    // Setting parameters for first service call
    srv1.request.module = tm_msgs::SetIO::Request::MODULE_CONTROLBOX;
    srv1.request.type = tm_msgs::SetIO::Request::TYPE_DIGITAL_OUT;
    srv1.request.pin = 1;
    srv1.request.state = tm_msgs::SetIO::Request::STATE_OFF;

    // Setting parameters for second service call
    srv2.request.module = tm_msgs::SetIO::Request::MODULE_CONTROLBOX;
    srv2.request.type = tm_msgs::SetIO::Request::TYPE_DIGITAL_OUT;
    srv2.request.pin = 2;
    srv2.request.state = tm_msgs::SetIO::Request::STATE_OFF;

    // Make the first service call
    if (client.call(srv1))
    {
        if (srv1.response.ok)
            ROS_INFO("SetIO to robot: Pin 1 set successfully to OFF");
        else
            ROS_WARN("SetIO to robot: Command executed but response for pin 1 not okay");
    }
    else
    {
        ROS_ERROR("Error: SetIO service call for pin 1 failed");
        return 1;  // Return with error code
    }

    // Make the second service call
    if (client.call(srv2))
    {
        if (srv2.response.ok)
            ROS_INFO("SetIO to robot: Pin 2 set successfully to OFF");
        else
            ROS_WARN("SetIO to robot: Command executed but response for pin 2 not okay");
    }
    else
    {
        ROS_ERROR("Error: SetIO service call for pin 2 failed");
        return 1;  // Return with error code
    }

    ROS_INFO("IO setting operation completed.");
    return 0;  // Successful completion
}
