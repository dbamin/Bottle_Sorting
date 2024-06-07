// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/SetPositions.h"

int main(int argc, char **argv)
{  
    // Initialize the ROS node
    ros::init(argc, argv, "demo_set_positions");      
    ros::NodeHandle nh_demo; 

    // Create a service client for setting positions
    ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
    tm_msgs::SetPositions srv;

    // Prepare the request
    srv.request.motion_type = tm_msgs::SetPositions::Request::PTP_J;  // Point-to-Point motion in joint space
    srv.request.positions = {0, 0, 1.58, 0, 1.58, 0};  // Desired joint positions
    srv.request.velocity = 0.4;  // Velocity in radians per second
    srv.request.acc_time = 0.2;  // Acceleration time
    srv.request.blend_percentage = 10;  // Blending percentage for smooth transitions
    srv.request.fine_goal = false;  // If true, robot will decelerate to stop exactly at the goal position

    // Call the service and handle the response
    if (client.call(srv))
    {
        if (srv.response.ok)
            ROS_INFO("SetPositions command successfully sent to robot.");
        else
            ROS_WARN("SetPositions command sent, but the robot did not confirm the operation was successful.");
    }
    else
    {
        ROS_ERROR("Failed to send SetPositions command to the robot.");
        return 1;  // Return error
    }

    return 0;  // Successful execution
}
