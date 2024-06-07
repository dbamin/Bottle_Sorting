#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>
#include <thread>
#include <chrono>

// TM Driver header
#include "tm_msgs/SetIO.h"

// Global flag to check if the robot is ready to release
bool robot_ready_to_release = false;

// Callback function to update the flag when the message is received
void releaseReadyCallback(const std_msgs::String::ConstPtr& msg) {
    std::string data = msg->data;
    if (data.find("Ready to place the branded bottle for class ID") != std::string::npos) {
        robot_ready_to_release = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_set_io_release");
    ros::NodeHandle nh_demo;

    ros::Subscriber sub = nh_demo.subscribe("robot_release_feedback", 1, releaseReadyCallback);

    // Wait until the robot is ready to release
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok() && !robot_ready_to_release) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SetIO>("tm_driver/set_io");

    tm_msgs::SetIO srv;

    // Publisher to publish the message
    ros::Publisher release_pub = nh_demo.advertise<std_msgs::String>("release_status", 10);

    // Setting parameters for the service calls
    srv.request.module = tm_msgs::SetIO::Request::MODULE_CONTROLBOX;
    srv.request.type = tm_msgs::SetIO::Request::TYPE_DIGITAL_OUT;

    // Turn off digital pins 0 and 1
    for (int pin = 0; pin <= 1; ++pin) {
        srv.request.pin = pin;
        srv.request.state = tm_msgs::SetIO::Request::STATE_OFF;

        if (client.call(srv)) {
            if (srv.response.ok) 
                ROS_INFO_STREAM("SetIO to robot: Digital Pin " << pin << " turned OFF");
            else 
                ROS_WARN_STREAM("SetIO to robot, Digital Pin " << pin << " attempt, but response not yet ok ");
        } else {
            ROS_ERROR_STREAM("Error setting IO to robot: Digital Pin " << pin << " failed");
            return 1; // Return with error code
        }
    }

    ROS_INFO("IO setting operation completed.");

    // Wait for 4 seconds for the gripper to turn off
    std::this_thread::sleep_for(std::chrono::seconds(4));

    // Publish the message
    std_msgs::String msg;
    msg.data = "Gripper turned off";
    release_pub.publish(msg);

    ROS_INFO("Message published: Gripper turned off");

    return 0; // Successful completion
}
