#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>
#include <thread>
#include <chrono>

// TM Driver header
#include "tm_msgs/SetIO.h"

// Global flag to check if the robot is ready to grip
bool robot_ready_to_grip = false;

// Callback function to update the flag when the message is received
void gripReadyCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "Robot is ready to grip") {
        robot_ready_to_grip = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_set_io");
    ros::NodeHandle nh_demo;

    ros::Subscriber sub = nh_demo.subscribe("robot_feedback", 1, gripReadyCallback);

    // Wait until the robot is ready to grip
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok() && !robot_ready_to_grip) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SetIO>("tm_driver/set_io");

    tm_msgs::SetIO srv;

    // Publisher to publish the message
    ros::Publisher grip_pub = nh_demo.advertise<std_msgs::String>("grip_status", 10);

    // Setting parameters for the service calls
    srv.request.module = tm_msgs::SetIO::Request::MODULE_CONTROLBOX;
    srv.request.type = tm_msgs::SetIO::Request::TYPE_DIGITAL_OUT;

    // Turn on digital pins 0 and 1
    for (int pin = 0; pin <= 1; ++pin) {
        srv.request.pin = pin;
        srv.request.state = tm_msgs::SetIO::Request::STATE_ON;

        if (client.call(srv)) {
            if (srv.response.ok) 
                ROS_INFO_STREAM("SetIO to robot: Digital Pin " << pin << " turned ON");
            else 
                ROS_WARN_STREAM("SetIO to robot, Digital Pin " << pin << " attempt, but response not yet ok ");
        } else {
            ROS_ERROR_STREAM("Error setting IO to robot: Digital Pin " << pin << " failed");
            return 1; // Return with error code
        }
    }

    ROS_INFO("IO setting operation completed.");

    // Wait for 4 seconds for the gripper to turn on
    std::this_thread::sleep_for(std::chrono::seconds(4));

    // Publish the message
    std_msgs::String msg;
    msg.data = "The bottle is gripped";
    grip_pub.publish(msg);

    ROS_INFO("Message published: The bottle is gripped");

    return 0; // Successful completion
}
