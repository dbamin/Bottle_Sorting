// node_6.cpp

// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tm_msgs/SetPositions.h>
#include <tm_msgs/FeedbackState.h>

// std headers
#include <sstream>
#include <cstdlib>
#include <cmath>

// Global variables to store feedback states
std::vector<double> current_joint_positions(6, 0.0);
bool position_reached = false;
bool bottle_gripped = false;
const double tolerance = 0.01; // 1% tolerance

// Callback to update current joint positions
void feedbackCallback(const tm_msgs::FeedbackState::ConstPtr& msg) {
    current_joint_positions = msg->joint_pos;
}

// Callback to set flag when the bottle is gripped
void gripStatusCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "The bottle is gripped") {
        bottle_gripped = true;
        ROS_INFO("Received message: The bottle is gripped");
    }
}

// Function to check if the current position is within the tolerance range of the target position
bool isPositionReached(const std::vector<double>& target_positions, const std::vector<double>& current_positions) {
    for (size_t i = 0; i < target_positions.size(); ++i) {
        double diff = std::abs(current_positions[i] - target_positions[i]);
        if (diff > tolerance) {
            return false;
        }
    }
    return true;
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "demo_set_positions");      
    ros::NodeHandle nh_demo; 

    // Create a service client to set positions
    ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");

    // Create a publisher for the bottle detection readiness message
    ros::Publisher ready_pub = nh_demo.advertise<std_msgs::String>("bottle_detection_ready", 10);

    // Subscribe to grip status
    ros::Subscriber grip_status_sub = nh_demo.subscribe("grip_status", 1, gripStatusCallback);

    // Wait until the bottle is gripped
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok() && !bottle_gripped) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Prepare the service request
    tm_msgs::SetPositions srv;
    std::vector<double> target_positions = {0.195, 0.089, 1.285, 0.207, 1.545, 0.211}; // Target positions in radians

    srv.request.motion_type = tm_msgs::SetPositions::Request::PTP_J;
    srv.request.positions = target_positions;
    srv.request.velocity = 20; // rad/s
    srv.request.acc_time = 0.2;
    srv.request.blend_percentage = 10;
    srv.request.fine_goal = false;

    // Call the service to set positions
    if (client.call(srv)) {
        if (srv.response.ok) {
            ROS_INFO("SetPositions command sent to robot.");
        } else {
            ROS_WARN("SetPositions command sent, but response not yet ok.");
        }
    } else {
        ROS_ERROR("Failed to call SetPositions service.");
        return 1;
    }

    // Loop to check if the robot has reached the target position
    ros::Subscriber feedback_sub = nh_demo.subscribe("/feedback_states", 1, feedbackCallback);
    while (ros::ok()) {
        ros::spinOnce();

        if (isPositionReached(target_positions, current_joint_positions)) {
            std_msgs::String ready_msg;
            ready_msg.data = "Ready to detect the bottle.";
            ready_pub.publish(ready_msg);
            ROS_INFO("Ready to detect the bottle.");
            
            position_reached = true;
            break;
        }

        loop_rate.sleep();
    }

    if (!position_reached) {
        ROS_WARN("Robot did not reach the target position within the expected time.");
    }

    return 0;
}
