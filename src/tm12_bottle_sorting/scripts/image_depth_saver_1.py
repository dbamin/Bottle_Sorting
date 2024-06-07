#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from sensor_msgs.msg import Image
import message_filters

# Initialize a flag to check if the image has been saved
image_saved = False
saved_cv_image = None
saved_depth_image = None

def callback(color_msg, depth_msg):
    global image_saved, saved_cv_image, saved_depth_image
    bridge = CvBridge()
    
    if not image_saved:
        # Convert your ROS Image message to OpenCV2 format only once
        try:
            saved_cv_image = bridge.imgmsg_to_cv2(color_msg, "bgr8")
            saved_depth_image = bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except CvBridgeError as e:
            print(e)
            return

        # Define directory to save images
        save_path = '/home/shantarao/catkin_ws/src/tm12_bottle_sorting/Saved_image'
        if not os.path.exists(save_path):
            os.makedirs(save_path)

        # Fixed filenames for the images
        image_name = 'rgb1.jpeg'
        depth_name = 'depth1.png'

        # Save RGB and depth images
        cv2.imwrite(os.path.join(save_path, image_name), saved_cv_image)
        cv2.imwrite(os.path.join(save_path, depth_name), saved_depth_image)

        # Set the flag to True indicating the images have been saved
        image_saved = True

    # Always publish the saved images
    if saved_cv_image is not None and saved_depth_image is not None:
        pub_depth.publish(bridge.cv2_to_imgmsg(saved_depth_image, encoding="passthrough"))
        pub_color.publish(bridge.cv2_to_imgmsg(saved_cv_image, encoding="bgr8"))

def main():
    rospy.init_node('image_depth_saver', anonymous=True)

    # Create subscribers for both color and depth images
    global image_sub, depth_sub, pub_color, pub_depth
    image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

    # Create publishers for both color and depth images
    pub_color = rospy.Publisher("1RGB_photo", Image, queue_size=10)
    pub_depth = rospy.Publisher("1D_depth_photo", Image, queue_size=10)
    
    # Synchronize the subscribers by time
    ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
    ts.registerCallback(callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()
