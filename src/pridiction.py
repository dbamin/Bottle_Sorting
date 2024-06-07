import cv2
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

# Initialize ROS node
rospy.init_node('rgbd_camera_yolo', anonymous=True)

# Create a CV bridge to convert ROS images to OpenCV format
bridge = CvBridge()

# Paths
image_save_path = '/home/rh/catkin_ws/src/rgbd_camera_input'
yolo_script_path = '/home/rh/catkin_ws/src/yolov5'
output_directory = 'static_output'  # Consistent output directory for YOLOv5

# Publisher for the class ID
result_publisher = rospy.Publisher('bottle_sorting_topic', String, queue_size=10)

def handle_single_image(msg):
    try:
        # Convert ROS image to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image_256 = cv2.resize(cv_image, (256, 256), interpolation=cv2.INTER_LINEAR)

        # Save the image
        img_filename = "camera_image_256.jpg"
        cv2.imwrite(os.path.join(image_save_path, img_filename), cv_image_256)

        # YOLOv5 detection
        detection_command = f"python3 {yolo_script_path}/detect.py --weights {yolo_script_path}/runs/train/exp/weights/best.pt --img 256 --conf 0.25 --source {os.path.join(image_save_path, img_filename)} --save-txt --name {output_directory}"
        os.system(detection_command)

        # Read the YOLOv5 output and process it
        result_file = os.path.join(yolo_script_path, f'runs/detect/{output_directory}/labels', img_filename.replace('.jpg', '.txt'))
        if os.path.exists(result_file):
            with open(result_file, 'r') as file:
                class_ids = [line.split()[0] for line in file]
                if class_ids:
                    # Write only the class ID to a new simplified result file
                    simplified_result_file = os.path.join(image_save_path, 'simplified_result.txt')
                    with open(simplified_result_file, 'w') as sfile:
                        sfile.write('\n'.join(class_ids))

                    # Publish the first class ID
                    result_publisher.publish(class_ids[0])

    except CvBridgeError as e:
        rospy.logerr(f"Failed to convert ROS image to OpenCV format: {e}")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
    finally:
        rospy.signal_shutdown("Image processed")

# Subscribe to the RGB-D camera image topic
image_sub = rospy.Subscriber("/camera/color/image_raw", Image, handle_single_image)

# Keep the script running
rospy.spin()
