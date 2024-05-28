import rclpy
from rclpy.executors import Executor as RosExecutor
from rclpy.node import Node
# ROS2CLI API
import ros2topic.api # get_topic_names_and_types(node=NODE_INSTANCE)
# OpenCv
from cv_bridge import CvBridge
import cv2
# Interfaces
import sensor_msgs.msg
# For getting time and date data
from datetime import datetime
# Displaying cameras
import matplotlib.pyplot
# Multithreading
import threading
# Processing signaling
import signal

def main(args=None):
    # Initalize ROS
    rclpy.init(args=args)

    # Initalizing the camera node
    camera_node = CameraProcessingNode()

    rclpy.spin(camera_node)

    # ROS2 threading
    # threading.Thread(target=ros2_thread_launch, args=[camera_node]).start()
    # Set the handler
    signal.signal(signal.SIGINT, sigint_handler)

    # Kill rclpy proccess
    rclpy.shutdown()

if __name__ == "__main__":
    main()

def sigint_handler(signal, frame):
    """
    SIGINT handler

    We have to know when to tell rclpy to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    """
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal, frame)

# Thread worker that spins the node
def ros2_thread_launch(node_argument): 
    node_argument.get_logger().info("Attempting to spin node.")
    # ROS Node Execution (Spin)
    rclpy.spin(node_argument)
    node_argument.get_logger().info("Leaving ROS2 spin thread.")

class CameraProcessingNode(Node):
    active_subscribers = []
    opencv_bridge = CvBridge()
    
    # Function constructor
    def __init__(self):
        # datetime instance of when the ROS node instance was initalzied
        dt_o = datetime.now()
        # Provide HH:MM:SS of when the ROS node instance was initalized
        dt_hhmmss = f"{dt_o.hour}_{dt_o.minute}_{dt_o.second}"
        # Call the parent class (rclpy.node.Node) constructor
        super().__init__(f"astra_cameras_cv_{dt_hhmmss}")

        self.get_logger().info(f"Subscriber node astra_cameras_cv_{dt_hhmmss} started")

        self.subscribe_to_cameras()

    # Get topics by fetching from the API
    def get_topics(self):
        return ros2topic.api.get_topic_names_and_types(node=self)

    def fetch_image_topics(self):
        # dict (JSON) response data
        ret_data = {}
        # Convert the 1d array of tuples to a dict (JSON)
        # Make use of the ros2cli / ros2topic API
        for topic_data in self.get_topics():
            # Set variables for readability
            topic_name = topic_data[0]
            topic_type = topic_data[1][0]
            # Check the topic's type
            if topic_type != 'sensor_msgs/msg/CompressedImage':
                # If it does not exist, skip this data
                continue
            ret_data[topic_name] = topic_type
        return ret_data

    def subscribe_to_cameras(self):
        # Get the camera topics
        topic_keys = self.fetch_image_topics().keys()
        self.get_logger().info(f"Camera topics seen: {topic_keys}")
        for image_topic in topic_keys:
            self.get_logger().info(f"Attempting to subscribe to topic \"{image_topic}\"")
            # Callback provided to general subscribers for cameras
            # msg is of type sensor_msgs.msg.CompressedImage
            def general_camera_callback(msg):
                self.get_logger().info(f"Camera data received.")
                current_frame = self.opencv_bridge.compressed_imgmsg_to_cv2(msg)
            
                b,g,r = cv2.split(current_frame)
                frame_rgb = cv2.merge((r,g,b))
            
                cv2.imshow(f"{image_topic}", frame_rgb)
                cv2.waitKey(1)
                
                # matplotlib.pyplot.imshow(current_frame)
                # matplotlib.pyplot.title(f"{image_topic}")
                # matplotlib.pyplot.show()
            # Create the subscriber
            self.create_subscription(
                sensor_msgs.msg.CompressedImage,
                image_topic,
                general_camera_callback,
                10
            )