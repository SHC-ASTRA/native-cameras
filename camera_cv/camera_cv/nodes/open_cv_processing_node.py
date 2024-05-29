# ROS Python library
import rclpy
# ROS node
from rclpy.node import Node
# ROS parameters
from rcl_interfaces.msg import ParameterDescriptor
# OpenCv
from cv_bridge import CvBridge
import cv2
# For getting time and date data
from datetime import datetime
# Queues, FIFO
import queue

# Typing

type captured_frame queue.SimpleQueue[dict[str, ]]

class CameraProcessingNode(Node):
    opencv_bridge = CvBridge()
    # Queue of frames for processing
    frame_queue = queue.SimpleQueue()
    # Current set of parameters
    frames_parameters = {}

    # Class constructor
    def __init__(self):
        # datetime instance of when the ROS node instance was initalzied
        dt_o = datetime.now()
        # Provide HH:MM:SS of when the ROS node instance was initalized
        dt_hhmmss = f"{dt_o.hour}_{dt_o.minute}_{dt_o.second}"
        # Call the parent class (rclpy.node.Node) constructor
        super().__init__(f"astra_cameras_cv_{dt_hhmmss}")

        self.get_logger().info(f"Subscriber node astra_cameras_cv_{dt_hhmmss} started")

        


        self.subscribe_to_cameras()

    ###
    # Functions are defined in reverse order of their dependency upon each other
    # a function that calls another (makes a subcall) will be below its subcall
    # target definition
    ###

    def display_frames(self):
        pass

    def concatenate_frames(self):
        pass

    def rotate_frames(self):
        pass
    
    # Dequeue frames from the running queue
    def dequeue_frame_set(self):
        pass
    
    # Process a frame into the running queue
    def queue_frame(self, frame):
        

    # Perform color swapping in the event the image
    # from the node is being colored incorrectly
    def color_correct_frame(self, origin_frame):
        b,g,r = cv2.split(current_frame)
        color_corrected_frame = cv2.merge((r,g,b))
        queue_frame(color_correct_frame)

    # Intake an Image message and convert it into frames
    def intake_image(self, ros_message):
        # Convert the current frame
        current_frame = self.opencv_bridge.imgmsg_to_cv2(msg)
        self.color_correct_frame(current_frame)
        
    # Intake a CompressedImage message and convert it into frames
    def intake_compressed_image(self, ros_message):
        # Convert the current frame
        current_frame = self.opencv_bridge.compressed_imgmsg_to_cv2(msg)
        self.color_correct_frame(current_frame)