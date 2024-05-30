# ROS Python library
import rclpy
# ROS node
from rclpy.node import Node
# ROS parameters
import rcl_interfaces.msg # ParameterDescriptor
# ROS interfaces
    # InternodeCommunication InternodeCameraCompressed InternodeCameraRaw
import camera_cv_interfaces.msg 
# OpenCv
from cv_bridge import CvBridge
import cv2
# For getting time and date data
from datetime import datetime
# Queues, FIFO
import queue

# type captured_frame queue.SimpleQueue[dict[str, ]]

class CVProcessingNode(Node):
    opencv_bridge = CvBridge()
    # Queue of frames for processing
    frame_queue = 
    # Frame values, for queue management
    frame_values = {}
    # Subscriptions
    raw_sub = None
    compressed_sub = None
    # Subscription Config
    sub_config = None

    # Class constructor
    def __init__(self, in_config={raw: True, compressed: True}):
        # datetime instance of when the ROS node instance was initalzied
        dt_o = datetime.now()
        # Provide HH:MM:SS of when the ROS node instance was initalized
        dt_hhmmss = f"{dt_o.hour}_{dt_o.minute}_{dt_o.second}"
        # Call the parent class (rclpy.node.Node) constructor
        super().__init__(f"open_cv_processor_{dt_hhmmss}")

        self.get_logger().info(f"Subscriber node open_cv_processor_{dt_hhmmss} started")

        # Done for each new image recieved
        # self.declare_parameter('my_parameter', 'world')

        # Subscription configuration assignment
        self.sub_config = in_config
        # Subscribe to relays, checking config
        self.subscribe_to_relays()

    def subscribe_to_relays(self):
        if self.sub_config.raw:

            raw_sub = self.create_subscription(
                camera_cv_interfaces.msg.InternodeCameraRaw,
                "/astra/intraprocess/native_cameras/cam_proc_out/raw"
                self.intake_raw_image
                10
            )
        if self.sub_config.compressed:
            compressed_sub = self.create_subscription(
                camera_cv_interfaces.msg.InternodeCameraCompressed,
                "astra/intraprocess/native_cameras/cam_proc_out/compressed"
                self.intake_compressed_image
                10
            )

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
        pass
        

    # Perform color swapping in the event the image
    # from the node is being colored incorrectly
    def color_correct_frame(self, origin_frame):
        b,g,r = cv2.split(current_frame)
        color_corrected_frame = cv2.merge((r,g,b))
        queue_frame(color_correct_frame)

    def inital_process_frame(self, frame, internode_data):
        # Process through the data
        # add it to a list and process
        # if frame.

        self.color_correct_frame(current_frame)

    # Intake an Image message and convert it into frames
    def intake_raw_image(self, ros_message):
        # Convert the current frame
        current_frame = self.opencv_bridge.imgmsg_to_cv2(msg.raw_image.data)
        inital_process_frame(current_frame, msg.internode_data)
        
    # Intake a CompressedImage message and convert it into frames
    def intake_compressed_image(self, ros_message):
        # Convert the current frame
        current_frame = self.opencv_bridge.compressed_imgmsg_to_cv2(msg.compressed_image.data)
        inital_process_frame(current_frame, msg.internode_data)