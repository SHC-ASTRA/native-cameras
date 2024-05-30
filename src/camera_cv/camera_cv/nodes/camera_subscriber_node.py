# ROS Python library
import rclpy
# ROS node
from rclpy.node import Node
# ROS parameters
import rcl_interfaces.msg # ParameterDescriptor
# ROS interfaces
    # InternodeCommunication InternodeCameraCompressed InternodeCameraRaw
import camera_cv_interfaces.msg
import sensor_msgs.msg # CompressedImage Image
# For getting time and date data
from datetime import datetime

class CameraProcessingNode(Node):
    active_subscribers = []
    # Array of last TOPIC_NUMBER - 1 frame datas
    frame_data = []
    frame_data_val = len(fetch_image_topics.keys())
    # If image transport is compressed
    compressed_enable = False;
    # Publisher to the OpenCV processor
    opencv_publisher = None
    # Interface
    interface_type_str = None
    interface_type = None
    # Node name
    self.name_assignment = None

    # Function constructor
    def __init__(self, set_compressed=False):
        # Compressed image transport
        self.compressed_enable = set_compressed
        # Image Interface Type
        self.interface_type_str = f"'sensor_msgs/msg/{"Compressed" if self.compressed_enable else ''}Image'"
        if self.compressed_enable:
            self.interface = sensor_msgs.msg.CompressedImage
        else:
            self.interface = sensor_msgs.msg.Image
        # datetime instance of when the ROS node instance was initalzied
        dt_o = datetime.now()
        # Provide HH:MM:SS of when the ROS node instance was initalized
        dt_hhmmss = f"{dt_o.hour}_{dt_o.minute}_{dt_o.second}"
        self.name_assignment = f"native_cameras_subscriber{'_compressed' if self.compressed_enable else ''}_{dt_hhmmss}" 
        # Call the parent class (rclpy.node.Node) constructor
        super().__init__(self.name_assignment)

        self.get_logger().info(f"Node {self.name_assignment} started")
        
        self.get_logger().info(f"{self.name_assignment} is making use of interface topic {interface_type_str}")


        cur_message = "Attempting to create publisher"
        # Publishing to an OpenCV processor
        if self.compressed_enable:
            # Compressed Publisher
            self.opencv_publisher = self.create_publisher(
                camera_cv_interfaces.msg.InternodeCameraCompressed,
                "/astra/intraprocess/native_cameras/cam_proc_out/compressed",
                10
            )
            # Update the output message
            cur_message += f"with topic \"/astra/intraprocess/native_cameras/cam_proc_out/compressed\""
        else:
            # Raw Publisher
            self.opencv_publisher = self.create_publisher(
                camera_cv_interfaces.msg.InternodeCameraRaw,
                "/astra/intraprocess/native_cameras/cam_proc_out/raw",
                10
            )
            # Update the output message
            cur_message += f"with topic \"/astra/intraprocess/native_cameras/cam_proc_out/raw\""

        # Display the message
        self.get_logger().info(f"{cur_message}")

        # Subscribe to available cameras
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
            if topic_type != interface_type:
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
            def general_camera_callback(msg):
                self.get_logger().info(f"Camera data received.")


                # This data should be passed off to a node, go nowhere further,
                # or some third thing

                # PUBLISH TO THE SUBSCRIBER
                # PUBLISH TO THE SUBSCRIBER
                # PUBLISH TO THE SUBSCRIBER
                # PUBLISH TO THE SUBSCRIBER
                # PUBLISH TO THE SUBSCRIBER
                # PUBLISH TO THE SUBSCRIBER


                """ 
                current_frame = self.opencv_bridge.compressed_imgmsg_to_cv2(msg)
                # Perform color swapping in the event the image
                # from the node is being colored incorrectly
                b,g,r = cv2.split(current_frame)
                frame_rgb = cv2.merge((r,g,b))
            
                cv2.imshow(f"{image_topic}", frame_rgb)
                cv2.waitKey(1)
                """
                
            # Create the subscriber
            active_subscribers.append(
                self.create_subscription(
                    self.interface_type,
                    image_topic,
                    general_camera_callback,
                    10
                )
            )