## Class-Level Documentation

### CameraProcessingNode inherits rclpy.Node

#### Required Co-Processes
* v4l2_cameras
* CVProcessingNode

#### ROS2 Communication
* Depends on `set_compressed` in constructor call
* Subscription to all currently running topics of type `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`
* A publisher to the OpenCV Processing Node on topic `/astra/intraprocess/native_cameras/cam_proc_out/raw` or `/astra/intraprocess/native_cameras/cam_proc_out/compressed`

#### Node Name:
* native_cameras_subscriber_HH:MM:SS or native_cameras_subscriber_compressed_HH:MM:SS
* HH:MM:SS corresponds to hour minute second

#### Function:
Performs subscription to the cameras that are currently running when the Node is launched. This data is sent to a topic that will be recieved by the OpenCV Processing Node.

#### Constructor
`set_compressed` must be set in order to handle `sensor_msgs/msg/CompressedImage`

### CVProcessingNode inherits rclpy.Node

#### Required Co-Processes
* Publisher to `/astra/intraprocess/native_cameras/cam_proc_out`

#### ROS2 Communication
* Subscription to `/astra/intraprocess/native_cameras/cam_proc_out/raw`
* Subscription to `/astra/intraprocess/native_cameras/cam_proc_out/compressed`

#### Node Name:
* open_cv_processor_HH:MM:SS
* HH:MM:SS corresponds to hour minute second

#### Function:
Recieves data from the given ROS2 subscriptions and produces it as a visual set of images in a grapihcal window

### Constructor
`in_config` is default `{raw: True, compressed: True}`, modify boolean values in order to disable a subscriber