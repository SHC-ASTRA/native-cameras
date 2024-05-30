# rclpy
import rclpy
import rclpy.executors # Executor
# Process signaling
import signal


def compressed_sub_node():
    RosHandler.initalize()
    # Executor Multithreading
    threaded_executor = rclpy.executors.MultiThreadedExecutor()
  
    # Node Initalization

    threaded_executor.add_node(pubnode);
    threaded_executor.add_node(subnode);
    threaded_executor.spin();

    RosHandler.ros_shutdown(""" ROS NODE """)

def raw_sub_node():
    RosHandler.initalize()
    # Executor Multithreading
    threaded_executor = rclpy.executors.MultiThreadedExecutor()

    # Node Initalization

    threaded_executor.add_node(pubnode);
    threaded_executor.add_node(subnode);
    threaded_executor.spin();

    RosHandler.ros_shutdown(""" ROS NODE """)

class RosHandler():
    # ROS2 initalizer
    @staticmethod
    def initalize():
        rclpy.init(args=None)
        signal.signal(signal.SIGINT, RosHandler.sigint_handler)

    # Signal Interrupt (Ctrl+C) handler
    @staticmethod
    def sigint_handler(signal, frame):
        """
        SIGINT handler

        We have to know when to tell rclpy to shut down, because
        it's in a child thread which would stall the main thread
        shutdown sequence. So we use this handler to call
        rclpy.shutdown() and then call the previously-installed
        SIGINT handler for Flask
        """
        RosHandler.ros_shutdown()
        if prev_sigint_handler is not None:
            prev_sigint_handler(signal, frame)

    # ROS2 shutdown check
    @staticmethod
    def ros_shutdown(kill_node=None):
        # If rclpy is currently functioning
        if rclpy.ok():
            if kill_node not None:
                kill_node.get_logger().info("Shutting down rclpy.")
            rclpy.shutdown()