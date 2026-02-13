from controller import Controller
import rclpy

if __name__ == '__main__':
    # ROS initialization # NOTE: This is good FOR NOW, until is multithreading and not multiprocess
    rclpy.init()

    Controller().start_session()

    rclpy.shutdown()

