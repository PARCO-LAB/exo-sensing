from controller import Controller
import rospy

if __name__ == '__main__':
    # ROS initialization
    rospy.init_node("sensors_acquisition_node")

    Controller().start_session()

