import rospy
import std_msgs
from geometry_msgs.msg import PoseStamped
from scripts.interfaces.generic_interface import GenericInterface


class FetchInterface(GenericInterface):
    def __init__(self):
        super().__init__()
        rospy.init_node('rostopic_wrapper_node')
        rospy.loginfo("entering rostopic wrapper server ...")

        self.graspPublisher = rospy.Publisher('/grasp', std_msgs.msg.Empty, queue_size=10)
        self.placePublisher = rospy.Publisher('/place', std_msgs.msg.Empty, queue_size=10)
        self.navigationPublisher = rospy.Publisher('/navigation', std_msgs.msg.Empty, queue_size=10)
        self.cartesianPublisher = rospy.Publisher('/cartesian', std_msgs.msg.Empty, queue_size=10)
        self.playBackPublisher = rospy.Publisher('/play_back', std_msgs.msg.Empty, queue_size=10)
        self.runsRecognitionPublisher = rospy.Publisher('/run_recognition', std_msgs.msg.Empty, queue_size=10)


        self.objectInfoSubscriber = rospy.Subscriber("/object_info",  PoseStamped, self.get_object_info)
        self.robotInfoSubscriber = rospy.Subscriber("/robot_info",  PoseStamped, self.get_robot_info)

        self.name = "Fetch"

    def grasp(self): self.graspPublisher.publish()

    def place(self):
        self.placePublisher.publish()

    def navigate(self):
        self.navigationPublisher.publish()

    def cartesian(self):
        self.cartesianPublisher.publish()

    def play_back(self):
        self.playBackPublisher.publish()

    def recognize_object(self):
        self.runRecognitionPublisher.publish()

    def get_object_info(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        #the return value depends on what is the format you need
        return
    def get_robot_info(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        #the return value depends on what is the format you need
        return

        

