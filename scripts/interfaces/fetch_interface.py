import rospy
from rospy import std_msgs

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
        self.runRecognitionPublisher = rospy.Publisher('/run_recognition', std_msgs.msg.Empty, queue_size=10)

        self.name = "Fetch"

    def grasp(self):
        self.graspPublisher.publish()

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
