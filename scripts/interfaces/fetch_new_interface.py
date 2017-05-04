import rospy
import std_msgs
import geometry_msgs
import moveit_commander
import actionlib
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from scripts.interfaces.generic_interface import GenericInterface
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface
from robot_controllers_msgs.msg import (QueryControllerStatesAction,
                                       QueryControllerStatesGoal,
                                       ControllerState)
from control_msgs.msg import (FollowJointTrajectoryGoal,
                             FollowJointTrajectoryAction,
                             FollowJointTrajectoryResult)


class FetchInterface(GenericInterface):
    def __init__(self):
        super().__init__()
        rospy.init_node('rostopic_wrapper_node')
        rospy.loginfo("entering rostopic wrapper server ...")

        self.graspPublisher = rospy.Publisher('/grasp', std_msgs.msg.Empty, queue_size=10)
        self.placePublisher = rospy.Publisher('/place', std_msgs.msg.Empty, queue_size=10)
        self.navigationPublisher = rospy.Publisher('/navigation', PoseStamped, queue_size=10)
        self.cartesianPublisher = rospy.Publisher('/cartesian', geometry_msgs.msg.Transform, queue_size=10)
        self.playBackPublisher = rospy.Publisher('/play_back', std_msgs.msg.Empty, queue_size=10)
        self.runsRecognitionPublisher = rospy.Publisher('/run_recognition', std_msgs.msg.Empty, queue_size=10)
        self.startRecordingPublisher = rospy.Publisher('/start_recording', std_msgs.msg.Empty, queue_size=10)


        self.objectInfoSubscriber = rospy.Subscriber("/object_info",  PoseStamped, self.get_object_info)
        self.robotInfoSubscriber = rospy.Subscriber("/robot_info",  PoseStamped, self.get_robot_info)
        self.armJointsSubscriber = rospy.Subscriber('/joint_states', JointState, self.get_current_arm_joint_values)

        self.name = "Fetch"
        self.joint_states_list = []
        self.move_group = MoveGroupInterface("arm", "base_link")
        self._controller_client = actionlib.SimpleActionClient(
                                        controller_states,
                                        QueryControllerStatesAction)
        self._controller_client.wait_for_server()
        self._gravity_comp_controllers = ["arm_controller/gravity_compensation"]
        self._non_gravity_comp_controllers = list()
        self._non_gravity_comp_controllers.append(
                    "arm_controller/follow_joint_trajectory")
        self._non_gravity_comp_controllers.append(
                    "arm_with_torso_controller/follow_joint_trajectory")


    def grasp(self):
        self.graspPublisher.publish()

    def place(self):
        self.placePublisher.publish()

    #Input: base_pose: a list contains 6 elements (x, y, x, y, z, w) indicating the position and orientation
    #       NOTE: there is no z in the position. z will be hard coded as 0 since the robot is on the ground
    def navigate(self, base_pose):
        while not rospy.is_shutdown():
            goal = geometry_msgs.msg.PoseStamped()
            goal.pose.position = geometry_msgs.msg.Point(base_pose[0], base_pose[1], 0)
            goal.pose.orientation = geometry_msgs.msg.Quaternion(base_pose[2], base_pose[3], base_pose[4], base_pose[5])
            self.navigationPublisher.publish(goal)
            break
        return


    def up():
        return cartesian_direction = geometry_msgs.msg.Vector3(0, 0, 1)
    def down():
        return cartesian_direction = geometry_msgs.msg.Vector3(0, 0, -1)
    def left():
        return cartesian_direction = geometry_msgs.msg.Vector3(0, 1, 0)
    def right():
        return cartesian_direction = geometry_msgs.msg.Vector3(0, -1, 1)
    def forward():
        return cartesian_direction = geometry_msgs.msg.Vector3(1, 0, 0)
    def backward():
        return cartesian_direction = geometry_msgs.msg.Vector3(-1, 0, 0)


    #Input: move_goal: contains two fields: name(string, like up and down), number
    def cartesian(self, direction, meters):
        # self.cartesianPublisher.publish()
        while not rospy.is_shutdown():
            cartesian_states = {0 : up,
                                1 : down,
                                2 : left,
                                3 : right,
                                4 : forward,
                                5 : backward,

            }
            cartesian_direction = cartesian_states[direction.number]()
            cartesian_goal = geometry_msgs.msg.Transform()
            cartesian_goal.translation.x = cartesian_direction.x * meters
            cartesian_goal.translation.y = cartesian_direction.y * meters
            cartesian_goal.translation.z = cartesian_direction.z * meters
            self.cartesianPublisher.publish(cartesian_goal)
            break
        return

    def start_recording(self):
        self.startRecordingPublisher.publish()

    def play_back(self):
        self.playBackPublisher.publish()

    def recognize_object(self):
        self.runRecognitionPublisher.publish()

    def get_object_info(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        #the return value depends on what is the format you need
        return (position, orientation)
    def get_robot_info(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        #the return value depends on what is the format you need
        return (position, orientation)
    def get_current_arm_joint_values(self, msg):
        self.joint_states_list = list(msg.position)
        self.joint_states_list = joint_states_list[6:]
        #the return value is a list contains the current arm joint values
        return [joint_states_list]

    # when you record some arm joint values as specific name, and want to move it back: use this function directly. You do not need to 
    # publish a topic to do this. Calling this function should be fine.
    def move_arm_to_pose(self, msg):
        self.un_relax_arm()
        for joint in self.joint_values:
            if rospy.is_shutdown():
                break
            # Plans the joints in joint_names to angles in pose
            self.move_group.moveToJointPosition(self.joint_names, joint, wait=False)

            # Since we passed in wait=False above we need to wait here
            self.move_group.get_move_action().wait_for_result()
            result = self.move_group.get_move_action().get_result()

            if result:
                # Checking the MoveItErrorCode
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Play Back ...")
                else:
                    # If you get to this point please search for:
                    # moveit_msgs/MoveItErrorCodes.msg
                    rospy.logerr("Arm goal in state: %s",
                                 self.move_group.get_move_action().get_state())
            else:
                rospy.logerr("MoveIt! failure no result returned.")

    def un_relax_arm(self):
        '''Turns on gravity compensation controller and turns
        off other controllers
        '''

        goal = QueryControllerStatesGoal()

        for controller in self._non_gravity_comp_controllers:
            state = ControllerState()
            state.name = controller
            state.state = state.RUNNING
            goal.updates.append(state)

        self._controller_client.send_goal(goal)




        

