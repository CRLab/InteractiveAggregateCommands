class Task: pass

class GoTo(Task):
    def __init__(self,location):
        self.type = "GoTo"
        self.location = location

class Recognize(Task):
	def __init__(self, as_id):
		self.as_id = as_id

	def __str__(self):
		return "RECOGNIZE AS " + self.as_id

class Execute(Task):
	def __init__(self, task_list):
		self.task_list = task_list

class Record(Task):
	def __init__(self, task_list, command_id):
		self.command_id = command_id
		self.task_list = task_list

class DefineLocation(Task):
	def __init__(self, location_name):
		self.location_name = location_name

class AdjustBy(Task):
	def __init__(self, pose_task):
		self.pose_task = pose_task

class DefinePose(Task):
	def __init__(self, pose_name):
		self.pose_name = pose_name

class Direction(Task):
	UP = 0
	DOWN = 1
	LEFT = 2
	RIGHT = 3
	BACKWARD = 4
	FORWARD = 5
	def __init__(self, direction):
		self.direction = direction

class MoveBy(Task):
	def __init__(self, direction, meters):
		self.direction = direction
		self.meters = meters

class GoTo(Task):
	def __init__(self, location_name):
		self.location_name = location_name

class FollowMe(Task):
	def __init__(self, seconds=0):
		self.seconds = seconds

class GraspObject(Task):
	def __init__(self, object_name):
		self.object_name = object_name

class PlaceObject(Task):
	def __init__(self, object_name):
		self.object_name = object_name

class AskForObject(Task):
	def __init__(self, object_name, reference_name):
		self.object_name = object_name
		self.reference_name = reference_name

class EnactPose(Task):
	def __init__(self, pose_name):
		self.pose_name = pose_name

class MoveHand(Task):
	def __init__(self, location_name):
		self.location_name = location_name
