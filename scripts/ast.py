class Task: pass
class Command: pass

class GoTo(Task):
    def __init__(self,location):
        self.type = "GoTo"
        self.location = location
    def __str__(self):
    	return "Goto"

class Recognize(Command):
	def __init__(self, as_id):
		self.as_id = as_id

	def __str__(self):
		return "RECOGNIZE AS " + self.as_id

class Execute(Command):
	def __init__(self, task_list):
		self.task_list = task_list
	def __str__(self):
		return "Execute:{\n" + str(self.task_list) + "}\n"

class Record(Command):
	def __init__(self, task_list, command_id):
		self.command_id = command_id
		self.task_list = task_list
	def __str__(self):
		return "Record:{\nid:" + self.command_id + ",\n" + str(self.task_list) + "}\n"

class DefineLocation(Command):
	def __init__(self, location_name):
		self.location_name = location_name
	def __str__(self):
		return "Pose { name : \"" + self.location_name + "\" }"

class AdjustBy(Command):
	def __init__(self, pose_task):
		self.pose_task = pose_task
	def __str__(self):
		return "Pose { name : \"" + str(self.pose_task) + "\" }"

class DefinePose(Command):
	def __init__(self, pose_name):
		self.pose_name = pose_name
	def __str__(self):
		return "Pose { name : \"" + self.pose_name + "\" }"

class Direction(Task):
	UP = 0
	DOWN = 1
	LEFT = 2
	RIGHT = 3
	BACKWARD = 4
	FORWARD = 5
	def __init__(self, direction):
		self.direction_name = direction
		if direction is "up":
			self.direction = self.UP
		elif direction is "down":
			self.direction = self.DOWN
		elif direction is "right":
			self.direction = self.RIGHT
		elif direction is "left":
			self.direction = self.LEFT
		elif direction is "forward":
			self.direction = self.FORWARD
		elif direction is "backward":
			self.direction = self.BACKWARD

	def __str__(self):
		return self.direction

class MoveBy(Task):
	def __init__(self, direction, meters):
		self.direction = direction
		self.meters = meters
	def __str__(self):
		return "task"

class GoTo(Task):
	def __init__(self, location_name):
		self.location_name = location_name
	def __str__(self):
		return "task"

class FollowMe(Task):
	def __init__(self, seconds=0):
		self.seconds = seconds
	def __str__(self):
		return "task"

class GraspObject(Task):
	def __init__(self, object_name):
		self.object_name = object_name
	def __str__(self):
		return "task"

class PlaceObject(Task):
	def __init__(self, object_name):
		self.object_name = object_name
	def __str__(self):
		return "task"

class AskForObject(Task):
	def __init__(self, object_name, reference_name):
		self.object_name = object_name
		self.reference_name = reference_name
	def __str__(self):
		return "task"

class EnactPose(Task):
	def __init__(self, pose_name):
		self.pose_name = pose_name
	def __str__(self):
		return "enact pose " + self.pose_name

class MoveHand(Task):
	def __init__(self, location_name):
		self.location_name = location_name
	def __str__(self):
		return "Move hand to " + self.location_name

class TaskList():
	def __init__(self, task=None):
		self.tasks = []
		if task is not None:
			self.tasks.append(task)

	def __add__(self, task):
		self.tasks.append(task)
		return self

	def __str__(self):
		output = "tasks:[\n"
		for task in self.tasks:
			output += "\t" + str(task) + ",\n"
		return output + "]\n"