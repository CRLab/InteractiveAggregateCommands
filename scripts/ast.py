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
	def __init__(self):
		pass

class AdjustBy(Task):
	def __init__(self):
		pass

class DefinePose(Task):
	def __init__(self):
		pass

class Direction(Task):
	def __init__(self):
		pass

class MoveBy(Task):
	def __init__(self):
		pass

class GoTo(Task):
	def __init__(self):
		pass

class FollowMe(Task):
	def __init__(self):
		pass

class GraspObject(Task):
	def __init__(self):
		pass

class PlaceObject(Task):
	def __init__(self):
		pass

class AskForObject(Task):
	def __init__(self):
		pass

class EnactPose(Task):
	def __init__(self):
		pass

class MoveHand(Task):
	def __init__(self):
		pass
