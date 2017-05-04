class Task:
    pass


class Command:
    pass


class Recognize(Command):
    def __init__(self, as_id):
        self.as_id = as_id

    def __str__(self):
        return "Recognize { name : \"" + self.as_id + "\" }"

    def pretty_print(self):
        return "recognize object as " + self.as_id


class SwitchRobot(Command):
    def __init__(self, robot_name):
        self.robot_name = robot_name

    def __str__(self):
        return "SwitchRobot { name : \"" + self.robot_name + "\" }"

    def pretty_print(self):
        return "switch to robot " + self.robot_name


class Execute(Command):
    def __init__(self, task_list):
        self.task_list = task_list

    def __str__(self):
        return "Execute:{\n" + str(self.task_list) + "}\n"

    def pretty_print(self):
        return "execute " + self.task_list.pretty_print()


class Record(Command):
    def __init__(self, task_list, command_id):
        self.command_id = command_id
        self.task_list = task_list

    def __str__(self):
        return "Record:{\nid:" + str(self.command_id) + ",\n" + str(self.task_list) + "}\n"

    def pretty_print(self):
        return "record {} as {}".format(self.command_id, self.task_list.pretty_print())


class DefineLocation(Command):
    def __init__(self, location_name):
        self.location_name = location_name

    def __str__(self):
        return "Location { name : \"" + self.location_name + "\" }"

    def pretty_print(self):
        return "you are in location " + self.location_name


class AdjustBy(Command):
    def __init__(self, pose_task):
        self.pose_task = pose_task

    def __str__(self):
        return "AdjustBy { name : \"" + str(self.pose_task) + "\" }"

    def pretty_print(self):
        return "adjust course by " + self.pose_task.pretty_print()


class DefinePose(Command):
    def __init__(self, pose_name):
        self.pose_name = pose_name

    def __str__(self):
        return "Pose { name : \"" + self.pose_name + "\" }"

    def pretty_print(self):
        return "you are in pose " + self.pose_name


class ReturnCommand(Command):
    def __init__(self, task_name):
        self.task_name = task_name

    def __str__(self):
        return "ReturnTask { name : \"" + self.task_name + "\" }"

    def pretty_print(self):
        return "what is command " + self.task_name


class Pause(Command):
    def __init__(self):
        pass

    def __str__(self):
        return "Pause"

    def pretty_print(self):
        return "pause"


class Stop(Command):
    def __init__(self):
        pass

    def __str__(self):
        return "Stop"

    def pretty_print(self):
        return "stop"


class Quit(Command):
    def __init__(self):
        pass

    def __str__(self):
        return "Quit"

    def pretty_print(self):
        return "quit"


class Direction:
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
        return "Direction{ " + self.direction_name + " }"

    def pretty_print(self):
        return self.direction_name


class GoTo(Task):
    def __init__(self, location_name):
        self.location_name = location_name

    def __str__(self):
        return "Goto { location_name : \"" + self.location_name + "\" }"

    def pretty_print(self):
        return "go to " + self.location_name


class MoveBy(Task):
    def __init__(self, direction, meters):
        self.direction = direction
        self.meters = meters

    def __str__(self):
        return "MoveBy { direction : \"" + str(self.direction) + "\", meters : " + str(self.meters) + " }"

    def pretty_print(self):
        return "moving " + self.direction.pretty_print() + " by " + str(self.meters) + " meters"


class FollowMe(Task):
    def __init__(self, seconds=0):
        self.seconds = seconds

    def __str__(self):
        if self.seconds == 0:
            return "FollowMe{ time : indefinitely }"
        else:
            return "FollowMe{ time : " + str(self.seconds) + " }"

    def pretty_print(self):
        if self.seconds == 0:
            return "follow me"
        else:
            return "follow me for " + str(self.seconds) + " seconds"


class GraspObject(Task):
    def __init__(self, object_name):
        self.object_name = object_name

    def __str__(self):
        return "GraspObject{ name : \"" + self.object_name + "\" }"

    def pretty_print(self):
        return "grasp " + self.object_name


class PlaceObject(Task):
    def __init__(self, object_name):
        self.object_name = object_name

    def __str__(self):
        return "PlaceObject{ name: \"" + self.object_name + "\" }"

    def pretty_print(self):
        return "place " + self.object_name


class AskForObject(Task):
    def __init__(self, reference_name):
        self.reference_name = reference_name

    def __str__(self):
        return "AskForObject{ reference_name: \"" + self.reference_name + "\" }"

    def pretty_print(self):
        return "ask for object as " + self.reference_name


class EnactPose(Task):
    def __init__(self, pose_name):
        self.pose_name = pose_name

    def __str__(self):
        return "EnactPose{" + self.pose_name + "}"

    def pretty_print(self):
        return "enact pose " + self.pose_name


class MoveHand(Task):
    def __init__(self, location_name):
        self.location_name = location_name

    def __str__(self):
        return "MoveHand{ name: \"{}\"}".format(self.location_name)

    def pretty_print(self):
        return "move hand to " + self.location_name


class TaskList:
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

    def __iter__(self):
        return iter(self.tasks)

    def pretty_print(self):
        output = ""
        for task in self.tasks[:-1]:
            output += task.pretty_print() + " and then "
        return output + self.tasks[-1].pretty_print()


class RecordedTask(Task):
    def __init__(self, task_name):
        self.task_name = task_name

    def __str__(self):
        return "RecordedTask : { name : \"" + self.task_name + "\" }"

    def pretty_print(self):
        return self.task_name
