class AlexaMsg(object):
    def __init__(self):
        self.statement = ""

    def __str__(self):
        return self.statement


class AlexaAdjustCourse(AlexaMsg):
    def __init__(self, direction, meters):
        super().__init__()
        self.statement = "adjust course by moving {} by {}".format(direction, meters)
        self.direction = direction
        self.meters = meters


class AlexaRecognizeObject(AlexaMsg):
    def __init__(self, object_name):
        super().__init__()
        self.object_name = object_name
        self.statement = "recognize object as {}".format(object_name)


class AlexaExecute(AlexaMsg):
    def __init__(self, task_list):
        super().__init__()
        self.task_list = task_list
        self.statement = "execute {}".format(task_list)


class AlexaRecordCommand(AlexaMsg):
    def __init__(self, task_list, command_id):
        super().__init__()
        self.task_list = task_list
        self.command_id = command_id
        self.statement = "record {} as {}".format(task_list, command_id)


class AlexaReturnCommand(AlexaMsg):
    def __init__(self, command_id):
        super().__init__()
        self.command_id = command_id
        self.statement = "what is command {}".format(command_id)


class AlexaDefinePose(AlexaMsg):
    def __init__(self, pose):
        super().__init__()
        self.pose = pose
        self.statement = "you are in pose {}".format(pose)


class AlexaSwitchRobot(AlexaMsg):
    def __init__(self, robot):
        super().__init__()
        self.robot = robot
        self.statement = "switch to robot {}".format(robot)


class AlexaPause(AlexaMsg):
    def __init__(self):
        super().__init__()
        self.statement = "pause"


class AlexaStop(AlexaMsg):
    def __init__(self):
        super().__init__()
        self.statement = "stop"


class AlexaQuit(AlexaMsg):
    def __init__(self):
        super().__init__()
        self.statement = "quit"
