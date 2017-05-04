class SucessfullyWroteObjectMsg(object):
    def __init__(self, object_name):
        self.object_name = object_name

    def __str__(self):
        return "Successfully wrote object {}".format(self.object_name)


class UnableToParseTaskListMsg(object):
    def __init__(self, task_list):
        self.task_list = task_list

    def __str__(self):
        return "Unable to parse task list {}".format(self.task_list)


class SuccessfullyExecutedTaskListMsg(object):
    def __str__(self):
        return "Successfully executed task list"


class NotYetImplementedMsg(object):
    def __str__(self):
        return "Function not yet implemented"


class ExecutingTasks(object):
    def __str__(self):
        return "Executing tasks now"


class RecordedCommandSuccessfullyMsg(object):
    def __init__(self, command_id):
        self.command_id = command_id

    def __str__(self):
        return "command {} successfully defined".format(self.command_id)


class DefineLocationSuccessMsg(object):
    def __init__(self, location_name):
        self.location_name = location_name

    def __str__(self):
        return "Location {} successfully defined".format(self.location_name)


class ReturnCommandMsg(object):
    def __init__(self, command_id, task_list):
        self.command_id = command_id
        self.task_list = task_list

    def __str__(self):
        return "Command {} is {}".format(self.command_id, self.task_list)


class AdjustByResultMsg(object):
    def __str__(self):
        return "Successfully adjusted pose"


class DefinePoseSuccessMsg(object):
    def __init__(self, pose_name):
        self.pose_name = pose_name

    def __str__(self):
        return "Pose {} successfully defined".format(self.pose_name)


class QuittingMsg(object):
    def __str__(self):
        return "Now quitting program"


class SuccessfullyParaphrasedMsg(object):
    def __str__(self):
        return "Successfully paraphrased message"


class UnsuccessfullyParaphrasedMsg(object):
    def __init__(self, task_list):
        self.task_list = task_list

    def __str__(self):
        return "Unsuccessfully paraphrased task_list {}".format(self.task_list)


class AlexaPlayback(object):
    def __str__(self):
        return "play back"


class AlexaRecordTrajectory(object):
    def __str__(self):
        return "record trajectory"


class PlaybackSuccessMsg(object):
    def __str__(self):
        return "Play back successful"


class RecordTrajectorySuccessMsg(object):
    def __str__(self):
        return "Successfully recorded trajectory"
