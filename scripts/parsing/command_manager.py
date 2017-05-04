#!/usr/bin/env python
import os

import pickle
import time
import requests
import json

from scripts.interfaces.fetch_interface import FetchInterface, GenericInterface
from scripts.parsing.task_parser import task_parser

from scripts.types.alexa_phrases import AlexaAdjustCourse, AlexaRecognizeObject, AlexaExecute, AlexaRecordCommand, \
    AlexaReturnCommand, AlexaDefinePose, AlexaSwitchRobot, AlexaPause, AlexaStop, AlexaQuit
from scripts.types.ast import Recognize, SwitchRobot, Execute, DefineLocation, AdjustBy, DefinePose, Pause, Stop, Quit, \
    ReturnCommand, MoveBy, Record, GoTo, FollowMe, GraspObject, PlaceObject, AskForObject, EnactPose, MoveHand, \
    RecordedTask
from scripts.types.messages import SucessfullyWroteObjectMsg, SuccessfullyExecutedTaskListMsg, UnableToParseTaskListMsg, \
    NotYetImplementedMsg, ExecutingTasks, RecordedCommandSuccessfullyMsg, DefineLocationSuccessMsg, ReturnCommandMsg, \
    AdjustByResultMsg, DefinePoseSuccessMsg, QuittingMsg


class PassCommand:
    def __init__(self, commandsInFilename, infoOutFilename):
        self.commandsInFilename = commandsInFilename
        self.infoOutFilename = infoOutFilename

    def read(self):
        while not os.path.isfile(self.commandsInFilename):
            time.sleep(0.1)

        return pickle.load(open(self.commandsInFilename, "rb"))

    def write(self, message):
        pickle.dump(message, open(self.infoOutFilename, "wb"))


class CommandState:
    def __init__(self):
        self.locations = {}
        self.objects = {}
        self.poses = {}
        self.commands = {}

    def addCommand(self, name, command):
        self.commands[name] = command

    def addLocation(self, name, location):
        self.locations[name] = location

    def addPose(self, name, pose):
        self.poses[name] = pose

    def addObject(self, name, object):
        self.objects[name] = object

    def getCommand(self, name):
        return self.commands[name]

    def getLocation(self, name):
        return self.locations[name]

    def getPose(self, name):
        return self.poses[name]

    def getObject(self, name):
        return self.objects[name]


class Paraphraser:
    def __init__(self, url):
        self.url = url

    def paraphraseCommand(self, phrase):
        #return requests.post(self.url + "/paraphrase", data=json.dumps({'data': phrase)).text
        if isinstance(phrase, AlexaAdjustCourse):
            return AdjustBy(MoveBy(phrase.direction, phrase.meters))

        elif isinstance(phrase, AlexaRecognizeObject):
            return Recognize(phrase.object_name)

        elif isinstance(phrase, AlexaExecute):
            #task_list = requests.post(self.url + "/paraphrase", data=json.dumps({'data': phrase.task_list)).text
            task_list = phrase.task_list
            return Execute(task_list)

        elif isinstance(phrase, AlexaRecordCommand):
            # task_list = requests.post(self.url + "/paraphrase", data=json.dumps({'data': phrase.task_list)).text
            task_list = phrase.task_list
            return Record(task_list, phrase.command_id)

        elif isinstance(phrase, AlexaReturnCommand):
            return ReturnCommand(phrase.command_id)

        elif isinstance(phrase, AlexaDefinePose):
            return DefinePose(phrase.pose)

        elif isinstance(phrase, AlexaSwitchRobot):
            return SwitchRobot(phrase.robot)

        elif isinstance(phrase, AlexaPause):
            return Pause()

        elif isinstance(phrase, AlexaStop):
            return Stop()

        elif isinstance(phrase, AlexaQuit):
            return Quit()

        return phrase

    def add_task(self, task_name):
        requests.post(self.url + "/add_task", data=json.dumps({'data': task_name}))

    def add_location(self, location_name):
        requests.post(self.url + "/add_location", data=json.dumps({'data': location_name}))

    def add_pose(self, pose_name):
        requests.post(self.url + "/add_pose", data=json.dumps({'data': pose_name}))

    def add_object(self, object_name):
        requests.post(self.url + "/add_object", data=json.dumps({'data': object_name}))


class CommandParserClient:
    def __init__(self, command_passer, robot_interface, command_state, paraphrase_detector, task_parser):
        self.robotInterface = robot_interface
        self.commandPasser = command_passer
        self.commandState = command_state
        self.paraphraseDetector = paraphrase_detector
        self.taskParser = task_parser

    def interpretTask(self, task):
        if isinstance(task, GoTo):
            location = self.commandState.getLocation(task.location_name)
            result = self.robotInterface.moveTo(location)
            print(result)
            return result

        elif isinstance(task, MoveBy):
            result = self.robotInterface.cartesianTransform(task.direction, task.meters)
            print(result)
            return result

        elif isinstance(task, FollowMe):
            self.robotInterface.followMe()

        elif isinstance(task, GraspObject):
            self.robotInterface.graspObject(self.commandState.getObject(task.object_name))

        elif isinstance(task, PlaceObject):
            self.robotInterface.placeObject(self.commandState.getObject(task.object_name))

        elif isinstance(task, AskForObject):
            self.robotInterface.askForObject()

        elif isinstance(task, EnactPose):
            self.robotInterface.enactPose(self.commandState.getPose(task.pose_name))

        elif isinstance(task, MoveHand):
            pass

        elif isinstance(task, RecordedTask):
            task_list = self.commandState.getCommand(task.task_name)
            self.interpretTaskList(task_list)

    def interpretTaskList(self, task_list):
        parsed_list = self.taskParser.parse(task_list)
        if parsed_list is None:
            return UnableToParseTaskListMsg(task_list)

        for task in parsed_list:
            self.interpretTask(task)

        return SuccessfullyExecutedTaskListMsg()

    def interpretCommand(self):
        command = self.commandPasser.read()
        command = self.paraphraseDetector.paraphraseCommand(command)

        if isinstance(command, Recognize):
            self.robotInterface.recognize_object()
            self.commandPasser.write(SucessfullyWroteObjectMsg(command.as_id))

        elif isinstance(command, SwitchRobot):
            self.commandPasser(NotYetImplementedMsg())

        elif isinstance(command, Execute):
            self.commandPasser.write(ExecutingTasks())
            result = self.interpretTaskList(command.task_list)
            print(result)

        elif isinstance(command, Record):
            self.commandState.addCommand(command.command_id, self.taskParser.parse(command.task_list))
            self.paraphraseDetector.add_task(command.command_id)
            self.commandPasser(RecordedCommandSuccessfullyMsg(command.command_id))

        elif isinstance(command, DefineLocation):
            currentLocation = self.robotInterface.getCurrentLocation()
            self.commandState.addLocation(command.location_name, currentLocation)
            self.paraphraseDetector.add_location(command.location_name)
            self.commandPasser.write(DefineLocationSuccessMsg(command.location_name))

        elif isinstance(command, ReturnCommand):
            task = self.commandState.getCommand(command.task_name)
            self.commandPasser.write(ReturnCommandMsg(command.task_name, task))

        elif isinstance(command, AdjustBy):
            result = self.robotInterface.cartesianTransform(command.pose_task.direction, command.pose_task.meters)
            self.commandPasser.write(AdjustByResultMsg())

        elif isinstance(command, DefinePose):
            currentPose = self.robotInterface.getCurrentPose()
            self.commandState.addPose(command.pose_name, currentPose)
            self.paraphraseDetector.add_pose(command.pose_name)
            self.commandPasser.write(DefinePoseSuccessMsg(command.pose_name))

        elif isinstance(command, Pause):
            self.commandPasser(NotYetImplementedMsg())

        elif isinstance(command, Stop):
            self.commandPasser(NotYetImplementedMsg())

        elif isinstance(command, Quit):
            self.commandPasser.write(QuittingMsg())
            exit(0)


if __name__ == '__main__':
    command_passer = PassCommand("in.txt", "out.txt")
    robot_interface = GenericInterface()
    command_state = CommandState()
    paraphrase_detector = Paraphraser('localhost:8080')

    command_parser = CommandParserClient(command_passer=command_passer,
                                         robot_interface=robot_interface,
                                         command_state=command_state,
                                         paraphrase_detector=paraphrase_detector,
                                         task_parser=task_parser)

    while True:
        command_parser.interpretCommand()
        time.sleep(0.1)

