#!/usr/bin/env python
import os

import pickle
import time
import requests
import json
import logging

from scripts.interfaces.fetch_interface import FetchInterface, GenericInterface
from scripts.parsing.task_parser import task_parser

from scripts.types.alexa_phrases import AlexaAdjustCourse, AlexaRecognizeObject, AlexaExecute, AlexaRecordCommand, \
    AlexaReturnCommand, AlexaDefinePose, AlexaSwitchRobot, AlexaPause, AlexaStop, AlexaQuit
from scripts.types.ast import Recognize, SwitchRobot, Execute, DefineLocation, AdjustBy, DefinePose, Pause, Stop, Quit, \
    ReturnCommand, MoveBy, Record, GoTo, FollowMe, GraspObject, PlaceObject, AskForObject, EnactPose, MoveHand, \
    RecordedTask, TaskNotFound
from scripts.types.messages import SucessfullyWroteObjectMsg, SuccessfullyExecutedTaskListMsg, UnableToParseTaskListMsg, \
    NotYetImplementedMsg, ExecutingTasks, RecordedCommandSuccessfullyMsg, DefineLocationSuccessMsg, ReturnCommandMsg, \
    AdjustByResultMsg, DefinePoseSuccessMsg, QuittingMsg, SuccessfullyParaphrasedMsg, UnsuccessfullyParaphrasedMsg


class PassCommand:
    def __init__(self, commandsInFilename, infoOutFilename):
        self.commandsInFilename = commandsInFilename
        self.infoOutFilename = infoOutFilename

    def read(self):
        time_waited = 0
        while not os.path.isfile(self.commandsInFilename) and time_waited < 5:
            time.sleep(0.1)
            time_waited += 0.1

        if time_waited >= 5:
            return None

        obj = pickle.load(open(self.commandsInFilename, "rb"))
        self.removeMessage()
        return obj

    def write(self, message):
        pickle.dump(message, open(self.infoOutFilename, "wb"))

    def removeMessage(self):
        if os.path.isfile(self.commandsInFilename):
            os.remove(self.commandsInFilename)


class CommandState:
    def __init__(self):
        self.locations = {}
        self.objects = {}
        self.poses = {}
        self.commands = {}
        self.logger = logging.getLogger(__name__)

    def addCommand(self, name, command):
        self.commands[name] = command

    def addLocation(self, name, location):
        self.locations[name] = location

    def addPose(self, name, pose):
        self.poses[name] = pose

    def addObject(self, name, val):
        self.objects[name] = val

    def getCommand(self, name):
        if name in self.commands:
            return self.commands[name]
        else:
            self.logger.error("Could not find command {}".format(name))
            return TaskNotFound()

    def getLocation(self, name):
        if name in self.locations:
            return self.locations[name]
        else:
            self.logger.error("Could not find location {}".format(name))
            return None

    def getPose(self, name):
        if name in self.poses:
            return self.poses[name]
        else:
            self.logger.error("Could not find pose {}".format(name))
            return None

    def getObject(self, name):
        if name in self.objects:
            return self.objects[name]
        else:
            self.logger.error("Could not find object {}".format(name))
            return None


class Paraphraser:
    def __init__(self, url, task_parser):
        self.url = url
        self.headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
        self.task_parser = task_parser
        self.logger = logging.getLogger(__name__)
        try:
            print(requests.post(url + "/start_test_env"))
        except requests.exceptions.RequestException as e:  # This is the correct syntax
            pass

    def paraphraseCommand(self, phrase):
        if isinstance(phrase, AlexaAdjustCourse):
            return AdjustBy(MoveBy(phrase.direction, phrase.meters)), SuccessfullyParaphrasedMsg()

        elif isinstance(phrase, AlexaRecognizeObject):
            return Recognize(phrase.object_name), SuccessfullyParaphrasedMsg()

        elif isinstance(phrase, AlexaExecute):
            try:
                task_list = requests.post(self.url + "/paraphrase_this",
                                          data=json.dumps({'data': phrase.task_list}),
                                          headers=self.headers).text
            except requests.exceptions.RequestException as e:  # This is the correct syntax
                self.logger.error("Unable to read from server")
                task_list = phrase.task_list

            parsed_list = self.task_parser.parse(task_list)
            if parsed_list is None:
                return Execute([]), UnableToParseTaskListMsg(task_list)

            return Execute(parsed_list), SuccessfullyParaphrasedMsg()

        elif isinstance(phrase, AlexaRecordCommand):
            try:
                task_list = requests.post(self.url + "/paraphrase_this",
                                          data=json.dumps({'data': phrase.task_list}),
                                          headers=self.headers).text
            except requests.exceptions.RequestException as e:  # This is the correct syntax
                self.logger.error("Unable to read from server")
                task_list = phrase.task_list

            parsed_list = self.task_parser.parse(task_list)
            if parsed_list is None:
                return Record([], phrase.command_id), UnableToParseTaskListMsg(task_list)

            return Record(parsed_list, phrase.command_id), SuccessfullyParaphrasedMsg()

        elif isinstance(phrase, AlexaReturnCommand):
            return ReturnCommand(phrase.command_id), SuccessfullyParaphrasedMsg()

        elif isinstance(phrase, AlexaDefinePose):
            return DefinePose(phrase.pose), SuccessfullyParaphrasedMsg()

        elif isinstance(phrase, AlexaSwitchRobot):
            return SwitchRobot(phrase.robot), SuccessfullyParaphrasedMsg()

        elif isinstance(phrase, AlexaPause):
            return Pause(), SuccessfullyParaphrasedMsg()

        elif isinstance(phrase, AlexaStop):
            return Stop(), SuccessfullyParaphrasedMsg()

        elif isinstance(phrase, AlexaQuit):
            return Quit(), SuccessfullyParaphrasedMsg()

        return phrase

    def add_task(self, task_name):
        #requests.post(self.url + "/add_task", data=json.dumps({'data': task_name}))
        pass

    def add_location(self, location_name):
        #requests.post(self.url + "/add_location", data=json.dumps({'data': location_name}))
        pass

    def add_pose(self, pose_name):
        #requests.post(self.url + "/add_pose", data=json.dumps({'data': pose_name}))
        pass

    def add_object(self, object_name):
        #requests.post(self.url + "/add_object", data=json.dumps({'data': object_name}))
        pass


class CommandParserClient:
    def __init__(self, command_passer, robot_interface, command_state, paraphrase_detector):
        self.robotInterface = robot_interface
        assert(isinstance(self.robotInterface, GenericInterface))

        self.commandPasser = command_passer
        self.commandState = command_state
        self.paraphraseDetector = paraphrase_detector
        self.logger = logging.getLogger(__name__)

    def interpretTask(self, task):
        self.logger.info("Intrepreting task {}".format(task))
        if isinstance(task, GoTo):
            self.logger.info("Handling task as GoTo")
            location = self.commandState.getLocation(task.location_name)
            result = self.robotInterface.moveTo(location)
            print(result)
            return result

        elif isinstance(task, MoveBy):
            self.logger.info("Handling task as MoveBy")
            result = self.robotInterface.cartesianTransform(task.direction, task.meters)
            print(result)
            return result

        elif isinstance(task, FollowMe):
            self.logger.info("Handling task as FollowMe")
            self.robotInterface.followMe()

        elif isinstance(task, GraspObject):
            self.logger.info("Handling task as GraspObject")
            self.robotInterface.graspObject(self.commandState.getObject(task.object_name))

        elif isinstance(task, PlaceObject):
            self.logger.info("Handling task as PlaceObject")
            self.robotInterface.placeObject(self.commandState.getObject(task.object_name))

        elif isinstance(task, AskForObject):
            self.logger.info("Handling task as AskForObject")
            self.robotInterface.askForObject()

        elif isinstance(task, EnactPose):
            self.logger.info("Handling task as EnactPose")
            self.robotInterface.enactPose(self.commandState.getPose(task.pose_name))

        elif isinstance(task, MoveHand):
            self.logger.info("Handling task as MoveHand")
            pass

        elif isinstance(task, RecordedTask):
            self.logger.info("Handling task as RecordedTask")
            task_list = self.commandState.getCommand(task.task_name)
            if isinstance(task_list, TaskNotFound):
                self.logger.error("Could not find command {}".format(task.task_name))
                return
            self.interpretTaskList(task_list)

    def interpretTaskList(self, task_list):
        for task in task_list:
            self.interpretTask(task)

        return SuccessfullyExecutedTaskListMsg()

    def interpretCommand(self):
        self.logger.info("Interpreting command")
        command = self.commandPasser.read()
        if command is None:
            self.logger.error("Was unable to read command from web server. Trying again...")
            self.commandPasser.removeMessage()
            return

        self.logger.info("Received command {}".format(command))
        command, success = self.paraphraseDetector.paraphraseCommand(command)
        self.logger.info("Paraphrased command {}".format(command))

        if isinstance(success, UnableToParseTaskListMsg):
            self.logger.error("Unable to parse command: {}".format(success))
            self.commandPasser.write(success)
            return

        if isinstance(command, Recognize):
            self.robotInterface.recognize_object()
            self.commandPasser.write(SucessfullyWroteObjectMsg(command.as_id))

        elif isinstance(command, SwitchRobot):
            self.commandPasser.write(NotYetImplementedMsg())

        elif isinstance(command, Execute):
            self.commandPasser.write(ExecutingTasks())
            result = self.interpretTaskList(command.task_list)
            print(result)

        elif isinstance(command, Record):
            self.commandState.addCommand(command.command_id, command.task_list)
            self.paraphraseDetector.add_task(command.command_id)
            self.commandPasser.write(RecordedCommandSuccessfullyMsg(command.command_id))

        elif isinstance(command, DefineLocation):
            currentLocation = self.robotInterface.getCurrentLocation()
            self.commandState.addLocation(command.location_name, currentLocation)
            self.paraphraseDetector.add_location(command.location_name)
            self.commandPasser.write(DefineLocationSuccessMsg(command.location_name))

        elif isinstance(command, ReturnCommand):
            task = self.commandState.getCommand(command.task_name)
            self.commandPasser.write(ReturnCommandMsg(command.task_name, task.pretty_print()))

        elif isinstance(command, AdjustBy):
            result = self.robotInterface.cartesianTransform(command.pose_task.direction, command.pose_task.meters)
            self.commandPasser.write(AdjustByResultMsg())

        elif isinstance(command, DefinePose):
            currentPose = self.robotInterface.getCurrentPose()
            self.commandState.addPose(command.pose_name, currentPose)
            self.paraphraseDetector.add_pose(command.pose_name)
            self.commandPasser.write(DefinePoseSuccessMsg(command.pose_name))

        elif isinstance(command, Pause):
            self.commandPasser.write(NotYetImplementedMsg())

        elif isinstance(command, Stop):
            self.commandPasser.write(NotYetImplementedMsg())

        elif isinstance(command, Quit):
            self.commandPasser.write(QuittingMsg())
            exit(0)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    command_passer = PassCommand("out.txt", "in.txt")
    robot_interface = GenericInterface()
    command_state = CommandState()
    paraphrase_detector = Paraphraser('http://long.cs.columbia.edu:80', task_parser)

    command_parser = CommandParserClient(command_passer=command_passer,
                                         robot_interface=robot_interface,
                                         command_state=command_state,
                                         paraphrase_detector=paraphrase_detector)

    while True:
        command_parser.interpretCommand()
        time.sleep(0.1)

