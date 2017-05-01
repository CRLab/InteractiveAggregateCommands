#!/usr/bin/env python

import rospy
import pickle
import ast
from rospy import std_msgs
import scripts.interfaces as interfaces

from scripts.types.ast import Recognize, SwitchRobot, Execute, DefineLocation, AdjustBy, DefinePose, Pause, Stop, Quit
from scripts.types.messages import SucessfullyWroteObject


class PassCommand:
    def __init__(self, commandsInFilename, infoOutFilename):
        self.commandsInFilename = commandsInFilename
        self.infoOutFilename = infoOutFilename

    def read(self):
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


class CommandParserClient:
    def __init__(self, commandPasser, robotInterface, commandState):
        self.robotInterface = robotInterface
        self.commandPasser = commandPasser
        self.commandState = commandState

    def interpretTask(self, task):
        pass

    def interpretCommand(self):
        command = self.commandPasser.read()

        if isinstance(command, Recognize):
            self.robotInterface.recognize_object()
            self.commandPasser.write(SucessfullyWroteObject())

        elif isinstance(command, SwitchRobot):
            pass

        elif isinstance(command, Execute):
            pass

        elif isinstance(command, DefineLocation):
            pass

        elif isinstance(command, AdjustBy):
            pass

        elif isinstance(command, DefinePose):
            pass

        elif isinstance(command, Pause):
            pass

        elif isinstance(command, Stop):
            pass

        elif isinstance(command, Quit):
            pass


        def process_command(s):
            if isinstance(current_Task, Recognize):
                objects[current_task.object_name] = get_current_object()

            elif isinstance(current_task, SwitchRobot):
                pass

            elif isinstance(current_task, Execute):
                pass

            elif isinstance(current_task, Record):
                print("Recording command " + current_task.command_id)


            elif isinstance(current_task, DefineLocation):
                print("Defining location " + current_task.location_name)
                result = define_location(current_task.location_name)
                print(result)

            elif isinstance(current_task, AdjustBy):
                print("Adjusting course by " + current_task.pretty_print())
                result = adjust_by(current_task.pose_task)
                print(result)

            elif isinstance(current_task, DefinePose):
                print("Defining pose " + current_task.pose_name)
                result = define_pose(current_task.pose_name)
                print(result)

            elif isinstance(current_task, Pause):
                print("Pausing execution")
                success = pause_execution()
                if success:
                    print("Paused execution successfully")
                else:
                    print("Failed to pause execution")

            elif isinstance(current_task, Stop):
                print("Stopping execution")
                success = stop_execution()
                if success:
                    print("Stopped execution successfully")
                else:
                    print("Failed to stop execution")

            elif isinstance(current_task, Quit):
                print("Quitting...")
                exit(0)
