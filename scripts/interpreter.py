# -*- coding: utf-8 -*-
# @Author: David Watkins
# @Date:   2017-04-20 03:46:44
# @Last Modified by:   David Watkins
# @Last Modified time: 2017-04-21 03:39:12

from ast import *
from command_parser import command_parser
from task_parser import task_parser
import fuzzy_parser

tasks = {}
objects = {}
locations = { "home": (0,0,0) }
robots = ["mico", "fetch"]

def get_current_object():
    pass

#Give me the current location of the end effector
def get_current_location():
    #subscriber to 'robot_info'
    #return msg(PoseStamped)
    pass

#Go to the specified location
def goto_location(location):
    #pass a posestamped into go_to function to move the robot

    pass

def get_current_pose():
    pass

def interpret_task(current_task):
    if isinstance(current_Task, Recognize):
        objects[current_task.object_name] = get_current_object()
    elif isinstance(current_task, Execute):
        pass
    elif isinstance(current_task, Record):
        pass
    elif isinstance(current_task, DefineLocation):
        pass
    elif isinstance(current_task, AdjustBy):
        pass
    elif isinstance(current_task, DefinePose):
        pass
    elif isinstance(current_task, Direction):
        pass
    elif isinstance(current_task, MoveBy):
        pass
    elif isinstance(current_task, GoTo):
        pass
    elif isinstance(current_task, FollowMe):
        pass
    elif isinstance(current_task, GraspObject):
        pass
    elif isinstance(current_task, PlaceObject):
        pass
    elif isinstance(current_task, AskForObject):
        pass
    elif isinstance(current_task, EnactPose):
        pass
    elif isinstance(current_task, MoveHand):
        pass
    elif isinstance(current_task, Task_list):
        pass

def parse_unprocessed(s):
    if isinstance(s, UnprocessedExecute):
        s.task_list = fuzzy_parser.parse(s.task_list)
        return s.pretty_print()

    elif isinstance(s, UnprocessedRecord):
        s.task_list = fuzzy_parser.parse(s.task_list)
        return s.pretty_print()

    else:
        return ""

def parse_command(s):
    initial_parse = command_parser.parse(s)
    if initial_parse is not None:
        initial_parse = parse_unprocessed(initial_parse)
        return task_parser.parse(initial_parse)
    else:
        return task_parser.parse(s)

if __name__ == "main":
    while True:

        try:
            s = raw_input('aggregate > ')
        except EOFError:
            break
        if not s: continue
        result = parser.parser.parse(s)
        print(result)
        if result is not None:
            interpret_task(result)