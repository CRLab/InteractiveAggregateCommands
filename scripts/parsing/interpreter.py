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
poses = {}

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
    return None, True

def stop_execution():
    return True

def pause_execution():
    return True

def define_pose(pose_name):
    if pose_name in poses:
        return "Unable to define pose. Pose " + pose_name + " is already defined"
    current_pose, success = get_current_pose()
    if not success:
        return "Unable to get current pose."
    poses[pose_name] = current_pose
    return "Successfully defined pose " + pose_name

def adjust_by(pose_task):
    result, success = robot_interface.adjust_by(pose_task.direction, pose_task.meters)
    if success:
        return "Successfully executed pose task"
    else:
        return "Failed to execute pose task"

def interpret_task(current_task):
    
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
        command = parse_command(s)
        process_command(command)


#!/usr/bin/env python
import logging
from random import randint

from flask import Flask, render_template
from flask_ask import Ask, request, session, question, statement
from flask_sslify import SSLify

import rospy
from std_msgs.msg import String

alexa_valid_phrases_topic = "AlexaValidPhrases"
alexa_detected_phrases_topic = "AlexaDetectedPhrases"
valid_phrases = []

app = Flask(__name__)
sslify = SSLify(app)
ask = Ask(app, "/")
logging.getLogger('flask_ask').setLevel(logging.DEBUG)

@ask.launch
def launch():
    help_text = render_template('help_text')
    return statement(help_text)

def assign_valid_phrases(phrases_str_msg):
    phrases_str = phrases_str_msg.data
    valid_phrases = phrases_str.lower().split(",")


@ask.intent('SendCommandIntent', mapping={'command':'Command'})
def send_instruction(command):
    print(command)
    if command in valid_phrases:
        publisher.publish(command)
        success_msg = render_template('success_msg', command=command)
        return statement(success_msg)
    else:
        invalid_command_msg = render_template('invalid_command_msg', command=command)
        return statement(invalid_command_msg)

@ask.intent('AMAZON.HelpIntent')
def help():
    help_text = render_template('help_text')
    return question(help_text).reprompt(help_text)


@ask.intent('AMAZON.CancelIntent')
def cancel():
    bye_text = render_template('bye')
    return statement(bye_text)


@ask.session_ended
def session_ended():
    return "", 200

@app.route('/')
def hello_world():
    return 'Hello, World!'

if __name__ == '__main__':
    rospy.init_node('graspit_alexa_controller')
    port = rospy.get_param("~port")
    url = rospy.get_param("~url")

    publisher = rospy.Publisher(alexa_detected_phrases_topic, String, queue_size=10)
    subscriber = rospy.Subscriber(alexa_valid_phrases_topic, String, assign_valid_phrases)

    app.run(debug=True, host=url, port=port)
