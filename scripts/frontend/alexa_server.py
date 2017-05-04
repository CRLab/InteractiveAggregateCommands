from flask import Flask, render_template
from flask_ask import Ask, statement
from scripts.types.ast import Direction
from scripts.types.alexa_phrases import AlexaAdjustCourse, AlexaRecognizeObject, AlexaQuit, AlexaStop, AlexaPause, \
    AlexaSwitchRobot, AlexaDefinePose, AlexaReturnCommand, AlexaRecordCommand, AlexaExecute
import pickle
import os
import time

app = Flask(__name__)
ask = Ask(app, '/')


def transfer_to_ros(msg):
    pickle.dump(msg, open("out.txt", "wb"))


def read_from_ros():
    time_waited = 0
    while not os.path.isfile("in.txt") and time_waited < 5:
        time.sleep(0.1)
        time_waited += 0.1

    if time_waited >= 5:
        return "Unable to communicate with command parser"

    msg = str(pickle.load(open("in.txt", "rb")))
    os.remove("in.txt")
    return msg


@ask.intent('RecognizeObjectIntent', mapping={'ObjectName': 'object_name'})
def recognize_object(object_name):
    msg = AlexaRecognizeObject(object_name)
    transfer_to_ros(msg)
    response = read_from_ros()

    return statement(response)


@ask.intent('ExecuteIntent', mapping={'TaskList': 'task_list'})
def execute(task_list):
    msg = AlexaExecute(task_list)
    transfer_to_ros(msg)
    response = read_from_ros()

    return statement(response)


@ask.intent('RecordCommandIntent', mapping={'TaskList': 'task_list', 'CommandId': 'command_id'})
def record_command(task_list, command_id):
    msg = AlexaRecordCommand(task_list, command_id)
    transfer_to_ros(msg)
    response = read_from_ros()

    return statement(response)


@ask.intent('ReturnCommandIntent', mapping={'CommandId': 'command_id'})
def return_command(command_id):
    msg = AlexaReturnCommand(command_id)
    transfer_to_ros(msg)
    response = read_from_ros()

    return statement(response)


@ask.intent('AdjustCourseIntent', mapping={'Direction': 'direction', 'Integer': 'integer', 'Decimal': 'decimal'})
def adjust_course(direction, integer, decimal):
    meters = float(integer + "." + decimal)
    direction = Direction(direction)

    msg = AlexaAdjustCourse(direction, meters)
    transfer_to_ros(msg)
    response = read_from_ros()

    return statement(response)


@ask.intent('DefinePoseIntent', mapping={'Pose': 'pose'})
def define_pose(pose):
    msg = AlexaDefinePose(pose)
    transfer_to_ros(msg)
    response = read_from_ros()

    return statement(response)


@ask.intent('SwitchRobotIntent', mapping={'Robot': 'robot'})
def switch_robot(robot):
    msg = AlexaSwitchRobot(robot)
    transfer_to_ros(msg)
    response = read_from_ros()

    return statement(response)


@ask.intent('PauseIntent')
def pause():
    msg = AlexaPause()
    transfer_to_ros(msg)
    response = read_from_ros()

    return statement(response)


@ask.intent('StopIntent')
def stop():
    msg = AlexaStop()
    transfer_to_ros(msg)
    response = read_from_ros()

    return statement(response)


@ask.intent('QuitIntent')
def alexa_quit():
    msg = AlexaQuit()
    transfer_to_ros(msg)
    response = read_from_ros()

    return statement(response)


@ask.intent('AMAZON.HelpIntent')
def help():
    help_text = render_template('help_text')
    return statement(help_text)


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
    import sys
    print(sys.path)
    app.run(debug=True, host="0.0.0.0", port=8080)
