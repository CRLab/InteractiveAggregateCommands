'''
A parser implementation for the humanoid aggregate commands pipeline
It is designed with four macro actions:
    - Commands : asynchronous execution steps given to the commander
    - Pose Tasks : Moving the robot in a particular direction
    - Directions : Six directions to move an end effector
    - Tasks : Task primitives to be executed by a pipeline

These are parsed using a lexer/parser paradigm similar to a compiler to remove ambiguity and generate
more complex actions
'''

import ply.yacc as yacc
from lexer import Lexer
from ast import *

tokens = Lexer.tokens
############################## COMMANDS ##############################
#command : 
    #RECOGNIZE AS ID
    #EXECUTE <task list>
    #RECORD <task list> as ID
    #YOU ARE IN LOCATION ID
    #ADJUST COURSE BY <pose_task>
    #YOU ARE IN POSE ID

def p_command_recognize(p):
    'command : RECOGNIZE OBJECT AS ID'
    p[0] = Recognize(p[4])

def p_command_execute(p):
    'command : EXECUTE task_list'
    p[0] = Execute(p[2])

def p_command_record(p):
    'command : RECORD task_list AS ID'
    p[0] = Record(p[2], p[4])

def p_command_define_location(p):
    'command : YOU ARE IN LOCATION ID'
    p[0] = DefineLocation(p[4])

def p_command_adjust_course(p):
    'command : ADJUST COURSE BY pose_task'
    p[0] = AdjustBy(p[4])

def p_command_define_pose(p):
    'command : YOU ARE IN POSE ID'
    p[0] = DefinePose(p[4])

############################## DIRECTIONS ##############################
#direction :
    #UP
    #DOWN
    #LEFT
    #RIGHT
    #FORWARD
    #BACKWARD
def p_direction(p):
    'direction : UP'
    '          | DOWN'
    '          | LEFT'
    '          | RIGHT'
    '          | BACKWARD'
    '          | FORWARD'
    p[0] = Direction(p[1])


############################## POSE TASKS ##############################
#pose_task :
    #MOVING direction BY NUMBER METERS
    #MOVE direction BY NUMBER METERS
def p_pose_move_by(p):
    'pose_task : MOVING direction BY NUMBER METERS'
    '          | MOVE direction BY NUMBER METERS'
    p[0] = MoveBy(p[2], p[4])

############################## TASKS ##############################


#task :
    #pose_task
    #GO TO ID
    #FOLLOW ME
    #FOLLOW ME FOR NUMBER SECONDS
    #GRASP OBJECT ID
    #PLACE OBJECT ID
    #ASK FOR OBJECT ID AS ID
    #ENACT POSE ID
    #MOVE HAND TO ID

#task_list :
    #[]
    #task_list AND THEN task

def p_task_pose_task(p):
    'task : pose_task'
    p[0] = p[1]

def p_task_goto(p):
    'task : GO TO ID'
    p[0] = GoTo(p[3])

def p_task_follow_me(p):
    'task : FOLLOW ME'
    p[0] = FollowMe()

def p_task_follow_me_for_seconds(p):
    'task : FOLLOW ME FOR NUMBER SECONDS'
    p[0] = FollowMe(p[3])

def p_task_grasp_object(p):
    'task : GRASP OBJECT ID'
    p[0] = GraspObject(p[3])

def p_task_place_object(p):
    'task : PLACE OBJECT ID'
    p[0] = PlaceObject(p[3])

def p_task_ask_for_object(p):
    'task : ASK FOR OBJECT ID AS ID'
    p[0] = AskForObject(p[4], p[6])

def p_task_enact_pose_id(p):
    'task : ENACT POSE ID'
    p[0] = EnactPose(p[3])

def p_task_move_hand_to_id(p):
    'task : MOVE HAND TO ID'
    p[0] = MoveHand(p[4])

def p_task_list(p):
    'task_list : task'
    p[0] = TaskList(p[1])

def p_task_list2(p):
    'task_list : task_list AND THEN task'
    p[0] = p[1] + p[4]

def p_empty(p):
    'empty :'
    pass

# Error rule for syntax errors
def p_error(p):
    print("Syntax error in input!")

def p_top_level(p):
    'start : command'
    p[0] = p[1]

# Build the parser
parser = yacc.yacc(start='start')

while True:
   try:
       s = input('aggregate > ')
   except EOFError:
       break
   if not s: continue
   result = parser.parse(s)
   print(result)