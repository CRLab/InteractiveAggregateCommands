"""
A parser implementation for the humanoid aggregate commands pipeline
It is designed with four macro actions:
    - Commands : asynchronous execution steps given to the commander
    - Pose Tasks : Moving the robot in a particular direction
    - Directions : Six directions to move an end effector
    - Tasks : Task primitives to be executed by a pipeline

These are parsed using a lexer/parser paradigm similar to a compiler to remove ambiguity and generate
more complex actions
"""
import ply.lex as lex
import ply.yacc as yacc
from ast import *

from scripts.types.ast import Quit, SwitchRobot, Stop, Pause, DefinePose, AdjustBy, DefineLocation, Record, Execute, \
    Recognize, Direction, MoveBy, GoTo, FollowMe, GraspObject, PlaceObject, AskForObject, EnactPose, MoveHand, \
    RecordedTask, TaskList


class Lexer(object):
    # List of token names.   This is always required
    tokens = (
        'GO',
        'TO',
        'FOLLOW',
        'ME',
        'FOR',
        'SECONDS',
        'GRASP',
        'OBJECT',
        'ASK',
        'AS',
        'ENACT',
        'POSE',
        'MOVE',
        'HAND',
        # 'REQUEST',
        # 'GRASPING',
        # 'MOVEMENT',
        # 'PRIMITIVE',
        'RECOGNIZE',
        'EXECUTE',
        'RECORD',
        'YOU',
        'ARE',
        'IN',
        'ADJUST',
        'COURSE',
        'BY',
        'AND',
        'THEN',
        # 'THIS',
        'BACKWARD',
        'FORWARD',
        'UP',
        'MOVING',
        'LEFT',
        'DOWN',
        'RIGHT',
        'ID',
        'LOCATION',
        'NUMBER',
        'PLACE',
        'METERS',
        'PAUSE',
        'STOP',
        'SWITCH',
        'ROBOT',
        'QUIT',
        'NAMED',
    )

    # Definitions of tokens go in functions to increase precidence
    def t_GO(self, t):
        r"""go"""
        return t

    def t_TO(self, t):
        r"""to"""
        return t

    def t_FOLLOW(self, t):
        r"""follow"""
        return t

    def t_METERS(self, t):
        r"""meters"""
        return t

    def t_ME(self, t):
        r"""me"""
        return t

    def t_FOR(self, t):
        r"""for"""
        return t

    def t_SECONDS(self, t):
        r"""seconds"""
        return t

    def t_GRASP(self, t):
        r"""grasp"""
        return t

    def t_OBJECT(self, t):
        r"""object"""
        return t

    def t_ASK(self, t):
        r"""ask"""
        return t

    def t_AS(self, t):
        r"""as"""
        return t

    def t_ENACT(self, t):
        r"""enact"""
        return t

    def t_POSE(self, t):
        r"""pose"""
        return t

    def t_MOVE(self, t):
        r"""move"""
        return t

    def t_HAND(self, t):
        r"""hand"""
        return t

    def t_RECOGNIZE(self, t):
        r"""recognize"""
        return t

    def t_EXECUTE(self, t):
        r"""execute"""
        return t

    def t_RECORD(self, t):
        r"""record"""
        return t

    def t_YOU(self, t):
        r"""you"""
        return t

    def t_ARE(self, t):
        r"""are"""
        return t

    def t_IN(self, t):
        r"""in"""
        return t

    def t_ADJUST(self, t):
        r"""adjust"""
        return t

    def t_COURSE(self, t):
        r"""course"""
        return t

    def t_BY(self, t):
        r"""by"""
        return t

    def t_AND(self, t):
        r"""and"""
        return t

    def t_THEN(self, t):
        r"""then"""
        return t

    def t_BACKWARD(self, t):
        r"""backward"""
        return t

    def t_FORWARD(self, t):
        r"""forward"""
        return t

    def t_UP(self, t):
        r"""up"""
        return t

    def t_MOVING(self, t):
        r"""moving"""
        return t

    def t_LEFT(self, t):
        r"""left"""
        return t

    def t_DOWN(self, t):
        r"""down"""
        return t

    def t_RIGHT(self, t):
        r"""right"""
        return t

    def t_LOCATION(self, t):
        r"""location"""
        return t

    def t_PLACE(self, t):
        r"""place"""
        return t

    def t_STOP(self, t):
        r"""stop"""
        return t

    def t_PAUSE(self, t):
        r"""pause"""
        return t

    def t_SWITCH(self, t):
        r"""switch"""
        return t

    def t_ROBOT(self, t):
        r"""robot"""
        return t

    def t_QUIT(self, t):
        r"""quit"""
        return t

    def t_NAMED(self, t):
        r"""named"""
        return t

    t_ID = r'[a-zA-Z][a-zA-Z\s]*[a-zA-Z]'

    # A regular expression rule with some action code
    # Note addition of self parameter since we're in a class
    def t_NUMBER(self, t):
        r"""\d+"""
        t.value = int(t.value)
        return t

    # Define a rule so we can track line numbers
    def t_newline(self, t):
        r"""\n+"""
        t.lexer.lineno += len(t.value)

    # A string containing ignored characters (spaces and tabs)
    t_ignore = ' \t'

    # Error handling rule
    def t_error(self, t):
        print("Illegal character '%s'" % t.value[0])
        t.lexer.skip(1)

    # Build the lexer
    def build(self, **kwargs):
        self.lexer = lex.lex(module=self, **kwargs)

    # Test it output
    def test(self, data):
        self.lexer.input(data)
        while True:
            tok = self.lexer.token()
            if not tok:
                break
            print(tok)


tokens = Lexer.tokens


############################## DIRECTIONS ##############################
# direction :
# UP
# DOWN
# LEFT
# RIGHT
# FORWARD
# BACKWARD
def p_direction(p):
    """direction : UP
                 | DOWN
                 | LEFT
                 | RIGHT
                 | BACKWARD
                 | FORWARD"""
    p[0] = Direction(p[1])


############################## POSE TASKS ##############################
# pose_task :
# MOVING direction BY NUMBER METERS
# MOVE direction BY NUMBER METERS
def p_pose_move_by(p):
    """pose_task : MOVING direction BY NUMBER METERS
                 | MOVE direction BY NUMBER METERS"""
    p[0] = MoveBy(p[2], p[4])


############################## TASKS ##############################


# task :
# pose_task
# GO TO ID
# FOLLOW ME
# FOLLOW ME FOR NUMBER SECONDS
# GRASP OBJECT ID
# PLACE OBJECT ID
# ASK FOR OBJECT NAMED ID
# ENACT POSE ID
# MOVE HAND TO ID

# task_list :
# []
# task_list AND THEN task

def p_task_pose_task(p):
    """task : pose_task"""
    p[0] = p[1]


def p_task_goto(p):
    """task : GO TO ID"""
    p[0] = GoTo(p[3])


def p_task_follow_me(p):
    """task : FOLLOW ME"""
    p[0] = FollowMe()


def p_task_follow_me_for_seconds(p):
    """task : FOLLOW ME FOR NUMBER SECONDS"""
    p[0] = FollowMe(p[3])


def p_task_grasp_object(p):
    """task : GRASP OBJECT ID"""
    p[0] = GraspObject(p[3])


def p_task_place_object(p):
    """task : PLACE OBJECT ID"""
    p[0] = PlaceObject(p[3])


def p_task_grasp_object2(p):
    """task : GRASP ID"""
    p[0] = GraspObject(p[2])


def p_task_place_object2(p):
    """task : PLACE ID"""
    p[0] = PlaceObject(p[2])


def p_task_ask_for_object(p):
    """task : ASK FOR OBJECT NAMED ID"""
    p[0] = AskForObject(p[5])


def p_task_enact_pose_id(p):
    """task : ENACT POSE ID"""
    p[0] = EnactPose(p[3])


def p_task_move_hand_to_id(p):
    """task : MOVE HAND TO ID"""
    p[0] = MoveHand(p[4])


def p_task_recorded_task(p):
    """task : ID"""
    p[0] = RecordedTask(p[1])


def p_task_list(p):
    """task_list : task"""
    p[0] = TaskList(p[1])


def p_task_list2(p):
    """task_list : task_list AND THEN task"""
    p[0] = p[1] + p[4]


# Error rule for syntax errors
def p_error(p):
    print("Syntax error in input!")


def p_top_level(p):
    """start : task_list"""
    p[0] = p[1]


# Build the parser
lexer_instance = Lexer()
lexer_instance.build()  # Build the lexer
command_parser = yacc.yacc(start='start', lexer=lexer_instance.lexer)
