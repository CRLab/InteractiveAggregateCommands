# Yacc example

import ply.yacc as yacc
from lexer import tokens
from ast import *

def p_command_recognize(p):
    '''command : RECOGNIZE OBJECT AS ID'''
    pass

#command : 
    #RECOGNIZE AS ID
    #EXECUTE <task list>
    #RECORD <task list> as ID
    #YOU ARE IN LOCATION ID
    #ADJUST COURSE BY <pose_task>
    #YOU ARE IN POSE ID

#pose_task :
    #MOVING direction BY NUMBER METERS
    #MOVE direction BY NUMBER METERS

    #MOVE IN DIRECTION direction FOR NUMBER METERS

#direction :
    #UP
    #DOWN
    #LEFT
    #RIGHT
    #FORWARD
    #BACKWARD

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

def p_task_goto(p):
    'task : GO TO ID'
    p[0] = GoTo(p[3])

# Error rule for syntax errors
def p_error(p):
    print("Syntax error in input!")

# Build the parser
parser = yacc.yacc()

while True:
   try:
       s = raw_input('aggregate > ')
   except EOFError:
       break
   if not s: continue
   result = parser.parse(s)
   print(result)