# -*- coding: utf-8 -*-
# @Author: David Watkins
# @Date:   2017-04-20 03:46:44
# @Last Modified by:   David Watkins
# @Last Modified time: 2017-04-20 11:35:27

from ast import *
import parser

def interpret_task(current_task):
	if isinstance(current_Task, Recognize):
		pass
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

def handle_sequence(sequence):
	for task in sequence:
		interpret_task(sequence)

if __name__ == "main":
	while True:
	   try:
	       s = raw_input('aggregate > ')
	   except EOFError:
	       break
	   if not s: continue
	   result = parser.parser.parse(s)
	   print(result)