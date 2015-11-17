#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('youbot_behavior_simple_test')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 17 2015
@author: Spyros Maniatopoulos
'''
class SimpleTestSM(Behavior):
	'''
	Simple behavior for the purposes of testing FlexBE - youBot integration.
	'''


	def __init__(self):
		super(SimpleTestSM, self).__init__()
		self.name = 'Simple Test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:503 y:80, x:130 y:452
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:112 y:68
			OperatableStateMachine.add('Announce_Execution_Start',
										LogState(text="Execution is starting!", severity=Logger.REPORT_INFO),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Low})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
