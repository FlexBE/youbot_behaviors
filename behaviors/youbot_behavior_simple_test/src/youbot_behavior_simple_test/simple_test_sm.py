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
from youbot_flexbe_states.execute_arm_trajectory_state import ExecuteTrajectoryState
from flexbe_navigation_states.move_base_state import MoveBaseState
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
		front_pose_traj = [[0.0, 1.1094, -4.0187, 1.789, 0.0]]
		traj_time = [10.0]
		target_pose = [1.0, 1.0, 45.0] # [m, m, degrees]
		origin_pose = [0.0, 0.0, 0.0]  # [m, m, degrees]
		# x:582 y:424, x:458 y:265
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

        # [/MANUAL_CREATE]


		with _state_machine:
			# x:129 y:68
			OperatableStateMachine.add('Announce_Execution_Start',
										LogState(text="Execution is starting!", severity=Logger.REPORT_INFO),
										transitions={'done': 'Move_Forward_and_Turn'},
										autonomy={'done': Autonomy.Low})

			# x:137 y:421
			OperatableStateMachine.add('Move_Arm_Forward',
										ExecuteTrajectoryState(target_pose=front_pose_traj, time=traj_time),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low})

			# x:131 y:189
			OperatableStateMachine.add('Move_Forward_and_Turn',
										MoveBaseState(target_pose=target_pose),
										transitions={'arrived': 'Move_to_Origin', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Low, 'failed': Autonomy.Low})

			# x:143 y:298
			OperatableStateMachine.add('Move_to_Origin',
										MoveBaseState(target_pose=origin_pose),
										transitions={'arrived': 'Move_Arm_Forward', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Low, 'failed': Autonomy.Low})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

    # [/MANUAL_FUNC]
