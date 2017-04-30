#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.log_state import LogState
from youbot_flexbe_states.execute_arm_trajectory_state import ExecuteTrajectoryState
from flexbe_navigation_states.move_base_state import MoveBaseState
from youbot_flexbe_states.gripper_command_state import GripperCommandState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import Pose2D
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
		target_pose = Pose2D(1.0, 1.0, 45.0) # [m, m, degrees]
		origin_pose = Pose2D(0.0, 0.0, 0.0)  # [m, m, degrees]
		# x:950 y:88, x:441 y:221
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.target_pose = target_pose
		_state_machine.userdata.origin_pose = origin_pose

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

        # [/MANUAL_CREATE]


		with _state_machine:
			# x:138 y:67
			OperatableStateMachine.add('Announce_Execution_Start',
										LogState(text="Execution is starting!", severity=Logger.REPORT_INFO),
										transitions={'done': 'Open_Gripper'},
										autonomy={'done': Autonomy.Low})

			# x:637 y:79
			OperatableStateMachine.add('Move_Arm_Forward',
										ExecuteTrajectoryState(target_pose=front_pose_traj, time=traj_time),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low})

			# x:140 y:415
			OperatableStateMachine.add('Move_Forward_and_Turn',
										MoveBaseState(),
										transitions={'arrived': 'Close_Gripper', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'waypoint': 'target_pose'})

			# x:651 y:415
			OperatableStateMachine.add('Move_to_Origin',
										MoveBaseState(),
										transitions={'arrived': 'Move_Arm_Forward', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'waypoint': 'origin_pose'})

			# x:147 y:213
			OperatableStateMachine.add('Open_Gripper',
										GripperCommandState(gripper_cmd=GripperCommandState.OPEN_GRIPPER, time=5.0),
										transitions={'done': 'Move_Forward_and_Turn', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low})

			# x:435 y:415
			OperatableStateMachine.add('Close_Gripper',
										GripperCommandState(gripper_cmd=GripperCommandState.CLOSE_GRIPPER, time=5.0),
										transitions={'done': 'Move_to_Origin', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

    # [/MANUAL_FUNC]
