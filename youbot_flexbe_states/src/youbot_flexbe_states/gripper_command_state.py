#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from control_msgs.msg import *
from trajectory_msgs.msg import *

"""
Created on 11/27/2015

@author: Spyros Maniatopoulos
"""

class GripperCommandState(EventState):
    """
    Executes a custom trajectory.

    -- gripper_cmd      int     Identifier of the pre-defined commands.
                                The state's class variables can be used here.
    -- time             float   Relative time in seconds from starting
                                the execution to when the corresponding
                                target gripper state should be reached.

    <= done                     Gripper command was successfully executed.
    <= failed                   Failed to send or execute gripper command.
    """

    CLOSE_GRIPPER = 0
    OPEN_GRIPPER  = 1

    def __init__(self, gripper_cmd, time = 5.0):
        """Constructor"""

        super(GripperCommandState, self).__init__(outcomes = ['done', 'failed'])

        self._joint_names = ['gripper_finger_joint_l',
                             'gripper_finger_joint_r']

        self._gripper_joint_positions = {
            #  gripper_finger_joint_l, gripper_finger_joint_r
            0: [0.001, 0.001],
            1: [0.010, 0.010] #NOTE: 0.011 results in goal tolerance violation
        }

        self._gripper_cmd = gripper_cmd
        self._time = time

        self._action_topic = "/arm_1/gripper_controller/follow_joint_trajectory"

        self._client = ProxyActionClient({self._action_topic: FollowJointTrajectoryAction})

        self._done = False
        self._failed = False


    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""

        if self._done:
            return 'done'
        if self._failed:
            return 'failed'

        if self._client.has_result(self._action_topic):
            result = self._client.get_result(self._action_topic)
            if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
                self._done = True
                return 'done'
            elif result.error_code == FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
                Logger.logwarn('Probably done, but goal tolerances violated (%d)' % result.error_code)
                self._done = True
                return 'done'
            else:
                Logger.logwarn('Gripper trajectory failed to execute: (%d) %s' % (result.error_code, result.error_string))
                self._failed = True
                return 'failed'


    def on_enter(self, userdata):
        """Create and send action goal"""

        self._done = False
        self._failed = False

        # Create and populate action goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self._joint_names

        target_joint_positions = self._gripper_joint_positions[self._gripper_cmd]

        point = JointTrajectoryPoint()
        point.positions = target_joint_positions
        point.time_from_start = rospy.Duration.from_sec(self._time)
        goal.trajectory.points.append(point)

        # Send the action goal for execution
        try:
            self._client.send_goal(self._action_topic, goal)
        except Exception as e:
            Logger.logwarn("Unable to send follow joint trajectory action goal:\n%s" % str(e))
            self._failed = True


    def on_exit(self, userdata):
        if not self._client.has_result(self._action_topic):
            self._client.cancel(self._action_topic)
            Logger.loginfo('Cancelled active action goal.')
