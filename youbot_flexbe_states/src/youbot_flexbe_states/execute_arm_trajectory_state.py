#!/usr/bin/env python

import rospy
import math
import actionlib

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from control_msgs.msg import *
from trajectory_msgs.msg import *

"""
Created on 11/17/2015

@author: Spyros Maniatopoulos
"""

class ExecuteTrajectoryState(EventState):
    """
    Executes a custom trajectory.

    target_pose     float[][]   Trajectory to be executed, given as a
                                list of time steps where each step
                                contains a list of target joint values.
    time            float[]     Relative time in seconds from starting
                                the execution when the corresponding
                                time step should be reached.

    <= done                     Trajectory was successfully executed.
    <= failed                   Failed to send or execute trajectory.
    """

    def __init__(self, target_pose, time):
        """Constructor"""

        super(ExecuteTrajectoryState, self).__init__(outcomes = ['done', 'failed'])

        self._joint_names = ['arm_joint_1', 'arm_joint_2',
                             'arm_joint_3', 'arm_joint_4',
                             'arm_joint_5']
        self._target_pose = target_pose
        self._time = time

        self._action_topic = "/arm_1/arm_controller/follow_joint_trajectory"

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
                Logger.logwarn('Joint trajectory failed to execute: (%d) %s' % (result.error_code, result.error_string))
                self._failed = True
                return 'failed'


    def on_enter(self, userdata):
        """Create and send action goal"""

        self._done = False
        self._failed = False

        # Create and populate action goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self._joint_names

        for i in range(len(self._target_pose)):
            point = JointTrajectoryPoint()
            point.positions = self._target_pose[i]
            point.time_from_start = rospy.Duration.from_sec(self._time[i])
            goal.trajectory.points.append(point)

        print "Trajectory", goal.trajectory

        # Send the action goal for execution
        try:
            self._client.send_goal(self._action_topic, goal)
        except Exception as e:
            Logger.logwarn("Unable to send follow joint trajectory action goal:\n%s" % str(e))
            self._failed = True
