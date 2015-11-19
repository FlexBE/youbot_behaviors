#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from tf import transformations

"""
Created on 11/19/2015

@author: Spyros Maniatopoulos
"""

class MoveBaseState(EventState):
    """
    Navigates a robot to a desired position and orientation using move_base.

    -- target_pose     float[]     Pose to navigate to [x,y,theta] in [m,m,deg]

    <= arrived                  Navigation to target pose succeeded.
    <= failed                   Navigation to target pose failed.
    """

    def __init__(self, target_pose):
        """Constructor"""

        super(MoveBaseState, self).__init__(outcomes = ['arrived', 'failed'])


        self._target_pose = target_pose

        self._action_topic = "/move_base"

        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})

        self._arrived = False
        self._failed = False


    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""

        if self._arrived:
            return 'arrived'
        if self._failed:
            return 'failed'

        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                self._arrived = True
                return 'arrived'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('Navigation failed: %s' % str(status))
                self._failed = True
                return 'failed'


    def on_enter(self, userdata):
        """Create and send action goal"""

        self._arrived = False
        self._failed = False

        # Create and populate action goal
        goal = MoveBaseGoal()

        pt = Point(x = self._target_pose[0], y = self._target_pose[1])
        qt = transformations.quaternion_from_euler(0, 0, self._target_pose[2])

        goal.target_pose.pose = Pose(position = pt,
                                     orientation = Quaternion(*qt))

        goal.target_pose.header.frame_id = "odom"
        # goal.target_pose.header.stamp.secs = 5.0

        # Send the action goal for execution
        try:
            self._client.send_goal(self._action_topic, goal)
        except Exception as e:
            Logger.logwarn("Unable to send navigation action goal:\n%s" % str(e))
            self._failed = True
