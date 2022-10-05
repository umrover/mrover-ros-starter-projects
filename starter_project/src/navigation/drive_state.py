from multiprocessing import context
from state import BaseState
import numpy as np
from context import Context
from drive import get_drive_command

import rospy

class DriveState(BaseState):
    logger: rospy.Publisher

    def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=['driving_to_point', 'reached_point'],
        )

    def evaluate(self, ud):
        target = np.array([5.0, 5.0, 0.0])
        # get the rovers pose, if it doesn't exist stay in DriveState with outcome "driving_to_point"
        pose = self.context.rover.get_pose()
        
        rospy.logdebug(pose)

        if pose is None:
            return 'driving_to_point'

        # get the drive command (and completion status) based on target and pose (HINT: use get_drive_command())
        twist, completion_status = get_drive_command(target, pose, 0.5, 0.2)

        rospy.logdebug('completion_status %s' % completion_status)

        if completion_status:
            self.context.rover.send_drive_stop()

            # if we are finished getting to the target, return with outcome "reached_point"
            return 'reached_point'
        else:
            # send the drive command to the rover
            self.context.rover.send_drive_command(twist)

            # tell smach to stay in the DriveState by returning with outcome "driving_to_point"
            return 'driving_to_point'
