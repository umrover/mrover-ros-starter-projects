from multiprocessing import context
import rospy
from state import BaseState
import numpy as np
from context import Context
from drive import get_drive_command

class DriveState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            #TODO:
            add_outcomes=["driving_to_point", "reached_point"]
        )

    def evaluate(self, ud):
        target = np.array([4.5, 2.5, 0.0])
        #TODO: get the rovers pose, if it doesn't exist stay in DriveState with outcome "driving_to_point"
        pose = self.context.rover.get_pose()
        if pose == None:
            return "driving_to_point"
        #TODO: get the drive command (and completion status) based on target and pose (HINT: use get_drive_command())
        effort = get_drive_command(target, pose, 0.3, 0.5)
        #TODO: if we are finished getting to the target, return with outcome "reached_point"
        if effort[1] == True:
            return "reached_point"
        #TODO: send the drive command to the rover
        self.context.rover.send_drive_command(effort[0])
        #TODO: tell smach to stay in the DriveState by returning with outcome "driving_to_point"
        return "driving_to_point"
