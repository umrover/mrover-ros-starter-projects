from multiprocessing import context
from state import BaseState
import numpy as np
from context import Context
from drive import get_drive_command
import rospy

class DriveState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            #TODO:
            add_outcomes=["driving_to_point", "reached_point"],
        )

    def evaluate(self, ud):
        target = np.array([3.0, 3.0, 0.0])
        #TODO: get the rovers pose, if it doesn't exist stay in DriveState with outcome "driving_to_point"
        SE3_pose = self.context.rover.get_pose()
        if SE3_pose == None:
            return "driving_to_point"
        #TODO: get the drive command (and completion status) based on target and pose (HINT: use get_drive_command())
        get_drive = get_drive_command(target, SE3_pose, 1.0, 0.2)
        #TODO: if we are finished getting to the target, return with outcome "reached_point"
        #rospy.logerr(f"completion_status = {get_drive[1]}")
        if get_drive[1]:
            self.context.rover.send_drive_stop()
            return "reached_point"
        #TODO: send the drive command to the rover
        else:
            self.context.rover.send_drive_command(get_drive[0])
            return "driving_to_point"
        #TODO: tell smach to stay in the DriveState by returning with outcome "driving_to_point"
