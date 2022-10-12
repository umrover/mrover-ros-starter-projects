from math import dist
import rospy
from state import BaseState
from context import Context
from geometry_msgs.msg import Twist

ANGULAR_ERROR = 20

class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            #TODO: add outcomes
            add_outcomes=["working", "failure", "success"],
        )

    def evaluate(self, ud):
        #TODO: get the tag's location and properties
        tag = self.context.env.get_fid_data()
        #TODO: if we don't have a tag: go to the Done State (with outcome 'failure')
        if (tag.dist == 0.0):
            return "failure"
        #TODO: if we are within angular and distance tolerances: go to Done State (with outcome 'success')
        elif (abs(tag.x) <= ANGULAR_ERROR) & (abs(tag.dist) >= 0.004):
            return "success"
        #TODO: figure out the twist command to be applied to move rover to tag
        move_cmd = Twist()
        if tag.x > ANGULAR_ERROR:
            move_cmd.linear.x = 0
            move_cmd.linear.y = 0
            move_cmd.angular.z = -1
        elif tag.x < ANGULAR_ERROR * (-1):
            move_cmd.linear.x = 0
            move_cmd.linear.y = 0
            move_cmd.angular.z = 1
        else:
            move_cmd.linear.x = 1
            move_cmd.linear.y = 1
            move_cmd.angular.z = 0
            

        #TODO: send Twist command to rover
        self.context.rover.send_drive_command(move_cmd)

        #TODO: stay in the TagSeekState (with outcome 'working')
        return "working"