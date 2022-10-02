from state import BaseState
from context import Context
from geometry_msgs.msg import Twist

class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            #TODO: add outcomes
            add_outcomes=["failure", "working", "success"],
        )

    def evaluate(self, ud):
        DISTANCE_TOLERANCE = 0.2
        ANUGLAR_TOLERANCE = 0.2
        #TODO: get the tag's location and properties
        tag = self.context.env.get_fid_data()
        #TODO: if we don't have a tag: go to the Done State (with outcome 'failure')
        if tag == None:
            return "failure"
        #TODO: if we are within angular and distance tolerances: go to Done State (with outcome 'success')
        if tag.closenessMetric < DISTANCE_TOLERANCE and abs(tag.xTagCenterPixel) < ANUGLAR_TOLERANCE:
            return "success"

        #TODO: figure out the twist command to be applied to move rover to tag
        #angular 0,0,z
        #linear x,0,0
        # p loop?
        linear_vel = [0,0,0]
        if tag.closenessMetric >= DISTANCE_TOLERANCE:
            linear_vel = [1,0,0]
        if tag.xTagCenterPixel >= ANUGLAR_TOLERANCE:
            angular_vel = [0,0,tag.xTagCenterPixel]
        twist = Twist(linear_vel, angular_vel)

        #TODO: send Twist command to rover
        self.context.rover.send_drive_command(twist)

        #TODO: stay in the TagSeekState (with outcome 'working')
        return "working"