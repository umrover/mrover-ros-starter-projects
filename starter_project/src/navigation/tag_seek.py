from state import BaseState
from context import Context
from geometry_msgs.msg import Twist

import rospy

class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            #TODO: add outcomes
            add_outcomes=["failure", "working", "success"],
        )

    def evaluate(self, ud):
        DISTANCE_TOLERANCE = 0.99
        ANUGLAR_TOLERANCE = 0.3
        #TODO: get the tag's location and properties
        tag = self.context.env.get_fid_data()

        #TODO: if we don't have a tag: go to the Done State (with outcome 'failure')
        # changed to stay in Tag Seek State
        if tag is None:
            return "failure"
        rospy.logerr(f"found tag")

        #TODO: if we are within angular and distance tolerances: go to Done State (with outcome 'success')
        if tag.closenessMetric < DISTANCE_TOLERANCE and abs(tag.xTagCenterPixel) < ANUGLAR_TOLERANCE:
            return "success"

        #TODO: figure out the twist command to be applied to move rover to tag
        #angular 0,0,z
        #linear x,0,0
        twist = Twist()
        if tag.closenessMetric >= DISTANCE_TOLERANCE:
            twist.linear.x = 1
        if abs(tag.xTagCenterPixel) >= ANUGLAR_TOLERANCE:
            rospy.logerr(f"angular = {tag.xTagCenterPixel}")
            twist.angular.z = tag.xTagCenterPixel

        #TODO: send Twist command to rover
        self.context.rover.send_drive_command(twist)

        #TODO: stay in the TagSeekState (with outcome 'working')
        return "working"