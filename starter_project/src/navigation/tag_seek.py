from state import BaseState
from context import Context
from geometry_msgs.msg import Twist

class TagSeekState(BaseState):
    ANGULAR_TOLERANCE = 0.3
    DISTANCE_TOLERANCE = 0.99

    def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=['failure', 'working', 'success'],
        )

    def evaluate(self, ud):
        # get the tag's location and properties
        tag = self.context.env.get_fid_data()
        
        # if we don't have a tag: go to the Done State (with outcome 'failure')
        if tag is None:
            return 'failure' 

        # if we are within angular and distance tolerances: go to Done State (with outcome 'success')
        if tag.clossnessMetric < self.DISTANCE_TOLERANCE and abs(tag.xTagCenterPixel) < self.ANGULAR_TOLERANCE:
            return 'success'

        # figure out the twist command to be applied to move rover to tag
        linear_vel = [0, 0, 0]
        if tag.clossnessMetric >= self.DISTANCE_TOLERANCE:
            linear_vel = [1, 0, 0]
        
        angular_vel = [0, 0, 0]
        if abs(tag.xTagCenterPixel) >= self.ANGULAR_TOLERANCE:
            angular_vel = [0, 0, tag.xTagCenterPixel]
        
        twist = Twist(linear_vel, angular_vel)

        # send Twist command to rover
        self.context.rover.send_drive_command(twist)

        # stay in the TagSeekState (with outcome 'working')
        return 'working'
