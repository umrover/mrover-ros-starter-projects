from state import BaseState
from context import Context
from geometry_msgs.msg import Twist

class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            #TODO: add outcomes
            add_outcomes=["failure", "success", "working"],
        )

    def evaluate(self, ud):
        #TODO: get the tag's location and properties
        tag = self.context.env.get_fid_data()
        
        #TODO: if we don't have a tag: go to the Done State (with outcome 'failure')
        if(tag == None):
            return "failure"

        #TODO: if we are within angular and distance tolerances: go to Done State (with outcome 'success')
        if(tag.dist < 0.9 and abs(tag.x) < 0.2):
            return "success" 
        #TODO: figure out the twist command to be applied to move rover to tag
        linear_vel = [0,0,0]
        if(tag.dist >= 0.9):
            linear_vel = [1,0,0]
        angular_vel = [0,0,0]
        if(tag.x >= 0.2):
            angular_vel = [0,0,tag.x]


        #TODO: send Twist command to rover
        twist = Twist()
        twist.angular = angular_vel
        twist.linear = linear_vel
        self.context.rover.send_drive_command(twist)

        #TODO: stay in the TagSeekState (with outcome 'working')
        return "working"
        pass