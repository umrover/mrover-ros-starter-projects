from abc import ABC
from typing import List
from navigation.drive import get_drive_command

import smach
from context import Context
from geometry_msgs.msg import Twist


class BaseState(smach.State, ABC):
    """
    Custom base state which handles termination cleanly via smach preemption.
    """

    context: Context

    def __init__(
        self,
        context: Context,
        add_outcomes: List[str] = None,
        add_input_keys: List[str] = None,
        add_output_keys: List[str] = None,
    ):
        add_outcomes = add_outcomes or []
        add_input_keys = add_input_keys or []
        add_output_keys = add_output_keys or []
        super().__init__(
            add_outcomes + ["terminated"],
            add_input_keys,
            add_output_keys,
        )
        self.context = context

    def execute(self, ud):
        """
        Override execute method to add logic for early termination.
        Base classes should override evaluate instead of this!
        :param ud:  State machine user data
        :return:    Next state, 'terminated' if we want to quit early
        """
        if self.preempt_requested():
            self.service_preempt()
            return "terminated"
        return self.evaluate(ud)

    def evaluate(self, ud: smach.UserData) -> str:
        """Override me instead of execute!"""
        pass


class DoneState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=["done"],
        )

    def evaluate(self, ud):
        # Stop rover
        cmd_vel = Twist()
        self.context.rover.send_drive_command(cmd_vel)
        return "done"


class DriveState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=["drive", "done"],
        )
    
    def evaluate(self):
        # move rover to 5, 5
        dc = get_drive_command([5, 5], self.context.rover.get_pose(), 0.0, 0.0)
        if not dc[1]:
            self.context.rover.send_drive_command(dc[0])
            return "drive"
        else:
            return "done"


