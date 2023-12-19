from robile_safety_features import behaviors
from robile_safety_features.VelCommand import VelCommand
import math


class TestBehaviors:
    
    def test_instantiation(self):
        bb = behaviors.BatteryStatus2bb()

        assert isinstance(bb, behaviors.BatteryStatus2bb), "Instantiation somehow went wrong?"

    def test_fail_instantiation(self):
        bb = behaviors.BatteryStatus2bb()

        assert not isinstance(bb, behaviors.LaserScan2bb), "What a surprise"
    
    def test_backward_motion_isinstance(self):
        vel_command = VelCommand()

        assert (max(vel_command.lin_vel) == 0.0) and (min(vel_command.lin_vel)) == 0.0  , "Expected zero motion!" 

        vel_command.reverse_motion()

        assert (vel_command.lin_vel[0] < 0.0) and (max(vel_command.lin_vel) == 0.0), "Robot not moving backwards!"