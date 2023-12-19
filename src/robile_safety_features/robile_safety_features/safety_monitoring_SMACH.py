#!/usr/bin/env python3

import rclpy
#import executive_smach.smach.smach as smach
import smach

from robile_safety_features.VelCommand import VelCommand
from robile_safety_features.Laser_scan import LaserScanSubscribe
from robile_safety_features.Battery_voltage import BatteryVoltage
#from executive_smach.smach_ros.smach_ros import SmachNode


# Reference: https://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine [but in ROS1]

class MonitorBatteryAndCollision(smach.State):
    """State to monitor the battery level and possible collisions
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        ### YOUR CODE HERE ###
        smach.State.__init__(self, outcomes=['Battery_Low','Battery_High','Is_Colliding','Not_Colliding'],
                             input_keys=['battery_threshold','laser_threshold'])

        self.battery_voltage_sub = BatteryVoltage()
        self.laser_scan_sub =  LaserScanSubscribe()


    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        ### YOUR CODE HERE ###
        laser_scan_range = self.laser_scan_sub.getLaserScanData()
        curr_battery_val = self.battery_voltage_sub.getBatteryVoltage()
        laser_collide_cond=[scan_range < userdata.laser_threshold for scan_range in laser_scan_range]

        if True in laser_collide_cond:
            return 'Is_Colliding'
        if userdata.battery_threshold > curr_battery_val:
            return 'Battery_Low'
        if True not in laser_collide_cond:
            return 'Not_Colliding'
        if userdata.battery_threshold < curr_battery_val:
            return 'Battery_High'

        #raise NotImplementedError()


class RotateBase(smach.State):
    """State to rotate the Robile base
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        ### YOUR CODE HERE ###
        smach.State.__init__(self, outcomes=['Is_Rotating'])
        self.velCommand = VelCommand()

        #raise NotImplementedError()

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        ### YOUR CODE HERE ###
        self.velCommand.set_vel([0.0, 0.0, 0.0], [0.0, 0.0, 0.5])
        self.velCommand.publish_vel()
        return 'Is_Rotating'
        #raise NotImplementedError()

class StopBase(smach.State):
    """State to rotate the Robile base
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        ### YOUR CODE HERE ###
        smach.State.__init__(self, outcomes=['Success'])
        self.velCommand = VelCommand()
        #raise NotImplementedError()

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        ### YOUR CODE HERE ###
        self.velCommand.reset_vel()
        self.velCommand.publish_vel()
        return 'Success'
        #raise NotImplementedError()

# TODO: define any additional if necessary
### YOUR CODE HERE ###

def main(args=None):
    """Main function to initialise and execute the state machine
    """

    # TODO: make it a ROS2 node, set any threshold values, and define state transitions
    ### YOUR CODE HERE ###
    rclpy.init(args=None)
    #smachNode = SmachNode("State_Machine_node")
    state_machine = smach.StateMachine(outcomes=['Abort'])
    state_machine.userdata.battery_threshold = 30.0
    state_machine.userdata.laser_threshold = 0.25

    transitions = {
        'Battery_Low': 'RotateBase',
        'Battery_High': 'MonitorBatteryAndCollision',
        'Is_Colliding': 'StopBase',
        'Not_Colliding': 'MonitorBatteryAndCollision'
        }
    remapping = {
        'battery_threshold': 'battery_threshold',
        'laser_threshold': 'laser_threshold'
        }

    with state_machine:
        smach.StateMachine.add('MonitorBatteryAndCollision', MonitorBatteryAndCollision(node=None),
                                transitions=transitions,
                                remapping=remapping)
        smach.StateMachine.add('RotateBase', RotateBase(node=None),
                                transitions={'Is_Rotating': 'MonitorBatteryAndCollision'})
        smach.StateMachine.add('StopBase', StopBase(node=None),
                                transitions={'Success': 'MonitorBatteryAndCollision'})

    outcome = state_machine.execute()
    print(f"Outcome: {outcome}")

    # rclpy.spin(smachNode)
    # smachNode.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()