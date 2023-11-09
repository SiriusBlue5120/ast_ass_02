#!/usr/bin/env python3

import rclpy
import smach

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import yaml
import time

# Reference: https://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine [but in ROS1]

class MonitorBatteryAndCollision(smach.State):
    """State to monitor the battery level and possible collisions
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        ### YOUR CODE HERE ###
        smach.State.__init__(self, outcomes=['Battery_Low','Battery_High','Is_Colliding','Not_Colliding'],
                             input_keys=['battery_val','laser_val'])
        self.battery_threshold = 40.0
        self.laser_threshold = 0.25
        #raise NotImplementedError()

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        ### YOUR CODE HERE ###
        if userdata.laser_val >self.laser_threshold:
            return 'Not_Colliding'
        if userdata.laser_val <self.laser_threshold:
            return 'Is_Colliding'
        if userdata.battery_val < self.battery_threshold:
            return 'Battery_Low'
        if userdata.battery_val > self.battery_threshold:
            return 'Battery_High'
        
        #raise NotImplementedError()

    
class RotateBase(smach.State):
    """State to rotate the Robile base
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        ### YOUR CODE HERE ###
        smach.State.__init__(self, outcomes=['Is_Rotating'])
        self.velocity = Twist()
        self.publish = node
        
        #raise NotImplementedError()

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        ### YOUR CODE HERE ###
        self.velocity.angular=[1.0,2.0,3.0]
        self.publish.send(self.velocity)
        return 'Is_Rotating'
        #raise NotImplementedError()

class StopBase(smach.State):
    """State to rotate the Robile base
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        ### YOUR CODE HERE ###
        smach.State.__init__(self, outcomes=['Success'])
        self.velocity = Twist()
        self.publish = node
        #raise NotImplementedError()

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        ### YOUR CODE HERE ###
        self.velocity.linear=[0.0,0.0,0.0]
        self.velocity.angular=[0.0,0.0,0.0]
        self.publish.send(self.velocity)
        return 'Success'
        #raise NotImplementedError()

# TODO: define any additional if necessary
### YOUR CODE HERE ###

def main(args=None):
    """Main function to initialise and execute the state machine
    """

    # TODO: make it a ROS2 node, set any threshold values, and define state transitions
    ### YOUR CODE HERE ###
    state_machine = smach.StateMachine(outcomes=['Abort'])
    with state_machine:
        smach.StateMachine.add('MonitorBatteryAndCollision', MonitorBatteryAndCollision(), 
                                 transitions={'Battery_Low':'RotateBase', 'Battery_High':'MonitorBatteryAndCollision','Is_Colliding':'StopBase','Not_Colliding':'MonitorBatteryAndCollision'})
        smach.StateMachine.add('RotateBase', RotateBase(), 
                                 transitions={'Is_Rotating':'MonitorBatteryAndCollision'})
        smach.StateMachine.add('StopBase', StopBase(), 
                                 transitions={'Success':'MonitorBatteryAndCollision'})

    outcome = state_machine.execute()

if __name__ == "__main__":
    main()