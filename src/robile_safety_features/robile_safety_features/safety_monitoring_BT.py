#!/usr/bin/env python3

import py_trees as pt
import py_trees_ros as ptr
import operator

import py_trees.console as console
import rclpy
import sys
from robile_safety_features.behaviors import *
from robile_safety_features.behaviors import Rotate
from robile_safety_features.behaviors import StopMotion
from robile_safety_features.behaviors import BatteryStatus2bb
from robile_safety_features.behaviors import LaserScan2bb 

def check_battery_low_on_blackboard(blackboard: pt.blackboard.Blackboard) -> bool:
    return blackboard.battery_low_warning

def check_collision(blackboard: pt.blackboard.Blackboard) -> bool:
    return blackboard.collision_condition

def create_root() -> pt.behaviour.Behaviour:
    """Structures a behavior tree to monitor the battery status, and start
    to rotate if the battery is low and stop if it detects an obstacle in front of it.
    """

    # we define the root node
    root = pt.composites.Parallel(name="root",
                                  policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))    

    ### we create a sequence node called "Topics2BB" and a selector node called "Priorities"
    topics2BB = pt.composites.Sequence("Topics2BB", memory=False)
    priorities = pt.composites.Selector("Priorities", memory=False)

    ### we create an "Idle" node, which is a running node to keep the robot idle
    idle = pt.behaviours.Running(name="Idle")
    
    """
    TODO:  The first and second level of the tree structure is defined above, but please
    define the rest of the tree structure.

    Class definitions for your behaviours are provided in behaviours.py; you also need to fill out
    the behaviour implementations!

    HINT: Some behaviors from pt.behaviours may be useful to use as well.
    """

    ### YOUR CODE HERE ###
    battery2bb = BatteryStatus2bb()
    laserScan2bb = LaserScan2bb()

    # TODO: construct the behavior tree structure using the nodes and behaviors defined above
    # HINT: for reference, the sample tree structure in the README.md file might be useful
    rotate_platform = Rotate(name='RotatePlatform')
    stop_platform = StopMotion(name='StopPlatform')

    battery_emergency = pt.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=rotate_platform
    )
    collision_emergency = pt.decorators.EternalGuard(
        name="Colliding?",
        condition=check_collision,
        blackboard_keys={"collision_condition"},
        child=stop_platform
    )
    
    root.add_children([topics2BB, priorities])
    priorities.add_children([collision_emergency,battery_emergency,idle])
    topics2BB.add_children([battery2bb,laserScan2bb])

    ### YOUR CODE HERE ###

    return root

def main():
    """Initialises and executes the behavior tree
    """
    rclpy.init(args=None)

    root = create_root()
    tree = ptr.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    # frequency of ticks
    tree.tick_tock(period_ms=100)    

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
