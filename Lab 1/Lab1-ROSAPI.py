import rclpy, tf2_ros
from rclpy.node import Node
import numpy as np

import hello_helpers.hello_misc as hm

class MyNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'my_node', 'my_node', wait_for_first_pointcloud=False)
        # my_node's main logic goes here
        
        # Move the arm and gripper back to it's 'stow' position.
        self.stow_the_robot()
        
        # Extend telescoping arm all the way out
        # And raise the lift to max height simultaneously
        self.move_to_pose({'joint_arm': 0.5, 'joint_lift': 1.0}, blocking=True)
        # Once lifted, move all three wrist motors one at a time
        self.move_to_pose({'wrist_yaw': np.radians(45)}, blocking=True)
        self.move_to_pose({'wrist_pitch': np.radians(45)}, blocking=True)
        self.move_to_pose({'wrist_roll': np.radians(45)}, blocking=True)

        # Open and close the gripper
        self.move_to_pose({'stretch_gripper': 80}, blocking=True)
        self.move_to_pose({'stretch_gripper': 20}, blocking=True)

        # Rotate two motors connected to RealSense (head) one at a time
        self.move_to_pose({'head_pan': np.radians(45)}, blocking=True)
        self.move_to_pose({'head_tilt': np.radians(-45)}, blocking=True)

        # Reset back to stow position
        self.stow_the_robot()

        # Drive robot 0.5 meters forward
        self.move_to_pose({'translate_mobile_base': 0.5}, blocking=False)

        # Rotate robot 180 degrees and drive 0.5 m back to starting position
        self.move_to_pose({'rotate_mobile_base': np.radians(180)}, blocking=True)
        self.move_to_pose({'translate_mobile_base': 0.5}, blocking=False)

node = MyNode()
node.main()



