import time
import numpy as np
import stretch_body.robot
robot = stretch_body.robot.Robot()
robot.startup()

# Move the arm and gripper back to it's 'stow' position.
robot.stow()

# Extend telescoping arm all the way out
# And raise the lift to max height simultaneously
robot.arm.move_to(0.5)
robot.lift.move_to(1.0)
robot.push_command()
robot.arm.wait_until_at_setpoint()

# Once lifted, move all three wrist motors one at a time
robot.end_of_arm.move_to('wrist_yaw', 0.5)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint('wrist_yaw')

robot.end_of_arm.move_to('wrist_pitch', 0.5)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint('wrist_pitch')

robot.end_of_arm.move_to('wrist_roll', 0.5)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint('wrist_roll')

# Open and close the gripper
robot.end_of_arm.move_to('stretch_gripper', 80)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint('stretch_gripper')

robot.end_of_arm.move_to('stretch_gripper', 20)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint('stretch_gripper')

# Rotate two motors connected to RealSense (head) one at a time
robot.head.move_by('head_pan', np.radians(45))
robot.push_command()
robot.head.wait_until_at_setpoint('head_pan')

robot.head.move_by('head_tilt', np.radians(-45))
robot.push_command()
robot.head.wait_until_at_setpoint('head_tilt')

# Reset back to stow position
robot.stow()

# Drive robot 0.5 meters forward
robot.base.translate_by(0.5)
robot.push_command()
time.sleep(3.0)  # wait for motion to complete

# Rotate robot 180 degrees and drive 0.5 m back to starting position
robot.base.rotate_by(np.radians(180))
robot.base.translate_by(0.5)
robot.push_command()
time.sleep(5.0)