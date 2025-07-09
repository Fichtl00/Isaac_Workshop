import numpy as np
import omni.usd as usd

from isaacsim.robot_setup.assembler import AssembledRobot, RobotAssembler

base_robot_path = "/World/Robot_assembly/Robot/iisy_15"
attach_robot_path = "/World/Robot_assembly/Gripper/Robotiq_2F_140"
base_robot_mount_frame = "/link_6"
attach_robot_mount_frame = "/robotiq_arg2f_base_link"
fixed_joint_offset = np.array([0.0, 0.0, -0.178])
fixed_joint_orient = np.array([0.0, 1.0, 0.0, 0.0])  # Beispiel, ggf. anpassen
single_robot = True # merge articulations

stage = usd.get_context().get_stage()

if not stage.GetPrimAtPath("/World/Robot_assembly").IsValid():
    print("Please run assemble_scene.py first to assemble the robot before executing this script.")
    exit()

robot_assembler: AssembledRobot = RobotAssembler()
assembled_robot = robot_assembler.assemble_articulations(
    base_robot_path,
    attach_robot_path,
    base_robot_mount_frame,
    attach_robot_mount_frame,
    fixed_joint_offset,
    fixed_joint_orient,
    mask_all_collisions=True,
    single_robot=single_robot
)

print("Robot and Gripper assembled")
