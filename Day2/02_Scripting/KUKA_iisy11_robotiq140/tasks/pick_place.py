# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from typing import Optional

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper


class PickPlace(tasks.PickPlace):
    def __init__(
        self,
        name: str = "iisy11_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=np.array([0.0515, 0.0515, 0.0515]),
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        asset_path = os.path.join(os.path.dirname(__file__), "../iisy11_1300_Robotiq_2f_140.usd")
        add_reference_to_stage(usd_path=asset_path, prim_path="/lbr_iisy11_r1300")
        # define the gripper
        # Adjusted for isaac sim 4.5 compatibility and stable pick and place
        gripper = ParallelGripper(
            end_effector_prim_path="/lbr_iisy11_r1300/ee_link/robotiq_arg2f_base_link",
            joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
            joint_opened_positions=np.array([0,-0.84]),
            joint_closed_positions=np.array([0.8,-0.2]),
            action_deltas=np.array([-0.6,0.2]),
            #use_mimic_joints=True,
        )
        # define the manipulator
        manipulator = SingleManipulator(
            prim_path="/lbr_iisy11_r1300",
            name="lbr_iisy11_r1300",
            end_effector_prim_path="/lbr_iisy11_r1300/ee_link/robotiq_arg2f_base_link",
            gripper=gripper,
        )
        joints_default_positions = np.zeros(12)
        joints_default_positions[7] = 0.628
        joints_default_positions[8] = 0.628
        manipulator.set_joints_default_state(positions=joints_default_positions)
        return manipulator
