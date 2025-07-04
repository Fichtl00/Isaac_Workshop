# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from typing import Optional, Sequence

import omni.kit.app
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.robot.manipulators.grippers.gripper import Gripper
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper


class SingleManipulatorFixed(SingleArticulation):
    """Provides high level functions to set/ get properties and actions of a manipulator with a single end effector
    and optionally a gripper.

    Args:

        prim_path (str): prim path of the Prim to encapsulate or create.
        end_effector_prim_path (str): end effector prim path to be used to track the rigid body that corresponds
                                        to the end effector. One of the following args can be specified only:
                                        end_effector_prim_name or end_effector_prim_path.
        name (str, optional): shortname to be used as a key by Scene class. Note: needs to be unique if the
                                object is added to the Scene. Defaults to "single_manipulator".
        position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                        Defaults to None, which means left unchanged.
        translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                            (with respect to its parent prim). shape is (3, ).
                                                            Defaults to None, which means left unchanged.
        orientation (Optional[Sequence[float]], optional): quaternion orientation in the world/ local frame of the prim
                                                            (depends if translation or position is specified).
                                                            quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                            Defaults to None, which means left unchanged.
        scale (Optional[Sequence[float]], optional): local scale to be applied to the prim's dimensions. shape is (3, ).
                                                        Defaults to None, which means left unchanged.
        visible (Optional[bool], optional): set to false for an invisible prim in the stage while rendering. Defaults to True.
        gripper (Gripper, optional): Gripper to be used with the manipulator. Defaults to None.
    """

    def __init__(
        self,
        prim_path: str,
        end_effector_prim_path: str,
        name: str = "single_manipulator",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        gripper: Gripper = None,
    ) -> None:
        self._end_effector_prim_path = end_effector_prim_path
        self._gripper = gripper
        self._end_effector = None
        SingleArticulation.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            articulation_controller=None,
        )
        return

    @property
    def end_effector(self) -> SingleRigidPrim:
        """
        Returns:
            SingleRigidPrim: end effector of the manipulator (can be used to get its world pose, angular velocity..etc).
        """
        return self._end_effector

    @property
    def gripper(self) -> Gripper:
        """
        Returns:
            Gripper: gripper of the manipulator (can be used to open or close the gripper, get its world pose or angular velocity..etc).
        """
        return self._gripper

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates an articulation view using physX tensor api.
            This needs to be called after each hard reset (i.e stop + play on the timeline) before interacting with any
            of the functions of this class.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.
        """
        SingleArticulation.initialize(self, physics_sim_view=physics_sim_view)
        self._end_effector = SingleRigidPrim(prim_path=self._end_effector_prim_path, name=self.name + "_end_effector")
        self._end_effector.initialize(physics_sim_view)
        if isinstance(self._gripper, ParallelGripper):
            self._gripper.initialize(
                physics_sim_view=physics_sim_view,
                articulation_apply_action_func=self.apply_action,
                get_joint_positions_func=self.get_joint_positions,
                set_joint_positions_func=self.set_joint_positions,
                dof_names=self.dof_names,
            )
        if isinstance(self._gripper, SurfaceGripper):
            self._gripper.initialize(physics_sim_view=physics_sim_view, articulation_num_dofs=self.num_dof)
        return

    def post_reset(self) -> None:
        """Resets the manipulator, the end effector and the gripper to its default state."""
        SingleArticulation.post_reset(self)
        self._end_effector.post_reset()
        if self._gripper != None:
            self._gripper.post_reset()
        return
