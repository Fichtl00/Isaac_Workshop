# SPDX-FileCopyrightText: Copyright (c) 2025 Institute for Production and Informatics, Kempten University.
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
# Based on the original script by NVIDIA Corporation, modified for educational purposes.

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import numpy as np
import os

from controllers.pick_place import PickPlaceController
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from tasks.pick_place import PickPlace

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)

# Add the Pick Place Scene to the environment
scene_path = os.path.join(os.path.dirname(__file__), "../Scene/Pick_Place_Scene.usd")
add_reference_to_stage(usd_path=scene_path, prim_path="/Environment")



target_position = np.array([-0.3, 0.6, 0])
target_position[2] = 0.075
my_task = PickPlace(name="iisy11_pick_place", target_position=target_position)

my_world.add_task(my_task)
my_world.reset()
my_iisy11 = my_world.scene.get_object("lbr_iisy11_r1300")
# initialize the controller
my_controller = PickPlaceController(name="controller", robot_articulation=my_iisy11, gripper=my_iisy11.gripper)
task_params = my_world.get_task("iisy11_pick_place").get_params()
articulation_controller = my_iisy11.get_articulation_controller()
i = 0
reset_needed = False
is_done = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
            is_done = False
        observations = my_world.get_observations()
        # forward the observation values to the controller to get the actions
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            end_effector_offset=np.array([0, 0, 0]),
        )
        if my_controller.is_done():
            if not is_done:
                print("done picking and placing")
                is_done = True
        articulation_controller.apply_action(actions)
    if args.test is True:
        break
simulation_app.close()
