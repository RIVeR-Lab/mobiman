from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
#from omni.isaac.manipulators import SingleManipulator
#from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
#from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.extensions import enable_extension
# Enable ROS bridge extension
enable_extension("omni.isaac.ros_bridge")
simulation_app.update()
import omni.isaac.ros_bridge
import numpy as np

my_world = World(stage_units_in_meters=1.0)
#TODO: change this to your own path
asset_path = "/home/akmandor/ros_workspaces/mobiman_ws/src/mobiman/mobiman_simulation/models/usd/jackal_jaco/jackal_jaco.usd"
#asset_path = "/home/akmandor/ros_workspaces/mobiman_ws/src/mobiman/mobiman_simulation/models/usd/scenes/jackal_jaco_ground.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/jackal_jaco")
#define the gripper
#gripper = ParallelGripper(
    #We chose the following values while inspecting the articulation
#    end_effector_prim_path="/World/cobotta/onrobot_rg6_base_link",
#    joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
#    joint_opened_positions=np.array([0, 0]),
#    joint_closed_positions=np.array([0.628, -0.628]),
#    action_deltas=np.array([-0.628, 0.628]),
#)
#define the manipulator
#my_denso = my_world.scene.add(SingleManipulator(prim_path="/World/cobotta", 
#                                                name="cobotta_robot",
#                                                end_effector_prim_name="onrobot_rg6_base_link", gripper=gripper))

#set the default positions of the other gripper joints to be opened so
#that its out of the way of the joints we want to control when gripping an object for instance.
#joints_default_positions = np.zeros(12)
#joints_default_positions[7] = 0.628
#joints_default_positions[8] = 0.628
#my_denso.set_joints_default_state(positions=joints_default_positions)
my_world.scene.add_default_ground_plane()
my_world.reset()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    '''
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        i += 1
        
        gripper_positions = my_denso.gripper.get_joint_positions()
        if i < 500:
            #close the gripper slowly
            my_denso.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] + 0.1, gripper_positions[1] - 0.1]))
        if i > 500:
            #open the gripper slowly
            my_denso.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] - 0.1, gripper_positions[1] + 0.1]))
        if i == 1000:
            i = 0
        '''

simulation_app.close()