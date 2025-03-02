import os

#Drake
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.multibody.parsing import Parser

def add_custom_robot(plant,base_dir):
    """
    Adds a custom robot to the plant from an SDF file.

    Args:
        plant: The MultibodyPlant to which the robot will be added.
        sdf_path: The path to the SDF file of the robot.

    Returns:
        The model instance of the added robot.
    """

    sdf_path= os.path.join(base_dir,'common-files','GEN3-LITE_SDF_NO_GRIPPER.sdf')

    if not os.path.exists(sdf_path):
        raise FileNotFoundError(f"SDF file not found at {sdf_path}")

    parser = Parser(plant)
    robot_model = parser.AddModels(sdf_path)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("BASE"))
    return robot_model