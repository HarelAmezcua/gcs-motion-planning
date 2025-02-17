import os
import numpy as np

from pydrake.all import (
    Parser, RigidTransform, RotationMatrix
)

def get_parent_directory():
    # Get the absolute path of the current working directory
    current_dir = os.getcwd()
    parent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
    return parent_dir

def add_custom_robot(plant):
    """
    Adds a custom robot to the plant from an SDF file.

    Args:
        plant: The MultibodyPlant to which the robot will be added.
        sdf_path: The path to the SDF file of the robot.

    Returns:
        The model instance of the added robot.
    """
    # Get the path to the robot SDF file
    sdf_path = os.path.join(get_parent_directory(), 'common-files','models','KINOVA_GEN3_LITE_PRIMITIVES','model.sdf')

    if not os.path.exists(sdf_path):
        raise FileNotFoundError(f"SDF file not found at {sdf_path}")
    
    parser = Parser(plant)
    robot_model = parser.AddModels(sdf_path)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("BASE"))
    return robot_model

def add_custom_wsg(plant):
    """
    Adds a custom gripper to the plant from an SDF file.

    Args:
        plant: The MultibodyPlant to which the gripper will be added.
        gripper_path: The path to the SDF file of the gripper.

    Returns:
        The model instance of the added gripper.
    """
    # Get the path to the gripper SDF file
    gripper_path = os.path.join(get_parent_directory(), 'common-files','models','final_gripper','files','final_gripper.sdf')

    if not os.path.exists(gripper_path):
        raise FileNotFoundError(f"SDF file not found at {gripper_path}")
    parser = Parser(plant)
    wsg_model = parser.AddModels(gripper_path)
    return wsg_model

def weld_gripper_to_robot(plant, robot_model, gripper_model, ee_link_name="FINAL_JOINT_FRAME"):
    """
    Welds the gripper to the robot's end effector link.

    Args:
        plant: The MultibodyPlant containing the robot and gripper.
        robot_model: The model instance of the robot.
        gripper_model: The model instance of the gripper.
        ee_link_name: The name of the robot's end effector link.
    """
    ee_frame = plant.GetFrameByName(ee_link_name, robot_model)  # Extracting End-Effector frame
    gripper_frame = plant.GetFrameByName("body_gripper", gripper_model)  # Using gripper_model instance

    # Weld the frames together
    plant.WeldFrames(ee_frame, gripper_frame)

def add_table(plant):
    """
    Adds a table to the plant from an SDF file.

    Args:
        plant: The MultibodyPlant to which the table will be added.
        table_path: The path to the SDF file of the table.

    Returns:
        The model instance of the added table.
    """
    # Get the path to the table SDF file
    table_path = os.path.join(get_parent_directory(), 'common-files','scene_objects','table','table_wide.sdf')

    if not os.path.exists(table_path):
        raise FileNotFoundError(f"SDF file not found at {table_path}")
    parser = Parser(plant)
    table_model = parser.AddModels(table_path)
    translation_vector = RigidTransform([0.0, 0.0, 0.0])
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("table_body"),translation_vector)
    return table_model
    
def add_shelf(plant):
    """
    Adds a shelf to the plant from an SDF file.

    Args:
        plant: The MultibodyPlant to which the shelf will be added.
        shelf_path: The path to the SDF file of the shelf.

    Returns:
        The model instance of the added shelf.
    """
    # Get the path to the shelf SDF file
    shelf_path = os.path.join(get_parent_directory(), 'common-files','scene_objects','shelve_1','shelves.sdf')

    parser = Parser(plant)
    table_model = parser.AddModels(shelf_path)

    # Transform for final position
    R = RotationMatrix.Identity()
    t = [0.4, 0, 0.4]
    X_WS = RigidTransform(R,t)

    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("shelves_body"), X_WS)
    return table_model

def add_bin(plant):
    """
    Adds a shelf to the plant from an SDF file.

    Args:
        plant: The MultibodyPlant to which the shelf will be added.
        bin_path: The path to the SDF file of the shelf.

    Returns:
        The model instance of the added shelf.
    """
    bin_path = os.path.join(get_parent_directory(), 'common-files','scene_objects','bin','bin.sdf')

    parser = Parser(plant)
    bin_model = parser.AddModels(bin_path)

    # Transform for final position
    R = RotationMatrix.Identity()
    t = [0.1, -0.6, 0.0]
    X_WS = RigidTransform(R,t)

    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("bin_base"), X_WS)
    return bin_model

def add_objects(plant):
    """
    Adds objects to the plant from SDF files.
    Args:
        plant: The MultibodyPlant to which the objects will be added.
    Returns:
        A list of model instances of the added objects.
    """
    alphabet_soup_path = os.path.join(get_parent_directory(), 'common-files','hope_objects','alphabet_soup','alphabet_soup.sdf')
    mustard_path = os.path.join(get_parent_directory(), 'common-files','hope_objects','mustard','mustard.sdf')    
    cherries_path = os.path.join(get_parent_directory(), 'common-files','hope_objects','cherries','cherries.sdf')        
    
    object_paths = [alphabet_soup_path, mustard_path, cherries_path]
    objects_body_frame = { alphabet_soup_path: "body_alphabet_soup", mustard_path: "body_mustard", cherries_path: "body_cherries" }
    
    for object_path in object_paths:
        if not os.path.exists(object_path):
            raise FileNotFoundError(f"SDF file not found at {object_path}")
    
    object_positions = [
        [0.35, 0, 0.535],        
        [0.35, 0, 0.28],
        [0.35, 0, 0.01]
    ]
    
    object_positions = np.array(object_positions)
    parser = Parser(plant)
    #random.shuffle(object_positions)
    objects = []
    
    for i in range(len(object_paths)):
        object_model = parser.AddModels(object_paths[i])[0]
        body_frame_name = objects_body_frame[object_paths[i]]
        plant.SetDefaultFreeBodyPose(
            plant.GetBodyByName(body_frame_name, object_model),
            RigidTransform(object_positions[i])
        )
        objects.append(object_model)
    
    return objects
