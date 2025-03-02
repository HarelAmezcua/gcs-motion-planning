
import numpy as np

from pydrake.common.value import AbstractValue
from pydrake.geometry import (
    QueryObject,
    Rgba,
    Role,
    MeshcatVisualizer,
)
from pydrake.math import RollPitchYaw, RotationMatrix
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydraKE.solvers import Solve
from pydrake.all import(
    MeshcatVisualizer,
    MeshcatVisualizerParams,
)

from auxiliar.importing_objects import add_custom_robot, add_custom_wsg, add_table, add_shelf, add_bin, weld_gripper_to_robot

def initialize_dictionaries(alphabet_soup_model, mustard_model, cherries_model):
    object_dictionary = {
        alphabet_soup_model: {"frame_name": "body_alphabet_soup", "level": "Upper_level", "p_AQ_lower": [-0.01, -0.01, 0.035], "p_AQ_upper": [0.01,0.01,0.075]},
        mustard_model: {"frame_name": "body_mustard", "level": "Middle_level", "p_AQ_lower": [-0.035, -0.01, 0.02], "p_AQ_upper": [0.02,0.01,0.1]},
        cherries_model: {"frame_name": "body_cherries", "level": "Lower_level", "p_AQ_lower": [-0.035, -0.01, 0.02], "p_AQ_upper": [0.02,0.01,0.04]},
    }

    initial_guess_dictionary = {
        "Upper_level": np.array([0.08272460326162771, 0.14916581203514745, 1.5243876747212592, 0.2842089939295549, 0.1440074552134592, -0.2981349776251228], dtype=np.float64), 
        "Middle_level": np.array([0.18712170333911837, 0.0737186397351014, 1.8561300106972425, -0.23452860383219773, 0.21507161400030814, 0.26623220176260004], dtype=np.float64),
        "Lower_level": np.array([0.43617725045954725, -1.1199212481107446, 2.1184269163304155, -1.047214945101428, 1.5942907993927722, 1.5847549429295138], dtype=np.float64)
    }

    return object_dictionary, initial_guess_dictionary


def add_constraints(ik, plant, plant_context, gripper_frame, object_frame, object_pose, object_p_AQ_lower, object_p_AQ_upper):
    """Adds constraints to the IK problem."""
    # Fix object pose
    ik.AddPositionConstraint(
        frameB=object_frame,
        p_BQ=[0, 0, 0],
        frameA=plant.world_frame(),
        p_AQ_lower=object_pose.translation(),
        p_AQ_upper=object_pose.translation(),
    )
    ik.AddOrientationConstraint(
        frameAbar=object_frame,
        R_AbarA=RotationMatrix(),
        frameBbar=plant.world_frame(),
        R_BbarB=object_pose.rotation(),
        theta_bound=0.0,
    )

    # Gripper position constraints
    ik.AddPositionConstraint(
        frameB=gripper_frame,
        p_BQ=[0, 0.0, 0.0],
        frameA=object_frame,
        p_AQ_lower=object_p_AQ_lower,
        p_AQ_upper=object_p_AQ_upper,
    )

    # Desired orientation
    desired_rotation_matrix = RotationMatrix(RollPitchYaw([0, 1.5708, 0]))
    ik.AddOrientationCost(
        frameAbar=plant.world_frame(),
        R_AbarA=desired_rotation_matrix,
        frameBbar=gripper_frame,
        R_BbarB=RotationMatrix(),
        c=100.0,
    )

    ik.AddMinimumDistanceLowerBoundConstraint(0.001, 0.05)


def set_initial_guess(plant,plant_context, initial_guess_dictionary, location):
    """Sets the initial guess for the IK problem."""
    positions_name = plant.GetPositionNames()
    robot_position_indices = []
    robot_positions_names = ["KINOVA_GEN3_LITE_J0_q", "KINOVA_GEN3_LITE_J1_q", 
                         "KINOVA_GEN3_LITE_J2_q", "KINOVA_GEN3_LITE_J3_q", 
                         "KINOVA_GEN3_LITE_J4_q", "KINOVA_GEN3_LITE_J5_q"]

    for outer_iterator in range(6):
        iterator = 0
        for position_name in positions_name:
            if position_name == robot_positions_names[outer_iterator]:
                robot_position_indices.append(iterator)
                break
            iterator += 1

    q0 = plant.GetPositions(plant_context)
    q0[robot_position_indices] = initial_guess_dictionary[location]   
    return q0



def solve_ik(ik, q0):
    """Sets up and solves the IK problem."""
    prog = ik.get_mutable_prog()
    q = ik.q()
    prog.AddQuadraticErrorCost(10*np.identity(len(q)), q0, q)
    prog.SetInitialGuess(q, q0)
    result = Solve(prog)
    return result, q

def create_scene_and_multibody_plant():
    """
    Creates the scene and MultibodyPlant, adds models, and finalizes the plant.

    Returns:
        A tuple containing the builder, plant, scene graph, and model instances of the robot, gripper, table, and shelf.
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    robot_model = add_custom_robot(plant)[0]
    gripper_model = add_custom_wsg(plant)[0]
    table_model = add_table(plant)[0]
    shelf_model = add_shelf(plant)[0]
    bin_model = add_bin(plant)[0]
    #alphabet_soup_model, mustard_model, cherries_model  = add_objects(plant)
    alphabet_soup_model, mustard_model, cherries_model  = None, None, None

    
    weld_gripper_to_robot(plant, robot_model, gripper_model, ee_link_name="FINAL_JOINT_FRAME")
    plant.Finalize()
    
    return builder, plant, scene_graph, robot_model, gripper_model, table_model, shelf_model, alphabet_soup_model, mustard_model, cherries_model

def setup_diagram_and_context(builder, plant, scene_graph, meshcat):
    """
    Sets up the diagram and context for the simulation.

    Args:
        builder: The DiagramBuilder to which the plant and scene graph are added.
        plant: The MultibodyPlant containing the robot and gripper.
        scene_graph: The SceneGraph for visualization.

    Returns:
        A tuple containing the diagram, context, plant context, and initial positions.
    """
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder,
        scene_graph,
        meshcat,
        MeshcatVisualizerParams(delete_prefix_initialization_event=False),
    )

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    q0 = plant.GetPositions(plant_context)
    
    return diagram, context, plant_context, q0, visualizer

