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
