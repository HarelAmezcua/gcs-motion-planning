from pydrake.math import RollPitchYaw
from pydrake.math import RigidTransform
# (Optional) Clear any previous MeshCat elements
meshcat.Delete()
meshcat.DeleteAddedControls()

def make_rigid_transform(rpy, xyz):
    """
    Create a RigidTransform from roll-pitch-yaw (in radians) and (x, y, z).
    
    Args:
        rpy: [roll, pitch, yaw] in radians
        xyz: [x, y, z] in meters

    Returns:
        A drake RigidTransform instance.
    """
    roll, pitch, yaw = rpy
    x, y, z = xyz
    rotation = RollPitchYaw(roll, pitch, yaw).ToRotationMatrix()
    translation = [x, y, z]
    return RigidTransform(rotation, translation)

from pydrake.all import (
    InverseKinematics,
    RotationMatrix,
    RigidTransform,
    Solve,
)

def create_seeds_for_transforms(
    plant,
    plant_context,
    diagram,
    gripper_model,
    transform_list,
    initial_guess_dictionary=None,
    constraints_dict=None
):
    """
    Solve IK for a list of desired end-effector transforms.

    Args:
        plant: A MultibodyPlant instance.
        plant_context: The plant's Context.
        diagram: The Diagram containing plant (optional if you need it for referencing).
        gripper_model: ModelInstanceIndex or similar reference for the gripper.
        transform_list: A list of dictionaries, each containing { "rpy": [...], "xyz": [...], ... }.
        initial_guess_dictionary: (Optional) For setting robot initial joint guesses per transform.
        constraints_dict: (Optional) Dictionary of constraints or bounding boxes for orientation, position, etc.

    Returns:
        A list of joint solutions (each is a numpy array) corresponding to each transform.
    """
    seeds = []

    # Access the end-effector (tool) frame.
    gripper_frame = plant.GetFrameByName("tool_center_point", gripper_model)

    for i, transform_spec in enumerate(transform_list):
        print(f"\n--- Solving IK for Transform #{i+1} ---")

        # Build the IK problem
        ik = InverseKinematics(plant, plant_context)

        # Convert [roll, pitch, yaw, x, y, z] into a RigidTransform
        desired_pose = make_rigid_transform(
            rpy=transform_spec["rpy"],
            xyz=transform_spec["xyz"],
        )

        # Add position constraint. 
        #   Example: The origin of the gripper_frame must be within ± some tolerance of desired_pose’s translation.
        pos_lower = desired_pose.translation() - 1e-2
        pos_upper = desired_pose.translation() + 1e-2
        ik.AddPositionConstraint(
            frameB=gripper_frame, 
            p_BQ=[0, 0, 0],           # The point Q in the gripper frame, e.g., the "tip"
            frameA=plant.world_frame(), 
            p_AQ_lower=pos_lower,
            p_AQ_upper=pos_upper
        )

        # Add orientation constraint.
        #   Example: The gripper frame’s orientation must be within a small angular tolerance from the desired R.
        desired_R = desired_pose.rotation()
        ik.AddOrientationConstraint( 
            frameAbar=plant.world_frame(),
            R_AbarA=desired_R,
            frameBbar=gripper_frame,
            R_BbarB=RotationMatrix(),
            theta_bound=0.1  # rad tolerance
        )
        ik.AddMinimumDistanceLowerBoundConstraint(0.0001, 0.001)

        # If you want to add custom bounding boxes for position or orientation from constraints_dict, do so here
        if constraints_dict is not None:
            # e.g. p_AQ_lower, p_AQ_upper, etc.
            pass

        # Set an initial guess for the solver
        if initial_guess_dictionary:
            q0 = set_initial_guess_from_dictionary(
                plant, plant_context, initial_guess_dictionary, transform_spec
            )
        else:
            # Or simply use the current context as the guess
            q0 = plant.GetPositions(plant_context).copy()

        ik.prog().SetInitialGuess(ik.q(), q0)

        # Solve the IK
        result = Solve(ik.prog())
        if result.is_success():
            print(f"IK succeeded for transform #{i+1}.")
            q_sol = result.GetSolution(ik.q())
            seeds.append(q_sol)
        else:
            print(f"IK failed for transform #{i+1}.")
            # You may want to append None or skip
            seeds.append(None)

    return seeds

def set_initial_guess_from_dictionary(plant, plant_context, initial_guess_dictionary, transform_spec):
    """
    A placeholder function that picks an initial guess (q0) for the solver
    based on your dictionary plus the transform's index or type.
    Modify as needed for your usage.
    """
    # For demonstration, just return the current positions:
    return plant.GetPositions(plant_context).copy()

import time

# Build the scene and plant as usual
(builder, plant, scene_graph, robot_model, 
 gripper_model, table_model, shelf_model, 
 alphabet_soup_model, mustard_model, cherries_model) = create_scene_and_multibody_plant()

# Setup the diagram and context
diagram, context, plant_context, q0, visualizer = setup_diagram_and_context(builder, plant, scene_graph)

# Example transforms list: each item has RPY (radians) and xyz (meters)
# Suppose we want to place the end-effector at these two poses.
transform_list = [
    {
        "rpy": [0, np.pi/2, 0],      # roll, pitch, yaw
        "xyz": [0.35, 0.0, 0.6] # x, y, z
    },
    {
        "rpy": [0, np.pi/2, 0],
        "xyz": [0.35, 0, 0.35]
    },
    {
        "rpy": [0, np.pi/2, 0],
        "xyz": [0.25, 0.5, 0.15]
    }
]

"""
        [0.35, 0, 0.535],        
        [0.35, 0, 0.28],
        [0.35, 0, 0.01]"""
# Optionally define your own initial guesses or constraints
initial_guess_dictionary = {}
constraints_dict = {
    # Example custom bounding:
    # "p_AQ_lower": np.array([x_min, y_min, z_min]),
    # "p_AQ_upper": np.array([x_max, y_max, z_max]),
}
 
# Now create seeds for each transform
ik_solutions = create_seeds_for_transforms(
    plant,
    plant_context,
    diagram,
    gripper_model,
    transform_list,
    initial_guess_dictionary,
    constraints_dict,
)

# Visualize each solution
for i, seed in enumerate(ik_solutions):
    if seed is None:
        print(f"Solution {i+1}: None (IK Failed)")
        continue
    print(f"Solution {i+1}: {seed}")
    plant.SetPositions(plant_context, seed)
    diagram.ForcedPublish(context)
    time.sleep(1)

