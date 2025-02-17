# This file contains auxiliary functions that are used in the main code.
import numpy as np
import pydot
from IPython.display import SVG, display

from pydrake.solvers import Solve
from pydrake.math import RotationMatrix

from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.multibody.tree import Body
from pydrake.multibody.inverse_kinematics import InverseKinematics


from src.auxiliar.importing_objects import add_custom_robot, add_custom_wsg, add_table, add_shelf, add_bin, weld_gripper_to_robot


def LoadRobot(plant: MultibodyPlant) -> Body:
    """Setup your plant, and return the body corresponding to your
    end-effector."""
    
    robot_model = add_custom_robot(plant)[0]
    gripper_model = add_custom_wsg(plant)[0]
    table_model = add_table(plant)[0]
    #shelf_model = add_shelf(plant)[0]
    #bin_model = add_bin(plant)[0]    
    weld_gripper_to_robot(plant, robot_model, gripper_model, ee_link_name="FINAL_JOINT_FRAME")
    
    end_effector_body = plant.GetBodyByName("body_gripper", gripper_model)
    return end_effector_body

def get_default_position():
    plant = MultibodyPlant(0.0)
    LoadRobot(plant)
    plant.Finalize()
    context = plant.CreateDefaultContext()
    return plant.GetPositions(context)

def MyInverseKinematics(X_WE, plant=None, context=None):
    if not plant:
        plant = MultibodyPlant(0.0)
        LoadRobot(plant)
        plant.Finalize()
    if not context:
        context = plant.CreateDefaultContext()
    # E = ee_body.body_frame()
    gripper_model = plant.GetModelInstanceByName("final_gripper")
    frame = plant.GetFrameByName("tool_center_point", gripper_model)

    ik = InverseKinematics(plant, context)

    ik.AddPositionConstraint(
        frame, [0, 0, 0], plant.world_frame(), X_WE.translation(), X_WE.translation()
    )

    ik.AddOrientationConstraint(
        frame, RotationMatrix(), plant.world_frame(), X_WE.rotation(), 0.001
    )

    prog = ik.get_mutable_prog()
    q = ik.q()

    q0 = plant.GetPositions(context)
    prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
    prog.SetInitialGuess(q, q0)
    result = Solve(ik.prog())
    if not result.is_success():
        print("IK failed")
        return None
    plant.SetPositions(context, result.GetSolution(q))
    return result.GetSolution(q)


def VisualizeConnectivity(iris_regions):
    graph = pydot.Dot("IRIS region connectivity")
    keys = list(iris_regions.keys())
    for k in keys:
        graph.add_node(pydot.Node(k))
    for i in range(len(keys)):
        v1 = iris_regions[keys[i]]
        for j in range(i + 1, len(keys)):
            v2 = iris_regions[keys[j]]
            if v1.IntersectsWith(v2):
                graph.add_edge(pydot.Edge(keys[i], keys[j], dir="both"))
    display(SVG(graph.create_svg()))