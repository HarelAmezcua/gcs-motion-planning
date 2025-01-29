import numpy as np
import time

from pydrake.math import RigidTransform
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.meshcat import JointSliders
from pydrake.visualization import AddDefaultVisualization

from pydrake.geometry.optimization import (
    HPolyhedron)
from pydrake.geometry import (
    Rgba,
    Sphere,
)
from pydrake.all import Context, Meshcat, SceneGraph

from src.auxiliar.auxiliar_functions import LoadRobot
from src.auxiliar.helper import GenerateRegion
from src.manipulation import running_as_notebook
from typing import Dict

def IrisIndicator(
    scene_graph: SceneGraph,
    scene_graph_context: Context,
    meshcat: Meshcat,
    iris_regions: Dict[str, HPolyhedron],
    p_WIndicator: list,
    q: None
):
    radius = 0.1

    meshcat.SetTransform("iris_indicator", RigidTransform(p_WIndicator))

    query_object = scene_graph.get_query_output_port().Eval(scene_graph_context)
    has_collisions = query_object.HasCollisions()
    if has_collisions:
        meshcat.SetObject("iris_indicator", Sphere(radius), Rgba(1, 0, 0, 1))

    in_any_region = False
    for r in iris_regions.values():
        if r.PointInSet(q):
            if has_collisions:
                print("You found a counter-example!")
                # TODO(russt): Automatically shrink the iris region.
            elif not in_any_region:
                in_any_region = True
                meshcat.SetObject("iris_indicator", Sphere(radius), Rgba(0, 1, 0, 1))

    if not has_collisions and not in_any_region:
        meshcat.SetObject("iris_indicator", Sphere(radius), Rgba(0.5, 0.5, 0.5, 1))




def JointTeleop(meshcat, seeds_joint_teleop, iris_regions, config, q):
    meshcat.Delete()
    meshcat.DeleteAddedControls()
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    LoadRobot(plant)
    plant.Finalize()

    sliders = builder.AddSystem(JointSliders(meshcat, plant))

    AddDefaultVisualization(builder, meshcat)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    scene_graph_context = scene_graph.GetMyContextFromRoot(context)
    sliders_context = sliders.GetMyContextFromRoot(context)

    q
    if len(q) == 0:
        q = plant.GetPositions(plant_context)
    sliders.SetPositions(q)

    # Implements a version of JointSliders.run() which can also watch for extra UI events.
    iris_button_name = "Compute new IRIS region"
    meshcat.AddButton(iris_button_name)
    iris_button_clicks = 0

    stop_button_name = "Stop Joint Teleop"
    print(
        f"Press the '{stop_button_name}' button in Meshcat to continue or press 'Escape'"
    )
    meshcat.AddButton(stop_button_name, "Escape")

    diagram.ForcedPublish(context)
    if not running_as_notebook:
        return

    while meshcat.GetButtonClicks(stop_button_name) < 1:
        # Check if the sliders have changed.
        old_positions = plant.GetPositions(plant_context)
        new_positions = sliders.get_output_port().Eval(sliders_context)

        if meshcat.GetButtonClicks(iris_button_name) > iris_button_clicks:
            iris_button_clicks = meshcat.GetButtonClicks(iris_button_name)
            # TODO(russt): Get the name from meshcat (#19666)
            region_name = "JointTeleopRegion"
            region_num = 0
            while f"{region_name}{region_num}" in iris_regions.keys():
                region_num += 1
            meshcat.AddButton("Generating region (please wait)")
            seeds_joint_teleop.append(new_positions)
            GenerateRegion(f"{region_name}{region_num}", new_positions, config)
            meshcat.DeleteButton("Generating region (please wait)")
        elif np.array_equal(new_positions, old_positions):
            time.sleep(1e-3)
            continue

        # Publish the new positions.
        plant.SetPositions(plant_context, new_positions)
        IrisIndicator(
            scene_graph,
            scene_graph_context,
            meshcat,
            iris_regions,
            p_WIndicator=[1, 1, 1],  
            q = q          
        )
        diagram.ForcedPublish(context)

    meshcat.DeleteAddedControls()