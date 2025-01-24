from pydrake.geometry.optimization import (        
    IrisOptions)
import multiprocessing as mp

def create_iris_options():
    iris_options = IrisOptions()
    iris_options.iteration_limit = 10
    iris_options.num_collision_infeasible_samples = 3
    iris_options.require_sample_point_is_contained = True
    iris_options.relative_termination_threshold = 0.01
    iris_options.termination_threshold = -1
    iris_options.configuration_space_margin = 1e-2

    use_existing_regions_as_obstacles = True
    regions_as_obstacles_scale_factor = 0.95
    # We can compute some regions in parallel.
    num_parallel = mp.cpu_count()


    config = {
    "iris_options": iris_options,
    "use_existing_regions_as_obstacles": use_existing_regions_as_obstacles,
    "iris_regions": {},
    "regions_as_obstacles_scale_factor": regions_as_obstacles_scale_factor,
    "iris_filename": "iris_file",
    "num_parallel": num_parallel
    }
    
    return iris_options, config