# import functions to read xml file and visualize commonroad objects
#import matplotlib
#matplotlib.use('Agg')

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
import math


################ collision checker ##############################
import numpy as np
import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.boundary import boundary
from time import time
from commonroad_dc.collision.trajectory_queries import trajectory_queries
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker, create_collision_object
from commonroad.prediction.prediction import TrajectoryPrediction, SetBasedPrediction


def get_trajectory_list(loaded, obb_hull_preprocess=True):
    traj_matrix=loaded["trajectories"].reshape(1000,-1)
    start_time=np.asfortranarray(loaded["start_time_step"])
    half_car_length, half_car_width=loaded["car_shape"]/2

    # create a trajectory batch
    trajectories_batch=pycrcc.OBBTrajectoryBatch(traj_matrix,start_time, half_car_length, half_car_width)
    # preprocess the trajectory batch for continuous collision detection using OBB Hull method (see also Tutorial 03)
    if obb_hull_preprocess:
        trajectories_batch.preprocess_()

    return trajectories_batch.to_tvobstacle()


def get_scenario_dynamic_obstacles_list(scenario):
    dyn_obstacles_list=list()
    for dyn_obst in scenario.dynamic_obstacles:
        if isinstance(dyn_obst.prediction,TrajectoryPrediction):
            co=create_collision_object(dyn_obst.prediction)
            # preprocess using obb hull for continuous collision detection
            co, err=trajectory_queries.trajectory_preprocess_obb_sum(co)
            dyn_obstacles_list.append(co)
        else:
            if isinstance(dyn_obst.prediction,SetBasedPrediction):
                co=create_collision_object(dyn_obst.prediction)
                dyn_obstacles_list.append(co)
            else:
                raise Exception('Unknown dynamic obstacle prediction type: ' + str(type(dyn_obst.prediction)))
    return dyn_obstacles_list

def GetRotationAndLaneWidth(scenario):
    # get lane rotation angle (theta)
    delta_x = scenario.lanelet_network.lanelets[0].left_vertices[-1][0] - scenario.lanelet_network.lanelets[0].left_vertices[0][0]
    delta_y = scenario.lanelet_network.lanelets[0].left_vertices[-1][1] - scenario.lanelet_network.lanelets[0].left_vertices[0][1]
    theta_rad = math.atan(delta_y/delta_x)
    theta = math.degrees(theta_rad)
    # get lane width
    print("raw width ",(scenario.lanelet_network.lanelets[0].left_vertices[0][1]-scenario.lanelet_network.lanelets[0].right_vertices[0][1]))
    lane_width = abs( (scenario.lanelet_network.lanelets[0].left_vertices[0][1]-scenario.lanelet_network.lanelets[0].right_vertices[0][1])* math.cos(theta_rad))

    return theta,lane_width
################################################################

def extract_data(file_path):
    # read in the scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # get ego initial state in the scenario
    INIT_STATE = planning_problem.initial_state
    
    # get dynamic obstacles in the scenario
    obstacles = []
    for dyn_obst in scenario.dynamic_obstacles:
        obs_pos = dyn_obst.initial_state.position
        obs_orien = dyn_obst.initial_state.orientation
        obs_vel = dyn_obst.initial_state.velocity
        #obs_acc = dyn_obst.initial_state.acceleration
        obstacles.append((obs_pos[0],obs_pos[1],obs_orien,obs_vel))
    
    theta, lane_width = GetRotationAndLaneWidth(scenario)
    
    print("theta in degrees is ", theta)
    print("lane width is ", lane_width)


    # plot the planning problem and the scenario for the fifth time step
    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    scenario.draw(rnd)#(rnd, draw_params={'time_begin': 5})
    planning_problem_set.draw(rnd)
    rnd.render()
    plt.show()

    return INIT_STATE.position[0],INIT_STATE.position[1],INIT_STATE.orientation,INIT_STATE.velocity , obstacles

def all_functions(file_path):
    # generate path of the file to be opened
    # read in the scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    INIT_STATE = planning_problem.initial_state
    print ("initial state: ", INIT_STATE)
    for state in planning_problem.goal.state_list:
        print("goal state: ",state)

    for state in scenario.obstacle_states_at_time_step(0):
        print ("scenario obstacles: ",state)

    dyn_obstacles_list=list()
    for dyn_obst in scenario.dynamic_obstacles:
        print("dyn_obs: " ,dyn_obst)
        obs_pos = dyn_obst.initial_state.position
        obs_orien = dyn_obst.initial_state.orientation
        obs_vel = dyn_obst.initial_state.velocity
        #obs_acc = dyn_obst.initial_state.acceleration
        #print("dyn obs * pos,orient,vel,acc:", obs_pos ,obs_orien,obs_vel,obs_acc)



    ################### collision checker #########################
    data=np.load("/home/mais/testCommonRoad/commonroad_io/commonroad/tutorials/collision_checker/USA_US101-3_3_T-1_waypoints.npz")
    print("data: ",data)
    loaded_data = {"car_shape": data["car_shape"], "trajectories": data["trajectories"], "start_time_step": data["start_time_step"]}
    data.close()

    trajectories = get_trajectory_list(loaded_data)
    waypoints=loaded_data["trajectories"]
    car_shape=loaded_data["car_shape"]
    car_half_length, car_half_width = (car_shape/2)

    # create the road boundary with default method uses oriented rectangles
    road_boundary_obstacle, road_boundary_sg_rectangles=boundary.create_road_boundary_obstacle(scenario)
    road_polygons = boundary.create_road_polygons(scenario, method='whole_polygon', triangulate=False)

    # Draw an exemplary part of trajectory batch (here: 50 trajectories) and the road boundary
    n_traj_draw=50
    offset=350

    rnd = MPRenderer(figsize=(25, 10))
    scenario.draw(rnd)
    road_boundary_sg_rectangles.draw(rnd)
    for tvo in trajectories[offset:offset+n_traj_draw]:
        tvo.draw(rnd, draw_params={'facecolor': 'green'})
    rnd.render()


    # check computation time over ten runs
    num_trials=10
    cur_time_1=time()
    for i in range(num_trials):
        # check if the trajectory collides with the road boundary
        ret=trajectory_queries.trajectories_collision_static_obstacles(trajectories, road_boundary_sg_rectangles, method='grid', num_cells=32, auto_orientation=True, optimize_triangles=True)
    cur_time_2 = time()

    print("%s out of %s checked trajectories do not collide with the road boundary." % (ret.count(-1), len(trajectories)))
    print("Time for %s trajectory checks: " % (len(trajectories)) + str((cur_time_2-cur_time_1)/num_trials)+ " sec.")


    dynamic_obstacles=get_scenario_dynamic_obstacles_list(scenario)
    # check computation time over ten runs
    num_trials=10

    cur_time_1=time()
    for i in range(num_trials):
        res_dynamic = trajectory_queries.trajectories_collision_dynamic_obstacles(trajectories, dynamic_obstacles, method='box2d')
    cur_time_2 = time()

    print("%s out of %s trajectories do not collide with the other vehicles" % (res_dynamic.count(-1), len(trajectories)))
    print("Time for %s trajectory checks: " % (len(trajectories)) + str((cur_time_2-cur_time_1)/num_trials)+ " sec.")
    ############################################################






    # plot the planning problem and the scenario for the fifth time step
    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    scenario.draw(rnd)#(rnd, draw_params={'time_begin': 5})
    planning_problem_set.draw(rnd)
    rnd.render()
    plt.show()

    return INIT_STATE.position[0],INIT_STATE.position[1],INIT_STATE.orientation,INIT_STATE.velocity



