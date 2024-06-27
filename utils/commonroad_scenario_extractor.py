# import functions to read xml file and visualize commonroad objects
#import matplotlib
#matplotlib.use('Agg')

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.trajectory import State as StTraj
from ffstreams.utils.common import get_heading
from matplotlib.patches import Rectangle as plt_rect

import math
import copy
import os
import pandas as pd
################ collision checker ##############################
import numpy as np
import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.boundary import boundary
from time import time
from commonroad_dc.collision.trajectory_queries import trajectory_queries
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from vehiclemodels import parameters_vehicle3
from commonroad.geometry.shape import Rectangle

#from commonroad.visualization.draw_params import DynamicObstacleParams

from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker, create_collision_object
from commonroad.prediction.prediction import TrajectoryPrediction, SetBasedPrediction
from commonroad.scenario.state import State,InitialState

SHIFT_IN_FFSTREAM_Y = 48.25
SCENARIO_ROTATION_DEGREES = 0

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
    if delta_x>0 and delta_y >0:
        theta_rad += + 3.14
    theta = math.degrees(theta_rad) 
    # get lane width
    center_vertices = scenario.lanelet_network.lanelets[0].center_vertices
    print(center_vertices[0])
    print(center_vertices[-1])
    print(len(center_vertices))
    print(len(scenario.lanelet_network.lanelets))
    print(scenario.lanelet_network.lanelets[0].left_vertices[0][1])
    print(scenario.lanelet_network.lanelets[0].right_vertices[0][1])
    print("raw width ",(scenario.lanelet_network.lanelets[0].left_vertices[0][1]-scenario.lanelet_network.lanelets[0].right_vertices[0][1]))
    # lane_width = abs( (scenario.lanelet_network.lanelets[0].left_vertices[0][1]-scenario.lanelet_network.lanelets[0].right_vertices[0][1])* math.cos(theta_rad))
    # get lane width
    current_lane = scenario.lanelet_network.lanelets[0]
    p = [current_lane.left_vertices[0][0],current_lane.left_vertices[0][1]]
    q = [current_lane.right_vertices[0][0],current_lane.right_vertices[0][1]]
    lane_width = math.dist(p, q)
    if lane_width < 2.5 :
        lane_width = 3.5
    return theta,lane_width


def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy
################################################################

def extract_data(file_path):
    global SHIFT_IN_FFSTREAM_Y
    global SCENARIO_ROTATION_DEGREES
    # read in the scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # get ego initial state in the scenario
    INIT_STATE = planning_problem.initial_state

    # usefull parameters
    theta, lane_width = GetRotationAndLaneWidth(scenario)  #theta in degrees
    #spetial case with one scenario
    if file_path == "ffstreams/scenarios/commonroad/keep_lane_scenarios/USA_US101-22_3_T-1.xml":
        theta -= 2.5
    #####
    SCENARIO_ROTATION_DEGREES = theta


    print("theta in degrees is ", theta)
    print("lane width is ", lane_width)
    angle = -1*math.radians(SCENARIO_ROTATION_DEGREES)
    # get dynamic obstacles in the scenario
    total_time_steps = scenario.dynamic_obstacles[0].prediction.final_time_step
    print("total_time_steps: ",total_time_steps)
    future_trajectories_obstacles = []
    init_obstacles = []
    for dyn_obst in scenario.dynamic_obstacles:
        print("dyn",dyn_obst)
        #print(dyn_obst.obstacle_shape)
        obs_pos = dyn_obst.initial_state.position
        obs_pos[0],obs_pos[1] = rotate((INIT_STATE.position[0],INIT_STATE.position[1]),(obs_pos[0],obs_pos[1]),angle)
        obs_pos[0] = obs_pos[0] - INIT_STATE.position[0]
        if dyn_obst.obstacle_type == ObstacleType.TRUCK:
            obs_pos[0] -= (dyn_obst.obstacle_shape.length - 5.5)
        obs_pos[1] = obs_pos[1] + SHIFT_IN_FFSTREAM_Y -INIT_STATE.position[1]
        obs_orien = dyn_obst.initial_state.orientation - theta
        obs_vel = dyn_obst.initial_state.velocity
        #obs_acc = dyn_obst.initial_state.acceleration
        #obstacles.append((obs_pos[0],obs_pos[1],obs_orien,obs_vel))
        init_obstacles.append(((obs_pos[0], obs_pos[1]), (5.5, 2.5), (obs_vel, 0)))
        traj = []
        total_steps = dyn_obst.prediction.final_time_step
        if total_steps > total_time_steps:
            total_time_steps = total_steps
        for i in range(1,total_steps):
            if dyn_obst.state_at_time(i) == None:
                total_time_steps = i
                break
            p_x = dyn_obst.state_at_time(i).position[0]
            p_y = dyn_obst.state_at_time(i).position[1] 
            
            p_x , p_y = rotate((INIT_STATE.position[0],INIT_STATE.position[1]),(p_x,p_y),angle)
           
            p_x = p_x - INIT_STATE.position[0]
            #if dyn_obst.obstacle_type == ObstacleType.TRUCK:
            #    p_x -= (dyn_obst.obstacle_shape.length - 5.5)
            p_y = p_y + SHIFT_IN_FFSTREAM_Y -INIT_STATE.position[1]
            
            p_v = dyn_obst.state_at_time(i).velocity
            #print("x y v ",dyn_obst.state_at_time(i).position[0],dyn_obst.state_at_time(i).position[1],dyn_obst.state_at_time(i).velocity)
            traj.append((p_x,p_y,p_v))
        # fix error in obs velocity
        """
        for i in range(len(traj)):
            if traj[i][2] == 0 and i > 0:
                as_list = list(traj[i])
                delta_vx = (traj[i][0]-traj[i-1][0])/0.1
                delta_vy = (traj[i][1]-traj[i-1][1])/0.1
                as_list[2] = math.sqrt(delta_vx*delta_vx + delta_vy*delta_vy)
                as_tuple = tuple(as_list)
                traj[i] = as_tuple
        """
        future_trajectories_obstacles.append(traj)
        
    #print(future_trajectories_obstacles) ##  [[ (obs1_x1,obs1_y1,obs1_v1),(obs1_x2,obs1_y2,obs1_v2), ... ],[(obs2_x1,obs2_y1,obs2_v1),(obs2_x2,obs2_y2,obs2_v2) , ...]]
    #####################################
    ego_orientation_ff = INIT_STATE.orientation - theta
    ego_x_ff = 0 #INIT_STATE.position[0]
    ego_y_ff = SHIFT_IN_FFSTREAM_Y#INIT_STATE.position[1] + SHIFT_IN_FFSTREAM_Y
    
    # plot the planning problem and the scenario for the fifth time step
    # plt.figure(figsize=(25, 10))
    # rnd = MPRenderer()
    # scenario.draw(rnd)#(rnd, draw_params={'time_begin': 5})
    # planning_problem_set.draw(rnd)
    # rnd.render()
    # plt.show()

    return ego_x_ff,ego_y_ff,ego_orientation_ff,INIT_STATE.velocity , init_obstacles , theta, lane_width, future_trajectories_obstacles,total_time_steps

def draw_traj_with_scenario(trajectory,file_path):

    global SHIFT_IN_FFSTREAM_Y
    global SCENARIO_ROTATION_DEGREES

    # time step in CommonRoad is 0.1 second
    scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    initial_state = planning_problem.initial_state
    initial_state.time_step = 0
    state_list = []

    some_state = initial_state
    N = len(trajectory.x)
    print("len of traj is ",N)
    i = 0
    while i < N*2:
        # compute new position
        # add new state to state_list
        some_state=copy.deepcopy(initial_state)
        p_x = trajectory.x[int(i/2)]+ initial_state.position[0]
        p_y = trajectory.y[int(i/2)] + initial_state.position[1]- SHIFT_IN_FFSTREAM_Y
        angle = math.radians(SCENARIO_ROTATION_DEGREES)
        p_x , p_y = rotate((initial_state.position[0],initial_state.position[1]),(p_x,p_y),angle)
        p_yaw = trajectory.yaw[int(i/2)]
        some_state.position[0] = p_x 
        some_state.position[1] = p_y  
        some_state.orientation = p_yaw + angle # should it be in degrees??!! #TOCHECK
        some_state.time_step = i
        state_list.append(some_state)
        i+=2
    
    #for s in state_list:
    #    print("*****",s)
    vehicle3 = parameters_vehicle3.parameters_vehicle3()
    ego_vehicle_shape = Rectangle(length=vehicle3.l, width=vehicle3.w)
    # create the planned trajectory starting at time step 1
    ego_vehicle_trajectory = Trajectory(initial_time_step=2, state_list=state_list[1:])
    # create the prediction using the planned trajectory and the shape of the ego vehicle
    ego_vehicle_prediction = TrajectoryPrediction(trajectory=ego_vehicle_trajectory,shape=ego_vehicle_shape)
    # the ego vehicle can be visualized by converting it into a DynamicObstacle
    ego_vehicle_type = ObstacleType.CAR
    ego_vehicle = DynamicObstacle(obstacle_id=100, obstacle_type=ego_vehicle_type,
                              obstacle_shape=ego_vehicle_shape, initial_state=initial_state,
                              prediction=ego_vehicle_prediction)
    #ego_params = DynamicObstacleParams()
    #ego_params.vehicle_shape.occupancy.shape.facecolor = "g"
    road_boundary_obstacle, road_boundary_sg_rectangles=boundary.create_road_boundary_obstacle(scenario)

    # plot the planning problem and the scenario for the fifth time step
    max_time_step = min(N*2 , 32)
    #draw_params = {'dynamic_obstacle': {'shape': {'rectangle': {'facecolor': 'g'}}}}
    #gif_path = os.path.join(os.getcwd(), os.path.dirname(__file__), 'gifs/tut2.gif')
    
    #rnd = MPRenderer()
    #rnd.create_video([scenario, ego_vehicle], gif_path, 2, 20, draw_params=draw_params, dt=1)
    for i in range(0, max_time_step ,2):
        plt.figure(figsize=(25, 10))
        rnd = MPRenderer()
        scenario.draw(rnd, draw_params={'time_begin': i})
        road_boundary_sg_rectangles.draw(rnd)
        ego_vehicle.draw(rnd, draw_params={'facecolor': 'green','time_begin': i})
        planning_problem_set.draw(rnd)
        rnd.render()
        plt.show()
    """
    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    scenario.draw(rnd, draw_params={'time_begin': 5})
    ego_vehicle.draw(rnd, draw_params={'facecolor': 'green'})
    planning_problem_set.draw(rnd)
    rnd.render()
    plt.show()
    """

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
    #################################################################
    initial_state = planning_problem.initial_state
    initial_state.time_step = 0
    state_list = []
    
    orientation = initial_state.orientation
    some_state = initial_state
    for i in range(4):
        # compute new position
        # add new state to state_list
        some_state=copy.deepcopy(initial_state)
        some_state.position[0] = i
        some_state.time_step = i
        state_list.append(some_state)
    
    #for s in state_list:
    #    print("*****",s)
    vehicle3 = parameters_vehicle3.parameters_vehicle3()
    ego_vehicle_shape = Rectangle(length=vehicle3.l, width=vehicle3.w)
    # create the planned trajectory starting at time step 1
    ego_vehicle_trajectory = Trajectory(initial_time_step=1, state_list=state_list[1:])
    # create the prediction using the planned trajectory and the shape of the ego vehicle
    ego_vehicle_prediction = TrajectoryPrediction(trajectory=ego_vehicle_trajectory,shape=ego_vehicle_shape)

    # the ego vehicle can be visualized by converting it into a DynamicObstacle
    ego_vehicle_type = ObstacleType.CAR
    ego_vehicle = DynamicObstacle(obstacle_id=100, obstacle_type=ego_vehicle_type,
                              obstacle_shape=ego_vehicle_shape, initial_state=initial_state,
                              prediction=ego_vehicle_prediction)
    #ego_params = DynamicObstacleParams()
    #ego_params.vehicle_shape.occupancy.shape.facecolor = "g"
    ################### collision checker #########################
    data=np.load("/home/mais/testCommonRoad/commonroad_io/commonroad/tutorials/collision_checker/USA_US101-3_3_T-1_waypoints.npz")
    print("data: ",data)
    loaded_data = {"car_shape": data["car_shape"], "trajectories": data["trajectories"], "start_time_step": data["start_time_step"]}
    data.close()

    trajectories = get_trajectory_list(loaded_data)
    waypoints=loaded_data["trajectories"]
    #print(" waypoints : ", waypoints)
    #print(" start_time_step : ", loaded_data["start_time_step"])
    car_shape=loaded_data["car_shape"]
    car_half_length, car_half_width = (car_shape/2)

    # create the road boundary with default method uses oriented rectangles
    road_boundary_obstacle, road_boundary_sg_rectangles=boundary.create_road_boundary_obstacle(scenario)
    road_polygons = boundary.create_road_polygons(scenario, method='whole_polygon', triangulate=False)

    # Draw an exemplary part of trajectory batch (here: 50 trajectories) and the road boundary
    n_traj_draw=1
    offset=350

    rnd = MPRenderer(figsize=(25, 10))
    scenario.draw(rnd)
    road_boundary_sg_rectangles.draw(rnd)
    ego_vehicle.draw(rnd, draw_params={'facecolor': 'green'})
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



def  extract_front_obstacle(obstacles,q_ego):
    ### obstacles is a list of tuples (x pos, y pos, linear velocity)
    ### q_ego is the configuration of the ego vehicle , a tuple (x pos, y pos, linear velocity)
    front_obs_idx = 0
    found_front_obs = False
    for i in range(len(obstacles)):
        if abs(obstacles[i][0][1] - q_ego[1]) < 1 :  # compare Y position
            if obstacles[i][0][0] > q_ego[0] :        # compare X position
                if not found_front_obs:
                    found_front_obs = True
                    front_obs_idx = i
                else:
                    if obstacles[i][0][0] < obstacles[front_obs_idx][0][0]: # found a closed front obstacle
                        front_obs_idx = i
    return front_obs_idx


def rotate_obstacles_in_scene(obs_pred_traj,num_obstacles,file_path):
    global SHIFT_IN_FFSTREAM_Y
    global SCENARIO_ROTATION_DEGREES
    # read in the scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # get ego initial state in the scenario
    INIT_STATE = planning_problem.initial_state

    # usefull parameters
    theta, lane_width = GetRotationAndLaneWidth(scenario)  #theta in degrees
    #spetial case with one scenario
    if file_path == "ffstreams/scenarios/commonroad/keep_lane_scenarios/USA_US101-22_3_T-1.xml":
        theta -= 2.5
    #####
    SCENARIO_ROTATION_DEGREES = theta

    angle = -1*math.radians(SCENARIO_ROTATION_DEGREES)
    total_steps = obs_pred_traj[0,0,:,:].shape[0]
    for o in range(num_obstacles):
        for i in range(total_steps):
            p_x = obs_pred_traj[o,0,i,0]
            p_y = obs_pred_traj[o,0,i,1]
            
            p_x , p_y = rotate((INIT_STATE.position[0],INIT_STATE.position[1]),(p_x,p_y),angle)
           
            p_x = p_x - INIT_STATE.position[0]
            #if dyn_obst.obstacle_type == ObstacleType.TRUCK:
            #    p_x -= (dyn_obst.obstacle_shape.length - 5.5)
            p_y = p_y + SHIFT_IN_FFSTREAM_Y -INIT_STATE.position[1]

            obs_pred_traj[o,0,i,0] = p_x
            obs_pred_traj[o,0,i,1] = p_y
        
    return obs_pred_traj


# def get_reference_xy(scenario,init_position,goal_position):
#     init_state = InitialState(position = init_position)
#     init_state.position[0] = init_position[0]
#     init_state.position[1] = init_position[1]

#     goal_state = InitialState(position = goal_position)
#     goal_state.position[0] = goal_position[0]
#     goal_state.position[1] = goal_position[1]

#     reference_ids = []
#     reference_wx = []
#     reference_wy = []

#     current_id = scenario.lanelet_network.find_most_likely_lanelet_by_state([init_state])[0]
#     current_lane = scenario.lanelet_network.find_lanelet_by_id(current_id)
#     goal_lane_id = scenario.lanelet_network.find_most_likely_lanelet_by_state([goal_state])[0]
#     reference_ids.append(current_id)
#     successor_list = current_lane.successor
#     successor_lanes = []
#     for i in successor_list:
#         successor = scenario.lanelet_network.find_lanelet_by_id(i)
#         successor_lanes.append(successor)
#         if i == goal_lane_id:
#             reference_ids.append(i)
#         elif goal_lane_id in successor.successor:
#             reference_ids.append(i)
#             reference_ids.append(goal_lane_id)
    
#     for id in reference_ids:
#         lane = scenario.lanelet_network.find_lanelet_by_id(id)
#         for i in range(len(lane.center_vertices)-1):
#             reference_wx.append(lane.center_vertices[i][0])
#             reference_wy.append(lane.center_vertices[i][1])
#         # append last point in last lane
#         if id == reference_ids[-1]:
#             reference_wx.append(lane.center_vertices[-1][0])
#             reference_wy.append(lane.center_vertices[-1][1])
#     return reference_wx,reference_wy 
def extract_distance_to_intersection(scenario,ego_state,go_left):
    INIT_STATE = InitialState(position = [ego_state.x,ego_state.y])
    if ego_state.y > -14.6 and not go_left:
        return None
    if ego_state.y > -14.7 and go_left:
        return None
    current_id = scenario.lanelet_network.find_most_likely_lanelet_by_state([INIT_STATE])[0]
    if current_id == 475:
        lane = scenario.lanelet_network.find_lanelet_by_id(current_id)
        intersection_position=[lane.center_vertices[-1][0],lane.center_vertices[-1][1]]
        distance_to_intersection = math.dist([ego_state.x,ego_state.y],intersection_position)
        distance_to_intersection -= 5
        distance_to_intersection = max(distance_to_intersection,0)
        return distance_to_intersection - 5
    else:
        return None

def extract_map_features(scene_path,init_position,goal_position,intersection = True):
    distance_to_intersection = None
    scenario, planning_problem_set = CommonRoadFileReader(scene_path).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
    lanelets = scenario.lanelet_network.lanelets
    INIT_STATE = InitialState(position = init_position)
    x_init = INIT_STATE.position[0]
    y_init = INIT_STATE.position[1]
    GOAL_STATE = InitialState(position = goal_position)
    GOAL_STATE.position[0] = goal_position[0]
    GOAL_STATE.position[1] = goal_position[1]
    reference_ids = []
    reference_wx = []
    reference_wy = []

    current_id = scenario.lanelet_network.find_most_likely_lanelet_by_state([INIT_STATE])[0]
    current_lane = scenario.lanelet_network.find_lanelet_by_id(current_id)
    goal_lane_id = scenario.lanelet_network.find_most_likely_lanelet_by_state([GOAL_STATE])[0]
    reference_ids.append(current_id)
    successor_list = current_lane.successor
    successor_lanes = []
    for i in successor_list:
        successor = scenario.lanelet_network.find_lanelet_by_id(i)
        successor_lanes.append(successor)
        if i == goal_lane_id:
            reference_ids.append(i)
        elif goal_lane_id in successor.successor:
            reference_ids.append(i)
            reference_ids.append(goal_lane_id)
    
    for id in reference_ids:
        lane = scenario.lanelet_network.find_lanelet_by_id(id)
        if intersection and id == 475 : #lane before intersection
            intersection_position=[lane.center_vertices[-1][0],lane.center_vertices[-1][1]]
            distance_to_intersection = math.dist(init_position,intersection_position)
        for i in range(len(lane.center_vertices)-1):
            reference_wx.append(lane.center_vertices[i][0])
            reference_wy.append(lane.center_vertices[i][1])
        # append last point in last lane
        if id == reference_ids[-1]:
            reference_wx.append(lane.center_vertices[-1][0])
            reference_wy.append(lane.center_vertices[-1][1])
    if intersection:
        reference_wx = [x_init] + reference_wx[2:]
        reference_wy = [y_init] + reference_wy[2:]
    else:
        reference_wx = [x_init] + reference_wx[9:]
        reference_wy = [y_init] + reference_wy[9:]

    # get lane width
    p = [current_lane.left_vertices[0][0],current_lane.left_vertices[0][1]]
    q = [current_lane.right_vertices[0][0],current_lane.right_vertices[0][1]]
    lane_width = math.dist(p, q)

    #leave only two obstacles
    if intersection:
        scenario.remove_obstacle(scenario.dynamic_obstacles[0])
        scenario.remove_obstacle(scenario.dynamic_obstacles[1])
        scenario.remove_obstacle(scenario.dynamic_obstacles[1])
        scenario.remove_obstacle(scenario.dynamic_obstacles[0])
    else:
        scenario.remove_obstacle(scenario.dynamic_obstacles[2])
        scenario.remove_obstacle(scenario.dynamic_obstacles[2])
        scenario.remove_obstacle(scenario.dynamic_obstacles[3])
        scenario.remove_obstacle(scenario.dynamic_obstacles[3])
        scenario.remove_obstacle(scenario.dynamic_obstacles[3])
        scenario.remove_obstacle(scenario.dynamic_obstacles[3])
        scenario.remove_obstacle(scenario.dynamic_obstacles[5])
        scenario.remove_obstacle(scenario.dynamic_obstacles[3])
    # #### save obstacle profiles into a file ##################
    # dyn_obs_x = []
    # dyn_obs_y = []
    # dyn_obs_yaw = []
    # dyn_obs_v = []
    # dyn_obs_a = []
    # dyn_obs_t = []
    # dyn_obs_length = []
    # dyn_obs_width = []
    # dyn_obs_id = []
    # dyn_obst = scenario.dynamic_obstacles[2]
    # t = []
    # x = []
    # y = []
    # for i in range(100):
    #     t.append(i)
    #     x.append(dyn_obst.state_at_time(i).position[0])
    #     y.append(dyn_obst.state_at_time(i).position[1])
    # for ti in np.arange(0.0, 100.0, 0.25):
    #     dyn_obs_id.append(dyn_obst.obstacle_id)
    #     dyn_obs_t.append(int(ti*4))
    #     dyn_obs_length.append(dyn_obst.obstacle_shape.length)
    #     dyn_obs_width.append(dyn_obst.obstacle_shape.width)

    #     dyn_obs_x.append(np.interp(ti ,t,x))
    #     dyn_obs_y.append(np.interp(ti ,t,y))
    #     dyn_obs_yaw.append(dyn_obst.state_at_time(0).orientation)

    # dyn_obs_v.append(0)    
    # for i in range(1,len(dyn_obs_t))  :  
    #     p = [dyn_obs_x[i],dyn_obs_y[i]]
    #     q = [dyn_obs_x[i-1],dyn_obs_y[i-1]]
    #     ds = math.dist(p, q)
    #     dyn_obs_v.append(ds/0.1)
    
    # dyn_obs_a.append(0) 
    # for i in range(1,len(dyn_obs_t))  :  
    #     dv = dyn_obs_v[i] - dyn_obs_v[i-1]
    #     dyn_obs_a.append(dv/0.1)

            
     
    # data = {}
    # data['time'] = dyn_obs_t
    # data['id'] = dyn_obs_id
    # data['x'] = dyn_obs_x
    # data['y'] = dyn_obs_y
    # data['orientation'] = dyn_obs_yaw
    # data['velocity'] = dyn_obs_v
    # data['acceleration'] = dyn_obs_a
    # data['length'] = dyn_obs_length
    # data['width'] = dyn_obs_width
    
    # df = pd.DataFrame(data=data)
    # file_path = 'ffstreams/scenarios/commonroad/obstacles_profiles/data_obstacle_2_highway.csv'
    # os.makedirs(os.path.dirname(file_path), exist_ok=True)
    # df.to_csv(file_path, index=False)
    # ####### end of save obstacle profiles into a file ##############################
    
    #### try to add obstacle
    # example_obs = scenario.dynamic_obstacles[0]
    # id = 4
    # type = ObstacleType.CAR
    # shape = example_obs.obstacle_shape
    # init_state = InitialState(position = [0,0])
    # init_state.time_step = 0
    # init_state.orientation = 1.4273
    # new_obs = DynamicObstacle(obstacle_id = id,obstacle_type = type,obstacle_shape=shape,initial_state=init_state)

    # scenario.add_objects(new_obs)
    ####################
    # df = pd.read_csv('/home/mais2/QCNet/ffstreams/auto_driving/gifs/general_gifs/exp_20240617-000220_left_turn_79/data_68_success.csv')
    # time_stp = 80#66#52#30
    # time_stp_i = 46#33#11# 0
    # plt.figure(figsize=(25, 10))
    # rnd = MPRenderer()
    # rnd.draw_params.time_begin = time_stp
    # scenario.draw(rnd)
    # # #planning_problem_set.draw(rnd)
    # rnd.render()
    # rect = plt_rect(xy=(df['x'][time_stp_i]-2.5, df['y'][time_stp_i]-1.0), width=5.0, height=2.0, angle=math.degrees(df['yaw'][time_stp_i]), rotation_point='center', color='green',zorder = 20)
    # ax = plt.gca()
    # ax.add_patch(rect)
    # plt.xlim(300, 425)
    # plt.ylim(-50, 40)
    # import matplotlib.font_manager as font_manager

    # font_prop = font_manager.FontProperties(size=14)
    # ax.text(310, -45, "t = 8.0 s", rotation=0,fontproperties=font_prop, verticalalignment='bottom')

    # plt.plot(df['x'][time_stp_i:time_stp_i+24],df['y'][time_stp_i:time_stp_i+24],linewidth=4,color='green',zorder = 20) 
    # plt.show()

    #####################
    # c = df['v'][time_stp_i:time_stp_i+24]
    # plt.scatter(df['x'][time_stp_i:time_stp_i+24], df['y'][time_stp_i:time_stp_i+24], c=c, marker='_',zorder = 20)
    
    # # NPOINTS = 25
    # # COLOR='blue'
    # # RESFACT=10
    # # MAP='winter'
    # # get higher resolution data
    # x = df['x'][time_stp_i:time_stp_i+25]
    # y = df['y'][time_stp_i:time_stp_i+25]
    # dydx = df['v'][time_stp_i:time_stp_i+25]

    # from matplotlib.collections import LineCollection
    # from matplotlib.colors import BoundaryNorm, ListedColormap

    # # Create a set of line segments so that we can color them individually
    # # This creates the points as an N x 1 x 2 array so that we can stack points
    # # together easily to get the segments. The segments array for line collection
    # # needs to be (numlines) x (points per line) x 2 (for x and y)
    # points = np.array([x, y]).T.reshape(-1, 1, 2)
    # segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # # fig, axs = plt.subplots(2, 1, sharex=True, sharey=True)

    # # Create a continuous norm to map from data points to colors
    # norm = plt.Normalize(dydx.min(), dydx.max())
    # lc = LineCollection(segments, cmap='viridis', norm=norm)
    # # Set the values used for colormapping
    # lc.set_array(dydx)
    # lc.set_linewidth(2)
    # line = ax.add_collection(lc)
    # plt.colorbar(line, ax=ax,zorder = 20)

    # # Use a boundary norm instead
    # cmap = ListedColormap(['r', 'g', 'b'])
    # norm = BoundaryNorm([-1, -0.5, 0.5, 1], cmap.N)
    # lc = LineCollection(segments, cmap=cmap, norm=norm)
    # lc.set_array(dydx)
    # lc.set_linewidth(2)
    # line = ax.add_collection(lc)
    # plt.colorbar(line, ax=ax,zorder = 20)

    # ax.set_xlim(x.min(), x.max())
    
    # xHiRes,yHiRes = highResPoints(x,y,RESFACT)
    # npointsHiRes = len(xHiRes)

    # cm = plt.get_cmap(MAP)

    # ax.set_color_cycle([cm(1.*i/(npointsHiRes-1)) 
                        # for i in range(npointsHiRes-1)])

    # color1 = '#FB575D'
    # color2 = '#15251B'
    # color = get_color_gradient(color1, color2, len(x))
    # plt.plot(x,y,color=color,linewidth=4,zorder = 20) 
    # cm = plt.get_cmap(MAP)
    # ax.set_prop_cycle([cm(1.*i/(NPOINTS-1)) for i in range(NPOINTS-1)])
    # for i in range(NPOINTS-1):
    #     ax.plot(x[i:i+2],y[i:i+2])

    
    # plt.plot(df['x'][time_stp_i],df['y'][time_stp_i],'ro',zorder = 20) 
    # plt.plot(goal_position[0],goal_position[1],'ro',zorder = 20) 
    # plt.plot(reference_wx,reference_wy,'go',zorder = 20) 
    # plt.show()
    
    # # plot the planning problem and the scenario for the fifth time step
    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    rnd.draw_params.time_begin = 0
    scenario.draw(rnd)
    #planning_problem_set.draw(rnd)
    rnd.render()
    rect = plt_rect(xy=(x_init-2.5, y_init-1.0), width=5.0, height=2.0, angle=math.degrees(-0.654), rotation_point='center', color='green',zorder = 20)
    ax = plt.gca()
    ax.add_patch(rect)
    # plt.plot(x_init,y_init,'ro',zorder = 20) 
    # plt.plot(goal_position[0],goal_position[1],'ro',zorder = 20) 
    # plt.plot(reference_wx,reference_wy,'g-',zorder = 20) 
    plt.show()
    # # end plot
    ################### collision check example #################
    # # create a trajectory for the ego vehicle starting at time step 0
    # position = [[2.5, 0.0], [4.5, 0.0], [6.5, 0.0], [8.5, 0.0], [10.5, 0.0], [12.5, 0.0], [14.5, 0.0]]
    # state_list = list()
    # for k in range(0, len(position)):
    #     state_list.append(InitialState(time_step= k,position = position[k], orientation = 1.5))
    # trajectory = Trajectory(0, state_list)

    # # create the shape of the ego vehicle
    # shape = Rectangle(length=4.5, width=2.0)
    # # create a TrajectoryPrediction object consisting of the trajectory and the shape of the ego vehicle
    # traj_pred = TrajectoryPrediction(trajectory=trajectory, shape=shape)
    # # params = {'minkowski_sum_circle': False,
    # #           'minkowski_sum_circle_radius': 1.0,
    # #           'resolution': 16,'color':'green'}
    # co = create_collision_object(traj_pred)
    # cc = create_collision_checker(scenario)
    # co2 = create_collision_object(traj_pred)
    # print('Collision between the trajectory of the ego vehicle and objects in the environment: ', cc.collide(co))
    # print('Collision : ', co2.collide(co))

    # plt.figure(figsize=(25, 10))
    # rnd = MPRenderer()
    # scenario.lanelet_network.draw(rnd)
    # cc.draw(rnd)
    # # co.params['color'] = 'green'   
    # co.draw(rnd)
    # rnd.render()
    # plt.show()

    return reference_wx,reference_wy , lane_width ,scenario,distance_to_intersection


def collision_check(ego_traj,obs_traj):
    # ego trajectory (50 length)
    state_list_obj1 = list()
    for k in range(len(ego_traj.x)):
        state_list_obj1.append(InitialState(time_step= k,position = [ego_traj.x[k],ego_traj.y[k]], orientation = ego_traj.yaw[k]))
    trajectory_obj1 = Trajectory(0, state_list_obj1)

    # create the shape of the ego vehicle and obstacles
    shape = Rectangle(length=5.0, width=2.0)
    # create a TrajectoryPrediction object consisting of the trajectory and the shape of the ego vehicle
    traj_pred_obj1 = TrajectoryPrediction(trajectory=trajectory_obj1, shape=shape)

    co = create_collision_object(traj_pred_obj1)
    # obstacle traj (60,2)
    state_list_obj2 = list()
    for k in range(0, len(ego_traj.x)):
        yaw = get_heading(obs_traj[k*2,0],obs_traj[k*2,1],obs_traj[k*2+2,0],obs_traj[k*2+2,1])
        state_list_obj2.append(InitialState(time_step= k,position = [obs_traj[k*2,0],obs_traj[k*2,1]], orientation = yaw))
    trajectory_obj2 = Trajectory(0, state_list_obj2)

    # create a TrajectoryPrediction object consisting of the trajectory and the shape of the ego vehicle
    traj_pred_obj2 = TrajectoryPrediction(trajectory=trajectory_obj2, shape=shape)

    co2 = create_collision_object(traj_pred_obj2)

    #debug
    # plt.figure(figsize=(25, 10))
    # rnd = MPRenderer()
    # # scenario.lanelet_network.draw(rnd)
    # co.draw(rnd)
    # # co.params['color'] = 'green'   
    # co2.draw(rnd)
    # rnd.render()
    # plt.show()
    return co2.collide(co)



def plot_pred(scenario,ego_state,obs_pred_traj,all_pred,all_prob,wx,wy,time_step,ego_traj,trajectories2,intersection = True):

    x_init = ego_state.x
    y_init = ego_state.y
    heading_deg = math.degrees(ego_state.yaw)
    ego_width = 5.0
    ego_height = 2.0
    ## plot the planning problem and the scenario for the fifth time step
    plt.figure(figsize=(25, 10))
    # plt.ioff()
    rnd = MPRenderer()
    rnd.draw_params.time_begin = time_step
    if not intersection:
        heading_deg_obs = math.degrees(-0.654)
        if len(scenario.dynamic_obstacles) >3:
            scenario.remove_obstacle(scenario.dynamic_obstacles[2])


    scenario.draw(rnd)
    #planning_problem_set.draw(rnd)
    rnd.render()
    
    rect = plt_rect(xy=(x_init-ego_width/2, y_init-ego_height/2), width=ego_width, height=ego_height, angle=heading_deg, rotation_point='center', color='green',zorder = 20)
    if not intersection:
        rect = plt_rect(xy=(obs_pred_traj[0,0,0,0]-ego_width/2, obs_pred_traj[0,0,0,1]-ego_height/2), width=ego_width, height=ego_height, angle=heading_deg_obs, rotation_point='center', color='blue',zorder = 20)
    ax = plt.gca()
    ax.add_patch(rect)
    
    # plt.plot(wx,wy,'r-',zorder = 20) 
    #plot predicted trajectories
    if all_pred is None:
        obs_x = obs_pred_traj[0,0,:50,0]
        obs_y = obs_pred_traj[0,0,:50,1] 
        plt.plot(obs_x,obs_y,'b-',zorder = 20) 
    else:
        for i in range(all_pred.shape[0]):
            for k in range(all_pred.shape[1]):
                obs_x = all_pred[i,k,:50,0] 
                obs_y = all_pred[i,k,:50,1] 
                plt.plot(obs_x,obs_y,'-',c='turquoise',zorder = 20) 
            # plot predicted trajectory with highest probability
            highest_probability_idx = np.argmax(all_prob[i])
            obs_x = all_pred[i,highest_probability_idx,:50,0]
            obs_y = all_pred[i,highest_probability_idx,:50,1] 
            plt.plot(obs_x,obs_y,'b-',zorder = 20) 
            # plot predicted trajectory with second highest probability
            second_high_prob_idx = np.argsort(all_prob[i])[-2]
            obs_x = all_pred[i,second_high_prob_idx,:50,0]
            obs_y = all_pred[i,second_high_prob_idx,:50,1] 
            plt.plot(obs_x,obs_y,'-',c='dodgerblue',zorder = 20) 
    plt.plot(ego_traj.x,ego_traj.y,'g-',zorder = 21) 
    plt.plot(x_init,y_init,'ro',zorder = 20) 
    if intersection:
        plt.xlim(300, 425)
        plt.ylim(-50, 40)
    if trajectories2 is not None:
        ego_traj2 = trajectories2[0]
        plt.plot(ego_traj2.x,ego_traj2.y,'y:',zorder = 21) 

    #plt.show()
    filename =  f'{time_step}.png'
    plt.savefig(filename)
    # plt.cla()

    ## end plot

    