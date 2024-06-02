import numpy as np
#import time
import ffstreams.utils.commonroad_scenario_extractor as cr_extractor
import ffstreams.ffstreams.frenet_optimal_trajectory_general as opt
from ffstreams.ffstreams.ffstreams_algorithm import solve_ffstreams_general
from ffstreams.utils.apollo_utils import ObstacleState,EgoState

from predict import predict_traj
import yaml
with open('ffstreams/config/config.yml', 'r') as file:
    config = yaml.safe_load(file)
config = config['general']


def get_const_speed_obs_traj(obstacles,scene_path,time_step):
    num_obstacles = len(obstacles)
    if time_step <= 50:
        obs_traj = np.empty([num_obstacles, 1, 60,2]) # dimensions of predicted trajectories from QCNet
        for i in range(num_obstacles):
            obs_init_x = obstacles[i].x
            obs_init_y = obstacles[i].y
            obs_init_v = obstacles[i].v
            for k in range(60):
                obs_traj[i,0,k,0] = obs_init_x + obs_init_v*(k/10.0) # x position of obstacle
                obs_traj[i,0,k,1] = obs_init_y  # y position of obstacle
        pred_obs_traj = obs_traj
        pred_traj = None
        pred_prob = None
        pred_obs_traj_sec = None
    ### predict trajectories
    else:
        pred_traj,pred_prob = predict_traj(time_step)
        pred_obs_traj = np.empty([num_obstacles, 1, 60,2])
        ### get trajectories with the highest probability
        for i in range(num_obstacles):
            highest_probability_idx = np.argmax(pred_prob[i])
            pred_obs_traj[i,0,:,:] = pred_traj[i,highest_probability_idx,:,:]
        ### rotate the predicted trajectories to transfer to Frenet 
        # pred_obs_traj = rotate_obstacles_in_scene(pred_obs_traj,num_obstacles,scene_path)
        # pred_obs_traj = obs_traj
        # predicted trajetories with second highest prob
        pred_obs_traj_sec = np.empty([num_obstacles, 1, 60,2])
        for i in range(num_obstacles):
            second_high_prob_idx = np.argsort(pred_prob[i])[-2]
            pred_obs_traj_sec[i,0,:,:] = pred_traj[i,second_high_prob_idx,:,:]
        

    return pred_obs_traj,pred_traj,pred_prob, pred_obs_traj_sec

def main():
    counter_exp = 1
    #statistics_arr = np.zeros(counter_exp)
    scene_path = config['scene_path']+ config['scenario']
    delta_time = config['delta_time']
    standard_delta = config['standard_delta']
    total_time_steps = 1000
    #folder = time.strftime("%Y%m%d-%H%M%S")
    intersection = True
    go_left = True # if false go straight
    if intersection:
        if go_left:
            goal_position = [300,-13]
        else:
            goal_position = [366.5,30]
    curr_time = 0
    #get reference lane and set initial state
    wx,wy,lane_width,q0,scenario = cr_extractor.extract_map_features(scene_path,goal_position)
    ego_state = EgoState()
    ego_state.x = q0[0]
    ego_state.y = q0[1]
    ego_state.v = q0[2]
    ego_state.ds = q0[2]
    # ego_state.yaw = 0
    # curr_dl = 0
    # curr_ddl = 0
    # acc0 = 0
    obstacles = []
    obs = ObstacleState()
    obs.x = 300
    obs.y = -13
    obs.v = 0.1
    obstacles.append(obs)

    for i in range(0,total_time_steps,int(delta_time/standard_delta)):
        print("time step: ", i)
        obs_pred_traj,all_pred_traj,all_prob,obs_pred_traj_sec = get_const_speed_obs_traj(obstacles,scene_path,time_step=i)
        # if i > 50 :
        #     cr_extractor.plot_pred(scenario,ego_state,all_pred_traj,all_prob,wx,wy,i)
        trajectories,confs,traj_dict,final_traj_type = solve_ffstreams_general(ego_state,obstacles,obs_pred_traj,wx,wy,lane_width)

        if len(trajectories) < 1:
            print("SOMETHING IS WRONG")
            if i > 50 :
                trajectories2,confs,traj_dict,final_traj_type = solve_ffstreams_general(ego_state,obstacles,obs_pred_traj_sec,wx,wy,lane_width)
                cr_extractor.plot_pred(scenario,ego_state,all_pred_traj,all_prob,wx,wy,i,ego_traj = None,trajectories2=trajectories2)
        
        else: 
            if i > 50 :
                trajectories2,confs,traj_dict,final_traj_type = solve_ffstreams_general(ego_state,obstacles,obs_pred_traj_sec,wx,wy,lane_width)
                cr_extractor.plot_pred(scenario,ego_state,all_pred_traj,all_prob,wx,wy,i, ego_traj = trajectories[0],trajectories2=trajectories2)
        
            new_ego_state = EgoState()
            new_ego_state.dl = trajectories[0].d_d[1]
            new_ego_state.ddl = trajectories[0].d_dd[1]
            new_ego_state.dds = trajectories[0].s_dd[1]
            # new_obs = []
            # for obs in future_trajectories_obstacles:
            #     new_obs.append(((obs[i][0], obs[i][1]), (5.5, 2.5), (obs[i][2], 0)))
            
            new_ego_state.x = trajectories[0].x[1]
            new_ego_state.y = trajectories[0].y[1]
            new_ego_state.s = trajectories[0].s[1]
            new_ego_state.l = trajectories[0].d[1]
            new_ego_state.ds = trajectories[0].s_d[1]

            ego_state = new_ego_state

        
        #opt.call(wx,wy)

        #init_x, init_y,init_heading,init_speed,init_obstacles,scen_rotation, lane_width,future_trajectories_obstacles,total_time_steps = extractor.extract_data(scene_path)






if __name__ == '__main__':
    main()