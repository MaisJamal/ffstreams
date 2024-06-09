import numpy as np
import math
import time
import os
import imageio.v2 as imageio
import random 

import ffstreams.utils.commonroad_scenario_extractor as cr_extractor
import ffstreams.ffstreams.frenet_optimal_trajectory_general as opt
from ffstreams.ffstreams.ffstreams_algorithm import solve_ffstreams_general
from ffstreams.utils.apollo_utils import ObstacleState,EgoState
from ffstreams.utils.statistics import Statistics

import pandas as pd
from predict import predict_traj
import yaml
with open('ffstreams/config/config.yml', 'r') as file:
    config = yaml.safe_load(file)
config = config['general']

# def generate_gt_obstacles(init_x,init_y,init_v,init_a,scenario,obs_go_left):
#     init_position = [init_x,init_y]
#     if obs_go_left:
#         goal_position = [300,-13]
#     else:
#         goal_position = [300,-13]

#     obs_ref_x, obs_ref_y = cr_extractor.get_reference_xy(scenario,init_position,goal_position)

def get_const_speed_obs_traj(obstacles,scene_path,time_step):
    num_obstacles = len(obstacles)
    no_prediction = False
    if no_prediction or (not no_prediction and time_step>130):
        obstacle_profile_pth = config['obstacle_profile_path']
        data_obstacle = pd.read_csv(obstacle_profile_pth)
        obs_traj = np.empty([num_obstacles, 1, 60,2]) # dimensions of predicted trajectories from QCNet
        for i in range(num_obstacles):
            for k in range(60):
                obs_init_x = data_obstacle['x'][time_step+k]
                obs_init_y = data_obstacle['y'][time_step+k]
                obs_traj[i,0,k,0] = obs_init_x 
                obs_traj[i,0,k,1] = obs_init_y 
        pred_obs_traj = obs_traj
        pred_traj = None
        pred_prob = None
        pred_obs_traj_sec = None

    else:
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
    counter_exp = 10
    count_success = 0
    # statistics_arr = np.zeros(counter_exp)
    folder = time.strftime("%Y%m%d-%H%M%S")
    
    scene_path = config['scene_path']+ config['scenario']
    delta_time = config['delta_time']
    standard_delta = config['standard_delta']
    obstacle_profile_pth = config['obstacle_profile_path']
    total_time_steps = 150
    #folder = time.strftime("%Y%m%d-%H%M%S")
    intersection = True
    go_left = True # if false go straight
    if intersection:
        if go_left:
            goal_position = [300,-13]
        else:
            goal_position = [366.5,30]

    for exp in range(counter_exp):
        success = False
        failed = True
        stat = Statistics()
        stat.general = True
        #get reference lane and set initial state
        ego_state = EgoState()
        ego_state.x = 360.5051#q0[0]
        ego_state.y = -30.7933#q0[1]
        ego_state.v = random.uniform(3.6,10)#3.6#q0[2]
        ego_state.ds = ego_state.v
        ego_state.yaw = 1.57
        
        init_position = [ego_state.x,ego_state.y]
        wx,wy,lane_width,scenario = cr_extractor.extract_map_features(scene_path,init_position,goal_position)

        data_obstacle = pd.read_csv(obstacle_profile_pth)
        filenames = []
        for i in range(52,total_time_steps,int(delta_time/standard_delta)):
            print("time step: ", i)
            obstacles = []
            obs = ObstacleState()
            obs.x = data_obstacle['x'][i]
            obs.y = data_obstacle['y'][i]
            obs.yaw = data_obstacle['orientation'][i]
            obs.v = data_obstacle['velocity'][i]
            obs.a = data_obstacle['acceleration'][i]
            obstacles.append(obs)
        
            obs_pred_traj,all_pred_traj,all_prob,obs_pred_traj_sec = get_const_speed_obs_traj(obstacles,scene_path,time_step=i)
            # if i > 50 :
            #     cr_extractor.plot_pred(scenario,ego_state,all_pred_traj,all_prob,wx,wy,i)
            trajectories,confs,traj_dict,final_traj_type = solve_ffstreams_general(ego_state,obstacles,obs_pred_traj,wx,wy,lane_width)

            if len(trajectories) < 1:
                print("SOMETHING IS WRONG")
                if i > 50 :
                    # trajectories2,confs,traj_dict,final_traj_type = solve_ffstreams_general(ego_state,obstacles,obs_pred_traj_sec,wx,wy,lane_width)
                    trajectories2 = None
                    # cr_extractor.plot_pred(scenario,ego_state,obs_pred_traj,all_pred_traj,all_prob,wx,wy,i,ego_traj = None,trajectories2=trajectories2)
            
            else: 
                if i > 50 :
                    # trajectories2,confs,traj_dict,final_traj_type = solve_ffstreams_general(ego_state,obstacles,obs_pred_traj_sec,wx,wy,lane_width)
                    trajectories2 = None
                    cr_extractor.plot_pred(scenario,ego_state,obs_pred_traj,all_pred_traj,all_prob,wx,wy,i, ego_traj = trajectories[0],trajectories2=trajectories2)
                filename =  f'{i}.png'
                filenames.append(filename)
                ######## for statistics ##############
                stat.t.append(i*standard_delta)
                stat.x.append(ego_state.x)
                stat.y.append(ego_state.y)
                stat.v.append(ego_state.v)
                stat.a.append(ego_state.a)
                stat.yaw.append(ego_state.yaw)
                
                stat.s.append(ego_state.s)
                stat.v_s.append(ego_state.ds)
                stat.a_s.append(ego_state.dds)
                stat.j_s.append(ego_state.ddds)

                stat.l.append(ego_state.l)
                stat.v_l.append(ego_state.dl)
                stat.a_l.append(ego_state.ddl)
                stat.j_l.append(ego_state.dddl)
                stat.decisions.append(final_traj_type)
                
                stat.general_obs_x.append(obs.x)  
                stat.general_obs_y.append(obs.y) 
                stat.general_obs_yaw.append(obs.yaw) 
                stat.general_obs_v.append(obs.v) 
                stat.general_obs_a.append(obs.a) 
                ######################################

                new_ego_state = EgoState()
                
                # new_obs = []
                # for obs in future_trajectories_obstacles:
                #     new_obs.append(((obs[i][0], obs[i][1]), (5.5, 2.5), (obs[i][2], 0)))
                new_ego_state.yaw = trajectories[0].yaw[1]
                new_ego_state.x = trajectories[0].x[1]
                new_ego_state.y = trajectories[0].y[1]

                new_ego_state.s = trajectories[0].s[1]
                new_ego_state.l = trajectories[0].d[1]
                new_ego_state.ds = trajectories[0].s_d[1]
                new_ego_state.dl = trajectories[0].d_d[1]
                new_ego_state.ddl = trajectories[0].d_dd[1]
                new_ego_state.dds = trajectories[0].s_dd[1]
                new_ego_state.dddl = trajectories[0].d_ddd[1]
                new_ego_state.ddds = trajectories[0].s_ddd[1]

                new_ego_state.v = math.sqrt(new_ego_state.ds*new_ego_state.ds+new_ego_state.dl*new_ego_state.dl)
                new_ego_state.a = math.sqrt(new_ego_state.dds*new_ego_state.dds+new_ego_state.ddl*new_ego_state.ddl)
                ego_state = new_ego_state

            if ego_state.x < 325:
                success = True
                failed = False
                if success:
                    count_success +=1
                break
        stat.save_to_file(config['gif_path']+'exp_'+folder,exp,failed)
        # save gif
        timestr = config['gif_path']+ 'exp_'+folder+'/gif_exp_'+ str(exp) +'.gif'
        with imageio.get_writer(timestr, mode='I') as writer:
            for filename in filenames:
                image = imageio.imread(filename)
                writer.append_data(image)
                
        # Remove files
        for filename in set(filenames):
            os.remove(filename) 
    param_file = config['gif_path']+'exp_'+folder+'/statistics.txt'
    os.makedirs(os.path.dirname(param_file), exist_ok=True)

    f = open(param_file, "w")
    f.write("Successful experiments: %s\n" % count_success )
    f.write("Total experiments: %s\n" % counter_exp )
    f.close()

    


if __name__ == '__main__':
    main()