import numpy as np
import matplotlib.pyplot as plt
from ffstreams.utils.viewer import get_rect
import matplotlib.patches as mpatches
import os
import subprocess
import re

from ffstreams.auto_driving.apollo.apollo_streams import get_yield, get_follow_speed
from ffstreams.auto_driving.general.general_streams import get_overtake_general,get_stop_general,get_yield_general, get_stop_at_intersection,get_follow_speed_general,get_stop_rand_general,get_yield_rand_general, get_follow_rand_general

import ffstreams.utils.apollo_config as cfg
from ffstreams.utils.apollo_utils import extract_front_obstacle

from ffstreams.utils.translator import translate_to_pddl_apollo,translate_to_pddl_cr
from ffstreams.ffstreams.frenet_optimizer_cr import FrenetPath

from ffstreams.ffstreams.frenet_optimizer_general import generate_target_course

import yaml
with open('ffstreams/config/config.yml', 'r') as file:
    config = yaml.safe_load(file)
config = config['general']


ARRAY = np.array



def solve_ffstreams(ego_state,obstacles):
    goal_left = False
    q0 = ARRAY([ego_state.s,ego_state.l,ego_state.v])
    acc0 = ego_state.a
    curr_dl = ego_state.dl
    curr_ddl = ego_state.ddl
    
    # for connectivity graph
    confs = []
    conf_num = 1
    traj_dict = {} 
    traj_array = np.full((10,10), False, dtype=bool) # for connectivity graph
    traj_type = {} # for connectivity graph
    confs.append(q0)

    trajectories = []
    # call available applicable streams
    const_dv = 2.5
    target_speed =min(cfg.MAX_SPEED,q0[2]+const_dv)
    print(" follow target speed : ",target_speed)
    follow_output = next(get_follow_speed(q0,acc0,curr_dl,curr_ddl,target_speed))
    if follow_output is not None:
        q = ARRAY([follow_output[0].x[-1],follow_output[0].y[-1],follow_output[0].s_d[-1]])
        confs.append(q)
        traj_dict[(0,conf_num)] = follow_output[0]
        traj_array[0][conf_num] = True
        traj_type[(0,conf_num)] = "FOLLOW"
        conf_num +=1
    front_obs_idx = extract_front_obstacle(obstacles,q0)    
    yield_output = next(get_yield(q0,acc0,curr_dl,curr_ddl,obstacles[front_obs_idx])) #TODO add condition of existing front obstacle
    if yield_output is not None:
        q = ARRAY([yield_output[0].x[-1],yield_output[0].y[-1],yield_output[0].s_d[-1]])
        confs.append(q)
        traj_dict[(0,conf_num)] = yield_output[0]
        traj_array[0][conf_num] = True
        traj_type[(0,conf_num)] = "YIELD"
        conf_num +=1
    plot_candidate_trajectories(follow_output,yield_output,q0,obstacles)

    """ # debug
    print("Connectivity array between configurations:")
    print(traj_array)
    for i in range(len(confs)):
        print("q",i,":",confs[i])
    """
    there_is_front_obs = False
    translate_to_pddl_apollo(goal_left,there_is_front_obs,confs,traj_dict,traj_type,traj_array,obstacles,cfg.FILE_PATH,cfg.PROBLEM_FILE)
    planner_path = os.getcwd() + "/ffstreams/ffplanner/ff"
    pddl_path = os.getcwd() + "/"+ cfg.FILE_PATH
    planner_output=subprocess.run([planner_path,"-p", pddl_path, "-o", "apollo_domain.pddl", "-f" ,"problem.pddl","-s","3"],capture_output=True)

    planner_output = str(planner_output)  
    q_from ,q_to = extract_plan(planner_output) 
    #ss3 = time.time()
    final_traj_type = "NONE"
    for i in range(len(q_from)):    
        if confs[q_to[i]][1] == cfg.wy_middle_lane_center[0] :
            overtake = True
        trajectories.append(traj_dict[(q_from[i],q_to[i])])
        final_traj_type = traj_type[(q_from[i],q_to[i])]

    return trajectories,confs,traj_dict,final_traj_type



def plot_candidate_trajectories(follow_output,yield_output,q0,obstacles):
    show_output = True # important2
    if show_output:
        dt = 0.2
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(cfg.tx, cfg.ty,"y--")
        plt.plot(cfg.tx2, cfg.ty2,"y--")
        plt.plot(cfg.txBound1, cfg.tyBound1,"c")
        plt.plot(cfg.txBound2, cfg.tyBound2,"c")
        plt.plot(cfg.txBound3, cfg.tyBound3,"c")
        for obs in obstacles:  #plot obstacles
            rect = get_rect((obs.s,obs.l),obs.shape) # in the form [(left,bottom),x_extend,y_extend]
            rect_patch=mpatches.Rectangle(rect[0], rect[1], rect[2])#((31,15),14,7, fill = False,color = "purple",linewidth = 2)
            plt.gca().add_patch(rect_patch)
        if follow_output is not None:
            plt.plot(follow_output[0].x[0:], follow_output[0].y[0:], "-g")
            plt.plot(follow_output[0].x[0],follow_output[0].y[0], "vc")
        if yield_output is not None:
            plt.plot(yield_output[0].x[0:], yield_output[0].y[0:], "-r")
            plt.plot(yield_output[0].x[0],yield_output[0].y[0], "vc")
        #if LANE_CHANGE and change_left_output is not None:
        #    plt.plot(change_left_output[0].x[0:], change_left_output[0].y[0:], "-b")
        #    plt.plot(change_left_output[0].x[0],change_left_output[0].y[0], "vc")
        #if len(full_traj.x) >0: 
        #    print(len(full_traj.x),len(full_traj.y))
        #    plt.plot(full_traj.x[0:], full_traj.y[0:], "-b")
        #    plt.plot(full_traj.x[0],full_traj.y[0], "vc")

        #    for obs in obstacles:  #plot obstacles
        #        print("final t ",full_traj.t[-1])
        #        rect = get_rect((obs.x+obs[2][0]*full_traj.t[-1],obs[0][1]),obs[1]) # in the form [(left,bottom),x_extend,y_extend]
        #        rect_patch=mpatches.Rectangle(rect[0], rect[1], rect[2])#((31,15),14,7, fill = False,color = "purple",linewidth = 2)
        #        plt.gca().add_patch(rect_patch)
        #if overtake_decision:
        #    plt.plot(overtake_traj.x[0:], overtake_traj.y[0:], "-b")
        #    plt.plot(overtake_traj.x[0],overtake_traj.y[0], "vc")
        plt.xlim(0, 100)
        plt.ylim(-5,5)
        plt.grid(True)
        plt.pause(0.1)


def extract_plan(string):
    q_from = []
    q_to = []
    idx = [m.start() for m in re.finditer('LEFT_CHANGE', string)]
    
    if not idx:
        idx = [m.start() for m in re.finditer('KEEP_SPEED', string)]
    
    if not idx:
        idx = [m.start() for m in re.finditer('KEEP_LANE_YIELD', string)]

    if not idx:
        idx = [m.start() for m in re.finditer('OVERTAKE', string)]
    
    if not idx:
        idx = [m.start() for m in re.finditer('STOP', string)]

    for i in idx:
        k = string[i:]
        #print(k.split()[1])
        #print(k.split()[2])
        
        q_from_1 = k.split()[1]
        q_from_1 = int(q_from_1[1:])

        q_to_1 = k.split()[2]
        idx_rn = [m.start() for m in re.finditer('n', q_to_1)]
        #print(q_to)
        q_to_1 = q_to_1[0:idx_rn[0]-1]
        q_to_1 = int(q_to_1[1:])
        
        print(k.split()[0],"  ",q_from_1, "  ",q_to_1)
        q_from.append(q_from_1)
        q_to.append(q_to_1)
    

    return q_from,q_to



def solve_ffstreams_general(ego_state,obstacles,obs_pred_traj,wx,wy,lane_width,extra_candidates = 0,dist_to_intersection = None,intersection = True,overtake_decision=False,overtake_counter=0,overtake_traj = None,overtake_needed=False):
    goal_left = False
    max_speed = config['max_speed']
    const_dv = config['const_dv']
    file_path = config['path']
    problem_file = "problem.pddl"
    q0 = ARRAY([ego_state.x,ego_state.y,ego_state.ds])
    acc0 = ego_state.dds
    curr_dl = ego_state.dl
    curr_ddl = ego_state.ddl
    
    # for connectivity graph
    confs = []
    conf_num = 1
    traj_dict = {} 
    traj_array = np.full((100,100), False, dtype=bool) # for connectivity graph
    traj_type = {} # for connectivity graph
    confs.append(q0)

    trajectories = []
    # call available applicable streams
    
    target_speed =min(max_speed,q0[2]+const_dv)
    print(" follow target speed : ",target_speed)
    follow_output = next(get_follow_speed_general(ego_state,target_speed,wx,wy))
    if follow_output is not None:
        print("Follow candidate exists")
        q = ARRAY([follow_output[0].x[-1],follow_output[0].y[-1],follow_output[0].s_d[-1]])
        confs.append(q)
        traj_dict[(0,conf_num)] = follow_output[0]
        traj_array[0][conf_num] = True
        traj_type[(0,conf_num)] = "FOLLOW"
        conf_num +=1
    # front_obs_idx = extract_front_obstacle(obstacles,q0)   
    front_obs_idx = 0 
    target_speed =max(0,q0[2]-(const_dv))
    yield_output = next(get_yield_general(ego_state,target_speed,wx,wy)) #TODO add condition of existing front obstacle
    if yield_output is not None:
        print("Yield candidate exists")
        q = ARRAY([yield_output[0].x[-1],yield_output[0].y[-1],yield_output[0].s_d[-1]])
        confs.append(q)
        traj_dict[(0,conf_num)] = yield_output[0]
        traj_array[0][conf_num] = True
        traj_type[(0,conf_num)] = "YIELD"
        conf_num +=1

    # stop_output = next(get_stop_general(ego_state,wx,wy)) #TODO add condition of existing front obstacle
    # # stop_output = None
    # if stop_output is not None:
    #     print("Stop candidate exists")
    #     q = ARRAY([stop_output[0].x[-1],stop_output[0].y[-1],stop_output[0].s_d[-1]])
    #     confs.append(q)
    #     traj_dict[(0,conf_num)] = stop_output[0]
    #     traj_array[0][conf_num] = True
    #     traj_type[(0,conf_num)] = "STOP"
    #     conf_num +=1
    # stop_at_intersection_output = None
    # if dist_to_intersection is not None:
    stop_at_intersection_output = None
    overtake_output = None
    if intersection:
        stop_at_intersection_output = next(get_stop_at_intersection(ego_state,dist_to_intersection,wx,wy)) #TODO add condition of existing front obstacle
        if stop_at_intersection_output is not None:
            print("Stop candidate exists")
            q = ARRAY([stop_at_intersection_output[0].x[-1],stop_at_intersection_output[0].y[-1],stop_at_intersection_output[0].s_d[-1]])
            confs.append(q)
            traj_dict[(0,conf_num)] = stop_at_intersection_output[0]
            traj_array[0][conf_num] = True
            traj_type[(0,conf_num)] = "STOP"
            conf_num +=1
    elif overtake_needed:
        if not overtake_decision:
            target_speed =min(max_speed,q0[2]+const_dv)
            print(" overtake target speed : ",target_speed)
            overtake_output = next(get_overtake_general(ego_state,target_speed,wx,wy))
            if overtake_output is not None:
                print("Follow candidate exists")
                q = ARRAY([overtake_output[0].x[-1],overtake_output[0].y[-1],overtake_output[0].s_d[-1]])
                confs.append(q)
                traj_dict[(0,conf_num)] = overtake_output[0]
                traj_array[0][conf_num] = True
                traj_type[(0,conf_num)] = "OVERTAKE"
                conf_num +=1
        else:
            delta_t = 0.2
            overtake_output = FrenetPath()
            overtake_output.t = [i-(delta_t*overtake_counter) for i in overtake_traj.t[overtake_counter:]] 
            overtake_output.x = overtake_traj.x[overtake_counter:]
            overtake_output.y = overtake_traj.y[overtake_counter:]
            overtake_output.s = overtake_traj.s[overtake_counter:]
            overtake_output.s_d = overtake_traj.s_d[overtake_counter:]
            overtake_output.s_dd = overtake_traj.s_dd[overtake_counter:]
            overtake_output.s_ddd = overtake_traj.s_ddd[overtake_counter:]
            overtake_output.d = overtake_traj.d[overtake_counter:]
            overtake_output.d_d = overtake_traj.d_d[overtake_counter:]
            overtake_output.d_dd = overtake_traj.d_dd[overtake_counter:]
            overtake_output.d_ddd = overtake_traj.d_ddd[overtake_counter:]
            overtake_output.yaw = overtake_traj.yaw[overtake_counter:]
            overtake_output = [overtake_output]
            q_end = ARRAY([overtake_output[0].x[-1],overtake_output[0].y[-1],overtake_output[0].s_d[-1]])
            confs.append(q_end)
            
            traj_dict[(0,conf_num)] = overtake_output[0]#change_right_output[0]
            traj_array[0][conf_num] = True
            traj_type[(0,conf_num)] = "OVERTAKE"
            conf_num +=1
    ############### generate more candidate trajectories #######
    
    for extra in range(extra_candidates):
        target_speed =min(max_speed,q0[2]+(const_dv*2))
        follow_output = next(get_follow_rand_general(ego_state,target_speed,wx,wy))
        if follow_output is not None:
            print("Random Follow candidate exists")
            q = ARRAY([follow_output[0].x[-1],follow_output[0].y[-1],follow_output[0].s_d[-1]])
            confs.append(q)
            traj_dict[(0,conf_num)] = follow_output[0]
            traj_array[0][conf_num] = True
            traj_type[(0,conf_num)] = "FOLLOW"
            conf_num +=1
        front_obs_idx = extract_front_obstacle(obstacles,q0)   
        target_speed =max(0,q0[2]-(const_dv*2))
        yield_output = next(get_yield_rand_general(ego_state,target_speed,wx,wy)) #TODO add condition of existing front obstacle
        if yield_output is not None:
            print("Random Yield candidate exists")
            q = ARRAY([yield_output[0].x[-1],yield_output[0].y[-1],yield_output[0].s_d[-1]])
            confs.append(q)
            traj_dict[(0,conf_num)] = yield_output[0]
            traj_array[0][conf_num] = True
            traj_type[(0,conf_num)] = "YIELD"
            conf_num +=1

        stop_output = next(get_stop_rand_general(ego_state,wx,wy)) #TODO add condition of existing front obstacle
        # stop_output = None
        if stop_output is not None:
            print("Stop candidate exists")
            q = ARRAY([stop_output[0].x[-1],stop_output[0].y[-1],stop_output[0].s_d[-1]])
            confs.append(q)
            traj_dict[(0,conf_num)] = stop_output[0]
            traj_array[0][conf_num] = True
            traj_type[(0,conf_num)] = "STOP"
            conf_num +=1

    #############################################################
    
    
    
    plot_candidate_trajectories_general(follow_output,yield_output,stop_at_intersection_output,overtake_output,ego_state,obstacles,wx,wy)

    # """ # debug
    # print("Connectivity array between configurations:")
    # print(traj_array)
    # for i in range(len(confs)):
    #     print("q",i,":",confs[i])
    # """
    there_is_front_obs = False
    translate_to_pddl_cr(goal_left,there_is_front_obs,confs,traj_dict,traj_type,traj_array,obstacles,file_path,problem_file,obs_pred_traj,dist_to_intersection)
    planner_path = os.getcwd() + "/ffstreams/ffplanner/ff"
    pddl_path = os.getcwd() + "/"+config['path']
    planner_output=subprocess.run([planner_path,"-p", pddl_path, "-o", "general_domain.pddl", "-f" ,"problem.pddl","-s","3"],capture_output=True)

    planner_output = str(planner_output)  
    q_from ,q_to = extract_plan(planner_output) 
    # #ss3 = time.time()
    final_traj_type = "NONE"
    for i in range(len(q_from)):    
    #     if confs[q_to[i]][1] == cfg.wy_middle_lane_center[0] :
    #         overtake = True
        trajectories.append(traj_dict[(q_from[i],q_to[i])])
        final_traj_type = traj_type[(q_from[i],q_to[i])]
        if final_traj_type == "OVERTAKE":
            overtake_decision = True

    return trajectories,confs,traj_dict,final_traj_type,overtake_decision



def plot_candidate_trajectories_general(follow_output,yield_output,stop_output,overtake_output,ego_state,obstacles,wx,wy):
    show_output = True # important2
    if show_output:
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

        plt.plot(tx, ty,"y--")

        # for obs in obstacles:  #plot obstacles
        #     rect = get_rect((obs.s,obs.l),obs.shape) # in the form [(left,bottom),x_extend,y_extend]
        #     rect_patch=mpatches.Rectangle(rect[0], rect[1], rect[2])#((31,15),14,7, fill = False,color = "purple",linewidth = 2)
        #     plt.gca().add_patch(rect_patch)
        if follow_output is not None:
            plt.plot(follow_output[0].x[0:], follow_output[0].y[0:], "-g")
            plt.plot(follow_output[0].x[0],follow_output[0].y[0], "vc")
        if yield_output is not None:
            plt.plot(yield_output[0].x[0:], yield_output[0].y[0:], "-r")
            plt.plot(yield_output[0].x[0],yield_output[0].y[0], "vc")
        if stop_output is not None:
            plt.plot(stop_output[0].x[0:], stop_output[0].y[0:], "-b")
            plt.plot(stop_output[0].x[0],stop_output[0].y[0], "vc")
        if overtake_output is not None:
            plt.plot(overtake_output[0].x[0:], overtake_output[0].y[0:], "-b")
            plt.plot(overtake_output[0].x[0],overtake_output[0].y[0], "vc")
        # plt.xlim(0, 100)
        # plt.ylim(-5,5)
        plt.grid(True)
        plt.pause(0.1)