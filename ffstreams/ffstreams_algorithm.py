import numpy as np
import matplotlib.pyplot as plt
from utils.viewer import get_rect
import matplotlib.patches as mpatches
import os
import subprocess
import re

from auto_driving.apollo.apollo_streams import get_yield, get_follow_speed
import utils.apollo_config as cfg
from utils.apollo_utils import extract_front_obstacle

from utils.translator import translate_to_pddl_apollo

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
    #plot_candidate_trajectories(follow_output,yield_output,q0,obstacles)


    print("Connectivity array between configurations:")
    print(traj_array)
    for i in range(len(confs)):
        print("q",i,":",confs[i])
    
    there_is_front_obs = False
    translate_to_pddl_apollo(goal_left,there_is_front_obs,confs,traj_dict,traj_type,traj_array,obstacles,cfg.FILE_PATH,cfg.PROBLEM_FILE)
    planner_path = os.getcwd() + "/ffplanner/ff"
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
            rect = get_rect((obs.x,obs.y),obs.shape) # in the form [(left,bottom),x_extend,y_extend]
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
