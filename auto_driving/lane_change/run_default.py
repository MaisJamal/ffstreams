from __future__ import print_function

import os
from turtle import goto
from isort import file
#from this import d
import numpy as np
import subprocess
import re
import imageio.v2 as imageio
from random import choices
from metrics.OPM import calc_OPM_metric

from utils.viewer import get_rect

from matplotlib import gridspec
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.patches as mpatches
import random
from ffstreams.frenet_optimizer import generate_target_course,FrenetPath,get_traj
from auto_driving.lane_change.lane_change_streams import get_yield_change_gen,get_traj_change_gen,get_motion_gen ,get_connect_gen,get_yield_motion_gen,get_follow_motion_gen,get_second_connect_gen
import utils.settings as stg
import time
from utils.translator import translate_to_pddl_change_lane,translate_to_pddl_2_1, translate_to_pddlplus
from utils.statistics import Statistics

ARRAY = np.array

def extract_plan(string):
    q_from = []
    q_to = []
    idx = [m.start() for m in re.finditer('MOVE', string)]

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


def build_connectivity_graph(confs):
    ### build connectivity array of trajectories ###
    traj_dict = {}
    traj_array = np.full((len(confs),len(confs)), False, dtype=bool)
    for i in range(len(confs)):
        for j in range(len(confs)):
            if i != j :
                if (i,j) in traj_dict :
                    traj_array[i][j] = True
                else:
                    output = next(get_motion_gen(confs[i],confs[j]))
                    if output is not None:
                        traj_dict[(i,j)] = output[0]
                        traj_array[i][j] = True
    return traj_array, traj_dict


def plot_traj(traj,obstacles,obstacles_xs,obstacles_ys,dt,exp,folder):####################### plot and save plan as gif ###############
    save_gifs = True
    #folder = time.strftime("%Y%m%d-%H%M%S")
    plt.figure(figsize=(16, 5))


    filenames = []
    for i in range(len(traj.x)):
        
        filename = f'{i}.png'
        filenames.append(filename)
        gs = gridspec.GridSpec(1, 3, width_ratios=[3,1, 1]) 
        plt.subplot(gs[0])
        plt.cla()
        

        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        
        plt.plot(stg.tx, stg.ty,"y--",zorder = 0)
        plt.plot(stg.tx2, stg.ty2,"y--",zorder = 0)
        plt.plot(stg.tx3, stg.ty3,"y--",zorder = 0)
        plt.plot(stg.tx4, stg.ty4,"y--",zorder = 0)

        plt.plot(stg.txBound1, stg.tyBound1,"c",zorder = 0)
        plt.plot(stg.txBound2, stg.tyBound2,"c",zorder = 0)
        plt.plot(stg.txBound3, stg.tyBound3,"c",zorder = 0)
        plt.plot(stg.txBound4, stg.tyBound4,"c",zorder = 0)
        plt.plot(stg.txBound5, stg.tyBound5,"c",zorder = 0)
        
        plt.plot(traj.x, traj.y, "-r")

        for k in range(len(obstacles)):#plot obstacles
            obstacle_x = obstacles_xs[k][i]
            obstacle_y = obstacles_ys[k][i]
            rect = get_rect((obstacle_x,obstacle_y),obstacles[k][1]) # return object of the form [(left,bottom),x_extend,y_extend]
            rect_patch=mpatches.Rectangle(rect[0], rect[1], rect[2])#((31,15),14,7, fill = False,color = "purple",linewidth = 2)
            plt.gca().add_patch(rect_patch)
        #plt.add_patch(Rectangle((35, 50), 2, 6))
        #plt.plot(full_traj.x[i], full_traj.y[i], "vc")

        length = 5.5
        width = 2.5
        left = traj.x[i] - length/2
        bottom =  traj.y[i] - width/2
        if i < (len(traj.x) -1):
            ego_angle = traj.yaw[i] #math.atan2(traj.y[i+1] -traj.y[i] ,traj.x[i+1] - traj.x[i])/math.pi*180
        else: 
            ego_angle = 0.0
        ego_rect_patch=mpatches.Rectangle((left,bottom), length, width ,angle = ego_angle,color = "black")#((31,15),14,7, fill = False,color = "purple",linewidth = 2)
        plt.gca().add_patch(ego_rect_patch)

        #print("t: ", full_traj.t)
        plt.rcParams["figure.figsize"] = [7.00, 3.50]
        plt.xlim(0 , traj.x[-1])#plt.xlim(0 , stg.ENV_WIDTH)
        plt.ylim(0, 100)#plt.ylim(full_traj.y[0] - stg.area, full_traj.y[0] + stg.area)
        plt.title("v[km/h]:" + str(traj.s_d[i] * 3.6)[0:4])
        plt.grid(True)

        plt.subplot(gs[1])
        plt.plot(traj.x, [speed* 3.6 for speed in traj.s_d],"-r")
        #plt.plot(traj.x, [yaw*100 for yaw in traj.yaw],"-b")
        plt.plot(traj.x[i],traj.s_d[i]* 3.6, "vc")      
        plt.title("v[km/h]:" + str(traj.s_d[i] * 3.6)[0:4])

        plt.subplot(gs[2])
        plt.plot(traj.x, [acc for acc in traj.s_dd],"-r")   
        plt.plot(traj.x[i],traj.s_dd[i], "vc")    
        plt.title("a[m/s2]:" + str(traj.s_dd[i] )[0:4])
        # save frame
        if save_gifs:
            plt.savefig(filename)
        #plt.pause(0.1) 
    ################################### save parameters to a file ###############################
    dateAtime = time.strftime("%Y%m%d-%H%M%S")
    param_file = timestr = 'auto_driving/gifs/lane_change_gifs/single_exp'+folder+'/gif_exp_param_'+ str(exp) +'.txt'
    os.makedirs(os.path.dirname(param_file), exist_ok=True)
    f = open(param_file, "w")
    f.write("obstacles:\n")
    f.write("%s\n" % obstacles )
    #f.write("obstacles' accelerations:\n")
    #f.write("%s\n" % accel )
    f.close()
    ###############################################################################################    
    if save_gifs:    
        timestr = 'auto_driving/gifs/lane_change_gifs/single_exp'+folder+'/gif_exp_'+ str(exp) +'.gif'

        with imageio.get_writer(timestr, mode='I') as writer:
            for filename in filenames:
                image = imageio.imread(filename)
                writer.append_data(image)
                
        # Remove files
        for filename in set(filenames):
            os.remove(filename)       


def solve_pddl_lane_change(q0,acc0,curr_dl,curr_ddl,target_y, target_speed,obstacles):
    file_path = "auto_driving/lane_change/"
    problem_file = "problem.pddl"
    rand_x_goal = 20
    confs = []
    confs.append(q0)

    trajectories = []
    
    #ss1 = time.time()
    output = next(get_traj_change_gen(q0,acc0,curr_dl,curr_ddl,target_y,target_speed))
    #ss2 = time.time()
    #print("time for get_traj_change_gen", ss2-ss1)

    show_output = False  # important
    if show_output:
        dt = 0.2
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(stg.tx, stg.ty,"y--")
        plt.plot(stg.tx2, stg.ty2,"y--")
        plt.plot(stg.txBound1, stg.tyBound1,"c")
        plt.plot(stg.txBound2, stg.tyBound2,"c")
        plt.plot(stg.txBound3, stg.tyBound3,"c")
        for obs in obstacles:  #plot obstacles
            rect = get_rect((obs[0][0],obs[0][1]),obs[1]) # in the form [(left,bottom),x_extend,y_extend]
            rect_patch=mpatches.Rectangle(rect[0], rect[1], rect[2])#((31,15),14,7, fill = False,color = "purple",linewidth = 2)
            plt.gca().add_patch(rect_patch)
        plt.plot(output[0].x[0:], output[0].y[0:], "-r")
        plt.plot(output[0].x[0],output[0].y[0], "vc")
        plt.xlim(0, 800)
        area = 50
        plt.ylim(output[0].y[1] - area, output[0].y[1] + area)
        plt.title("v[m/s]:" + str(output[0].s_d[1] )[0:4] + "a[m/s2]:" + str(output[0].s_dd[1] )[0:4])
        plt.grid(True)
        plt.pause(0.1)
    ### build connectivity array of trajectories ###
    traj_dict = {}
    traj_array = np.full((2,2), False, dtype=bool)
    if output is not None:
        traj_dict[(0,1)] = output[0]
        traj_array[0][1] = True
        goal = ARRAY([output[0].x[-1],target_y, target_speed])
    else:
        goal = ARRAY([rand_x_goal,target_y, target_speed])
    print("Connectivity array between configurations:")
    print(traj_array)
    confs.append(goal)

    for i in range(len(confs)):
        print("q",i,":",confs[i])

    direction = "left"
    start = time.time()
    translate_to_pddl_change_lane(direction,confs,traj_dict,traj_array,obstacles,file_path,problem_file)
    print("time of printing: ",time.time() - start)

    start = time.time() 
    planner_path = os.getcwd() + "/ffplanner/ff"
    pddl_path = os.getcwd() + "/auto_driving/lane_change/"
    planner_output=subprocess.run([planner_path,"-p", pddl_path, "-o", "lane_change_domain.pddl", "-f" ,"problem.pddl","-s","3"],capture_output=True)
    print("excution time of FF planner: ",  time.time() - start)
    planner_output = str(planner_output)  
    q_from ,q_to = extract_plan(planner_output) 
    #ss3 = time.time()
    for i in range(len(q_from)):    
        if confs[q_to[i]][1] == stg.wy_middle_upper_lane[0] :
            overtake = True
        trajectories.append(traj_dict[(q_from[i],q_to[i])])
    #print("time for appending trajectories: ",time.time()-ss3)
    return trajectories,confs,traj_dict

def update_obstacles(obstacles,dt, accelerations,obs6_update_time,curr_time):
    obstacles_new = []
    i = 0
    for obs in obstacles:   
        b_x = obs[0][0]
        b_y = obs[0][1]
        b_speed = obs[2][0]
        b_x = b_x + b_speed * dt + 0.5 * accelerations[i] * dt * dt
        #((40, 48.25), (5.5, 2.5), (2, 0))
        b_speed = b_speed + accelerations[i] * dt
        ############# update last obstacle's lane
        if i == 5 and curr_time > obs6_update_time and b_y > 51.75:
            b_y -= 0.7
        ##############
        obstacles_new.append(((b_x,b_y),obs[1],(b_speed,0)))
        i+=1

    return obstacles_new

def main():
    lane_width = 3.5
    stg.init(lane_width)  

    counter_exp = 50
    statistics_arr = np.zeros(counter_exp)
    count_success = 0
    OPM_values = []
    folder = time.strftime("%Y%m%d-%H%M%S")


    ############ change to left lane problem ###############

    for exp in range(counter_exp):
        
        INIT_SPEED = 29 # m/s
        curr_time =  0.0
        dt = 0.2
        curr_dl = 0 # lateral speed
        curr_ddl = 0 # lateral acceleration
        acc0 = 0 #longitudinal acceleration
        all_y=[stg.wy_middle_lower_lane[0],stg.wy_middle_upper_lane[0],stg.wy_middle_lower_lower_lane[0]]
        q0 = ARRAY([0,all_y[0], INIT_SPEED])
        target_y = all_y[1]
        target_speed = 30  

        obsfront_x = random.uniform(50.0,65.0)
        obsfront_v = random.uniform(26.0,32.0)
        obsfront_a = 0.0

        obs2_x = random.uniform(-85.0,85.0)
        obs2_v = random.uniform(26.0,32.0)
        obs2_a = random.uniform(-3.0,3.0)
        
        obs3_x = random.uniform(-85.0,85.0)
        obs3_v = random.uniform(26.0,32.0)
        obs3_a = random.uniform(-3.0,3.0)
        
        obs4_x = random.uniform(-85.0,85.0)
        obs4_v = random.uniform(26.0,32.0)
        obs4_a = random.uniform(-3.0,3.0)
        
        obs5_x = random.uniform(-85.0,85.0)
        obs5_v = random.uniform(26.0,32.0)
        obs5_a = random.uniform(-3.0,3.0)
        
        obs6_x = random.uniform(-85.0,85.0)
        obs6_y = 55.25
        obs6_v = random.uniform(26.0,32.0)
        obs6_a = 0.0
        obs6_update_time = random.uniform(0.0,54.0)
        
        obstacles = [((obsfront_x, 48.25), (5.5, 2.5), (obsfront_v, 0)),
                    ((obs2_x, 51.75), (5.5, 2.5), (obs2_v, 0)),
                    ((obs3_x, 51.75), (5.5, 2.5), (obs3_v, 0)),
                    ((obs4_x, 51.75), (5.5, 2.5), (obs4_v, 0)),
                    ((obs5_x, 51.75), (5.5, 2.5), (obs5_v, 0)),
                    ((obs6_x, obs6_y),(5.5, 2.5), (obs6_v, 0))]
        
        accelerations = [obsfront_a , obs2_a, obs3_a , obs4_a, obs5_a, obs6_a]
        
        ####### for statistics ######################
        stat = Statistics()
        
        stat.t.append(curr_time)
        stat.yaw.append(0.0)
        stat.s = [q0[0]]
        stat.v_s = [q0[2]]
        stat.a_s = [0.0]
        stat.j_s = [0.0]

        stat.l = [q0[1]]
        stat.v_l = [curr_dl]
        stat.a_l = [curr_ddl]
        stat.j_l = [0.0]

        stat.front_obs_s = [obstacles[0][0][0]]
        stat.front_obs_v_s = [obstacles[0][2][0]]
        stat.front_obs_l = [obstacles[0][0][1]]
        

        stat.other_obs_no = len(obstacles) - 1
        for i in range(stat.other_obs_no):
            stat.other_obs_s.append([obstacles[i+1][0][0]])  #2d
            stat.other_obs_v_s.append([obstacles[i+1][2][0]]) #2d
            stat.other_obs_a_s.append(accelerations[i+1]) #1d
            stat.other_obs_l.append([obstacles[i+1][0][1]]) #2d

        ####### for ploting obstacles ###############
        obstacles_xs = {}
        obstacles_ys = {}
        for i in range(len(obstacles)):
            obstacles_xs[i] = [obstacles[i][0][0]]
            obstacles_ys[i] = [obstacles[i][0][1]]
        #############################################
        plan_found = False
        new_obs = obstacles
        final_traj = FrenetPath()
        ###
        final_traj.x = [q0[0]]
        final_traj.s = [q0[0]]
        final_traj.y = [q0[1]]
        final_traj.s_d = [q0[2]]
        final_traj.s_dd = [0]
        final_traj.yaw = [0]
        
        final_traj.s_ddd = [0]
        final_traj.d_dd = [0]
        final_traj.d_ddd = [0]
        #####
        reached_end = False
        total_excution_time = 0
        cycle_count = 0
        while curr_time <60.0 and not reached_end:
            start_time = time.time()
            print("current t: ", curr_time)
            confs = []
            confs.append(q0)

            #goal = ARRAY([output[0].x[-1],target_y, target_speed])
            s1 = time.time()
            trajectories,confs,traj_dict = solve_pddl_lane_change(q0,acc0,curr_dl,curr_ddl,target_y, target_speed,new_obs)
            s2 = time.time()
            print("time of func solve_pddl_lane_change is ", s2 - s1)
            if len(trajectories) < 1:
                print("no plan found")

                yield_traj = next(get_yield_change_gen(q0,0,0,0,new_obs[0]))
                s3 = time.time()
                print("time of func get_yiel: ",s3-s2)
                show_output = False # important
                if show_output:
                    plt.cla()
                    # for stopping simulation with the esc key.
                    plt.gcf().canvas.mpl_connect(
                        'key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                    plt.plot(stg.tx, stg.ty,"y--")
                    plt.plot(stg.tx2, stg.ty2,"y--")
                    plt.plot(stg.txBound1, stg.tyBound1,"c")
                    plt.plot(stg.txBound2, stg.tyBound2,"c")
                    plt.plot(stg.txBound3, stg.tyBound3,"c")
                    for obs in new_obs:  #plot obstacles
                        rect = get_rect((obs[0][0],obs[0][1]),obs[1]) # in the form [(left,bottom),x_extend,y_extend]
                        rect_patch=mpatches.Rectangle(rect[0], rect[1], rect[2])#((31,15),14,7, fill = False,color = "purple",linewidth = 2)
                        plt.gca().add_patch(rect_patch)
                    plt.plot(yield_traj[0].x[1:], yield_traj[0].y[1:], "-r")
                    plt.plot(yield_traj[0].x[1],yield_traj[0].y[1], "vc")
                    plt.xlim(0, 800)
                    area = 50
                    plt.ylim(yield_traj[0].y[1] - area, yield_traj[0].y[1] + area)
                    plt.title("v[m/s]:" + str(yield_traj[0].s_d[1] )[0:4] + "a[m/s2]:" + str(yield_traj[0].s_dd[1] )[0:4])
                    plt.grid(True)
                    plt.pause(0.1)

                final_traj.x = final_traj.x + [yield_traj[0].x[1]]
                final_traj.y = final_traj.y + [yield_traj[0].y[1]]
                final_traj.s_d = final_traj.s_d + [yield_traj[0].s_d[1]]
                final_traj.s_dd = final_traj.s_dd + [yield_traj[0].s_dd[1]]
                final_traj.yaw = final_traj.yaw + [yield_traj[0].yaw[1]]
                # for calculating OPM metric
                final_traj.s_ddd = final_traj.s_ddd + [yield_traj[0].s_ddd[1]]
                final_traj.d_dd = final_traj.d_dd + [yield_traj[0].d_dd[1]]
                final_traj.d_ddd = final_traj.d_ddd + [yield_traj[0].d_ddd[1]]
                #
                curr_dl = yield_traj[0].d_d[1]
                curr_ddl = yield_traj[0].d_dd[1]
                acc0 = yield_traj[0].s_dd[1]
                s4 = time.time()
                print("time of filling final traj: ",s4-s3)
                new_obs = update_obstacles(new_obs,dt,accelerations,obs6_update_time,curr_time)
                s5 = time.time()
                print("time of updating obstacles: ",s5-s4)
                q0[0] = final_traj.x[-1]
                q0[1] = final_traj.y[-1]
                q0[2] = final_traj.s_d[-1]
                         
            else:
                ### DEBUG ###
                #print("traj x: ", trajectories[0].x)
                #print("traj y: ", trajectories[0].y)
                #print("traj speed: ", trajectories[0].s_d)
                #print("traj acceleration: ", trajectories[0].s_dd)
                plan_found = True
                final_traj.x = final_traj.x + [trajectories[0].x[1]]
                final_traj.y = final_traj.y + [trajectories[0].y[1]]
                final_traj.s_d = final_traj.s_d + [trajectories[0].s_d[1]]
                final_traj.s_dd = final_traj.s_dd + [trajectories[0].s_dd[1]]
                final_traj.yaw = final_traj.yaw + [trajectories[0].yaw[1]]

                # for calculating OPM metric
                final_traj.s_ddd = final_traj.s_ddd + [trajectories[0].s_ddd[1]]
                final_traj.d_dd = final_traj.d_dd + [trajectories[0].d_dd[1]]
                final_traj.d_ddd = final_traj.d_ddd + [trajectories[0].d_ddd[1]]
                #
                curr_dl = trajectories[0].d_d[1]
                curr_ddl = trajectories[0].d_dd[1]
                acc0 = trajectories[0].s_dd[1]
                s6 = time.time()
                print("time of filling final traj: ",s6-s2)
                new_obs = update_obstacles(new_obs,dt,accelerations,obs6_update_time,curr_time)
                s7 = time.time()
                print("time of updating obstacles: ",s7-s6)
                q0[0] = final_traj.x[-1]
                q0[1] = final_traj.y[-1]
                q0[2] = final_traj.s_d[-1]
                
                
            #save obstacles xs and ys for ploting:
            for i in range(len(new_obs)):
                obstacles_xs[i].append(new_obs[i][0][0])
                obstacles_ys[i].append(new_obs[i][0][1])

            ##################
            curr_time += 0.2 
            excution_t = time.time() - start_time
            total_excution_time += excution_t  
            cycle_count +=1

            ####### for statistics ######################            
            stat.t.append(curr_time)
            stat.yaw.append(final_traj.yaw[-1])
            
            stat.s.append(final_traj.x[-1])
            stat.v_s.append(final_traj.s_d[-1])
            stat.a_s.append(final_traj.s_dd[-1])
            stat.j_s.append(final_traj.s_ddd[-1])

            stat.l.append(final_traj.y[-1])
            stat.v_l.append(curr_dl)
            stat.a_l.append(curr_ddl)
            stat.j_l.append(final_traj.d_ddd[-1])

            stat.front_obs_s.append(new_obs[0][0][0])
            stat.front_obs_v_s.append(new_obs[0][2][0])
            stat.front_obs_l.append(new_obs[0][0][1])
        
            for i in range(stat.other_obs_no):
                stat.other_obs_s[i].append(new_obs[i+1][0][0])  #2d
                stat.other_obs_v_s[i].append(new_obs[i+1][2][0]) #2d
                stat.other_obs_l[i].append(new_obs[i+1][0][1]) #2d
            ###########################################

            if abs(final_traj.y[-1] - (all_y[1]) ) < 0.01:
                reached_end = True
                statistics_arr[exp] = total_excution_time /cycle_count
                count_success += 1 

        #print(len(final_traj.s_dd),len(final_traj.d_dd),len(final_traj.s_ddd),len(final_traj.s_ddd))    
        OPM_values.append(calc_OPM_metric(final_traj))
        stat.save_to_file('auto_driving/gifs/lane_change_gifs/single_exp'+folder,exp)
        plot_traj(final_traj,obstacles,obstacles_xs,obstacles_ys,dt,exp,folder)# trajectories[0]
    param_file = 'auto_driving/gifs/lane_change_gifs/single_exp'+folder+'/statistics.txt'
    os.makedirs(os.path.dirname(param_file), exist_ok=True)

    f = open(param_file, "w")
    f.write("Successful experiments: %s\n" % count_success )
    f.write("Total experiments: %s\n" % counter_exp )
    f.write("Statistics array: %s\n" % statistics_arr)
    f.write("OPM metric: %s\n" % OPM_values)
    f.close()
    
    #input("end of exp?")

   
   

if __name__ == '__main__':
    main()