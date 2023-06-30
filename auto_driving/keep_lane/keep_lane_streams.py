from imaplib import Time2Internaldate
from os import terminal_size
import numpy as np
import utils.settings as stg
from ffstreams.frenet_optimizer import FrenetPath,get_traj,get_traj_change_lane,get_traj_yield,get_traj_follow_speed
import random
from random import choices
from numpy.linalg import norm
import time

ARRAY = np.array





def get_yield(q1,acc0,curr_dl,curr_ddl,front_obstacle): # stream for generating YIELD trajectory  # TODO
    global wy_middle_lower_lane
    while True:
        b_x = front_obstacle[0][0]
        b_y = front_obstacle[0][1]
        b_speed = front_obstacle[2][0] 
        dec_y = stg.wy_middle_lower_lane[0]
        dec_x = b_x - 3 * q1[2]  # q1---------*dec------*obs--q2  # delta_x = delta_t(3s) * v_q1
        dec_v = q1[2]
        
        thereIsTraj,traj = get_traj_yield(q1[0],q1[1],q1[2],acc0,curr_dl,curr_ddl,dec_x,dec_y,b_speed)
        
        if(thereIsTraj):
            yield (traj, )
            
        else:
            yield None


def get_follow_speed(q1,acc0,curr_dl,curr_ddl,desired_v): # stream for generating YIELD trajectory  # TODO
    global wy_middle_lower_lane
    while True:
        dec_y = stg.wy_middle_lower_lane[0]
        
        thereIsTraj,traj = get_traj_follow_speed(q1[0],q1[1],q1[2],acc0,curr_dl,curr_ddl,dec_y,desired_v)

        if(thereIsTraj):
            yield (traj, )
            
        else:
            yield None


def get_traj_change_gen(q1,acc0,curr_dl,curr_ddl,target_y,target_speed):#,tim1):
    
    global LANE_WIDTH
    approx_target_x = 400

    while True:
        x0 = q1[0]
        y0 = q1[1]
        speed0 = q1[2]

        target_x = q1[0] + approx_target_x
        sf = time.time()
        thereIsTraj,traj = get_traj_change_lane(x0,y0,speed0,acc0,curr_dl,curr_ddl,target_x,target_y,target_speed)
        print("****** time for getting the traj: ", time.time()-sf)
        if(thereIsTraj):
            final_traj = cut_traj(traj,target_y)
            yield (final_traj, )
        else:
            yield None


def cut_traj(long_traj,target_y):
    traj = FrenetPath()
    k_cut = len(long_traj.y)
    counter = 0
    for i in range(len(long_traj.x)):
        if abs(long_traj.y[i] - target_y) < 0.0001:
            counter += 1
            if counter > 1 :
                k_cut = i
                #print(k_cut)
                break


    if k_cut < len(long_traj.y):
        traj.x = long_traj.x[0:k_cut+1] 
        traj.y = long_traj.y[0:k_cut+1] 
        traj.s = long_traj.s[0:k_cut+1] 
        traj.s_d = long_traj.s_d[0:k_cut+1]  
        traj.s_dd = long_traj.s_dd[0:k_cut+1] 
        traj.s_ddd = long_traj.s_ddd[0:k_cut+1]  
        traj.d = long_traj.d[0:k_cut+1] 
        traj.d_d = long_traj.d_d[0:k_cut+1] 
        traj.d_dd = long_traj.d_dd[0:k_cut+1] 
        traj.d_ddd =  long_traj.d_ddd[0:k_cut+1] 
        traj.yaw = long_traj.yaw[0:k_cut+1] 

        traj.t = long_traj.t[0:k_cut+1] 

        return traj
    else:
        return long_traj