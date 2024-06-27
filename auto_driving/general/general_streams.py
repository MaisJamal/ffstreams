import numpy as np
import utils.settings as stg
from ffstreams.frenet_optimizer_cr import FrenetPath,get_traj,get_traj_change_lane,get_traj_yield,get_traj_follow_speed,get_traj_change_lane_overtake
from ffstreams.frenet_optimizer_general import get_traj_stop_at_intersection, get_traj_stop_general,get_traj_overtake_general,get_traj_follow_speed_general,get_traj_yield_general
import random
from utils.apollo_utils import EgoState

def combine(traj1,traj2):
    total_traj = FrenetPath()
    total_traj.x = traj1.x + traj2.x 
    total_traj.y = traj1.y + traj2.y
    total_traj.s = traj1.s + traj2.s
    total_traj.s_d = traj1.s_d + traj2.s_d
    total_traj.s_dd = traj1.s_dd + traj2.s_dd
    total_traj.s_ddd = traj1.s_ddd + traj2.s_ddd
    total_traj.d = traj1.d + traj2.d
    total_traj.d_d = traj1.d_d + traj2.d_d
    total_traj.d_dd = traj1.d_dd + traj2.d_dd
    total_traj.d_ddd =  traj1.d_ddd + traj2.d_ddd
    total_traj.yaw = traj1.yaw + traj2.yaw

    total_traj.t = traj1.t + traj2.t

    return total_traj

def extract_new_ego_state(traj):
    new_ego_state = EgoState()
    new_ego_state.t = traj.t[-1]
    new_ego_state.l = traj.d[-1]
    new_ego_state.dl = traj.d_d[-1]
    new_ego_state.ddl = traj.d_dd[-1]
    new_ego_state.dddl = traj.d_ddd[-1]
    new_ego_state.s = traj.s[-1]
    new_ego_state.ds = traj.s_d[-1]
    new_ego_state.dds = traj.s_dd[-1]
    new_ego_state.ddds = traj.s_ddd[-1]

    new_ego_state.x = traj.x[-1]
    new_ego_state.y = traj.y[-1]
    new_ego_state.yaw = 0
    new_ego_state.v = traj.s_d[-1]
    new_ego_state.a = traj.s_dd[-1]
    return new_ego_state

def get_stop_general(ego_state,wx,wy): # stream for generating YIELD trajectory  # TODO
    #q1,acc0,curr_dl,curr_ddl
    while True:
        Time_to_stop = 3
        v = max(0,ego_state.v-7)
        print(" stop target speed : ",v)
        thereIsTraj,traj = get_traj_stop_general(ego_state,v,Time_to_stop,wx,wy)
        # generate second traj
        if(thereIsTraj):
            Time_to_stop2 = 2
            new_ego_state = extract_new_ego_state(traj)
            thereIsTraj2,traj2 = get_traj_stop_general(new_ego_state,0,Time_to_stop2,wx,wy)
      
        if(thereIsTraj and thereIsTraj2):
            total_traj = combine(traj,traj2)
            yield (total_traj, )
            
        else:
            yield None


def get_stop_at_intersection(ego_state,dist_to_intersection,wx,wy): # stream for generating YIELD trajectory  # TODO
    #q1,acc0,curr_dl,curr_ddl
    while True:
        v =0
        print("intersection stop target speed : ",v)
        thereIsTraj,traj = get_traj_stop_at_intersection(ego_state,v,dist_to_intersection,wx,wy)

        if(thereIsTraj):
            yield (traj, )
            
        else:
            yield None

def get_yield_general(ego_state,target_speed,wx,wy): # stream for generating YIELD trajectory  # TODO
    #q1,acc0,curr_dl,curr_ddl
    while True:
        # b_x = front_obstacle.x
        # b_y = front_obstacle.y
        # b_speed = front_obstacle.v
        
        # #b_speed = max(b_speed,q1[2]-2)
        # if b_speed >  ego_state.v :
        #     v =  ego_state.v 
        # else:
        #     v = max( ego_state.v-2.5,b_speed)
        # v = max(v,0.00)
        v = max(target_speed,0.00)
        print(" yield target speed : ",v)
        thereIsTraj,traj = get_traj_yield_general(ego_state,v,wx,wy)
        
        if(thereIsTraj):
            yield (traj, )
            
        else:
            yield None


def get_follow_speed_general(ego_state,target_speed,wx,wy): # stream for generating YIELD trajectory  # TODO

    while True:
        
        thereIsTraj,traj = get_traj_follow_speed_general(ego_state,target_speed,wx,wy)

        if(thereIsTraj):
            yield (traj, )
            
        else:
            yield None


def get_overtake_general(ego_state,target_speed,wx,wy): # stream for generating YIELD trajectory  # TODO

    while True:
        
        thereIsTraj,traj = get_traj_overtake_general(ego_state,target_speed,wx,wy)

        if(thereIsTraj):
            yield (traj, )
            
        else:
            yield None

def get_yield_rand_general(ego_state,target_speed,wx,wy): # stream for generating YIELD trajectory  # TODO
    #q1,acc0,curr_dl,curr_ddl
    while True:
        v = max(target_speed,0.00)
        v = random.uniform(v, ego_state.v )
        print(" random yield target speed : ",v)
        thereIsTraj,traj = get_traj_yield_general(ego_state,v,wx,wy)
        
        if(thereIsTraj):
            yield (traj, )
            
        else:
            yield None


def get_follow_rand_general(ego_state,target_speed,wx,wy): # stream for generating YIELD trajectory  # TODO

    while True:
        target_speed = random.uniform(ego_state.v ,target_speed)
        print(" random follow target speed : ",target_speed)
        thereIsTraj,traj = get_traj_follow_speed_general(ego_state,target_speed,wx,wy)

        if(thereIsTraj):
            yield (traj, )
            
        else:
            yield None

def get_stop_rand_general(ego_state,wx,wy): # stream for generating YIELD trajectory  # TODO
    #q1,acc0,curr_dl,curr_ddl
    while True:
        Time_to_stop = 4
        v = max(0,ego_state.v-7)
        v = random.uniform(v, ego_state.v-5)
        v = max(0,v)
        print(" stop target speed : ",v)
        thereIsTraj,traj = get_traj_stop_general(ego_state,v,Time_to_stop,wx,wy)
        
        if(thereIsTraj):
            yield (traj, )
            
        else:
            yield None