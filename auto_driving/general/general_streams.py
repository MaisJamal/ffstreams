import numpy as np
import ffstreams.utils.settings as stg
from ffstreams.ffstreams.frenet_optimizer_cr import FrenetPath,get_traj,get_traj_change_lane,get_traj_yield,get_traj_follow_speed,get_traj_change_lane_overtake
from ffstreams.ffstreams.frenet_optimizer_general import get_traj_follow_speed_general,get_traj_yield_general




def get_yield_general(ego_state,front_obstacle,wx,wy): # stream for generating YIELD trajectory  # TODO
    #q1,acc0,curr_dl,curr_ddl
    while True:
        b_x = front_obstacle.x
        b_y = front_obstacle.y
        b_speed = front_obstacle.v
        
        #b_speed = max(b_speed,q1[2]-2)
        if b_speed >  ego_state.v :
            v =  ego_state.v 
        else:
            v = max( ego_state.v-2.5,b_speed)
        v = max(v,0.00)
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