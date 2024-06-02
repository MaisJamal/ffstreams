from imaplib import Time2Internaldate
import numpy as np

import ffstreams.utils.apollo_config as cfg
from ffstreams.ffstreams.frenet_optimizer_apollo import get_traj_yield, get_traj_follow_speed

ARRAY = np.array



def get_yield(q1,acc0,curr_dl,curr_ddl,front_obstacle): # stream for generating YIELD trajectory  # TODO

    while True:
        b_x = front_obstacle.s
        b_y = front_obstacle.l
        b_speed = front_obstacle.v 
        dec_y = cfg.wy_middle_lane_center[0]
        dec_x = b_x - 3 * q1[2]  # q1---------*dec------*obs--q2  # delta_x = delta_t(3s) * v_q1
        
        dec_v = q1[2]
        #b_speed = max(b_speed,q1[2]-2)
        if b_speed > q1[2] :
            v = q1[2] 
        else:
            v = max(q1[2]-2.5,b_speed)
        v = max(v,0.00)
        print("end v for yield ", v, "  , initial speed ",q1[2]," , front obs speed : ",b_speed)
        thereIsTraj,traj = get_traj_yield(q1[0],q1[1],q1[2],acc0,curr_dl,curr_ddl,dec_y,v)
        
        if(thereIsTraj):
            yield (traj, )
            
        else:
            yield None


def get_follow_speed(q1,acc0,curr_dl,curr_ddl,desired_v): # stream for generating YIELD trajectory  # TODO

    while True:
        dec_y = cfg.wy_middle_lane_center[0]
        
        thereIsTraj,traj = get_traj_follow_speed(q1[0],q1[1],q1[2],acc0,curr_dl,curr_ddl,dec_y,desired_v)

        if(thereIsTraj):
            yield (traj, )
            
        else:
            yield None


