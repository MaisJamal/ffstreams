from imaplib import Time2Internaldate
from os import terminal_size
import numpy as np
import utils.settings as stg
from ffstreams.frenet_optimizer import FrenetPath,get_traj,get_traj_change_lane,get_traj_yield
import random
from random import choices
from numpy.linalg import norm
import time

ARRAY = np.array # No hashing
#ARRAY = list # No hashing
#ARRAY = tuple # Hashing

dt = 0.2

LastT = 0

#####
#D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
#N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 2.0  # robot radius [m]

BASE_CONSTANT = 1
BASE_VELOCITY = 0.25

def get_difference(p1, p2):
    assert len(p1) == len(p2)
    return np.array(p2) - np.array(p1)

def get_length(vec, norm=2):
    return np.linalg.norm(vec, ord=norm)

def get_distance(p1, p2, **kwargs):
    return get_length(get_difference(p1, p2), **kwargs)
"""
def distance_fn(q1, q2):
    distance = get_distance(q1.values[:2], q2.values[:2])
    return BASE_CONSTANT + distance / BASE_VELOCITY
"""

def move_cost_fn(traj):
    #q1 = np.concatenate(([traj.x[0]], [traj.y[0]]), axis=0) 
    #q2 = np.concatenate(([traj.x[-1]], [traj.y[-1]]), axis=0) 
    global LastT 
    LastT= traj.t[-1]
    #print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLast T is ",LastT)
    return (traj.t[-1]-traj.t[0])#get_distance(q1,q2)


#######################################################
def get_obstacle_update_test(b):
    #print("UUUUUUUUUUUUUUUUUUUpdated obstacle: ",b)
    return True
#######################################################

def get_cfree_test(traj,b,p): # obstacle: obstacles[0][0] is of the form (x,y)
    #def test(t,b,p):
        # I suppost that always exists a trajectory between q1 and q2 so t is an array [q1,q2] and q of the form: ARRAY([x,y,speed])
      #  if not collisions :
         #   return True
    start_time = time.time()
    b_x = p[0]#b[0][0]
    b_y = p[1]#b[0][1]
    b_speed = b[2][0]
    #print("*********************obstacle in test: ", b_x,b_y,"**********************************")
    """
    if b_speed == 0:
        coord_b = np.concatenate(([b_x], [b_y]), axis=0) 
        coord_q1 = np.concatenate(([traj.x[0]], [traj.y[0]]), axis=0)  
        coord_q2 = np.concatenate(([traj.x[-1]], [traj.y[-1]]), axis=0) 
        #print("uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu")
        #print("traj.y[-1]:  ", traj.y[-1], "   b_y:  " ,b_y)
        if coord_q1[0]-2.5<= b_x <=coord_q2[0]+2.5:
            if abs(get_distance(coord_q1,coord_b) + get_distance(coord_b,coord_q2) - get_distance(coord_q1,coord_q2))<0.0001 :
                #print(coord_q1,coord_q2,p)
                #print("there is a collision")
                return False
            elif traj.y[-1]==b_y and traj.y[0]!=b_y:
                for i in range(len(traj.x)):
                    #print("traj.y ",i, " ",traj.y[i],"traj.x[i] ",traj.x[i])
                    if abs(traj.y[i]-b_y) < 0.5:
                        if traj.x[i]<b_x+2.5:
                            #print("****************false*******************")
                            return False
                        break
    else:
    """
    #start_time = time.time()
    for i in range(len(traj.t)):
        b_x_next = b_x + traj.t[i]*b[2][0]
        #print("b_x: ",b_x_next ," traj_x: " ,traj.x[i]," traj_t: ",traj.t[i] )
        if abs(b_x_next-traj.x[i])<=2.0 and abs(b_y-traj.y[i])<=1.5 :
            #print("COLLISION")
            print("cfree was called. the time needed: ", time.time()-start_time)
            return False
    print("cfree was called. the time needed: ", time.time()-start_time)
    return True  

def get_connect_gen(q1,weights):
    global ENV_WIDTH
    global LANE_WIDTH 
    global MAX_SPEED
    global TARGET_SPEED
    global wy_middle_lower_lane
    global wy_middle_upper_lane
   
    while True:
        
        all_y=[stg.wy_middle_lower_lane[0],stg.wy_middle_upper_lane[0],stg.wy_middle_lower_lower_lane[0]]
        #weights = [0.5 , 0.5]
        #rand_x = np.random.randint(min(q1[0]+5,stg.ENV_WIDTH-1),stg.ENV_WIDTH)
        

        rand_y = choices(all_y,weights)[0]  #random.choice(all_y)#np.random.randint((stg.ENV_WIDTH/4 - stg.LANE_WIDTH+0.5)*10  , (stg.ENV_WIDTH/4 + stg.LANE_WIDTH-0.8)*10 )/10.0
        #rand_y = stg.wy_middle_upper_lane[0]
        
        if rand_y == stg.wy_middle_lower_lane[0] :
            rand_x = q1[0] + np.random.randint(60,110)  #TODO dont harcode it
            rand_speed = random.uniform(1, 3)
            q2 = ARRAY([rand_x,rand_y,q1[2]+rand_speed])
            #q2 = np.concatenate((q2,[ min(3,q1[2]-rand_speed) ]),axis=0)
            #q2 = np.concatenate((q2,[rand_speed]),axis=0)
        else:
            #rand_x = q1[0] + np.random.randint(30,80) # (10,110)
            rand_x = q1[0] + np.random.randint(40,60) # for one experiment when second obstacle speed = -2
            rand_speed = random.uniform(1, 6)
            q2 = ARRAY([rand_x,rand_y,q1[2]+rand_speed])
            #q2 = np.concatenate((q2,[q1[2]+rand_speed]),axis=0)
       
        yield (q2,)

def get_second_connect_gen(q1,weights):
    global ENV_WIDTH
    global LANE_WIDTH 
    global MAX_SPEED
    global TARGET_SPEED
    global wy_middle_lower_lane
    global wy_middle_upper_lane
   
    while True:
        
        all_y=[stg.wy_middle_lower_lane[0],stg.wy_middle_upper_lane[0],stg.wy_middle_lower_lower_lane[0]]
        #weights = [0.5 , 0.5]
        #rand_x = np.random.randint(min(q1[0]+5,stg.ENV_WIDTH-1),stg.ENV_WIDTH)
        

        rand_y = choices(all_y,weights)[0]  #random.choice(all_y)#np.random.randint((stg.ENV_WIDTH/4 - stg.LANE_WIDTH+0.5)*10  , (stg.ENV_WIDTH/4 + stg.LANE_WIDTH-0.8)*10 )/10.0
        #rand_y = stg.wy_middle_upper_lane[0]
        
        if rand_y == stg.wy_middle_lower_lane[0] :
            rand_x = q1[0] + np.random.randint(60,110)  #TODO dont harcode it
            rand_speed = random.uniform(1, 3)
            q2 = ARRAY([rand_x,rand_y,q1[2]+rand_speed])
            #q2 = np.concatenate((q2,[ min(3,q1[2]-rand_speed) ]),axis=0)
            #q2 = np.concatenate((q2,[rand_speed]),axis=0)
        else:
            if q1[2] < 3 :
                rand_x = q1[0] + np.random.randint(30,40) # (10,110)
                rand_speed = random.uniform(4.5, 5.5)
                q2 = ARRAY([rand_x,rand_y,q1[2]+rand_speed])
            elif q1[2] < 3.5 :
                rand_x = q1[0] + np.random.randint(35,40) # (10,110)
                rand_speed = random.uniform(4.5, 5.5)
                q2 = ARRAY([rand_x,rand_y,q1[2]+rand_speed])
            elif q1[2] < 4.5 :
                rand_x = q1[0] + np.random.randint(40,50) # (10,110)
                rand_speed = random.uniform(5.0, 6.0)
                q2 = ARRAY([rand_x,rand_y,q1[2]+rand_speed])
            elif q1[2] < 5.5 :
                rand_x = q1[0] + np.random.randint(40,50) # (10,110)
                rand_speed = random.uniform(5.0, 6.0)
                q2 = ARRAY([rand_x,rand_y,q1[2]+rand_speed])


            #rand_x = q1[0] + np.random.randint(40,50) # (10,110)
            #rand_speed = random.uniform(2, 6)
            #q2 = ARRAY([rand_x,rand_y,q1[2]+rand_speed])
            
       
        yield (q2,)            

def get_yield_motion_gen(q1,q2,front_obstacle): # stream for generating YIELD trajectory  # TODO
    global wy_middle_lower_lane
    while True:
        b_x = front_obstacle[0][0]
        b_y = front_obstacle[0][1]
        b_speed = front_obstacle[2][0]
        dec_y = stg.wy_middle_lower_lane[0]
        dec_x = b_x - 4 * q1[2]  # q1---------*dec------*obs--q2  # delta_x = delta_t(3s) * v_q1
        dec_v = q1[2]
        
        thereIsTraj,traj = get_traj(q1[0],q1[1],q1[2],dec_x,dec_y,dec_v)
        if(thereIsTraj):
            thereIsTraj2,traj2 = get_traj(dec_x,dec_y,dec_v,q2[0],q2[1],b_speed)
            if(thereIsTraj2):
                # TODO append traj1 and traj2
                combined_traj = stg.combine(traj,traj2)
                yield (combined_traj, )
            else:
                yield None
        else:
            yield None

def get_follow_motion_gen(q1): # stream for generating YIELD trajectory  # TODO
    
    while True:
        new_q_x = q1[0] + 10  # follow for 10 meters
        new_q_y = q1[1]
        new_q_v = q1[2]
        
        thereIsTraj,traj = get_traj(q1[0],q1[1],q1[2],new_q_x,new_q_y,new_q_v)
        if(thereIsTraj):
            yield (traj, )
        else:
            yield None

def get_motion_gen(q1, q2):#,tim1):
   # def fn(q1, q2, fluents=[]):
    while True:
        """
        path=FrenetPath()
        points_num =  int(stg.PLANNING_T/stg.DT) +1
        path.t = np.arange(0,stg.PLANNING_T,stg.DT)
        if len(path.t) != points_num:
            path.t = np.concatenate((path.t, [stg.PLANNING_T]), axis=0)
        path.x = np.linspace(q1[0],q2[0],points_num)
        path.y = np.linspace(q1[1],q2[1],points_num)
        path.s_d = np.linspace(q1[2],q2[2],points_num)
        """

        #print("stream MOTION was calleddddddddddddddddddddddddddddddddddd")

       
         # target speed [m/s]
        #print(x0,y0,speed0,target_x,target_y,target_speed)
        
        x0 = q1[0]
        y0 = q1[1]
        speed0 = q1[2]

        target_x = q2[0]
        target_y = q2[1]
        target_speed = q2[2]
        if q2[0]< q1[0]+5 or q1[0]==q2[0] :#or get_distance(q1,q2) > 80 :
            yield None
        else:
            
            thereIsTraj,traj = get_traj(x0,y0,speed0,target_x,target_y,target_speed)
            if(thereIsTraj):
               yield (traj, )
            else:
                yield None
            #"""
            #yield (path,)

def get_yield_change_gen(q1,acc0,curr_dl,curr_ddl,front_obstacle): # stream for generating YIELD trajectory  # TODO
    global wy_middle_lower_lane
    while True:
        b_x = front_obstacle[0][0]
        b_y = front_obstacle[0][1]
        b_speed = front_obstacle[2][0]
        dec_y = stg.wy_middle_lower_lane[0]
        dec_x = b_x - 3 * q1[2]  # q1---------*dec------*obs--q2  # delta_x = delta_t(3s) * v_q1
        dec_v = q1[2]
        
        thereIsTraj,traj = get_traj_yield(q1[0],q1[1],q1[2],acc0,curr_dl,curr_ddl,dec_x,dec_y,b_speed)
        #get_traj(q1[0],q1[1],q1[2],dec_x,dec_y,b_speed)
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