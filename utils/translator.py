
from imaplib import Time2Internaldate
from os import terminal_size
import numpy as np
from zmq import CONFLATE
import utils.settings as stg
from ffstreams.frenet_optimizer import FrenetPath,get_traj
import random
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


def get_connect_gen(q1):
    global ENV_WIDTH
    global LANE_WIDTH 
    global MAX_SPEED
    global TARGET_SPEED
    global wy_middle_lower_lane
    global wy_middle_upper_lane
    #i = 1
    while True:
        #primary configurations for testing: q1,q2,q3
        #q=[ARRAY([49.906,48.25]),ARRAY([25.919,52.564]),ARRAY([70,51.75]),ARRAY([75,48.25]),ARRAY([94.906,48.25]),ARRAY([119.906,48.25]),ARRAY([144.906,48.25]),ARRAY([160.906,48.25]),ARRAY([184.906,48.25])]
        #all_x = list(range(0,195,5))
        start_time = time.time()
        all_y=[stg.wy_middle_lower_lane[0],stg.wy_middle_upper_lane[0]]
        #rand_x = np.random.randint(min(q1[0]+5,stg.ENV_WIDTH-1),stg.ENV_WIDTH)
        rand_x = np.random.randint(0,180)
        rand_y = random.choice(all_y)#np.random.randint((stg.ENV_WIDTH/4 - stg.LANE_WIDTH+0.5)*10  , (stg.ENV_WIDTH/4 + stg.LANE_WIDTH-0.8)*10 )/10.0
        q2 = ARRAY([rand_x,rand_y])
        
        rand_speed = random.uniform(0, 4)
        q2 = np.concatenate((q2,[q1[2]+rand_speed-2]),axis=0)
        #print("called: ",i)
        #curr_time = time.time()
        #print("current time is : ",curr_time)
        #i+=1
        print("stream connect was called ", time.time()-start_time)
        yield (q2,)
            


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
        start_time = time.time()
        #path = [q1,q2]
       # t=Trajectory(path)
        #if get_distance(q1,q2) > 150 or q2[0]< q1[0] or q1[0]==q2[0] or get_distance(q1,q2) < 20:
        if q2[0]< q1[0]+5 or q1[0]==q2[0] :#or get_distance(q1,q2) > 80 :
            print("motion was called. time needed is " , time.time()-start_time,"from x0,y0,speed0: ",x0,y0,speed0,"to target x,y,speed: ",target_x,target_y,target_speed)
            yield None
        else:
            #traj=tuple(path)
            #print("type of ",traj,"is ", type(traj))
            #print(x0,target_x,y0,target_y,speed0,target_speed)
            
            #print (traj.x)
            
            thereIsTraj,traj = get_traj(x0,y0,speed0,target_x,target_y,target_speed)
            print("motion was called. time needed is " , time.time()-start_time,"from x0,y0,speed0: ",x0,y0,speed0,"to target x,y,speed: ",target_x,target_y,target_speed)
            #"""
            if(thereIsTraj):
                #print("tttttt: ", traj.t)
               # traj.t = traj.t + tim1
                #print("tttttt: ", f)
             #   traj.x = np.concatenate((path.x, [q2[0]]), axis=0)
            #    traj.y = np.concatenate((path.y, [q2[1]]), axis=0)
             #   traj.s_d = np.concatenate((path.s_d, [q2[2]]), axis=0)
                #print (traj.x)
              #  lastT = traj.t[-1]
              #  print("kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk  ",lastT)
              #  yield (traj, ARRAY([lastT]))
               yield (traj, )
            else:
                yield None
            #"""
            #yield (path,)




def translate_to_pddlplus(confs,traj_dict,traj_array,obstacles,file_name):
    #INIT_SPEED = 20 / 3.6
    #all_y=[stg.wy_middle_lower_lane[0],stg.wy_middle_upper_lane[0]]
    total_time = 10 #seconds
    delta_t = 0.4  # should be even
    #confs = []
    #q0 = ARRAY([0,all_y[0], INIT_SPEED])### x , y , speed , t =0
    #confs.append(q0)

      
    #goal = ARRAY([50,  48.25,INIT_SPEED])
    #confs.append(goal)
    confs_times = np.full(len(confs), 200)

    x0 = confs[0][0]
    y0 = confs[0][1]
    speed0 = confs[0][2]

    target_x = confs[-1][0]
    target_y = confs[-1][1]
    target_speed = confs[-1][2]

    #f= open("examples/autonomous_driving/third_try_prob.pddl","w+")
    f= open(file_name,"w+")
    f.write("(define (problem driving01)\n")
    f.write("\t(:domain auto-driving)\n")
    f.write("\t(:objects\n")
    f.write("\t\t")
    for i in range(len(confs)):
        f.write("q%d " % (i))

    cut_traj_dict = {}
    #thereIsTraj,traj =get_traj(x0,y0,speed0,target_x,target_y,target_speed)
    for i in range(len(confs)):
        for j in range(len(confs)):
            if traj_array[i][j]:
                cut_traj_dict[(i,j)] = FrenetPath()
                cut_traj_dict[(i,j)].x = traj_dict[(i,j)].x[0::int(delta_t*10/2)]
                cut_traj_dict[(i,j)].y = traj_dict[(i,j)].y[0::int(delta_t*10/2)]
                cut_traj_dict[(i,j)].t = traj_dict[(i,j)].t[0::int(delta_t*10/2)]
                for k in range(len(cut_traj_dict[(i,j)].x)-1):
                    f.write("q%d_%d_%d " % (i,j,k+1))
    
    f.write(" - conf\n")
    f.write("\t\t")
    if len(obstacles):
        for i in range(len(obstacles)):
            f.write("obs%d " % (i))
        f.write(" - obstacles\n")
    f.write("\t)\n")
    f.write("\t(:init\n")
    f.write("\t\t(ego_at q0)\n")
    #f.write("\t\t(= (at_x q0) 0)\n")
   # f.write("\t\t(= (at_y q0) 0)\n")
    #f.write("\t\t(= (at_time q0) 0)\n")
    f.write("\t\t(= (curr_time) 0)\n\n")

    for i in range(len(confs)):
        for j in range(len(confs)):
            if traj_array[i][j]:
                f.write("\t\t(traj q%d q%d)\n\n"  % (i,j))
                f.write("\t\t(next q%d q%d_%d_1 q%d)\n" % (i,i,j,j))

                for k in range(1,len(cut_traj_dict[(i,j)].x)):
                    f.write("\t\t(= (at_x q%d_%d_%d) %.2f )\n" % (i,j,k,cut_traj_dict[(i,j)].x[k]))
                    f.write("\t\t(= (at_y q%d_%d_%d) %.2f )\n" % (i,j,k,cut_traj_dict[(i,j)].y[k]))
                    f.write("\t\t(= (at_time q%d_%d_%d) %.2f )\n" % (i,j,k,cut_traj_dict[(i,j)].t[k]))
                    if k != (len(cut_traj_dict[(i,j)].x)-1):
                        f.write("\t\t(next q%d_%d_%d q%d_%d_%d q%d)\n\n" % (i,j,k,i,j,k+1,j))
                f.write("\t\t(next q%d_%d_%d q%d q%d)\n" % (i,j,len(cut_traj_dict[(i,j)].x)-1,j,j))
                f.write("\n\t\t(= (time_of_traj q%d q%d ) %.2f)\n" % (i,j,cut_traj_dict[(i,j)].t[-1]+ delta_t))
                f.write("\n\n")
                if j!= (len(confs)-1):
                    confs_times[j]= cut_traj_dict[(i,j)].t[-1]+ delta_t

    confs_times[0] = 0
    for i in range(len(confs)):
        f.write("\t\t(= (at_x q%d) %.2f )\n" % (i, confs[i][0]))
        f.write("\t\t(= (at_y q%d) %.2f )\n" % (i, confs[i][1]))
        f.write("\t\t(= (at_time q%d) %.2f )\n\n" % (i, confs_times[i]))
    f.write("\n\n")

    ############# defining obstacles ########################
    if not len(obstacles):
        for i in range(len(obstacles)):
            
            obs_start_x = obstacles[i][0][0]
            obs_start_y = obstacles[i][0][1]
            obs_speed = obstacles[i][2][0]
            
            #f.write("\t\t(obstacle obs%d_%d)\n" % (i,k))
            f.write("\t\t(= (obst_at_x obs%d) %.2f)\n" % (i,obs_start_x))
            f.write("\t\t(= (obst_at_y obs%d) %.2f)\n" % (i,obs_start_y))
            f.write("\t\t(= (obst_at_speed obs%d) %.2f)\n" % (i,obs_speed))
            

    f.write("\t)\n")
    f.write("\t(:goal (and (ego_at q%d)  ))\n" %(len(confs)-1))
    f.write(")\n")
   
    #print("finished writing")
    f.close()



def translate_to_pddl_2_1(confs,traj_dict,traj_array,obstacles,file_name,problem_file_name):
    #INIT_SPEED = 20 / 3.6
    #all_y=[stg.wy_middle_lower_lane[0],stg.wy_middle_upper_lane[0]]
    total_time = 10 #seconds
    delta_t = 0.4  # should be even
    #confs = []
    #q0 = ARRAY([0,all_y[0], INIT_SPEED])### x , y , speed , t =0
    #confs.append(q0)

      
    #goal = ARRAY([50,  48.25,INIT_SPEED])
    #confs.append(goal)
    confs_times = np.full(len(confs), 200)

    x0 = confs[0][0]
    y0 = confs[0][1]
    speed0 = confs[0][2]

    target_x = confs[-1][0]
    target_y = confs[-1][1]
    target_speed = confs[-1][2]

    #f= open("examples/autonomous_driving/third_try_prob.pddl","w+")
    f= open(file_name+problem_file_name,"w+")
    f.write("(define (problem driving01)\n")
    f.write("\t(:domain auto-driving)\n")
    f.write("\t(:objects\n")
    f.write("\t\t")
    for i in range(len(confs)):
        f.write("q%d " % (i))

    cut_traj_dict = {}
    #thereIsTraj,traj =get_traj(x0,y0,speed0,target_x,target_y,target_speed)
    for i in range(len(confs)):
        for j in range(len(confs)):
            if traj_array[i][j]:
                cut_traj_dict[(i,j)] = FrenetPath()
                cut_traj_dict[(i,j)].x = traj_dict[(i,j)].x[0::int(delta_t*10/2)]
                cut_traj_dict[(i,j)].y = traj_dict[(i,j)].y[0::int(delta_t*10/2)]
                cut_traj_dict[(i,j)].t = traj_dict[(i,j)].t[0::int(delta_t*10/2)]
                for k in range(len(cut_traj_dict[(i,j)].x)-1):
                    f.write("q%d_%d_%d " % (i,j,k+1))
    
    f.write(" - conf\n")
    f.write("\t\t")
    if len(obstacles):
        for i in range(len(obstacles)):
            f.write("obs%d " % (i))
        f.write(" - obstacles\n")
    f.write("\t)\n")
    f.write("\t(:init\n")
    f.write("\t\t(ego_at q0)\n")
    #f.write("\t\t(= (at_x q0) 0)\n")
   # f.write("\t\t(= (at_y q0) 0)\n")
    #f.write("\t\t(= (at_time q0) 0)\n")
    f.write("\t\t(= (curr_time) 0)\n")
    f.write("\t\t(idle)\n\n")
    
    

    for i in range(len(confs)):
        for j in range(len(confs)):
            if traj_array[i][j]:
                f.write("\t\t(traj q%d q%d)\n\n"  % (i,j))
                f.write("\t\t(next q%d q%d_%d_1 q%d)\n" % (i,i,j,j))

                for k in range(1,len(cut_traj_dict[(i,j)].x)):
                    f.write("\t\t(= (at_x q%d_%d_%d) %.2f )\n" % (i,j,k,cut_traj_dict[(i,j)].x[k]))
                    f.write("\t\t(= (at_y q%d_%d_%d) %.2f )\n" % (i,j,k,cut_traj_dict[(i,j)].y[k]))
                    f.write("\t\t(= (at_time q%d_%d_%d) %.2f )\n" % (i,j,k,cut_traj_dict[(i,j)].t[k]))
                    if k != (len(cut_traj_dict[(i,j)].x)-1):
                        f.write("\t\t(next q%d_%d_%d q%d_%d_%d q%d)\n\n" % (i,j,k,i,j,k+1,j))
                f.write("\t\t(next q%d_%d_%d q%d q%d)\n" % (i,j,len(cut_traj_dict[(i,j)].x)-1,j,j))
                f.write("\n\t\t(= (time_of_traj q%d q%d ) %.2f)\n" % (i,j,cut_traj_dict[(i,j)].t[-1]+ delta_t))
                f.write("\n\n")
                if j!= (len(confs)-1):
                    confs_times[j]= cut_traj_dict[(i,j)].t[-1]+ delta_t

    confs_times[0] = 0
    for i in range(len(confs)):
        f.write("\t\t(= (at_x q%d) %.2f )\n" % (i, confs[i][0]))
        f.write("\t\t(= (at_y q%d) %.2f )\n" % (i, confs[i][1]))
        f.write("\t\t(= (at_time q%d) %.2f )\n\n" % (i, confs_times[i]))
    f.write("\n\n")

    ############# defining obstacles ########################
    if len(obstacles):
        for i in range(len(obstacles)):
            
            obs_start_x = obstacles[i][0][0]
            obs_start_y = obstacles[i][0][1]
            obs_speed = obstacles[i][2][0]
            
            #f.write("\t\t(obstacle obs%d_%d)\n" % (i,k))
            f.write("\t\t(= (obst_at_x obs%d) %.2f)\n" % (i,obs_start_x))
            f.write("\t\t(= (obst_at_y obs%d) %.2f)\n" % (i,obs_start_y))
            f.write("\t\t(= (obst_at_speed obs%d) %.2f)\n" % (i,obs_speed))
            

    f.write("\t)\n")
    f.write("\t(:goal (and (ego_at q%d)  ))\n" %(len(confs)-1))
    f.write("\t(:metric minimize (curr_time))\n")
    f.write(")\n")
    
   
    #print("finished writing")
    f.close()




def translate_to_pddl_change_lane(direction,confs,traj_dict,traj_array,obstacles,file_name,problem_file_name):
    #INIT_SPEED = 20 / 3.6
    #all_y=[stg.wy_middle_lower_lane[0],stg.wy_middle_upper_lane[0]]
    total_time = 10 #seconds
    delta_t = 0.2  # should be even
    #confs = []
    #q0 = ARRAY([0,all_y[0], INIT_SPEED])### x , y , speed , t =0
    #confs.append(q0)

      
    #goal = ARRAY([50,  48.25,INIT_SPEED])
    #confs.append(goal)
    confs_times = np.full(len(confs), 200)

    x0 = confs[0][0]
    y0 = confs[0][1]
    speed0 = confs[0][2]

    target_x = confs[-1][0]
    target_y = confs[-1][1]
    target_speed = confs[-1][2]

    #f= open("examples/autonomous_driving/third_try_prob.pddl","w+")
    f= open(file_name+problem_file_name,"w+")
    f.write("(define (problem driving01)\n")
    f.write("\t(:domain auto-driving)\n")
    f.write("\t(:objects\n")
    f.write("\t\t")
    for i in range(len(confs)):
        f.write("q%d " % (i))

    cut_traj_dict = {}
    #thereIsTraj,traj =get_traj(x0,y0,speed0,target_x,target_y,target_speed)
    for i in range(len(confs)):
        for j in range(len(confs)):
            if traj_array[i][j]:
                cut_traj_dict[(i,j)] = FrenetPath()
                cut_traj_dict[(i,j)].x = traj_dict[(i,j)].x[0::int(delta_t*10/2)]
                cut_traj_dict[(i,j)].y = traj_dict[(i,j)].y[0::int(delta_t*10/2)]
                cut_traj_dict[(i,j)].t = traj_dict[(i,j)].t[0::int(delta_t*10/2)]
                for k in range(len(cut_traj_dict[(i,j)].x)-1):
                    f.write("q%d_%d_%d " % (i,j,k+1))
    
    f.write(" - conf\n")
    f.write("\t\t")
    if len(obstacles):
        for i in range(len(obstacles)):
            f.write("obs%d " % (i))
        f.write(" - obstacles\n")
    f.write("\t)\n")
    f.write("\t(:init\n")
    f.write("\t\t(ego_at q0)\n")
    #f.write("\t\t(= (at_x q0) 0)\n")
   # f.write("\t\t(= (at_y q0) 0)\n")
    #f.write("\t\t(= (at_time q0) 0)\n")
    f.write("\t\t(= (curr_time) 0)\n")
    f.write("\t\t(idle)\n\n")
    f.write("\t\t(on_right_lane)\n\n")
    
    for i in range(len(confs)):
        for j in range(len(confs)):
            if traj_array[i][j]:
                f.write("\t\t(traj q%d q%d)\n\n"  % (i,j))
                if direction =="left":
                    f.write("\t\t(left_traj q%d q%d)\n\n"  % (i,j))
                else:
                    f.write("\t\t(right_traj q%d q%d)\n\n"  % (i,j))
                f.write("\t\t(next q%d q%d_%d_1 q%d)\n" % (i,i,j,j))

                for k in range(1,len(cut_traj_dict[(i,j)].x)):
                    f.write("\t\t(= (at_x q%d_%d_%d) %.2f )\n" % (i,j,k,cut_traj_dict[(i,j)].x[k]))
                    f.write("\t\t(= (at_y q%d_%d_%d) %.2f )\n" % (i,j,k,cut_traj_dict[(i,j)].y[k]))
                    f.write("\t\t(= (at_time q%d_%d_%d) %.2f )\n" % (i,j,k,cut_traj_dict[(i,j)].t[k]))
                    if k != (len(cut_traj_dict[(i,j)].x)-1):
                        f.write("\t\t(next q%d_%d_%d q%d_%d_%d q%d)\n\n" % (i,j,k,i,j,k+1,j))
                f.write("\t\t(next q%d_%d_%d q%d q%d)\n" % (i,j,len(cut_traj_dict[(i,j)].x)-1,j,j))
                f.write("\n\t\t(= (time_of_traj q%d q%d ) %.2f)\n" % (i,j,cut_traj_dict[(i,j)].t[-1]+ delta_t))
                f.write("\n\n")
                if j!= (len(confs)-1):
                    confs_times[j]= cut_traj_dict[(i,j)].t[-1]+ delta_t

    confs_times[0] = 0
    for i in range(len(confs)):
        f.write("\t\t(= (at_x q%d) %.2f )\n" % (i, confs[i][0]))
        f.write("\t\t(= (at_y q%d) %.2f )\n" % (i, confs[i][1]))
        f.write("\t\t(= (at_time q%d) %.2f )\n\n" % (i, confs_times[i]))
    f.write("\n\n")

    ############# defining obstacles ########################
    if len(obstacles):
        for i in range(len(obstacles)):
            
            obs_start_x = obstacles[i][0][0]
            obs_start_y = obstacles[i][0][1]
            obs_speed = obstacles[i][2][0]
            
            #f.write("\t\t(obstacle obs%d_%d)\n" % (i,k))
            f.write("\t\t(= (obst_at_x obs%d) %.2f)\n" % (i,obs_start_x))
            f.write("\t\t(= (obst_at_y obs%d) %.2f)\n" % (i,obs_start_y))
            f.write("\t\t(= (obst_at_speed obs%d) %.2f)\n" % (i,obs_speed))
            

    f.write("\t)\n")
    if direction =="left":
        f.write("\t(:goal (and (on_left_lane)  ))\n")
    else:
        f.write("\t(:goal (and (on_right_lane)  ))\n")
    f.write("\t(:metric minimize (curr_time))\n")
    f.write(")\n")
    
   
    #print("finished writing")
    f.close()