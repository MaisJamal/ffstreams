
from imaplib import Time2Internaldate
from os import terminal_size
import numpy as np
from zmq import CONFLATE
import utils.settings as stg
from ffstreams.frenet_optimizer import FrenetPath,get_traj
import random
from numpy.linalg import norm
import time

ARRAY = np.array 



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


def translate_to_pddl_keep_lane(there_is_front_obs,confs,traj_dict,traj_type,traj_array,obstacles,file_name,problem_file_name):
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
    f.write("\t\t(= (cost) 0)\n")
    f.write("\t\t(on_init_lane)\n")
    #overtaking
    if (there_is_front_obs):
        f.write("\t\t(there_is_front_obs)\n")

    f.write("\t\t(idle)\n\n")
    for i in range(len(confs)):
        for j in range(len(confs)):
            if traj_array[i][j]:
                f.write("\t\t(traj q%d q%d)\n\n"  % (i,j))
                tj_type = traj_type[(i,j)]
                
                if tj_type == "YIELD":
                    f.write("\t\t(yield_traj q%d q%d)\n\n"  % (i,j))
                elif tj_type == "FOLLOW":
                    f.write("\t\t(keep_speed_traj q%d q%d)\n\n"  % (i,j))
                elif tj_type == "CHANGE_LEFT":
                    f.write("\t\t(left_traj q%d q%d)\n\n"  % (i,j))
                elif tj_type == "CHANGE_RIGHT":
                    f.write("\t\t(right_traj q%d q%d)\n\n"  % (i,j))
                elif tj_type == "OVERTAKE":
                    f.write("\t\t(overtake_traj q%d q%d)\n\n"  % (i,j))

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
    f.write("\t(:goal (and (on_init_lane)(moved_forward)  ))\n")

    f.write("\t(:metric minimize (cost))\n")
    f.write(")\n")
    
   
    #print("finished writing")
    f.close()






def translate_to_pddl_cr(there_is_front_obs,confs,traj_dict,traj_type,traj_array,obstacles,file_name,problem_file_name):
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
    f.write("\t\t(= (cost) 0)\n")
    f.write("\t\t(on_init_lane)\n")
    #overtaking
    if (there_is_front_obs):
        f.write("\t\t(there_is_front_obs)\n")

    f.write("\t\t(idle)\n\n")
    for i in range(len(confs)):
        for j in range(len(confs)):
            if traj_array[i][j]:
                f.write("\t\t(traj q%d q%d)\n\n"  % (i,j))
                tj_type = traj_type[(i,j)]
                
                if tj_type == "YIELD":
                    f.write("\t\t(yield_traj q%d q%d)\n\n"  % (i,j))
                elif tj_type == "FOLLOW":
                    f.write("\t\t(keep_speed_traj q%d q%d)\n\n"  % (i,j))
                elif tj_type == "CHANGE_LEFT":
                    f.write("\t\t(left_traj q%d q%d)\n\n"  % (i,j))
                elif tj_type == "CHANGE_RIGHT":
                    f.write("\t\t(right_traj q%d q%d)\n\n"  % (i,j))
                elif tj_type == "OVERTAKE":
                    f.write("\t\t(overtake_traj q%d q%d)\n\n"  % (i,j))

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
    f.write("\t(:goal (and (moved_forward)  ))\n")

    f.write("\t(:metric minimize (cost))\n")
    f.write(")\n")
    
   
    #print("finished writing")
    f.close()