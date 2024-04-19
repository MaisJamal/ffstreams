import json
import time
from bottle import run, post, request, response,get
import math
import numpy as np
from ffstreams.ffstreams_algorithm import solve_ffstreams
from utils.apollo_utils import parse_req_msg,UpdateToEgoOrigin,UpdateToWorldOrigin,InterpolateTraj,CorrectVelocityAcceleration
import utils.apollo_config as cfg
import yaml

with open('config/config.yml', 'r') as file:
    config = yaml.safe_load(file)

DEBUG = False

def get_nearst_point(curr_x,curr_y,xs,ys):
  idx_nearst = 0
  delta_x = xs[0]-curr_x
  delta_y = ys[0]-curr_y
  dist_nearst = math.sqrt(delta_x*delta_x + delta_y*delta_y)
  for i in range(1,len(xs)):
    delta_x = xs[i]-curr_x
    delta_y = ys[i]-curr_y
    dist = math.sqrt(delta_x*delta_x + delta_y*delta_y)
    if dist < dist_nearst:
      dist_nearst = dist
      idx_nearst = i 
  return idx_nearst 

end_of_traj = False

@post('/process')
def my_process():
    global end_of_traj
    request_msg = json.loads(request.body.read())
    #print(request_msg)

    ego_state, obstacles, timestamp,lane_width = parse_req_msg(request_msg)
    ## debug ##
    print("debug request: ",ego_state," current time: ", timestamp)
    for Obs in obstacles:
       print(Obs)
    ###########

   # generated_task = transformHTTPRequestToTask(ego_state, obstacles)
    ego_state,obstacles = UpdateToEgoOrigin(ego_state, obstacles)
    speed_limit = 16.67 # m/s = 60 km/h
    max_acc = 5
    file_path = config['apollo']['path']
    problem_file = "problem.pddl"
    #debug
    print(ego_state)
    for obs in obstacles:
       print(obs)
    
    #init_x, init_y,init_heading,init_speed,init_scene_obstacles, lane_width
    cfg.init(lane_width,speed_limit,max_acc,file_path,problem_file) 
    trajectories = []
    #trajectories,confs,traj_dict,final_traj_type = solve_ffstreams(ego_state=ego_state,obstacles=obstacles)

    if len(trajectories) < 1:
            print("No trajectory was found")
    else: 
        traj =trajectories[0]
        scene_angle = ego_state.yaw #0#3.14
        traj = UpdateToWorldOrigin(traj,ego_state,scene_angle)
        
        t = traj.t
        a = traj.s_dd
        v = traj.s_d
        x = traj.x #[ego_state.x - k   for k in traj.s] 
        y = traj.y #[ego_state.y + k   for k in traj.d] 
        print("yaw traj ",traj.yaw)
        traj = InterpolateTraj(traj)
        #traj = CorrectVelocityAcceleration(ego_state,traj)
        heading = [scene_angle]*len(t)
        if DEBUG:
          print(traj.t)
          print("x ", traj.x)
          print("y ", traj.y)
          print(traj.s)
          print(traj.d)
          print(traj.s_d) # longitudinal velocity
          print(traj.s_dd)    # longitudinal acceleration
          print("yaw ",heading)
    ############ Trajectory example ##################
    
    curr_v = ego_state.v
    curr_x = ego_state.x
    curr_y = ego_state.y
    
    t1 = np.arange(0.1, 1.1, 0.02) # start, end , step
    t1 = t1.tolist()
    t2 = np.arange(1.1, 7.1, 0.1) # start, end , step
    t2 = t2.tolist()
    t = t1 + t2

    heading = [0]*len(t)
    a = [0.3]* len(t)
    v = [(curr_v + a[0]* ti) for ti in t]
    x = [(curr_x + math.cos(heading[0])* curr_v* ti + math.cos(heading[0])* a[0]*ti*ti) for ti in t]
    y = [curr_y]* len(t)
    
    print("first point: x ", x[0]," y ",y[0]," t ",t[0]," v ", v[0]," a ",a[0],"len of traj : ",len(x))

    trajectory_info = {}
    trajectory_info["x"] = x
    trajectory_info["y"] = y
    trajectory_info["heading"] = heading
    trajectory_info["v"] = v
    trajectory_info["a"] = a
    trajectory_info["relative_time"] = t

    respond_ = {"trajectory": trajectory_info , "run_info":"run successfully..."}
    time.sleep(0.1)
    return json.dumps(respond_)
  
run(host='127.0.0.3', port=9000, debug=True)