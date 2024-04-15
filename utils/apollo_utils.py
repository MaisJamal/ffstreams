import numpy as np

import math

class EgoState:
    def __init__(self):
        self.t = 0
        self.l = 0
        self.dl = 0
        self.ddl = 0
        self.s = 0
        self.ds = 0
        self.dds = 0

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0
        self.a = 0
    def __str__(self):
        return f"Ego State: x {self.x}, y {self.y}, v {self.v}, a {self.a}, s {self.s}, ds {self.ds},  dds {self.dds}, l {self.l}, dl {self.dl}, ddl {self.ddl}, yaw(rad) {self.yaw}"


class ObstacleState:
    def __init__(self):
        self.t = 0
        self.l = 0
        self.s = 0
        
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0
        self.shape = (5.5, 2.5)
    def __str__(self):
        return f"Obstacle: s {self.s}, l {self.l}, x {self.x}, y {self.y}, v {self.v}"



def parse_req_msg(request_msg):    
    curr_time = request_msg["vehicle_state"]["timestamp"]
    lane_width = request_msg["vehicle_state"]["lane_width"]

    ego_state = EgoState()
    ego_state.x = request_msg["vehicle_state"]["x"]
    ego_state.y = request_msg["vehicle_state"]["y"]
    ego_state.yaw = request_msg["vehicle_state"]["heading"]
    ego_state.v = request_msg["vehicle_state"]["linear_velocity"]
    ego_state.a = request_msg["vehicle_state"]["linear_acceleration"]
    print("received x, y, yaw: ",ego_state.x,ego_state.y, ego_state.yaw)
    num_obstacles = request_msg["vehicle_state"]["obstacles_num"]
    obstacles = request_msg["obstacles"]
    
    ego_state.s = request_msg["vehicle_state"]["s"]
    ego_state.ds = request_msg["vehicle_state"]["ds"]
    ego_state.dds = request_msg["vehicle_state"]["dds"]
    ego_state.l = request_msg["vehicle_state"]["l"]
    ego_state.dl = request_msg["vehicle_state"]["dl"]
    ego_state.ddl = request_msg["vehicle_state"]["ddl"]

    obstacles_state = []
    
    for i in range(1,num_obstacles+1):
        NewObs = ObstacleState()
        NewObs.x = float(obstacles["obs_x"+str(i)])
        NewObs.y = float(obstacles["obs_y"+str(i)])
        NewObs.s = float(obstacles["obs_s"+str(i)])
        NewObs.l = float(obstacles["obs_l"+str(i)])
        NewObs.yaw = float(obstacles["obs_theta"+str(i)])
        NewObs.v = float(obstacles["obs_speed"+str(i)])
        obstacles_state.append(NewObs)

    if len(obstacles_state) < 1 :
        print("No obstacles, adding a virtual one...")
        temp_obs = ObstacleState()
        temp_obs.v = ego_state.v + 10
        temp_obs.s = 110 + ego_state.s
        temp_obs.l = 0
        obstacles_state.append(temp_obs)
    return ego_state, obstacles_state, curr_time,lane_width


def UpdateToEgoOrigin(ego_state, obstacles):
    ego_s = ego_state.s
    print("ego s : ",ego_s)
   
    for obs in obstacles:
        print("obs_s ",obs.s)
        obs.s = obs.s - ego_s
    ego_state.s = 0
    return ego_state,obstacles
    

def UpdateToWorldOrigin(traj,ego_state,angle):
    NewOrigin = (ego_state.x,ego_state.y)
    fix_start_l = traj.d[0]
    for i in range(len(traj.s)):
        traj.x[i] = traj.s[i] + NewOrigin[0]
        traj.y[i] = traj.d[i] + NewOrigin[1] - fix_start_l
        #traj.x[i],traj.y[i] = rotate(NewOrigin,(traj.x[i],traj.y[i]),angle)
          
    return traj


def  extract_front_obstacle(obstacles,q_ego):
    ### obstacles is a list of tuples (x pos, y pos, linear velocity)
    ### q_ego is the configuration of the ego vehicle , a tuple (x pos, y pos, linear velocity)
    front_obs_idx = 0
    found_front_obs = False
    i = 0
    for obs in obstacles:
        if abs(obs.l - q_ego[1]) < 1 :  # compare Y position
            if obs.s > q_ego[0] :        # compare X position
                if not found_front_obs:
                    found_front_obs = True
                    front_obs_idx = i
                else:
                    if obs.s < obstacles[front_obs_idx].s: # found a closed front obstacle
                        front_obs_idx = i
        i=i+1
    return front_obs_idx

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def InterpolateTraj(traj):
    # t = [0.0, 0.2, 0.4, 0.6000000000000001, 0.8, 1.0,
    #  1.2000000000000002, 1.4000000000000001, 1.6, 1.8, 2.0,
    #  2.2, 2.4000000000000004, 2.6, 2.8000000000000003, 3.0,
    #  3.2, 3.4000000000000004, 3.6, 3.8000000000000003, 4.0,
    #  4.2, 4.4, 4.6000000000000005, 4.800000000000001]
    print("before " ,len(traj.t))
    t_ref = traj.t
    x_ref = traj.x
    y_ref = traj.y
    v_ref = traj.s_d
    a_ref = traj.s_dd
    
    new_t = []
    new_x = []  
    new_y = []
    new_v = []
    new_a = []

    for ti in np.arange(0.1, 1.1, 0.02):
        new_t.append(ti)
        new_x.append(np.interp(ti ,t_ref,x_ref)) 
        new_y.append(np.interp(ti ,t_ref,y_ref))
        new_v.append(np.interp(ti ,t_ref,v_ref))
        new_a.append(np.interp(ti ,t_ref,a_ref))

    for ti in np.arange(1.1, 7.2, 0.1):
        new_t.append(ti)
        new_x.append(np.interp(ti ,t_ref,x_ref)) 
        new_y.append(np.interp(ti ,t_ref,y_ref))
        new_v.append(np.interp(ti ,t_ref,v_ref))
        new_a.append(np.interp(ti ,t_ref,a_ref))

    traj.t = new_t
    traj.x = new_x
    traj.y = new_y
    traj.s_d = new_v
    traj.s_dd = new_a
    print("after " ,len(traj.t))

    return traj


def CorrectVelocityAcceleration(ego_state,traj):
    # correct v and a 
    # traj.s_d == v
    # traj.s_dd == a
    new_v = [ego_state.v]
    new_a = [ego_state.a]
    for i in range(1,len(traj.t)):
        delta_x = traj.x[i] - traj.x[i-1]
        delta_y = traj.y[i] - traj.y[i-1]
        delta_t = traj.t[i]-traj.t[i-1]
        dist = math.sqrt(delta_x*delta_x + delta_y*delta_y)
        v = dist / delta_t
        new_v.append(v)
        a = (new_v[i]-new_v[i-1] ) / delta_t
        new_a.append(a)
    traj.s_d = new_v
    traj.s_dd = new_a

    return traj


"""
#####
 ### validate trajectory ###
    vx = []
    vs = []
    vy = []
    vl = []
    v_total = []
    heading_rad = []

    for i in range(len(traj.x) - 1):
        dx = traj.x[i + 1] - traj.x[i]
        dy = traj.y[i + 1] - traj.y[i]
        dt = traj.t[i + 1] - traj.t[i]
        ds = math.hypot(dx, dy)
        ds2 = traj.s[i + 1] - traj.s[i]
        if ds == ds2:
            print("ds is correct")
        else:
            print("ds is not identical: ds1 = ",ds," while ds2 = ",ds2)
        dl = traj.d[i + 1] - traj.d[i]
        vx = dx/dt
        vy = dy/dt
        vs = ds/dt
        vl = dl/dt
        v_tot = ds/dt
        v_tot2 = math.hypot(vx,vy)
        v_tot3 = math.hypot(vs,vl)
        print("different v : v1 = ", v_tot," ,v2 = ",v_tot2," ,v3 = ",v_tot3)
        v_total.append(ds/dt)
        
        heading_rad.append(math.atan2(dy, dx))
    ###########################
"""