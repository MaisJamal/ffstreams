


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
        print("No obstacles")

    return ego_state, obstacles_state, curr_time,lane_width


def UpdateToEgoOrigin(ego_state, obstacles):
    ego_s = ego_state.s
    print("ego s : ",ego_s)
   
    for obs in obstacles:
        print("obs_s ",obs.s)
        obs.s = obs.s - ego_s
    ego_state.s = 0
    return ego_state,obstacles
    


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