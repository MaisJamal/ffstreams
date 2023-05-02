import numpy as np

def init():
    global ENV_WIDTH
    global LANE_WIDTH
    global MAX_SPEED
    global MAX_ACCEL
    global DT
    global TARGET_SPEED
    global TARGET_L
    global ROBOT_RADIUS
    global show_animation
    global PLANNING_T
    global area
    global wx_middle_lane
    global wy_middle_lower_lane
    global wy_middle_upper_lane
    global wy_middle_lower_lower_lane
    global wy_middle_upper_upper_lane
    global bound1
    global bound2
    global bound3
    global bound4
    global tx
    global ty
    global tx2
    global ty2
    global tx3
    global ty3
    global tx4
    global ty4
    global txBound1
    global tyBound1
    global txBound2
    global tyBound2
    global txBound3
    global tyBound3
    global txBound4
    global tyBound4
    global txBound5
    global tyBound5
    #global csp
    # for animation
    area = 30.0
    show_animation = False
    #### environment parameters
    ENV_WIDTH = 200 #2 #100  # meters
    #ENV_HEIGHT = ENV_WIDTH / 2
    LANE_WIDTH = 3.5 #0.1 #3.5 # meters
    #wx_middle_lane = [0.0, 10.0, 20.5, 35.0, 70.5, ENV_WIDTH]
    #wy_middle_lower_lane = [ENV_WIDTH/4-LANE_WIDTH/2 , ENV_WIDTH/4-LANE_WIDTH/2 ,ENV_WIDTH/4-LANE_WIDTH/2, ENV_WIDTH/4-LANE_WIDTH/2, ENV_WIDTH/4-LANE_WIDTH/2, ENV_WIDTH/4-LANE_WIDTH/2]
    wx_middle_lane = [0.0, ENV_WIDTH*3]
    wy_middle_lower_lane = [ENV_WIDTH/4-LANE_WIDTH/2, ENV_WIDTH/4-LANE_WIDTH/2]
    wy_middle_upper_lane  = [pt + LANE_WIDTH for pt in wy_middle_lower_lane]
    wy_middle_lower_lower_lane = [pt - LANE_WIDTH for pt in wy_middle_lower_lane]
    wy_middle_upper_upper_lane = [pt + LANE_WIDTH for pt in wy_middle_upper_lane]

    tx = np.arange(0, wx_middle_lane[-1], 0.1)
    ty = np.full((len(tx),), wy_middle_lower_lane[0])

    tx2 = np.arange(0, wx_middle_lane[-1], 0.1)
    ty2 = np.full((len(tx2),), wy_middle_upper_lane[0])
    
    tx3 = np.arange(0, wx_middle_lane[-1], 0.1)
    ty3 = np.full((len(tx3),), wy_middle_lower_lower_lane[0])

    tx4 = np.arange(0, wx_middle_lane[-1], 0.1)
    ty4 = np.full((len(tx4),), wy_middle_upper_upper_lane[0])

    #csp = []
    # lane bounds from down to up:
    bound1 = [pt - LANE_WIDTH/2 for pt in wy_middle_lower_lane]
    bound2 = [pt + LANE_WIDTH/2 for pt in wy_middle_lower_lane]
    bound3 = [pt + LANE_WIDTH*1.5 for pt in wy_middle_lower_lane]
    bound4 = [pt - LANE_WIDTH*1.5 for pt in wy_middle_lower_lane]
    bound5 = [pt + LANE_WIDTH*1.5 + LANE_WIDTH for pt in wy_middle_lower_lane]

    ######


    txBound1 = np.arange(0, wx_middle_lane[-1], 0.1)
    tyBound1 = np.full((len(txBound1),), bound1[0]) 

    txBound2 = np.arange(0, wx_middle_lane[-1], 0.1)
    tyBound2 = np.full((len(txBound2),), bound2[0]) 

    txBound3 = np.arange(0, wx_middle_lane[-1], 0.1)
    tyBound3 = np.full((len(txBound3),), bound3[0]) 

    txBound4 = np.arange(0, wx_middle_lane[-1], 0.1)
    tyBound4 = np.full((len(txBound4),), bound4[0]) 

    txBound5 = np.arange(0, wx_middle_lane[-1], 0.1)
    tyBound5 = np.full((len(txBound5),), bound5[0]) 

    # Parameter
    MAX_SPEED = 80.0 / 3.6  # maximum speed [m/s]
    MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
    #MAX_CURVATURE = 1.0  # maximum curvature [1/m]
    #MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
    #D_ROAD_W = 1.0  # road width sampling length [m]
    DT = 0.2  # time tick [s]
    PLANNING_T = 0.4
    MAX_T = 5.0  # max prediction time [m]
    MIN_T = 4.0  # min prediction time [m]
    TARGET_SPEED = 20.0 / 3.6  # target speed [m/s]
    ### added by me
    TARGET_L = 1

    #####
    #D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
    #N_S_SAMPLE = 1  # sampling number of target speed
    ROBOT_RADIUS = 2.0  # robot radius [m]


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []

def combine(traj1,traj2):
    new_traj = FrenetPath()
    idx = len(traj2.s_d) -1
    for i in range(len(traj2.s_d)):
        if abs(traj2.s_d[i]- traj2.s_d[-1]) < 0.01 :
            idx = i
            break
    new_traj.t = np.concatenate((traj1.t, [i+traj1.t[-1] for i in traj2.t[2:idx]]),axis = 0)
    new_traj.x = np.concatenate((traj1.x, traj2.x[2:idx]), axis=0) 
    new_traj.y = np.concatenate((traj1.y, traj2.y[2:idx]), axis=0) 
    #new_traj.s = np.concatenate((traj1.s, traj2.s[2:idx]), axis=0)
    new_traj.s_d = np.concatenate((traj1.s_d, traj2.s_d[2:idx]), axis=0) 
    #new_traj.s_dd = np.concatenate((traj1.s_dd, traj2.s_dd[2:idx]), axis=0)  
    #new_traj.s_ddd = np.concatenate((traj1.s_ddd, traj2.s_ddd[2:idx]), axis=0) 
    #new_traj.d = np.concatenate((traj1.d, traj2.d[2:idx]), axis=0) 
    #new_traj.d_d = np.concatenate((traj1.d_d, traj2.d_d[2:idx]), axis=0)
    #new_traj.d_dd = np.concatenate((traj1.d_dd, traj2.d_dd[2:idx]), axis=0)
    #new_traj.d_ddd = np.concatenate((traj1.d_ddd, traj2.d_ddd[2:idx]), axis=0)
    return new_traj

