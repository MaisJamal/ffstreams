import numpy as np

def init(lane_width,speed_limit, max_acc,file_path,problem_file):
    global ENV_WIDTH
    global LANE_WIDTH
    global MAX_SPEED
    global MAX_ACC
    global FILE_PATH
    global PROBLEM_FILE
    global wy_middle_lane_center
    global wy_left_lane_center
    global tx
    global ty
    global tx2
    global ty2
    global txBound1
    global tyBound1
    global txBound2
    global tyBound2
    global txBound3
    global tyBound3


    FILE_PATH = file_path
    PROBLEM_FILE = problem_file


    ENV_WIDTH = 100 # meters
    LANE_WIDTH = lane_width
    MAX_SPEED = speed_limit
    MAX_ACC = max_acc

    wx_center_lane = [0.0, ENV_WIDTH]
    wy_middle_lane_center = [0, 0]
    wy_left_lane_center  = [pt + LANE_WIDTH for pt in wy_middle_lane_center]
    #wy_middle_lower_lower_lane = [pt - LANE_WIDTH for pt in wy_right_lane_center]
    #wy_middle_upper_upper_lane = [pt + LANE_WIDTH for pt in wy_left_lane_center]

    tx = np.arange(0, wx_center_lane[-1], 0.1)
    ty = np.full((len(tx),), wy_middle_lane_center[0])

    tx2 = np.arange(0, wx_center_lane[-1], 0.1)
    ty2 = np.full((len(tx2),), wy_left_lane_center[0])
    
    tx3 = np.arange(0, wx_center_lane[-1], 0.1)
    #ty3 = np.full((len(tx3),), wy_middle_lower_lower_lane[0])

    tx4 = np.arange(0, wx_center_lane[-1], 0.1)
    #ty4 = np.full((len(tx4),), wy_middle_upper_upper_lane[0])

    #csp = []
    # lane bounds from down to up:
    bound1 = [pt - LANE_WIDTH/2 for pt in wy_middle_lane_center]
    bound2 = [pt + LANE_WIDTH/2 for pt in wy_middle_lane_center]
    bound3 = [pt + LANE_WIDTH*1.5 for pt in wy_middle_lane_center]
    bound4 = [pt - LANE_WIDTH*1.5 for pt in wy_middle_lane_center]
    bound5 = [pt + LANE_WIDTH*1.5 + LANE_WIDTH for pt in wy_middle_lane_center]

    ######


    txBound1 = np.arange(0, wx_center_lane[-1], 0.1)
    tyBound1 = np.full((len(txBound1),), bound1[0]) 

    txBound2 = np.arange(0, wx_center_lane[-1], 0.1)
    tyBound2 = np.full((len(txBound2),), bound2[0]) 

    txBound3 = np.arange(0, wx_center_lane[-1], 0.1)
    tyBound3 = np.full((len(txBound3),), bound3[0]) 

    txBound4 = np.arange(0, wx_center_lane[-1], 0.1)
    tyBound4 = np.full((len(txBound4),), bound4[0]) 

    txBound5 = np.arange(0, wx_center_lane[-1], 0.1)
    tyBound5 = np.full((len(txBound5),), bound5[0]) 


