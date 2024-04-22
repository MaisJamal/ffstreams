"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""

import numpy as np

from numpy import arange
import random

import matplotlib.pyplot as plt
import copy
import math
import sys
import os
import time

from numpy.lib.nanfunctions import _nanpercentile_dispatcher

import ffstreams.utils.settings as stg

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../QuinticPolynomialsPlanner/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../CubicSpline/")

try:
    from ffstreams.ffstreams.quintic_polynomials_planner import QuinticPolynomial
    import ffstreams.ffstreams.cubic_spline_planner
except ImportError:
    raise

SIM_LOOP = 500

# Parameter
MAX_SPEED = 40.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 1.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
D_ROAD_W = 1.0  # road width sampling length [m]
DT = 0.2  # time tick [s]
MAX_T = 10.0  # max prediction time [m]
MIN_T = 4.0  # min prediction time [m]
TARGET_SPEED = 20.0 / 3.6  # target speed [m/s]
### added by me
TARGET_L = 1
DELTA_TARGET_S = 10
LANE_WIDTH = 3.4
#####
D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 2.0  # robot radius [m]

# cost weights
K_J = 0.1
K_T = 0.9
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0

show_animation = False


class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


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


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []
    #print("dfa ",min_time,max_time)
    # generate path to each offset goal
    #for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):
    di = TARGET_L 
    # Lateral motion planning
    for Ti in np.arange(MIN_T, MAX_T, DT):
        fp = FrenetPath()

        # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
        #print (c_d, di)
        lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

        fp.t = [t for t in np.arange(0.0, Ti, DT)]
        fp.d = [lat_qp.calc_point(t) for t in fp.t]
        fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
        fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
        fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

        # Longitudinal motion planning (Velocity keeping)
        for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                            TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
            #print(tv)
            #start_t =time.time()
            tfp = copy.deepcopy(fp)
            #print("time needed for copy: ", time.time()-start_t)
            #start_t =time.time()
            lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)
            #print("time needed for quarticpoly: ", time.time()-start_t)
            #start_t =time.time()
            tfp.s = [lon_qp.calc_point(t) for t in fp.t]
            tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]
            #print("time needed for derivatives calculation: ", time.time()-start_t)

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            # square of diff from target speed
            ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

            #  tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
            tfp.cd = K_J * Jp + K_T * Ti + K_D * (tfp.d[-1]-TARGET_L) ** 2
            tfp.cv = K_J * Js + K_T * Ti + K_D * ds
            tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

            frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ######
            if csp == 1:
                ix, iy = fp.s[i] , stg.wy_middle_lower_lane[0]
            else:
                ix, iy = fp.s[i] , stg.wy_middle_upper_lane[0]
            #####
            #ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = 0 #csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.hypot(dx, dy))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):
    if ob is None:
        return True
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            continue
        #elif not check_collision(fplist[i], ob):
        #    continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
    #start_t = time.time()
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    #print("time needed for calc fre paths: ", time.time()-start_t)

    #start_t = time.time()
    fplist = calc_global_paths(fplist, csp)
    #print("time needed for calc global paths: ", time.time()-start_t)

    #start_t = time.time()
    fplist = check_paths(fplist, ob)
    #print("time needed for check paths: ", time.time()-start_t)

    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return best_path



def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))
    return rx, ry, ryaw, rk, csp

def generate_target_course_edited(x, y):
    #csp = cubic_spline_planner.Spline2D(x, y)
    #s = np.arange(0, csp.s[-1], 0.1)

    #rx, ry, ryaw, rk = [], [], [], []
    #for i_s in s:
    #    ix, iy = csp.calc_position(i_s)
    #    rx.append(ix)
     #   ry.append(iy)
    #    ryaw.append(csp.calc_yaw(i_s))
    #    rk.append(csp.calc_curvature(i_s))
    rx = np.arange(0, x[-1], 0.1)
    ry = np.full((len(rx),), y[0])
    #print("rx: ",rx)
    #print("ry: ",ry)
    #print("r yaw: ",ryaw)
    #print("rk: ",rk)
    #print("csp: ",csp)
    #input("continue?")
    return rx, ry

def calc_ds_l0(x_target,y_target,x0,y0): # this is a special case when the road is placed horizontaly 
    delta_s = x_target - x0
    if(y_target==stg.wy_middle_lower_lane[0]):
        l0 = y0 - stg.wy_middle_lower_lane[0]
    else:
        l0 = y0 - stg.wy_middle_upper_lane[0]
    return delta_s, l0


def calc_s_l(x,y,tx): # this is a special case when the road is placed horizontaly 
    s = (x - tx[1] ) / 0.1
    if y > LANE_WIDTH/2 :
        l = LANE_WIDTH/2
    elif y < -1* LANE_WIDTH/2 :
        l = -1* LANE_WIDTH/2
    else:
        l = y
    return s, l


def get_traj(x0,y0,speed0,target_x,target_y,target_speed):

    ob = None 
    traj = FrenetPath()
    #traj.t = [0]
    last_t = 0
    #tx, ty = stg.tx, stg.ty#generate_target_course_edited(stg.wx_middle_lane, stg.wy_middle_lower_lane)
    #tx2, ty2 = stg.tx2, stg.ty2#generate_target_course_edited(stg.wx_middle_lane, stg.wy_middle_upper_lane)
    #course bounds 
    #txBound1, tyBound1 = generate_target_course_edited(stg.wx_middle_lane, stg.bound1)
    #txBound2, tyBound2 = generate_target_course_edited(stg.wx_middle_lane, stg.bound2)
    #txBound3, tyBound3 = generate_target_course_edited(stg.wx_middle_lane, stg.bound3)

    if (target_y==stg.wy_middle_lower_lane[0]):
        csp = 1
        #print("csp1",stg.wy_middle_lower_lane[0])
    else:
        csp = 2
    global TARGET_SPEED
    global TARGET_L
    TARGET_SPEED = target_speed
    global DELTA_TARGET_S
    #print("target x: ",target_x)
    #print("target y: ",target_y)
    #print("tx: ",tx)
    DELTA_TARGET_S, c_d = calc_ds_l0(target_x,target_y,x0,y0)
    TARGET_L = 0
    #print("Target delta s, l are: ",DELTA_TARGET_S,", ",c_d)
    #course bounds 

    show_animation = False
    # initial state
    c_speed = speed0  # current speed [m/s]
    
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = x0  # current course position

    area = 30.0  # animation area length [m]

    path = frenet_optimal_planning(
                csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
    if path is None:
        return False,None
    #print(path.x)
    #print("length x ",len(path.x))
     
    if show_animation:  # pragma: no cover
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(stg.tx, stg.ty,"y--")
        plt.plot(stg.tx2, stg.ty2,"y--")
        #plt.plot(tx3, ty3,"y--")
        plt.plot(stg.txBound1, stg.tyBound1,"c")
        plt.plot(stg.txBound2, stg.tyBound2,"c")
        plt.plot(stg.txBound3, stg.tyBound3,"c")
        #plt.plot(txBound4, tyBound4,"c")

        #plt.plot(ob[:, 0], ob[:, 1], "xk")
        plt.plot(path.x[1:], path.y[1:], "-r")
        plt.plot(path.x[1], path.y[1], "vc")
        plt.xlim(path.x[1] - area, path.x[1] + area + 10)
        plt.ylim(path.y[1] - area, path.y[1] + area)
        plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
        plt.grid(True)
        plt.pause(0.0001)
    
    #print("first path speed: ",path.s_d)
    #print("path t: ", path.t)
    #print("path x: ", path.x)
    #print("path y: ", path.y)
    #input('Continue?')
    final_path = False
    first_path = True
    kk = 1
    while not final_path:
        #print("k = ", kk)
        kk+=1
        for j in range(len(path.x)):
            #print("j = ", j)
            #print("path y j=0 , target y: ",path.y[j] , target_y)
            if path.x[j] >= target_x :
                final_path = True
                if abs(path.y[j] - target_y)<=0.2:#np.hypot(path.x[j] - target_x, path.y[j] - target_y) <= 1.0:
                    #print("Goal was reached")
                    if abs(TARGET_SPEED - path.s_d[j]) <= 0.2:
                        #print("Goal speed was reached")
                        traj.x = np.concatenate((traj.x, path.x[1:j+1]), axis=0) 
                        traj.y = np.concatenate((traj.y, path.y[1:j+1]), axis=0) 
                        traj.s = np.concatenate((traj.s, path.s[1:j+1]), axis=0)
                        traj.s_d = np.concatenate((traj.s_d, path.s_d[1:j+1]), axis=0) 
                        traj.s_dd = np.concatenate((traj.s_dd, path.s_dd[1:j+1]), axis=0)  
                        traj.s_ddd = np.concatenate((traj.s_ddd, path.s_ddd[1:j+1]), axis=0) 
                        traj.d = np.concatenate((traj.d, path.d[1:j+1]), axis=0) 
                        traj.d_d = np.concatenate((traj.d_d, path.d_d[1:j+1]), axis=0)
                        traj.d_dd = np.concatenate((traj.d_dd, path.d_dd[1:j+1]), axis=0)
                        traj.d_ddd = np.concatenate((traj.d_ddd, path.d_ddd[1:j+1]), axis=0)

                        traj.t = np.concatenate((traj.t, [i+last_t for i in path.t[1:j+1]]),axis = 0)
                        #traj.x = np.concatenate((traj.x, [target_x]), axis=0)
                        #traj.y = np.concatenate((traj.y, [target_y]), axis=0)
                        #traj.s_d = np.concatenate((traj.s_d, [target_speed]), axis=0)
                        #traj.t = np.concatenate((traj.t, [traj.t[-1]+DT]),axis=0)
                        
                        #print("traj speed: ",traj.s_d)
                        #print("traj t: ", traj.t)
                        #print("length of traj t: ", len(traj.t))
                        #print("length of traj x: ", len(traj.x))
                        #print("length of traj y: ", len(traj.y))
                        #input("continue?")
                        #print("j is ",j)
                        if show_animation:  # pragma: no cover
                            plt.cla()
                            # for stopping simulation with the esc key.
                            plt.gcf().canvas.mpl_connect(
                                'key_release_event',
                                lambda event: [exit(0) if event.key == 'escape' else None])
                            plt.plot(stg.tx, stg.ty,"y--")
                            plt.plot(stg.tx2, stg.ty2,"y--")
                            plt.plot(stg.txBound1, stg.tyBound1,"c")
                            plt.plot(stg.txBound2, stg.tyBound2,"c")
                            plt.plot(stg.txBound3, stg.tyBound3,"c")
                            if ob is not None:
                                plt.plot(ob[:, 0], ob[:, 1], "xk")
                            plt.plot(traj.x[0:], traj.y[0:], "-r")
                            plt.plot(traj.x[0], traj.y[0], "vc")
                            plt.xlim(0, 200)
                            plt.ylim(traj.y[1] - area, traj.y[1] + area)
                            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
                            plt.grid(True)
                            plt.pause(0.0001)
                        return True,traj
                    else:
                        #print("end speed is ",path.s_d[j])
                        #print("target speed is ",TARGET_SPEED)
                        #print("Goal was reached But SPEED was NOT")
                        return False,None
                else:
                    return False,None
       
        if first_path:
            traj.t = np.concatenate((traj.t,  path.t),axis = 0)

            traj.x = np.concatenate((traj.x, path.x), axis=0) 
            traj.y = np.concatenate((traj.y, path.y), axis=0) 
            traj.s = np.concatenate((traj.s, path.s), axis=0)
            traj.s_d = np.concatenate((traj.s_d, path.s_d), axis=0) 
            traj.s_dd = np.concatenate((traj.s_dd, path.s_dd), axis=0)  
            traj.s_ddd = np.concatenate((traj.s_ddd, path.s_ddd), axis=0) 
            traj.d = np.concatenate((traj.d, path.d), axis=0) 
            traj.d_d = np.concatenate((traj.d_d, path.d_d), axis=0)
            traj.d_dd = np.concatenate((traj.d_dd, path.d_dd), axis=0)
            traj.d_ddd = np.concatenate((traj.d_ddd, path.d_ddd), axis=0)

            first_path = False
        else:
            traj.t = np.concatenate((traj.t, [i+last_t for i in path.t[1:]]),axis = 0)

            traj.x = np.concatenate((traj.x, path.x[1:]), axis=0) 
            traj.y = np.concatenate((traj.y, path.y[1:]), axis=0) 
            traj.s = np.concatenate((traj.s, path.s[1:]), axis=0)
            traj.s_d = np.concatenate((traj.s_d, path.s_d[1:]), axis=0) 
            traj.s_dd = np.concatenate((traj.s_dd, path.s_dd[1:]), axis=0)  
            traj.s_ddd = np.concatenate((traj.s_ddd, path.s_ddd[1:]), axis=0) 
            traj.d = np.concatenate((traj.d, path.d[1:]), axis=0) 
            traj.d_d = np.concatenate((traj.d_d, path.d_d[1:]), axis=0)
            traj.d_dd = np.concatenate((traj.d_dd, path.d_dd[1:]), axis=0)
            traj.d_ddd = np.concatenate((traj.d_ddd, path.d_ddd[1:]), axis=0)

        last_t = path.t[-1]+last_t #traj.t[-1]
        #print("with addition of last t: ",last_t," : ",[i+last_t for i in path.t[0:-1]])
        s0 = path.s[-1]
        c_d = path.d[-1]
        c_d_d = path.d_d[-1]
        c_d_dd = path.d_dd[-1]
        c_speed = path.s_d[-1]
        path = frenet_optimal_planning(
                csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
        #print("path speed: ",path.s_d)
        #print("path t: ", path.t)
        #print("path x: ", path.x)
       # print("path y: ", path.y)
       # input('Continue?')
        if path is None:
            return False,None
    return False,None


def get_traj_change_lane(x0,y0,speed0,acc0,curr_dl,curr_ddl,target_x,target_y,target_speed):
    global MAX_SPEED
    global MAX_ACCEL
    global MAX_T
    global MIN_T 

    MAX_T = 10
    MIN_T = 4

    MAX_SPEED = 120.0 / 3.6  # maximum speed [m/s]
    MAX_ACCEL = 1  # maximum acceleration [m/ss]
    ob = None 
    traj = FrenetPath()
    #traj.t = [0]
    last_t = 0
    #tx, ty = stg.tx, stg.ty#generate_target_course_edited(stg.wx_middle_lane, stg.wy_middle_lower_lane)
    #tx2, ty2 = stg.tx2, stg.ty2#generate_target_course_edited(stg.wx_middle_lane, stg.wy_middle_upper_lane)
    #course bounds 
    #txBound1, tyBound1 = generate_target_course_edited(stg.wx_middle_lane, stg.bound1)
    #txBound2, tyBound2 = generate_target_course_edited(stg.wx_middle_lane, stg.bound2)
    #txBound3, tyBound3 = generate_target_course_edited(stg.wx_middle_lane, stg.bound3)

    if (target_y==stg.wy_middle_lower_lane[0]):
        csp = 1
        #print("csp1",stg.wy_middle_lower_lane[0])
    else:
        csp = 2
    global TARGET_SPEED
    global TARGET_L
    TARGET_SPEED = target_speed
    global DELTA_TARGET_S
    #print("target x: ",target_x)
    #print("target y: ",target_y)
    #print("tx: ",tx)
    DELTA_TARGET_S, c_d = calc_ds_l0(target_x,target_y,x0,y0)
    TARGET_L = 0
    #print("Target delta s, l are: ",DELTA_TARGET_S,", ",c_d)
    #course bounds 

    show_animation = False
    show_final_traj = False # important

    # initial state
    c_speed = speed0  # current speed [m/s]
    
    c_d_d = curr_dl # current lateral speed [m/s]
    c_d_dd = curr_ddl  # current lateral acceleration [m/s]
    s0 = x0  # current course position
    s_dd = acc0
    area = 30.0  # animation area length [m]

    path = frenet_optimal_planning_corrected(
                csp, s0, c_speed, s_dd, c_d, c_d_d, c_d_dd, ob)
    if path is None:
        return False,None
    #print(path.x)
    #print("length x ",len(path.x))
     
    if show_animation:  # pragma: no cover
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(stg.tx, stg.ty,"y--")
        plt.plot(stg.tx2, stg.ty2,"y--")
        #plt.plot(tx3, ty3,"y--")
        plt.plot(stg.txBound1, stg.tyBound1,"c")
        plt.plot(stg.txBound2, stg.tyBound2,"c")
        plt.plot(stg.txBound3, stg.tyBound3,"c")
        #plt.plot(txBound4, tyBound4,"c")

        #plt.plot(ob[:, 0], ob[:, 1], "xk")
        plt.plot(path.x[1:], path.y[1:], "-r")
        plt.plot(path.x[1], path.y[1], "vc")
        plt.xlim(path.x[1] - area, path.x[1] + area + 10)
        plt.ylim(path.y[1] - area, path.y[1] + area)
        plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
        plt.grid(True)
        plt.pause(0.0001)
    
    #print("first path speed: ",path.s_d)
    #print("path t: ", path.t)
    #print("path x: ", path.x)
    #print("path y: ", path.y)
    #input('Continue?')
    final_path = False
    first_path = True
    kk = 1
    while not final_path:
        #print("k = ", kk)
        kk+=1
        for j in range(len(path.x)):
            #print("j = ", j)
            #print("path y j=0 , target y: ",path.y[j] , target_y)
            #if path.x[j] >= target_x :
                
                if abs(path.y[j] - target_y)<=0.01:#np.hypot(path.x[j] - target_x, path.y[j] - target_y) <= 1.0:
                    final_path = True
                    #print("Goal was reached")
                    if abs(TARGET_SPEED - path.s_d[j]) <= 0.2:
                        #print("Goal speed was reached")
                        if not first_path:
                            traj.x = np.concatenate((traj.x, path.x[1:j+1]), axis=0) 
                            traj.y = np.concatenate((traj.y, path.y[1:j+1]), axis=0) 
                            traj.s = np.concatenate((traj.s, path.s[1:j+1]), axis=0)
                            traj.s_d = np.concatenate((traj.s_d, path.s_d[1:j+1]), axis=0) 
                            traj.s_dd = np.concatenate((traj.s_dd, path.s_dd[1:j+1]), axis=0)  
                            traj.s_ddd = np.concatenate((traj.s_ddd, path.s_ddd[1:j+1]), axis=0) 
                            traj.d = np.concatenate((traj.d, path.d[1:j+1]), axis=0) 
                            traj.d_d = np.concatenate((traj.d_d, path.d_d[1:j+1]), axis=0)
                            traj.d_dd = np.concatenate((traj.d_dd, path.d_dd[1:j+1]), axis=0)
                            traj.d_ddd = np.concatenate((traj.d_ddd, path.d_ddd[1:j+1]), axis=0)
                            traj.yaw = np.concatenate((traj.yaw, path.yaw[1:j+1]), axis=0)

                            traj.t = np.concatenate((traj.t, [i+last_t for i in path.t[1:j+1]]),axis = 0)
                        else:
                            traj.x = path.x[0:j+1]
                            traj.y = path.y[0:j+1]
                            traj.s = path.s[0:j+1]
                            traj.s_d = path.s_d[0:j+1]
                            traj.s_dd = path.s_dd[0:j+1]
                            traj.s_ddd = path.s_ddd[0:j+1]
                            traj.d = path.d[0:j+1]
                            traj.d_d = path.d_d[0:j+1]
                            traj.d_dd = path.d_dd[0:j+1]
                            traj.d_ddd = path.d_ddd[0:j+1]
                            traj.yaw = path.yaw[0:j+1]
                            traj.t = path.t[0:j+1]
                        #traj.x = np.concatenate((traj.x, [target_x]), axis=0)
                        #traj.y = np.concatenate((traj.y, [target_y]), axis=0)
                        #traj.s_d = np.concatenate((traj.s_d, [target_speed]), axis=0)
                        #traj.t = np.concatenate((traj.t, [traj.t[-1]+DT]),axis=0)
                        
                        #print("traj speed: ",traj.s_d)
                        #print("traj t: ", traj.t)
                        #print("length of traj t: ", len(traj.t))
                        #print("length of traj x: ", len(traj.x))
                        #print("length of traj y: ", len(traj.y))
                        #input("continue?")
                        #print("j is ",j)
                        
                        if show_final_traj and len(traj.y) > 1:  # pragma: no cover
                            plt.cla()
                            # for stopping simulation with the esc key.
                            plt.gcf().canvas.mpl_connect(
                                'key_release_event',
                                lambda event: [exit(0) if event.key == 'escape' else None])
                            plt.plot(stg.tx, stg.ty,"y--")
                            plt.plot(stg.tx2, stg.ty2,"y--")
                            plt.plot(stg.txBound1, stg.tyBound1,"c")
                            plt.plot(stg.txBound2, stg.tyBound2,"c")
                            plt.plot(stg.txBound3, stg.tyBound3,"c")
                            if ob is not None:
                                plt.plot(ob[:, 0], ob[:, 1], "xk")
                            plt.plot(traj.x[0:], traj.y[0:], "-r")
                            plt.plot(traj.x[0], traj.y[0], "vc")
                            plt.xlim(0, 800)
                            plt.ylim(traj.y[1] - area, traj.y[1] + area)
                            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
                            plt.grid(True)
                            plt.pause(0.1)
                        return True,traj
                    else:
                        #print("end speed is ",path.s_d[j])
                        #print("target speed is ",TARGET_SPEED)
                        #print("Goal was reached But SPEED was NOT")
                        return False,None
                #else:
                #    return False,None
       
        if first_path:
            traj.t = np.concatenate((traj.t,  path.t),axis = 0)

            traj.x = np.concatenate((traj.x, path.x), axis=0) 
            traj.y = np.concatenate((traj.y, path.y), axis=0) 
            traj.s = np.concatenate((traj.s, path.s), axis=0)
            traj.s_d = np.concatenate((traj.s_d, path.s_d), axis=0) 
            traj.s_dd = np.concatenate((traj.s_dd, path.s_dd), axis=0)  
            traj.s_ddd = np.concatenate((traj.s_ddd, path.s_ddd), axis=0) 
            traj.d = np.concatenate((traj.d, path.d), axis=0) 
            traj.d_d = np.concatenate((traj.d_d, path.d_d), axis=0)
            traj.d_dd = np.concatenate((traj.d_dd, path.d_dd), axis=0)
            traj.d_ddd = np.concatenate((traj.d_ddd, path.d_ddd), axis=0)
            traj.yaw = np.concatenate((traj.yaw, path.yaw), axis=0)

            first_path = False
        else:
            traj.t = np.concatenate((traj.t, [i+last_t for i in path.t[1:]]),axis = 0)

            traj.x = np.concatenate((traj.x, path.x[1:]), axis=0) 
            traj.y = np.concatenate((traj.y, path.y[1:]), axis=0) 
            traj.s = np.concatenate((traj.s, path.s[1:]), axis=0)
            traj.s_d = np.concatenate((traj.s_d, path.s_d[1:]), axis=0) 
            traj.s_dd = np.concatenate((traj.s_dd, path.s_dd[1:]), axis=0)  
            traj.s_ddd = np.concatenate((traj.s_ddd, path.s_ddd[1:]), axis=0) 
            traj.d = np.concatenate((traj.d, path.d[1:]), axis=0) 
            traj.d_d = np.concatenate((traj.d_d, path.d_d[1:]), axis=0)
            traj.d_dd = np.concatenate((traj.d_dd, path.d_dd[1:]), axis=0)
            traj.d_ddd = np.concatenate((traj.d_ddd, path.d_ddd[1:]), axis=0)
            traj.yaw = np.concatenate((traj.yaw, path.yaw[1:]), axis=0)

        last_t = path.t[-1]+last_t #traj.t[-1]
        #print("with addition of last t: ",last_t," : ",[i+last_t for i in path.t[0:-1]])
        s0 = path.s[-1]
        c_d = path.d[-1]
        c_d_d = path.d_d[-1]
        c_d_dd = path.d_dd[-1]
        c_speed = path.s_d[-1]
        s_dd = path.s_dd[-1]
        path = frenet_optimal_planning_corrected(
                csp, s0, c_speed, s_dd, c_d, c_d_d, c_d_dd, ob)
        #print("path speed: ",path.s_d)
        #print("path t: ", path.t)
        #print("path x: ", path.x)
       # print("path y: ", path.y)
       # input('Continue?')
        if path is None:
            return False,None
    return False,None


def get_traj_yield(x0,y0,speed0,acc0,curr_dl,curr_ddl,target_x,target_y,target_speed):
    global MAX_SPEED
    global MAX_ACCEL
    global MAX_T
    global MIN_T 

    low_acc = False
    if low_acc:
        MAX_T = 5.1
        MIN_T = 5

        #MAX_T = 10
        #MIN_T = 4


        MAX_SPEED = 120.0 / 3.6  # maximum speed [m/s]
        MAX_ACCEL = 1  # maximum acceleration [m/ss]
    else:
        MAX_T = 5.1
        MIN_T = 5

        MAX_SPEED = 120.0 / 3.6  # maximum speed [m/s]
        MAX_ACCEL = 15  # maximum acceleration [m/ss]

    ob = None 
    

    if (target_y==stg.wy_middle_lower_lane[0]):
        csp = 1
        #print("csp1",stg.wy_middle_lower_lane[0])
    else:
        csp = 2
    global TARGET_SPEED
    global TARGET_L
    TARGET_SPEED = target_speed
    global DELTA_TARGET_S
    
    DELTA_TARGET_S, c_d = calc_ds_l0(target_x,target_y,x0,y0)
    TARGET_L = 0
    

    show_final_traj = False #important

    # initial state
    c_speed = speed0  # current speed [m/s]
    
    c_d_d = curr_dl # current lateral speed [m/s]
    c_d_dd = curr_ddl  # current lateral acceleration [m/s]
    s0 = x0  # current course position
    s_dd = acc0
    area = 30.0  # animation area length [m]

    path = frenet_optimal_planning_corrected(
                csp, s0, c_speed, s_dd, c_d, c_d_d, c_d_dd, ob)
    if path is None:
        return False,None
    else:
        if show_final_traj and len(path.y) > 1:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(stg.tx, stg.ty,"y--")
            plt.plot(stg.tx2, stg.ty2,"y--")
            plt.plot(stg.txBound1, stg.tyBound1,"c")
            plt.plot(stg.txBound2, stg.tyBound2,"c")
            plt.plot(stg.txBound3, stg.tyBound3,"c")
            if ob is not None:
                plt.plot(ob[:, 0], ob[:, 1], "xk")
            plt.plot(path.x[0:], path.y[0:], "-r")
            plt.plot(path.x[0], path.y[0], "vc")
            plt.xlim(0, 800)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.1)
        return True, path
     
    return False,None

def get_traj_follow_speed(x0,y0,speed0,acc0,curr_dl,curr_ddl,target_y,target_speed):
    global MAX_SPEED
    global MAX_ACCEL
    global TARGET_SPEED
    global TARGET_L
    global DELTA_TARGET_S
    global MAX_T
    global MIN_T 
    low_acc = False
    if low_acc:
        MAX_T = 5.1
        MIN_T = 5

        #MAX_T = 10
        #MIN_T = 5
        
        MAX_SPEED = 120.0 / 3.6  # maximum speed [m/s]
        MAX_ACCEL = 1  # maximum acceleration [m/ss]
    else:
        MAX_T = 5.1
        MIN_T = 5
        
        MAX_SPEED = 120.0 / 3.6  # maximum speed [m/s]
        MAX_ACCEL = 15  # maximum acceleration [m/ss]
    ob = None 
    target_x = 20 # any target

    if (target_y==stg.wy_middle_lower_lane[0]):
        csp = 1
        #print("csp1",stg.wy_middle_lower_lane[0])
    else:
        csp = 2
   
    TARGET_SPEED = target_speed
    
    DELTA_TARGET_S, c_d = calc_ds_l0(target_x,target_y,x0,y0)
    TARGET_L = 0
    

    show_final_traj = False #important

    # initial state
    c_speed = speed0  # current speed [m/s]
    
    c_d_d = curr_dl # current lateral speed [m/s]
    c_d_dd = curr_ddl  # current lateral acceleration [m/s]
    s0 = x0  # current course position
    s_dd = acc0
    area = 30.0  # animation area length [m]

    path = frenet_optimal_planning_corrected(
                csp, s0, c_speed, s_dd, c_d, c_d_d, c_d_dd, ob)
    if path is None:
        return False,None
    else:
        if show_final_traj and len(path.y) > 1:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(stg.tx, stg.ty,"y--")
            plt.plot(stg.tx2, stg.ty2,"y--")
            plt.plot(stg.txBound1, stg.tyBound1,"c")
            plt.plot(stg.txBound2, stg.tyBound2,"c")
            plt.plot(stg.txBound3, stg.tyBound3,"c")
            if ob is not None:
                plt.plot(ob[:, 0], ob[:, 1], "xk")
            plt.plot(path.x[0:], path.y[0:], "-r")
            plt.plot(path.x[0], path.y[0], "vc")
            plt.xlim(0, 800)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.1)
        return True, path
     
    return False,None

def get_traj_change_lane_overtake(x0,y0,speed0,acc0,curr_dl,curr_ddl,target_y,target_speed):
    global MAX_SPEED
    global MAX_ACCEL
    global TARGET_SPEED
    global TARGET_L
    global DELTA_TARGET_S
    global MAX_T
    global MIN_T 

    low_acc = False
    if low_acc:
        # MAX_T = 5.1
        # MIN_T = 5

        MAX_T = 10
        MIN_T = 5
        
        MAX_SPEED = 120.0 / 3.6  # maximum speed [m/s]
        MAX_ACCEL = 1  # maximum acceleration [m/ss]
    else:
        MAX_T = 3
        MIN_T = 2
        
        MAX_SPEED = 120.0 / 3.6  # maximum speed [m/s]
        MAX_ACCEL = 15  # maximum acceleration [m/ss]

    ob = None 
    target_x = 20 # any target
    

    if (target_y==stg.wy_middle_lower_lane[0]):
        csp = 1
        #print("csp1",stg.wy_middle_lower_lane[0])
    else:
        csp = 2
   
    TARGET_SPEED = target_speed
    
    DELTA_TARGET_S, c_d = calc_ds_l0(target_x,target_y,x0,y0)
    TARGET_L = 0
    

    show_final_traj = False #important

    # initial state
    c_speed = speed0  # current speed [m/s]
    
    c_d_d = curr_dl # current lateral speed [m/s]
    c_d_dd = curr_ddl  # current lateral acceleration [m/s]
    s0 = x0  # current course position
    s_dd = acc0
    area = 30.0  # animation area length [m]

    path = frenet_optimal_planning_corrected(
                csp, s0, c_speed, s_dd, c_d, c_d_d, c_d_dd, ob)
    if path is None:
        return False,None
    else:
        if show_final_traj and len(path.y) > 1:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(stg.tx, stg.ty,"y--")
            plt.plot(stg.tx2, stg.ty2,"y--")
            plt.plot(stg.txBound1, stg.tyBound1,"c")
            plt.plot(stg.txBound2, stg.tyBound2,"c")
            plt.plot(stg.txBound3, stg.tyBound3,"c")
            if ob is not None:
                plt.plot(ob[:, 0], ob[:, 1], "xk")
            plt.plot(path.x[0:], path.y[0:], "-r")
            plt.plot(path.x[0], path.y[0], "vc")
            plt.xlim(0, 800)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.1)
        return True, path
     
    return False,None

def frenet_optimal_planning_corrected(csp, s0, c_speed , s_dd, c_d, c_d_d, c_d_dd, ob):
    #start_t = time.time()
    fplist = calc_frenet_paths_corrected(c_speed,s_dd, c_d, c_d_d, c_d_dd, s0)
    #print("time needed for calc fre paths: ", time.time()-start_t)

    #start_t = time.time()
    fplist = calc_global_paths(fplist, csp)
    #print("time needed for calc global paths: ", time.time()-start_t)

    #start_t = time.time()
    fplist = check_paths(fplist, ob)
    #print("time needed for check paths: ", time.time()-start_t)

    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return best_path

def calc_frenet_paths_corrected(c_speed,s_dd, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []
    #print("dfa ",min_time,max_time)
    # generate path to each offset goal
    #for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):
    di = TARGET_L 
    # Lateral motion planning
    for Ti in np.arange(MIN_T, MAX_T, DT):
        fp = FrenetPath()

        # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
        #print (c_d, di)
        lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

        fp.t = [t for t in np.arange(0.0, Ti, DT)]
        fp.d = [lat_qp.calc_point(t) for t in fp.t]
        fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
        fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
        fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

        # Longitudinal motion planning (Velocity keeping)
        for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                            TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
            #print(tv)
            #start_t =time.time()
            tfp = copy.deepcopy(fp)
            #print("time needed for copy: ", time.time()-start_t)
            #start_t =time.time()
            lon_qp = QuarticPolynomial(s0, c_speed, s_dd, tv, 0.0, Ti)
            #print("time needed for quarticpoly: ", time.time()-start_t)
            #start_t =time.time()
            tfp.s = [lon_qp.calc_point(t) for t in fp.t]
            tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]
            #print("time needed for derivatives calculation: ", time.time()-start_t)

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            # square of diff from target speed
            ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

            #  tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
            tfp.cd = K_J * Jp + K_T * Ti + K_D * (tfp.d[-1]-TARGET_L) ** 2
            tfp.cv = K_J * Js + K_T * Ti + K_D * ds
            tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

            frenet_paths.append(tfp)

    return frenet_paths

if __name__ == '__main__':
   # d = arange(10,20,0.5)
    #for i in d:
    #    print(i)
    #print(random.uniform(+1.7,-1.7))
    target_x = 200
    target_y = +0
    x0=0
    y0=50
    speed0 = 20.9 / 3.6
    target_speed = 20.9 / 3.6  # target speed [m/s]
    #main2(target_x,target_y,target_speed)
    get_traj_follow_speed (0.0, 48.25 , 29.0, 0 ,0,0 , 51.75,30)
    #get_traj(x0,y0,speed0,target_x,target_y,target_speed)
    #for i in range(3):
     #   target_x = random.uniform(50,80)
    #    target_y = random.uniform(-1.7,+1.7)
    #    target_speed = (21+random.uniform(+0.0,+0.6)) / 3.6  # target speed [m/s]
     #   print("Target x: ",target_x,", target y: ",target_y,", target speed: ",target_speed)
     #   main(target_x,target_y,target_speed)
    
