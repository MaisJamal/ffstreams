try:
    from Tkinter import Tk, Canvas, Toplevel, LAST
except ModuleNotFoundError:
    from tkinter import Tk, Canvas, Toplevel, LAST

import numpy as np
ARRAY = np.array

import matplotlib.pyplot as plt
import tkinter as tk

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

#### environment parameters
ENV_WIDTH = 200 #2 #100  # meters
LANE_WIDTH = 3.5#0.1 #3.5 # meters
ROBOT_RADIUS = 2.0  # robot radius [m]

######

class PRMViewer(object):
    def __init__(self, width=500, height=500, title='PRM', background='tan'):
        tk = Tk()
        tk.withdraw()
        top = Toplevel(tk)
        top.wm_title(title)
        #top.protocol('WM_DELETE_WINDOW', top.destroy)
        self.width = width
        self.height = height
        #self.canvas = Canvas(top, width=self.width, height=self.height, background=background)
        self.canvas = Canvas(top, width=1000, height=self.height, background=background)
        self.canvas.pack()

    def pixel_from_point(self, point):
        global ENV_WIDTH
        (x, y) = point
        # return (int(x*self.width), int(self.height - y*self.height))
        return (x * self.width/ENV_WIDTH*2, self.height - y * self.height/ENV_WIDTH*2)

    def draw_point(self, point, radius=5):
        (x, y) = self.pixel_from_point(point)
        self.canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill='black')

    def draw_line(self, segment):
        (point1, point2) = segment
        (x1, y1) = self.pixel_from_point(point1)
        (x2, y2) = self.pixel_from_point(point2)
        self.canvas.create_line(x1, y1, x2, y2, fill='black', width=2)

    def draw_line_boundery(self, segment):
        (point1, point2) = segment
        (x1, y1) = self.pixel_from_point(point1)
        (x2, y2) = self.pixel_from_point(point2)
        self.canvas.create_line(x1, y1, x2, y2, fill='pink', width=2)

    def draw_arrow(self, point1, point2):
        (x1, y1) = self.pixel_from_point(point1)
        (x2, y2) = self.pixel_from_point(point2)
        self.canvas.create_line(x1, y1, x2, y2, fill='black', width=2, arrow=LAST)

    def draw_rectangle(self, box, width=2, color='brown'):
        (point1, point2) = box
        (x1, y1) = self.pixel_from_point(point1)
        (x2, y2) = self.pixel_from_point(point2)
        self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, width=width)

    def draw_circle(self, center, radius, width=2, color='black'):
        (x1, y1) = self.pixel_from_point(np.array(center) - radius * np.ones(2))
        (x2, y2) = self.pixel_from_point(np.array(center) + radius * np.ones(2))
        self.canvas.create_oval(x1, y1, x2, y2, outline='black', fill=color, width=width)

    def clear(self):
        self.canvas.delete('all')


#################################################################

def get_delta(q1, q2):
    return np.array(q2) - np.array(q1)

def get_distance(q1, q2):
    return np.linalg.norm(get_delta(q1, q2))


######## added by Mais ######
def get_distance_time(q1,q2):
    #delta = np.array(q2) - np.array(q1)
    euc_distance = np.linalg.norm(get_delta(q1[0:2], q2[0:2]))  # euclidean distance
    avg_speed = (q1[2]+q2[2])/2   #average speed
    return  euc_distance/avg_speed  # cost is the time, d/v
#############################


def contains(q, box):
    (lower, upper) = box
    return np.greater_equal(q, lower).all() and np.greater_equal(upper, q).all()
    #return np.all(q >= lower) and np.all(upper >= q)

def sample_line(segment, step_size=.02):
    (q1, q2) = segment
    diff = get_delta(q1, q2)
    dist = np.linalg.norm(diff)
    for l in np.arange(0., dist, step_size):
        yield tuple(np.array(q1) + l * diff / dist)
    yield q2


def line_collides(line, box):  # TODO - could also compute this exactly
    """
    global ROBOT_RADIUS
    upper_line = np.array(line)
    upper_line[0][1] +=   ROBOT_RADIUS 
    upper_line[1][1] +=   ROBOT_RADIUS 
        
    lower_line = np.array(line)
    lower_line[0][1] -=   ROBOT_RADIUS 
    lower_line[1][1] -=   ROBOT_RADIUS 

    tuple_upper = (ARRAY(upper_line[0]),ARRAY(upper_line[1]))
    tuple_lower = (ARRAY(lower_line[0]),ARRAY(lower_line[1]))
    """
    #bool = any(contains(p, box) for p in sample_line(tuple_upper))
    #bool = bool and any(contains(p, box) for p in sample_line(tuple_lower))
    return any(contains(p, box) for p in sample_line(line))#bool and any(contains(p, box) for p in sample_line(line))


def is_collision_free(line, boxes):
    return not any(line_collides(line, box) for box in boxes)


def create_box(center, extents):
    (x, y) = center
    (w, h) = extents
    lower = (x - w / 2., y - h / 2.)
    upper = (x + w / 2., y + h / 2.)
    return np.array(lower), np.array(upper)

def get_rect(center, extents):
    (x, y) = center
    (w, h) = extents
    lower = (x - w / 2., y - h / 2.)
    return np.array(lower),np.array(w),np.array(h)

def sample_box(box):
    (lower, upper) = box
    return np.random.random(2) * (upper - lower) + lower

def inf_sequence():
    return iter(int, 1)

def draw_environment(obstacles, regions):
    viewer = PRMViewer()
    for box in obstacles:
        viewer.draw_rectangle(box, color='brown')
    for name, region in regions.items():
        if name != 'env':
            viewer.draw_rectangle(region, color='green')
    return viewer

def draw_solution(segments, obstacles, regions):
    viewer = draw_environment(obstacles, regions)
    if segments is None:
        return
    ###Mais  lane has a slope
    """
    right_lane_bound = (ARRAY([0.1, 0]),ARRAY([1, 0.9]))     # x - y = 0.1
    middle_lane_bound = (ARRAY([0, 0.1]),ARRAY([0.9, 1]))    # x - y = -0.1
    left_lane_bound = (ARRAY([0, 0.3]),ARRAY([0.7, 1]))      # x - y = - 0.3
    """
    ##### lane is horizontal : y = ENV_WIDTH/4
    right_lane_bound = (ARRAY([0, ENV_WIDTH/4 - LANE_WIDTH]),ARRAY([ENV_WIDTH , ENV_WIDTH/4 - LANE_WIDTH]))
    middle_lane_bound = (ARRAY([0, ENV_WIDTH/4 ]),ARRAY([ENV_WIDTH, ENV_WIDTH/4]))
    left_lane_bound = (ARRAY([0, ENV_WIDTH/4 + LANE_WIDTH]),ARRAY([ENV_WIDTH , ENV_WIDTH/4 + LANE_WIDTH]))
    #########
    viewer.draw_line_boundery(right_lane_bound)
    viewer.draw_line_boundery(middle_lane_bound)
    viewer.draw_line_boundery(left_lane_bound)
    ####
    for line in segments:
        viewer.draw_line(line)
        #for p in [p1, p2]:
        for p in sample_line(line):
            viewer.draw_point(p, radius=2)


def draw_roadmap(roadmap, obstacles, regions):
    viewer = draw_environment(obstacles, regions)
    ###Mais  lane has a slope
    """
    right_lane_bound = (ARRAY([0.1, 0]),ARRAY([1, 0.9]))     # x - y = 0.1
    middle_lane_bound = (ARRAY([0, 0.1]),ARRAY([0.9, 1]))    # x - y = -0.1
    left_lane_bound = (ARRAY([0, 0.3]),ARRAY([0.7, 1]))      # x - y = - 0.3
    """
    right_lane_bound = (ARRAY([0, ENV_WIDTH/4 - LANE_WIDTH]),ARRAY([ENV_WIDTH , ENV_WIDTH/4 - LANE_WIDTH]))
    middle_lane_bound = (ARRAY([0, ENV_WIDTH/4 ]),ARRAY([ENV_WIDTH, ENV_WIDTH/4]))
    left_lane_bound = (ARRAY([0, ENV_WIDTH/4 + LANE_WIDTH]),ARRAY([ENV_WIDTH , ENV_WIDTH/4 + LANE_WIDTH]))
    #########
    viewer.draw_line_boundery(right_lane_bound)
    viewer.draw_line_boundery(middle_lane_bound)
    viewer.draw_line_boundery(left_lane_bound)
    ####
    for line in roadmap:
        viewer.draw_line(line)

######## added by Mais  ###### the configuration q is [x,y,v]


def draw_roadmap_3d(roadmap, obstacles, regions):
    viewer = draw_environment(obstacles, regions)
    ###Mais  lane has a slope
    """
    right_lane_bound = (ARRAY([0.1, 0]),ARRAY([1, 0.9]))     # x - y = 0.1
    middle_lane_bound = (ARRAY([0, 0.1]),ARRAY([0.9, 1]))    # x - y = -0.1
    left_lane_bound = (ARRAY([0, 0.3]),ARRAY([0.7, 1]))      # x - y = - 0.3
    """
    ##### lane is horizontal : y = 0.5
    right_lane_bound = (ARRAY([0, ENV_WIDTH/4 - LANE_WIDTH]),ARRAY([ENV_WIDTH , ENV_WIDTH/4 - LANE_WIDTH]))
    middle_lane_bound = (ARRAY([0, ENV_WIDTH/4 ]),ARRAY([ENV_WIDTH, ENV_WIDTH/4]))
    left_lane_bound = (ARRAY([0, ENV_WIDTH/4 + LANE_WIDTH]),ARRAY([ENV_WIDTH , ENV_WIDTH/4 + LANE_WIDTH]))
    #########



    viewer.draw_line_boundery(right_lane_bound)
    viewer.draw_line_boundery(middle_lane_bound)
    viewer.draw_line_boundery(left_lane_bound)
    ####
    for line in roadmap:
        line_1 = np.array(line)
        #print("line is ", line_1)
        tuple = (ARRAY(line_1[0][0:2]),ARRAY(line_1[1][0:2]))
        #print(tuple)
        viewer.draw_line(tuple)


def draw_solution_3d(segments, obstacles, regions):
    viewer = draw_environment(obstacles, regions)
    if segments is None:
        return
    ###Mais  lane has a slope : x - y = -0.1
    """           
    right_lane_bound = (ARRAY([0.1, 0]),ARRAY([1, 0.9]))
    middle_lane_bound = (ARRAY([0, 0.1]),ARRAY([0.9, 1]))
    left_lane_bound = (ARRAY([0, 0.3]),ARRAY([0.7, 1]))
    """
    ##### lane is horizontal : y = 0.5
    right_lane_bound = (ARRAY([0, ENV_WIDTH/4 - LANE_WIDTH]),ARRAY([ENV_WIDTH , ENV_WIDTH/4 - LANE_WIDTH]))
    middle_lane_bound = (ARRAY([0, ENV_WIDTH/4 ]),ARRAY([ENV_WIDTH, ENV_WIDTH/4]))
    left_lane_bound = (ARRAY([0, ENV_WIDTH/4 + LANE_WIDTH]),ARRAY([ENV_WIDTH , ENV_WIDTH/4 + LANE_WIDTH]))
    #########
    viewer.draw_line_boundery(right_lane_bound)
    viewer.draw_line_boundery(middle_lane_bound)
    viewer.draw_line_boundery(left_lane_bound)

    #viewer2 = draw_environment([], {})
    #fig, ax = plt.subplots()
    #plt.xlabel('Time')
    #plt.ylabel('Velocity')
    
    
    ####
    for line in segments:
        line_1 = np.array(line)
        tuple = (ARRAY(line_1[0][0:2]),ARRAY(line_1[1][0:2]))
        #print(tuple)
        viewer.draw_line(tuple)
        #for p in [p1, p2]:
        for p in sample_line(tuple):
            viewer.draw_point(p, radius=2)
    
    
 #####################################
 #euc_distance = np.linalg.norm(get_delta(q1[0:2], q2[0:2]))  # euclidean distance
  #  avg_speed = (q1[2]+q2[2])/2   #average speed
  #  return  euc_distance/avg_speed  # cost is the time, d/v

def plot_graph(segments):
    counter = 0
    xs = np.array([0])
    times = np.array([0])
    speeds = np.array([0])
    accs =  np.array([])#np.array([0])
    for line in segments:
        line1 = np.array(line)
        if counter == 0 :
            speeds = np.array([line1[0][2]])    
        xs = np.append(xs, np.array([line1[1][0]]), axis=0)
        next_speed = line1[1][2]
        avg_speed = (next_speed + speeds[-1])/2   #average speed
        speeds = np.append(speeds, np.array([next_speed]), axis=0)
        delta_t = (np.abs(line1[1][0]-line1[0][0]))/avg_speed
        next_t = times[-1] + delta_t
        times = np.append(times, np.array([next_t]) ,axis=0)

        acc = (next_speed - speeds[-2])/delta_t
        accs = np.append(accs,np.array([acc]), axis= 0)
        counter += 1

    #for i in range(speeds.size-1):
       # acc = speeds[i+1] - speeds[i]
        #accs = np.append(accs,np.array([acc]), axis= 0)
    """
    new_accs = np.array([0])
    ts = np.arange(times[-1])
    k = 0
    for t in ts:
        while t > times[k+1]:
            k += 1
        x_t = [times[k], times[k]]
        y_v = [speeds[k], speeds[k]]
        y_v_new = np.interp(t, x_t, y_v)
        new_accs = np.append(new_accs,np.array([y_v_new]), axis= 0)
    """

    root= tk.Tk() 

    fig = plt.Figure(figsize=(5,4), dpi=100)
    ax1 = fig.add_subplot(111)
    line = FigureCanvasTkAgg(fig,root)
    line.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
    ax1.set_title('Speed Vs. Longitude  ')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('v (m/s)')
    ax1.plot(xs, speeds,'-o')
   # plt.show


    fig2 = plt.Figure(figsize=(5,4), dpi=100)
    ax2 = fig2.add_subplot(111)
    line2 = FigureCanvasTkAgg(fig2,root)
    line2.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
    ax2.set_title('Speed Vs. Time  ')
    ax2.set_xlabel('t (s)')
    ax2.set_ylabel('v (m/s)')
    ax2.plot(times, speeds,'-o')


    fig3 = plt.Figure(figsize=(5,4), dpi=100)
    ax3 = fig3.add_subplot(111)
    line3 = FigureCanvasTkAgg(fig3,root)
    line3.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
    ax3.set_title('Acceleration Vs. Time  ')
    ax3.set_xlabel('t (s)')
    ax3.set_ylabel('a (m/ss)')
    #print("accs : ", accs)
    #print("times: ", times)
    for iter in range(times.size-1):
        ax3.plot([times[iter],times[iter+1]],[accs[iter],accs[iter]],'b')
        if iter + 1 < accs.size :
            ax3.plot([times[iter+1],times[iter+1]],[accs[iter],accs[iter+1]],'b')
    #ax3.plot([0, -1.2],[0.5,-1.2])
    #ax3.plot([0, 1.9],[0.5,1.9])

    """
    fig4 = plt.Figure(figsize=(5,4), dpi=100)
    ax4 = fig4.add_subplot(111)
    line4 = FigureCanvasTkAgg(fig4,root)
    line4.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
    ax4.set_title('Acceleration Vs. Time  ')
    ax4.set_xlabel('t (s)')
    ax4.set_ylabel('a (m/ss)')
    ax4.plot(ts, new_accs[1:19],'-o')
    """
    #root.mainloop()
