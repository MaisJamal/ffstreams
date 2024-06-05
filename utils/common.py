import numpy as np
import math
from ffstreams.ffstreams import cubic_spline_planner



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


def get_limit_coord_obs(center,height,width,degrees):

    pt1 = [center[0] - width/2, center[1] - height/2]
    pt2 = [center[0] - width/2, center[1] + height/2]
    pt3 = [center[0] + width/2, center[1] + height/2]
    pt4 = [center[0] + width/2, center[1] - height/2]

    rectangle = [pt1, pt2, pt3, pt4, pt1]

    rectangle_rotated = [rotate(center, pt,  math.radians(degrees)) for pt in rectangle]

    rectangle = np.array(rectangle)
    rectangle_rotated = np.array(rectangle_rotated)

    # these are what you need
    x_min, y_min = np.min(rectangle_rotated,axis=0)
    x_max, y_max = np.max(rectangle_rotated,axis=0)

    return x_max,x_min,y_max,y_min

def get_heading(p1_x,p1_y,p2_x,p2_y):
    # get heading in degrees
    heading_radians = math.atan2(p2_y-p1_y, p2_x-p1_x)
    # heading_degrees = math.degrees(heading_radians)
    return heading_radians