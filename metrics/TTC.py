"""

Time-To-Collision Metric:

Calculate TTC longitudinally  for two vehicles (leading and following vehicles) moving on the same lane,
when the following vehicle is moving at a speed higher than the leading vehilce's speed

#### negative TTC ####
Paper: New Algorithms for Computing the Time-to-Collision in Freeway Traffic Simulation Models
https://downloads.hindawi.com/journals/cin/2014/761047.pdf


         X_f - X_l - l
TTC =  ----------------
          V_f - V_l

#### positive TTC (used) ####
Paper: A general formulation for time-to-collision safety indicator      
https://www.researchgate.net/publication/274348618_A_general_formulation_for_time-to-collision_safety_indicator


         X_l - X_f - l_l
TTC =  -------------------
           V_f - V_l

where X_f, X_l are longitudinal position of the following and leading vehicle,
      V_f, V_l are longitudinal velocities of the following and leading vehicle,
      l_l is the length of the leading vehicle.

"""


import utils.settings as stg




def calc_TTC_metric(x_following, v_following , x_leading,v_leading):
    stg.init()
    l = stg.CAR_LENGTH
    if v_leading >= v_following:
        return float('inf')
    TTC = (x_leading - x_following - l)/(v_following - v_leading)
    return TTC


def calc_TTC_traj(s , v_s , front_obs_s, front_obs_v_s):
    TTCs = []
    for i in range(len(s)):
        ttc = calc_TTC_metric(s[i],v_s[i],front_obs_s[i],front_obs_v_s[i])
        TTCs.append(ttc)
    return TTCs

if __name__ == '__main__':
    xf = 0
    vf = 30
    xl = 40
    vl = 26
    
    print(calc_TTC_metric(xf,vf,xl,vl))



