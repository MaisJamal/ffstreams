"""

Occupant's Preference Metric:
( longitudinal acceleration threshold,  longitudinal deceleration threshold, lateral acceleration threshold(abs),
    maximum longitudinal jerk(abs), maximum lateral jerk(abs))

Paper: Self-Driving like a Human driver instead of a Robocar: Personalized comfortable driving experience for autonomous vehicles 
https://arxiv.org/pdf/2001.03908.pdf

"""

from ffstreams.ffstreams.frenet_optimizer import FrenetPath


# metric parameter
DRIVING_MODE = {'public_transportation':[0.93, 0.93, 0.93 , 0.6 , 0.6],'normal_driver':[1.47, 2.0 ,4.0 , 0.9 , 0.9],'aggressive_driver':[ 3.07, 5.08, 5.6,2.0,2.0],
             'extremly_aggressive_driver':[7.6,7.6,7.6,2.0,2.0]}

EMMERGENCY_BRAKING_ACC = 5.08  # if acc < -5.08 ==> airbag deployment starts





def calc_OPM_metric(traj):
    
    driving_mode = []

    max_lon_acc = max(traj.s_dd)
    min_lon_acc = abs(min(traj.s_dd))
    max_abs_lat_acc = abs(max(traj.d_dd, key=abs))
    max_abs_lon_jerk = abs(max(traj.s_ddd, key=abs))
    max_abs_lat_jerk = abs(max(traj.d_ddd, key=abs))

    OPM_values = [max_lon_acc,min_lon_acc,max_abs_lat_acc,max_abs_lon_jerk,max_abs_lat_jerk]

    #print("OPM trajectory values: ",OPM_values)

    for i in range(len(OPM_values)):
        # define trajectory driving mode for each value#
        for mode in DRIVING_MODE:

            if OPM_values[i] < DRIVING_MODE[mode][i]:
                driving_mode.append(mode)
                break
        if len(driving_mode) < i+1:
            driving_mode.append('abnormal_driving')

    # define the worst existing driving mode #
    driving_all_modes = ['abnormal_driving' , 'extremly_aggressive_driver' , 'aggressive_driver' , 'normal_driver' , 'public_transportation'] # worst to best
    for m in driving_all_modes :
        if m in driving_mode:
            overall_driving_mode = m 
            break

    return overall_driving_mode


if __name__ == '__main__':
    tr = FrenetPath()
    tr.s_dd = [0.40 , 0.100, -200]
    tr.d_dd = [0.50 , -0.20 ,- 0.30,-0.100]
    tr.s_ddd = [ -0.50 , 0.100, 0.544]
    tr.d_ddd = [-0.6000, 0.50 ,0.6006]
    print(calc_OPM_metric(tr))



