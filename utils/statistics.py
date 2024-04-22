
import numpy as np
import pandas as pd
from ffstreams.metrics.TTC import calc_TTC_traj, calc_TTC_traj_lane_change
import os

class Statistics:

    def __init__(self):
        self.t = []
        self.yaw = []
        self.s = []
        self.v_s = []
        self.a_s = []
        self.j_s = []

        self.l = []
        self.v_l = []
        self.a_l = []
        self.j_l = []
        
        self.front_obs_s = []
        self.front_obs_v_s = []
        self.front_obs_l = []
        

        self.other_obs_no = 0
        self.other_obs_s = [] #2d
        self.other_obs_v_s = [] #2d
        self.other_obs_a_s = [] #1d
        self.other_obs_l = [] #2d

        self.decisions = []
        self.TTC = []
    
    def get_TTC(self):
        if len(self.front_obs_s) < 1:
            self.TTC = calc_TTC_traj_lane_change(self.s , self.v_s , self.other_obs_s, self.other_obs_v_s,self.other_obs_no)
        else:
            self.TTC = calc_TTC_traj(self.s , self.v_s , self.front_obs_s, self.front_obs_v_s)

    def save_to_file(self,path,exp_no,failed = False):

        self.get_TTC()

        # preprocess data
        length = len( self.t)
        data = {}
        data['time'] = self.t
        data['yaw'] = self.yaw
        data['s'] = self.s
        data['vel_s'] = self.v_s
        data['acc_s'] = self.a_s
        data['jerk_s'] = self.j_s

        data['l'] = self.l
        data['vel_l'] = self.v_l
        data['acc_l'] = self.a_l
        data['jerk_l'] = self.j_l

        if len(self.front_obs_s) > 0 :
            data['front_obs_s'] = self.front_obs_s
            data['front_obs_vel_s'] = self.front_obs_v_s
            data['front_obs_l'] = self.front_obs_l

        for i in range(self.other_obs_no):
            data['obs_'+ str(i+1) +'_s'] =  self.other_obs_s[i]
            data['obs_'+ str(i+1) +'_vel_s'] =  self.other_obs_v_s[i]
            if len(self.other_obs_a_s) > 0 :
                data['obs_'+ str(i+1) +'_a_s'] =  [self.other_obs_a_s[i] ] * length
            data['obs_'+ str(i+1) +'_l'] =  self.other_obs_l[i]

        if (len(self.decisions) < 1):   # lane_change scenario
            data['decisions'] = len(self.t) * ["NONE"]
        else: #overtake & keep lane scenario
            data['decisions'] = self.decisions + ["NONE"]

        data['TTC'] = self.TTC
        
        ##### debug #####
        #for key in data.keys() :
            #print(key, "len " , len(data[key]))

        df = pd.DataFrame(data=data)
        
        # save to a file
        if failed :
            file_path = path+'/data_'+str(exp_no)+'_failed.csv'
        else:
            file_path = path+'/data_'+str(exp_no)+'.csv'
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        df.to_csv(file_path, index=False)


    
