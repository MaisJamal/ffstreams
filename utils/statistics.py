
import numpy as np
import pandas as pd
from metrics.TTC import calc_TTC_traj
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
        self.TTC = calc_TTC_traj(self.s , self.v_s , self.front_obs_s, self.front_obs_v_s)

    def save_to_file(self,path,exp_no):

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

        data['front_obs_s'] = self.front_obs_s
        data['front_obs_vel_s'] = self.front_obs_v_s
        data['front_obs_l'] = self.front_obs_l

        for i in range(self.other_obs_no):
            data['obs_'+ str(i+1) +'_s'] =  self.other_obs_s[i]
            data['obs_'+ str(i+1) +'_vel_s'] =  self.other_obs_v_s[i]
            data['obs_'+ str(i+1) +'_a_s'] =  [self.other_obs_a_s[i] ] * length
            data['obs_'+ str(i+1) +'_l'] =  self.other_obs_l[i]

        data['decisions'] = self.decisions + ["NONE"]

        data['TTC'] = self.TTC
        #for key in data.keys() :
         #   print(key, "len " , len(data[key]))

        df = pd.DataFrame(data=data)
        
        # save to a file
        file_path = path+'/data_'+str(exp_no)+'.csv'
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        df.to_csv(file_path, index=False)


    
