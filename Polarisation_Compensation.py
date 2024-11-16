#!/usr/bin/env python
# coding: utf-8
from timeit import default_timer as timer
import nidaqmx
from datetime import datetime
import time
from pylablib.core.utils import strpack
from pylablib.devices import Thorlabs  # pip install --user pylablib
from scipy.optimize import fsolve
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
from time import sleep
import jds6600
from utils import * 
plt.rcParams.update({'font.size': 22})

# Script for polarization compensation with 4 LCVRs, where the last one is just used during fine tuning
# Running the Compensation, Input parameters:
S_undis = np.array([1,np.sqrt(1/3), np.sqrt(1/3), -np.sqrt(1/3)])        # desired undisturbed state
start_ret = np.pi                     # starting retardance for all 4 LCVRs

N = 310                                 # number of samples per polarization measurement
# number of measuemens after which the motor is homed again
home_step = 5
angle_fast = 74.42363808*np.pi/180      # angle of fast axis of rotating QWP in rad

# min and max retardanes of LCVRs in pi, max_ret - min_ret should be greater than 2
min_ret = 1/5  # minimum possible retardance of LCVRs
max_ret = 11/5  # minimum possible retardance of LCVRs

finetuning_threshold = 0.97 # fidelity threshold after which the fine tuning starts
stopping_threshold = 0.995  # fidelity threshold after which the compensation stops immediatily 
rounds = 1                              # number of rounds the fine tuning goes through all LCVRs
try_step = 0.01                         # voltage stepsize for fine tuning function

parameters = np.empty(shape=(4, 0))     # Arrays for the Polarimeter
fidelity = np.empty(shape=(1, 0))

# Arrays for the compensation
retardance = np.empty(shape=(4, 0))  # retardances of LCVRs
volt = np.empty(shape=(4, 0))        # voltages applied to get the retardances 
S_dis = np.empty(shape=(4,0))

# Create an instance of the Compensation class
compensation = Compensation()

# import characterisation of the LCs
data_LC_1 = np.genfromtxt('Data_LC/Compensation_LC_1.csv',skip_header=1, delimiter=',', dtype=None)
data_LC_2 = np.genfromtxt('Data_LC/Compensation_LC_2.csv',skip_header=1, delimiter=',', dtype=None)
data_LC_3 = np.genfromtxt('Data_LC/Compensation_LC_3.csv',skip_header=1, delimiter=',', dtype=None)
data_LC_4 = np.genfromtxt('Data_LC/Compensation_LC_5.csv',skip_header=1, delimiter=',', dtype=None)

# load background voltage
data_b = np.genfromtxt('Data_background.csv', skip_header=1,delimiter=',', dtype=None) # select correct backgrond voltage file
V_back = np.mean(data_b[:]) 

# Connect to the motor
wp_mount = KDC101(8)      
wp_mount.SetStageModel("PRM1Z8")
DAQ_path = "Dev2/ai1" # path to DAQ

# connect to function generator
# 1st LCVR connected to channel one of fg 1
# 2st LCVR connected to channel two of fg 1
# 3st LCVR connected to channel one of fg 2
# 4st LCVR connected to channel two of fg 2
fg_1 = jds6600.JDS6600(port='COM12') # change 'COM12' to the right port
fg_1.connect()
fg_1.set_channels(channel1=False, channel2=False)  # turn output of jds6600 off

fg_2 = jds6600.JDS6600(port='COM11') # change 'COM11' to the right port
fg_2.connect()
fg_2.set_channels(channel1=False, channel2=False)  # turn output of jds6600 off

# set starting voltage
fg_1.set_amplitude(channel=1, value=data_LC_1[:, 0][np.where(data_LC_1[:, 1] == compensation.find_nearest(data_LC_1[:, 1], start_ret))[0][0]])
fg_1.set_amplitude(channel=2, value=data_LC_2[:, 0][np.where(data_LC_2[:, 1] == compensation.find_nearest(data_LC_2[:, 1], start_ret))[0][0]])
fg_2.set_amplitude(channel=1, value=data_LC_3[:, 0][np.where(data_LC_3[:, 1] == compensation.find_nearest(data_LC_3[:, 1], start_ret))[0][0]])
fg_2.set_amplitude(channel=2, value=data_LC_4[:, 0][np.where(data_LC_4[:, 1] == compensation.find_nearest(data_LC_4[:, 1], start_ret))[0][0]])

# save starting voltage
volt = np.append(volt, np.array([data_LC_1[:, 0][np.where(data_LC_1[:, 1] == compensation.find_nearest(data_LC_1[:, 1], start_ret))[0][0]],
                                 data_LC_2[:, 0][np.where(data_LC_2[:, 1] == compensation.find_nearest(data_LC_2[:, 1], start_ret))[0][0]],
                                 data_LC_3[:, 0][np.where(data_LC_3[:, 1] == compensation.find_nearest(data_LC_3[:, 1], start_ret))[0][0]],
                                 data_LC_4[:, 0][np.where(data_LC_4[:, 1] == compensation.find_nearest(data_LC_4[:, 1], start_ret))[0][0]]]).reshape(4, 1), axis=1)

# save starting retardance
retardance = np.append(retardance, np.array([data_LC_1[:, 1][np.where(data_LC_1[:, 1] == compensation.find_nearest(data_LC_1[:, 1], start_ret))[0][0]],
                                             data_LC_2[:, 1][np.where(data_LC_2[:, 1] == compensation.find_nearest(data_LC_2[:, 1], start_ret))[0][0]],
                                             data_LC_3[:, 1][np.where(data_LC_3[:, 1] == compensation.find_nearest(data_LC_3[:, 1], start_ret))[0][0]],
                                             data_LC_4[:, 1][np.where(data_LC_4[:, 1] == compensation.find_nearest(data_LC_4[:, 1], start_ret))[0][0]]]).reshape(4, 1), axis=1)


# set function generators in the right mode
for q in range(4):
    if q < 2:
        # set square wave mode
        fg_1.set_waveform(channel=q+1, value='square')
        # set frequency
        fg_1.set_frequency(channel=q+1, value=2e3) 
        # set offset to 0 V
        fg_1.set_offset(channel=q+1, value=0)
    
    else:
        # set square wave mode
        fg_2.set_waveform(channel=q-1, value='square')
        # set frequency
        fg_2.set_frequency(channel=q-1, value=2e3)   
        # set offset to 0 V
        fg_2.set_offset(channel=q-1, value=0)

# turn LCVRs on on
fg_1.set_channels(channel1=True, channel2=True)
fg_2.set_channels(channel1=True, channel2=True)
sleep(0.1)

i = 0                               # Set measurement counter to zero

# Measure polarisation
parameters, fidelity, i = compensation.Polarimeter(parameters, fidelity, S_undis, i, angle_fast,  V_back, wp_mount, DAQ_path, home_step, N)

j = 0 # counter for iterations before fine tuning, compensation stops, when j == 10 -> no solution can be found
while fidelity[-1] < finetuning_threshold:
    # Calculate retardances and voltages
    retardance, volt, S_dis = compensation.Compensate(parameters,S_dis, S_undis, retardance, volt, j, min_ret, max_ret, data_LC_1, data_LC_2, data_LC_3)
    # apply voltages
    fg_1.set_amplitude(channel=1, value=volt[0, -1])
    fg_1.set_amplitude(channel=2, value=volt[1, -1])
    fg_2.set_amplitude(channel=1, value=volt[2, -1])
    fg_2.set_amplitude(channel=2, value=volt[3, -1])
    sleep(0.1)
    # Measure polarisation
    parameters, fidelity, i = compensation.Polarimeter(parameters, fidelity, S_undis, i, angle_fast,  V_back, wp_mount, DAQ_path, home_step, N)
    j = j+1
    if j == 10:
        break

for k in range(rounds):
    print('Fine Tuning, round:', k+1)
    # tries little voltage adjustments for all LCVRs
    parameters, fidelity, retardance, volt, i = compensation.Fine_Tuning(parameters, fidelity, S_undis, retardance, volt, try_step, i, home_step, N, angle_fast,stopping_threshold, V_back, fg_1, fg_2, data_LC_1, data_LC_2, data_LC_3, data_LC_4, wp_mount, DAQ_path)  

print('Final fidelity: ', fidelity[-1])
 
wp_mount.obj.send_comm_data(0x0465, b'1', 1)       # stop rotation
fg_1.set_channels(channel1=False, channel2=False)  # turn output of jds6600 off
fg_2.set_channels(channel1=False, channel2=False)  # turn output of jds6600 off

# Save data
df=pd.DataFrame({"Parameter S0 ":parameters[0,:],"Parameter S1 ":parameters[1,:],"Parameter S2 ":parameters[2,:],
                 "Parameter S3 ":parameters[3,:],"Fidelity":fidelity})
df.to_csv("Data_Compensation_data_polarimeter.csv",index=False)

df=pd.DataFrame({"retardance LC1":retardance[0,:],"retardance LC2":retardance[1,:],
                 "retardance LC3":retardance[2,:],"retardance LC4":retardance[3,:],"Volt LC1":volt[0,:],"Volt LC2":volt[1,:],"Volt LC3":volt[2,:],"Volt LC4":volt[3,:]})
df.to_csv("Data_Compensation_data_LCVRs.csv",index=False)
