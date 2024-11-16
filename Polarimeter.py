from pylablib.devices import Thorlabs # pip install --user pylablib
from pylablib.core.utils import strpack
import time
import math
from datetime import datetime
import nidaqmx
import numpy as np
from timeit import default_timer as timer
import matplotlib.pyplot as plt
import pandas as pd
from utils import * 
# script for the polarization tomography/ polarimeter

# General functions

rounds = 50 # number of measurements
N = 310 # number of samples per measurement
home_steps = 5 # number of measurement after which the motor is homed again
angle_fast = 74.42363808*np.pi/180      #  angle of rotating QWP fast axis in radians
S_undis = np.array([1,0,1,0])          # expected polarisation for calculating a fidelity

# Prepare empty arrays
position = np.empty(shape=(1,0))
voltage = np.empty(shape=(1,0))
intensity = np.empty(shape=(1,0))
parameters = np.empty(shape=(4,0))
fidelity = np.empty(shape=(1,0))

# load background voltage
data_b = np.genfromtxt('Data_background.csv', skip_header=1,delimiter=',', dtype=None) # select correct backgrond voltage file
V_back = np.mean(data_b[:]) 

# Connect to motor
wp_mount = KDC101(9)
wp_mount.SetStageModel("PRM1Z8") # choose right motor
DAQ_path = "Dev2/ai1" # path to DAQ

i = 0 # mesurement count
for k in range(rounds): # measure Stokes parameters
    parameters, fidelity, i = Compensation.Polarimeter(parameters, fidelity, S_undis, i, angle_fast,  V_back, wp_mount, DAQ_path, home_step, N)

wp_mount.obj.send_comm_data(0x0465,b'1',1)      # stop rotation

df=pd.DataFrame({"Parameter S0 ":parameters[0,:],"Parameter S1 ":parameters[1,:],"Parameter S2 ":parameters[2,:],
                 "Parameter S3 ":parameters[3,:],"Fidelity":fidelity})
df.to_csv("Analyser_Stokes_params_and_fidelity.csv",index=False)
