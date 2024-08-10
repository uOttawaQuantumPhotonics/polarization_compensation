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
# skript for the polarization tomography/ polarimeter

# you will need to download and instal =l Thorlabs APT software if you do not already have it
# after installation of APT, open anaconda prompt and execute the following command
# pip install --user pylablib
# Thorlabs runs on APT communications protocol
# https://pylablib-v0.readthedocs.io/en/latest/_modules/pylablib/aux_libs/devices/Thorlabs.html#KinesisDevice

# Needed commands to control motorized rotation mount taken from:
"""
Created on Thu Sep 30 20:01:34 2021
Ver 0.2 
16-Nov-2021
@author: Daniel Hutama; dhuta087@uottawa.ca
"""
class KDC101():
    def __init__(self, indx):
        self.SN = Thorlabs.list_kinesis_devices()[indx][0]
        self.Description = Thorlabs.list_kinesis_devices()[indx][1]
        # self.obj = Thorlabs.kinesis.KinesisDevice(self.SN)
        self.obj = Thorlabs.kinesis.KinesisMotor(self.SN)
        self.ManualURL = 'https://www.thorlabs.com/software/apt/APT_Communications_Protocol_Rev_15.pdf'
        self._forward_pos=False
    
    def SetStageModel(self, StageModelStr):
        #PRMTZ8
        self.Stage = StageModelStr
        if self.Stage.startswith('PRMT') == True:
            self.ScalingFactor = 1919.6418578623391
            now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            print('{}   |   <SET SCALING>'.format(now))
            print('<< Scaling factor set to {} >>'.format(self.ScalingFactor))
        if self.Stage.startswith('PRM1') == True:
            self.ScalingFactor = 1919.6418578623391
            now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            print('{}   |   <SET SCALING>'.format(now))
            print('<< Scaling factor set to {} >>'.format(self.ScalingFactor))
        if self.Stage.startswith('Z8') == True:
            self.ScalingFactor = 34304.10969
            now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            print('{}   |   <SET SCALING>'.format(now))
            print('<< Scaling factor set to {} >>'.format(self.ScalingFactor))            
        #elif, else for other stages
        else:
            print('<< WARNING: Scaling factor was not automatically set. >>')
    
    def GetInfo(self):
         now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
         print('{}   |   <GET INFO>'.format(now))
         tmp = self.obj.get_device_info()
         return tmp
         
    def GetInfoDetailed(self):
        now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        print('{}   |   <GET FULL INFO>'.format(now))
        tmp = self.obj.get_full_info()
        return tmp
        
    def BlinkScreen(self):
        self.obj.blink()
        now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        print('{}   |   <BLINK SCREEN>'.format(now))
 
    def GetScale(self):
        now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        print('{}   |   <GET SCALE>'.format(now))
        tmp = self.obj._get_scale()
        #(position velociy acceleration)
        return tmp
    

    def get_status_n(self):
        """
        Get numerical status of the device.
        For details, see APT communications protocol.
        """
        self.obj.send_comm(0x0429,0x01)
        data=self.obj.recv_comm().data
        return strpack.unpack_uint(data[2:6],"<")

    status_bits=[(1<<0,"sw_bk_lim"),(1<<1,"sw_fw_lim"),
                (1<<4,"moving_bk"),(1<<5,"moving_fw"),(1<<6,"jogging_bk"),(1<<7,"jogging_fw"),
                (1<<9,"homing"),(1<<10,"homed"),(1<<12,"tracking"),(1<<13,"settled"),
                (1<<14,"motion_error"),(1<<24,"current_limit"),(1<<31,"enabled")]
  
    def GetStatus(self):
        """
        Get device status.
        Return list of status strings, which can include ``"sw_fw_lim"`` (forward limit switch reached),``"sw_bk_lim"`` (backward limit switch reached),
        ``"moving_fw"`` (moving forward), ``"moving_bk"`` (moving backward),
        ``"homing"`` (homing), ``"homed"`` (homing done), ``"tracking"``, ``"settled"``,
        ``"motion_error"`` (excessive position error), ``"current_limit"`` (motor current limit exceeded), or ``"enabled"`` (motor is enabled).
        """
        status_n=self.get_status_n()
        
        now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        #print('{}   |   <GET STATUS>'.format(now))
        return [s for (m,s) in self.status_bits if status_n&m]
    
    def is_homed(self):
        tmp = self.GetStatus()
        res = 'homed' in tmp
        if res == True:
            print('<< Stage is homed. >>')
        else: 
            print('<< Stage is NOT homed. >>')
        return res
    
    def is_moving(self):
        tmp = self.GetStatus()
        res_fw = 'moving_fw' in tmp
        res_bk = 'moving_bk' in tmp
        if res_fw == True:
            #print('<< Stage is moving Forward. >>')
            return True
        elif res_bk == True:
           # print('<< Stage is moving Backward. >>')
            return True
        elif res_bk | res_fw == False:
           # print(' << Stage is NOT moving. >>')
            return False
        #else:
           # print(' << Error: Check code at is_moving(). >>')
            
        
    def wait_for_status(self, status, Timeout = 60, Period = 3):
        # status is the desired string value in self.status_bits.
        # Timeout is the amount of time this function will run before giving up.
        # Period is the time between consecutive status queries. 
        start = time.time()
        elapsed = time.time()-start
        flag = 0
        while flag == 0 and elapsed<Timeout:
            if self.is_moving() == True:
                elapsed = time.time() - start
                flag = 0
                time.sleep(Period)
            elif status in self.GetStatus():
                flag = 1
                elapsed = time.time()-start
                time.sleep(Period)
        #print('<< Time elapsed is {:.3f} seconds.'.format(elapsed))
        
    def Dev_GetPosition_APT(self):
        flag = self.is_moving()
        '''
        if flag == True:
            count = 0
            while flag == True and count < 5:
                time.sleep(0.5) #wait to stop moving
                flag = self.is_moving()
                count = count + 1
        '''
        self.obj.send_comm(0x0411, 0x01)
        msg = self.obj.recv_comm()
        data = msg.data
        return strpack.unpack_int(data[2:6], "<")
    
    def GetPosition(self):
        #now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        #print('{}   |   <GET POSITION>'.format(now))       
        res = self.Dev_GetPosition_APT()/self.ScalingFactor
        #print('<< Current Position is {:.4f} >>'.format(res))
        return res
        
    def StepFwd(self, stepsize):
        self.obj.move_by(stepsize)
        time.sleep(0.75)
        #self.GetPosition()
    
    def StepBwd(self, stepsize=-1):
        self.obj.move_by(stepsize)
        self.GetPosition()
        
    
    def SendHome(self, Status = 'homed', timeout=60, period=1):
        self.obj.send_comm(0x0443,0x01)
        self.wait_for_status(Status, timeout, period)
        
    def Dev_SetPosition_APT(self, position, Status='homed', period=1):
        # move to a given position.
        if self.ScalingFactor != 1919.6418578623391:
            Status = 'enabled'
        self.obj.send_comm_data(0x0453,b'\x01\x00'+strpack.pack_int(int(position),4,'<'))
        self.wait_for_status(Status, Period = period)
        now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
       #print('{}   |   <GET POSITION>'.format(now))
        #time.sleep(0.5)
        pos = self.GetPosition()
        flag = math.isclose(float(pos), float(position)/self.ScalingFactor, abs_tol = 0.01)
        if  flag == True:
            print('<< Position is properly set. >>')
            print('<< Current position is {:.4f}'.format(pos))
        else:
            print('<< ERROR: Unable to set desired position. >>')
            print('<< Current position is {:.4f}'.format(pos))

    def SetPosition(self, position):
        #optional arguments can be changed in Dev_SetPosition_APT()
        APT_pos = position*self.ScalingFactor
        self.Dev_SetPosition_APT(APT_pos)
        #print('<< Unresolvable error has magnitude {:.4f}'.format(abs(position - self.Dev_GetPosition_APT()/self.ScalingFactor)))
        return self.GetPosition()
    
    def get_velocity_params(self, scale=True):
        """
        Get current velocity parameters ``(max_velocity, acceleration)``
        
        If ``scale==True``, return these in counts/sec and counts/sec^2 respectively; otherwise, return in internal units.
        """
        self.obj.send_comm_data(0x0414,1)
        msg=self.recv_comm_data()
        data=msg.data
        acceleration=strpack.unpack_int(data[6:10],"<")
        max_velocity=strpack.unpack_int(data[10:14],"<")
        if scale:
            acceleration/=(self._time_conv**2*2**16)
            max_velocity/=(self._time_conv*2**16)
        return max_velocity,acceleration
    
    def set_velocity_params(self, max_velocity, acceleration=None):
        if acceleration is None:
            acceleration=self.get_velocity_params(scale=False)[1]
        else:
            acceleration*=self._time_conv**2*2**16
        max_velocity*=(self._time_conv*2**16)
        data=b"\x01\x00"+b"\x00\x00\x00\x00"+strpack.pack_int(int(acceleration),4,"<")+strpack.pack_int(int(max_velocity),4,"<")
        self.obj.send_comm_data(0x0413,data)
        return self.get_velocity_params()
    
    def move(self, steps=1):
        """Move by `steps` (positive or negative) from the current position"""
        self.obj.send_comm_data(0x0448,b"\x01\x00"+strpack.pack_int(int(steps),4,"<"))
        
    def jog(self, direction):
        """Jog in the given direction (``"+"`` or ``"-"``)"""
        if not direction: # 0 or False also mean left
            direction="-"
        if direction in [1, True]:
            direction="+"
        if direction not in ["+","-"]:
            raise KinesisError("unrecognized direction: {}".format(direction))
        _jog_fw=(self.obj._forward_pos and direction=="+") or ( (not self.obj._forward_pos) and direction=="-")
        self.obj.send_comm_data(0x0457,1,1 if _jog_fw else 2)
        
    def stop(self, immediate=False, sync=True, timeout=None):
        """
        Stop the motion.

        If ``immediate==True`` make an abrupt stop; otherwise, slow down gradually.
        If ``sync==True``, wait until the motion is stopped.
        """
        self.obj.send_comm_data(0x0465,1,1 if immediate else 2)
        if sync:
            self.wait_for_stop(timeout=timeout)

# General functions
# function for finding nearest element to a value in an array
def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]

rounds = 50 # number of measurements
N = 310 # number of samples per measurement
home_steps = 5 # number of measurement after which he motor is homed again
angle_fast = 74.42363808*np.pi/180      #  angle of rotating QWP fast axis in radien
S_expect = np.array([1,0,1,0])          # expected polarisation for calculating a fidelity

# Prepare empty arrays
position = np.empty(shape=(1,0))
voltage = np.empty(shape=(1,0))
intensity = np.empty(shape=(1,0))
parameters = np.empty(shape=(4,0))
parameters_norm = np.empty(shape=(3,0))
fidelity = np.empty(shape=(1,0))

# load background voltage
data_b = np.genfromtxt('Data_background.csv', skip_header=1,delimiter=',', dtype=None) # select correct backgrond voltage file
V_back = np.mean(data_b[:]) 

# Connect to motor
wp_mount = KDC101(9)
wp_mount.SetStageModel("PRM1Z8") # choose right motor

for i in range(rounds):
    # check if homing is needed
    if i % home_step == 0:
    	wp_mount.obj.send_comm_data(0x0465, b'1', 1)  # stop rotation
        time.sleep(2.5)
        wp_mount.SendHome()                           # send home
        wp_mount.obj.send_comm_data(0x0457, b'1', 1)  # start rotation
        time.sleep(1)
        print('Homed!')
        
    task = nidaqmx.Task("task") # connect to photodiode
    task.ai_channels.add_ai_voltage_chan("Dev2/ai1", min_val=0, max_val=10) # change "Dev2/ai1" to correct DAQ path
    
    for j in range(N):
       position = np.append(position,-wp_mount.GetPosition()*np.pi/180)           # measure position in rad
       voltage = np.append(voltage,task.read(number_of_samples_per_channel=1)-V_back)    # measure voltage
    task.close()                # disconnect from photodiode
    task = None 
    
    # find data point 2 pi later
    phi_last = int(np.where(position[N*i:N*(i+1)] == find_nearest(position[N*i:N*(i+1)],position[N*i]+2*np.pi))[0])
    
    # calculate A, B, C, D (eq. A2 - A5)
    A = 2/len(voltage[0:phi_last])*np.sum(voltage[0:phi_last])                
    B = 4/len(voltage[0:phi_last])*np.sum(voltage[0:phi_last]* np.sin(2*(position[:phi_last]+angle_fast)))
    C = 4/len(voltage[0:phi_last])*np.sum(voltage[0:phi_last]* np.cos(4*(position[:phi_last]+angle_fast)))
    D = 4/len(voltage[0:phi_last])*np.sum(voltage[0:phi_last]* np.sin(4*(position[:phi_last]+angle_fast)))

    # Calculate parameters and normalize (eq. A6)
    parameters = np.append(parameters, np.array([A-C, 2*C, 2*D, -B]).reshape(4, 1)/(np.sqrt((2*C)**2+(2*D)**2+(-B)**2)), axis=1)
    # calculate fidelity (eq. 11)
    fidelity = np.append(fidelity, 0.5*(1+S_expect[1]*parameters[1, -1]+S_expect[2]*parameters[2, -1]+S_expect[3]*parameters[3, -1]))

    i = i+1  # increase measurement count

    print('Stokes parameters:', parameters[:, -1])
    print('Fidelity:', fidelity[-1])
    
wp_mount.obj.send_comm_data(0x0465,b'1',1)      # stop rotation

df=pd.DataFrame({"voltage":voltage,"position":position})
df.to_csv("Analyser_voltage_and_position.csv",index=False)

df=pd.DataFrame({"Parameter S0 ":parameters[0,:],"Parameter S1 ":parameters[1,:],"Parameter S2 ":parameters[2,:],
                 "Parameter S3 ":parameters[3,:],"Fidelity":fidelity})
df.to_csv("Analyser_Stokes_params_and_fidelity.csv",index=False)