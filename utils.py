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
plt.rcParams.update({'font.size': 22})

# Fuctions to controll Rotation Motor
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
        self._forward_pos = False

    def SetStageModel(self, StageModelStr):
        # PRMTZ8
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
        # elif, else for other stages
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
        # (position velociy acceleration)
        return tmp

    def get_status_n(self):
        """
        Get numerical status of the device.
        For details, see APT communications protocol.
        """
        self.obj.send_comm(0x0429, 0x01)
        data = self.obj.recv_comm().data
        return strpack.unpack_uint(data[2:6], "<")

    status_bits = [(1 << 0, "sw_bk_lim"), (1 << 1, "sw_fw_lim"),
                   (1 << 4, "moving_bk"), (1 << 5, "moving_fw"), (1 <<
                                                                  6, "jogging_bk"), (1 << 7, "jogging_fw"),
                   (1 << 9, "homing"), (1 << 10, "homed"), (1 <<
                                                            12, "tracking"), (1 << 13, "settled"),
                   (1 << 14, "motion_error"), (1 << 24, "current_limit"), (1 << 31, "enabled")]

    def GetStatus(self):
        """
        Get device status.
        Return list of status strings, which can include ``"sw_fw_lim"`` (forward limit switch reached),``"sw_bk_lim"`` (backward limit switch reached),
        ``"moving_fw"`` (moving forward), ``"moving_bk"`` (moving backward),
        ``"homing"`` (homing), ``"homed"`` (homing done), ``"tracking"``, ``"settled"``,
        ``"motion_error"`` (excessive position error), ``"current_limit"`` (motor current limit exceeded), or ``"enabled"`` (motor is enabled).
        """
        status_n = self.get_status_n()

        now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        #print('{}   |   <GET STATUS>'.format(now))
        return [s for (m, s) in self.status_bits if status_n & m]

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
        # else:
           # print(' << Error: Check code at is_moving(). >>')

    def wait_for_status(self, status, Timeout=60, Period=3):
        # status is the desired string value in self.status_bits.
        # Timeout is the amount of time this function will run before giving up.
        # Period is the time between consecutive status queries.
        start = time.time()
        elapsed = time.time()-start
        flag = 0
        while flag == 0 and elapsed < Timeout:
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
        # self.GetPosition()

    def StepBwd(self, stepsize=-1):
        self.obj.move_by(stepsize)
        self.GetPosition()

    def SendHome(self, Status='homed', timeout=60, period=1):
        self.obj.send_comm(0x0443, 0x01)
        self.wait_for_status(Status, timeout, period)

    def Dev_SetPosition_APT(self, position, Status='homed', period=1):
        # move to a given position.
        if self.ScalingFactor != 1919.6418578623391:
            Status = 'enabled'
        self.obj.send_comm_data(0x0453, b'\x01\x00' +
                                strpack.pack_int(int(position), 4, '<'))
        self.wait_for_status(Status, Period=period)
        now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
       #print('{}   |   <GET POSITION>'.format(now))
        # time.sleep(0.5)
        pos = self.GetPosition()
        flag = math.isclose(float(pos), float(position) /
                            self.ScalingFactor, abs_tol=0.01)
        if flag == True:
            print('<< Position is properly set. >>')
            print('<< Current position is {:.4f}'.format(pos))
        else:
            print('<< ERROR: Unable to set desired position. >>')
            print('<< Current position is {:.4f}'.format(pos))

    def SetPosition(self, position):
        # optional arguments can be changed in Dev_SetPosition_APT()
        APT_pos = position*self.ScalingFactor
        self.Dev_SetPosition_APT(APT_pos)
        #print('<< Unresolvable error has magnitude {:.4f}'.format(abs(position - self.Dev_GetPosition_APT()/self.ScalingFactor)))
        return self.GetPosition()

    def get_velocity_params(self, scale=True):
        """
        Get current velocity parameters ``(max_velocity, acceleration)``

        If ``scale==True``, return these in counts/sec and counts/sec^2 respectively; otherwise, return in internal units.
        """
        self.obj.send_comm_data(0x0414, 1)
        msg = self.recv_comm_data()
        data = msg.data
        acceleration = strpack.unpack_int(data[6:10], "<")
        max_velocity = strpack.unpack_int(data[10:14], "<")
        if scale:
            acceleration /= (self._time_conv**2*2**16)
            max_velocity /= (self._time_conv*2**16)
        return max_velocity, acceleration

    def set_velocity_params(self, max_velocity, acceleration=None):
        if acceleration is None:
            acceleration = self.get_velocity_params(scale=False)[1]
        else:
            acceleration *= self._time_conv**2*2**16
        max_velocity *= (self._time_conv*2**16)
        data = b"\x01\x00"+b"\x00\x00\x00\x00" + \
            strpack.pack_int(int(acceleration), 4, "<") + \
            strpack.pack_int(int(max_velocity), 4, "<")
        self.obj.send_comm_data(0x0413, data)
        return self.get_velocity_params()

    def move(self, steps=1):
        """Move by `steps` (positive or negative) from the current position"""
        self.obj.send_comm_data(0x0448, b"\x01\x00" +
                                strpack.pack_int(int(steps), 4, "<"))

    def jog(self, direction):
        """Jog in the given direction (``"+"`` or ``"-"``)"""
        if not direction:  # 0 or False also mean left
            direction = "-"
        if direction in [1, True]:
            direction = "+"
        if direction not in ["+", "-"]:
            raise KinesisError("unrecognized direction: {}".format(direction))
        _jog_fw = (self.obj._forward_pos and direction ==
                   "+") or ((not self.obj._forward_pos) and direction == "-")
        self.obj.send_comm_data(0x0457, 1, 1 if _jog_fw else 2)

    def stop(self, immediate=False, sync=True, timeout=None):
        """
        Stop the motion.

        If ``immediate==True`` make an abrupt stop; otherwise, slow down gradually.
        If ``sync==True``, wait until the motion is stopped.
        """
        self.obj.send_comm_data(0x0465, 1, 1 if immediate else 2)
        if sync:
            self.wait_for_stop(timeout=timeout)

# Generel Functions
# function for finding nearest element to a value in an array

class Compensation():

    def find_nearest(array, value): 
        """
        This function finds the position of the element in the array that is closest to the value.
        """

        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return array[idx]

    # Polarimeter Function
    def Polarimeter(parameters, fidelity, S_undis, i, angle_fast,  V_back, wp_mount, home_step=5, N=310):
        """ This function measures and calculated the Stokes polarization parameters.
            
            ARGUMENTS:
                parameters: np.array, array containing the measured Stokes parameters
                fidelity: np.array, array containing the measured fidelity
                S_undis: np.array, desired undisturbed state for calculating the fidelity
                i: int, counts the number of times the polarization was measured
                home_step: int, number of polarization measurements after which the rotation mount is homed again
                N: int, number of samples for calculating the Stokes parameters
                angle_fast: float, angle of fast axis of the rotating QWP in rad
                V_back: float, background voltage of the photodiode
                wp_mount: Used rotation mount
        """

        print('Measure polarization')
        pos = np.empty(shape=(1, 0))
        voltage = np.empty(shape=(1, 0))

        if i % home_step == 0:                                # check if homing is needed
            wp_mount.obj.send_comm_data(0x0465, b'1', 1)      # stop rotation
            time.sleep(2.5)
            wp_mount.SendHome()                               # send home
            wp_mount.obj.send_comm_data(0x0457, b'1', 1)      # start rotation
            time.sleep(1)
            print('Rotation mount is homed!')

        task = nidaqmx.Task("task")                           # connect to photodiode
        task.ai_channels.add_ai_voltage_chan("Dev2/ai1", min_val=0, max_val=10)  # change "Dev2/ai1" to correct DAQ path

        for j in range(N):
            pos = np.append(pos, -wp_mount.GetPosition()*np.pi/180)     # measure position in rad
            # measure voltage of photodiode
            voltage = np.append(voltage, np.asarray(task.read(number_of_samples_per_channel=1))-V_back)
        # disconnect from photodiode
        task.close()
        task = None

        # find rotation position 2 pi later
        phi_last = int(np.where(pos[0:N] == Compensation.find_nearest(pos[0:N], pos[0]+2*np.pi))[0])

        A = 2/len(voltage[0:phi_last])*np.sum(voltage[0:phi_last])                # calculate parameters
        B = 4/len(voltage[0:phi_last])*np.sum(voltage[0:phi_last]* np.sin(2*(pos[:phi_last]+angle_fast)))
        C = 4/len(voltage[0:phi_last])*np.sum(voltage[0:phi_last]* np.cos(4*(pos[:phi_last]+angle_fast)))
        D = 4/len(voltage[0:phi_last])*np.sum(voltage[0:phi_last]* np.sin(4*(pos[:phi_last]+angle_fast)))

        # Calculate parameters and normalize
        parameters = np.append(parameters, np.array([A-C, 2*C, 2*D, -B]).reshape(4, 1)/(np.sqrt((2*C)**2+(2*D)**2+(-B)**2)), axis=1)
        # calculate fidelity
        fidelity = np.append(fidelity, 0.5*(1+S_undis[1]*parameters[1, -1]+S_undis[2]*parameters[2, -1]+S_undis[3]*parameters[3, -1]))

        i = i+1  # increase measurement count

        print('Current SOP:', parameters[:, -1])
        print('current fidelity:', fidelity[-1])

        return parameters, fidelity, i

    # Compensation Function

    def all_LC(delta): # matrix of all 4 LCs
        """ This function caluculates the Mueller matrix for all 4 LCVRs
            
            ARGUMENTS:
                delta: np.array, array containing the retardances of the LCVR
        """
        return np.array([[1, 0, 0, 0],

                [0, np.cos(delta[1])*np.cos(delta[3]) - np.cos(delta[2])*np.sin(delta[1])*np.sin(delta[3]),
                np.cos(delta[3])*np.sin(delta[0])*np.sin(delta[1]) + np.sin(delta[3])*(np.cos(delta[1])*np.cos(delta[2])*np.sin(delta[0])+np.cos(delta[0])*np.sin(delta[2])),
                np.sin(delta[0])*np.sin(delta[2])*np.sin(delta[3]) - np.cos(delta[0])*(np.cos(delta[3])*np.sin(delta[1])+np.cos(delta[1])*np.cos(delta[2])*np.sin(delta[3]))],

                [0, np.sin(delta[1])*np.sin(delta[2]), 
                np.cos(delta[0])*np.cos(delta[2])-np.cos(delta[1])*np.sin(delta[0])*np.sin(delta[2]),
                np.cos(delta[2])*np.sin(delta[0])+np.cos(delta[0])*np.cos(delta[1])*np.sin(delta[2])],

                [0, np.cos(delta[2])*np.cos(delta[3])*np.sin(delta[1]) + np.cos(delta[1])*np.sin(delta[3]),
                -np.cos(delta[1])*np.cos(delta[2])*np.cos(delta[3])*np.sin(delta[0]) - np.cos(delta[0])*np.cos(delta[3])*np.sin(delta[2]) + np.sin(delta[0])*np.sin(delta[1])*np.sin(delta[3]),
                -np.cos(delta[3])*np.sin(delta[0])*np.sin(delta[2]) + np.cos(delta[0])*(np.cos(delta[1])*np.cos(delta[2])*np.cos(delta[3])-np.sin(delta[1])*np.sin(delta[3]))]])

    def func_Comp(delta, delta4, s_dis, s_undis):
        """ Function used for calculating the desired retardances. 
            eq. 25, M_LCVR * S_dis - S_undis = 0, retardence of the 4th LCVR doesnt change

            ARGUMENTS:
                delta: np.array, array containing the retardances of the LCVR 1 to 3
                delta3: float, retardance of the 4th LCVR
                s_dis: np.array, disturbed polarization state before the LCVRs
                s_undis: np.array, disired undisturben polarization state
        """
        return [s_dis[1] * (np.cos(delta[1])*np.cos(delta4) - np.cos(delta[2])*np.sin(delta[1])*np.sin(delta4)) +
                s_dis[2] * (np.cos(delta4)*np.sin(delta[0])*np.sin(delta[1]) + np.sin(delta4)*(np.cos(delta[1])*np.cos(delta[2])*np.sin(delta[0])+np.cos(delta[0])*np.sin(delta[2]))) +
                s_dis[3] * (np.sin(delta[0])*np.sin(delta[2])*np.sin(delta4) - np.cos(delta[0])*(np.cos(delta4)*np.sin(delta[1])+np.cos(delta[1])*np.cos(delta[2])*np.sin(delta4))) - s_undis[1],

                s_dis[1] * (np.sin(delta[1])*np.sin(delta[2])) +
                s_dis[2] * (np.cos(delta[0])*np.cos(delta[2])-np.cos(delta[1])*np.sin(delta[0])*np.sin(delta[2])) +
                s_dis[3] * (np.cos(delta[2])*np.sin(delta[0])+np.cos(delta[0])*np.cos(delta[1])*np.sin(delta[2])) - s_undis[2],

                s_dis[1] * (np.cos(delta[2])*np.cos(delta4)*np.sin(delta[1]) + np.cos(delta[1])*np.sin(delta4)) +
                s_dis[2] * (-np.cos(delta[1])*np.cos(delta[2])*np.cos(delta4)*np.sin(delta[0]) - np.cos(delta[0])*np.cos(delta4)*d(delta[2]) + np.sin(delta[0])*np.sin(delta[1])*np.sin(delta4)) +
                s_dis[3] * (-np.cos(delta4)*np.sin(delta[0])*np.sin(delta[2]) + np.cos(delta[0])*(np.cos(delta[1])*np.cos(delta[2])*np.cos(delta4)-np.sin(delta[1])*np.sin(delta4))) - s_undis[3]]

    # Compensation Calculation
    def Compensate(parameters,S_dis, S_undis, retardance, volt, j, min_ret, max_ret, data_LC_1, data_LC_2, data_LC_3):
        """ Function used for calculating the desired voltages given the disturbed polarization state and the characterizations of the LCVRs

            ARGUMENTS:
                parameters: np.array, array containing the measured Stokes parameters
                S_dis: np.array, disturbed polarization state before the LCVRs
                S_undis: np.array, disired undisturben polarization state
                retardance: np.array, applied retardances of LCVRs
                volt: np.array, voltages applied to get the retardances of the LCVRs
                j: int, counter for iterations before fine tuning, compensation stops, when j == 10 -> no solution can be found
                min_ret: float, min retardane of LCVRs in pi, max_ret - min_ret should be greater than 2
                max_ret: float, max retardane of LCVRs in pi, max_ret - min_ret should be greater than 2
                data_LC_1: np.array, voltage - retardance characterisation of the LCVR 1
                data_LC_2: np.array, voltage - retardance characterisation of the LCVR 2
                data_LC_3: np.array, voltage - retardance characterisation of the LCVR 3
        """
        print('Calculate Compensation, Try number ', j+1)
        ret = np.empty(shape=(1, 0))
        # get actual disturbed input light
        S_dis = np.append(S_dis,np.transpose(Compensation.all_LC(retardance[:, -1]))@ parameters[:, -1].reshape(4,1),axis=1)

        # calculate needed retardances for LCs 1-3
        ret = fsolve(Compensation.func_Comp, x0=(np.pi, np.pi, np.pi), args=(retardance[3, -1], S_dis[:,-1], S_undis))  
        
        # get retardances in desired range -> should be between 1/5 pi and 11/5 pi for the used LCVRs
        for p in range(3):
            if ret[p] < min_ret * np.pi:
                ret[p] = ret[p] + (math.ceil((np.abs(ret[p]) + min_ret *np.pi)/(2 * np.pi))) * 2 * np.pi

            if ret[p] > max_ret * np.pi:
                ret[p] = ret[p] - (math.ceil((ret[p] - max_ret * np.pi)/(2 * np.pi))) * 2 * np.pi

        # save new retardances (LCVR 4 stays the same)
        retardance = np.append(retardance, np.array([ret[0], ret[1], ret[2], retardance[3, -1]]).reshape(4, 1), axis=1)

        # find right voltages (LCVR 4 stays the same)
        volt = np.append(volt, np.array([data_LC_1[:, 0][np.where(data_LC_1[:, 1] == Compensation.find_nearest(data_LC_1[:, 1], ret[0]))[0][0]],
                                        data_LC_2[:, 0][np.where(data_LC_2[:, 1] == Compensation.find_nearest(data_LC_2[:, 1], ret[1]))[0][0]],
                                        data_LC_3[:, 0][np.where(data_LC_3[:, 1] == Compensation.find_nearest(data_LC_3[:, 1], ret[2]))[0][0]],
                                        volt[3, -1]]).reshape(4, 1), axis=1)
        return retardance, volt, S_dis

    # Fine Tuning function 
    def Fine_Tuning(parameters, fidelity, S_undis, retardance, volt, try_step, i, home_step, N, angle_fast,stopping_threshold, V_back, fg_1, fg_2, data_LC_1, data_LC_2, data_LC_3, data_LC_4, wp_mount):
        """ Function used for the fine tuning after the compensation

            ARGUMENTS:
                parameters: np.array, array containing the measured Stokes parameters
                fidelity: np.array, array containing the measured fidelity
                S_undis: np.array, disired undisturben polarization state
                retardance: np.array, applied retardances of LCVRs
                volt: np.array, voltages applied to get the retardances of the LCVRs
                try_step: float, voltage stepsize for fine tuning function
                i: int, counts the number of times the polarization was measured
                home_step: int, number of polarization measurements after which the rotation mount is homed again
                N: int, number of samples for calculating the Stokes parameters
                angle_fast: float, angle of fast axis of the rotating QWP in rad
                stopping_treshold: 
                V_back: float, background voltage of the photodiode
                fg_1: function generator for LCVRR 1 and 2
                fg_2: function generator for LCVRR 3 and 4
                data_LC_1: np.array, voltage - retardance characterisation of the LCVR 1
                data_LC_2: np.array, voltage - retardance characterisation of the LCVR 2
                data_LC_3: np.array, voltage - retardance characterisation of the LCVR 3
                data_LC_4: np.array, voltage - retardance characterisation of the LCVR 4
                wp_mount: Used rotation mount
        """
        
        # fine tune applied voltages for all 4 LCVRs
        for z in range(4):
            if fidelity[-1] > stopping_threshold:
                break
            print('Fine tune LCVR:', z+1)
            print('Increase applied voltage')
            # try a higher voltage
            volt = np.append(volt, (np.array([volt[0, -1], volt[1, -1], volt[2, -1], volt[3, -1]]) + (np.arange(4) == z) * try_step).reshape(-1, 1), axis=1)
            # find corresponding retardance
            retardance = np.append(retardance, np.array([data_LC_1[:, 1][np.where(data_LC_1[:, 0] == Compensation.find_nearest(data_LC_1[:, 0], volt[0, -1]))[0][0]],
                                                        data_LC_2[:, 1][np.where(data_LC_2[:, 0] == Compensation.find_nearest(data_LC_2[:, 0], volt[1, -1]))[0][0]],
                                                        data_LC_3[:, 1][np.where(data_LC_3[:, 0] == Compensation.find_nearest(data_LC_3[:, 0], volt[2, -1]))[0][0]],
                                                        data_LC_4[:, 1][np.where(data_LC_4[:, 0] == Compensation.find_nearest(data_LC_4[:, 0], volt[3, -1]))[0][0]]]).reshape(4, 1), axis=1)
            # apply new voltage
            if z < 2:
                fg_1.set_amplitude(channel=z+1, value=volt[z, -1])
            else:
                fg_2.set_amplitude(channel=z-1, value=volt[z, -1])
            sleep(0.1)

            # Measure new polarisation
            parameters, fidelity, i = Compensation.Polarimeter(parameters, fidelity, S_undis, i, home_step, N, angle_fast, V_back, wp_mount)

            if fidelity[-1] > stopping_threshold:
                break

            while fidelity[-1] >= fidelity[-2]: # keep going up as long as the fidelity gets higher
                print('Fidelity got higher')
                volt = np.append(volt, (np.array([volt[0, -1], volt[1, -1], volt[2, -1], volt[3, -1]]) + (np.arange(4) == z) * try_step).reshape(-1, 1), axis=1)
                retardance = np.append(retardance, np.array([data_LC_1[:, 1][np.where(data_LC_1[:, 0] == Compensation.find_nearest(data_LC_1[:, 0], volt[0, -1]))[0][0]],
                                                            data_LC_2[:, 1][np.where(data_LC_2[:, 0] == Compensation.find_nearest( data_LC_2[:, 0], volt[1, -1]))[0][0]],
                                                            data_LC_3[:, 1][np.where(data_LC_3[:, 0] == Compensation.find_nearest(data_LC_3[:, 0], volt[2, -1]))[0][0]],
                                                            data_LC_4[:, 1][np.where(data_LC_4[:, 0] == Compensation.find_nearest(data_LC_4[:, 0], volt[3, -1]))[0][0]]]).reshape(4, 1), axis=1)
                if z < 2:
                    fg_1.set_amplitude(channel=z+1, value=volt[z, -1])
                else:
                    fg_2.set_amplitude(channel=z-1, value=volt[z, -1])
                sleep(0.1)

                # measure again
                parameters, fidelity, i = Compensation.Polarimeter(parameters, fidelity, S_undis, i, home_step, N, angle_fast, V_back, wp_mount)

                if fidelity[-1] > stopping_threshold:
                    break

            if fidelity[-1] < fidelity[-2]:
                print('Fidelity got lower')
                # go back to last voltage
                volt = np.append(volt, (np.array([volt[0, -1], volt[1, -1], volt[2, -1], volt[3, -1]]) - ( np.arange(4) == z) * try_step).reshape(-1, 1), axis=1)
                retardance = np.append(retardance, np.array([data_LC_1[:, 1][np.where(data_LC_1[:, 0] == Compensation.find_nearest(data_LC_1[:, 0], volt[0, -1]))[0][0]],
                                                            data_LC_2[:, 1][np.where(data_LC_2[:, 0] == Compensation.find_nearest( data_LC_2[:, 0], volt[1, -1]))[0][0]],
                                                            data_LC_3[:, 1][np.where(data_LC_3[:, 0] == Compensation.find_nearest( data_LC_3[:, 0], volt[2, -1]))[0][0]],
                                                            data_LC_4[:, 1][np.where(data_LC_4[:, 0] == Compensation.find_nearest(data_LC_4[:, 0], volt[3, -1]))[0][0]]]).reshape(4, 1), axis=1)
                if z < 2:
                    fg_1.set_amplitude(channel=z+1, value=volt[z, -1])
                else:
                    fg_2.set_amplitude(channel=z-1, value=volt[z, -1])
                sleep(0.1)

            # try a lower voltage; same code as up, but with substracting voltage step
            print('Decrease applied voltage')
            volt = np.append(volt, (np.array([volt[0, -1], volt[1, -1], volt[2, -1], volt[3, -1]]) - (np.arange(4) == z) * try_step).reshape(-1, 1), axis=1)
            retardance = np.append(retardance, np.array([data_LC_1[:, 1][np.where(data_LC_1[:, 0] == Compensation.find_nearest(data_LC_1[:, 0], volt[0, -1]))[0][0]],
                                                        data_LC_2[:, 1][np.where(data_LC_2[:, 0] == Compensation.find_nearest(data_LC_2[:, 0], volt[1, -1]))[0][0]],
                                                        data_LC_3[:, 1][np.where(data_LC_3[:, 0] == Compensation.find_nearest(data_LC_3[:, 0], volt[2, -1]))[0][0]],
                                                        data_LC_4[:, 1][np.where(data_LC_4[:, 0] == Compensation.find_nearest(data_LC_4[:, 0], volt[3, -1]))[0][0]]]).reshape(4, 1), axis=1)

            if z < 2:
                fg_1.set_amplitude(channel=z+1, value=volt[z, -1])
            else:
                fg_2.set_amplitude(channel=z-1, value=volt[z, -1])
            sleep(0.1)

            parameters, fidelity, i = Compensation.Polarimeter(parameters, fidelity, S_undis, i, home_step, N, angle_fast, V_back, wp_mount)

            if fidelity[-1] > stopping_threshold:
                break

            while fidelity[-1] >= fidelity[-2]:
                print('Fidelity got higher')
                volt = np.append(volt, (np.array([volt[0, -1], volt[1, -1], volt[2, -1], volt[3, -1]]) - ( np.arange(4) == z) * try_step).reshape(-1, 1), axis=1)
                retardance = np.append(retardance, np.array([data_LC_1[:, 1][np.where(data_LC_1[:, 0] == Compensation.find_nearest(data_LC_1[:, 0], volt[0, -1]))[0][0]],
                                                            data_LC_2[:, 1][np.where(data_LC_2[:, 0] == Compensation.find_nearest(data_LC_2[:, 0], volt[1, -1]))[0][0]],
                                                            data_LC_3[:, 1][np.where(data_LC_3[:, 0] == Compensation.find_nearest(data_LC_3[:, 0], volt[2, -1]))[0][0]],
                                                            data_LC_4[:, 1][np.where(data_LC_4[:, 0] == Compensation.find_nearest(data_LC_4[:, 0], volt[3, -1]))[0][0]]]).reshape(4, 1), axis=1)

                if z < 2:
                    fg_1.set_amplitude(channel=z+1, value=volt[z, -1])
                else:
                    fg_2.set_amplitude(channel=z-1, value=volt[z, -1])
                sleep(0.1)

                parameters, fidelity, i = Compensation.Polarimeter(parameters, fidelity, S_undis, i, home_step, N, angle_fast, V_back, wp_mount)

                if fidelity[-1] > stopping_threshold:
                    break

            if fidelity[-1] < fidelity[-2]:
                print('Fidelity got lower')
                volt = np.append(volt, (np.array([volt[0, -1], volt[1, -1], volt[2, -1], volt[3, -1]]) + (np.arange(4) == z) * try_step).reshape(-1, 1), axis=1)
                retardance = np.append(retardance, np.array([data_LC_1[:, 1][np.where(data_LC_1[:, 0] == Compensation.find_nearest(data_LC_1[:, 0], volt[0, -1]))[0][0]],
                                                            data_LC_2[:, 1][np.where(data_LC_2[:, 0] == Compensation.find_nearest( data_LC_2[:, 0], volt[1, -1]))[0][0]],
                                                            data_LC_3[:, 1][np.where(data_LC_3[:, 0] == Compensation.find_nearest( data_LC_3[:, 0], volt[2, -1]))[0][0]],
                                                            data_LC_4[:, 1][np.where(data_LC_4[:, 0] == Compensation.find_nearest(data_LC_4[:, 0], volt[3, -1]))[0][0]]]).reshape(4, 1), axis=1)

                if z < 2:                                                   # set voltage back
                    fg_1.set_amplitude(channel=z+1, value=volt[z, -1])
                else:
                    fg_2.set_amplitude(channel=z-1, value=volt[z, -1])

        return parameters, fidelity, retardance, volt, i
