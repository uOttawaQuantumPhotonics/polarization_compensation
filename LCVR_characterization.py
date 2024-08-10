# %%
import numpy as np
import pandas as pd
import nidaqmx
import jds6600 
from time import sleep

# this script is used for the characterization of a LCVR; the background will be subtracted in the data processing
def sweep(v_min,v_max,stepsize):       # Sweeping Function
    x = v_min                          # start value of voltage
    volt = np.empty(shape=(2,0))       # create empty array for all voltage measurements
    volt_mean_std = np.empty(shape=(2,0))      # create empty array for mean and std voltage
    for i in range(int((v_max-v_min)/stepsize)+1):
        fg.set_amplitude(channel=1, value=x) # Set voltage
        sleep(0.1)                           # Sleeping time for LCVRs switching time

        # measure current voltage and offset of the function generator
        volt = np.append(volt,np.array([[fg.get_amplitude(1)],[fg.get_offset(channel=1)]]),axis=1)  

        # connect to the DAQ (Input Dev2/ai1)
        task = nidaqmx.Task("task")
        task.ai_channels.add_ai_voltage_chan("Dev2/ai1", min_val=0, max_val=10) # change "Dev2/ai1" to correct DAQ path
        # measure voltage of the photodiode (10 samples)
        samples = np.asarray(task.read(number_of_samples_per_channel=10))
        # save mean and std for the measurement       
        volt_mean_std = np.append(volt_mean_std,np.array([[np.mean(samples)],[np.std(samples, ddof=1)]]),axis=1)
        # diconnect from the DAQ
        task.close()
        task = None
    
        x += stepsize
    return volt_mean_std, volt

# Measurement
# Connect to function generator JDS6600; change 'COM11' to the right port
fg = jds6600.JDS6600(port='COM11')
fg.connect()
# turn output of JDS6600 off 
fg.set_channels(channel1=False, channel2=False)  

# set start configuration for JDS6600
# LCVR is connected to channel 1
fg.set_waveform(channel=1, value='square')       # set square wave mode
fg.set_frequency(channel=1, value=2e3)           # set frequency to 2 kHz
fg.set_offset(channel=1,value=0)                 # set offset to 0
fg.set_channels(channel1=True, channel2=False)   # turn channel 1 on

volt_mean_std_1 , volt_1 = sweep(0.1,16,0.01)   # Measure the data for voltage between 0.1 and 16 V with 0.01 V stepsize

# turn output of JDS6600 off 
fg.set_channels(channel1=False, channel2=False) 

# save the data
df=pd.DataFrame({"all_voltages":volt_1[0],"all_voltage_offsets":volt_1[1],"mean_voltages,0":volt_mean_std_1[0],"std_of_voltages":volt_mean_std_1[1]})
df.to_csv("Data_LCVR_characterization.csv",index=False)