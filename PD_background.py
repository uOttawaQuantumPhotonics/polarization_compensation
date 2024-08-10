# %%
import numpy as np
import pandas as pd
import nidaqmx

# this script is used to measure the beackground voltage of the PD for the polarimeter and the LCVR characterization
N = 80000 # Number of measurments for background ca. 10 s
# connect to the National Instruments 782258-01 USB-6361 DAQ (Input Dev2/ai1)
task = nidaqmx.Task("task")
task.ai_channels.add_ai_voltage_chan("Dev2/ai1" , min_val=0, max_val=10) # change "Dev2/ai1" to correct DAQ path
# measure voltage of the photodiode
samples = np.asarray(task.read(number_of_samples_per_channel=N))
# diconnect from the DAQ
task.close()
task = None

# save the data
df=pd.DataFrame({"Voltage Background":samples})
df.to_csv("Data_background.csv",index=False)