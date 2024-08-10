# %%
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 30})

# this script is used to calculate the retardance for a LCVR after the measurement
# Import Data
data_b = np.genfromtxt('Data_background.csv', skip_header=1,delimiter=',', dtype=None)
data = np.genfromtxt('Data_LCVR_characterization.csv', skip_header=1,delimiter=',', dtype=None)

# calculate average background
V_back = np.mean(data_b[:]) 
# calculate error in the mean of average background
V_back_er = np.std(data_b[:],ddof=1) / np.sqrt(np.size(data_b[:]))

# find max photodiode voltage for calculating the retardance
V_max = data[:,2].max() 

# calculate retardance (eq. B1)
retardance = np.arccos(1-(2*(data[:,2]-V_back)) / (V_max-V_back))

# Calculate retardance error via Gaussian error propagation (eq. B3)
def ret_er(V,V_back,Vmax,V_err,V_back_err):
    error = np.empty(shape=(1,0))
    # calculate gaussian error propargation for all data points exept for the maximum value
    for i in range(np.shape(V)[0]):
        if V[i] != Vmax:
            error = np.append(error,np.sqrt((V_err[i]**2+V_back_err**2)/
                                            ((V[i]-V_back)*(Vmax-V[i]))))
        else:
            error = np.append(error,0)
    return error

# calculate error for retardance
ret_er = ret_er(data[:,2],V_back,V_max,data[:,3],V_back_er) 

# Unwrapp data
def unwrapping(data):
    # find highest voltage
    pos = int(np.where(data == data[200:].max())[0][0])
    wrapped = np.copy(data)
    # wrap data
    wrapped[:pos] = 2*wrapped[200:].max()-wrapped[:pos]
    return wrapped

retardance_c = np.copy(retardance)
unwrapped = unwrapping(unwrapping(retardance_c)) # unwrap data twice

# save result for compensation
df=pd.DataFrame({"Voltage":data[:,0],"Retardance":unwrapped})
df.to_csv("Characterisation_LCVR.csv",index=False)

# Plotting
fig, ax = plt.subplots(figsize=(15,7.5))
ax.set_ylabel('Retardance [Waves]')
ax.set_xlabel('Voltage functiongenerator [V]')
ax.set_ylim(0,1.6)
plt.grid(which='minor')
plt.grid(which='major')
#plt.xscale("log")
plt.title("Caracterization LCVR")
plt.errorbar(data[:,0],unwrapped/(2*np.pi),yerr=ret_er/(2*np.pi), fmt='o',elinewidth=0.5,color='blue',barsabove=True,label='Measurement ratardance LCVR')
ax.legend()
#fig.savefig('Characterisation_LCVR.pdf',dpi=400)