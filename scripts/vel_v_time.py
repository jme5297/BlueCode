import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

plt.figure(num=None, figsize=(16, 9), dpi=80, facecolor='w', edgecolor='k')
directory = ''

data = pd.read_csv(directory+'data.csv', sep=',', index_col=False)
#mxlog = pd.read_csv(directory+'mxlog.csv', sep=',', index_col=False)
pts = pd.read_csv(directory+'pts.csv', sep=',',index_col=False, header=None, names=['lon', 'lat'])
datanp = np.array(data)
ptsnp = np.array(pts)
plt.hold(True)

plt.plot(data['time'], data['vel'], color='blue', linewidth=4.0, label='velocity (m/s)')
#plt.grid(b=True, which='both', linestyle='-')
#plt.xlabel('Time (sec)')
#plt.ylabel('Velocity (m/s)')

#plt.subplot(2,1,1)
plt.plot(data['time'], data['throtGain'], color='red', linewidth=4.0, label='throttle gain factor')
#plt.grid(b=True, which='both', linestyle='-')
#plt.xlabel('Time (sec)')
#plt.ylabel('Speed Gain Factor')

#plt.subplot(2,1,2)
plt.plot(data['time'], data['normThrot'], color='black', linewidth=4.0, label='normalized throttle (-1.0 to 1.0)')
plt.grid(b=True, which='both', linestyle='-')
plt.xlabel('Time (sec)')
#plt.ylabel('Throttle')
plt.legend()
plt.savefig('vel_v_time.png', bbox_inches='tight')
