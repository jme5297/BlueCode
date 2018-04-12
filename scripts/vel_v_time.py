import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

plt.figure(num=None, figsize=(16, 9), dpi=80, facecolor='w', edgecolor='k')
directory = ''

data = pd.read_csv(directory+'data.csv', sep=',', index_col=False)
mxlog = pd.read_csv(directory+'mxlog.csv', sep=',', index_col=False)
pts = pd.read_csv(directory+'pts.csv', sep=',',index_col=False, header=None, names=['lon', 'lat'])
datanp = np.array(data)
ptsnp = np.array(pts)

plt.subplot(3,1,1)
plt.plot(data['time'], data['vel'], color='blue')
plt.grid(b=True, which='both', linestyle='-')
plt.xlabel('Time (sec)')
plt.ylabel('Velocity (m/s)')

plt.subplot(3,1,2)
plt.plot(data['time'], data['throtGain'], color='red')
plt.grid(b=True, which='both', linestyle='-')
plt.xlabel('Time (sec)')
plt.ylabel('Speed Gain Factor')

plt.subplot(3,1,3)
plt.plot(data['time'], data['normThrot'], color='black')
plt.grid(b=True, which='both', linestyle='-')
plt.xlabel('Time (sec)')
plt.ylabel('Throttle')

plt.savefig('vel_v_time.png', bbox_inches='tight')
