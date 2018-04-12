import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

plt.figure().patch.set_facecolor('gray')
directory = '../build/'

data = pd.read_csv(directory+'data.csv', sep=',', index_col=False)
mlog = pd.read_csv(directory+'mxlog.csv', sep=',', index_col=False, header=None, names=['time', 'who', 'msg'])
pts = pd.read_csv(directory+'pts.csv', sep=',',index_col=False, header=None, names=['lon', 'lat'])
datanp = np.array(data)
ptsnp = np.array(pts)

plt.subplot(3,1,1)
plt.plot(data['time'], data['vel'], color='blue')
plt.grid(b=True, which='both', linestyle='-')
plt.xlabel('Time (sec)')
plt.ylabel('Velocity (m/s)')

plt.subplot(3,1,2)
plt.plot(data['time'], data['spdGain'], color='red')
plt.grid(b=True, which='both', linestyle='-')
plt.xlabel('Time (sec)')
plt.ylabel('Speed Gain Factor')

plt.subplot(3,1,3)
plt.plot(data['time'], data['normThrot'], color='black')
plt.grid(b=True, which='both', linestyle='-')
plt.xlabel('Time (sec)')
plt.ylabel('Throttle')


plt.show()
