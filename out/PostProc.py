import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

data = pd.read_csv('../build/data_0.csv', sep=',', index_col=False, header=None, names=['time', 'lat', 'lon'])
mlog = pd.read_csv('../build/mxlog_0.csv', sep=',', index_col=False, header=None, names=['time', 'who', 'msg'])
pts = pd.read_csv('../build/pts.csv', sep=',',index_col=False, header=None, names=['lat', 'lon'])
datanp = np.array(data)
ptsnp = np.array(pts)

for ii in range(0, len(pts)-1):
    plt.plot(ptsnp[ii,0],ptsnp[ii,1], marker='o', markersize=3, color='red')

drops = mlog.loc[mlog['msg'] == 'CTL sent payload-drop and image-taken signals.' ]

for ii in range(0, len(drops)-1):
    time = drops.iloc[ii]['time']
    idx = np.argmin(abs(datanp[:,0]-time))
    plt.plot(datanp[idx,1],datanp[idx,2], marker='o', markersize=10, color='green')
    plt.plot([datanp[idx,1],ptsnp[ii,0]],[datanp[idx,2],ptsnp[ii,1]], color='black')

plt.plot(data['lat'], data['lon'], color='blue')
plt.grid(b=True, which='both', color='0.65', linestyle='-')
plt.show()
