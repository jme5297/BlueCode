import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

plt.figure().patch.set_facecolor('gray')
directory = '../build/'

data = pd.read_csv(directory+'data.csv', sep=',', index_col=False)
print(data)
mlog = pd.read_csv(directory+'mxlog.csv', sep=',', index_col=False, header=None, names=['time', 'who', 'msg'])
pts = pd.read_csv(directory+'pts.csv', sep=',',index_col=False, header=None, names=['lon', 'lat'])
datanp = np.array(data)
ptsnp = np.array(pts)

for ii in range(0, len(pts)-1):
    plt.plot(ptsnp[ii,0],ptsnp[ii,1], marker='o', markersize=6, color='red')
    if ii != 0:
        plt.plot([ptsnp[ii-1,0], ptsnp[ii,0]],[ptsnp[ii-1,1], ptsnp[ii,1]],color='green', linestyle='--')
    else:
        plt.plot([datanp[0,1], ptsnp[ii,0]],[datanp[0,2], ptsnp[ii,1]],color='green', linestyle='--')

# Collect all of the payload drop locations
drops = mlog.loc[mlog['msg'] == 'CTL sent payload-drop and image-taken signals.' ]

plt.plot(datanp[0,1],datanp[0,2], marker='o', markersize=10, color='blue')

for ii in range(0, len(drops)-1):
    time = drops.iloc[ii]['time']
    idx = np.argmin(abs(datanp[:,0]-time))

    plt.plot(datanp[idx,1],datanp[idx,2], marker='o', markersize=10, color='green')
    plt.plot([datanp[idx,1],ptsnp[ii,0]],[datanp[idx,2],ptsnp[ii,1]], color='black', linestyle='--')

plt.plot(data['lon'], data['lat'], color='blue')
plt.grid(b=True, which='both', linestyle='-')
plt.xlabel('Lon (deg)')
plt.ylabel('Lat (deg)')
plt.title('Actual Path vs. Proposed NavPlan')
plt.axis('equal')
plt.show()
