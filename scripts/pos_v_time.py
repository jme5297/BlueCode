import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import pandas as pd
import numpy as np

plt.figure(num=None, figsize=(16, 9), dpi=80, facecolor='w', edgecolor='k')
ax = plt.gca()
fig = plt.gcf()

directory = '../build/'
data = pd.read_csv(directory+'data.csv', sep=',', index_col=False)
mlog = pd.read_csv(directory+'mxlog.csv', sep=',', index_col=False)
pts = pd.read_csv(directory+'pts.csv', sep=',',index_col=False, header=None, names=['lon', 'lat'])
datanp = np.array(data)
ptsnp = np.array(pts)

# Plot payload locations and their paths
plt.plot([datanp[0,1], ptsnp[0,0]],[datanp[0,2], ptsnp[0,1]],color='green', linestyle='--')
for ii in range(1, len(pts)):
    plt.plot(ptsnp[ii,0],ptsnp[ii,1], marker='o', markersize=6, color='red')
    plt.plot([ptsnp[ii-1,0], ptsnp[ii,0]],[ptsnp[ii-1,1], ptsnp[ii,1]],color='green', linestyle='--')

# Collect all of the payload drop times
drops = mlog.loc[mlog['msg'] == 'CTL sent payload-drop and image-taken signals.' ]

# Starting location and payload drop locations
plt.plot(datanp[0,1],datanp[0,2], marker='o', markersize=10, color='blue')
for ii in range(0, len(drops)):
    time = drops.iloc[ii]['time']
    idx = np.argmin(abs(datanp[:,0]-time))
    plt.plot(datanp[idx,1],datanp[idx,2], marker='o', markersize=10, color='green')
    plt.plot([datanp[idx,1],ptsnp[ii,0]],[datanp[idx,2],ptsnp[ii,1]], color='black', linestyle='--')

# Plot positional data, with color dependent on velocity
x = datanp[:,1]
y = datanp[:,2]
v = datanp[:,4]
points = np.array([x, y]).T.reshape(-1, 1, 2) 
segments = np.concatenate([points[:-1], points[1:]], axis=1)
norm = plt.Normalize(v.min(), v.max())
lc = LineCollection(segments, cmap='viridis', norm=norm)
lc.set_array(v)
lc.set_linewidth(2)
line = ax.add_collection(lc)
fig.colorbar(line, ax=ax)

plt.grid(b=True, which='both', linestyle='-')
plt.xlabel('Lon (deg)')
plt.ylabel('Lat (deg)')
plt.title('Actual Path vs. Proposed NavPlan')
plt.axis('equal')

plt.savefig('pos_v_time.png', bbox_inches='tight')
