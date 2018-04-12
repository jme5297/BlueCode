import matplotlib
matplotlib.use('Agg')
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib import cm

fig = plt.figure(num=None, figsize=(16, 9), dpi=80, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111, projection='3d')

directory = '../build/'
data = pd.read_csv(directory+'data.csv', sep=',', index_col=False)
mxlog = pd.read_csv(directory+'mxlog.csv', sep=',', index_col=False)
pts = pd.read_csv(directory+'pts.csv', sep=',',index_col=False, header=None, names=['lon', 'lat'])
datanp = np.array(data)
ptsnp = np.array(pts)

ax.set_xlim(min(data['lon']),max(data['lon']))
ax.set_ylim(min(data['lat']),max(data['lat']))
ax.set_zlim(min(data['vel']),max(data['vel']))

x = datanp[:,1]
y = datanp[:,2]
z = datanp[:,4]
throt = 0.5*(datanp[:-1,8]+datanp[1:,8])

points = np.array([x, y, z]).T.reshape(-1, 1, 3) 
segments = np.concatenate([points[:-1], points[1:]], axis=1)

# Create a continuous norm to map from data points to colors
norm = plt.Normalize(throt.min(), throt.max())
lc = Line3DCollection(segments, cmap='viridis', norm=norm)
lc.set_array(throt)
lc.set_linewidth(2)
line = ax.add_collection3d(lc, zs=z)

# Create colorbar
m = cm.ScalarMappable(cmap='viridis')
m.set_array([min(throt),max(throt)])
m.set_clim(vmin=min(throt),vmax=max(throt))
fig.colorbar(m)

ax.set_xlabel('Lon (deg)')
ax.set_ylabel('Lat (deg)')
ax.set_zlabel('Vel (m/s)')
ax.set_title('Actual Path')

plt.savefig('multi_v_time.png', bbox_inches='tight')