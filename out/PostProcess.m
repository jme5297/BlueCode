clear
clc

dirr = '../output/';
fileList = dir(strcat(dirr,'data*.csv'));
pts = csvread(strcat(dirr,'pts.csv'));

plRad = 9.5;
piRng = 0:0.01:2*pi;
latToM = 111050.0;
lonToM = 84397.0;

figure(1)
hold on

for i = [1,2,3,4,5]
    plot(pts(i,1), pts(i,2), '*r', 'linewidth',10)
    plot(pts(i,1) + plRad/lonToM*cos(piRng), pts(i,2) + plRad/latToM*sin(piRng), 'r', 'linewidth',1);
end

i = 3;
data = csvread(strcat(dirr,'data_9.csv'));

x = data(:,2)';
y = data(:,3)';
z = zeros(size(data,1),1)';
t = data(:,1)';
tf = data(size(data,1),1);
col = [t;0.7*t];

surface([x;x],[y;y],[z;z],col,...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);


xlabel('Lon (deg)')
ylabel('Lat (deg)')
title('Five-Waypoint Test With Obstacles')
hold off
grid minor
axis equal