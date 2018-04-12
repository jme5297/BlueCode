clear
clc

directory = '../build/';
dataset = 0;
headFactor = 0.00003;

data = table2array(readtable(strcat(directory,'data.csv')));
mxlog = readtable(strcat(directory,'mxlog.csv'));
pts = table2array(readtable(strcat(directory,'pts.csv')));

plRad = 9.13;
piRng = 0:0.01:2*pi;
latToM = 111050.0;
lonToM = 84397.0;

figure(1)
hold on

for i = 1:length(pts)-1
    plot(pts(i,1), pts(i,2), '*r', 'linewidth',10)
    plot(pts(i,1) + plRad/lonToM*cos(piRng), pts(i,2) + plRad/latToM*sin(piRng), 'r', 'linewidth',1);
end
plot(pts(length(pts),1), pts(length(pts),2), '*b', 'linewidth',10)
plot(pts(length(pts),1) + plRad/lonToM*cos(piRng), pts(length(pts),2) + plRad/latToM*sin(piRng), 'b', 'linewidth',1);

x = data(:,2)';
y = data(:,3)';
z = data(:,5)';
t = data(:,1)';
tf = data(size(data,1),1);
col = [data(:,9)';data(:,9)'];

surface([x;x],[y;y],[z;z],col,...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);
colorbar

xlabel('Lon (deg)')
ylabel('Lat (deg)')
zlabel('Something');
title('Five-Waypoint Test With Simulated Hill')
hold off
grid minor
daspect([1 1 10000.0])
%axis equal