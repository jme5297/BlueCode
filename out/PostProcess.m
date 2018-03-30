clear
clc

dirr = '../output/';
fileList = dir(strcat(dirr,'data*.csv'));
pts = csvread(strcat(dirr,'pts.csv'));

plRad = 10.0;
piRng = 0:0.01:2*pi;
latToM = 111050.0;
lonToM = 84397.0;

figure(1)
hold on
for i = 1:size(pts,1)
    plot(pts(i,1), pts(i,2), '*r', 'linewidth',10)
    plot(pts(i,1) + plRad/lonToM*cos(piRng), pts(i,2) + plRad/latToM*sin(piRng), 'r', 'linewidth',1);
end
for i = 1:size(fileList,1)
    i
    data = csvread(strcat(dirr,fileList(i).name));
    plot(data(1:size(data,1)-1,2),data(1:size(data,1)-1,3), 'linewidth',0.5);
end

hold off
grid minor
axis equal