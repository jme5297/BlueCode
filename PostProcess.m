clear
clc

dir = 'build/';
data = csvread(strcat(dir,'data.csv'));
pts = csvread(strcat(dir,'pts.csv'));

figure(1)
hold on
plot(data(:,2),data(:,3), 'b', 'linewidth',2);
for i = 1:size(pts,1)
    plot(pts(i,1), pts(i,2), '*r', 'linewidth',10)
end
hold off
grid minor
axis equal