clear
clc

dirr = '../build/';
fileList = dir(strcat(dirr,'data*.csv'));
pts = csvread(strcat(dirr,'pts.csv'));

figure(1)
hold on
for i = 1:size(fileList,1)
    data = csvread(strcat(dirr,fileList(i).name));
    plot(data(:,2),data(:,3), 'b', 'linewidth',1);
end
for i = 1:size(pts,1)
    plot(pts(i,1), pts(i,2), '*r', 'linewidth',10)
end

hold off
grid minor
axis equal