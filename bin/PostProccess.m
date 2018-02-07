data = csvread('out.csv');

X = data(:,2);
Y = data(:,3);

figure(1)
hold on
plot(X,Y);
axis equal