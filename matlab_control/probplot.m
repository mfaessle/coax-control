clear all;
clc;

load ViconData_hover_id

time = Data.time';


%%%
N = size(time,1);
x = linspace(0,100,N)';

states = Data.position(1:2,:)';
% trajectory = repmat(mean(Data.position(1:2,:),2)',N,1);
trajectory = Data.trajectory(1:2,:)';
horz_err = errorprobplot(states,trajectory,0);

states = Data.position(3,:)';
% trajectory = repmat(mean(Data.position(3,:),2)',N,1);
trajectory = Data.trajectory(3,:)';
vert_err = errorprobplot(states,trajectory,0);

states = Data.position(1:3,:)';
% trajectory = repmat(mean(Data.position(1:3,:),2)',N,1);
trajectory = Data.trajectory(1:3,:)';
tot_err = errorprobplot(states,trajectory,0);


figure(1)
plot(x,horz_err)
grid on;
xlabel('Percentage of Time')
ylabel('Horizontal Position Error Norm [m]')
title(horzcat('Total Duration: ', num2str(time(end)-time(1)),'s'))

figure(2)
plot(x,vert_err)
grid on;
xlabel('Percentage of Time')
ylabel('Vertical Position Error Norm [m]')
title(horzcat('Total Duration: ', num2str(time(end)-time(1)),'s'))

figure(3)
plot(x,tot_err)
grid on;
xlabel('Percentage of Time')
ylabel('Total Position Error Norm [m]')
title(horzcat('Total Duration: ', num2str(time(end)-time(1)),'s'))
