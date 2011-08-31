load ViconData_hover_newhy

time = Data.time;
position = Data.position;
trajectory = Data.trajectory;


figure(4)
plot3(position(1,:),position(2,:),position(3,:))
hold on;
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'red')
hold off;
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis equal

figure(5)

subplot(3,1,1)
plot(time,position(1,:))
hold on;
plot(time,trajectory(1,:),'red')
hold off;
xlabel('time [s]')
ylabel('x-position [m]')

subplot(3,1,2)
plot(time,position(2,:))
hold on;
plot(time,trajectory(2,:),'red')
hold off;
xlabel('time [s]')
ylabel('y-position [m]')

subplot(3,1,3)
plot(time,position(3,:))
hold on;
plot(time,trajectory(3,:),'red')
hold off;
xlabel('time [s]')
ylabel('z-position [m]')

% error probability plots
states = position(1:2,:)';
traj = trajectory(1:2,:)';
horz_err = errorprobplot(states,traj,0);

states = position(3,:)';
traj = trajectory(3,:)';
vert_err = errorprobplot(states,traj,0);

states = position(1:3,:)';
traj = trajectory(1:3,:)';
tot_err = errorprobplot(states,traj,0);

x = linspace(0,100,length(time))';

figure(6)

subplot(3,1,1)
plot(x,horz_err)
grid on;
xlabel('Percentage of Time')
ylabel('Horizontal Position Error Norm [m]')
title(horzcat('Total Duration: ', num2str(time(end)-time(1)),'s'))

subplot(3,1,2)
plot(x,vert_err)
grid on;
xlabel('Percentage of Time')
ylabel('Vertical Position Error Norm [m]')
title(horzcat('Total Duration: ', num2str(time(end)-time(1)),'s'))

subplot(3,1,3)
plot(x,tot_err)
grid on;
xlabel('Percentage of Time')
ylabel('Total Position Error Norm [m]')
title(horzcat('Total Duration: ', num2str(time(end)-time(1)),'s'))