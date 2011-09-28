load ViconData_heaveyaw

time = Data.time;
position = Data.position;
trajectory = Data.trajectory;

qx = Data.orientation(1,:)';
qy = Data.orientation(2,:)';
qz = Data.orientation(3,:)';
qw = Data.orientation(4,:)';
yaw = atan2(2*(qw.*qz+qx.*qy),1-2*(qy.^2+qz.^2));

figure(1)
plot3(position(1,:),position(2,:),position(3,:))
hold on;
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'red')
hold off;
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis equal

figure(2)

subplot(2,2,1)
plot(time,position(1,:))
hold on;
plot(time,trajectory(1,:),'red')
hold off;
xlabel('time [s]')
ylabel('x-position [m]')

subplot(2,2,2)
plot(time,position(2,:))
hold on;
plot(time,trajectory(2,:),'red')
hold off;
xlabel('time [s]')
ylabel('y-position [m]')

subplot(2,2,3)
plot(time,position(3,:))
hold on;
plot(time,trajectory(3,:),'red')
hold off;
xlabel('time [s]')
ylabel('z-position [m]')

subplot(2,2,4)
plot(time,yaw)
hold on;
plot(time,trajectory(10,:),'red')
hold off;
xlabel('time [s]')
ylabel('yaw [rad]')

% error probability plots
horz_err = errorprobplot(position(1:2,:)',trajectory(1:2,:)',0);
vert_err = errorprobplot(position(3,:)',trajectory(3,:)',0);
tot_err = errorprobplot(position(1:3,:)',trajectory(1:3,:)',0);
yaw_err = errorprobplot(yaw,trajectory(10,:)',0);

x = linspace(0,100,length(time))';

figure(3)

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

figure(4)
plot(x,yaw_err)
grid on;
xlabel('Percentage of Time')
ylabel('Yaw Error Norm [m]')
title(horzcat('Total Duration: ', num2str(time(end)-time(1)),'s'))
