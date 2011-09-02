load ViconData_step_yaw

time = Data.time;
position = Data.position;
trajectory = Data.trajectory;

qx = Data.orientation(1,:)';
qy = Data.orientation(2,:)';
qz = Data.orientation(3,:)';
qw = Data.orientation(4,:)';
yaw = atan2(2*(qw.*qz+qx.*qy),1-2*(qy.^2+qz.^2));

figure(1)

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