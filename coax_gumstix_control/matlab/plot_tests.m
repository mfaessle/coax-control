load report_step_z

figure(1)
plot3(Data.position(1,:),Data.position(2,:),Data.position(3,:))
hold on;
plot3(Data.trajectory(1,:),Data.trajectory(2,:),Data.trajectory(3,:),'red')
hold off;
legend('actual','nominal')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')

figure(2)

subplot(2,1,1)
plot(Data.time,Data.position)
hold on;
plot(Data.time,Data.trajectory(1:3,:),'--')
hold off;
legend('actual','nominal')
ylabel('Position [m]')

subplot(2,1,2)
plot(Data.time,Data.lintwist)
hold on;
plot(Data.time,Data.trajectory(4:6,:),'--')
hold off;
legend('actual','nominal')
ylabel('Velocity [m/s]')
xlabel('Time [s]')

% figure(3)
% 
% subplot(2,1,1)
% plot(Data.time,Data.inputs(1:3,:))
% legend('Forces')
% 
% subplot(2,1,2)
% plot(Data.time,Data.inputs(4,:))
% legend('yaw moment')

% qx = Data.orientation(1,:)';
% qy = Data.orientation(2,:)';
% qz = Data.orientation(3,:)';
% qw = Data.orientation(4,:)';
% yaw = atan2(2*(qw.*qz+qx.*qy),1-2*(qy.^2+qz.^2));
% pitch = asin(-(2*qx.*qz-2*qy.*qw));

figure(4)
plot(Data.time,Data.position-Data.trajectory(1:3,:))
ylabel('Position Error [m]')