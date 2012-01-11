load test_euler_values

qx = Data.orientation(1,:)';
qy = Data.orientation(2,:)';
qz = Data.orientation(3,:)';
qw = Data.orientation(4,:)';
roll = atan2(2*qy.*qz+2*qx.*qw,qw.^2-qx.^2-qy.^2+qz.^2);
pitch = asin(-(2*qx.*qz-2*qy.*qw));
yaw = atan2(2*(qw.*qz+qx.*qy),1-2*(qy.^2+qz.^2));

figure(1)
plot(Data.time,Data.gyros(3,:)-(Data.gyros(3,1)-yaw(1)))
hold on;
plot(Data.time,yaw,'r')
legend('imu','vicon')

% figure(2)
% plot(Data.time,Data.angtwist(2,:),'r')