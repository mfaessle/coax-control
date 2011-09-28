load ViconData_yaw

time = Data.time;

qx = Data.orientation(1,:)';
qy = Data.orientation(2,:)';
qz = Data.orientation(3,:)';
qw = Data.orientation(4,:)';
    
vicon_yaw = atan2(2*(qw.*qz+qx.*qy),1-2*(qy.^2+qz.^2));
vicon_r = Data.angtwist(1,:);

imu_r = Data.imu_r';

figure(1)
plot(time,vicon_r)
hold on;
plot(time,imu_r,'r')
hold off;
legend('vicon-r','imu-r')


figure(2)
plot(time,vicon_yaw)
hold on;
plot(time,cumtrapz(time,-imu_r)+vicon_yaw(1),'r')
plot(time,cumtrapz(time,vicon_r)+vicon_yaw(1),'g')
hold off;
legend('vicon-yaw','integrated imu-r','integrated vicon-r')