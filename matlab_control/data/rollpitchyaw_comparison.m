load ImuRollPitchYaw56

vicon_time = Data.vicon_time;
imu_time = Data.imu_time;

vicon_roll = Data.vicon_angles(:,1);
vicon_pitch = Data.vicon_angles(:,2);
vicon_yaw = Data.vicon_angles(:,3);
vicon_p = Data.vicon_rates(:,1);
vicon_q = Data.vicon_rates(:,2);
vicon_r = Data.vicon_rates(:,3);
imu_p = Data.imu_rates(:,1);
imu_q = Data.imu_rates(:,2);
imu_r = Data.imu_rates(:,3);
imu_roll = Data.imu_euler(:,1);
imu_pitch = Data.imu_euler(:,2);
imu_yaw = Data.imu_euler(:,3);

figure(1)

subplot(2,1,1)
plot(vicon_time,vicon_p)
hold on;
plot(imu_time,imu_p,'r')
hold off;
legend('vicon-p','imu-p')

subplot(2,1,2)
plot(vicon_time,vicon_roll)
hold on;
plot(imu_time,imu_roll,'r')
hold off;
legend('vicon-roll','imu-roll')

figure(2)

subplot(2,1,1)
plot(vicon_time,vicon_q)
hold on;
plot(imu_time,imu_q,'r')
hold off;
legend('vicon-q','imu-q')

subplot(2,1,2)
plot(vicon_time,vicon_pitch)
hold on;
plot(imu_time,-imu_pitch,'r')
hold off;
legend('vicon-pitch','imu-pitch')

figure(3)

subplot(2,1,1)
plot(vicon_time,vicon_r)
hold on;
plot(imu_time,imu_r,'r')
hold off;
legend('vicon-r','imu-r')

subplot(2,1,2)
plot(vicon_time,vicon_yaw)
hold on;
plot(imu_time,-imu_yaw+vicon_yaw(1)+imu_yaw(1),'r')
hold off;
legend('vicon-yaw','imu-yaw')