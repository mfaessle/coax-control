pid = nav_msgs_Odometry('connect','subscriber','odom56','odom56');
imuid = geometry_msgs_Quaternion('connect','subscriber','coax_imu56','coax_imu56');


N = 2000;
vicon_time = zeros(N,1);
vicon_roll = zeros(N,1);
vicon_pitch = zeros(N,1);
vicon_yaw = zeros(N,1);
vicon_p = zeros(N,1);
vicon_q = zeros(N,1);
vicon_r = zeros(N,1);
imu_time = zeros(N/2,1);
imu_yaw = zeros(N/2,1);
imu_p = zeros(N/2,1);
imu_q = zeros(N/2,1);
imu_r = zeros(N/2,1);

pause(5);

for i = 1:3
    odom = nav_msgs_Odometry('read',pid,1);
    imu = geometry_msgs_Quaternion('read',imuid,1);
end

j = 1;
tic
for i = 1:N
    
    time = toc;
    
    odom = nav_msgs_Odometry('read',pid,1);
    while (isempty(odom))
        odom = nav_msgs_Odometry('read',pid,1);
    end
    imu = geometry_msgs_Quaternion('read',imuid,1);

    qx       = odom.pose.pose.orientation.x;
    qy       = odom.pose.pose.orientation.y;
    qz       = odom.pose.pose.orientation.z;
    qw       = odom.pose.pose.orientation.w;
    
    vicon_time(i) = time;
    vicon_roll(i) = atan2(2*(qw*qx+qy*qz),1-2*(qx^2+qy^2));
    vicon_pitch(i) = asin(2*(qw*qy-qz*qx));
    vicon_yaw(i) = atan2(2*(qw*qz+qx*qy),1-2*(qy^2+qz^2));
    vicon_p(i) = odom.twist.twist.angular.x;
    vicon_q(i) = odom.twist.twist.angular.y;
    vicon_r(i) = odom.twist.twist.angular.z;
    if(~isempty(imu))
        imu_time(j) = time;
        imu_p(j) = imu.x;
        imu_q(j) = imu.y;
        imu_r(j) = imu.z;
        j = j+1;
    end
    
end

Data.imu_rates = [imu_p imu_q imu_r];
Data.imu_time = imu_time;
Data.vicon_rates = [vicon_p vicon_q vicon_r];
Data.vicon_angles = [vicon_roll vicon_pitch vicon_yaw];
Data.vicon_time = vicon_time;

save ImuRollPitchYaw Data

nav_msgs_Odometry('disconnect',pid);
geometry_msgs_Quaternion('disconnect',imuid);

%% Plots

% figure(1)
% plot(vicon_time,vicon_yaw)
% hold on;
% plot(imu_time,imu_yaw+(vicon_yaw(1)-imu_yaw(1)),'r')
% hold off;
% legend('vicon-yaw','imu-yaw')
% 
% figure(2)
% plot(vicon_time,vicon_r)
% hold on;
% plot(imu_time,imu_r,'r')
% hold off;
% legend('vicon-r','imu-r')

figure(1)
plot(vicon_time,vicon_p)
hold on;
plot(imu_time,imu_p,'r')
hold off;
legend('vicon-p','imu-p')

figure(2)
plot(vicon_time,vicon_roll)
hold on;
plot(imu_time,cumtrapz(imu_time,imu_p)+vicon_roll(1),'r')
hold off;
legend('vicon-roll','integrated imu-p')



