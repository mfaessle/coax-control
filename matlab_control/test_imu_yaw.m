pid = nav_msgs_Odometry('connect','subscriber','odom56','odom56');
imuid = geometry_msgs_Quaternion('connect','subscriber','coax_imu56','coax_imu56');

odom = nav_msgs_Odometry('read',pid,1);
imu = geometry_msgs_Quaternion('read',imuid,1);

N = 2000;
vicon_time = zeros(N,1);
vicon_yaw = zeros(N,1);
vicon_r = zeros(N,1);
imu_time = zeros(N/2,1);
imu_yaw = zeros(N/2,1);
imu_r = zeros(N/2,1);

pause(5);

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
    vicon_yaw(i) = atan2(2*(qw*qz+qx*qy),1-2*(qy^2+qz^2));
    vicon_r(i) = odom.twist.twist.angular.z;
    if(~isempty(imu))
        imu_time(j) = time;
        imu_yaw(j) = imu.z;
        imu_r(j) = imu.w;
        j = j+1;
    end
    
end

nav_msgs_Odometry('disconnect',pid);
geometry_msgs_Quaternion('disconnect',imuid);

%% Plots

figure(1)
plot(vicon_time,vicon_yaw)
hold on;
plot(imu_time,imu_yaw+(vicon_yaw(1)-imu_yaw(1)),'r')
hold off;
legend('vicon-yaw','imu-yaw')

figure(2)
plot(vicon_time,vicon_r)
hold on;
plot(imu_time,imu_r,'r')
hold off;
legend('vicon-r','imu-r')
