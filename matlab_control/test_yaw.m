pid = nav_msgs_Odometry('connect','subscriber','odom56','odom56');

buflength = 200;
yaw = zeros(buflength,1);
time = zeros(buflength,1);

figureHandle = figure('NumberTitle','off','Name','Realtime Plot');
axesHandle = axes('Parent',figureHandle,'YGrid','on');
plotHandle = plot(axesHandle,time,yaw);
xlabel('time [s]');
ylabel('yaw [rad]');
title('Real Time Yaw Plot');

while(1)

    odom = nav_msgs_Odometry('read',pid,1);
    while (isempty(odom))
        odom = nav_msgs_Odometry('read',pid,1);
    end

    qx       = odom.pose.pose.orientation.x;
    qy       = odom.pose.pose.orientation.y;
    qz       = odom.pose.pose.orientation.z;
    qw       = odom.pose.pose.orientation.w;

    roll_new = atan2(2*(qw*qx+qy*qz),1-2*(qx^2+qy^2));
    pitch_new = asin(2*(qw*qy-qz*qx));
    yaw_new = atan2(2*(qw*qz+qx*qy),1-2*(qy^2+qz^2));
    time_new = odom.header.stamp;
    
    yaw(1:end-1) = yaw(2:end);
    yaw(end) = yaw_new;
    time(1:end-1) = time(2:end);
    time(end) = time_new;
    
    set(plotHandle,'YData',yaw,'XData',time);
    set(figureHandle,'Visible','on');
end


nav_msgs_Odometry('disconnect',pid);