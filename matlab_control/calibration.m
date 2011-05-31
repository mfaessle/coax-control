pid = nav_msgs_Odometry('connect','subscriber','odom56','odom56');
% iid = geometry_msgs_Quaternion('connect','subscriber','coax_info56','coax_info56');
% tid = geometry_msgs_Quaternion('connect','publisher','trim56','trim56');
% mid = std_msgs_Bool('connect','publisher','nav_mode56','nav_mode56');
% cid = geometry_msgs_Quaternion('connect','publisher','raw_control56','raw_control56');
% cmid = geometry_msgs_Quaternion('connect','subscriber','control_mode56','control_mode56');

N = 5000;
TimeStamps = zeros(1,N);
Positions = zeros(3,N);
Orientations = zeros(4,N);
Lintwists = zeros(3,N);
Angtwists = zeros(3,N);

i = 1;
while i<N+1
    
    % Read Vicon data
    A = nav_msgs_Odometry('read',pid,1);
    while (isempty(A))
        A = nav_msgs_Odometry('read',pid,1);
    end
    
    pos = [A.pose.pose.position.x A.pose.pose.position.y A.pose.pose.position.z]';
    ori = [A.pose.pose.orientation.x A.pose.pose.orientation.y A.pose.pose.orientation.z A.pose.pose.orientation.w]';
    lintwist = [A.twist.twist.linear.x A.twist.twist.linear.y A.twist.twist.linear.z]';
    angtwist = [A.twist.twist.angular.x A.twist.twist.angular.y A.twist.twist.angular.z]';
    
    if (i == 1)
        A.header.stamp
    end
    TimeStamps(i) = A.header.stamp;
    Positions(:,i) = pos;
    Orientations(:,i) = ori;
    Lintwists(:,i) = lintwist;
    Angtwists(:,i) = angtwist;

    i = i+1;
end

Data.time = TimeStamps - TimeStamps(1);
Data.position = Positions;
Data.orientation = Orientations;
Data.lintwist = Lintwists;
Data.angtwist = Angtwists;

save ViconData Data

nav_msgs_Odometry('disconnect',pid);
% geometry_msgs_Quaternion('disconnect',iid);
% geometry_msgs_Quaternion('disconnect',tid);
% std_msgs_Bool('disconnect',mid);
% geometry_msgs_Quaternion('disconnect',cid);
% geometry_msgs_Quaternion('disconnect',cmid);
clear nav_msgs_Odometry;

q = mean(Orientations,2);
x = q(1);
y = q(2);
z = q(3);
w = q(4);

R = [1-2*y^2-2*z^2 2*x*y-2*z*w 2*x*z+2*y*w; ...
     2*x*y+2*z*w 1-2*x^2-2*z^2 2*y*z-2*x*w; ...
     2*x*z-2*y*w 2*y*z+2*x*w 1-2*x^2-2*y^2];