pid = nav_msgs_Odometry('connect','subscriber','odom56','odom56');
cdid = geometry_msgs_Pose('connect','subscriber','coax_data','coax_data');

for k = 1:3
    odom = nav_msgs_Odometry('read',pid,1);
    coax_data = geometry_msgs_Pose('read',cdid,1);
end
pause(0.1);

position = zeros(3,1);
rpy = zeros(3,1);
linvel = zeros(3,1);
angvel = zeros(3,1);

INPUTS = zeros(4,1);
POS = zeros(3,1);
RPY = zeros(3,1);
LINVEL = zeros(3,1);
ANGVEL = zeros(3,1);
TIME = 0;

N = 6000;

tic
for k = 1:N
    
    coax_data = geometry_msgs_Pose('read',cdid,1);
    while (isempty(coax_data))
        coax_data = geometry_msgs_Pose('read',cdid,1);
    end    
    odom = nav_msgs_Odometry('read',pid,1);
    
    if (~isempty(odom))
        position = [odom.pose.pose.position.x odom.pose.pose.position.y odom.pose.pose.position.z]';
        qx = odom.pose.pose.orientation.x;
        qy = odom.pose.pose.orientation.y;
        qz = odom.pose.pose.orientation.z;
        qw = odom.pose.pose.orientation.w;
        roll = atan2(2*qy*qz+2*qx*qw,qw^2-qx^2-qy^2+qz^2);
        pitch = asin(-(2*qx*qz-2*qy*qw));
        yaw = atan2(2*qx*qy+2*qz*qw,qw^2+qx^2-qy^2-qz^2);
        rpy = [roll pitch yaw]';
        linvel = [odom.twist.twist.linear.x odom.twist.twist.linear.y odom.twist.twist.linear.z]';
        angvel = [odom.twist.twist.angular.x odom.twist.twist.angular.y odom.twist.twist.angular.z]';
    end
    
    INPUTS(:,k) = [coax_data.orientation.x coax_data.orientation.y coax_data.orientation.z -coax_data.orientation.w]';
    POS(:,k) = position;
    RPY(:,k) = rpy;
    LINVEL(:,k) = linvel;
    ANGVEL(:,k) = [angvel(1:2); coax_data.position.z];
    TIME(k) = toc;
    
end
toc

Data.time = TIME - TIME(1);
Data.inputs = INPUTS;
Data.position = POS;
Data.rpy = RPY;
Data.linvel = LINVEL;
Data.angvel = ANGVEL;

% save manualidValidate Data

nav_msgs_Odometry('disconnect',pid);
geometry_msgs_Pose('disconnect',cdid);
clear