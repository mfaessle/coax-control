pid = nav_msgs_Odometry('connect','subscriber','odom56','odom56');
% aid = geometry_msgs_Quaternion('connect','subscriber','add_state56','add_state56');
cid = geometry_msgs_Quaternion('connect','publisher','raw_control56','raw_control56');

run parameter
%run normalize_linearize
raw_control = geometry_msgs_Quaternion('empty');
odom = nav_msgs_Odometry('read',pid,1);

FIRST_RUN = 1;
VEL = 0.05;
% set desired position and orientation
end_position = [0 0 1.5]';
end_orientation = 0;

while (1)

    % Read Odometry message, rotor speeds and stabilizer bar angles
    odom = nav_msgs_Odometry('read',pid,1);
    while (isempty(odom))
        odom = nav_msgs_Odometry('read',pid,1);
    end
%     add_state = geometry_msgs_Quaternion('read',aid,1);
%     while (isempty(add_state))
%         add_state = geometry_msgs_Quaternion('read',aid,1);
%     end
    
    % get current time
    time = clock;
    
    % extract state values from messages
    x        = odom.pose.pose.position.x;
    y        = odom.pose.pose.position.y;
    z        = odom.pose.pose.position.z;
    qx       = odom.pose.pose.orientation.x;
    qy       = odom.pose.pose.orientation.y;
    qz       = odom.pose.pose.orientation.z;
    qw       = odom.pose.pose.orientation.w;
    xdot     = odom.twist.twist.linear.x;
    ydot     = odom.twist.twist.linear.y;
    zdot     = odom.twist.twist.linear.z;
    p        = odom.twist.twist.angular.x;
    q        = odom.twist.twist.angular.y;
    r        = odom.twist.twist.angular.z;
%     Omega_up = add_state.x;
%     Omega_lo = add_state.y;
%     a_up     = add_state.z;
%     b_up     = add_state.w;
    [x y z]
    roll = atan2(2*(qw*qx+qy*qz),1-2*(qx^2+qy^2));
    pitch = asin(2*(qw*qy-qz*qx));
    yaw = atan2(2*(qw*qz+qx*qy),1-2*(qy^2+qz^2));
    
    % coax state
    % state = [x y z xdot ydot zdot roll pitch yaw p q r Omega_up Omega_lo a_up b_up]';
    state = [x y z xdot ydot zdot roll pitch yaw p q r 0 0 0 0]';
    
    if (FIRST_RUN)
        start_position = state(1:3);
        start_time = time;
        dist = norm(end_position - start_position);
        dir = (end_position - start_position)/dist;
        duration = dist/VEL;
        FIRST_RUN = 0;
    end
    
    % Compute trajectory
    dt = etime(time, start_time);
    if (dt < duration)
        desPosition = start_position + VEL*dt*dir;
        trajectory = [desPosition' VEL*dir' 0 0 0 dt/duration*end_orientation end_orientation/duration]';
    else
        trajectory = [end_position' 0 0 0 0 0 0 end_orientation 0]';
    end
    
    % compute control commands
    control_inputs = coax_control(state,trajectory,param,cont_param);
    
    % send control commands
    raw_control.x = control_inputs(1);
    raw_control.y = control_inputs(2);
    raw_control.z = control_inputs(3);
    raw_control.w = control_inputs(4);
    geometry_msgs_Quaternion('send',cid,raw_control);

    
end

nav_msgs_Odometry('disconnect',pid);
% geometry_msgs_Quaternion('disconnect',aid);
geometry_msgs_Quaternion('disconnect',cid);