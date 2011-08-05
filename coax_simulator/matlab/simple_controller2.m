pid = nav_msgs_Odometry('connect','subscriber','odom56','odom56');
% aid = geometry_msgs_Quaternion('connect','subscriber','add_state56','add_state56');
cid = geometry_msgs_Quaternion('connect','publisher','raw_control56','raw_control56');

run parameter
if(~exist('K_lqr'))
    run normalize_linearize
end
raw_control = geometry_msgs_Quaternion('empty');
odom = nav_msgs_Odometry('read',pid,1);

Ts = 0.01;

Omega_up = 0;
Omega_lo = 0;
a_up = 0;
b_up = 0;
prev_z_bar = [0 0 1]';
z_bar = [0 0 1]';

idle_time = 0;

FIRST_RUN = 1;
VEL = 0.05;
% set desired position and orientation
end_position = [0.1 0 0.5]';
end_orientation = 0;

States = [];
Inputs = [];
Time = [];
ROSStates = [];

state = [0 0 0  0 0 0  0 0 0  0 0 0  0 0  0 0 1]';


time = 0;
while time<20 %(1)

    % Read Odometry message, rotor speeds and stabilizer bar angles
    odom = nav_msgs_Odometry('read',pid,1);
    while (isempty(odom))
        odom = nav_msgs_Odometry('read',pid,1);
    end
    
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
    z_barx   = z_bar(1);
    z_bary   = z_bar(2);
    z_barz   = z_bar(3);

    %[x y z]
    roll = atan2(2*(qw*qx+qy*qz),1-2*(qx^2+qy^2));
    pitch = asin(2*(qw*qy-qz*qx));
    yaw = atan2(2*(qw*qz+qx*qy),1-2*(qy^2+qz^2));
    
    % coax state
    % state = [x y z xdot ydot zdot roll pitch yaw p q r Omega_up Omega_lo z_bar']';
    ROSstate = [x y z xdot ydot zdot roll pitch yaw p q r Omega_up Omega_lo z_bar']';
    
    if ((time >= idle_time) && FIRST_RUN)
        start_position = state(1:3);
        start_orientation = state(9);
        start_time = time;
        dist = norm(end_position - start_position);
        dir = (end_position - start_position)/dist;
        duration = dist/VEL;
        FIRST_RUN = 0;
    end
    if (time < idle_time)
        control_inputs = [0.35 0.35 0 0];
    else
        % Compute trajectory
        dt = time - start_time;
        if (dt < duration)
            desPosition = start_position + VEL*dt*dir;
            trajectory = [desPosition' VEL*dir' 0 0 0 dt/duration*(end_orientation-start_orientation)+start_orientation (end_orientation-start_orientation)/duration]';
        else
            trajectory = [end_position' 0 0 0 0 0 0 end_orientation 0]';
        end
    
        % compute control commands
        Omega_lo0 = sqrt(m*g/(k_Tup*k_Mlo/k_Mup + k_Tlo));
        Omega_up0 = sqrt(k_Mlo/k_Mup*Omega_lo0^2);
        % control_inputs = [(Omega_up0 - rs_bup)/rs_mup (Omega_lo0 - rs_blo)/rs_mlo -0.2 0.3]';
        control_inputs = coax_control(state,trajectory,param,cont_param);
    end
    
    % estimate internal states
    Omega_up_des = rs_mup*control_inputs(1) + rs_bup;
    Omega_lo_des = rs_mlo*control_inputs(2) + rs_blo;
    Omega_up = Omega_up + 1/Tf_motup*(Omega_up_des - Omega_up)*Ts;
    Omega_lo = Omega_lo + 1/Tf_motlo*(Omega_lo_des - Omega_lo)*Ts;

    % stabilizer bar orientation
    b_z_bardotz = 1/Tf_up*acos(prev_z_bar(3))*sqrt(prev_z_bar(1)^2 + prev_z_bar(2)^2);
    if (b_z_bardotz <= 0)
        b_z_bardot = [0 0 0]';
    else
        temp = prev_z_bar(3)*b_z_bardotz/(prev_z_bar(1)^2+prev_z_bar(2)^2);
        b_z_bardot = [-prev_z_bar(1)*temp -prev_z_bar(2)*temp b_z_bardotz]';
    end

    A_k = [0 r -q; -r 0 p; q -p 0];

    z_bar = prev_z_bar + (A_k*prev_z_bar + b_z_bardot)*Ts;
    z_bar = z_bar/norm(z_bar);
    prev_z_bar = z_bar;

    % send control commands
    raw_control.x = control_inputs(1);
    raw_control.y = control_inputs(2);
    raw_control.z = control_inputs(3);
    raw_control.w = control_inputs(4);
    geometry_msgs_Quaternion('send',cid,raw_control);
    
    Time = [Time; time];
    States = [States; state'];
    Inputs = [Inputs; control_inputs'];
    ROSStates = [ROSStates; ROSstate'];
    
    tstart = time;
    tstop = time + Ts;
    [~,state_inter] = ode45(@CoaX_grey_box,[tstart tstop],state,[],control_inputs,m,g,Ixx,Iyy,Izz,d_up,d_lo,k_springup,k_springlo,l_up,l_lo,k_Tup,k_Tlo,k_Mup,k_Mlo,Tf_motup,Tf_motlo,Tf_up,rs_mup,rs_bup,rs_mlo,rs_blo,zeta_mup,zeta_bup,zeta_mlo,zeta_blo,max_SPangle);
    
    state = state_inter(end,:)';
    
    if (state(3) <= 0)
        state(3) = 0.01;
        state(4:6) = [0 0 0]';
    end
    
    time = tstop;
    
end

Data.time = Time;
Data.state = States;
Data.input = Inputs;
Data.rosstate = ROSStates;

save ROSData Data

nav_msgs_Odometry('disconnect',pid);
% geometry_msgs_Quaternion('disconnect',aid);
geometry_msgs_Quaternion('disconnect',cid);


%% plot
close all

figure(1)
plot(Data.time,Data.rosstate(:,1:3))
legend('x','y','z')
title('ROS States')

figure(2)
plot(Data.time,Data.state(:,1:3))
legend('x','y','z')
title('Matlab States')

figure(3)
subplot(2,1,1)
plot(Data.time,Data.input(:,1:2))
legend('u_{mot,up}','u_{mot,lo}')
title('Motor Inputs')

subplot(2,1,2)
plot(Data.time,Data.input(:,3:4))
legend('u_{serv1}','u_{serv2}')
title('Servo Inputs')