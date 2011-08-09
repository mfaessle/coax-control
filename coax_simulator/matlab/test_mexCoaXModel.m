% [a, p] = system('rospack find coax_simulator');
% addpath(strcat(p, '/bin'));

% Remember, always reset the model if you plan to start the
% simulation over

clear
clc

run parameter
if(~exist('K_lqr','var'))
    run normalize_linearize
end

p.mass = m;
p.inertia = [Ixx, Iyy, Izz];
p.offset = [d_up, d_lo];
p.linkage_factor = [l_up, l_lo];
p.spring_constant = [k_springup, k_springlo];
p.thrust_factor = [k_Tup, k_Tlo];
p.moment_factor = [k_Mup, k_Mlo];
p.following_time = [Tf_up, Tf_motup, Tf_motlo];
p.speed_conversion = [rs_mup, rs_bup, rs_mlo, rs_blo];
p.phase_lag = [zeta_mup, zeta_bup, zeta_mlo, zeta_blo];
p.max_swashplate_angle = max_SPangle;

% Reset the model
mexCoaXModel('reset');

% Set the parameters
mexCoaXModel('set_params', p);

% Set the initial state
Omega_lo0 = sqrt(m*g/(k_Tup*k_Mlo/k_Mup + k_Tlo));
Omega_up0 = sqrt(k_Mlo/k_Mup*Omega_lo0^2);

s.position = [0, 0, 0.01];
s.linear_velocity = [0, 0, 0];
s.rotation = [0, 0, 0];
s.angular_velocity = [0, 0, 0];
s.rotor_speed = [Omega_up0, Omega_lo0];
s.bar_direction = [0, 0, 1];

mexCoaXModel('set_state', s);

x0 = [s.position s.linear_velocity s.rotation s.angular_velocity s.rotor_speed s.bar_direction]';

% trajectory parameters
FIRST_RUN = 1;
VEL = 0.05;
end_position = [0.1 0 0.3]';
end_orientation = 0;
idle_time = 0;

N = 2000;
U = zeros(N,4);
X = zeros(N,17);
XROS = zeros(N,17);
T = zeros(N,1);
ACCROS = zeros(N,3);

dt = 0.01;
t0 = 0;
X(1,:) = x0';
XROS(1,:) = x0';
T(1) = t0;

x = x0;
tnow = t0+dt;
t = t0;

for i = 1:N-1
    
    tau = tnow - t0;
    if ((tau >= idle_time) && FIRST_RUN)
        start_position = s.position';
        start_orientation = s.rotation(3);
        dist = norm(end_position - start_position);
        dir = (end_position - start_position)/dist;
        duration = dist/VEL;
        FIRST_RUN = 0;
    end
    
    if (tau < idle_time)
        trajectory = [0 0 0.08 0 0 0 0 0 0 0 0]';
    else
        if (tau < duration)
            desPosition = start_position + VEL*tau*dir;
            trajectory = [desPosition' VEL*dir' 0 0 0 tau/duration*(end_orientation-start_orientation)+start_orientation (end_orientation-start_orientation)/duration]';
        else
            trajectory = [end_position' 0 0 0 0 0 0 end_orientation 0]';
        end
    end
    
    control = coax_control(x,trajectory,param,cont_param);
    %%%%%%%%%%%%%%%%%%%
    
    % Set the command
    % control = [(Omega_up0 - rs_bup)/rs_mup, (Omega_lo0 - rs_blo)/rs_mlo, 0, 0]';
    % control = [0, 0, 0, 0];
    mexCoaXModel('set_cmd', control);

    % Update until the time provided
    mexCoaXModel('update', tnow);

    % Get the state after the update
    s = mexCoaXModel('get_state');
    
    %%%%%%%%%%%%%%%%%%%
    [time,state] = ode45(@CoaX_grey_box,[t tnow],x,[],control,m,g,Ixx,Iyy,Izz,d_up,d_lo,k_springup,k_springlo,l_up,l_lo,k_Tup,k_Tlo,k_Mup,k_Mlo,Tf_motup,Tf_motlo,Tf_up,rs_mup,rs_bup,rs_mlo,rs_blo,zeta_mup,zeta_bup,zeta_mlo,zeta_blo,max_SPangle);
    
    t = tnow;
    x = state(end,:)';
    
    %%%%%%%%%%%%%%%%%%%
    
    U(i,:) = control';
    X(i+1,:) = x';
    XROS(i+1,:) = [s.position s.linear_velocity s.rotation s.angular_velocity s.rotor_speed s.bar_direction];
    T(i+1) = tnow;
    ACCROS(i+1,:) = s.acceleration;
    
    tnow = tnow + dt;
end


%% Plot
close all

figure(1)

subplot(3,1,1)
plot(T,X(:,1:3))
hold on;
plot(T,XROS(:,1:3),'--')
hold off;
legend('x','y','z','xROS','yROS','zROS')
title('Position')
axis tight

subplot(3,1,2)
plot(T,X(:,4:6))
hold on;
plot(T,XROS(:,4:6),'--')
hold off;
legend('xdot','ydot','zdot','xdot ROS','ydot ROS','zdot ROS')
title('Linear Velocity')
axis tight

subplot(3,1,3)
plot(T,[zeros(1,3); diff(X(:,4:6))/dt])
hold on;
plot(T,ACCROS,'--')
hold off;
legend('xddot','yddot','zddot','xddot ROS','yddot ROS','zddot ROS')
title('Linear Acceleration')
axis tight

figure (2)
subplot(2,1,1)
plot(T,X(:,7:9))
hold on;
plot(T,XROS(:,7:9),'--')
hold off;
legend('\phi','\theta','\psi','\phi ROS','\theta ROS','\psi ROS')
title('Rotation')
axis tight

subplot(2,1,2)
plot(T,X(:,10:12))
hold on;
plot(T,XROS(:,10:12),'--')
hold off;
legend('\phidot','\thetadot','\psidot','\phidotROS','\thetadotROS','\psidotROS')
title('Angular Velocity')
axis tight

figure(3)
plot(T,X(:,13:14))
hold on;
plot(T,XROS(:,13:14),'--')
hold off;
legend('\omega_{up}','\omega_{lo}','\omega_{up} ROS','\omega_{lo} ROS')
title('Rotor speeds')
axis tight

figure(4)
plot(T,X(:,15:17))
hold on;
plot(T,XROS(:,15:17),'--')
hold off;
legend('x','y','z','x ROS','y ROS','z ROS')
title('Stabilizer Bar Orientation')
axis tight

% figure(5)
% 
% subplot(2,1,1)
% plot(T,U(:,1:2))
% legend('u_{mot,up}','u_{mot,lo}')
% title('Motor Inputs')
% axis tight
% 
% subplot(2,1,2)
% plot(T,U(:,3:4))
% legend('u_{serv1}','u_{serv2}')
% title('Servo Inputs')
% axis tight
