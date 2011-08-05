%%
clear
clc

run parameter
if(~exist('K_lqr'))
    run normalize_linearize
end

%%
stepsize = 1e-2;
tsim = 20;

% x = [x y z xdot ydot zdot roll pitch yaw p q r Omega_up Omega_lo a_up b_up]

Omega_lo0 = sqrt(m*g/(k_Tup*k_Mlo/k_Mup + k_Tlo));
Omega_up0 = sqrt(k_Mlo/k_Mup*Omega_lo0^2);

t0 = 0;
x0 = [0 0 0  0 0 0  0 0 0  0 0 0  0 0  0 0 1]';

% Outputs
X = [];
T = [];
U = [];

FIRST_RUN = 1;
VEL = 0.05;
end_position = [0.1 0 0.5]';
end_orientation = 0;

t = t0;
x = x0;
tic
while (t < tsim)
    
    tstart = t;
    tstop = tstart + stepsize;
    
    if (FIRST_RUN)
        start_position = x(1:3);
        start_orientation = x(9);
        dist = norm(end_position - start_position);
        dir = (end_position - start_position)/dist;
        duration = dist/VEL;
        FIRST_RUN = 0;
    end
    
    % Compute trajectory
    dt = t-t0;
    if (dt < duration)
        desPosition = start_position + VEL*dt*dir;
        trajectory = [desPosition' VEL*dir' 0 0 0 dt/duration*(end_orientation-start_orientation)+start_orientation (end_orientation-start_orientation)/duration]';
    else
        trajectory = [end_position' 0 0 0 0 0 0 end_orientation 0]';
    end
    
    % control = [u_motup u_motlo u_serv1 u_serv2]  u_mot in [0,1]; u_serv in [-1, 1]
    control = coax_control(x,trajectory,param,cont_param);
    % control = [(Omega_up0 - rs_bup)/rs_mup (Omega_lo0 - rs_blo)/rs_mlo 0 0]';
    
    %[time,state] = ode45(@coax_eom,[tstart tstop],x,[],control,param);
    [time,state] = ode45(@CoaX_grey_box,[tstart tstop],x,[],control,m,g,Ixx,Iyy,Izz,d_up,d_lo,k_springup,k_springlo,l_up,l_lo,k_Tup,k_Tlo,k_Mup,k_Mlo,Tf_motup,Tf_motlo,Tf_up,rs_mup,rs_bup,rs_mlo,rs_blo,zeta_mup,zeta_bup,zeta_mlo,zeta_blo,max_SPangle);
    
    t = time(end);
    x = state(end,:)';
    
    if (x(3) <= 0)
        x(3) = 0.01;
        x(4:6) = [0 0 0]';
    end
    
    X = [X; x'];
    T = [T; t];
    U = [U; control'];
end
toc

%% Plot
close all

figure(1)

subplot(4,1,1)
plot(T,X(:,1:6))
legend('x','y','z','u','v','w')
title('Translational states')
axis tight

subplot(4,1,2)
plot(T,X(:,7:12))
legend('\phi','\theta','\psi','p','q','r')
title('Rotational states')
axis tight

subplot(4,1,3)
plot(T,X(:,13:14))
legend('\omega_{up}','\omega_{lo}')
title('Rotor speeds')
axis tight

subplot(4,1,4)
plot(T,X(:,15:17))
legend('x','y','z')
title('Upper thrust vector direction')
axis tight

figure(2)

subplot(2,1,1)
plot(T,U(:,1:2))
legend('u_{mot,up}','u_{mot,lo}')
title('Motor Inputs')
axis tight

subplot(2,1,2)
plot(T,U(:,3:4))
legend('u_{serv1}','u_{serv2}')
title('Servo Inputs')
axis tight
