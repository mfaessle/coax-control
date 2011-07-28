%%
clear
clc

run parameter
run normalize_linearize

%%
stepsize = 1e-2;
tsim = 15;

% x = [x y z xdot ydot zdot roll pitch yaw p q r Omega_up Omega_lo a_up b_up]

Omega_lo0 = sqrt(m*g/(k_Tup*k_Mlo/k_Mup + k_Tlo));
Omega_up0 = sqrt(k_Mlo/k_Mup*Omega_lo0^2);

t0 = 0;
x0 = [0 0 0  0 0 0  0 0 0  0 0 0  Omega_up0 Omega_lo0  0 0 1]';

% Outputs
X = [];
T = [];
U = [];

FIRST_RUN = 1;
VEL = 0.05;
end_position = [0.5 0 0]';
end_orientation = 0;

t = t0;
x = x0;
while (t < tsim)
    
    tstart = t;
    tstop = tstart + stepsize;
    
    if (FIRST_RUN)
        start_position = x(1:3);
        dist = norm(end_position - start_position);
        dir = (end_position - start_position)/dist;
        duration = dist/VEL;
        FIRST_RUN = 0;
    end
    
    % Compute trajectory
    dt = t-t0;
    if (dt < duration)
        desPosition = start_position + VEL*dt*dir;
        trajectory = [desPosition' VEL*dir' 0 0 0 dt/duration*end_orientation end_orientation/duration]';
    else
        trajectory = [end_position' 0 0 0 0 0 0 end_orientation 0]';
    end
    
    % control = [u_motup u_motlo u_serv1 u_serv2]  u_mot in [0,1]; u_serv in [-1, 1]
    control = coax_control(x,trajectory,param,cont_param);
    
    [time,state] = ode45(@coax_eom,[tstart tstop],x,[],control,param);
    
    t = time(end);
    x = state(end,:)';
    
    X = [X; x'];
    T = [T; t];
    U = [U; control'];
end


%% Plot
figure(1)

subplot(4,1,1)
plot(T,X(:,1:6))
grid on;
legend('x','y','z','u','v','w')
title('Translational states')

subplot(4,1,2)
plot(T,X(:,7:12))
grid on;
legend('\phi','\theta','\psi','p','q','r')
title('Rotational states')

subplot(4,1,3)
plot(T,X(:,13:14))
grid on;
legend('\omega_{up}','\omega_{lo}')
title('Rotor speeds')

subplot(4,1,4)
plot(T,X(:,15:17))
grid on;
legend('x','y','z')
title('Upper thrust vector direction')

figure(2)

subplot(2,1,1)
plot(T,U(:,1:2))
grid on;
legend('u_{mot,up}','u_{mot,lo}')
title('Motor Inputs')

subplot(2,1,2)
plot(T,U(:,3:4))
grid on;
legend('u_{serv1}','u_{serv2}')
title('Servo Inputs')
