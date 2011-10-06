%%
clear
clc

run parameter


%%
stepsize = 1e-2;
Ts_highlevel = 10*stepsize;
tsim = 5;

run create_polynomial

n = 6;
p = 3;
N = 10; % prediction horizon
run mpc_matrices

N_sim = tsim/stepsize;
T = linspace(0,tsim,N_sim+1)';

x_traj = a*(T'.^5) + b*(T'.^4) + c*(T'.^3);
v_traj = 5*a*(T'.^4) + 4*b*(T'.^3) + 3*c*(T'.^2);
a_traj = 20*a*(T'.^3) + 12*b*(T'.^2) + 6*c*(T');

% x = [x y z xdot ydot zdot roll pitch yaw p q r Omega_up Omega_lo a_up b_up]
Omega_lo0 = sqrt(m*g/(k_Tup*k_Mlo/k_Mup + k_Tlo));
Omega_up0 = sqrt(k_Mlo/k_Mup*Omega_lo0^2);
x0 = [0 0 0  0 0 0  0 0 0  0 0 0  Omega_up0 Omega_lo0  0 0 1]';
t0 = T(1);

% Outputs
X = zeros(N_sim+1,length(x0));
U = zeros(N_sim+1,4);
FDES_M = zeros(N_sim+1,3);
FDES_MPC = zeros(N_sim+1,3);

X(1,:) = x0';
while (X(1,9) > pi)
    X(1,9) = X(1,9) - 2*pi;
end
while (X(1,9) < -pi)
    X(1,9) = X(1,9) + 2*pi;
end

x = x0;
tic
for i = 1:N_sim
    
    tstart = T(i);
    tstop = T(i+1);
    
    if ((mod(i,Ts_highlevel/stepsize) == 1) || (Ts_highlevel/stepsize == 1))
        t = linspace(Ts_highlevel,N*Ts_highlevel,N) + T(i);
        x_T = zeros(length(t),3);
        v_T = zeros(length(t),3);
        a_T = zeros(length(t),3);
        for j = 1:3
            x_T = x_T + (traj_param(:,j)*t.^(6-j))';
            v_T = v_T + ((6-j)*traj_param(:,j)*t.^(5-j))';
            a_T = a_T + ((6-j)*(5-j)*traj_param(:,j)*t.^(4-j))';
        end
        
        % take care of the end of a polynomial!!!
        Xss = zeros(n*N,1);
        Xss(1:n:n*N) = x_T(:,1);
        Xss(2:n:n*N) = x_T(:,2);
        Xss(3:n:n*N) = x_T(:,3);
        Xss(4:n:n*N) = v_T(:,1);
        Xss(5:n:n*N) = v_T(:,2);
        Xss(6:n:n*N) = v_T(:,3);
        Uss = zeros(p*N,1);
        Uss(1:p:p*N) = m*a_T(:,1);
        Uss(2:p:p*N) = m*a_T(:,2);
        Uss(3:p:p*N) = m*a_T(:,3);
        Fss = -(Xss'*Q_tilde*B_tilde + Uss'*R_tilde);
        
        F_des = quadprog(H,F'*X(i,1:6)'+Fss',A_ineq,b_ineq,[],[],[],[],[],optimset('LargeScale','off','Display','off'));
        F_des = F_des(1:p) + [0 0 m*g]';
    end
    
    % desired yaw
    yaw_T = zeros(2,1);
    %%% make front look in direction of velocity
    
    % control = [u_motup u_motlo u_serv1 u_serv2]  u_mot in [0,1]; u_serv in [-1, 1]
    control = coax_control(x,F_des,yaw_T,param,cont_param);
    % control(3:4) = zeros(2,1);
%     trajectory = [x_traj(:,i)' v_traj(:,i)' a_traj(:,i)' 0 0]';
%     [control, F_des_mod] = coax_control_old(x,trajectory,param,cont_param);
    
    % normalize yaw
    while (x(9) > pi)
        x(9) = x(9) - 2*pi;
    end
    while (x(9) < -pi)
        x(9) = x(9) + 2*pi;
    end
    
    [time,state] = ode45(@CoaX_grey_box,[tstart tstop],x,[],control,m,g,Ixx,Iyy,Izz,d_up,d_lo,k_springup,k_springlo,l_up,l_lo,k_Tup,k_Tlo,k_Mup,k_Mlo,Tf_motup,Tf_motlo,Tf_up,rs_mup,rs_bup,rs_mlo,rs_blo,zeta_mup,zeta_bup,zeta_mlo,zeta_blo,max_SPangle);
    
    x = state(end,:)';
    
    X(i+1,:) = x';
    U(i,:) = control';
%     FDES_M(i,:) = F_des_mod';
%     FDES_MPC(i,:) = F_des';
end
toc

% FDES_M(end,:) = [0 0 m*g]';
% FDES_MPC(end,:) = [0 0 m*g]';

%% Plot
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

figure(3)
plot3(X(:,1),X(:,2),X(:,3))
hold on;
plot3(x_traj(1,:),x_traj(2,:),x_traj(3,:),'r')
hold off;
grid on;
xlabel('x')
ylabel('y')
zlabel('z')

figure(4)

subplot(3,1,1)
plot(T,X(:,1))
hold on;
plot(T,x_traj(1,:),'red')
hold off;
legend('x','x-traj')

subplot(3,1,2)
plot(T,X(:,2))
hold on;
plot(T,x_traj(2,:),'red')
hold off;
legend('y','y-traj')

subplot(3,1,3)
plot(T,X(:,3))
hold on;
plot(T,x_traj(3,:),'red')
hold off;
legend('z','z-traj')

