function [F_des] = mpc_control(state, time_now, N, mpc_mat, Ts, m, traj_param, t_switch, next_traj_param)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

n_poly = size(traj_param,2) - 1; % order of the trajectory polynomial
n = 6; % number of states
p = 3; % number of inputs

F = mpc_mat.F;
H = mpc_mat.H;
A_ineq = mpc_mat.A_ineq;
b_ineq = mpc_mat.b_ineq;
Q_tilde = mpc_mat.Q_tilde;
B_tilde = mpc_mat.B_tilde;
R_tilde = mpc_mat.R_tilde;
g = 9.81;

t_Xss = linspace(Ts,N*Ts,N) + time_now;
t_Uss = linspace(0,(N-1)*Ts,N) + time_now;
x_T = zeros(3,N);
v_T = zeros(3,N);
a_T = zeros(3,N);

for j = 1:n_poly+1
    x_T = x_T + (traj_param(:,j)*t_Xss.^(n_poly+1-j));
end
for j = 1:n_poly
    v_T = v_T + ((n_poly+1-j)*traj_param(:,j)*(t_Xss.^(n_poly-j).*ones(1,N)));
end
for j = 1:n_poly-1
    a_T = a_T + ((n_poly+1-j)*(n_poly-j)*traj_param(:,j)*(t_Uss.^(n_poly-1-j).*ones(1,N)));
end
if (t_Xss(N) > t_switch)
    ns = round((t_switch - time_now)/Ts);
    x_T(:,ns+1:N) = zeros(3,N-ns);
    v_T(:,ns+1:N) = zeros(3,N-ns);
    for j = 1:n_poly+1 % get polynomial info for prediction horizon
        x_T(:,ns+1:N) = x_T(:,ns+1:N) + (next_traj_param(:,j)*t_Xss(ns+1:end).^(n_poly+1-j));
        v_T(:,ns+1:N) = v_T(:,ns+1:N) + ((n_poly+1-j)*next_traj_param(:,j)*(t_Xss(ns+1:end).^(n_poly-j).*ones(1,N-ns)));
    end
end
if (t_Uss(N) > t_switch)
    ns = round((t_switch - time_now)/Ts) + 1;
    for j = 1:n_poly-1
        a_T(:,ns+1:N) = a_T(:,ns+1:N) + ((n_poly+1-j)*(n_poly-j)*next_traj_param(:,j)*(t_Uss(ns+1:end).^(n_poly-1-j).*ones(1,N-ns)));
    end
end

Xss = zeros(n*N,1);
Xss(1:n:n*N) = x_T(1,:);
Xss(2:n:n*N) = x_T(2,:);
Xss(3:n:n*N) = x_T(3,:);
Xss(4:n:n*N) = v_T(1,:);
Xss(5:n:n*N) = v_T(2,:);
Xss(6:n:n*N) = v_T(3,:);
Uss = zeros(p*N,1);
Uss(1:p:p*N) = m*a_T(1,:);
Uss(2:p:p*N) = m*a_T(2,:);
Uss(3:p:p*N) = m*a_T(3,:);
Fss = -(Xss'*Q_tilde*B_tilde + Uss'*R_tilde);

F_des = quadprog(H,F'*state+Fss',A_ineq,b_ineq,[],[],[],[],[],optimset('LargeScale','off','Display','off'));
F_des = F_des(1:p) + [0 0 m*g]';

end

