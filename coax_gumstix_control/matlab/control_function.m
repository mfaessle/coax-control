function [Fx_des, Fy_des, Fz_des, Mz_des, e_i] = control_function(state, Rb2w, trajectory, e_i_prev, dt, param, contr_param)

%=================================
%%% Function inputs
%=================================

% States
x = state(1); % x position
y = state(2); % y position
z = state(3); % z position
xdot = state(4); % u velocity
ydot = state(5); % v velocity
zdot = state(6); % w velocity
roll = state(7); % roll angle
pitch = state(8); % pitch angle
yaw = state(9); % yaw angle
p = state(10); % body roll rate
q = state(11); % body pitch rate
r = state(12); % body yaw rate

% Desired trajectory
x_T = trajectory(1); % x position reference
y_T = trajectory(2); % y position reference
z_T = trajectory(3); % z position reference
xdot_T = trajectory(4); % x velocity reference
ydot_T = trajectory(5); % y velocity reference
zdot_T = trajectory(6); % z velocity reference
xddot_T = trajectory(7); % x acceleration reference
yddot_T = trajectory(8); % y acceleration reference
zddot_T = trajectory(9); % z acceleration reference
psi_T = trajectory(10); % reference orientation around z
psidot_T = trajectory(11); % referenze rotational speed around z

% Parameter
m = param.m;
g = param.g;
Ixx = param.Ixx;
Iyy = param.Iyy;
Izz = param.Izz;
d_up = param.d_up;
d_lo = param.d_lo;
k_springup = param.k_springup;
k_springlo = param.k_springlo;
l_up = param.l_up;
l_lo = param.l_lo;
k_Tup = param.k_Tup;
k_Tlo = param.k_Tlo;
k_Mup = param.k_Mup;
k_Mlo = param.k_Mlo;
Tf_motup = param.Tf_motup;
Tf_motlo = param.Tf_motlo;
Tf_up = param.Tf_up;
rs_mup = param.rs_mup;
rs_bup = param.rs_bup;
rs_mlo = param.rs_mlo;
rs_blo = param.rs_blo;
zeta_mup = param.zeta_mup;
zeta_bup = param.zeta_bup;
zeta_mlo = param.zeta_mlo;
zeta_blo = param.zeta_blo;
max_SPangle = param.max_SPangle;
Omega_max = param.Omega_max;

% Control parameters
K_p = contr_param.K_p;
K_v = contr_param.K_v;
K_i = contr_param.K_i;
K_psi = contr_param.K_psi;
K_psi_i = contr_param.K_psi_i;
K_omegaz = contr_param.K_omegaz;

%==================
%%% Desired Forces and Moments
%==================
Rw2b = Rb2w';

e_p = [x y z]' - [x_T y_T z_T]';
e_v = [xdot ydot zdot]' - [xdot_T ydot_T zdot_T]';
e_i(1:3,1) = e_i_prev(1:3,1) + e_p*dt; % anti windup?


F_des = -K_p*e_p - K_v*e_v - K_i*e_i(1:3,1) + m*[xddot_T yddot_T zddot_T]' + m*g*[0 0 1]';

ori_error = atan2(Rb2w(2,1),Rb2w(1,1)) - psi_T;
while (ori_error > pi)
    ori_error = ori_error - 2*pi;
end
while (ori_error < -pi)
    ori_error = ori_error + 2*pi;
end
e_i(4) = real(e_i_prev(4) + ori_error*dt);
Myaw_des = -K_psi*ori_error - K_psi_i*e_i(4) - K_omegaz*(r-Rw2b(3,3)*psidot_T);


%==================
%%% Function Outputs
%==================

Fx_des = F_des(1);
Fy_des = F_des(2);
Fz_des = F_des(3);
Mz_des = Myaw_des;

end