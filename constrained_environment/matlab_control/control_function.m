function [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(state, Rb2w, trajectory, e_i_prev, dt, param, contr_param)

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
Omega_up = state(13); % upper rotor speed
Omega_lo = state(14); % lower rotor speed
z_barx = state(15); % stabilizer bar z-axis x-component
z_bary = state(16); % stabilizer bar z-axis y-component
z_barz = state(17); % stabilizer bar z-axis z-component

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
K_pq = contr_param.K_pq;
K_psi = contr_param.K_psi;
K_psi_i = contr_param.K_psi_i;
K_omegaz = contr_param.K_omegaz;
K_lqr = contr_param.K_lqr;
T_inv = contr_param.T_inv_lqr;
W = contr_param.W_lqr;

%=================================
%%% Control inputs
%=================================

%==================
%%% High level part
%==================
Rw2b = Rb2w';

e_p = [x y z]' - [x_T y_T z_T]';
e_v = [xdot ydot zdot]' - [xdot_T ydot_T zdot_T]';
%e_i = e_i_prev + e_p*dt; % anti windup?
e_i(1:3,1) = e_i_prev(1:3,1) + e_p*dt;
e_pq = [q p 0]';

e_psi = atan2(Rb2w(2,1),Rb2w(1,1)) - psi_T;
while (e_psi > pi)
    e_psi = e_psi - 2*pi;
end
while (e_psi < -pi)
    e_psi = e_psi + 2*pi;
end
e_i(4) = real(e_i_prev(4) + e_psi*dt);
e_omegaz = r - [0 0 1]*Rw2b*[0 0 psidot_T]';

F_des = -K_p*e_p - K_v*e_v - K_i*e_i(1:3,1) - Rb2w*K_pq*e_pq + m*[xddot_T yddot_T zddot_T]';% + m*g*[0 0 1]';
F_des(3) = 0;
Mz_des = -K_psi*e_psi - K_psi_i*e_i(4) - K_omegaz*e_omegaz;

%==================
%%% Low level part
%==================

% z_Tupz = cos(l_up*acos(z_barz));
% if (z_Tupz < 1)
%     temp = sqrt((1-z_Tupz^2)/(z_barx^2 + z_bary^2));
%     z_Tup = [z_barx*temp z_bary*temp z_Tupz]';
% else
%     z_Tup = [0 0 1]';
% end
% zeta = zeta_mup*Omega_up + zeta_bup;
% RzT = [cos(zeta) -sin(zeta) 0; sin(zeta) cos(zeta) 0; 0 0 1];
% z_Tup = RzT*z_Tup;
% 
% stepsize = 2000; % initial stepsize
% tol = 1e-4; % tolerance to stop iterations
% % initial omega^2 value (case when z_Tlo and z_Tup would be parallel with
% % magnitude of F_des)
% Omegalo_sq = 0.95*(norm(F_des) + k_Tup/k_Mup*Mz_des)/(k_Tup*k_Mlo/k_Mup+k_Tlo);
% steps = 0;
% 
% temp1 = 1/k_Tlo*(Rw2b*F_des + Mz_des*z_Tup); % to improve performance
% temp2 = 1/k_Tlo*k_Tup*k_Mlo/k_Mup*z_Tup; % to improve performance
% while 1
%     % z_Tlo = 1/k_Tlo/Omegalo_sq*(Rw2b*F_des + Mz_des*z_Tup - k_Tup*k_Mlo/k_Mup*Omegalo_sq*z_Tup);
%     z_Tlo = temp1/Omegalo_sq - temp2;
% 
%     z_norm = norm(z_Tlo);
%     if (abs(z_norm - 1) < tol || Omegalo_sq > Omega_max^2)
%         break
%     elseif (z_norm < 1)
%         stepsize = stepsize/2;
%         Omegalo_sq = Omegalo_sq - stepsize;
%     else
%         Omegalo_sq = Omegalo_sq + stepsize;
%     end
%     steps = steps + 1;
% end
% z_Tlo = z_Tlo/norm(z_Tlo); % should be close to a unit vector already!
% 
% Omega_lo_des = sqrt(Omegalo_sq);
% Omega_up_des = sqrt(1/k_Mup*(k_Mlo*Omegalo_sq - Mz_des));
% 
% % rotation of thrust direction by zeta around z-axis (positive direction!!!)
% zeta = zeta_mlo*Omega_lo + zeta_blo;
% RzT = [cos(zeta) -sin(zeta) 0; sin(zeta) cos(zeta) 0; 0 0 1];
% z_Tlo = RzT*z_Tlo;
% 
% z_SPz = cos(1/l_lo*acos(z_Tlo(3)));
% %%% SP tilt Saturation
% if (z_SPz < cos(max_SPangle))
%     z_SPz = cos(max_SPangle);
% end
% 
% if (z_SPz < 1)
%     temp = sqrt((1-z_SPz^2)/(z_Tlo(1)^2 + z_Tlo(2)^2));
%     z_SP = [z_Tlo(1)*temp z_Tlo(2)*temp z_SPz]';
% else
%     z_SP = [0 0 1]';
% end
% 
% b_lo_des = asin(z_SP(1));
% a_lo_des = asin(-z_SP(2)/cos(b_lo_des));
% 
% % implement the conversion from physical to actual input values !!!
% motor_up = (Omega_up_des - rs_bup)/rs_mup;
% motor_lo = (Omega_lo_des - rs_blo)/rs_mlo;
% servo1 = a_lo_des/max_SPangle;
% servo2 = b_lo_des/max_SPangle;

%% LQR Control

% Upper thrust vector direction
z_Tupz = cos(l_up*acos(z_barz));
if (z_Tupz < 1)
    temp = sqrt((1-z_Tupz^2)/(z_barx^2 + z_bary^2));
    z_Tup = [z_barx*temp z_bary*temp z_Tupz]';
else
    z_Tup = [0 0 1]';
end
zeta = zeta_mup*Omega_up + zeta_bup;
RzT = [cos(zeta) -sin(zeta) 0; sin(zeta) cos(zeta) 0; 0 0 1];
z_Tup = RzT*z_Tup;

% Omega_lo0 = sqrt(m*g/(k_Tup*k_Mlo/k_Mup + k_Tlo));
% Omega_up0 = sqrt(k_Mlo/k_Mup*Omega_lo0^2);
% 
% a_up = -asin(z_Tup(2));
% b_up = asin(z_Tup(1)/cos(a_up));
% 
% error = [state(1:14); a_up; b_up] - [x_T y_T z_T xdot_T ydot_T zdot_T 0 0 psi_T 0 0 psidot_T Omega_up0 Omega_lo0 0 0]';
% error(1:3) = Rw2b*error(1:3);
% error(4:6) = Rw2b*error(4:6);
% if (error(9) > pi)
%     error(9) = error(9) - 2*pi;
% elseif (error(9) < -pi)
%     error(9) = error(9) + 2*pi;
% end
% inputs = -W*K_lqr*T_inv*error;
% 
% % feed forward on rotor speeds
% motor_up = inputs(1) + (Omega_up0 - rs_bup)/rs_mup;
% motor_lo = inputs(2) + (Omega_lo0 - rs_blo)/rs_mlo;
% % correct for phase lag of servo inputs
% a_SP = inputs(3)*max_SPangle;
% b_SP = inputs(4)*max_SPangle;
% z_SP = [sin(b_SP) -sin(a_SP)*cos(b_SP) cos(a_SP)*cos(b_SP)]';
% z_Tloz = cos(l_lo*acos(z_SP(3)));
% if (z_Tloz < 1)
%     temp = sqrt((1-z_Tloz^2)/(z_SP(1)^2 + z_SP(2)^2));
%     z_Tlo = [z_SP(1)*temp z_SP(2)*temp z_Tloz]';
% else
%     z_Tlo = [0 0 1]';
% end
% zeta = zeta_mlo*Omega_lo + zeta_blo;
% RzT = [cos(zeta) -sin(zeta) 0; sin(zeta) cos(zeta) 0; 0 0 1];
% z_Tlo = RzT*z_Tlo;
% 
% z_SPz = cos(1/l_lo*acos(z_Tlo(3)));
% if (z_SPz < 1)
%     temp = sqrt((1-z_SPz^2)/(z_Tlo(1)^2 + z_Tlo(2)^2));
%     z_SP = [z_Tlo(1)*temp z_Tlo(2)*temp z_SPz]';
% else
%     z_SP = [0 0 1]';
% end
% b_lo_des = asin(z_SP(1));
% a_lo_des = asin(-z_SP(2)/cos(b_lo_des));
% servo1 = a_lo_des/max_SPangle;
% servo2 = b_lo_des/max_SPangle;

%%% Test new lateral control

% rhs = (F_des(1:2) - k_Tup*Omega_up^2*Rb2w(1:2,:)*z_Tup)/(k_Tlo*Omega_lo^2);
% z_Tloz = 1;
% tol = 1e-8;
% while 1
%     z_Tloxy = Rb2w(1:2,1:2)\(rhs-z_Tloz*Rb2w(1:2,3));
%     norm_z_Tlo = norm([z_Tloxy' z_Tloz]);
%     
%     if (abs(norm_z_Tlo - 1) < tol)
%         break;
%     elseif (norm_z_Tlo < 1)
%         z_Tloz = z_Tloz + abs(norm_z_Tlo - 1);
%     else
%         z_Tloz = z_Tloz - abs(norm_z_Tlo - 1);
%     end
% end
% 
% % saturation
% if (z_Tloz < cos(max_SPangle*l_lo))
%     z_Tloz = cos(max_SPangle*l_lo);
%     z_Tloxy = z_Tloxy/norm(z_Tloxy)*sqrt(1-z_Tloz^2);
% end
% z_Tlo = [z_Tloxy' z_Tloz]';

z_Tlo_x = 1/(k_Tlo*Omega_lo^2)*(Rw2b(1,:)*F_des);% - k_Tup*Omega_up^2*z_Tup(1));
z_Tlo_y = 1/(k_Tlo*Omega_lo^2)*(Rw2b(2,:)*F_des);% - k_Tup*Omega_up^2*z_Tup(2));
z_Tlo = [z_Tlo_x z_Tlo_y sqrt(1-z_Tlo_x^2-z_Tlo_y^2)]';

zeta = zeta_mlo*Omega_lo + zeta_blo;
RzT = [cos(zeta) -sin(zeta) 0; sin(zeta) cos(zeta) 0; 0 0 1];
z_Tlo_p = RzT*z_Tlo;

z_SPz = cos(1/l_lo*acos(z_Tlo_p(3)));
if (z_SPz < 1)
    temp = sqrt((1-z_SPz^2)/(z_Tlo(1)^2 + z_Tlo(2)^2));
    z_SP = [z_Tlo_p(1)*temp z_Tlo_p(2)*temp z_SPz]';
else
    z_SP = [0 0 1]';
end
b_lo_des = asin(z_SP(1));
a_lo_des = asin(-z_SP(2)/cos(b_lo_des));
servo1 = a_lo_des/(max_SPangle);
servo2 = b_lo_des/(max_SPangle);

%%% Test new heave-yaw control
Kp_F = 20*m;
Kd_F = 10*m;
Kp_M = 50*Izz;
Kd_M = 6*Izz;

Fz_des = -Kp_F*(z-z_T) - Kd_F*(zdot-zdot_T) + m*zddot_T + m*g;
ori_error = yaw-psi_T;
while (ori_error > pi)
    ori_error = ori_error - 2*pi;
end
while (ori_error < -pi)
    ori_error = ori_error + 2*pi;
end
Mz_des = -Kp_M*ori_error - Kd_M*(r-psidot_T);

A = k_Tup/k_Mup*Mz_des*Rb2w(3,:)*z_Tup;
B = k_Tup/k_Mup*k_Mlo*Rb2w(3,:)*z_Tup + k_Tlo*Rb2w(3,:)*z_Tlo;

Omega_lo_des = sqrt((A + Fz_des)/B);
Omega_up_des = sqrt((k_Mlo*Omega_lo_des^2 - Mz_des)/k_Mup);
motor_up = (Omega_up_des - rs_bup)/rs_mup;
motor_lo = (Omega_lo_des - rs_blo)/rs_mlo;

trim_values = zeros(4,1);

end