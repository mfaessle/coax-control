function [cont_inputs] = coax_control(state,trajectory,param,cont_param)

% States
x        = state(1);      % x position
y        = state(2);      % y position
z        = state(3);      % z position
u        = state(4);      % u velocity
v        = state(5);      % v velocity
w        = state(6);      % w velocity
roll     = state(7);      % roll angle
pitch    = state(8);      % pitch angle
yaw      = state(9);      % yaw angle
p        = state(10);     % body roll rate
q        = state(11);     % body pitch rate
r        = state(12);     % body yaw rate 
Omega_up = state(13);     % upper rotor speed
Omega_lo = state(14);     % lower rotor speed
a_up     = state(15);     % stabilizer bar z-axis x-component
b_up     = state(16);     % stabilizer bar z-axis y-component

% Desired trajectory
x_T      = trajectory(1);     % x position reference
y_T      = trajectory(2);     % y position reference
z_T      = trajectory(3);     % z position reference
xdot_T   = trajectory(4);     % x velocity reference
ydot_T   = trajectory(5);     % y velocity reference
zdot_T   = trajectory(6);     % z velocity reference
xddot_T  = trajectory(7);     % x acceleration reference
yddot_T  = trajectory(8);     % y acceleration reference
zddot_T  = trajectory(9);     % z acceleration reference
psi_T    = trajectory(10);    % reference orientation around z
psidot_T = trajectory(11);    % referenze rotational speed around z

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

% Control Parameter
K_lqr = cont_param.K_lqr;
T_inv = cont_param.T_inv_lqr; 
W = cont_param.W_lqr; 

%================
%%% LQR
%================

Omega_lo0 = sqrt(m*g/(k_Tup*k_Mlo/k_Mup + k_Tlo));
Omega_up0 = sqrt(k_Mlo/k_Mup*Omega_lo0^2);

error = T_inv*(state - [x_T y_T z_T xdot_T ydot_T zdot_T 0 0 psi_T 0 0 psidot_T Omega_up0 Omega_lo0 0 0]');
% error = T_inv*(state - [x_T y_T z_T xdot_T ydot_T zdot_T 0 0 psi_T 0 0 psidot_T 0 0 0 0]');
inputs = -W*K_lqr*error;

Omega_lo0 = sqrt(m*g/(k_Tup*k_Mlo/k_Mup + k_Tlo));
Omega_up0 = sqrt(k_Mlo/k_Mup*Omega_lo0^2);

cont_inputs = zeros(4,1);
cont_inputs(1) = inputs(1) + (Omega_up0 - rs_bup)/rs_mup;
cont_inputs(2) = inputs(2) + (Omega_lo0 - rs_blo)/rs_mlo;
cont_inputs(3) = inputs(3);
cont_inputs(4) = inputs(4);

% Saturation
if (cont_inputs(1) < 0)
    cont_inputs(1) = 0;
elseif (cont_inputs(1) > 1)
    cont_inputs(1) = 1;
end
if (cont_inputs(2) < 0)
    cont_inputs(2) = 0;
elseif (cont_inputs(2) > 1)
    cont_inputs(2) = 1;
end
if (cont_inputs(3) < -1)
    cont_inputs(3) = -1;
elseif (cont_inputs(3) > 1)
    cont_inputs(3) = 1;
end
if (cont_inputs(4) < -1)
    cont_inputs(4) = -1;
elseif (cont_inputs(4) > 1)
    cont_inputs(4) = 1;
end

end

