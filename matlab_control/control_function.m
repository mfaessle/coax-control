function [motor_up, motor_lo, servo1, servo2, e_i] = control_function(state, Rb2w, trajectory, e_i_prev, dt, cont_const, contr_param)

%=================================
%%% Function inputs
%=================================

% States
x        = state(1);           % x position
y        = state(2);           % y position
z        = state(3);           % z position
u        = state(4);           % u velocity
v        = state(5);           % v velocity
w        = state(6);           % w velocity
p        = state(7);           % body roll rate
q        = state(8);           % body pitch rate
r        = state(9);           % body yaw rate 
Omega_up = state(10);          % upper rotor speed
Omega_lo = state(11);          % lower rotor speed
z_barx   = state(12);          % stabilizer bar z-axis x-component
z_bary   = state(13);          % stabilizer bar z-axis y-component
z_barz   = state(14);          % stabilizer bar z-axis z-component
roll = state(15);
pitch = state(16);
yaw = state(17);

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
psi_T    = trajectory(10);     % reference orientation around z
psidot_T = trajectory(11);     % referenze rotational speed around z 

% Constants
g           = cont_const.g;
m           = cont_const.m;
k_Tup       = cont_const.k_Tup;
k_Tlo       = cont_const.k_Tlo;
k_Mup       = cont_const.k_Mup;
k_Mlo       = cont_const.k_Mlo;
l_up        = cont_const.l_up;
l_lo        = cont_const.l_lo;
Omega_max   = cont_const.Omega_max;
max_SPangle = cont_const.max_SPangle;
zeta_mup    = cont_const.zeta_mup;
zeta_bup    = cont_const.zeta_bup;
zeta_mlo    = cont_const.zeta_mlo;
zeta_blo    = cont_const.zeta_blo;

% Control parameters
K_p = contr_param.K_p;
K_v = contr_param.K_v;
K_i = contr_param.K_i;
K_pq = contr_param.K_pq;
K_psi = contr_param.K_psi;
K_omegaz = contr_param.K_omegaz;

%=================================
%%% Control inputs
%=================================

%==================
%%% High level part
%==================
Rw2b         = Rb2w';

e_p          = [x y z]' - [x_T y_T z_T]';
e_v          = Rb2w*[u v w]' - [xdot_T ydot_T zdot_T]';
e_i          = e_i_prev + e_p*dt; % anti windup?
e_pq         = [q p 0]';

z_w          = [0 0 1]';
x_c          = [cos(psi_T) sin(psi_T) 0]';
x_b          = Rb2w(:,1);
y_b          = Rb2w(:,2);
x_bproj      = (x_b - (x_b'*z_w)*z_w)/norm(x_b - (x_b'*z_w)*z_w);
y_bproj      = (y_b - (y_b'*z_w)*z_w)/norm(y_b - (y_b'*z_w)*z_w);

e_psi        = -sign(y_bproj'*x_c)*acos(x_bproj'*x_c);
e_omegaz     = r - [0 0 1]*Rw2b*[0 0 psidot_T]';

F_des        = -K_p*e_p - K_v*e_v - K_i*e_i - Rb2w*K_pq*e_pq + m*g*[0 0 1]' + m*[xddot_T yddot_T zddot_T]';
Mz_des       = -K_psi*e_psi - K_omegaz*e_omegaz;

%==================
%%% Low level part
%==================

z_Tupz      = cos(l_up*acos(z_barz));
if (z_Tupz < 1)
    temp    = sqrt((1-z_Tupz^2)/(z_barx^2 + z_bary^2));
    z_Tup   = [z_barx*temp z_bary*temp z_Tupz]';
else
    z_Tup   = [0 0 1]';
end

% rotation of thrust direction by zeta around z-axis
% could be part of the observer which delivers z_Tup directly to controller
zeta        = zeta_mup*Omega_up + zeta_bup;
RzT         = [cos(zeta) -sin(zeta) 0; sin(zeta) cos(zeta) 0; 0 0 1];
z_Tup       = RzT*z_Tup;

stepsize     = 2000; % initial stepsize
tol          = 1e-4; % tolerance to stop iterations
% initial omega^2 value (case when z_Tlo and z_Tup would be parallel with
% magnitude of F_des)
Omegalo_sq   = 0.95*(norm(F_des) + k_Tup/k_Mup*Mz_des)/(k_Tup*k_Mlo/k_Mup+k_Tlo);
steps        = 0;

temp1        = 1/k_Tlo*(Rw2b*F_des + Mz_des*z_Tup);  % to improve performance
temp2        = 1/k_Tlo*k_Tup*k_Mlo/k_Mup*z_Tup;      % to improve performance
while 1
    % z_Tlo = 1/k_Tlo/Omegalo_sq*(Rw2b*F_des + Mz_des*z_Tup - k_Tup*k_Mlo/k_Mup*Omegalo_sq*z_Tup);
    z_Tlo = temp1/Omegalo_sq - temp2;

    z_norm = norm(z_Tlo);
    if (abs(z_norm - 1) < tol || Omegalo_sq > Omega_max^2)
        break
    elseif (z_norm < 1)
        stepsize = stepsize/2;
        Omegalo_sq = Omegalo_sq - stepsize;
    else
        Omegalo_sq = Omegalo_sq + stepsize;
    end
    steps = steps + 1;
end
z_Tlo        = z_Tlo/norm(z_Tlo); % should be close to a unit vector already!

Omega_lo_des = sqrt(Omegalo_sq);
Omega_up_des = sqrt(1/k_Mup*(k_Mlo*Omegalo_sq - Mz_des));

% rotation of thrust direction by zeta around z-axis (positive direction!!!)
zeta         = zeta_mlo*Omega_lo + zeta_blo;
RzT          = [cos(zeta) -sin(zeta) 0; sin(zeta) cos(zeta) 0; 0 0 1];
z_Tlo        = RzT*z_Tlo;   

z_SPz        = cos(1/l_lo*acos(z_Tlo(3)));
%%% SP tilt Saturation
if (z_SPz < cos(max_SPangle))
    z_SPz    = cos(max_SPangle);
end

if (z_SPz < 1)
    temp     = sqrt((1-z_SPz^2)/(z_Tlo(1)^2 + z_Tlo(2)^2));
    z_SP     = [z_Tlo(1)*temp z_Tlo(2)*temp z_SPz]';
else
    z_SP     = [0 0 1]';
end

b_lo_des     = asin(z_SP(1));
a_lo_des     = asin(-z_SP(2)/cos(b_lo_des));

% implement the conversion from physical to actual input values !!!
% motor_up     = (Omega_up_des + 21.5)/430;
% motor_lo     = (Omega_lo_des + 21.5)/430;
% servo1       = a_lo_des/0.26;%a_lo_des/0.26;
% servo2       = b_lo_des/0.26;%b_lo_des/0.26;

%% Test PID
c            = 0.3;
a_lo_des     = -c*Rw2b(2,:)*[F_des(1) F_des(2) 0];
b_lo_des     = c*Rw2b(1,:)*[F_des(1) F_des(2) 0];
omega_lo_sq  = (Rw2b(3,:)*F_des + k_Tup/k_Mup*Mz_des)/(k_Tup/k_Mup*k_Mlo + k_Tlo);
Omega_lo_des = sqrt(omega_lo_sq);
Omega_up_des = sqrt(1/k_Mup*(k_Mlo*omega_lo_sq - Mz_des));

motor_up     = (Omega_up_des + 21.5)/430;
motor_lo     = (Omega_lo_des + 21.5)/430;
servo1       = a_lo_des/0.26;%a_lo_des/0.26;
servo2       = b_lo_des/0.26;%b_lo_des/0.26;

%% Test LQR controller

% load K_new 
% load T_inv 
% load W
% state = T_inv(1:14,1:14)*[x-x_T y-y_T z-z_T u-xdot_T v-ydot_T w-zdot_T roll pitch yaw-psi_T p q r-psidot_T Omega_up Omega_lo]';
% out = -W*K_new*state;
% motor_up = (out(1) + sqrt(m*g/(2*k_Tup)) + 21.5)/430;
% motor_lo = (out(2) + sqrt(m*g/(2*k_Tlo)) + 21.5)/430;
% servo1 = out(3)/0.1;
% servo2 = out(4)/0.1;


end

