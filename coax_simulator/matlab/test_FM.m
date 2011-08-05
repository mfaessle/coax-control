run parameter

qx = 0.0274045334483;
qy = -0.04734162125;
qz = 0.00334408780377;
qw = 0.998497160497;
roll = atan2(2*(qw*qx+qy*qz),1-2*(qx^2+qy^2));
pitch = asin(2*(qw*qy-qz*qx));
yaw = atan2(2*(qw*qz+qx*qy),1-2*(qy^2+qz^2));

x = 0.0605228325054;
y = 0.0315342829308;
z = 0.423585729345;

u = 0.0689274916026;
v = 0.0603583174183;
w = 9.40186901227e-05;

p = 0.323559237606;
q = 0.0530269700116;
r = -0.00133176165819;

Omega_up = 227.183188;
Omega_lo = 239.472329;
z_bar = [-0.043390   -0.753383   0.656149];

trajectory = [end_position' 0 0 0 0 0 0 end_orientation 0]';
control_inputs = coax_control(state,trajectory,param,cont_param);

Omega_lo0 = sqrt(m*g/(k_Tup*k_Mlo/k_Mup + k_Tlo));
Omega_up0 = sqrt(k_Mlo/k_Mup*Omega_lo0^2);
state = [0 0 0 0 0 0 0 0 0 0 0 0 Omega_up0 Omega_lo0 0.1 -0.05 0.99373034571759];
% state = [x y z u v w roll pitch yaw p q r Omega_up Omega_lo z_bar]';

control = [(Omega_up0 - rs_bup)/rs_mup (Omega_lo0 - rs_blo)/rs_mlo -0.2 0.3]';


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
z_barx   = state(15);     % stabilizer bar z-axis x-component
z_bary   = state(16);     % stabilizer bar z-axis y-component
z_barz   = state(17);     % stabilizer bar z-axis z-component

% Controls
u_motup = control(1);
u_motlo = control(2);
u_serv1 = control(3);
u_serv2 = control(4);

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

% Upper thrust vector direction
z_Tupz      = cos(l_up*acos(z_barz));
if (z_Tupz < 1)
    temp    = sqrt((1-z_Tupz^2)/(z_barx^2 + z_bary^2));
    z_Tup   = [z_barx*temp z_bary*temp z_Tupz]';
else
    z_Tup   = [0 0 1]';
end
zeta        = zeta_mup*Omega_up + zeta_bup;
RzT         = [cos(zeta) -sin(zeta) 0; sin(zeta) cos(zeta) 0; 0 0 1];
z_Tup       = RzT*z_Tup;

% Lower thrust vector direction
a_SP = u_serv1*max_SPangle;
b_SP = u_serv2*max_SPangle;
z_SP        = [sin(b_SP) -sin(a_SP)*cos(b_SP) cos(a_SP)*cos(b_SP)]';
z_Tloz      = cos(l_lo*acos(z_SP(3)));
if (z_Tloz < 1)
    temp    = sqrt((1-z_Tloz^2)/(z_SP(1)^2 + z_SP(2)^2));
    z_Tlo   = [z_SP(1)*temp z_SP(2)*temp z_Tloz]';
else
    z_Tlo   = [0 0 1]';
end
zeta        = zeta_mlo*Omega_lo + zeta_blo;
RzT         = [cos(zeta) sin(zeta) 0; -sin(zeta) cos(zeta) 0; 0 0 1];
z_Tlo       = RzT*z_Tlo;
    
% Coordinate transformation body to world coordinates
c_r = cos(roll);
s_r = sin(roll);
c_p = cos(pitch);
s_p = sin(pitch);
c_y = cos(yaw);
s_y = sin(yaw);

Rb2w = [c_p*c_y (s_r*s_p*c_y-c_r*s_y) (c_r*s_p*c_y+s_r*s_y); ...
        c_p*s_y (s_r*s_p*s_y+c_r*c_y) (c_r*s_p*s_y-s_r*c_y); ...
        -s_p s_r*c_p c_r*c_p];

% Flapping Moments
cp = [-z_Tup(2) z_Tup(1) 0]'; % z_b x z_Tup
norm_cp = sqrt(cp(1)*cp(1) + cp(2)*cp(2) + cp(3)*cp(3));
if (norm_cp ~= 0)
    M_flapup  = 2*k_springup*cp/norm_cp*acos(z_Tup(3));
else
    M_flapup  = [0 0 0]';
end

cp   = [-z_Tlo(2) z_Tlo(1) 0]'; % z_b x z_Tlo
norm_cp = sqrt(cp(1)*cp(1) + cp(2)*cp(2) + cp(3)*cp(3));
if (norm_cp ~= 0)
    M_flaplo  = 2*k_springlo*cp/norm_cp*acos(z_Tlo(3));
else
    M_flaplo  = [0 0 0]';
end

% Thrust magnitudes
T_up = k_Tup*Omega_up*Omega_up;
T_lo = k_Tlo*Omega_lo*Omega_lo;

% Summarized Forces
F_thrust = T_up*z_Tup + T_lo*z_Tlo;
Fx = Rb2w(1,:)*F_thrust;
Fy = Rb2w(2,:)*F_thrust;
Fz = -m*g + Rb2w(3,:)*F_thrust;

% Summarized Moments
Mx = q*r*(Iyy-Izz) - T_up*z_Tup(2)*d_up - T_lo*z_Tlo(2)*d_lo + M_flapup(1) + M_flaplo(1);
My = p*r*(Izz-Ixx) + T_up*z_Tup(1)*d_up + T_lo*z_Tlo(1)*d_lo + M_flapup(2) + M_flaplo(2);
Mz = p*q*(Ixx-Iyy) - k_Mup*Omega_up*Omega_up + k_Mlo*Omega_lo*Omega_lo;

% State derivatives
xddot = 1/m*Fx;
yddot = 1/m*Fy;
zddot = 1/m*Fz;

rolldot = p + q*s_r*s_p/c_p + r*c_r*s_p/c_p;
pitchdot = q*c_r - r*s_r;
yawdot = q*s_r/c_p + r*c_r/c_p;

pdot = 1/Ixx*Mx;
qdot = 1/Iyy*My;
rdot = 1/Izz*Mz;

Omega_up_des = rs_mup*u_motup + rs_bup;
Omega_lo_des = rs_mlo*u_motlo + rs_blo;
Omega_updot = 1/Tf_motup*(Omega_up_des - Omega_up);
Omega_lodot = 1/Tf_motlo*(Omega_lo_des - Omega_lo);

b_z_bardotz = 1/Tf_up*acos(z_barz)*sqrt(z_barx^2 + z_bary^2);

[Tf_up z_barx z_bary z_barz acos(z_barz) sqrt(z_barx^2 + z_bary^2)]'

if (b_z_bardotz == 0)
    b_z_bardot = [0 0 0]';
else
    temp = z_barz*b_z_bardotz/(z_barx^2+z_bary^2);
    b_z_bardot = [-z_barx*temp -z_bary*temp b_z_bardotz]';
end

z_barxdot   = b_z_bardot(1) - q*z_barz + r*z_bary;
z_barydot   = b_z_bardot(2) - r*z_barx + p*z_barz;
z_barzdot   = b_z_bardot(3) - p*z_bary + q*z_barx;

% Function outputs
xdot = zeros(17,1);

xdot(1)  = state(4);
xdot(2)  = state(5);
xdot(3)  = state(6);
xdot(4)  = xddot;
xdot(5)  = yddot;
xdot(6)  = zddot;
xdot(7)  = rolldot;
xdot(8)  = pitchdot;
xdot(9)  = yawdot;
xdot(10) = pdot;
xdot(11) = qdot;
xdot(12) = rdot;
xdot(13) = Omega_updot;
xdot(14) = Omega_lodot;
xdot(15) = z_barxdot;
xdot(16) = z_barydot;
xdot(17) = z_barzdot;

[Fx Fy Fz]
[Mx My Mz]

xdot

b_z_bardot