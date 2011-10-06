function [cont_inputs] = coax_control(state,F_des,yaw_T,param,cont_param)

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

% Desired yaw trajectory
psi_T    = yaw_T(1);    % reference orientation around z
psidot_T = yaw_T(2);    % referenze rotational speed around z

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
K_pq = cont_param.K_pq;

% Rotation Matrix
Rb2w = [cos(yaw)*cos(pitch) cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll) cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
        sin(yaw)*cos(pitch) sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll) sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
        -sin(pitch) cos(pitch)*sin(roll) cos(pitch)*cos(roll)];
Rw2b = Rb2w';

% Upper thrust vector direction
z_Tupz      = cos(l_up*acos(z_barz));
if (z_Tupz < 1)
    temp    = sqrt((1-z_Tupz^2)/(z_barx^2 + z_bary^2));
    z_Tup_p  = [z_barx*temp z_bary*temp z_Tupz]';
else
    z_Tup_p  = [0 0 1]';
end
zeta        = zeta_mup*Omega_up + zeta_bup;
RzT         = [cos(zeta) -sin(zeta) 0; sin(zeta) cos(zeta) 0; 0 0 1];
z_Tup       = RzT*z_Tup_p;

% add roll/pitch rate damping
e_pq = [q p 0]';
F_des(1:2) = F_des(1:2) - Rb2w(1:2,:)*K_pq*e_pq;

%%% Test new lateral control
rhs = (F_des(1:2) - k_Tup*Omega_up^2*Rb2w(1:2,:)*z_Tup)/(k_Tlo*Omega_lo^2);
z_Tloz = 1;
lambda = 0.5;
tol = 1e-8;
while 1
    z_Tloxy = Rb2w(1:2,1:2)\(rhs-z_Tloz*Rb2w(1:2,3));
    norm_z_Tlo = norm([z_Tloxy' z_Tloz]);
    
    if (abs(norm_z_Tlo - 1) < tol)
        break;
    elseif (norm_z_Tlo < 1)
        z_Tloz = z_Tloz + lambda*abs(norm_z_Tlo - 1);
    else
        z_Tloz = z_Tloz - lambda*abs(norm_z_Tlo - 1);
    end
end

% saturation
if (z_Tloz < cos(max_SPangle*l_lo))
    z_Tloz = cos(max_SPangle*l_lo);
    z_Tloxy = z_Tloxy/norm(z_Tloxy)*sqrt(1-z_Tloz^2);
end
z_Tlo = [z_Tloxy' z_Tloz]';

% F_des_xy = [F_des(1:2); 0];
% z_Tlo_x = 1/(k_Tlo*Omega_lo^2)*(Rw2b(1,:)*F_des_xy - k_Tup*Omega_up^2*z_Tup(1));
% z_Tlo_y = 1/(k_Tlo*Omega_lo^2)*(Rw2b(2,:)*F_des_xy - k_Tup*Omega_up^2*z_Tup(2));
% z_Tlo = [z_Tlo_x z_Tlo_y sqrt(1-z_Tlo_x^2-z_Tlo_y^2)]';

zeta        = zeta_mlo*Omega_lo + zeta_blo;
RzT         = [cos(zeta) -sin(zeta) 0; sin(zeta) cos(zeta) 0; 0 0 1];
z_Tlo_p     = RzT*z_Tlo;

z_SPz        = cos(1/l_lo*acos(z_Tlo_p(3)));
if (z_SPz < 1)
    temp     = sqrt((1-z_SPz^2)/(z_Tlo(1)^2 + z_Tlo(2)^2));
    z_SP     = [z_Tlo_p(1)*temp z_Tlo_p(2)*temp z_SPz]';
else
    z_SP     = [0 0 1]';
end
b_lo_des     = asin(z_SP(1));
a_lo_des     = asin(-z_SP(2)/cos(b_lo_des));
cont_inputs(3) = a_lo_des/(max_SPangle);
cont_inputs(4) = b_lo_des/(max_SPangle);

%%% Test new heave-yaw control
Kp_M = 1*Izz;
Kd_M = 2*Izz;

Fz_des = F_des(3) - m*g;
Mz_des = -Kp_M*(yaw-psi_T) - Kd_M*(r-psidot_T);

A = k_Tup/k_Mup*Mz_des*Rb2w(3,:)*z_Tup;
B = k_Tup/k_Mup*k_Mlo*Rb2w(3,:)*z_Tup + k_Tlo*Rb2w(3,:)*z_Tlo;

Omega_lo_des = sqrt((m*g + A + Fz_des)/B);
Omega_up_des = sqrt((k_Mlo*Omega_lo_des^2 - Mz_des)/k_Mup);
cont_inputs(1) = (Omega_up_des - rs_bup)/rs_mup;
cont_inputs(2) = (Omega_lo_des - rs_blo)/rs_mlo;

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

