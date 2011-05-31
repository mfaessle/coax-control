function [out] = controller(in)
% Calculating the control inputs
% Inputs: Current state, Desired trajectory
% Outputs: Control inputs

%=================================
%%% Load global constants
%=================================

global g m k_Tup k_Tlo k_Mup k_Mlo l_lo l_up
global K_p K_v K_i K_pq K_psi K_omegaz e_i_prev Ts_lowlevel
global Omega_max max_SPangle
global zeta_mup zeta_bup zeta_mlo zeta_blo
global CONTROL

%=================================
%%% Function inputs
%=================================

% States
x        = in(1);      % x position
y        = in(2);      % y position
z        = in(3);      % z position
u        = in(4);      % u velocity
v        = in(5);      % v velocity
w        = in(6);      % w velocity
roll     = in(7);      % roll angle
pitch    = in(8);      % pitch angle
yaw      = in(9);      % yaw angle
p        = in(10);     % body roll rate
q        = in(11);     % body pitch rate
r        = in(12);     % body yaw rate 
Omega_up = in(13);     % upper rotor speed
Omega_lo = in(14);     % lower rotor speed
z_barx   = in(15);     % stabilizer bar z-axis x-component
z_bary   = in(16);     % stabilizer bar z-axis y-component
z_barz   = in(17);     % stabilizer bar z-axis z-component

% Desired trajectory
x_T      = in(18);     % x position reference
y_T      = in(19);     % y position reference
z_T      = in(20);     % z position reference
xdot_T   = in(21);     % x velocity reference
ydot_T   = in(22);     % y velocity reference
zdot_T   = in(23);     % z velocity reference
xddot_T  = in(24);     % x acceleration reference
yddot_T  = in(25);     % y acceleration reference
zddot_T  = in(26);     % z acceleration reference
psi_T    = in(27);     % reference orientation around z
psidot_T = in(28);     % referenze rotational speed around z

%=================================
%%% Control inputs
%=================================

if (CONTROL)
    
    %==================
    %%% High level part
    %==================
    Rb2w         = [cos(yaw)*cos(pitch) cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll) cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
                    sin(yaw)*cos(pitch) sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll) sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
                    -sin(pitch) cos(pitch)*sin(roll) cos(pitch)*cos(roll)];
    Rw2b         = Rb2w';
    
    e_p          = [x y z]' - [x_T y_T z_T]';
    e_v          = Rb2w*[u v w]' - [xdot_T ydot_T zdot_T]';
    e_i          = e_i_prev + Ts_lowlevel*e_p; % anti windup?
    e_pq         = [q p 0]';
    e_i_prev     = e_i;
    
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
    
    % % check
    % steps
    % Omegaup_sq = 1/k_Mup*(k_Mlo*Omegalo_sq - Mz_des);
    % F_err = Rw2b'*(k_Tup*Omegaup_sq*z_Tup + k_Tlo*Omegalo_sq*z_Tlo) - F_des

    Omega_lo_des = sqrt(Omegalo_sq);
    Omega_up_des = sqrt(1/k_Mup*(k_Mlo*Omegalo_sq - Mz_des));

    % rotation of thrust direction by zeta around z-axis (positive direction!!!)
    zeta         = zeta_mlo*Omega_lo_des + zeta_blo;
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
    u_serv1      = a_lo_des;
    u_serv2      = b_lo_des;

else
    
    Omega_up_des = sqrt(m*g/(2*k_Tup));
    Omega_lo_des = sqrt(m*g/(2*k_Tlo));
    u_serv1      = 0;
    u_serv2      = 0;
    
end

%=================================
%%% Rotor speed Saturation
%=================================

if (Omega_up_des > Omega_max)
    Omega_up_des = Omega_max;
elseif (Omega_up_des < 0)
    Omega_up_des = 0;
end
if (Omega_lo_des > Omega_max)
    Omega_lo_des = Omega_max;
elseif (Omega_lo_des < 0)
    Omega_lo_des = 0;
end

%=================================
%%% Function outputs
%=================================
% load K_new 
% load T_inv 
% load W
% state = T_inv(1:14,1:14)*[x y z u v w roll pitch yaw p q r Omega_up Omega_lo]';
% out = -W*K_new*state;
% out(1) = out(1) + sqrt(m*g/(2*k_Tup));
% out(2) = out(2) + sqrt(m*g/(2*k_Tlo));

out(1) = Omega_up_des;
out(2) = Omega_lo_des;
out(3) = u_serv1;      % responsible for a_lo
out(4) = u_serv2;      % responsible for b_lo

end