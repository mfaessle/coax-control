%% Script for normalizing and linearizing a dynamical system

% z = original (physical) states
% x = normalized states
% v = original (physical) inputs
% u = normalized inputs
% T = state normalization matrix (has normalization constants on its diagonal)
% W = input normalization matrix (has normalization constants on its diagonal)
% Normalization constants = maximum physical values that occur on the
% system

% Transformation:
% z = T*x;
% v = W*u;

%% Initialization

% Normalization matrices
%T = diag([1 1 1  1 1 1  pi/2 pi/2 pi/2  6*pi 6*pi 2*pi  195 195  0.1 0.1]);
T = eye(16);
% W = diag([500 500 0.3 0.3]);
W = eye(4);

% Create states
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 x14 x15 x16 real
x = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 x14 x15 x16]';

% Create Inputs
syms u1 u2 u3 u4 real
u = [u1 u2 u3 u4]';

% Normalize states and inputs
z = T*x;
v = W*u;

% Create rotation matrix (body to world)
Rb2w = [cos(z(9))*cos(z(8)) cos(z(9))*sin(z(8))*sin(z(7))-sin(z(9))*cos(z(7)) cos(z(9))*sin(z(8))*cos(z(7))+sin(z(9))*sin(z(7));
        sin(z(9))*cos(z(8)) sin(z(9))*sin(z(8))*sin(z(7))+cos(z(9))*cos(z(7)) sin(z(9))*sin(z(8))*cos(z(7))-cos(z(9))*sin(z(7));
        -sin(z(8)) cos(z(8))*sin(z(7)) cos(z(8))*cos(z(7))];

%% State equations
% dx/dt = T^-1 * f (T*x, W*u) (new equations in x and u)
clear f;
f(1,1) = z(4);
f(2,1) = z(5);
f(3,1) = z(6);

% Desired rotor speeds
Omega_up_des = rs_mup*v(1) + rs_bup;
Omega_lo_des = rs_mlo*v(2) + rs_blo;

% Thrust vector directions
a_lo = l_lo*v(3)*max_SPangle;
b_lo = l_lo*v(4)*max_SPangle;
z_Tup = [cos(z(15))*sin(z(16)) -sin(z(15)) cos(z(15))*cos(z(16))]'; % rot around body-x then body-y
z_Tlo = [cos(a_lo)*sin(b_lo) -sin(a_lo) cos(a_lo)*cos(b_lo)]';

% Thrust magnitudes
T_up = k_Tup*z(13)^2;
T_lo = k_Tlo*z(14)^2;

% Summarized Forces
Fx = Rb2w(1,:)*(T_up*z_Tup + T_lo*z_Tlo);
Fy = Rb2w(2,:)*(T_up*z_Tup + T_lo*z_Tlo);
Fz = -m*g + Rb2w(3,:)*(T_up*z_Tup + T_lo*z_Tlo);

M_flapup = 2*k_springup*[z(15) z(16) 0]';
M_flaplo = 2*k_springlo*[a_lo b_lo 0]';

% Summarized Moments
Mx = z(11)*z(12)*(Iyy-Izz) - T_up*z_Tup(2)*d_up - T_lo*z_Tlo(2)*d_lo + M_flapup(1) + M_flaplo(1);
My = z(10)*z(12)*(Izz-Ixx) + T_up*z_Tup(1)*d_up + T_lo*z_Tlo(1)*d_lo + M_flapup(2) + M_flaplo(2);
Mz = z(10)*z(11)*(Ixx-Iyy) - k_Mup*z(13)^2 + k_Mlo*z(14)^2;

f(4,1) = 1/m*Fx;
f(5,1) = 1/m*Fy;
f(6,1) = 1/m*Fz;
f(7,1) = z(10) + z(11)*tan(z(8))*sin(z(7)) + z(12)*tan(z(8))*cos(z(7));
f(8,1) = z(11)*cos(z(7)) - z(12)*sin(z(7));
f(9,1) = z(11)*sin(z(7))/cos(z(8)) + z(12)*cos(z(7))/cos(z(8));
f(10,1) = 1/Ixx*Mx;
f(11,1) = 1/Iyy*My;
f(12,1) = 1/Izz*Mz;
f(13,1) = 1/Tf_motup*(Omega_up_des - z(13));
f(14,1) = 1/Tf_motlo*(Omega_lo_des - z(14));
f(15,1) = -1/Tf_up*z(15) - l_up*z(10);
f(16,1) = -1/Tf_up*z(16) - l_up*z(11);
f = simplify(T\f);

%% Find equilibrium
% may get help by: eq = solve(f) -> u0 = simplify(eq.u)
Omega_lo0 = sqrt(m*g/(k_Tup*k_Mlo/k_Mup + k_Tlo));
Omega_up0 = sqrt(k_Mlo/k_Mup*Omega_lo0^2);

x0 = [0 0 0  0 0 0  0 0 0  0 0 0  Omega_up0 Omega_lo0  0 0]';
u0 = [(Omega_up0 - rs_bup)/rs_mup (Omega_lo0 - rs_blo)/rs_mlo 0 0]';

%% Linearize system around equilibrium
clear A;
for i=1 : length(x)
    A(:,i) = subs(diff(f,x(i)),{x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,u1,u2,u3,u4},{x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),x0(7),x0(8),x0(9),x0(10),x0(11),x0(12),x0(13),x0(14),x0(15),x0(16),u0(1),u0(2),u0(3),u0(4)});
end;

clear B;
for i=1 : length(u)
    B(:,i) = subs(diff(f,u(i)),{x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,u1,u2,u3,u4},{x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),x0(7),x0(8),x0(9),x0(10),x0(11),x0(12),x0(13),x0(14),x0(15),x0(16),u0(1),u0(2),u0(3),u0(4)});
end;

%% Embed normalized control:
% measure information of physical states z
% transform to normalized state information by: x = T^-1 * z
% run observer algorithm, compute normalized control input u
% transform to physical control input by: v = W*u

% rank(ctrb(A,B))

%=================================
%%% LQR Design
%=================================

Ts_lowlevel  = 0.01;            % sampling time of low level controller [s]

T_inv        = inv(T);          % state normalization

n            = size(A,1);
p            = size(B,2);

sysd         = c2d(ss(A,B,eye(n),zeros(n,p)),Ts_lowlevel,'zoh');
Ad           = sysd.A;
Bd           = sysd.B;

% Q            = 0.01*diag([5 5 10 0.5 0.5 0.5 2 2 2 0.01 0.01 0.01 0.1 0.1 1 1]);
% Q            = 0.01*diag([5.5 5.5 80  10 10 1  0.01 0.01 0.08  0.07 0.07 0.01  0.1 0.1 1 1]);
Q            = 0.01*diag([5.5 5.5 80  10 10 1  0.01 0.01 0.08  0.1 0.1 0.01  0.1 0.1 1 1]);
R            = eye(4);

K_lqr        = dlqr(Ad,Bd,Q,R);

% knock out feedback from roll/pitch
K_lqr(3:4,7:8) = zeros(2,2);
% knock out feedback from a_up/b_up
K_lqr(3:4,15:16) = zeros(2,2);

% Control parameter struct
contr_param.K_lqr = K_lqr;
contr_param.T_inv_lqr = T_inv; 
contr_param.W_lqr = W; 

