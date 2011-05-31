%% Script for normalizing and linearizing a dynamical system

run constants

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
T = diag([1 1 1  1 1 1  pi/2 pi/2 pi/2  6*pi 6*pi 2*pi  195 195  0.1 0.1 1]);
% W = diag([500 500 0.3 0.3]);
W = diag([1 1 0.1 0.1]);

% Create states
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 x14 x15 x16 x17 real
x = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 x14 x15 x16 x17]';

% Create Inputs
syms u1 u2 u3 u4 real
u = [u1 u2 u3 u4]';

% Normalize states and inputs
z = T*x;
v = W*u;

% Create rotation matrix (body to world)
R = [cos(z(9))*cos(z(8)) cos(z(9))*sin(z(8))*sin(z(7))-sin(z(9))*cos(z(7)) cos(z(9))*sin(z(8))*cos(z(7))+sin(z(9))*sin(z(7));
     sin(z(9))*cos(z(8)) sin(z(9))*sin(z(8))*sin(z(7))+cos(z(9))*cos(z(7)) sin(z(9))*sin(z(8))*cos(z(7))-cos(z(9))*sin(z(7));
     -sin(z(8)) cos(z(8))*sin(z(7)) cos(z(8))*cos(z(7))];

%% State equations
% dx/dt = T^-1 * f (T*x, W*u) (new equations in x and u)
clear f;
f(1,1) = z(4);
f(2,1) = z(5);
f(3,1) = z(6);
thrust = 1/m*[(k_Tup*v(1)^2*z(15) + k_Tlo*v(2)^2*sin(v(4))) ...
              (k_Tup*v(1)^2*z(16) - k_Tlo*v(2)^2*sin(v(3))*cos(v(4))) ...
              (k_Tup*v(1)^2*z(17) + k_Tlo*v(2)^2*cos(v(3))*cos(v(4)))]';
f(4,1) = R(1,:)*thrust;
f(5,1) = R(2,:)*thrust;
f(6,1) = -g + R(3,:)*thrust;
f(7,1) = z(10) + z(11)*tan(z(8))*sin(z(7)) + z(12)*tan(z(8))*cos(z(7));
f(8,1) = z(11)*cos(z(7)) - z(12)*sin(z(7));
f(9,1) = z(11)*sin(z(7))/cos(z(8)) + z(12)*cos(z(7))/cos(z(8));
f(10,1) = 1/Ixx*(z(11)*z(12)*(Iyy-Izz) - d_up*k_Tup*v(1)^2*z(15) - d_lo*k_Tlo*v(2)^2*sin(v(4)));
f(11,1) = 1/Iyy*(z(10)*z(12)*(Izz-Ixx) + d_up*k_Tup*v(1)^2*z(15) + d_lo*k_Tlo*v(2)^2*sin(v(3))*cos(v(4)));
f(12,1) = 1/Izz*(z(10)*z(11)*(Ixx-Iyy) - k_Mup*v(1)^2 + k_Mlo*v(2)^2);
f(13,1) = 1/Tf_motup*(v(1) - z(13));
f(14,1) = 1/Tf_motlo*(v(2) - z(14));
f(15,1) = l_up*(-z(11)*z(17)/l_up + z(12)*z(16)/l_up);
f(16,1) = l_up*(-z(12)*z(15)/l_up + z(10)*z(17)/l_up);
f(17,1) = l_up*(-z(10)*z(16)/l_up + z(11)*z(15)/l_up);
f = simplify(T\f);

%% Find equilibrium
% may get help by: eq = solve(f) -> u0 = simplify(eq.u)
x0 = [0 0 0  0 0 0  0 0 0  0 0 0  sqrt(m*g/(2*k_Tup)) sqrt(m*g/(2*k_Tlo))  0 0 1]';
u0 = [sqrt(m*g/(2*k_Tup)) sqrt(m*g/(2*k_Tlo)) 0 0];

%% Linearize system around equilibrium
clear A;
for i=1 : length(x)
    A(:,i) = subs(diff(f,x(i)),{x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,u1,u2,u3,u4},{x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),x0(7),x0(8),x0(9),x0(10),x0(11),x0(12),x0(13),x0(14),x0(15),x0(16),x0(17),u0(1),u0(2),u0(3),u0(4)});
end;

clear B;
for i=1 : length(u)
    B(:,i) = subs(diff(f,u(i)),{x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,u1,u2,u3,u4},{x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),x0(7),x0(8),x0(9),x0(10),x0(11),x0(12),x0(13),x0(14),x0(15),x0(16),x0(17),u0(1),u0(2),u0(3),u0(4)});
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

Q            = diag([10 10 10 3 3 3 0 0 1 5 5 5 0 0]);
R            = diag([1 1 10 10]);

K_new        = dlqr(Ad(1:14,1:14),Bd(1:14,:),Q,R);
save K_new K_new
save T_inv T_inv
save W W