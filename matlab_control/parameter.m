%=================================
%%% Make constants global
%=================================

global g m k_Tup k_Tlo k_Mup k_Mlo d_up d_lo Tf_up Tf_lo Tf_motup Tf_motlo
global Ixx Iyy Izz l_up l_lo I_up I_lo k_springup k_springlo J_dtup J_dtlo
global Cd rho Ax Ay Az
global zeta_mup zeta_bup zeta_mlo zeta_blo
global omega_up_hat omega_lo_hat z_barx_hat z_bary_hat z_barz_hat
global K_p K_v K_i K_pq K_psi K_omegaz e_i_prev Ts_lowlevel
global Omega_max max_SBangle max_SPangle
global CONTROL GYRO_MOMENTS DRAG

%=================================
%%% Constant model parameters
%=================================

g             = 9.81;            % gravitational acceleration [m/s]
m             = 0.302;           % helicopter mass [kg] (measured)
Ixx           = 1.837e-3;        % x-axis moment of inertia [kg*m^2] (Skybotix)
Iyy           = 1.87e-3;         % y-axis moment of inertia [kg*m^2] (Skybotix)
Izz           = 2.7786e-4;       % z-axis moment of inertia [kg*m^2] (Skybotix)
I_up          = 1e-4;            % upper rotor roll pitch inertia [kg*m^2]
I_lo          = 1e-4;            % lower rotor roll pitch inertia [kg*m^2]

J_dtup        = 1.474e-4;        % upper drivetrain inertia [kg*m^2]
J_dtlo        = 7.593e-5;        % lower drivetrain inertia [kg*m^2]

d_up          = 0.165;           % distance from CoG to upper rotor hub [m] (measured)
d_lo          = 0.103;           % distance from CoG to upper rotor hub [m] (measured)

Cd            = 1.05;            % drag coefficient [-]
rho           = 1.23;            % air density [kg/m^3]
Ax            = 0.02;            % area in x-direction [m^2]
Ay            = 0.02;            % area in y-direction [m^2]
Az            = 0.019;           % area in z-direction [m^2]

Tf_up         = 1.6;             % upper rotor following time [s] (Skybotix)
Tf_lo         = 0.08;            % lower rotor following time [s] (Skybotix)
l_up          = 0.6130;          % upper rotor linkage factor [-] (Skybotix)
l_lo          = 0.5176;          % lower rotor linkage factor [-] (Skybotix)
k_springup    = 0.0542;          % spring constant upper rotor [Nm/rad] (Skybotix)
k_springlo    = 0.2139;          % spring constant lower rotor [Nm/rad] (Skybotix)

k_Tup         = 2.2607e-5;       % thrust factor upper rotor [Ns^2/rad^2] (Skybotix)
k_Tlo         = 3.1531e-5;       % thrust factor lower rotor [Ns^2/rad^2] (Skybotix)
k_Mup         = 9.9708e-7;       % moment factor upper rotor [Nms^2/rad^2] (Skybotix)
k_Mlo         = 8.9737e-7;       % moment factor lower rotor [Nms^2/rad^2] (Skybotix)

Tf_motup      = 0.185;           % following time upper motor [s] (Skybotix)
Tf_motlo      = 0.115;           % following time lower motor [s] (Skybotix)

rs_mup        = 421.3723;        % slope of input to upper rotor speed conversion [rad/s] (measured)
rs_bup        = -0.2555;         % offset of input to upper rotor speed conversion [rad/s] (measured)
rs_mlo        = 437.7036;        % slope of input to lower rotor speed conversion [rad/s] (measured)
rs_blo        = -2.9047;         % offset of input to lower rotor speed conversion [rad/s] (measured)

zeta_mup      = -0.001282;       % slope of phase lag function upper rotor [s] (Skybotix)
zeta_bup      = 0.1789;          % offset of phase lag function upper rotor [rad] (Skybotix)
zeta_mlo      = 0.001282;        % slope of phase lag function lower rotor [s] (Skybotix)
zeta_blo      = -0.0094;         % offset of phase lag function lower rotor [rad] (Skybotix)

Omega_max     = 320;             % maximum rotor speed [rad/s]
max_SBangle   = 15/360*2*pi;     % maximum stabilizer bar tilt angle [rad]
max_SPangle   = 15/360*2*pi;     % maximum swash plate tilt angle [rad]
max_Sspeed    = 9.5/4;           % maximum swash plate tilting speed [rad/s]

%% Handle for control_function

param.m = m;
param.g = g;
param.Ixx = Ixx;
param.Iyy = Iyy;
param.Izz = Izz;
param.d_up = d_up;
param.d_lo = d_lo;
param.k_springup = k_springup;
param.k_springlo = k_springlo;
param.l_up = l_up;
param.l_lo = l_lo;
param.k_Tup = k_Tup;
param.k_Tlo = k_Tlo;
param.k_Mup = k_Mup;
param.k_Mlo = k_Mlo;
param.Tf_motup = Tf_motup;
param.Tf_motlo = Tf_motlo;
param.Tf_up = Tf_up;
param.rs_mup = rs_mup;
param.rs_bup = rs_bup;
param.rs_mlo = rs_mlo;
param.rs_blo = rs_blo;
param.zeta_mup = zeta_mup;
param.zeta_bup = zeta_bup;
param.zeta_mlo = zeta_mlo;
param.zeta_blo = zeta_blo;
param.max_SPangle = max_SPangle;
param.Omega_max = Omega_max;

