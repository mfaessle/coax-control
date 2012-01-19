%% load Vicon Data and Inputs
% y Outputs
% u Inputs
% load ViconData
load gumstix_sysid
Nstart = 1;
Nstop = size(Data.lintwist,2);
Time = Data.time(Nstart:Nstop)';

% single measurements must be in rows!
y = [Data.lintwist(:,Nstart:Nstop)' Data.angtwist(:,Nstart:Nstop)'];
u = Data.inputs(:,Nstart:Nstop)';

%% load parameters
load nlgr_hy
id_param = cell2mat(getpar(nlgr));
param = set_model_param(id_param);
param.m = 0.333;

%% create IDDATA object with CoaX data
z = iddata(y, u, [], 'SamplingInstants', Time);%, 'Name', 'CoaX_data');
set(z, 'InputName', {'Motor up' 'Motor lo' 'Servo roll' 'Servo pitch'}, ...
       'InputUnit', {'-' '-' '-' '-'});
set(z, 'OutputName', {'x vel' 'y vel' 'z vel' 'p' 'q' 'r'}, ...
       'OutputUnit', {'m/s' 'm/s' 'm/s' 'rad/s' 'rad/s' 'rad/s'});
set(z, 'TimeUnit', 's');

%% creating a CoaX IDNLGREY model object
FileName      = 'CoaX_grey_box_vel'; % File describing the model structure.
Order         = [6 4 17]; % Model orders [ny nu nx].
% Parameters    = getpar(nlgr);
Parameters    = {param.m; param.g; param.Ixx; param.Iyy; param.Izz; param.d_up; param.d_lo; param.k_springup; param.k_springlo; ...
                param.l_up; param.l_lo; param.k_Tup; param.k_Tlo; param.k_Mup; param.k_Mlo; param.Tf_motup; param.Tf_motlo; ...
                param.Tf_up; param.rs_mup; param.rs_bup; param.rs_mlo; param.rs_blo; param.zeta_mup; param.zeta_bup; ...
                param.zeta_mlo; param.zeta_blo; param.max_SPangle}; % Initial parameters.
Parameters{27} = 14/180*pi;
Omega_lo0 = sqrt(param.m*param.g/(param.k_Tup*param.k_Mlo/param.k_Mup + param.k_Tlo));
Omega_up0 = sqrt(param.k_Mlo/param.k_Mup*Omega_lo0^2);
InitialStates = [0 0 0  y(1,1:3)  0 0 0  y(1,4:6)  Omega_up0 Omega_lo0  0 0 1]'; % Initial initial states.
Ts            = 0;                     % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, 'Name', 'CoaX');
set(nlgr, 'InputName', {'Motor up' 'Motor lo' 'Servo roll' 'Servo pitch'}, ...
          'InputUnit', {'-' '-' '-' '-'}, ...
          'OutputName', {'x vel' 'y vel' 'z vel' 'p' 'q' 'r'}, ...
          'OutputUnit', {'m/s' 'm/s' 'm/s' 'rad/s' 'rad/s' 'rad/s'}, ...
          'TimeUnit', 's');

setinit(nlgr, 'Name', {'x-position' 'y-position' 'z-position' 'x-velocity' ...
                       'y-velocity' 'z-velocity' 'roll' 'pitch' ' yaw' 'p' ...
                       'q' 'r' 'Omega_up' 'Omega_lo' 'z_barx' 'z_bary' 'z_barz'});
setinit(nlgr, 'Unit', {'m' 'm' 'm' 'm/s' 'm/s' 'm/s' 'rad' 'rad' 'rad' 'rad/s' ...
                       'rad/s' 'rad/s' 'rad/s' 'rad/s' '-' '-' '-'});
setinit(nlgr, 'Fixed', {false false false false false false false false false ...
                        false false false false false false false false}); 
setinit(nlgr, 'Minimum', {-5 -5 0 -2 -2 -2 -pi/2 -pi/2 -2*pi -2.5 -2.5 -2.5 0 0 -0.25 -0.25 0});
setinit(nlgr, 'Maximum', {5 5 5 2 2 2 pi/2 pi/2 2*pi 2.5 2.5 2.5 320 320 0.25 0.25 1});
setpar(nlgr, 'Name', {'helicopter mass' ...
                      'gravitational acceleration' ...
                      'x-axis moment of inertia' ...
                      'y-axis moment of inertia' ...
                      'z-axis moment of inertia' ...
                      'distance from CoG to upper rotor hub' ...
                      'distance from CoG to lower rotor hub' ...
                      'spring constant upper rotor' ...
                      'spring constant lower rotor' ...
                      'upper rotor linkage factor' ...
                      'lower rotor linkage factor' ...
                      'thrust factor upper rotor' ...
                      'thrust factor lower rotor' ...
                      'moment factor upper rotor' ...
                      'moment factor lower rotor' ...
                      'rise time upper motor' ...
                      'rise time lower motor' ...
                      'stabilizer bar rise time' ...
                      'slope of input to upper rotor speed conversion' ...
                      'offset of input to upper rotor speed conversion' ...
                      'slope of input to lower rotor speed conversion' ...
                      'offset of input to lower rotor speed conversion' ...
                      'slope of phase lag function upper rotor' ...
                      'offset of phase lag function upper rotor' ...
                      'slope of phase lag function lower rotor' ...
                      'offset of phase lag function lower rotor' ...
                      'maximum swash plate tilt angle'});
setpar(nlgr, 'Unit', {'kg' 'm/s^2' 'kg*m^2' 'kg*m^2' 'kg*m^2' 'm' 'm' 'Nm/rad' ...
                      'Nm/rad' '-' '-' 'Ns^2/rad^2' 'Ns^2/rad^2' 'Nms^2/rad^2' ...
                      'Nms^2/rad^2' 's' 's' 's' 'rad/s' 'rad/s' 'rad/s' 'rad/s' ...
                      's' 'rad' 's' 'rad' 'rad'});
setpar(nlgr, 'Minimum', {eps(0); eps(0); eps(0); eps(0); eps(0); eps(0); eps(0); ...
                         eps(0); eps(0); eps(0); eps(0); eps(0); eps(0); eps(0); ...
                         eps(0); eps(0); eps(0); eps(0); eps(0); -5; eps(0); ...
                         -5; -0.1; -0.5; -0.1; -0.5; eps(0)}); 
setpar(nlgr, 'Maximum', {1; 10; 0.05; 0.05; 0.05; 0.2; 0.2; 0.5; 0.5; 1; 1; 5e-5; 5e-5; ...
                         1e-5; 1e-5; 1; 1; 5; 1000; 5; 1000; 5; 0.1; 0.5; 0.1; ...
                         0.5; 0.3});

% Set fixed parameters
nlgr.Parameters(1).Fixed  = true; % m
nlgr.Parameters(2).Fixed  = true; % g
nlgr.Parameters(3).Fixed  = true; % Ixx
nlgr.Parameters(4).Fixed  = true; % Iyy
nlgr.Parameters(5).Fixed  = true; % Izz
nlgr.Parameters(6).Fixed  = true; % d_up
nlgr.Parameters(7).Fixed  = true; % d_lo
% nlgr.Parameters(8).Fixed  = true; % k_springup
% nlgr.Parameters(9).Fixed  = true; % k_springlo
% nlgr.Parameters(10).Fixed = true; % l_up
% nlgr.Parameters(11).Fixed = true; % l_lo
% nlgr.Parameters(12).Fixed = true; % k_Tup
% nlgr.Parameters(13).Fixed = true; % k_Tlo
% nlgr.Parameters(14).Fixed = true; % k_Mup
% nlgr.Parameters(15).Fixed = true; % k_Mlo
% nlgr.Parameters(16).Fixed = true; % Tf_motup
% nlgr.Parameters(17).Fixed = true; % Tf_motlo
% nlgr.Parameters(18).Fixed = true; % Tf_up
nlgr.Parameters(19).Fixed = true; % rs_mup
nlgr.Parameters(20).Fixed = true; % rs_bup
nlgr.Parameters(21).Fixed = true; % rs_mlo
nlgr.Parameters(22).Fixed = true; % rs_blo
% nlgr.Parameters(23).Fixed = true; % zeta_mup
% nlgr.Parameters(24).Fixed = true; % zeta_bup
% nlgr.Parameters(25).Fixed = true; % zeta_mlo
% nlgr.Parameters(26).Fixed = true; % zeta_blo
nlgr.Parameters(27).Fixed = true; % max_SPangle
% getpar(nlgr, 'Fixed')

%% Simulation with initial parameter values
% nlgr.Algorithm.SimulationOptions.AbsTol = 1e-6;
% nlgr.Algorithm.SimulationOptions.RelTol = 1e-5;
% figure;
% compare(z, nlgr);

%% Parameter estimation

% nlgr.Algorithm.Criterion = 'Det';
nlgr.Algorithm.SearchMethod = 'Auto';
nlgr = pem(z, nlgr, 'Display', 'Full', 'MaxIter', 10);
% figure;
% compare(z, nlgr);

%% Summary of estimated model
present(nlgr);

%% Get Model Parameters
% id_param = cell2mat(getpar(nlgr));
% param = set_model_param(id_param); % set identified parameters
% 
% %% Simulate Model
% t0 = Time(1);
% % x0 = findstates(nlgr,z);
% x0 = cell2mat(getinit(nlgr));
% 
% % Outputs
% X = zeros(length(Time),length(x0));
% X(1,:) = x0';
% 
% t = t0;
% x = x0;
% for i=Nstart : Nstop-1
%     
%     tstart = Time(i-Nstart+1);
%     tstop = Time(i+1-Nstart+1);
%     
%     control = u(i-Nstart+1,:)';
%     
%     [time,state] = ode45(@CoaX_grey_box_vel,[tstart tstop],x,[],control,m,g,Ixx,Iyy,Izz,d_up,d_lo,k_springup,k_springlo,l_up,l_lo,k_Tup,k_Tlo,k_Mup,k_Mlo,Tf_motup,Tf_motlo,Tf_up,rs_mup,rs_bup,rs_mlo,rs_blo,zeta_mup,zeta_bup,zeta_mlo,zeta_blo,max_SPangle);
%     
%     x = state(end,:)';
%     
%     X(i-Nstart+1,:) = x';
% end
% 
% %% Plot
% figure;
% subplot(4,1,1)
% plot(Time,X(:,1:6))
% grid on;
% legend('x','y','z','u','v','w')
% title('Translational states')
% 
% subplot(4,1,2)
% plot(Time,X(:,7:12))
% grid on;
% legend('\phi','\theta','\psi','p','q','r')
% title('Rotational states')
% 
% subplot(4,1,3)
% plot(Time,X(:,13:14))
% grid on;
% legend('\omega_{up}','\omega_{lo}')
% title('Rotor speeds')
% 
% subplot(4,1,4)
% plot(Time,X(:,15:17))
% grid on;
% legend('x','y','z')
% title('Upper thrust vector direction')
% 
% figure;
% subplot(2,1,1)
% plot(Time,u(:,1:2))
% grid on;
% legend('u_{mot,up}','u_{mot,lo}')
% title('Motor Inputs')
% 
% subplot(2,1,2)
% plot(Time,u(:,3:4))
% grid on;
% legend('u_{serv1}','u_{serv2}')
% title('Servo Inputs')
