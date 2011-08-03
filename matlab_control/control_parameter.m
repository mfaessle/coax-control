
%=================================
%%% Trajectory following controller
%=================================

% K_p      = diag([0.05 0.05 0.2]);
% K_v      = diag([0.25 0.25 0.5]);
K_p      = 0.2*diag([1 1 2]);
K_v      = 0.4*diag([1 1 1]);
K_i      = diag([0.005 0.005 0.001]);
K_pq     = 0.06*diag([1 -1 0]);
K_psi    = 0.015;
K_psi_i  = 0.001;
K_omegaz = 0.005;

contr_param.K_p = K_p;
contr_param.K_v = K_v;
contr_param.K_i = K_i;
contr_param.K_pq = K_pq;
contr_param.K_psi = K_psi;
contr_param.K_psi_i = K_psi_i;
contr_param.K_omegaz = K_omegaz;

%=================================
%%% Observer parameters
%=================================

omega_up_hat = sqrt(m*g/(2*k_Tup));
omega_lo_hat = sqrt(m*g/(2*k_Tlo));
z_barx_hat   = 0;
z_bary_hat   = 0;
z_barz_hat   = 1;