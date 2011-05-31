
%=================================
%%% Trajectory following controller
%=================================

% K_p      = diag([0.05 0.05 0.2]);
% K_v      = diag([0.25 0.25 0.5]);
K_p      = 0.3*diag([1 1 1]);
K_v      = 0.3*diag([1 1 1]);
K_i      = diag([0 0 0.01]);
K_pq     = 0.1*diag([1 -1 0]);
K_psi    = 0.01;
K_omegaz = 0.01;

contr_param.K_p = K_p;
contr_param.K_v = K_v;
contr_param.K_i = K_i;
contr_param.K_pq = K_pq;
contr_param.K_psi = K_psi;
contr_param.K_omegaz = K_omegaz;

%=================================
%%% Observer parameters
%=================================

omega_up_hat = sqrt(m*g/(2*k_Tup));
omega_lo_hat = sqrt(m*g/(2*k_Tlo));
z_barx_hat   = 0;
z_bary_hat   = 0;
z_barz_hat   = 1;