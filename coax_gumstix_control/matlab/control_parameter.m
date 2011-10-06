
%=================================
%%% Trajectory following controller
%=================================

% K_p      = diag([0.05 0.05 0.2]);
% K_v      = diag([0.25 0.25 0.5]);
K_p      = diag([0.7 0.7 4]);
K_v      = diag([0.4 0.4 2]);
K_i      = zeros(3,3);%diag([0.005 0.005 0.001]);
K_psi    = 0.013893;
K_psi_i  = 0;%0.001;
K_omegaz = 0.00166716;

contr_param.K_p = K_p;
contr_param.K_v = K_v;
contr_param.K_i = K_i;
contr_param.K_psi = K_psi;
contr_param.K_psi_i = K_psi_i;
contr_param.K_omegaz = K_omegaz;