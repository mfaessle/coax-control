function [contr_param] = control_parameter()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%K_p      = diag([0.7 0.7 4]);
K_p      = diag([0.5 0.5 4]);
%K_v      = diag([0.5 0.5 2.5]);
K_v      = diag([0.3 0.3 2.5]);
%K_i      = diag([0.005 0.005 0.1]);
K_i      = zeros(3,3);
K_psi    = 0.013893;
%K_psi_i  = 0.0005;
K_psi_i  = 0;
K_omegaz = 0.00166716;

contr_param.K_p = K_p;
contr_param.K_v = K_v;
contr_param.K_i = K_i;
contr_param.K_psi = K_psi;
contr_param.K_psi_i = K_psi_i;
contr_param.K_omegaz = K_omegaz;

end

