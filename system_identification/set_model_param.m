function [param] = set_model_param(parameter)

param.m           = parameter(1);
param.g           = parameter(2);
param.Ixx         = parameter(3);
param.Iyy         = parameter(4);
param.Izz         = parameter(5);
param.d_up        = parameter(6);
param.d_lo        = parameter(7);
param.k_springup  = parameter(8);
param.k_springlo  = parameter(9);
param.l_up        = parameter(10);
param.l_lo        = parameter(11);
param.k_Tup       = parameter(12);
param.k_Tlo       = parameter(13);
param.k_Mup       = parameter(14);
param.k_Mlo       = parameter(15);
param.Tf_motup    = parameter(16);
param.Tf_motlo    = parameter(17);
param.Tf_up       = parameter(17);
param.rs_mup      = parameter(19);
param.rs_bup      = parameter(20);
param.rs_mlo      = parameter(21);
param.rs_blo      = parameter(22);
param.zeta_mup    = parameter(23);
param.zeta_bup    = parameter(24);
param.zeta_mlo    = parameter(25);
param.zeta_blo    = parameter(26);
param.max_SPangle = parameter(27);
param.Omega_max   = 320;

end

