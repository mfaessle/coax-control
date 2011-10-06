end_point = [0.5 0.5 0.5]';
t_end = tsim;

a = 0.001*rand(3,1);

b = -(2*a*t_end^5 + 3*end_point)/t_end^4;
c = (a*t_end^5 + 4*end_point)/t_end^3;

t = linspace(0,t_end,100);
x_traj = a*(t.^5) + b*(t.^4) + c*(t.^3);
v_traj = 5*a*(t.^4) + 4*b*(t.^3) + 3*c*(t.^2);

v_max = max(v_traj,[],2);

traj_param = [a b c];

% figure(1)
% plot3(x_traj(1,:),x_traj(2,:),x_traj(3,:))
% grid on;
% xlabel('x')
% ylabel('y')
% zlabel('z')
