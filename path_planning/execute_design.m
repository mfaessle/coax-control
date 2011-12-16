%%
clear
close all
warning off all
clc

%% Create Trajectory
Waypoints = [0 -1 1; 1.5 0 0; 0 1 1; -0.5 -1 1];
% Waypoints = [2 -1 1; 1.5 0 1.8; 2 1 1];

duration = 10;
boarders = [-2 2 -2 2 -0.6 4];

n_inter = 3;
extend_obst = 0.2;
extend_points = 0.01;

Obstacles = create_obstacles(n_inter,extend_obst,extend_points,boarders);

Polynomial = polynomial_config();

[traj_param, times, Pathpoints] = plan_trajectory(Waypoints,duration,Obstacles,Polynomial);

%% Save trajectory
times_poly = times;
poly_start_pos = Waypoints(1,:)';
poly_end_pos = Waypoints(end,:)';
save trajectory10 traj_param times_poly poly_start_pos poly_end_pos

%% Plots
Obst = create_obstacles(0,0,0,[-inf inf -inf inf -inf inf]);
n_obstCorner = Obst.n_vertices;
obstCorner = Obst.Vertices;
O = length(n_obstCorner);
NFzone = create_obstacles(0,extend_obst,0,[-inf inf -inf inf -inf inf]);
n_nfzCorner = NFzone.n_vertices;
nfzCorner = NFzone.Vertices;

figure(1)
hold on;
for j = 1:O
    plot_polygon(gcf,nfzCorner(sum(n_nfzCorner(1:(j-1)))+1:sum(n_nfzCorner(1:j)),:),NFzone.A(sum(NFzone.n(1:(j-1)))+1:sum(NFzone.n(1:j)),:),NFzone.b(sum(NFzone.n(1:(j-1)))+1:sum(NFzone.n(1:j))),[1 1 1])
    plot_polygon(gcf,obstCorner(sum(n_obstCorner(1:(j-1)))+1:sum(n_obstCorner(1:j)),:),Obst.A(sum(Obst.n(1:(j-1)))+1:sum(Obst.n(1:j)),:),Obst.b(sum(Obst.n(1:(j-1)))+1:sum(Obst.n(1:j))),[0.9 0.9 0.9])
end
plot3(Pathpoints(:,1),Pathpoints(:,2),Pathpoints(:,3),'.b','MarkerSize',10)
plot3(Pathpoints(:,1),Pathpoints(:,2),Pathpoints(:,3),'--b','LineWidth',1.3)
plot3(Waypoints(:,1),Waypoints(:,2),Waypoints(:,3),'.r','MarkerSize',15)
for i = 1:size(traj_param,1)/3
    t = linspace(times(i),times(i+1),100);
    x = zeros(3,100);
    for j = 1:size(traj_param,2)
        x = x + traj_param((i-1)*3+1:i*3,j)*((t.^(size(traj_param,2)-j)).*ones(1,100));
    end
    plot3(x(1,:),x(2,:),x(3,:),'g','LineWidth',2)
end
xlabel('$x [m]$')
ylabel('$y [m]$')
zlabel('$z [m]$')
title(horzcat('$Parameters: n = ',num2str(Polynomial.n_poly),',  k_r = ',num2str(Polynomial.kr),',  k_c = ',num2str(Polynomial.kc),'$'))
%grid on;
hold off;

t = [];
x = [];
v = [];
a = [];
for i = 1:size(traj_param,1)/3
    ti = linspace(times(i),times(i+1),100);
    xi = zeros(3,100);
    vi = zeros(3,100);
    ai = zeros(3,100);
    for j = 1:size(traj_param,2)
        xi = xi + traj_param((i-1)*3+1:i*3,j)*((ti.^(size(traj_param,2)-j)));
    end
    for j = 1:size(traj_param,2)-1
        vi = vi + (size(traj_param,2)-j)*traj_param((i-1)*3+1:i*3,j)*((ti.^(size(traj_param,2)-1-j)));
    end
    for j = 1:size(traj_param,2)-2
        ai = ai + (size(traj_param,2)-j)*(size(traj_param,2)-1-j)*traj_param((i-1)*3+1:i*3,j)*((ti.^(size(traj_param,2)-2-j)));
    end
    t = [t ti];
    x = [x xi];
    v = [v vi];
    a = [a ai];
end


figure(2)
subplot(3,1,1)
plot(t,x,'LineWidth',1.3)
ylabel('Position $[m]$')
legend('x','y','z',4)

subplot(3,1,2)
plot(t,v,'LineWidth',1.3)
ylabel('Velocity $[m/s]$')

subplot(3,1,3)
plot(t,a,'LineWidth',1.3)
ylabel('Acceleration $[m/s2]$')
xlabel('time $[s]$')
