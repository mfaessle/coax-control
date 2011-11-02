%%
clear
close all
clc

%% Create Trajectory
Waypoints = [0 -1 1; 1.5 0 0; 0 1 1; -0.5 -1 1];
duration = 10;
boarders = [-2 2 -2 2 -0.6 4];

Obstacles = create_obstacles(3,0.2,0.01,boarders);
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

figure(1)
hold on;
for j = 1:O
    K = convhull(obstCorner(sum(n_obstCorner(1:(j-1)))+1:sum(n_obstCorner(1:j)),1),obstCorner(sum(n_obstCorner(1:(j-1)))+1:sum(n_obstCorner(1:j)),2),obstCorner(sum(n_obstCorner(1:(j-1)))+1:sum(n_obstCorner(1:j)),3));
    for i = 1:size(K,1)
        fill3(obstCorner(sum(n_obstCorner(1:(j-1)))+K(i,:),1),obstCorner(sum(n_obstCorner(1:(j-1)))+K(i,:),2),obstCorner(sum(n_obstCorner(1:(j-1)))+K(i,:),3),[0.9 0.9 0.9])%,'edgecolor','none')
    end
end
plot3(Pathpoints(:,1),Pathpoints(:,2),Pathpoints(:,3),'.b','MarkerSize',10)
plot3(Pathpoints(:,1),Pathpoints(:,2),Pathpoints(:,3),'--k','LineWidth',1)
plot3(Waypoints(:,1),Waypoints(:,2),Waypoints(:,3),'.r','MarkerSize',15)
for i = 1:size(traj_param,1)/3
    t = linspace(times(i),times(i+1),100);
    x = zeros(3,100);
    for j = 1:size(traj_param,2)
        x = x + traj_param((i-1)*3+1:i*3,j)*((t.^(size(traj_param,2)-j)).*ones(1,100));
    end
    plot3(x(1,:),x(2,:),x(3,:),'g','LineWidth',2)
end
xlabel('x')
ylabel('y')
zlabel('z')
hold off;
grid on;

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
plot(t,x)
ylabel('Position [m]')
legend('x','y','z')

subplot(3,1,2)
plot(t,v)
ylabel('Velocity [m/s]')

subplot(3,1,3)
plot(t,a)
ylabel('Acceleration [m/s^2]')
xlabel('time [s]')
