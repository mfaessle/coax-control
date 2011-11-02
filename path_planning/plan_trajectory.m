function [traj_param, times, Pathpoints] = plan_trajectory(Waypoints, duration, Obstacles, Polynomial)
%traj_param, times, Pathpoints = plan_trajectory(Waypoints, duration, 
%Obstacles, Polynomial) creates a trajectory through way points defined in
%Waypoints. The trajectory starts at the first way point at time zero and
%ends at the last way point specified in Waypoints at time duration. 
%
%   The trajectory stays within the given environment and avoids obstacles 
%   defined in Obstacles. Polynomial contains the parameters of how the 
%   polynomial trajectory is built. See create_obstacles.m and
%   polynomial_config.m for details about the Obstacles and Polynomial
%   objects respectively.
%
%   traj_param and times are the original outputs from poly_trajectory.m 
%   whereas Pathpoints the Waypoints plus some additional way points that 
%   might have been created in this function.

%   Matthias Fässler 2011

%%
Points = [Obstacles.Vertices; Waypoints];

n_poly = Polynomial.n_poly;
kr = Polynomial.kr;
weights = Polynomial.weights;
kc = Polynomial.kc;
start_velaccel = Polynomial.start_velaccel;
end_velaccel = Polynomial.end_velaccel;

%% Check feasibility of Waypoints
if (~check_feasibility(Waypoints,Obstacles))
    error('plan_trajectory:Feasibility', 'Not all Waypoints are feasible!');
end

%% Compute shortest feasible path
p = size(Points,1);
W = zeros(p,p);
for i = 1:p
    for j = i+1:p
        W(i,j) = norm(Points(i,:)-Points(j,:));
    end
end
W = W + W'; % make weigths symmetric
feasible = 0;
while feasible == 0
    path = shortest_path(Obstacles.Vertices, Waypoints, W);
    % check feasibility
    feasible = 1;
    for j = 1:length(path)-1
        x = Points(path(j),:)';
        y = Points(path(j+1),:)';
        if (isvisible(x,y,Obstacles) == 0)
            feasible = 0;
            W(path(j),path(j+1)) = inf;
            W(path(j+1),path(j)) = inf;
        end
    end
end

%% Compute Polynomial Trajectory
Pathpoints = Points(path,:);
npp = size(Pathpoints,1);
segmentLength = zeros(npp-1,1);
for i = 1:npp-1
    if (i == 1)
        segmentLength(i) = norm(Pathpoints(i+1,:)-Pathpoints(i,:));
    else
        segmentLength(i) = segmentLength(i-1) + norm(Pathpoints(i+1,:)-Pathpoints(i,:));
    end
end
times = zeros(npp,1);
times(2:npp) = segmentLength/segmentLength(npp-1)*duration;

Weights = repmat(weights,npp-1,1);

traj_param = poly_trajectory(Pathpoints, times, n_poly, kr, Weights, kc, start_velaccel, end_velaccel);

%% Check Collision and recompute Trajectory if necessary
hit_obstacle = 1;
while hit_obstacle ~= 0
    hit_after = check_collision(traj_param,times,Obstacles);
    if (hit_after ~= 0) % collision
        Weights(hit_after,1) = Weights(hit_after,1) + 5000;
        traj_param = poly_trajectory(Pathpoints, times, n_poly, kr, Weights, kc, start_velaccel, end_velaccel);
    else
        hit_obstacle = 0;
    end
end

end

