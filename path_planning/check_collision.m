function [hit_in, obstacle] = check_collision(traj_param,times,Obstacles)
%hit_after = check_collision(traj_param,times,A,b,n_constr,boarders)
%computes the index of the polynomial of a picewise polynomial trajectory
%that hits an obstacle or the boarder of the environment. It returns 0 if
%there is no collision.
%
%   [hit_in, obstacle] = check_collision(traj_param,times,A,b,n_constr,boarders)
%   also returns the index of the obstacle that the trajectory is colliding
%   with. The return value obstacle will be 0 if the trajectory goes
%   outside of the boarders of the environment.
%
%   traj_param is a 3*m x n+1 matrix with the coefficients of all the
%   polynomials in the trajectory where m is the number of polynomials and
%   n is their order as it is computed by the function poly_trajectory.
%   times is a m+1 x 1 vector with the times at which the waypoints are
%   visited.
%   A = [A1; A2; ... Ak] and b = [b1; b2; ... bk] where k is the number of
%   obstacles and obstacle j is defined by the convex polygon Aj <= bj.
%   The vector n_constr is a k x 1 vector with the number of constraints 
%   that define each obstacle. Note that size(A) = [sum(n_constr), 3] and 
%   size(b) = [sum(n_constr),1].
%   boarders defines a cubical environment by a min and max value on each
%   coordinate. boarders = [xmin xmax ymin ymax zmin zmax]

% Matthias Fässler 2011

A = Obstacles.A;
b = Obstacles.b;
n_constr = Obstacles.n;
boarders = Obstacles.boarders;

O = length(n_constr);
m = length(times) - 1;
N = 100;
hit_in = 0;
obstacle = 0;

for i = 1:m
    t = linspace(times(i),times(i+1),N);
    x = zeros(3,N);
    for j = 1:size(traj_param,2)
        x = x + traj_param((i-1)*3+1:i*3,j)*((t.^(size(traj_param,2)-j)).*ones(1,N));
    end
    for k = 1:N
        xk = x(:,k);
        if (~isempty(A))
            for j = 1:O
                Aj = A(sum(n_constr(1:(j-1)))+1:sum(n_constr(1:j)),:);
                bj = b(sum(n_constr(1:(j-1)))+1:sum(n_constr(1:j)));
                if (Aj*xk <= bj) % hitting obstacle
                    hit_in = i;
                    obstacle = j;
                    return;
                end
            end
        end
        if (xk(1) < boarders(1) || xk(1) > boarders(2) || ...
                xk(2) < boarders(3) || xk(2) > boarders(4) || ...
                xk(3) < boarders(5) || xk(3) > boarders(6))
            hit_in = i;
            return;
        end
    end
end

end

