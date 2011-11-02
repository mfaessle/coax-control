function [feasible] = check_feasibility(Waypoints, Obstacles)
%feasible = check_feasibility(Waypoints, Obstacles) checks if all the
%points in Waypoints are feasible, i.e. not inside an obstacle or outside 
%the boarders of the environment. It returns 1 if all the way points are
%feasible and 0 otherwise.

% Matthias Fässler 2011

A = Obstacles.A;
b = Obstacles.b;
n = Obstacles.n;
boarders = Obstacles.boarders;

feasible = 1;

if (~isempty(A))
    if (any(Waypoints(:,1) < boarders(1)))
        feasible = 0;
        return;
    end
    if (any(Waypoints(:,1) > boarders(2)))
        feasible = 0;
        return;
    end
    if (any(Waypoints(:,2) < boarders(3)))
        feasible = 0;
        return;
    end
    if (any(Waypoints(:,2) > boarders(4)))
        feasible = 0;
        return;
    end
    if (any(Waypoints(:,3) < boarders(5)))
        feasible = 0;
        return;
    end
    if (any(Waypoints(:,3) > boarders(6)))
        feasible = 0;
        return;
    end
end

for i = 1:length(n)
    Ai = A(sum(n(1:i-1))+1:sum(n(1:i)),:);
    bi = b(sum(n(1:i-1))+1:sum(n(1:i)));
    for k = 1:size(Waypoints,1)
        if (Ai*Waypoints(k,:)' <= bi) % infeasible
            feasible = 0;
            return;
        end
    end
end

end