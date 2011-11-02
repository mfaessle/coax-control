function [visibility, occluding] = isvisible(x,y,Obstacles)
%isvisible(x,y,Obstacles) returns 1 if point y is visible from point x with
%obstacles defined by an Obstacles object (see create_obstacles.m).
%
%   [visibility, occluding] = isvisible(x,y,Obstacles) returns a k x 1 vector
%   occluding which has an entry 1 at row i if obstacle i is occluding the
%   direct path from x to y.

% Matthias Fässler 2011

A = Obstacles.A;
b = Obstacles.b;
n = Obstacles.n;

visibility = 1;
if (isempty(A))
    occluding = [];
    return;
end

if nargin<3 
  error('isvisible:NotEnoughInputs','Not enough input arguments.'); 
end
if nargin>3
  error('isvisible:TooManyInputs', 'Too many input arguments.'); 
end
if size(A,2) ~= 3
    error('isvisible:InputDim', 'A must have 3 columns.');
end
if size(b,2) ~= 1
    error('isvisible:InputDim', 'b must be a column vector.');
end
if size(A,1) ~= size(b,1)
    error('isvisible:InputDim', 'A and b must have the same number of rows.');
end
if size(A,1) ~= sum(n)
    error('isvisible:InputDim', 'A and b must have sum(n) rows.');
end

nobst = length(n);
occluding = zeros(nobst,1);

for j = 1:nobst
    Aj = A(sum(n(1:j-1))+1:sum(n(1:j)),:);
    bj = b(sum(n(1:j-1))+1:sum(n(1:j)));
    if (sum(Aj*x <= bj) == n(j) || sum(Aj*y <= bj) == n(j)) % check if any poitn is inside the obstacle
        visibility = 0;
        if (nargout == 2)
            occluding(j) = 1;
        else
            break;
        end
    else
        Aineq = [-ones(n(j),1) Aj*(y-x); -1 0; 0 1; 0 -1];
        bineq = [bj - Aj*x; 0; 1; 0];
        f = [1 0]';
        c = linprog(f,Aineq,bineq,[],[],[],[],[],optimset('LargeScale','off','Display','off'));

        if (c(1) < 1e-4)
            visibility = 0;
            if (nargout == 2)
                occluding(j) = 1;
            else
                break;
            end
        end
    end
end

end

