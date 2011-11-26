function [Obstacles] = create_obstacles(n_inter, extend, extend_points, boarders)
%Obstacles = create_obstacles(n_inter, extend, extend_points, boarders)
%creates an Obstacle object from the matrices defined in this function as
%well as the function arguments.
%
%   An obstacle  i is defined as a polynomial defined by Ai*x <= bi. For
%   every obstacle i also the number of constraints that define it have to
%   be stored in ni. All the matrices Ai, the vectors bi and the
%   constraints numbers are then stacked in A, b and n respectively.
%
%   An Obstacle object contains the vertices of all the obstacles as well
%   as points along their edges. It also contains the matrices A, b and n
%   as defined above, the boarders of the environment as well as the number
%   of vertices and points along the edges each obstacle has. Note that
%   points that are outside of the environments boarders are not kept in
%   the Obstacles object.
%
%   n_inter is the number of points along the edges between the vertices
%   that should be created. extend is the amount by which the obstacles are
%   extended in order to incorporate the dimensions of the helicopter.
%   extend_points is the amount by which the vertices that are created are
%   extended from the already extended obstacles. FInally, boarders defines
%   the boarders of the environment by a vetor 
%   [xmin xmax ymin ymax zmin zmax].

%   Matthias Fässler 2011

%% Obstacles
% A1 = kron(eye(3),[1 -1]');
% b1 = [1.5 -1 1 -0.5 0.2 0]';
% n1 = size(A1,1);
% 
% A2 = kron(eye(3),[1 -1]');
% b2 = [1.2 -0.8 0.5 -0.2 0.3 -0.1]';
% n2 = size(A2,1);
% 
% A3 = kron(eye(3),[1 -1]');
% b3 = [1.5 0 1.2 -1 0.4 0]';
% n3 = size(A3,1);
% 
% A4 = kron(eye(3),[1 -1]');
% b4 = [0.6 -0.2 0.6 -0.2 0.4 -0.25]';
% n4 = size(A4,1);
% 
% A5 = kron(eye(3),[1 -1]');
% b5 = [1 1 0.31 0.31 1.32 -0]';
% n5 = size(A5,1);
% 
% % summarize obstacles
% A = [A1; A5];
% b = [b1; b5];
% n = [n1 n5]; % number of constraints from obstacles

%% Obstacles
A1 = kron(eye(3),[1 -1]');
b1 = [2.5 -1.5 0.4 0.4 1.5 -0]';
n1 = size(A1,1);

% summarize obstacles
A = [A1];
b = [b1];
n = [n1]; % number of constraints from obstacles
O = length(n); % number of obstacles

%% Extend Obstacles
if (~isempty(A))
    O = length(n);
    for j = 1:O
        Aj = A(sum(n(1:j-1))+1:sum(n(1:j)),:);
        bj = b(sum(n(1:j-1))+1:sum(n(1:j)));
        for i = 1:length(bj)
            bj(i) = bj(i) + extend*norm(Aj(i,:));
        end
        b(sum(n(1:j-1))+1:sum(n(1:j))) = bj;
    end
end

%% Find vertices and Points along edges
[Vertices, n_vertices, interPoints, n_inter] = find_corners(A,b,n,extend_points,n_inter,boarders);

%% Create Obstacles Object
Obstacles.Vertices = [Vertices; interPoints];
Obstacles.A = A;
Obstacles.b = b;
Obstacles.n = n;
Obstacles.boarders = boarders;
Obstacles.n_vertices = n_vertices;
Obstacles.n_inter = n_inter;

end

