function [cornerPoints, n_corners, interPoints, n_inter] = find_corners(A, b, n, varargin)
%[cornerPoints, n_corners] = find_corners(A,b,n) computes the corners of
%obstacles defined by A and b
%   A = [A1; A2; ... Ak] and b = [b1; b2; ... bk] where k is the number of
%   obstacles and obstacle j is defined by the convex polygon Aj <= bj.
%   The vector n is a k x 1 vector with the number of constraints that define
%   each obstacle. Note that size(A) = [sum(n), 3] and size(b) =
%   [sum(n),1]. cornerPoints is a matrix that contains all the corners of all
%   the obstacles in its rows. The vector n_corners is a k x 1 vector with
%   the number of corners each obstacle has.
%   
%   [cornerPoints, n_corners] = find_corners(A,b,n,extend) moves the corners it
%   finds by an amount extend away from the obstacle. Default values is 1e-6.
%   
%   [cornerPoints, n_corners, interPoints, n_inter] = find_corners(A, b, n, extend, N)
%   is also computing N intermediate points between the extended corner
%   points along each edge. The vector n_inter is a k x 1 vector with
%   the number of intermediate points each obstacle has.
%   
%   [cornerPoints, n_corners, interPoints, n_inter] = find_corners(A, b, n, extend, N, boarders)
%   does not return corner Points that are outside of the boarders of the
%   environment devined by a 6 x 1 vector boarders = [xmin xmax ymin ymax
%   zmin zmax].

% Matthias Fässler 2011

if (isempty(A))
    cornerPoints = [];
    n_corners = [];
    interPoints = [];
    n_inter = [];
    return;
end

if nargin<3 
  error('find_corners:NotEnoughInputs','Not enough input arguments.'); 
end
if nargin>6
  error('find_corners:TooManyInputs', 'Too many input arguments.'); 
end
if size(A,2) ~= 3
    error('find_corners:InputDim', 'A must have 3 columns.');
end
if size(b,2) ~= 1
    error('find_corners:InputDim', 'b must be a column vector.');
end
if size(A,1) ~= size(b,1)
    error('find_corners:InputDim', 'A and b must have the same number of rows.');
end
if size(A,1) ~= sum(n)
    error('find_corners:nIputDim', 'A and b must have sum(n) rows.');
end

extend = 1e-6;
if (nargin > 3)
    if (varargin{1} > 0 && ~isempty(varargin{1}))
        extend = varargin{1};
    end
end
N = 0;
if (nargin > 4)
    if (~isempty(varargin{2}))
        N = varargin{2};
    end
end
boarders = [-inf inf -inf inf -inf inf];
if (nargin > 5)
    boarders = varargin{3};
end

cornerPoints = [];
Vertices = [];

O = length(n); % number of obstacles

n_corners = zeros(O,1);
n_vertices = zeros(O,1);

for j = 1:O
    Aj = A(sum(n(1:(j-1)))+1:sum(n(1:j)),:);
    bj = b(sum(n(1:(j-1)))+1:sum(n(1:j)));
    comb = nchoosek((1:size(Aj,1)),3);
    k = 1;
    kv = 1;
    Vj = zeros(size(comb,1),3);
    Cj = zeros(size(comb,1),3);
    for i = 1:size(comb,1)
        if (rank(Aj(comb(i,:),:)) == 3)
            % compute corner
            Vj(kv,:) = Aj(comb(i,:),:)\bj(comb(i,:));
            if (extend ~= 0)
                % extend
                M = Aj(comb(i,:),:);
                M(1,:) = M(1,:)/norm(M(1,:));
                M(2,:) = M(2,:)/norm(M(2,:));
                M(3,:) = M(3,:)/norm(M(3,:));
                v_extend = mean(M,1);
                v_extend = v_extend/norm(v_extend);
                Vj(kv,:) = Vj(kv,:) + extend*v_extend;
            end
            % check if out of boarders of environment
            if (Vj(kv,1) < boarders(1) || Vj(kv,1) > boarders(2) || ...
                    Vj(kv,2) < boarders(3) || Vj(kv,2) > boarders(4) || ...
                    Vj(kv,3) < boarders(5) || Vj(kv,3) > boarders(6))
                % Vertex out of boarders
            else
                Cj(k,:) = Vj(kv,:);
                k = k + 1;
            end
            kv = kv + 1;
        end
    end
    Vertices = [Vertices; Vj(1:kv-1,:)];
    n_vertices(j) = kv - 1;
    cornerPoints = [cornerPoints; Cj(1:k-1,:)];
    n_corners(j) = k-1;
end

%% Inter Corner Points
interPoints = [];
n_inter = zeros(O,1);

if (N ~= 0)
    z = (1:N)'/(N+1);
    interPoints = [];
    pot_interPoints = [];
    for j = 1:O
        Aj = A(sum(n(1:(j-1)))+1:sum(n(1:j)),:);
        bj = b(sum(n(1:(j-1)))+1:sum(n(1:j)));
        comb = nchoosek((1:n_vertices(j)),2);
        for i = 1:size(comb,1)
            Va = Vertices(sum(n_vertices(1:j-1))+comb(i,1),:);
            Vb = Vertices(sum(n_vertices(1:j-1))+comb(i,2),:);
            if (sum((Aj*Va' <= bj) ~= (Aj*Vb' <= bj)) < 3)
                pot_interPoints = [pot_interPoints; repmat(Va,N,1)+z*(Vb-Va)];
            end
        end
        k = 0;
        for i = 1:size(pot_interPoints,1)
            if (pot_interPoints(i,1) < boarders(1) || pot_interPoints(i,1) > boarders(2) || ...
                    pot_interPoints(i,2) < boarders(3) || pot_interPoints(i,2) > boarders(4) || ...
                    pot_interPoints(i,3) < boarders(5) || pot_interPoints(i,3) > boarders(6))
                % out of boarders
            else
                k = k + 1;
                interPoints = [interPoints; pot_interPoints(i,:)];
            end
        end
        n_inter(j) = k;
    end
end
                
end

