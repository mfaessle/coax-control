function [Path, Length] = shortest_path(Nodes, W)
%[Path, Length] = shortest_path(Nodes, W) computes the sequence of
%nodes which define the shortest path from the first point in Nodes to the
%last point in Nodes.
%   Nodes is a n x d matrix with the start point in the first row and the
%   end point in the last row. All the rows in between are potential way
%   points of the shortest path.
%
%   W is a n x n matrix containing the path lengths between all the points 
%   in Nodes. The path length from node i to node j is stored in W(i,j). 
%   If there is no connection between node i and node j, W(i,j) = inf.
%
%   Path is a vector containing the indices of the nodes in the order how
%   they build the shortest path and Length is the total length of the
%   shortest path.
%
%   The shortest path is derived using an A* depth-first-search algorithm. 
%   Lower bound of the cost to go from the current node i to the terminal
%   node T is either W(i,T) or the euclidean distance if W(i,T) = inf.

% Matthias Fässler 2011

if size(W,1) ~= size(W,2)
    error('shortest_path:InputDim', 'W must be a square matrix.');
end
if size(Nodes,1) ~= size(W,1)
    error('shortest_path:InputDim', 'The size of W must be equal to size(Nodes,1).');
end

n_nodes = size(Nodes,1);

D = inf*ones(n_nodes,1);
D(1) = 0;
Parents = zeros(n_nodes,1);

Open = 1; % initialize with Start node in Open
size_open = 1;

while (size_open > 0)
    current_node = Open(1); % remove first node in Open
    if (size_open <= 1)
        Open = [];
    else
        Open = Open(2:size_open);
    end
    size_open = size_open - 1;

    for i = 1:n_nodes
        if (i ~= current_node && W(current_node,i) ~= inf)
            if (W(i,n_nodes) ~= inf) % define lower bound of cost to go
                h = W(i,n_nodes);
            else
                h = norm(Nodes(i,:) - Nodes(n_nodes,:));
            end
            % check if better cost so far
            % if (D(current_node) + W(current_node,i) < min(D(i),D(n_nodes))) %No A*
            if (D(current_node) + W(current_node,i) + h < D(n_nodes) && D(current_node) + W(current_node,i) < D(i))
                D(i) = D(current_node) + W(current_node,i);
                Parents(i) = current_node;
                if (i ~= n_nodes)
                    Open = [i Open];
                    size_open = size_open + 1;
                end
            end
        end
    end
end

Length = D(n_nodes);
i = n_nodes;
Path = n_nodes;
while (Parents(i) ~= 0)
    Path = [Parents(i) Path];
    i = Parents(i);
end

end

