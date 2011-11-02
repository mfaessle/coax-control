function [Path, Length] = shortest_path(Nodes, Waypoints, W)
%[Path, Length] = shortest_path(Nodes, Waypoints, W) computes the sequence of
%nodes which define the shortest path through the points and their order
%defined in Waypoints.
%   Nodes is a nn x d matrix with nodes that do not have to be visited on 
%   the path in d dimensional space in its rows. Waypoints is a nwp x d
%   matrix with nodes that must be visited on the path in its rows.
%
%   W is a nn+nwp x nn+nwp matrix containg the path lengths between all the
%   nodes defined by the matrix [Nodes; Waypoints]. The path length from 
%   node i to node j is stored in W(i,j). If there is no connection between
%   node i and node j, W(i,j) = inf.
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
if size(Nodes,1)+size(Waypoints,1) ~= size(W,1)
    error('shortest_path:InputDim', 'The size of W must be equal to size(Nodes,1)+size(Waypoints,1).');
end

Path = [];
n_wp = size(Waypoints,1);
Nodesk = [Nodes; Waypoints];

for k = 1:n_wp-1
    
    n_nodes = size(Nodes,1);
    n = n_nodes + n_wp; % number of nodes with current start and end
    
    Wk = W;
    for j = 1:n_wp
        if (j~=k && j~=k+1)
            Wk(:,n_nodes+j) = inf*ones(n_nodes+n_wp,1);
            Wk(n_nodes+j,:) = inf*ones(1,n_nodes+n_wp);
        end
    end
    idx_start = n_nodes + k;
    idx_end = idx_start + 1;
    
    D = inf*ones(n,1);
    D(idx_start) = 0;
    Parentsk = zeros(n,1);

    Open = idx_start; % initialize with Start node in Open
    size_open = 1;

    while (size_open > 0)
        current_node = Open(1); % remove first node in Open
        if (size_open <= 1)
            Open = [];
        else
            Open = Open(2:size_open);
        end
        size_open = size_open - 1;

        for i = 1:n
            if (i ~= current_node && Wk(current_node,i) ~= inf)
                if (Wk(i,idx_end) ~= inf) % define lower bound of cost to go
                    h = Wk(i,idx_end);
                else
                    h = norm(Nodesk(i,:) - Nodesk(idx_end,:));
                end
                % check if better cost so far
                % if (D(current_node) + Wk(current_node,i) < min(D(i),D(n))) %No A*
                if (D(current_node) + Wk(current_node,i) + h < D(idx_end) && D(current_node) + Wk(current_node,i) < D(i))
                    D(i) = D(current_node) + Wk(current_node,i);
                    Parentsk(i) = current_node;
                    if (i ~= idx_end)
                        Open = [i Open];
                        size_open = size_open + 1;
                    end
                end
            end
        end
    end

    Length = D(idx_end);
    i = idx_end;
    Pathk = idx_end;
    while (Parentsk(i) ~= 0)
        Pathk = [Parentsk(i) Pathk];
        i = Parentsk(i);
    end
    
    if k < n_wp-1
        Path = [Path Pathk(1:end-1)];
    else
        Path = [Path Pathk];
    end
end

end

