function [traj_param] = poly_trajectory(Waypoints, times, n, kr, mu, kc, varargin)
%poly_trajectory(Waypoints, times, n, kr, kc) finds a piecewise polynomial 
%trajectory of order n through the defined Waypoints at instances of time 
%times. It minmizes a weighted sum of the derivatives of the deviation
%from the straight line trajectory up to the order kr integrated over time. 
%It also forces continuity of the first kc derivatives of position.
%
%   Waypoints is a m+1 x d matrix with the desired waypoints in it's rows,
%   where m is the number of polynomials to be designed and d is the 
%   dimension of the space in which the polynomial is designed. The vector 
%   times contains the instances of time when these waypoints have to be 
%   reached. The m x kr+1 matrix mu contains the weights with wich the 
%   deviation of the different derivatives for each piece of the trajectory
%   are weighted. The output traj_param is a d*m x n+1 matrix with 
%   parameters of the first polynomial in the first d rows ans so on.
%
%   poly_trajectory(Waypoints, times, n, kr, kc, start_velaccel,
%   end_velaccel) incorporates a desirec acceleration and velocity at the
%   start and end point respectively. start_velaccel and end_velaccel are
%   d x 2 matrices with velocity in their first column and acceleration in
%   their second column. Default value is zero.
%
%   Note that in this code only the deviation in position gets minimized
%   with respect to the straight line trajectory. All derivatives of the
%   position are just punished according to their norms.

% Matthias Fässler 2011

if nargin<6 
  error('poly_trajectory:NotEnoughInputs','Not enough input arguments.'); 
end
if nargin>8
  error('poly_trajectory:TooManyInputs', 'Too many input arguments.'); 
end
if times(1) ~= 0
    error('poly_trajectory:InconsistentInput', 'times(1) must be 0.'); 
end
if size(Waypoints,1) ~= length(times)
    error('poly_trajectory:InputDim', 'Waypoints and times must have the same number of rows.');
end
if size(mu,2) ~= kr+1
    error('poly_trajectory:InputDim', 'size(mu,2) must be kr+1.');
end
if n<4+kc
    error('poly_trajectory:Input', 'n must be larger or equal to 4+kc.');
end

new_fact = [1 cumprod(1:n)];

%% Initialize some Values
dim = size(Waypoints,2);
start_velaccel = zeros(dim,2);
end_velaccel = zeros(dim,2);
if (nargin > 6)
    if (~isempty(varargin{1}))
        start_velaccel = varargin{1};
    end
end
if (nargin > 7)
    end_velaccel = varargin{2};
end
m = length(times) - 1;
param = zeros(dim,m*(n+1));

%% Normalize time
alpha = times(end);
times = times/alpha;

%% Normalize Space
beta1 = min(Waypoints,[],1)';
beta2 = max(Waypoints,[],1)' - beta1;
Waypoints = (Waypoints - repmat(beta1',m+1,1))./repmat(beta2',m+1,1);

for d = 1:dim
    %% Create H matrix
    H = zeros((n+1)*m);
    for h = 0:kr
        Hh = zeros((n+1)*m);
        terms = n-h+1;
        H_basis = zeros(n-h+1);
        for i = 0:terms-1
            for j = 0:terms-1
                num = new_fact(n-i+1)/new_fact(n-i-h+1)*new_fact(n-j+1)/new_fact(n-j-h+1);
                % num = factorial(n-i)/factorial(n-i-h)*factorial(n-j)/factorial(n-j-h);
                denom = 2*(n-h) + 1 - i - j;
                H_basis(i+1,j+1) = num/denom;
            end
        end
        for k = 1:m
            Hk = zeros(n+1-h);
            for i = 0:terms-1
                for j = 0:terms-1
                    denom = 2*(n-h) + 1 - i - j;
                    Hk(i+1,j+1) = (times(k+1)^(denom) - times(k)^(denom));
                end
            end
            Hh((k-1)*(n+1)+1:k*(n+1)-h,(k-1)*(n+1)+1:k*(n+1)-h) = Hk.*H_basis;
        end
        H = H + Hh.*kron(diag(mu(:,h+1)),ones(n+1));
    end

    %% Create f Vector (punish deviation in position from straight line)
    f = zeros((n+1)*m,1);
    for k = 1:m
        fk = zeros(n+1,1);
        C1 = (Waypoints(k+1,d)'-Waypoints(k,d)')/(times(k+1)-times(k));
        C0 = Waypoints(k,d)' - C1*times(k);
        for i = 1:n+1
            fk(i) = -2*(C1/(n+3-i)*(times(k+1)^(n+3-i)-times(k)^(n+3-i)) + C0/(n+2-i)*(times(k+1)^(n+2-i)-times(k)^(n+2-i)));
        end
        f((k-1)*(n+1)+1:k*(n+1)) = fk;
    end
    f = f.*kron(mu(:,1),ones(n+1,1));

    %% Create Equality Constraints Matrices
    A = zeros(kc*(m+1)+2*m,(n+1)*m);
    b = zeros(size(A,1),1);
    % Position
    ind = (n:-1:0);
    A0 = zeros(2*m,(n+1)*m);
    for j = 1:m
        up = times(j).^ind;
        lo = times(j+1).^ind;
        A0((j-1)*2+1:j*2,(j-1)*(n+1)+1:j*(n+1)) = [up; lo];
    end
    A(1:2*m,:) = A0;
    b0 = zeros(2*m,1);
    b0(1) = Waypoints(1,d)';
    b0(end) = Waypoints(end,d)';
    for j = 2:m
        b0((j-2)*2+2:(j-1)*2+1) = [Waypoints(j,d); Waypoints(j,d)];
    end
    b(1:2*m) = b0;
    % Derivatives of Position
    for k = 1:kc
        ind = (n:-1:k);
        factors = new_fact(ind+1)./new_fact(ind-k+1);
        % factors = factorial(ind)./factorial(ind-k);
        Ak = zeros(m+1,(n+1)*m);
        for j = 1:m
            if (j > 1)
                up = -(times(j).^(n-k:-1:0)).*factors;
            else
                up = (times(j).^(n-k:-1:0)).*factors;
            end
            lo = (times(j+1).^(n-k:-1:0)).*factors;
            Ak(j:j+1,(j-1)*(n+1)+1:j*(n+1)-k) = [up; lo];
        end
        A(2*m+(k-1)*(m+1)+1:2*m+k*(m+1),:) = Ak;
        %%%
        bk = zeros(m+1,1);
        if (k <= 2)
            bk(1) = start_velaccel(d,k);
            bk(end) = end_velaccel(d,k);
        end
        b(2*m+(k-1)*(m+1)+1:2*m+k*(m+1)) = bk;
    end
    if (kc < 2)
        % acceleration
        A2 = zeros(2,(n+1)*m);
        ind = (n:-1:2);
        factors = new_fact(ind+1)./new_fact(ind-2+1);
        % factors = factorial(ind)./factorial(ind-2);
        A2(1,1:n-1) = (times(1).^(n-2:-1:0)).*factors;
        A2(2,end-n:end-2) = (times(end).^(n-2:-1:0)).*factors;
        A = [A; A2];
        b2 = zeros(2,1);
        b2(1) = start_velaccel(d,2);
        b2(2) = end_velaccel(d,2);
        b = [b; b2];
        if (kc < 1)
            % velocity
            A1 = zeros(2,(n+1)*m);
            factors = (n:-1:1);
            A1(1,1:n) = (times(1).^(n-1:-1:0)).*factors;
            A1(2,end-n:end-1) = (times(end).^(n-1:-1:0)).*factors;
            A = [A; A1];
            b1 = zeros(2,1);
            b1(1) = start_velaccel(d,1);
            b1(2) = end_velaccel(d,1);
            b = [b; b1];
        end
    end
    %% Solve Quadratic Program
    dir_sol = [2*H A'; A zeros(size(A,1))]\[-f; b];
    c_norm = dir_sol(1:m*(n+1));

    % c_norm = quadprog(2*H,f,[],[],A,b,[],[],[],optimset('LargeScale','off','Display','off'));

    %% Scale Parameters to actual times and space
    av = alpha.^-(n:-1:0)';
    c = c_norm.*repmat(av,m,1);
    c = c*beta2(d);
    c(n+1:n+1:end) = c(n+1:n+1:end) + beta1(d)*ones(m,1);

    param(d,:) = c';
end

%% Get Trajectory Parameter Matrix from c
traj_param = zeros(d*m,n+1);
for j = 1:m
    traj_param((j-1)*d+1:j*d,:) = param(:,(j-1)*(n+1)+1:j*(n+1));
end

end

