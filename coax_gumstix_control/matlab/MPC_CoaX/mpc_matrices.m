A = diag([1 1 1],3);
B = [zeros(p); 1/m*eye(p)];
C = eye(n);
D = zeros(n,p);

sysd = c2d(ss(A,B,C,D),Ts_highlevel,'zoh');
Ad = sysd.A;
Bd = sysd.B;

Q = diag([1 1 10 2 2 10]);
R = eye(p);

[~,P]   = dlqr(Ad,Bd,Q,R);

Q_tilde = blkdiag(kron(eye(N-1),Q),P);
R_tilde = kron(eye(N),R);
A_tilde = zeros(N*n,n);
for i=1:N
    A_tilde((i-1)*n+1:(i*n),:) = Ad^i;   
end
B_tilde = [];
for i=1:N
    temp = zeros(n,N*p);
    for j=1:i
        temp(:,(j-1)*p+1:j*p) = Ad^(i-j)*Bd;
    end
    B_tilde = [B_tilde; temp];
end

F = A_tilde'*Q_tilde*B_tilde;
H = B_tilde'*Q_tilde*B_tilde + R_tilde;
H = (H+H')/2; %Just to prevent Warning from Matlab

% max force constraints
F_max = [0.1 0.1 2]';
F_min = [-0.1 -0.1 -m*g]';
A_ineq_f = [eye(p*N); -eye(p*N)];
b_ineq_f = [repmat(F_max,N,1); repmat(-F_min,N,1)];

% max change of force constraints
dF_max = 0.01*[1 1 1]';
A_ineq_df_up = diag(ones((N-1)*p,1),3) - eye(N*p);
A_ineq_df_lo = -diag(ones((N-1)*p,1),3) + eye(N*p);
A_ineq_df = [A_ineq_df_up(1:(N-1)*p,:); A_ineq_df_lo(1:(N-1)*p,:)];
b_ineq_df = [repmat(dF_max,N-1,1); repmat(dF_max,N-1,1)];

A_ineq = [A_ineq_f; A_ineq_df];
b_ineq = [b_ineq_f; b_ineq_df];

% cont_param.F = F;
% cont_param.H = H;
% cont_param.A_ineq = A_ineq;
% cont_param.b_ineq = b_ineq;