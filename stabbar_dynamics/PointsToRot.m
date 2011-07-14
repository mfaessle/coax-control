function [Rhat,That] = PointsToRot(m,d)

%m-(3,N) are the body frame coordinates
%d-(3,N) are the world frame coordinates
%d = R*m + T

N = size(m,2);
dbar = mean(d,2);
mbar = mean(m,2);

dc = d - repmat(dbar,1,N);
mc = m - repmat(mbar,1,N);

%H = sum(mc*dc')
H = zeros(3,3);
for i=1:N;
    H = H + mc(:,i)*dc(:,i)';
end

%H = U*S*V'
[U,S,V] = svd(H);

Rhat = V*U';
That = dbar - Rhat*mbar;