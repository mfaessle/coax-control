function [Polynomial] = polynomial_config()
%Polynomial = polynomial_config() returns a Polynomial object that contains
%the parameters about how the polynomial trajectory should be built.
%
%   n_poly is the order of the polynomials. The integrated and weighted
%   square norm of the 0th up to the kr-th derivative of position are
%   minimized. Up to the kc-th derivative of position is forced to be
%   continuous. start_velaccel and end_velaccel have the desired velocities
%   and accelerations at the start and the beginning of the trajectory in
%   its columns.

% Matthias Fässler

n_poly = 7;
kr = 2;
weights = [5000 100 1];
kc = 2;
start_velaccel = zeros(3,2);
end_velaccel = zeros(3,2);

Polynomial.n_poly = n_poly;
Polynomial.kr = kr;
Polynomial.weights = weights;
Polynomial.kc = kc;
Polynomial.start_velaccel = start_velaccel;
Polynomial.end_velaccel = end_velaccel;

end

