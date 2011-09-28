function [error_sort] = errorprobplot(states,trajectory,ploting)

%   states: Matrix with single state measurements in its rows
%   trajectory: Corresponding target states
%   ploting: 1 = draw plot, 2 = just return sorted error

N = size(states,1);
error = zeros(1,N);

for i = 1:N
    error(i) = norm(states(i,:)-trajectory(i,:));    
end

error_sort = sort(error);

x = linspace(0,100,N)';

if (ploting)
    figure;
    plot(x,error_sort)
    grid on;
    xlabel('Percentage of Time')
    ylabel('Error Norm')
end

end

