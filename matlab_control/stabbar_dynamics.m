run constants
prev_z_bar = [0.1 0 sqrt(1-0.1^2)]';
dt = 0.01;

i = 0;
figure(1)
hold on;
while i<50
    b_z_bardotz = 1/Tf_up*acos(prev_z_bar(3))*sqrt(prev_z_bar(1)^2 + prev_z_bar(2)^2);
    if (b_z_bardotz <= 0)
        b_z_bardot = [0 0 0]';
    else
        temp = prev_z_bar(3)*b_z_bardotz/(prev_z_bar(1)^2+prev_z_bar(2)^2);
        b_z_bardot = [-prev_z_bar(1)*temp -prev_z_bar(2)*temp b_z_bardotz]';
    end

    %A_k = [0 r -q; -r 0 p; q -p 0];
    A_k = zeros(3);

    z_bar = prev_z_bar + (A_k*prev_z_bar + b_z_bardot)*dt;
    z_bar = z_bar/norm(z_bar);
    plot(prev_z_bar(1),prev_z_bar(3),'*');
    fprintf('%f  %f   %f \n',i*dt,prev_z_bar(1),prev_z_bar(3));
    prev_z_bar = z_bar;
    
    i = i+1;
    pause(0.1)
end