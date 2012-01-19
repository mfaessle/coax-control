%% Load Data
load ViconData_coaxwalltilted
N_start = 3000;
N_stop = 12000;

time = Data.time(N_start:N_stop);

pos_des = Data.trajectory(1:3,N_start:N_stop);

pos_sheet = Data.pos_sheet(:,N_start:N_stop);
ori_sheet = Data.ori_sheet(:,N_start:N_stop);

pos_coax = Data.position(:,N_start:N_stop);
ori_coax = Data.orientation(:,N_start:N_stop);
inputs = Data.inputs(:,N_start:N_stop);

n = length(time);

%% Compute body axes
x_sheet = zeros(3,n);
y_sheet = zeros(3,n);
z_sheet = zeros(3,n);
x_coax = zeros(3,n);
y_coax = zeros(3,n);
z_coax = zeros(3,n);

horz_normal = zeros(3,n);
horz_dist = zeros(n,1);
horz_error = zeros(3,n);
horz_norm_error = zeros(n,1);
horz_antinorm_error = zeros(n,1);

for i = 1:n
    % sheet frame
    qx = ori_sheet(1,i);
    qy = ori_sheet(2,i);
    qz = ori_sheet(3,i);
    qw = ori_sheet(4,i);
    x_sheet(:,i) = [1-2*qy^2-2*qz^2 2*qx*qy+2*qz*qw 2*qx*qz-2*qy*qw]';
    y_sheet(:,i) = [2*qx*qy-2*qz*qw 1-2*qx^2-2*qz^2 2*qy*qz+2*qx*qw]';
    z_sheet(:,i) = [2*qx*qz+2*qy*qw 2*qy*qz-2*qx*qw 1-2*qx^2-2*qy^2]';
    % coax frame
    qx = ori_coax(1,i);
    qy = ori_coax(2,i);
    qz = ori_coax(3,i);
    qw = ori_coax(4,i);
    x_coax(:,i) = [1-2*qy^2-2*qz^2 2*qx*qy+2*qz*qw 2*qx*qz-2*qy*qw]';
    y_coax(:,i) = [2*qx*qy-2*qz*qw 1-2*qx^2-2*qz^2 2*qy*qz+2*qx*qw]';
    z_coax(:,i) = [2*qx*qz+2*qy*qw 2*qy*qz-2*qx*qw 1-2*qx^2-2*qy^2]';
    % errors and projections
    temp = [z_sheet(1:2,i)' 0]';
    horz_normal(:,i) = temp/norm(temp);
    
    horz_dist(i) = abs([pos_coax(1,i)-pos_sheet(1,i) pos_coax(2,i)-pos_sheet(2,i) 0]*horz_normal(:,i));
    
    horz_error(:,i) = [pos_coax(1,i)-pos_des(1,i) pos_coax(2,i)-pos_des(2,i) 0];
    horz_norm_error(i) = horz_error(:,i)'*horz_normal(:,i);
    horz_antinorm_error(i) = norm(horz_error(:,i)-horz_norm_error(i)*horz_normal(:,i));
end

horz_dist = medfilt2(horz_dist,[5 1]);

%% Plots
figure(1)
[~,mind] = min(horz_dist);
plot(horz_dist(1:mind),horz_norm_error(1:mind),'.')
hold on;
plot(horz_dist(mind+1:end),horz_norm_error(mind+1:end),'r.')
hold off;
legend('Moving towards Coax','Moving away from Coax')
xlabel('Horizontal Distance Wall-Coax [m]')
ylabel('Horizontal Error normal to wall [m]')

figure(2)
[~,mind] = min(horz_dist);
plot(horz_dist(1:mind),horz_antinorm_error(1:mind),'.')
hold on;
plot(horz_dist(mind+1:end),horz_antinorm_error(mind+1:end),'r.')
hold off;
legend('Moving towards Coax','Moving away from Coax')
xlabel('Horizontal Distance Wall-Coax [m]')
ylabel('Horizontal Error parallel to wall [m]')

figure(3)
[~,mind] = min(horz_dist);
plot(horz_dist(1:mind),pos_coax(3,1:mind)-pos_des(3,1:mind),'.')
hold on;
plot(horz_dist(mind+1:end),pos_coax(3,mind+1:end)-pos_des(3,mind+1:end),'r.')
hold off;
legend('Moving towards Coax','Moving away from Coax')
xlabel('Horizontal Distance Wall-Coax [m]')
ylabel('Vertical Error from desired position [m]')

figure(4)
[~,mind] = min(horz_dist);
subplot(2,1,1)
plot(horz_dist(1:mind),inputs(1:2,1:mind),'.')
hold on;
plot(horz_dist(mind+1:end),inputs(1:2,mind+1:end),'r.')
hold off;
grid on;
legend('Motor up','Motor lo','Moving towards Coax','Moving away from Coax')
xlabel('Horizontal Distance Wall-Coax [m]')
ylabel('Motor Inputs [-]')

subplot(2,1,2)
plot(horz_dist(1:mind),inputs(3:4,1:mind),'.')
hold on;
%plot(horz_dist(mind+1:end),inputs(3:4,mind+1:end),'r.')
hold off;
grid on;
legend('Servo roll','Servo pitch','Moving towards Coax','Moving away from Coax')
xlabel('Horizontal Distance Wall-Coax [m]')
ylabel('Servo Inputs [-]')



