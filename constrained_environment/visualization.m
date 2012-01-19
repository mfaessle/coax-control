%% Load Data
load ViconData_coaxwallb2
N_start = 3000;
N_stop = 15000;

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
end

%% Visualization

fastfwd = 4; % fast forward factor

Ls = 1;
Ws = 0.6;
La = 0.2;

x_sv = pos_sheet(1,1)+0.5*[Ls*x_sheet(1,1)+Ws*y_sheet(1,1) Ls*x_sheet(1,1)-Ws*y_sheet(1,1) -Ls*x_sheet(1,1)-Ws*y_sheet(1,1) -Ls*x_sheet(1,1)+Ws*y_sheet(1,1)]';
y_sv = pos_sheet(2,1)+0.5*[Ls*x_sheet(2,1)+Ws*y_sheet(2,1) Ls*x_sheet(2,1)-Ws*y_sheet(2,1) -Ls*x_sheet(2,1)-Ws*y_sheet(2,1) -Ls*x_sheet(2,1)+Ws*y_sheet(2,1)]';
z_sv = pos_sheet(3,1)+0.5*[Ls*x_sheet(3,1)+Ws*y_sheet(3,1) Ls*x_sheet(3,1)-Ws*y_sheet(3,1) -Ls*x_sheet(3,1)-Ws*y_sheet(3,1) -Ls*x_sheet(3,1)+Ws*y_sheet(3,1)]';

figureHandle = figure(1);
axesHandle = axes('Parent',figureHandle);

plotHandleCoax_x = plot3(axesHandle,pos_coax(1,1)+[0 La*x_coax(1,1)],pos_coax(2,1)+[0 La*x_coax(2,1)],pos_coax(3,1)+[0 La*x_coax(3,1)],'red');
hold on;
plotHandleCoax_y = plot3(axesHandle,pos_coax(1,1)+[0 La*y_coax(1,1)],pos_coax(2,1)+[0 La*y_coax(2,1)],pos_coax(3,1)+[0 La*y_coax(3,1)],'blue');
plotHandleCoax_z = plot3(axesHandle,pos_coax(1,1)+[0 La*z_coax(1,1)],pos_coax(2,1)+[0 La*z_coax(2,1)],pos_coax(3,1)+[0 La*z_coax(3,1)],'blue');

plotHandleSheet = fill3(x_sv,y_sv,z_sv,0.8*[1 1 1]);
plotHandleSheet_z = plot3(axesHandle,pos_sheet(1,1)+[0 La*z_sheet(1,1)],pos_sheet(2,1)+[0 La*z_sheet(2,1)],pos_sheet(3,1)+[0 La*z_sheet(3,1)],'red');

plotHandleDesPos = plot3(axesHandle,pos_des(1,1),pos_des(2,1),pos_des(3,1),'.green','MarkerSize',5);

xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title({'Wall Proximity Experiment',horzcat('Time: ',num2str(time(1)))});
grid on;
axis([min(pos_des(1,:))-1 max(pos_des(1,:))+1 min(pos_des(2,:))-1 max(pos_des(2,:))+1 min(pos_des(3,:))-1 max(pos_des(3,:))+1])

for i = 1:fastfwd:n
    set(plotHandleCoax_x,'XData',pos_coax(1,i)+[0 La*x_coax(1,i)],'YData',pos_coax(2,i)+[0 La*x_coax(2,i)],'ZData',pos_coax(3,i)+[0 La*x_coax(3,i)]);
    set(plotHandleCoax_y,'XData',pos_coax(1,i)+[0 La*y_coax(1,i)],'YData',pos_coax(2,i)+[0 La*y_coax(2,i)],'ZData',pos_coax(3,i)+[0 La*y_coax(3,i)]);
    set(plotHandleCoax_z,'XData',pos_coax(1,i)+[0 La*z_coax(1,i)],'YData',pos_coax(2,i)+[0 La*z_coax(2,i)],'ZData',pos_coax(3,i)+[0 La*z_coax(3,i)]);
    
    x_sv = pos_sheet(1,i)+0.5*[Ls*x_sheet(1,i)+Ws*y_sheet(1,i) Ls*x_sheet(1,i)-Ws*y_sheet(1,i) -Ls*x_sheet(1,i)-Ws*y_sheet(1,i) -Ls*x_sheet(1,i)+Ws*y_sheet(1,i)]';
    y_sv = pos_sheet(2,i)+0.5*[Ls*x_sheet(2,i)+Ws*y_sheet(2,i) Ls*x_sheet(2,i)-Ws*y_sheet(2,i) -Ls*x_sheet(2,i)-Ws*y_sheet(2,i) -Ls*x_sheet(2,i)+Ws*y_sheet(2,i)]';
    z_sv = pos_sheet(3,i)+0.5*[Ls*x_sheet(3,i)+Ws*y_sheet(3,i) Ls*x_sheet(3,i)-Ws*y_sheet(3,i) -Ls*x_sheet(3,i)-Ws*y_sheet(3,i) -Ls*x_sheet(3,i)+Ws*y_sheet(3,i)]';

    set(plotHandleSheet,'XData',x_sv,'YData',y_sv,'ZData',z_sv);
    set(plotHandleSheet_z,'XData',pos_sheet(1,i)+[0 La*z_sheet(1,i)],'YData',pos_sheet(2,i)+[0 La*z_sheet(2,i)],'ZData',pos_sheet(3,i)+[0 La*z_sheet(3,i)]);
    
    set(plotHandleDesPos,'XData',pos_des(1,i),'YData',pos_des(2,i),'ZData',pos_des(3,i));
    
    title({'Wall Proximity Experiment',horzcat('Time: ',num2str(time(i)))});
    
    set(figureHandle,'Visible','on');
end
hold off;

