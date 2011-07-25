load SBDynamics

SB_ori = SBDynamics.SBOrientation;
time = SBDynamics.time;

zero_ori = [-0.0509 -0.0108 0.9985]';

Pxy = SB_ori(:,1:2);

H = cov(Pxy);
[V,D] = eig(H);
[~,i] = max(max(D));
v = V(:,i);
v = v/norm(v); % direction with maximum variance

Pplane = (Pxy*v)*v';

alpha = zeros(size(SB_ori,1),1);
for i=1 : length(alpha)
    z_SB = [Pxy(i,:) SB_ori(i,3)]';
    z_SB = z_SB/norm(z_SB);
    
    alpha(i) = sign(Pxy(i,:)*v)*acos([0 0 1]*z_SB);
end

figure(1)
plot(SB_ori(:,1),SB_ori(:,2),'*red')
hold on;
line(0.1*[0 v(1)],0.1*[0 v(2)])
plot(Pplane(:,1),Pplane(:,2),'*green')
grid on;
hold off;

figure(2)
plot(time,alpha)
grid on;


Mbody = [-33 34 35 -22 25; ...
         -58 -60 61 20 -18; ...
         -41 -41 -41 39 39];  % markers in body frame (without stabilizer bar)
hinge_body = [0 0 180]';
L = 50; % length of part that carries stabbar marker

figure(3)
plot3(Mbody(1,:),Mbody(2,:),Mbody(3,:),'.k','MarkerSize',10)
hold on;
plot3(0,0,0,'.g','MarkerSize',20)
plot3([0 hinge_body(1)],[0 hinge_body(2)],[0 hinge_body(3)])
plot3(hinge_body(1),hinge_body(2),hinge_body(3),'.b','MarkerSize',10)
plot3(hinge_body(1)+[0 L*z_SB(1)],hinge_body(2)+[0 L*z_SB(2)],hinge_body(3)+[0 L*z_SB(3)],'r')
plot3(hinge_body(1)+L*z_SB(1),hinge_body(2)+L*z_SB(2),hinge_body(3)+L*z_SB(3),'.k','MarkerSize',10)
hold off;
xlabel('x [mm]')
ylabel('y [mm]')
zlabel('z [mm]')
axis equal;
grid on;

