load ViconData_plexiglass

time = Data.time;
position = Data.position;
velocity = Data.lintwist;
rates = Data.angtwist;

figure(1)

subplot(3,1,1)
plot(time,position(1,:));
subplot(3,1,2)
plot(time,position(2,:));
subplot(3,1,3)
plot(time,position(3,:));

figure(2)

subplot(3,1,1)
plot(time,velocity(1,:));
subplot(3,1,2)
plot(time,velocity(2,:));
subplot(3,1,3)
plot(time,velocity(3,:));

figure(3)

subplot(3,1,1)
plot(time,rates(1,:));
subplot(3,1,2)
plot(time,rates(2,:));
subplot(3,1,3)
plot(time,rates(3,:));

max(position,[],2) - min(position,[],2)
max(velocity,[],2) - min(velocity,[],2)
max(rates,[],2) - min(rates,[],2)