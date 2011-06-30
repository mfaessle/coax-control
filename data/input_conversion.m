%input   weight
C = [0       510;
0.45    280;
0.5     225;
0.55    175]

figure(1)
plot(C(:,1),C(:,2),'red')
hold on;
line(x([1 3]),y([1 3]))