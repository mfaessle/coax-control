load ViconData_SB

Time = Data.time;
SB = Data.bar';
Body = Data.bodyz';
N = length(Time);
    
timestep = 0.005;
tracelength = 20;

figureHandle = figure('NumberTitle','off','Name','Stabilizer Bar Data');
axesHandle = axes('Parent',figureHandle);

plotHandleBody = plot3(axesHandle,[0 Body(1,1)],[0 Body(1,2)],[0 Body(1,3)],'blue');
hold on;
plotHandleSB = plot3(axesHandle,[0 SB(1,1)],[0 SB(1,2)],[0 SB(1,3)],'red');
plotHandleTrace = plot3(axesHandle,SB(1,1),SB(1,2),SB(1,3),'--');
xlabel('x');
ylabel('y');
zlabel('z');
title('Play Stabilizer Bar Data');
grid on;
%axis([-0.05 0.05 -0.05 0.05 0 1])
axis([min(Body(:,1)) max(Body(:,1)) min(Body(:,2)) max(Body(:,2)) 0 1])
legend('Body-z','Bar')

for i = 1:N
    set(plotHandleBody,'XData',[0 Body(i,1)],'YData',[0 Body(i,2)],'ZData',[0 Body(i,3)]);
    set(plotHandleSB,'XData',[0 SB(i,1)],'YData',[0 SB(i,2)],'ZData',[0 SB(i,3)]);
    if (i > tracelength)
        set(plotHandleTrace,'XData',SB(i-tracelength:i,1),'YData',SB(i-tracelength:i,2),'ZData',SB(i-tracelength:i,3));
    else
        set(plotHandleTrace,'XData',SB(1:i,1),'YData',SB(1:i,2),'ZData',SB(1:i,3));
    end
    set(figureHandle,'Visible','on');
    
    %pause(timestep);
end