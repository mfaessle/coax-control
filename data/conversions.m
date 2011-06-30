load in2rpm
load volt2rpm

%% Input 2 Voltage conversion

linIn2rpmUp = polyfit(in2rpm.input(3:10),in2rpm.rpmUp(3:10),1);
linIn2rpmLo = polyfit(in2rpm.input(3:10),in2rpm.rpmLo(3:10),1);

input = linspace(min(in2rpm.input),max(in2rpm.input),100);
linRpmUp = linIn2rpmUp(1)*input + linIn2rpmUp(2);
linRpmLo = linIn2rpmLo(1)*input + linIn2rpmLo(2);

figure(1)
plot(in2rpm.input,in2rpm.rpmUp,'*')
hold on;
plot(input,linRpmUp)
plot(in2rpm.input,in2rpm.rpmLo,'*red')
plot(input,linRpmLo,'red')
hold off;
grid on;
title('Input to RPM Conversion')
xlabel('Input [-]')
ylabel('Rotor speed [rpm]')
legend('Upper rotor meas','Upper rotor linear','Lower rotor meas','Lower rotor linear',2)

%% Voltage 2 RPM conversion

linVolt2rpmUp = polyfit(volt2rpm.battState,volt2rpm.rpmUp,1);
linVolt2rpmLo = polyfit(volt2rpm.battState,volt2rpm.rpmLo,1);


battState = linspace(min(volt2rpm.battState),max(volt2rpm.battState),100);
linRpmUp = linVolt2rpmUp(1)*battState + linVolt2rpmUp(2);
linRpmLo = linVolt2rpmLo(1)*battState + linVolt2rpmLo(2);

figure(2)
plot(volt2rpm.battState,volt2rpm.rpmUp,'*')
hold on;
plot(battState,linRpmUp)
plot(volt2rpm.battState,volt2rpm.rpmLo,'*red')
plot(battState,linRpmLo,'red')
hold off;
grid on;
title('Battery Voltage to RPM Conversion')
xlabel('Battery State [V]')
ylabel('Rotor speed [rpm]')
legend('Upper rotor meas @(u=0.511)','Upper rotor linear','Lower rotor meas @(u=0.541)','Lower rotor linear',2)



voltCompFact = (linVolt2rpmUp(1) + linVolt2rpmLo(1))/(linIn2rpmUp(1) + linIn2rpmLo(1));

% apply voltage compensation
% du = (volt2rpm.battState(1) - currentBattState) * voltCompFact
% u_comp = u + du 

%% Battery State Voltage Conversion

linVoltConv = polyfit(volt2rpm.battState,volt2rpm.measVoltage,1);
battState = linspace(min(volt2rpm.battState),max(volt2rpm.battState),100);
linVolt = linVoltConv(1)*battState + linVoltConv(2);

figure(3)
plot(volt2rpm.battState,volt2rpm.measVoltage,'*')
hold on;
plot(battState,linVolt)
hold off;
grid on;
title('Battery State to true Voltage Conversion')
xlabel('Battery State [V]')
ylabel('True Voltage [V]')
legend('Voltage measured','Linear Conversion',2)


