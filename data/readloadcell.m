%% Initialize
instrreset;

s1 = serial('/dev/tty.usbmodem471','Timeout',1);
fopen(s1);

cmd = 'o0w1';
% cr = char(13);
% lf = char(10);
% crlf = strcat(cr,lf);
% cmd = strcat(cmd,crlf);

%% Calibrate

N = 5000;
data = zeros(N,1);
fread(s1,1,'char');
for i=1 : N
    fprintf(s1, cmd);
    msg = fscanf(s1,'%d');
    data(i) = msg;
    fread(s1,1,'char');
end
mean(data)

a = 0.004434551453279;
b = -17.100811494816192;

%% Zero

N = 1000;
data = zeros(N,1);
for i=1 : N
    fprintf(s1, cmd);
    data(i) = fscanf(s1,'%d');
    fread(s1,1,'char');
end
% take mean
b = mean(data);


%% Read

N = 2000;
force = zeros(N,1);
for i=1 : N
    fprintf(s1, cmd);
    force(i) = (fscanf(s1,'%d'))*a + b;
    fread(s1,1,'char');
    %fprintf('%f \n',force(i))
end
mean(force)
