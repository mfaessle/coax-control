m1id = geometry_msgs_Quaternion('connect','subscriber','marker1','marker1');
m2id = geometry_msgs_Quaternion('connect','subscriber','marker2','marker2');
m3id = geometry_msgs_Quaternion('connect','subscriber','marker3','marker3');
m4id = geometry_msgs_Quaternion('connect','subscriber','marker4','marker4');
m5id = geometry_msgs_Quaternion('connect','subscriber','marker5','marker5');
sbid = geometry_msgs_Quaternion('connect','subscriber','stabbar','stabbar');
iid = geometry_msgs_Quaternion('connect','subscriber','coax_info56','coax_info56');
tid = geometry_msgs_Quaternion('connect','publisher','trim56','trim56');
mid = std_msgs_Bool('connect','publisher','nav_mode56','nav_mode56');
cid = geometry_msgs_Quaternion('connect','publisher','raw_control56','raw_control56');
cmid = geometry_msgs_Quaternion('connect','subscriber','control_mode56','control_mode56');

nav_mode = std_msgs_Bool('empty');
nav_mode.data = 1;
std_msgs_Bool('send',mid,nav_mode); % switch to NAV_RAW_MODE

raw_control = geometry_msgs_Quaternion('empty');

volt_compUp = 0;
volt_compLo = 0;
Mbody = [-33.9434   33.3300   33.3245  -20.4397   24.9739; ...
         -59.0677  -60.2362   60.9161   20.7246  -17.8216; ...
         -44.4484  -44.8094  -44.9953   33.4755   34.3630];  % markers in body frame (without stabilizer bar)

N = 1000;

SBOrientation = zeros(N,3);
time = zeros(N,1);
t0 = clock;
i = 1;
tic
while 1
    
    dt = etime(clock, t0);
    if (dt < 2)
        motor_up = 0.35;
        motor_lo = 0.35;
    else
        motor_up = 0.35;
        motor_lo = 0.35;
    end
    
    % Voltage compensation
    info = geometry_msgs_Quaternion('read',iid,1);
    if (~isempty(info))
        v_bat = round(100*info.y)/100;
        volt_compUp = (12.22 - v_bat)*0.0279;
        volt_compLo = (12.22 - v_bat)*0.0287;
    end
    motor_up = motor_up + volt_compUp;
    motor_lo = motor_lo + volt_compLo;
    
    
    raw_control.x = motor_up;
    raw_control.y = motor_lo;
    raw_control.z = 0.0285;
    raw_control.w = 0.0921;
    geometry_msgs_Quaternion('send',cid,raw_control);
    
    % read marker positions
    [M1,M2,M3,M4,M5,SB] = read_marker_pos(m1id,m2id,m3id,m4id,m5id,sbid);

    M = [M1 M2 M3 M4 M5 SB];

    % check visibility
    mw = [];
    mb = [];
    for j=1 : size(Mbody,2)
        if (M(4,j) > 0.5)
            mw = [mw M(1:3,j)];
            mb = [mb Mbody(:,j)];
        end
    end

    % calculate transformation 
    [Rhat,That] = PointsToRot(mb,mw);
    
    % calculate stabilizer bar orientation
    if (SB(4) > 0.5)
        hinge_body = [0 0 180]';
        sb = SB(1:3);
        sb_body = Rhat'*(sb - That);

        z_SB = sb_body - hinge_body;
        z_SB = z_SB/norm(z_SB); % stabilizer bar orientation in body coordinates
    else
        z_SB = [0 0 0]';
    end

    pause(0.005);
    if (dt > 5)
        SBOrientation(i,:) = z_SB';
        time(i) = toc;
        i = i+1;
    end
    if (i > N)
        break;
    end
end
time = time - time(1);

% z_SB
% Rhat
% z_SB

SBDynamics.time = time;
SBDynamics.SBOrientation = SBOrientation;

save SBDynamics SBDynamics

geometry_msgs_Quaternion('disconnect',m1id);
geometry_msgs_Quaternion('disconnect',m2id);
geometry_msgs_Quaternion('disconnect',m3id);
geometry_msgs_Quaternion('disconnect',m4id);
geometry_msgs_Quaternion('disconnect',m5id);
geometry_msgs_Quaternion('disconnect',sbid);
geometry_msgs_Quaternion('disconnect',iid);
geometry_msgs_Quaternion('disconnect',tid);
std_msgs_Bool('disconnect',mid);
geometry_msgs_Quaternion('disconnect',cid);
geometry_msgs_Quaternion('disconnect',cmid);
clear
clc