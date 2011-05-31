pid = nav_msgs_Odometry('connect','subscriber','odom56','odom56');
iid = geometry_msgs_Quaternion('connect','subscriber','coax_info56','coax_info56');
tid = geometry_msgs_Quaternion('connect','publisher','trim56','trim56');
mid = std_msgs_Bool('connect','publisher','nav_mode56','nav_mode56');
cid = geometry_msgs_Quaternion('connect','publisher','raw_control56','raw_control56');
cmid = geometry_msgs_Quaternion('connect','subscriber','control_mode56','control_mode56');
    
nav_mode = std_msgs_Bool('empty');
nav_mode.data = 1;
std_msgs_Bool('send',mid,nav_mode); % switch to NAV_RAW_MODE

raw_control = geometry_msgs_Quaternion('empty');

omega = pi/2;
t0 = clock;
i = 0;
while i<1000
    tic
    
    dt = etime(clock, t0);
    if (dt < 2)
        motor_up = 0.4;
        motor_lo = 0.4;
    else
        motor_up = 0.4;
        motor_lo = 0.4;
    end
    
    servo1 = 0;
    
    raw_control.x = motor_up;
    raw_control.y = motor_lo;
    raw_control.z = servo1;
    raw_control.w = 0;
    geometry_msgs_Quaternion('send',cid,raw_control);
    
    i = i+1;
    if (toc < 0.0094);
        while (toc < 0.0094);
        end
    end
end

raw_control.x = 0;
raw_control.y = 0;
raw_control.z = 0;
raw_control.w = 0;
geometry_msgs_Quaternion('send',cid,raw_control);

nav_msgs_Odometry('disconnect',pid);
geometry_msgs_Quaternion('disconnect',iid);
geometry_msgs_Quaternion('disconnect',tid);
std_msgs_Bool('disconnect',mid);
geometry_msgs_Quaternion('disconnect',cid);
geometry_msgs_Quaternion('disconnect',cmid);
clear