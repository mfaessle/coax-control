%% Initialization
COAX56 = 1;
if (COAX56)
    pid = nav_msgs_Odometry('connect','subscriber','odom56','odom56');
    iid = geometry_msgs_Quaternion('connect','subscriber','coax_info56','coax_info56');
    tid = geometry_msgs_Quaternion('connect','publisher','trim56','trim56');
    mid = std_msgs_Bool('connect','publisher','nav_mode56','nav_mode56');
    cid = geometry_msgs_Quaternion('connect','publisher','raw_control56','raw_control56');
    cmid = geometry_msgs_Quaternion('connect','subscriber','control_mode56','control_mode56');
else
    pid = nav_msgs_Odometry('connect','subscriber','odom57','odom57');
    iid = geometry_msgs_Quaternion('connect','subscriber','coax_info57','coax_info57');
    tid = geometry_msgs_Quaternion('connect','publisher','trim57','trim57');
    mid = std_msgs_Bool('connect','publisher','nav_mode57','nav_mode57');
    cid = geometry_msgs_Quaternion('connect','publisher','raw_control57','raw_control57');
    cmid = geometry_msgs_Quaternion('connect','subscriber','control_mode57','control_mode57');
end

% General Constants
CONTROL_LANDED = 0; % State when helicopter has landed successfully
CONTROL_START = 1; % Start / Takeoff
CONTROL_TRIM = 2; % Trim Servos
CONTROL_HOVER = 3; % Hover
CONTROL_GOTOPOS = 4; % Go to Position
CONTROL_TRAJECTORY = 5; % Follow Trajectory
CONTROL_LANDING = 6; % Landing maneuver

TRAJECTORY_SPIRAL = 0;
TRAJECTORY_ROTINPLACE = 1;
TRAJECTORY_VERTOSCIL = 2;
TRAJECTORY_LYINGCIRCLE = 3;
TRAJECTORY_STANDINGCIRCLE = 4;
TRAJECTORY_YAWOSCIL = 5;

% Start Maneuver
START_HEIGHT = 0.3;
RISE_VELOCITY = 0.05; % [m/s];
RISE_TIME = START_HEIGHT/RISE_VELOCITY;
IDLE_TIME = 3;

% Trimming
TRIM_DURATION = 20;
FIRST_TRIM = 0;
AUTO_TRIM = 0; % enable/disable trim after takeoff
servo_trim = zeros(2,1);

% Land Maneuver
SINK_VELOCITY = 0.05; % [m/s] positive!
SINK_TIME = START_HEIGHT/SINK_VELOCITY;

% Gotopos Maneuver
GOTOPOS_VELOCITY = 0.05;

% Filtering
pqr_filter = zeros(3,4);
vel_filter = zeros(3,4);
%filt_window = [0.66 0.34]';
filt_window = 1/4*ones(4,1);

% Initial Values
run parameter
run control_parameter
run normalize_linearize
CONTROL_MODE = CONTROL_LANDED;
e_i = [0 0 0 0]';
FIRST_HOVER = 0;
FIRST_LANDING = 0;
FIRST_TRAJECTORY = 0;
TRAJECTORY_TYPE = TRAJECTORY_LYINGCIRCLE;
SERVICE_LANDING = 0;
SERVICE_TRAJECTORY = 0;
LOW_POWER_DETECTED = 0;
mode = geometry_msgs_Quaternion('empty');
info = geometry_msgs_Quaternion('empty');
last_ctrl_mode_request = 0;
motor_up = 0;
motor_lo = 0;
prev_Omega_up = 0;
prev_Omega_lo = 0;
jump_count = 0;
FIRST_RUN = 1;
i = 1;
TimeStamps = [];
Positions = [];
Orientations = [];
Lintwists = [];
Angtwists = [];
Inputs = [];
Trims = [];
servo1 = 0;
servo2 = 0;
volt_compUp = 0;
volt_compLo = 0;

fprintf('Ready to Rock \n');

while (1)
    
%% Control Mode Transitions
mode = geometry_msgs_Quaternion('read',cmid,1);
if (~isempty(mode) && (int8(mode.x)~=last_ctrl_mode_request))
    % state transition
    last_ctrl_mode_request = int8(mode.x); % make loop only enter once if transition request occurs
    switch int8(mode.x)
        case 1 % start/takeoff
            if (CONTROL_MODE == CONTROL_LANDED)
                % make sure CoaX is in NAV_RAW_MODE
                % necessary to allow control inputs
                info = geometry_msgs_Quaternion('read',iid,1);
                while (isempty(info))
                    info = geometry_msgs_Quaternion('read',iid,1);
                end
                
                if ((0.89*info.y + 1.39) > 11)
                    if (info.x < 5)
                        nav_mode = std_msgs_Bool('empty');
                        nav_mode.data = 1;
                        std_msgs_Bool('send',mid,nav_mode); % switch to NAV_RAW_MODE
                    end
                    % set initial trim
                    trim = geometry_msgs_Quaternion('empty');
                    if (COAX56)
                        trim.x = 0.0285;
                        trim.y = 0.0921;
                    else
                        trim.x = 0;
                        trim.y = 0;
                    end
                    geometry_msgs_Quaternion('send',tid,trim);
                    % switch to start procedure
                    CONTROL_MODE = CONTROL_START;
                    FIRST_START = 1;
                else
                    if (~LOW_POWER_DETECTED)
                        fprintf('Battery Low!!! (%f V) Start denied \n',(0.89*info.y + 1.39));
                    end
                    LOW_POWER_DETECTED = 1;
                end
            end
        case 3 % hover
            if ((CONTROL_MODE == CONTROL_GOTOPOS) || (CONTROL_MODE == CONTROL_TRAJECTORY))
                CONTROL_MODE = CONTROL_HOVER;
                FIRST_HOVER = 1;
            end
        case 4 % go to position
            if (CONTROL_MODE == CONTROL_HOVER)
                % CONTROL_MODE = CONTROL_GOTOPOS;
            end
        case 5 % follow trajectory
            if ((CONTROL_MODE == CONTROL_HOVER) || (CONTROL_MODE == CONTROL_GOTOPOS))
                CONTROL_MODE = CONTROL_TRAJECTORY;
                FIRST_TRAJECTORY = 1;
            end
        case 6 % land
            if (CONTROL_MODE == CONTROL_HOVER)
                CONTROL_MODE = CONTROL_LANDING;
                FIRST_LANDING = 1;
            end
        case 7 % Lost Zigbee connection
            fprintf('Lost Zigbee connection for more than 0.5s \n'); 
        case 8 % failed to call reach_nav_state
            if (CONTROL_MODE == CONTROL_START)
                CONTROL_MODE = CONTROL_LANDED;
                FIRST_START = 0;
                fprintf('Not possible to switch to NAV_RAW_MODE \n');
                fprintf('Set RC to "autonomous" and Kill Switch to off \n');
                fprintf('Check if Zigbee connection is working \n');
            end
        case 9 % quit
            %if (CONTROL_MODE == CONTROL_LANDED)
                % switch to NAV_STOP_MODE
                raw_control = geometry_msgs_Quaternion('empty');
                geometry_msgs_Quaternion('send',cid,raw_control);
                
                nav_mode = std_msgs_Bool('empty');
                nav_mode.data = 0;
                std_msgs_Bool('send',mid,nav_mode);
                
                break;
            %end
    end
end

%% Low power and Communication loss detection
info = geometry_msgs_Quaternion('read',iid,1);
if (~isempty(info)) % if empty catch it the next time
    if (((0.89*info.y + 1.39) < 11) && ~LOW_POWER_DETECTED)
        LOW_POWER_DETECTED = 1;
        fprintf('Battery Low!!! (%fV) Landing initialized \n',0.89*info.y + 1.39);
    end
%     if ((int8(info.x) ~= 7) && (CONTROL_MODE ~= CONTROL_LANDED))
%         % Lost Zigbee connection
%         fprintf('Zigbee has lost its connection for too long %d \n', CONTROL_MODE);
%         CONTROL_MODE = CONTROL_LANDED;
%     end
end


%% Reading Vicon data and calculating state
% Read Vicon data
odom = nav_msgs_Odometry('read',pid,1);
while (isempty(odom))
    odom = nav_msgs_Odometry('read',pid,1);
end
x        = odom.pose.pose.position.x;
y        = odom.pose.pose.position.y;
z        = odom.pose.pose.position.z;
xdot     = odom.twist.twist.linear.x;
ydot     = odom.twist.twist.linear.y;
zdot     = odom.twist.twist.linear.z;
p        = odom.twist.twist.angular.x;
q        = odom.twist.twist.angular.y;
r        = odom.twist.twist.angular.z;
qx       = odom.pose.pose.orientation.x;
qy       = odom.pose.pose.orientation.y;
qz       = odom.pose.pose.orientation.z;
qw       = odom.pose.pose.orientation.w;
Rb2w     = [1-2*qy^2-2*qz^2 2*qx*qy-2*qz*qw 2*qx*qz+2*qy*qw; ...
            2*qx*qy+2*qz*qw 1-2*qx^2-2*qz^2 2*qy*qz-2*qx*qw; ...
            2*qx*qz-2*qy*qw 2*qy*qz+2*qx*qw 1-2*qx^2-2*qy^2];


%% Time
time = clock;
if (FIRST_RUN)
    prev_time = time;
end
dt = etime(time,prev_time);

%% Vicon position jump detection
position = [x y z]';
velocity = [xdot ydot zdot]';
rotmat = Rb2w;
bodyrates = [p q r]';
if (FIRST_RUN)
    prev_position = position;
    prev_velocity = velocity;
    prev_rotmat = rotmat;
    prev_bodyrates = bodyrates;
    prev_z_bar = [0 0 1]';
    FIRST_RUN = 0;
end

if (jump_count > 0)
    if (jump_count > 20)
        if (CONTROL_MODE ~= CONTROL_LANDED)
            CONTROL_MODE = CONTROL_HOVER;
            FIRST_HOVER = 1;
            fprintf('Vicon estimate has jumped to another position!!! \n');
        end
        jump_count = 0;
    elseif (norm(position - prev_position) > 0.1)
        jump_count = jump_count + 1;
        position = prev_position;
        rotmat = prev_rotmat;
        velocity = prev_velocity;
        bodyrates = prev_bodyrates;
    else
        jump_count = 0;
        velocity = prev_velocity;
        bodyrates = prev_bodyrates;
    end
    
    x = position(1);
    y = position(2);
    z = position(3);
    Rb2w = rotmat;
elseif (norm(position - prev_position) > 0.05)
    % jump in position
    position = prev_position;
    velocity = prev_velocity;
    rotmat = prev_rotmat;
    bodyrates = prev_bodyrates;
    jump_count = 1;
    
    x = position(1);
    y = position(2);
    z = position(3);
    Rb2w = rotmat;
end

prev_position = position;
prev_velocity = velocity;
prev_rotmat = rotmat;
prev_bodyrates = bodyrates;

%% Filtering
pqr_filter = [bodyrates pqr_filter(:,1:end-1)];
p = pqr_filter(1,:)*filt_window;
q = pqr_filter(2,:)*filt_window;
r = pqr_filter(3,:)*filt_window;

vel_filter = [velocity vel_filter(:,1:end-1)];
xdot = vel_filter(1,:)*filt_window;
ydot = vel_filter(2,:)*filt_window;
zdot = vel_filter(3,:)*filt_window;

%% Internal States
% stabilizer bar orientation
b_z_bardotz = 1/Tf_up*acos(prev_z_bar(3))*sqrt(prev_z_bar(1)^2 + prev_z_bar(2)^2);
if (b_z_bardotz <= 0)
    b_z_bardot = [0 0 0]';
else
    temp = prev_z_bar(3)*b_z_bardotz/(prev_z_bar(1)^2+prev_z_bar(2)^2);
    b_z_bardot = [-prev_z_bar(1)*temp -prev_z_bar(2)*temp b_z_bardotz]';
end

A_k = [0 r -q; -r 0 p; q -p 0];

z_bar = prev_z_bar + (A_k*prev_z_bar + b_z_bardot)*dt;
z_bar = z_bar/norm(z_bar);
prev_z_bar = z_bar;

% Rotor speeds
prev_Omega_up_des = motor_up*430 - 21.5;
prev_Omega_lo_des = motor_lo*430 - 21.5;

Omega_up = prev_Omega_up + 1/Tf_motup*(prev_Omega_up_des - prev_Omega_up)*dt;
Omega_lo = prev_Omega_lo + 1/Tf_motlo*(prev_Omega_lo_des - prev_Omega_lo)*dt;
if (Omega_up < 0)
    Omega_up = 0;
end
if (Omega_lo < 0)
    Omega_lo = 0;
end
prev_Omega_up = Omega_up;
prev_Omega_lo = Omega_lo;

roll = atan2(2*(qw*qx+qy*qz),1-2*(qx^2+qy^2));
pitch = asin(2*(qw*qy-qz*qx));
yaw = atan2(2*(qw*qz+qx*qy),1-2*(qy^2+qz^2));
coax_state = [x y z  xdot ydot zdot  roll pitch yaw  p q r  Omega_up Omega_lo  z_bar']';


%% Control According to Control Mode
switch CONTROL_MODE
    case CONTROL_START
        % get start position from vicon
        % service call not possible -> switch to raw control manually!!
        % Make CoaX hover at a certain height
        
        if (FIRST_START)
            START_POSITION = coax_state(1:3);
            x_proj = Rb2w(:,1) - [0 0 Rb2w(3,1)]';
            START_ORIENTATION = atan2(x_proj(2),x_proj(1));
            START_TIME = time;
            
            FIRST_START = 0;
        end
        
        dt_start = etime(time,START_TIME);
        if (dt_start < IDLE_TIME/2)
            motor_up = 0.35;
            motor_lo = 0;
            servo1   = 0;
            servo2   = 0;
        elseif (dt_start < IDLE_TIME)
            motor_up = 0.35;
            motor_lo = 0.35;
            servo1   = 0;
            servo2   = 0;
        elseif (dt_start < IDLE_TIME + RISE_TIME)
            %rise
            desPosition = START_POSITION;
            desPosition(3) = START_POSITION(3) + RISE_VELOCITY*(dt_start - IDLE_TIME);
            trajectory = [desPosition' 0 0 RISE_VELOCITY 0 0 0 START_ORIENTATION 0]';
            [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);
        else
            if (AUTO_TRIM)
                CONTROL_MODE = CONTROL_TRIM;
                FIRST_TRIM = 1;
            else
                CONTROL_MODE = CONTROL_HOVER;
                % No FIRST_HOVER required !!!
            end
            hover_position = START_POSITION + [0 0 START_HEIGHT]';
            hover_orientation = START_ORIENTATION;
            trajectory = [hover_position' 0 0 0 0 0 0 hover_orientation 0]';
            [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);
        end
    
    case CONTROL_TRIM
        % hover in place, let integrator saturate to find servo trim values
        
        if (FIRST_TRIM)
            TRIM_TIME = time;
            FIRST_TRIM = 0;
            trim_incr = 1;
        end
        dt_trim = etime(time,TRIM_TIME);
        if (dt_trim > TRIM_DURATION)
            CONTROL_MODE = CONTROL_HOVER;
            e_i(1:2) = [0 0]';
            mean_trim = mean(servo_trim,2);
            trim = geometry_msgs_Quaternion('empty');
            trim.x = mean_trim(1);
            trim.y = mean_trim(2);
            geometry_msgs_Quaternion('send',tid,trim);
        elseif(dt_trim > 0.75*TRIM_DURATION)
            servo_trim(:,trim_incr) = trim_values(3:4);
            trim_incr = trim_incr + 1;
        end
        % hover position/orientation from control_start mode
        trajectory = [hover_position' 0 0 0 0 0 0 hover_orientation 0]';
        [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);

    case CONTROL_HOVER
        % Take current position from vicon and set it as hover position
        
        if (FIRST_HOVER)
            hover_position = coax_state(1:3);
            x_proj = Rb2w(:,1) - [0 0 Rb2w(3,1)]';
            hover_orientation = atan2(x_proj(2),x_proj(1));
            FIRST_HOVER = 0;
        end
        if (LOW_POWER_DETECTED)
            CONTROL_MODE = CONTROL_LANDING;
            FIRST_LANDING = 1;
        end

        trajectory = [hover_position' 0 0 0 0 0 0 hover_orientation 0]';
        [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);

    case CONTROL_GOTOPOS
        % take a desired position and orientation as input
        % move to this position (maybe with given acceleration / vmax)
        
        if (FIRST_GOTOPOS)
            initial_gotopos_position = coax_state(1:3);
            gotopos_time = time;
            gotopos_distance = norm(gotopos_position - initial_gotopos_position);
            gotopos_direction = (gotopos_position - initial_gotopos_position)/gotopos_distance;
            gotopos_duration = gotopos_distance/GOTOPOS_VELOCITY;
            FIRST_GOTOPOS = 0;
        end
        if (LOW_POWER_DETECTED)
            CONTROL_MODE = CONTROL_HOVER;
            FIRST_HOVER = 1;
        end
        
        dt_gotopos = etime(time, gotopos_time);
        if (dt_gotopos < gotopos_duration)
            desPosition = initial_gotopos_position + GOTOPOS_VELOCITY*dt_gotopos*gotopos_direction;
            trajectory = [desPosition' GOTOPOS_VELOCITY*gotopos_direction' 0 0 0 gotopos_orientation 0]';
            [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);
        else
            if (SERVICE_LANDING)
                CONTROL_MODE = CONTROL_LANDING;
                desPosition = START_POSITION + [0 0 START_HEIGHT]';
                trajectory = [desPosition' 0 0 0 0 0 0 gotopos_orientation 0]';
                [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);
            elseif (SERVICE_TRAJECTORY)
                CONTROL_MODE = CONTROL_TRAJECTORY;
                desPosition = coax_state(1:3);
                trajectory = [desPosition' 0 0 0 0 0 0 gotopos_orientation 0]';
                [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);
            else
                CONTROL_MODE = CONTROL_HOVER; % in the end
                hover_position = gotopos_position; % manually entered goto position (if possible) #####
                trajectory = [hover_position' 0 0 0 0 0 0 gotopos_orientation 0]';
                [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);
            end
        end
        
    case CONTROL_TRAJECTORY
        % use CONTROL_GOTOPOS to go to desired initial position
        % Start following trajectory
        % calculate desired state according to current time and desired
        % trajectory
        % calculate control inputs according to desired state

        current_position = coax_state(1:3);
        if (FIRST_TRAJECTORY)
            [~,initial_pose] = trajectory_generation(0,TRAJECTORY_TYPE);
            initial_trajectory_position = initial_pose(1:3);
            initial_trajectory_orientation = initial_pose(4);
            if 0;%(norm(current_position - initial_trajectory_position) > 0.05)
%                 CONTROL_MODE = CONTROL_GOTOPOS;
%                 FIRST_GOTOPOS = 1;
%                 SERVICE_TRAJECTORY = 1;
%                 gotopos_position = initial_trajectory_position;
%                 gotopos_orientation = initial_trajectory_orientation;
            else
                FIRST_TRAJECTORY = 0;
                trajectory_time = time;
            end
            desPosition = coax_state(1:3);
            trajectory = [desPosition' 0 0 0 0 0 0 initial_trajectory_orientation 0]';
            [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);
        else
            dt_traj = etime(time, trajectory_time);
            trajectory = trajectory_generation(dt_traj,TRAJECTORY_TYPE);
            [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);
%             if (dt_traj > 60)
%                 CONTROL_MODE = CONTROL_LANDING;
%                 FIRST_LANDING = 1;
%             end
        end
        
        if (LOW_POWER_DETECTED)
            CONTROL_MODE = CONTROL_HOVER;
            FIRST_HOVER = 1;
        end
        
    case CONTROL_LANDING
        % use CONTROL_GOTOPOS to go to takeoff position
        % sink until landed
        % stop motors
        
        current_position = coax_state(1:3);
        if (FIRST_LANDING)
            if (norm(current_position - (START_POSITION + [0 0 START_HEIGHT]')) > 0.1)
                CONTROL_MODE = CONTROL_GOTOPOS;
                FIRST_GOTOPOS = 1;
                SERVICE_LANDING = 1;
                gotopos_position = START_POSITION + [0 0 START_HEIGHT]';
                gotopos_orientation = START_ORIENTATION;
            else
                FIRST_LANDING = 0;
                landing_time = time;
            end
            desPosition = coax_state(1:3);
            trajectory = [desPosition' 0 0 0 0 0 0 START_ORIENTATION 0]';
            [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);
        else
            dt_land = etime(time, landing_time);
            if (dt_land < SINK_TIME)
                desPosition = START_POSITION + [0 0 START_HEIGHT]';
                desPosition(3) = desPosition(3) - SINK_VELOCITY*dt_land;
                trajectory = [desPosition' 0 0 -SINK_VELOCITY 0 0 0 START_ORIENTATION 0]';
                [motor_up, motor_lo, servo1, servo2, e_i, trim_values] = control_function(coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);
            elseif (dt_land < SINK_TIME + IDLE_TIME)
                motor_up = 0.35;
                motor_lo = 0.35;
                servo1 = 0;
                servo2 = 0;
            else
                CONTROL_MODE = CONTROL_LANDED; % in the end of maneuver
                e_i = [0 0 0 0]'; % flush integrator
                motor_up = 0;
                motor_lo = 0;
                servo1 = 0;
                servo2 = 0;
                nav_mode = std_msgs_Bool('empty');
                nav_mode.data = 0;
                std_msgs_Bool('send',mid,nav_mode); % switch to NAV_STOP_MODE
            end
        end
        
    case CONTROL_LANDED
        % turn off rotors completely
        motor_up = 0;
        motor_lo = 0;
        servo1 = 0;
        servo2 = 0;
        
end
% Voltage compensation
if (~isempty(info))
    v_bat = round(100*info.y)/100;
    volt_compUp = (12.22 - v_bat)*0.0279;
    volt_compLo = (12.22 - v_bat)*0.0287;
end;
if (motor_up > 0.05 || motor_lo > 0.05)
    motor_up = motor_up + volt_compUp;
    motor_lo = motor_lo + volt_compLo;
end

% send inputs
raw_control = geometry_msgs_Quaternion('empty');
raw_control.x = motor_up;
raw_control.y = motor_lo;
raw_control.z = servo1;
raw_control.w = servo2;
geometry_msgs_Quaternion('send',cid,raw_control);

prev_time = time;

%%%%%%%%%
if (CONTROL_MODE == CONTROL_TRAJECTORY)
    pos = [odom.pose.pose.position.x odom.pose.pose.position.y odom.pose.pose.position.z]';
    ori = [odom.pose.pose.orientation.x odom.pose.pose.orientation.y odom.pose.pose.orientation.z odom.pose.pose.orientation.w]';
    lintwist = [odom.twist.twist.linear.x odom.twist.twist.linear.y odom.twist.twist.linear.z]';
    angtwist = [odom.twist.twist.angular.x odom.twist.twist.angular.y odom.twist.twist.angular.z]';

    TimeStamps(i) = odom.header.stamp;
    Positions(:,i) = pos;
    Orientations(:,i) = ori;
    Lintwists(:,i) = lintwist;
    Angtwists(:,i) = angtwist;
    Inputs(:,i) = [motor_up-volt_compUp motor_lo-volt_compLo servo1 servo2]';
%     if (CONTROL_MODE == CONTROL_HOVER)
%         Trims(:,i) = trim_values;
%     else
%         Trims(:,i) = [0 0 0 0]';
%     end
    i = i+1;
end
%%%%%%%%%
% fprintf('CONTROL_MODE: %d \n',CONTROL_MODE);
end % end of loop

%%%%%%%%%
if (~isempty(TimeStamps))
    Data.time = TimeStamps - TimeStamps(1);
    Data.position = Positions;
    Data.orientation = Orientations;
    Data.lintwist = Lintwists;
    Data.angtwist = Angtwists;
    Data.inputs = Inputs;
    % Data.trim = Trims;

    save ViconData Data
end
%%%%%%%%%

nav_msgs_Odometry('disconnect',pid);
geometry_msgs_Quaternion('disconnect',iid);
geometry_msgs_Quaternion('disconnect',tid);
std_msgs_Bool('disconnect',mid);
geometry_msgs_Quaternion('disconnect',cid);
geometry_msgs_Quaternion('disconnect',cmid);

clear
