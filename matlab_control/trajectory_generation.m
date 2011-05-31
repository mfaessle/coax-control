function [trajectory, initial_pose] = trajectory_generation(t,TYPE)

% Reference contains:
% [x_ref y_ref z_ref u_ref v_ref w_ref udot_ref vdot_ref wdot_ref]

switch TYPE
    case 0 % SPIRAL
    
        radius     = 1;
        omega      = 2*pi/10;
        vel_vert   = 0.5;
        
        initial_pose = [radius 0 0 pi/2];
        
        x_ref      = radius*cos(omega*t);
        y_ref      = radius*sin(omega*t);
        z_ref      = vel_vert*t;
        xdot_ref   = -radius*omega*sin(omega*t);
        ydot_ref   = radius*omega*cos(omega*t);
        zdot_ref   = vel_vert;
        xddot_ref  = -radius*omega^2*cos(omega*t);
        yddot_ref  = -radius*omega^2*sin(omega*t);
        zddot_ref  = 0;
        psi_ref    = omega*t + initial_pose(4);
        psidot_ref = omega;
    
    case 1 % ROTINPLACE

        omega      = 2*pi/3;
        
        initial_pose = [1 1 1 0]';
        
        x_ref      = 0;
        y_ref      = 0;
        z_ref      = 0;
        xdot_ref   = 0;
        ydot_ref   = 0;
        zdot_ref   = 0;
        xddot_ref  = 0;
        yddot_ref  = 0;
        zddot_ref  = 0;
        psi_ref    = omega*t + initial_pose(4);
        psidot_ref = omega;
    
    case 2 % CLIMBING

        vel_vert   = 1;
        
        initial_pose = [1 1 0 0]';
        
        x_ref      = 0;
        y_ref      = 0;
        z_ref      = vel_vert*t;
        xdot_ref   = 0;
        ydot_ref   = 0;
        zdot_ref   = vel_vert;
        xddot_ref  = 0;
        yddot_ref  = 0;
        zddot_ref  = 0;
        psi_ref    = initial_pose(4);
        psidot_ref = 0;
    
    case 3 % LYINGCIRCLE

        radius     = 1;
        omega      = 2*pi/10;
        
        initial_pose = [radius 0 2 pi/2]';
        
        x_ref      = radius*cos(omega*t);
        y_ref      = radius*sin(omega*t);
        z_ref      = 0;
        xdot_ref   = -radius*omega*sin(omega*t);
        ydot_ref   = radius*omega*cos(omega*t);
        zdot_ref   = 0;
        xddot_ref  = -radius*omega^2*cos(omega*t);
        yddot_ref  = -radius*omega^2*sin(omega*t);
        zddot_ref  = 0;
        psi_ref    = omega*t + initial_pose(4);
        psidot_ref = omega;

    case 4 % STANDINGCIRCLE

        radius     = 1;
        omega      = 2*pi/10;
        
        initial_pose = [0 0 radius 0]';
        
        x_ref      = radius*sin(omega*t);
        y_ref      = 0;
        z_ref      = radius*cos(omega*t);
        xdot_ref   = radius*omega*cos(omega*t);
        ydot_ref   = 0*t;
        zdot_ref   = -radius*omega*sin(omega*t);
        xddot_ref  = -radius*omega^2*sin(omega*t);
        yddot_ref  = 0;
        zddot_ref  = -radius*omega^2*cos(omega*t);
        psi_ref    = initial_pose(4);
        psidot_ref = 0;

end

trajectory = [x_ref y_ref z_ref xdot_ref ydot_ref zdot_ref ...
              xddot_ref yddot_ref zddot_ref psi_ref psidot_ref];
         
         