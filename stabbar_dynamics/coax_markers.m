m1id = geometry_msgs_Quaternion('connect','subscriber','marker1','marker1');
m2id = geometry_msgs_Quaternion('connect','subscriber','marker2','marker2');
m3id = geometry_msgs_Quaternion('connect','subscriber','marker3','marker3');
m4id = geometry_msgs_Quaternion('connect','subscriber','marker4','marker4');
m5id = geometry_msgs_Quaternion('connect','subscriber','marker5','marker5');
sbid = geometry_msgs_Quaternion('connect','subscriber','stabbar','stabbar');

Mbody = [-33 34 35 -22 25; ...
         -58 -60 61 20 -18; ...
         -41 -41 -41 39 39];  % markers in body frame (without stabilizer bar)

N = 1000;

SB_ori = zeros(N,3);
time = zeros(N,1);

tic
for i=1:N
    
    % read marker positions
    [M1,M2,M3,M4,M5,SB] = read_marker_pos(m1id,m2id,m3id,m4id,m5id,sbid);

    M = [M1 M2 M3 M4 M5 SB];

    % check visibility
    mw = [];
    mb = [];
    for i=1 : size(Mbody,2)
        if (M(4,i) > 0.5)
            mw = [mw M(1:3,i)];
            mb = [mb Mbody(:,i)];
        end
    end

    % calculate transformation 
    [Rhat,That] = PointsToRot(mb,mw);
    
    % calculate stabilizer bar orientation
    if (SB.w > 0.5)
        hinge_body = [0 0 180]';
        sb = [SB.x SB.y SB.z]';
        sb_body = Rhat'*(sb - That);

        z_SB = sb_body - hinge_body;
        z_SB = z_SB/norm(z_SB); % stabilizer bar orientation in body coordinates
    else
        z_SB = [0 0 0]';
    end

    pause(0.005);
    SB_ori(i,:) = z_SB';
    time(i) = toc;
end
time = time - time(1);

% z_SB
% Rhat
% z_SB

geometry_msgs_Quaternion('disconnect',m1id);
geometry_msgs_Quaternion('disconnect',m2id);
geometry_msgs_Quaternion('disconnect',m3id);
geometry_msgs_Quaternion('disconnect',m4id);
geometry_msgs_Quaternion('disconnect',m5id);
geometry_msgs_Quaternion('disconnect',sbid);