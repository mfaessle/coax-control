function [M1,M2,M3,M4,M5,SB] = read_marker_pos(m1id,m2id,m3id,m4id,m5id,sbid)

Mst1 = geometry_msgs_Quaternion('read',m1id,1);
while (isempty(Mst1))
    Mst1 = geometry_msgs_Quaternion('read',m1id,1);
end
Mst2 = geometry_msgs_Quaternion('read',m2id,1);
while (isempty(Mst2))
    Mst2 = geometry_msgs_Quaternion('read',m2id,1);
end
Mst3 = geometry_msgs_Quaternion('read',m3id,1);
while (isempty(Mst3))
    Mst3 = geometry_msgs_Quaternion('read',m3id,1);
end
Mst4 = geometry_msgs_Quaternion('read',m4id,1);
while (isempty(Mst4))
    Mst4 = geometry_msgs_Quaternion('read',m4id,1);
end
Mst5 = geometry_msgs_Quaternion('read',m5id,1);
while (isempty(Mst5))
    Mst5 = geometry_msgs_Quaternion('read',m5id,1);
end
SBst = geometry_msgs_Quaternion('read',sbid,1);
while (isempty(SBst))
    SBst = geometry_msgs_Quaternion('read',sbid,1);
end

M1 = [Mst1.x Mst1.y Mst1.z Mst1.w]';
M2 = [Mst2.x Mst2.y Mst2.z Mst2.w]';
M3 = [Mst3.x Mst3.y Mst3.z Mst3.w]';
M4 = [Mst4.x Mst4.y Mst4.z Mst4.w]';
M5 = [Mst5.x Mst5.y Mst5.z Mst5.w]';
SB = [SBst.x SBst.y SBst.z SBst.w]';

end

