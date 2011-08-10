clear
clc

run coax_id_pose
save nlgr_pose nlgr

clear

run coax_id_vel
save nlgr_vel nlgr

clear 
clc

load nlgr_pose
present(nlgr);
load nlgr_vel
present(nlgr);