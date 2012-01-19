================
Getting Started:
================

1. Requirements:
	ROS		(http://www.ros.org)
	IPC		(http://www.cs.cmu.edu/~IPC/)
	ipc bridge	(https://alliance.seas.upenn.edu/~meam620/wiki/index.php?n=Roslab.IpcBridge)
	Matlab

2. Get "coax-software" from https://aslforge.ethz.ch/scm/?group_id=34

3. Get "coax-control" from https://github.com/mfaessle/coax-control

4. Compile and install communication api:
	4.1 Go to coax-software/communication
	4.2 (If Mac) Edit "CMakeLists.txt":
		Change line "SET(COMPILING_FOR_OSX no)" to "SET(COMPILING_FOR_OSX yes)"
	4.3 Run "cmake ."
	4.4 Run "make all install"

5. Compile "coax_server" ROS node:
	5.1 Run "roscd coax_server"
	5.2 Edit "CMakeLists.txt":
		Change line "set(COAXHOME $ENV{HOME}/coax/coax-software/deploy)" such that it is pointing to your coax-software/deploy folder
		(If Mac) Change lines "add_definitions(-DSBC_HAS_IO (...) -DLINUX)" to "add_definitions(-DSBC_HAS_IO (...) -DLINUX -DMACOSX)"
		(If Mac) Delete "rt" in lines "target_link_libraries( (...) pthread rt)"
	5.3 Run "rosmake coax_server"

6. Compile "coax_interface" ROS node:
	6.1 Run "roscd coax_interface"
	6.2 Edit "CMakeLists.txt":
		Change line "set(COAXHOME $ENV{HOME}/git/CoaX/coax-software/deploy)" such that it is the same as in .2
	6.3 Run "rosmake coax_interface"


================
Run Matlab Controller:
================

1. Requirements
	1.1 Make sure vicon_calibrate and vicon2odometry are pointing to the correct xml and vsk files respectively (in coax_interface56.launch)
	1.2 Vicon running and tracking CoaX
	1.3 Running roscore
	1.4 Running central

2. Start CoaX
	2.1 Switch RC on
	2.2 Switch CoaX on
	2.3 Release "kill switch" by moving yaw stick on the RC to the lower left corner and release it

3. Run "roslaunch coax_interface coax_interface56.launch" (replace 56 by 57 if using CoaX57)

4. Run "Coax_Control.m" Matlab script

5. Call ROS service to switch between control modes of the FSM in Coax_Control.m
	e.g. run "rosservice call /Coax56/set_control_mode 1" to start the CoaX and go into hover


================
Run coax_gumstix Controller:
================

1. Requirements
	1.1 Make sure vicon_calibrate and vicon2odometry are pointing to the correct xml and vsk files respectively (in coax_interface56.launch)
        1.2 Vicon running and tracking CoaX
        1.3 Running roscore
        1.4 Running central
        1.5 /etc/hosts files on computer and gumstix edited such that they can connect to each other

2. Start CoaX
        2.1 Switch RC on
        2.2 Switch CoaX on
        2.3 Release "kill switch" by moving yaw stick on the RC to the lower left corner and release it

3. Connecto to Gumstix
   	3.1 ssh gumros@192.168.129.194
	3.2 password: gumros

4. Change settings on Gumstix
   	4.1 sudo chmod 777 /dev/ttyS0 (enable reading an writing to coax board)
	4.1 export ROS_MASTER_URI=http://... (uri of roscore running on computer, must match witch /etc/hosts)

5. Lauch ROS nodes and Matlab
   	5.1 Run "roslaunch coax_gumstix_control launch_computer.launch" (on Computer)
	5.2 Run "roslaunch coax_gumstix_control launch_gumstix.launch" (on Gumstix)
	5.3 Run "Coax_Control.m" in "coax_gumstix_control/matlab" (on Computer)

6. Call ROS services
	6.1 Set Control Mode: "rosservice call /set_control_mode 1" (to make the CoaX take off and go to hover, see Matlab code for other commands)


================
Update Code on Gumstix:
================

1. Requirements
	1.1 None, git is installed on the gumstix and the code can directly be pulled form github

2. Pull new code from github
   	2.1 Run "git pull origin" in "coax-control" folder
	
3. Adapt CMakeLists.txt
   	3.1 Edit "coax_gumstix_control/CMakeLists.txt" on gumstix
	3.2 CoaX communication library path must point to the deploy folder -> set(COAXHOME $ENV{HOME}/coax/deploy) 

4. Recompile "coax_gumstix_control"


================
Vicon Calibration:
================

1. Requirements
	1.1 Place CoaX on Calibration stand in the Vicon space
	1.2 Put an Identiy .xml file in coax_vsk/calibration
	1.3 Make sure launch file in coax_vsk/launch takes the desired .xml file from 1.2 as input

2. Calibrate
	2.1 roscd coax_vsk/launch
	2.2 roslaunch CoaxCalibration.launch
	2.3 rostopic echo /vicon_calibrate/pose should show identity
	2.4 rosservice call /vicon_calibrate/toggle_calibration to start calibration
	2.5 wait until /vicon_calibrate/pose converges
	2.6 rosservice call /vicon_calibrate/toggle_calibration to stop calibration
	
3. Use the file from 1.2 as calibration file for experiments


================
Charging:
================

1. Requirements
	THUNDER POWER RC charger/discharger

2. Charging CoaX batteries: (Pro Lite V2 Li-Po 1350mAh 3s 11.1V)
	Maximum charging current: 5.4A
	Typically used charging current: 3A

3. Charging RC batteries: (RCX Transmitter Li-Polymer Battery 11.1V 1800mAh)
	Recommended charging current: 1.0A
	

================
Reprogramm IMU:
================

1. Requirements
	1.1 CoaX programming board (http://www.skybotix.com/support/wiki/index.php/File:Pb_top.jpg)
	1.2 MPLAB ICD 3 device
	1.3 Install MPLAB IDE software (Windows)

2. Connecting
	2.1 Start MPLAB IDE software and make sure CoaX is turned off
	2.2 Plug CoaX programming board onto CoaX board
	2.3 Connect MPLAB ICD 3 to computer and IMU Jack of CoaX programming board (Make sure that it is the IMU Jack otherwise the bootloader on the main dsPIC gets erased!)
	2.4 Switch CoaX on

3. Reprogramming (In MPLAB IDE)
	3.1 Select Programmer: Programmer -> Select Programmer -> MPLAB ICD 3
	3.2 Configure Device: Configure -> Select Device -> dsPIC33FJ256GP506
	3.3 Import File: File -> Import -> ASL_IMU.hex (download at http://support.skybotix.com/downloads.php)
	3.4 Erase PIC: Programmer -> Erase Flash Device
	3.5 Programm PIC: Programmer -> Program


================
Rotor Blades:
================

New rotor blades can be ordered on http://www.helipal.com/hm-5-10-z-01-main-rotor-blade.html?osCsid=cln4qah64681np9qqcbqbfunu3
