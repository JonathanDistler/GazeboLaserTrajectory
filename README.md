# GazeboLaserTrajectory

Part of work with Cornell's Aerospace Adversary Lab; the goal of this project is to determine the location of an anti-drone laser system based on its unique laser diffraction using a novel diode-based detection system.

Running the Gazebo simulation requires a Linux system. Since I am on Windows, I use WSL to run the environment.

---

## Setup

'''bash
wsl
cd ~
source /opt/ros/jazzy/setup.bash
'''

'''bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
'''

---

## Configuring Gazebo Environment

'''bash
cd PX4-Autopilot
make px4_sitl gz_x500
'''

'''bash
param set NAV_RCL_ACT 0
param set COM_RCL_EXCEPT 4
param set COM_RC_IN_MODE 4

param set COM_LOW_BAT_ACT 0
param set BAT_CRIT_THR 0.0
param set BAT_LOW_THR 0.0

commander arm -f
commander takeoff
'''

---

## Running Laser Script

'''bash
cd ~
python3 laser.py
'''
