# Gazebo Laser Trajectory

Work with Cornell's Aerospace Adversary Lab; the goal of this project is to determine the location of an anti-drone laser system based on its unique laser diffraction using a novel diode-based detection system.

Running the Gazebo simulation requires a Linux system. Since I am on Windows, I use WSL to run the environment. The following outlines this particular process, but it should be very similar on a truly Linux system. 


## Setup

```bash
wsl
cd ~
source /opt/ros/jazzy/setup.bash
```
Then, clone the repository for PX4-Autopilot to insantiate drones for the Gazebo environment. 

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```
Then, to configure the environment to override some unnecessary drone-related errors (e.g. low-battery, no GPS, etc.)

## Configuring Gazebo Environment

```bash
cd PX4-Autopilot
make px4_sitl gz_x500
```
Wait until the system fully loads and the x500 drone is present, then the following set the problematic parameters to redundant and then takes off to ~1 in the simulation

```bash
param set NAV_RCL_ACT 0
param set COM_RCL_EXCEPT 4
param set COM_RC_IN_MODE 4

param set COM_LOW_BAT_ACT 0
param set BAT_CRIT_THR 0.0
param set BAT_LOW_THR 0.0

commander arm -f
commander takeoff
```


## Running Laser Script
To run the script that mimics a laser and a intensity pattern, run the following (mine is saved to the home directory, else, "cd" to the correct directory):

```bash
cd ~
python3 gazebo_control_state_estimate.py
```

## Potential Errors
Sometimes the Gazebo window doesn't render with the drone properly, in that event run the following commands and then exit out of the rendered Gazebo world:

```bash
pkill -f gz
pkill -f gazebo
pkill -f px4

gz sim
```

## Helpful Commands
Because I'm running out of a wsl environment, the directories aren't very intuitive, so I found that the following terminal command opens up the wsl native files: 

```bash
explorer.exe . 
```