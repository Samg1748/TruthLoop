# TruthLoop

# Prior to Code Set-up
To set-up the TruthLoop system, you must launch CARLA (0.9.15+) and enable ROS communication between the CARLA machine, the LIMO, and the computer utilizing the MoCap data

# Set-up
Initialize the roscore on the Carla Machine
Rosrun limo_info_publisher_545 on the LIMO first
Rosrun Mocap_Communication.m on the MoCap laptop next to start the MoCap data publishing to ROS
Plant the straight_waypoints.csv in the same folder as the PID_V3 to simulate on the designated track
Rosrun carla_pub_PID_V3.py on the CARLA machine for the latest lateral PID between the CARLA and LIMO

# Alternate Set-up for synchronized system instead of TruthLoop
Rosrun limo_info_publisher_545 on the LIMO first
Rosrun carla_pub_PID_V3.py on the CARLA machine for the synchronized system

# Other scripts and their uses
If you wanted to try TruthLoop on the a difference course:
  - carla_auto.py pilots a auto-pilot car in CARLA to collect waypoints
  - carla_get_spawnpoint.py prints the spawnpoint of the spectator whenever prompted
