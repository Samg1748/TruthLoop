# TruthLoop

# Before Code Setup
To set up the TruthLoop system, you must: 
- Launch CARLA (0.9.15+)
- Enable ROS communication between the CARLA machine, the LIMO, and the computer utilizing the MoCap data (look up tutorial on multiple machine ROS through LAN)

# Setup for TruthLoop
- Initialize the roscore on the Carla Machine
- Rosrun limo_info_publisher_545.py on the LIMO first
- Rosrun Mocap_Communication.m on the MoCap laptop next to start the MoCap data publishing to ROS
- Plant the straight_waypoints.csv in the same folder as the PID_V3 to simulate on the designated track
- Rosrun carla_pub_PID_V3.py on the CARLA machine for the latest lateral PID between the CARLA and LIMO

# Alternate Setup for the Synchronized CARLA-LIMO System Instead of TruthLoop
- Rosrun limo_info_publisher_545 on the LIMO first
- Rosrun carla_pub_PID.py on the CARLA machine for the synchronized system

# Other Scripts and their Uses
If you want to try TruthLoop on a different course/track:
  - carla_auto.py: auto-pilots a car in CARLA to collect waypoints
  - carla_get_spawnpoint.py: prints the spawn point of the spectator whenever prompted
