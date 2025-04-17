#!/bin/bash

# Terminal 1: Launch Gazebo with the runway simulation
gnome-terminal -- bash -c "gz sim -v4 -r iris_runway.sdf; exec bash"

sleep 10
# Terminal 2: Launch the first drone
gnome-terminal -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --console --model JSON; exec bash"

sleep 3
# Terminal 3: Launch the second drone
gnome-terminal -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris -I1 --console --model JSON; exec bash"

