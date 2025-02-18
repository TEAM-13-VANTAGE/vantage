#!/bin/bash

# Set up the environment
echo "Setting up environment..."

# Set the path to your ArduPilot repository and simulation directory
ARDUPILOT_DIR=~/ardupilot
AUTOTEST_DIR=$ARDUPILOT_DIR/Tools/autotest

# Launch the first drone (ArduCopter 0)
echo "Launching ArduCopter drone 0..."

# Run the first drone in the background
$AUTOTEST_DIR/sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0 &
DRONE_0_PID=$!

# Wait for the first drone to start properly
sleep 5

# Launch the second drone (ArduCopter 1)
echo "Launching ArduCopter drone 1..."

# Run the second drone in the background
$AUTOTEST_DIR/sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I1 &
DRONE_1_PID=$!

# Wait for both drones to initialize
sleep 10

# Start Gazebo
echo "Starting Gazebo simulation..."

# Launch Gazebo in the background
gazebo --no-sound &
GAZEBO_PID=$!

# Wait for Gazebo to initialize
sleep 5

echo "Simulation is running!"

# Wait for user to exit the simulation
wait $DRONE_0_PID
wait $DRONE_1_PID
wait $GAZEBO_PID

echo "Simulation ended."
