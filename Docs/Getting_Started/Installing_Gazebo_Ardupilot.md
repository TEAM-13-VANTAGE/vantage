# Gazebo11 + SITL Tutorial for Ubuntu 20.04

## Install ArduPilot

1. Install git, g++, and CMake. 

```
sudo apt-get install git g++ cmake
```

2. Go to your home directory: 

```
cd
```

3. Clone the ArduPilot repository and enter it:

```
git clone https://github.com/ardupilot/ardupilot.git
cd ardupilot
```

4. Install dependencies: 

```
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

5. Reload your shell profile:

```
. ~/.profile
```

6. Configure Git to use https:// instead of git://: 

```
git config --global url.https://.insteadOf git://
```

7. Checkout the latest Copter build: 

```
git checkout Copter-4.0.4
git submodule update --init --recursive
```

8. Run Software In The Loop (SITL) once: 

```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

Wait about a minute and exit (`Ctrl+C`)

## Install Gazebo

1. Configure APT to accept software from the Gazebo mirror:

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

2. Add the key to APT and reload your package repository:

```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update -y
```

3. Install Gazebo11 and the development libraries:

```
sudo apt-get install gazebo11 libgazebo11-dev
```

4. Go to your home directory and install the Gazebo plugin for ArduPilot:

```
cd
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
```

5. Build and install the plugin:

```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

6. Add the setup and model paths to your shell configuration:

```
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
```

7. Restart the VM: 

```
shutdown -r now
```

## Drone Takeoff Simulation

To start the drone takeoff simulation, run this command in one terminal window: 

```
cd ~/ardupilot_gazebo
gazebo --verbose worlds/iris_arducopter_runway.world
```

In a second terminal, run: 

```
cd ~/ardupilot/ArduCopter
```

again in the second terminal, run: 

```
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
```

A bunch of windows should pop up including Gazebo (3D environment with drone and runway), ArduPilot, MAV Console, and MAV Map. 

In the terminal you started SITL in, the "STABILIZE> " prompt should now appear. 
Run the following commands: 

STABILIZE> `mode guided`

GUIDED> `arm throttle`

GUIDED> `takeoff 5`

The drone should takeoff in Gazebo

## References: 

- [ArduPilot installation guide](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md)

- [Gazebo installation guide](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md)