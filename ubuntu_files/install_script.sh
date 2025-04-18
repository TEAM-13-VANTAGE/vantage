# -------------------------------
# Install Dependencies
# -------------------------------
echo "------ Fixing Broken Packages and Installing Dependencies ------"
sudo apt --fix-broken install -y
sleep 1

sudo apt update
sleep 1

sudo apt install -y git python3-pip python3-rosdep2 rapidjson-dev libgflags-dev pkg-config \
                    software-properties-common ros-humble-ament-cmake ros-rolling-ament-cmake \
                    locales curl lsb-release gnupg default-jre wget
sleep 1

# Install Python tools
pip install --upgrade vcstool
sleep 1

# Update PATH
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
sleep 1

# -------------------------------
# Install ROS2
# -------------------------------
echo "------ Installing ROS2 ------"
locale
sleep 1

sudo apt install -y locales
sleep 1

sudo locale-gen en_US en_US.UTF-8
sleep 1

sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
sleep 1

# Add ROS2 Repository
sudo apt install -y software-properties-common
sleep 1

sudo add-apt-repository universe -y
sleep 1

sudo apt update && sudo apt install -y curl
sleep 1

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sleep 1

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sleep 1

# Install ROS2 packages
sudo apt update && sudo apt upgrade -y
sleep 1

sudo apt install -y ros-humble-desktop ros-humble-ros-base ros-dev-tools
sleep 1

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
sleep 1

# -------------------------------
# Setup ArduPilot Workspace
# -------------------------------
echo "------ Setting Up ArduPilot Workspace ------"
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws
sleep 1

vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
sleep 1

# Install ROS dependencies
sudo apt update
sleep 1

rosdep update
sleep 1

source /opt/ros/humble/setup.bash
sleep 1

rosdep install --from-paths src --ignore-src -r -y
sleep 1

# -------------------------------
# Install Micro-XRCE-DDS
# -------------------------------
echo "------ Installing Micro-XRCE-DDS ------"
sudo apt install -y default-jre
sleep 1

git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
sleep 1

cd Micro-XRCE-DDS-Gen
./gradlew assemble
sleep 1

export PATH=$PWD/scripts:$PATH
echo "export PATH=$PWD/scripts:$PATH" >> ~/.bashrc
source ~/.bashrc
sleep 1

# -------------------------------
# Build ArduPilot Packages
# -------------------------------
echo "------ Building ArduPilot Packages ------"
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_dds_tests --event-handlers=console_cohesion+
sleep 1

source ~/ardu_ws/install/setup.bash
sleep 1

# -------------------------------
# Install SITL
# -------------------------------
echo "------ Installing SITL ------"
colcon build --packages-up-to ardupilot_sitl
sleep 1

cd ~/ardu_ws/src/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
sleep 1

# -------------------------------
# Install Gazebo
# -------------------------------
echo "------ Installing Gazebo ------"
sudo apt update -y
sleep 1

sudo apt install -y curl lsb-release gnupg wget
sleep 1

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
sleep 1

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sleep 1

sudo apt update
sleep 1

sudo apt install -y gz-harmonic
sleep 1

# Install ROS2-Gazebo Dependencies
wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list
sleep 1

rosdep update
sleep 1

# -------------------------------
# Setup ArduPilot-Gazebo Integration
# -------------------------------
echo "------ Setting Up ArduPilot-Gazebo Integration ------"
cd ~/ardu_ws
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
sleep 1

# Set Gazebo Environment Variables
echo 'export GZ_VERSION=harmonic' >> ~/.bashrc
source ~/.bashrc
sleep 1

# -------------------------------
# Build ArduPilot-Gazebo Packages
# -------------------------------
echo "------ Building ArduPilot-Gazebo Packages ------"
cd ~/ardu_ws
source /opt/ros/humble/setup.bash
sleep 1

rosdep install --from-paths src --ignore-src -r -y
sleep 1

colcon build --packages-up-to ardupilot_gz_bringup
sleep 1

export LIBGL_ALWAYS_SOFTWARE=1

# -------------------------------
# Install Julia
# -------------------------------
echo "------ Installing Julia ------"
curl -fsSL https://install.julialang.org | sh
. /home/ubuntu/.bashrc
cd ~/vantage
julia -e 'using Pkg; Pkg.add("PyCall")'
cd ~/vantage/Particle-Simulation
julia --project=. -e 'using Pkg; Pkg.instantiate()'

# -------------------------------
# Install Python Dependencies
# -------------------------------
echo "------ Installing Python Dependencies ------"
cd ~/vantage
pip install -r requirements.txt
pip install pandas julia pyjulia
# Remove local pip version
pip uninstall -y matplotlib

# Remove system version (optional but safer to remove conflict)
sudo apt remove -y python3-matplotlib

# Reinstall only pip version
pip install matplotlib --upgrade

cd 


# Finalize Environment Setup
echo "------ Finalizing set up ------"
echo "source ~/ardu_ws/install/setup.bash" >> ~/.bashrc
echo 'export PATH=$HOME/ardu_ws/src/ardupilot/Tools/autotest:$PATH' >> ~/.bashrc
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc
source ~/.profile
sleep 1
cp -r ~/vantage/ubuntu_files/worlds ~/ardu_ws/install/ardupilot_gz_gazebo/share/ardupilot_gz_gazebo/
cp -r ~/vantage/ubuntu_files/models ~/ardu_ws/install/ardupilot_gz_description/share/ardupilot_gz_description/



echo "✅ Installation Complete!"