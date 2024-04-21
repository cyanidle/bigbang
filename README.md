# Bigbang

## Install
```bash
set -eux
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 &&
sudo apt-get update && sudo apt-get upgrade -y &&
sudo apt install ros-noetic-desktop -y &&
sudo apt install ros-noetic-rosserial-arduino \
    python3-pip build-essential git qtbase5-dev ros-noetic-cv-bridge -y &&
source /opt/ros/noetic/setup.bash && mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && catkin_init_workspace

cd ~/catkin_ws && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && catkin_make &&
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && source ~/.bashrc &&
cd ~/catkin_ws/src && git clone https://github.com/cyanidle/bigbang &&
cd ~/catkin_ws && catkin_make &&
pip3 install -r $(rospack find bigbang_eurobot)/py_lib/requirements.txt
```

