# sphere_games

ROS/Gazebo simulations of sphere robots for reinforcement learning activities and ROS communication for Sphero SPRK+ robots.

Designed in Ubuntu 16.04 using ROS Kinetic and Gazebo 7.

# Shared Instructions

## Installation
Make sure the following commands have been added to your .bashrc file or ran in the terminal used to launch rosrun, with [INSERT_PATH_TO_REPO] replaced with the actual path to where this repository has been cloned (without square brackets):
```
source /opt/ros/kinetic/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:[INSERT_PATH_TO_REPO]/plugins/build
export PYTHONPATH=$PYTHONPATH:[INSERT_PATH_TO_REPO]
```

Create and configure a virtual environment for managing Python packages:
```
sudo apt install virtualenv python-opencv
mkdir python2_env
virtualenv python2_env --python=python2.7
source python2_env/bin/activate
pip install numpy pyyaml rospkg catkin_pkg getkey
```

# Simulation Instructions

## Build
In a terminal,
```
cd [REPO_PATH]/plugins/
mkdir build
cd build
cmake ../
make
```

## Execution
In a terminal, run `roscore`

In a second terminal,
```
cd [REPO_PATH]/arenas/
source /usr/share/gazebo/setup.sh
rosrun gazebo_ros gazebo ctf_1v1_arena.world
```

In a third terminal, run the first simple proportional control agent script:
```
source ~/python2_env/bin/activate
cd [repo_path]/agents/
python red_simple.py
```

In a fourth terminal, run the second simple proportional control agent script:
```
source ~/python2_env/bin/activate
cd [repo_path]/agents/
python blue_simple.py
```

In a fifth terminal, run the simulation tracker python script:
```
source ~/python2_env/bin/activate
cd [repo_path]/host/
python sim_tracker.py
```

The red and blue spheres will both attempt to reach the other teams 'flag' and return to their own base as quickly as possible.

# Physical Robot Instructions

No longer accurate. Please see Autonomy Hackathon wiki for updated instructions.
