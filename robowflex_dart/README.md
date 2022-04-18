# Robowflex Dart

Modeling and planning through [DART (Dynamic Animation and Robotics Toolkit)](https://dartsim.github.io/).
Robots (`robowflex::darts::Robot`) and scene geometry (`robowflex::darts::Structure`) are added to a world (`robowflex::darts::World`), which is then used as the base for planning.
An [OMPL](http://ompl.kavrakilab.org/) state space is provided for worlds (`robowflex::darts::StateSpace`) which can control any joint in the world, for multiple robots.
Constraints and inverse kinematics are given through a Task Space Region specification (`robowflex::darts::TSR`).
For convenience, a helper class (`robowflex::darts::PlanBuilder`) makes it easy to specify motion planning queries.

# Scripts

There are a few example scripts to demonstrate the module, in the `scripts` directory.

**Before you start running any script, you need to run the `include/io/filepath_organizer.py` script to adjust the mesh paths.**

- solve_puzzle.cpp : Plan a manipulatable solution path with the LA-RRT-Connect. The environment must be in include/io/envs. The script can be run with `./solve_puzzle <environment_name>` <runtime in seconds>. The solution path is saved in `include/io/path_result/$env_name$.txt`

- solve_puzzle_animation.cpp : Plan a manipulatable solution path with LA-RRT-Connect and animate it with Robowflex DART. The script can be run with `./solve_puzzle_animation <environment_name> <runtime in seconds>`. The solution path is saved in `include/io/path_result/$env_name$.txt`

 - solve_escape_room.cpp : Plan a solution of the given escape room scenario with LA-RRT-Connect. The script can be run with `./solve_escape_room <room_name> <runtime in seconds>`. `room0` and `room1` environments are manipulable, `room2` and `room3` demonstrate the limitations of the LA-RRT-Connect algorithm. Solution path is saved in `include/io/path_result/$env_name$.txt`.
  
- maze_with_fetch.cpp : Constrained planning of the solution from the LA-RRT algorithm with a modified version of Fetch robot including x-y-axis movement and rotation (from RobowflexResources (link)). The script can be run with `./maze_with_fetch <environment_name>`. The solution path of the LA-RRT-Connect algorithm must be either given in `include/io/path_result/$env_name$.txt` or one of the scripts `solve_puzzle` or `solve_puzzle_animation` must be run before. 

- benchmark_main.cpp : Benchmarks a given environment with the specified algorithms in the .cpp file. The script can be run with `./benchmark_main <environment_name> <time(in seconds)> <run_count>` and creates a .db file using the OMPL (link) script. The results can be found in `include/io/db_files/$env_name$.db`.

- benchmark_room.cpp - Benchmarks the escape room scenarios from (paper link). The time and run count can be adjusted in the .cpp file. The results can be found in `include/io/db_files/$env_name$.db`

- escape_room.cpp - Manipulates the solution of the LA-RRT-Algorithm for an escape room scenario (`room0` or `room1`) with Fetch robot. Solution path must be given in `include/io/path_result/$env_name$.txt` or the script `solve_escape_room` must be run earlier.

# Installation Instructions

@@ -83,12 +73,4 @@ Or clone from the repository:
```sh
git clone git:://github.com/ompl/ompl.git
```
# Robowflex Puzzles

** Before you start running any script, you need to run the `include/io/filepath_organizer.py` script to adjust the mesh paths. **
  
Other environments can be added into the `include/io/envs` folder with the meshes-srdf-urdf file structure.

Benchmarking results are saved as `.db` files inside `include/io/db_files`.

Also, the solution paths are saved inside the `include/io/path_result` folder as a .txt file with the name of the solved environment.


# Installation Instructions

DART can either be installed via a PPA or be added as a catkin package in your local workspace.
See [DART's own installation instructions for more details](https://dartsim.github.io/install_dart_on_ubuntu.html#install-dart).

## PPA Installation

To install DART via PPA, simply do the following:

```sh
sudo apt-add-repository ppa:dartsim/ppa
sudo apt-get update  # not necessary since Bionic
```

After the PPA is installed, then get DART:
```sh
sudo apt-get install libdart6-all-dev
```

## Source Installation

Install DART's dependencies:
```sh
sudo apt-get install \
  libeigen3-dev \
  libassimp-dev \
  libccd-dev \
  libfcl-dev \
  libboost-regex-dev \
  libboost-system-dev \
  libopenscenegraph-dev \
  libnlopt-dev \
  coinor-libipopt-dev \
  libbullet-dev \
  libode-dev \
  liboctomap-dev \
  libflann-dev \
  libtinyxml2-dev \
  liburdfdom-dev \
  libxi-dev \
  libxmu-dev \
  freeglut3-dev \
  libopenscenegraph-dev
```

Then, add DART to your local catkin workspace:
```sh
git clone -b v6.10.0 git://github.com/dartsim/dart.git
```

You can also follow the DART installation instructions here: http://dartsim.github.io/install_dart_on_ubuntu.html
  
## OMPL Installation

The DART module relies on recent versions of OMPL (> 1.4.0).
If you are using an old OMPL (e.g., on Kinetic), either download the source into your local catkin workspace:
```sh
wget -O - https://github.com/ompl/ompl/archive/1.5.0.tar.gz | tar zxf -
```
Or clone from the repository:
```sh
git clone git:://github.com/ompl/ompl.git
```
