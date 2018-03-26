Compile dependencies

1. git submodule update --init
2. cd StochHMM
3. ./configure
4. make

Install

1. negomo is a ROS package, it must be under your ROS workspace

Compile negomo

1. cd utils
2. ./setup.sh
3. catkin bt (or whatever way you compile a ROS package)
4. (may require a rebuild. please follow instructions from script)

Generate models

1. cd utils
2. ./gen_header.sh initial70 during70
