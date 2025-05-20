# BPC-PRP project 2025

Authors:
- Vojtěch Vařecha, xvarec06
- Vojtěch Růžička, xruzic56

## Project summary

Within the BPC-PRP course, 3 main tasks are implemented on the prepared robot – driving along a line, driving along a corridor and escaping from a maze.
The implementation takes place at a higher level of abstraction using the ROS2 system and the C++ programming language.
The robot has, among other things, a differential chassis, IR line sensors, LiDAR, ultrasonic distance sensors and a camera.

## Simplified project file structure

```

bpc-prp
├── include/
├── notes/
├── paper/
│   └── paper.pdf
├── src/
│   ├── algorithms/
│   ├── nodes/
│   └── main.cpp
├── tests/
├── CMakeLists.txt
└── README.md

```


## Compilation and usage

### Prerequisities

Hardware:
- Fenrir robot (see https://github.com/Robotics-BUT/fenrir-project)
- PC or laptop
- WiFi network compatible with robot and computer

Software:
- OS compatible with ROS2, such as Ubuntu 22.04
- appropriate version of ROS2, such as *Humble* (for Ubuntu 22.04)
- C++ compiler (such as G++ ver. 13.3)
- CMake (at least ver. 3.20)

### Project compilation

Note: First command must be executed from project root directory.

```
cmake -S . -B build
cmake --build build
cd build && make
make run
```

### Usage

Computer keyboard:
- `W`: toggle forward motion
- `A`: toggle left rotation
- `S`: toggle backward motion
- `D`: toggle right rotation
- `F`: hold to move forward
- `B`: hold to move backward
- `C`: follow corridor using LiDAR
- `U`: folow corridor using ultrasound
- `I`: move forward using IMU
- `K`: follow line
- ` ` (*space*): stop all actions

Robot buttons:
1. Follow corridor using ultrasound sensors.
2. Escape from maze.
3. Stop all robot actions.

## Testing

Note: Test coverage of this project is very small.

### Prerequisities

Tests were executed on Ubuntu 22.04 OS.

Software needed:
- C++ compiler (such as G++ ver. 13.3)
- CMake (at least ver. 3.20)
- GoogleTest framework (see https://google.github.io/googletest/)

### Unit tests compilation and running

Note: First command must be executed from project root directory.

```
cd tests
cmake -S . -B test_build
cmake --build test_build
cd test_build && ctest
```

## Further documentation

More detailed documentation is present in `paper/paper.pdf` file.

