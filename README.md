# RM Auto Aim MVP

A minimal RoboMaster auto-aim vision system based on C++, OpenCV and Eigen.

## Features

- [ ] Camera / video input
- [ ] Armor detection
- [ ] PnP pose estimation
- [ ] Coordinate transformation
- [ ] Kalman filter tracking
- [ ] Linear prediction
- [ ] Basic fire control logic

## Dependencies

- C++17
- CMake
- OpenCV

## Build

```bash
cmake -B build
cmake --build build -j$(nproc)