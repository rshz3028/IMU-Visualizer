# IMU-Visualizer

A real-time 3D tool for comparing IMU orientation filters on live sensor data, written in C++20 with SDL3, OpenGL and Dear ImGui.

Two 9-axis IMUs (accelerometer, gyroscope, magnetometer) stream data over a serial port. The app runs five orientation estimators on that data side by side and draws the result on a 3D model. It also records each filter's output to CSV so you can compare them offline.

## Features

- **Five orientation filters, all written from scratch**
  - Accelerometer-only tilt
  - Complementary filter
  - Mahony filter
  - Madgwick filter
  - Multiplicative Extended Kalman Filter (MEKF)
- **Dual-IMU fusion**: run filters on IMU 1, IMU 2, an averaged *virtual IMU*, or a *rigid-body virtual IMU* that accounts for each sensor's mounting position and rotation
- **Live 3D view**: an OpenGL renderer with an orbit camera, a grid and axes, glTF (`.glb`) model loading, and primitive spawning
- **ImGui editor**: pick the orientation source and filter, and adjust each IMU's mount rotation at runtime
- **Evaluation recorder**: logs roll, pitch and yaw from every filter for every source to `IMU1.csv`, `IMU2.csv`, `AverageVIMU.csv` and `RigidBodyVIMU.csv`
- **Custom math library**: vectors, matrices and quaternions (`math3d`, `quaternion`), with no GLM dependency

## Project layout

```
src/            application code
src/filter/     orientation filters (accel, complementary, Mahony, Madgwick, MEKF)
include/        headers
assets/         shaders and 3D models
vendor/         third-party: SDL3 (submodule), glad, Dear ImGui
```

## Building (Windows)

Requirements: MinGW-w64 (`g++`, `gcc`, `mingw32-make`), CMake, and an OpenGL 3.3 capable GPU. glad and ImGui are in `vendor/`. SDL3 is a git submodule.

```bash
git clone --recursive https://github.com/rshz3028/IMU-Visualizer.git
cd IMU-Visualizer

# Build SDL3 once
cmake -S vendor/SDL -B vendor/SDL/build -G "MinGW Makefiles"
cmake --build vendor/SDL/build

mingw32-make        # builds build/imu_visualizer.exe
mingw32-make run    # builds and runs
mingw32-make clean
```

The build copies `assets/` and `SDL3.dll` into `build/` automatically.

## Serial data format

The app opens **`COM12` at 115200 baud** (change this in `src/main.cpp`). It expects one line per sample:

```
R,<timestamp>,<ax1>,<ay1>,<az1>,<gx1>,<gy1>,<gz1>,<mx1>,<my1>,<mz1>,<ax2>,<ay2>,<az2>,<gx2>,<gy2>,<gz2>,<mx2>,<my2>,<mz2>\n
```

Fields 1–9 are IMU 1 (accelerometer, gyroscope, magnetometer), and fields 10–18 are IMU 2. Any microcontroller that prints this line format will work.

## License

MIT. See [LICENSE](LICENSE). Third-party libraries in `vendor/` keep their own licenses.
