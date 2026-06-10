# Robot Control (Pure C++)

A lightweight robot control framework with hardware abstraction and differential-drive simulation.

## Build

```bash
cmake -S . -B build
cmake --build build
```

## Run

```bash
./build/robot_control
```

Keyboard controls:

- `W/S` forward / backward
- `A/D` turn left / right
- `Q/E` arc left / right
- `Space` stop
- `X` exit

## Architecture

```
IRobot (abstract)
  └── DifferentialDriveSim   # kinematic simulator (current)
  └── (future) SerialRobot   # real hardware driver
ControlLoop                  # fixed-rate control thread
```

## Test

```bash
ctest --test-dir build
```
