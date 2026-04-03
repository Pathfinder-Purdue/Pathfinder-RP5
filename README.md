# About
Pathfinder's code for the Raspberry Pi 5.

This should handle camera, LiDAR, communication with ESP32, neural networks, and decision making.

# Root Directory
`main.py`: multiprocessor orchestrator that manages starting every utility in processes and managing the queues how they communicate
`haptic_feedback.py`:
`haptic_zones.py`:
`indoor.py`:

# Directories
## computer_vision
Placeholder for computer vision API's that main file can pull from

## esp32_communication
APIs and backends for all the sensors whose data is received from the ESP32. 

Includes: gps, imu, and tof
Will include: esp32_recv, esp32_send

## indoor_nav
Neal's thingy

## lidar
API and backend for for lidar

## motor_strengths
Placeholder for motor strength calculation API for use in main file

## scripts
Testing scripts to be run as modules; they are gathered here to not clog the root dir

## utils
Structs and functions useful in many files

Includes: LatestQueue definition