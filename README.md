
# LSM9DS1 Driver for ROS2
This repository contains a C++ driver that interfaces with an LSM9DS1 sensor over I2C, and a ROS2 wrapper around it.

## Dependencies
-  libi2c-dev

## Setup
Default parameter values are listed here and can be changed in `config/params.yaml`.
``` 
    frequency: 100 # [Hz]
    i2c_interface: 1
    i2c_address_mag: 0x1e
    i2c_address_accelgyro: 0x6b
    accel rate: 3 # [119 Hz]
    accel scale: 0 # [2g]
    gyro rate: 3 # [119 Hz]
    gyro scale: 3 # [2000 deg/s]
    mag rate: 7 # [80 Hz]
    mag scale: 3 # [16 gauss]
```

Build the package in your workspace:

    colcon build --packages-select lsm9ds1_handler

Source setup.bash in your workspace:

    source install/setup.bash
    
Launch it:

    ros2 launch lsm9ds1_handler lsm9ds1_handler.launch.py


