
# LSM9DS1 Driver for ROS2
This repository contains a C++ driver that interfaces with an LSM9DS1 sensor over I2C, and a ROS2 wrapper around it.

## Dependencies
-  libi2c-dev

## Setup
Default parameter values are listed here and can be changed in `config/params.yaml`.
``` 
    i2c_interface: 1
    i2c_address_mag: 0x1e
    i2c_address_accelgyro: 0x6b
    accel rate: 0 # [Power Down]
    accel scale: 0 # [2g]
    gyro rate: 0 # [Power Down]
    gyro scale: 0 # [245 deg/s]
    mag rate: 0 # [0.625hz]
    mag scale: 0 # [4 gauss]
    frequency: 100 # [Hz]
```

Build the package in your workspace:

    colcon build --packages-select lsm9ds1_handler

Source setup.bash in your workspace:

    source install/setup.bash
    
Launch it:

    ros2 launch lsm9ds1_handler lsm9ds1_handler.launch.py


