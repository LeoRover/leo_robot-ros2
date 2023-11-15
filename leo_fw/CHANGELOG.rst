^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leo_fw
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2023-11-15)
------------------
* Update firmware binaries
* Move ImuCalibrator node into leo_fw package (`#8 <https://github.com/LeoRover/leo_robot-ros2/issues/8>`_)
* Add firmware parameter bridge (`#4 <https://github.com/LeoRover/leo_robot-ros2/issues/4>`_)
* Sort dependencies in package.xml
* Update copyright notices in source files
* Use ament cmake tests via colcon (`#7 <https://github.com/LeoRover/leo_robot-ros2/issues/7>`_)
* Mecanum Wheel Odometry in firmware_message_converter (`#5 <https://github.com/LeoRover/leo_robot-ros2/issues/5>`_)
* Reformat code
* Remove redundant imports from calibrate_imu script
* Contributors: Aleksander Szymański, Błażej Sowa

1.3.0 (2023-01-11)
------------------
* Show which wheels failed when testing encoders or torque sensors
* Update hw tests
* Refactor test result printing
* Add timeout for get_board_type and get_firmware_version services
* Include newer firmware binaries
* Use DirectNode in hardware tester
* Update flash logic
* Fix checking if uros agent is active when the service is disabled
* Contributors: Błażej Sowa

1.2.1 (2022-11-30)
------------------
* Fix resolving firmware topic names in firmware_message_converter
* Contributors: Błażej Sowa

1.2.0 (2022-09-21)
------------------
* Fix test_hw script
* Update leocore firmware to version 1.0.1
* Contributors: Błażej Sowa

1.1.0 (2022-08-31)
------------------
* Add IMU gyro calibration (`#3 <https://github.com/LeoRover/leo_robot-ros2/issues/3>`_)
* Contributors: Aleksander Szymański

1.0.1 (2022-08-10)
------------------
* Add firmware binaries
* Contributors: Błażej Sowa

1.0.0 (2022-08-10)
------------------
* Initial ROS2 port
* Contributors: Błażej Sowa
