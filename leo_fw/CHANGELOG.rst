^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leo_fw
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2025-09-13)
------------------
* Backport changes from Jazzy branch to Humble (`#37 <https://github.com/LeoRover/leo_robot-ros2/issues/37>`_)
  * Fix mypy and pylint errors (`#14 <https://github.com/LeoRover/leo_robot-ros2/issues/14>`_)
  * Add missing type annotations in `leo_fw` package nodes (`#18 <https://github.com/LeoRover/leo_robot-ros2/issues/18>`_)
  * Add parameter handling based on rover's model (`#19 <https://github.com/LeoRover/leo_robot-ros2/issues/19>`_)
  * Add `leo_filters` package (`#17 <https://github.com/LeoRover/leo_robot-ros2/issues/17>`_)
  * Include leocore firmware version 2.0.0 (`#20 <https://github.com/LeoRover/leo_robot-ros2/issues/20>`_)
  * Update `robot_frame` param (`#22 <https://github.com/LeoRover/leo_robot-ros2/issues/22>`_)
  * Use target_link_libraries instead of ament_target_dependencies (`#24 <https://github.com/LeoRover/leo_robot-ros2/issues/24>`_)
  * Fix node type errors (`#35 <https://github.com/LeoRover/leo_robot-ros2/issues/35>`_)
  * Use sphinx format docstrings (`#36 <https://github.com/LeoRover/leo_robot-ros2/issues/36>`_)
* Contributors: Błażej Sowa, Aleksander Szymański, Jan Hernas

1.5.0 (2024-11-18)
------------------
* Update firmware binaries (`#11 <https://github.com/LeoRover/leo_robot-ros2/issues/11>`_)
* Add merged odometry to firmware message converter (`#9 <https://github.com/LeoRover/leo_robot-ros2/issues/9>`_)
* Contributors: Aleksander Szymański, Błażej Sowa

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
