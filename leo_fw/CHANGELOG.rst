^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leo_fw
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.1 (2026-01-09)
------------------

2.5.0 (2025-12-29)
------------------
* Use sphinx format docstrings (`#36 <https://github.com/LeoRover/leo_robot-ros2/issues/36>`_)
* Add missing dependency on rcl_interfaces
* Contributors: Błażej Sowa

2.4.0 (2025-07-24)
------------------
* Fix node type errors (`#35 <https://github.com/LeoRover/leo_robot-ros2/issues/35>`_)
* Contributors: Błażej Sowa

2.3.0 (2025-05-28)
------------------
* Change wheel_odom_with_covariance topic name to wheel_odom (`#30 <https://github.com/LeoRover/leo_robot-ros2/issues/30>`_)
* Contributors: Błażej Sowa

2.2.0 (2025-05-23)
------------------

2.1.3 (2025-05-08)
------------------
* Use target_link_libraries instead of ament_target_dependencies (`#24 <https://github.com/LeoRover/leo_robot-ros2/issues/24>`_)
* Contributors: Błażej Sowa

2.1.2 (2025-05-08)
------------------
* Update `robot_frame` param (`#22 <https://github.com/LeoRover/leo_robot-ros2/issues/22>`_)
* Contributors: Aleksander Szymański

2.1.1 (2025-04-30)
------------------

2.1.0 (2025-04-30)
------------------
* Include leocore firmware version 2.0.0 (`#20 <https://github.com/LeoRover/leo_robot-ros2/issues/20>`_)
* Add `leo_filters` package (`#17 <https://github.com/LeoRover/leo_robot-ros2/issues/17>`_)
* Add parameter handling based on rover's model (`#19 <https://github.com/LeoRover/leo_robot-ros2/issues/19>`_)
* Add missing type annotations in `leo_fw` package nodes (`#18 <https://github.com/LeoRover/leo_robot-ros2/issues/18>`_)
* Contributors: Aleksander Szymański, Błażej Sowa, Jan Hernas

2.0.0 (2024-11-18)
------------------
* Update firmware binaries (`#11 <https://github.com/LeoRover/leo_robot-ros2/issues/11>`_) (`#12 <https://github.com/LeoRover/leo_robot-ros2/issues/12>`_)
* Fix mypy and pylint errors (`#14 <https://github.com/LeoRover/leo_robot-ros2/issues/14>`_)
* Contributors: Błażej Sowa, mergify[bot]

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
