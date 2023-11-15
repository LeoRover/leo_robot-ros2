^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leo_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2023-11-15)
------------------
* Add firmware parameter bridge (`#4 <https://github.com/LeoRover/leo_robot-ros2/issues/4>`_)
* Update mecanum_wheels description
* Use tf_frame_prefix for camera frame
* Add spawn_state_publisher argument
* Sort dependencies in package.xml
* Add mecanum_wheels argument to leo_bringup launch file (`#6 <https://github.com/LeoRover/leo_robot-ros2/issues/6>`_)
* Use ament cmake tests via colcon (`#7 <https://github.com/LeoRover/leo_robot-ros2/issues/7>`_)
* Mecanum Wheel Odometry in firmware_message_converter (`#5 <https://github.com/LeoRover/leo_robot-ros2/issues/5>`_)
* Contributors: Aleksander Szymański, Błażej Sowa

1.3.0 (2023-01-11)
------------------

1.2.1 (2022-11-30)
------------------
* Fix namespace retrieval in leo_system script
* Contributors: Błażej Sowa

1.2.0 (2022-09-21)
------------------
* Add image_proc to dependencies
* Run v4l2_camera node in container together with image_proc nodes
* Add camera calibration file
* Contributors: Błażej Sowa

1.1.0 (2022-08-31)
------------------

1.0.1 (2022-08-10)
------------------

1.0.0 (2022-08-10)
------------------
* Initial ROS2 port
* Contributors: Błażej Sowa, Aleksander Szymański
