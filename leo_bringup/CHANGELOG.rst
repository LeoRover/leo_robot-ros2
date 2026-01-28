^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leo_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.1 (2026-01-09)
------------------
* fix: Detect charging monitor sensor reset and reapply calibration (`#39 <https://github.com/LeoRover/leo_robot-ros2/issues/39>`_)
* Contributors: Błażej Sowa

2.5.0 (2025-12-29)
------------------
* feat: Add charging monitor node (`#38 <https://github.com/LeoRover/leo_robot-ros2/issues/38>`_)
* Contributors: Błażej Sowa

2.4.0 (2025-07-24)
------------------
* Complementary filter improvements (`#34 <https://github.com/LeoRover/leo_robot-ros2/issues/34>`_)
* Contributors: Jan Hernas

2.3.0 (2025-05-28)
------------------
* Improve camera quality (`#33 <https://github.com/LeoRover/leo_robot-ros2/issues/33>`_)
  * Set camera tuning file for OV5647 NoIR
  * Increase ov5647 resolution
  * Recalibrate camera for new resolution
  * Change camera name for imx477
  * Improve compression quality for imx477
* Contributors: Błażej Sowa

2.2.0 (2025-05-23)
------------------
* Use services instead of topics for reboot and shutdown commands (`#28 <https://github.com/LeoRover/leo_robot-ros2/issues/28>`_)
* Specify camera frame_id (`#27 <https://github.com/LeoRover/leo_robot-ros2/issues/27>`_)
* Add publish_odom_tf argument to leo_bringup launch file (`#26 <https://github.com/LeoRover/leo_robot-ros2/issues/26>`_)
* Contributors: Błażej Sowa

2.1.3 (2025-05-08)
------------------

2.1.2 (2025-05-08)
------------------
* Update `robot_frame` param (`#22 <https://github.com/LeoRover/leo_robot-ros2/issues/22>`_)
* Contributors: Aleksander Szymański

2.1.1 (2025-04-30)
------------------

2.1.0 (2025-04-30)
------------------
* Improve ov5647 compression quality
* Add `leo_filters` package (`#17 <https://github.com/LeoRover/leo_robot-ros2/issues/17>`_)
* Fix camera config file path
* Add parameter handling based on rover's model (`#19 <https://github.com/LeoRover/leo_robot-ros2/issues/19>`_)
* New camera implementation (`#16 <https://github.com/LeoRover/leo_robot-ros2/issues/16>`_)
* Contributors: Aleksander Szymański, Błażej Sowa, Jan Hernas

2.0.0 (2024-11-18)
------------------
* Add web_video_server to dependencies (`#13 <https://github.com/LeoRover/leo_robot-ros2/issues/13>`_) (`#15 <https://github.com/LeoRover/leo_robot-ros2/issues/15>`_)
* Contributors: mergify[bot]

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
