^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package picknik_twist_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2025-02-09)
------------------
* Fix deprecated realtime_tools header imports (`#14 <https://github.com/PickNikRobotics/picknik_controllers/issues/14>`_)
* Contributors: Sebastian Castro

0.0.3 (2023-07-24)
------------------
* Use Twist not TwistStamped (`#11 <https://github.com/PickNikRobotics/picknik_controllers/issues/11>`_)
* Contributors: Alex Moriarty

0.0.2 (2023-07-14)
------------------
* fix typo (`#10 <https://github.com/PickNikRobotics/picknik_controllers/issues/10>`_)
  ABI breaking change fixes a typo from original internal package rename for open sourcing
  PicknikTwistControler -> PicknikTwistController
* Contributors: Anthony Baker

0.0.1 (2023-07-11)
------------------
* Initial Release of picknik_twist_controller
  * Originally this was used internally and there was an attempt to release it to ros2_controllers here: https://github.com/ros-controls/ros2_controllers/pull/300
  * The goal is to still move this upstream but it needs to be refactored before going upstream.
* twist_controller -> picknik_twist_controller (`#3 <https://github.com/PickNikRobotics/picknik_controllers/issues/3>`_)
  * twist_controller -> picknik_twist_controller
  1. prefix twist_controller with picknik_twist_controller
  When we merge twist_controller into ros2_controllers we can depricate
  this one and not have naming conflicts as users migrate
  * cmake: 3.8 -> 3.16
  bump to oldest version used on Ubuntu Focal + ROS 2 Humble
  https://www.ros.org/reps/rep-2000.html#humble-hawksbill-may-2022-may-2027
  ---------
* Contributors: Alexander Moriarty @moriarty, Anthony Baker @abake48, @livanov93, @destogl, @MarqRazz, @Abishalini, @JafarAbdi
