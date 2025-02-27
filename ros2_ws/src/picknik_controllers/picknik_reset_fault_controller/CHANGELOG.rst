^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package picknik_reset_fault_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2023-07-24)
------------------

0.0.2 (2023-07-14)
------------------

0.0.1 (2023-07-11)
------------------
* Initial Release of picknik_reset_fault_controller
  * Originally this was used internally and there was an attempt to release it to ros2_controllers
  * After discussion with the ros2 controllers WG over slack we have decided to open source it here first
  * The goal is to still move this upstream but it can be worked on here first and moved in the future
* fault_controller -> picknik_reset_fault_controller (`#2 <https://github.com/PickNikRobotics/picknik_controllers/issues/2>`_)
  * fault_controller -> picknik_reset_fault_controller
  This commit does two things:
  1) renames fault_controller to reset_fault_controller
  2) prefixes with picknik\_
  The first change, is to be more specific what this controller is used
  for.
  The second change is because we want to move this controller into
  ros2_controllers and when that is complete we can drop the picknik\_ and
  depricate this version allowing for a transition period.
  ---------
* Contributors: Alexander Moriarty @moriarty, Anthony Baker @abake48, @livanov93, @destogl, @MarqRazz, @Abishalini, @JafarAbdi
