Known Issues and Solutions
==================================================

.. toctree:: 
   :maxdepth: 6

Position Errors
----------------------

**Issue Description**: There might be some error in the visual recognition of the box position, causing a deviation in the suction cup's position when the robot picks up the box. Due to potential errors in the pallet coordinate system calibration, there could also be deviations in the placement position.

**Solution**: These two types of errors are systematic errors. Before execution, you can run the software in test mode to determine the pick-up and placement errors. Then, refer to the pose adjustment methods in section 'Simulation' to compensate for these errors. Use pick-up point offset to compensate for the pick-up error and placement point offset to compensate for the placement error.

6.3.Pose Errors
----------------------

**Issue Description**: Incorrect pick-up and placement poses.

**Solution**: Reset the pick-up and placement reference poses. After the suction cup grips the box, manually move the robot near the pick-up position, ensuring the long side of the box is perpendicular to the conveyor belt's movement direction. Switch from virtual to real mode and save the new pick-up reference pose. For the placement reference pose, after gripping the box with the suction cup, manually move the robot near the placement point, ensuring the long side of the box is parallel to one side of the pallet. Switch to real mode and save the new placement reference pose.

Inaccessible Position
----------------------

**Issue Description**: The robot reports an inaccessible position error and cannot reach the designated position.

**Solution**: Please check if the current tool number and workpiece number settings are correct. If the settings are correct, it is recommended to reset the pick-up intermediate point and the placement intermediate point. For the pick-up intermediate point, set it at an appropriate distance above the pick-up point, ensuring that its posture does not differ significantly from the pick-up reference posture. For the placement intermediate point, set it above the center of the pallet at a height greater than the pallet's center + stacking height + safety height, ensuring that its posture does not differ significantly from the placement reference posture.
