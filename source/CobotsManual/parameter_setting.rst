Robot parameter setting
===========================

Set the installation method
----------------------------

The default installation mode of the robot is horizontal installation. When the installation mode of the robot is changed, the actual installation mode of the robot must be set on this page in time to ensure the normal operation of the robot.

The user clicks the "Fixed mounting" tab in the robot's 3D virtual display area to enter the robot Fixed mounting. On the mode setting page, select "Front Mount", "Flip Mount" or "Side Mount", and click the "Apply" button to complete the robot installation mode setting.

.. image:: teaching_pendant_software/025.png
   :width: 6in
   :align: center

.. centered:: Figure 3.1-1 Fixed mounting

Considering more flexible and rich robot deployment scenarios, we provide a free mounting function. Users click the "360-degree free mounting" tab in the robot's 3D virtual display area to enter the robot free mounting mode setting page. Manually adjust the "base tilt" and "base rotation" angles, and the 3D model will show the mounting effect accordingly. After modification, click the "Apply" button to complete the robot mounting method setting.

.. image:: teaching_pendant_software/026.png
   :width: 6in
   :align: center
   
.. centered:: Figure 3.1-2 360 degree free mounting

.. important::
   After the installation of the robot is completed, the installation method of the robot must be set correctly, otherwise it will affect the use of the robot's dragging function and collision detection function.

Set end load
--------------------

Under the menu bar of "Robot Settings" in "Initialize", click "End Load" to enter End load page .

When configuring the end load, please enter the mass of the end tool used and the corresponding center of mass coordinates into the "Load mass" and "Load mass center coordinates X, Y and Z" input boxes and apply.

.. important:: 
   The load mass cannot exceed the maximum load range of the robot.

   - FR3:3kg

   - FR5:5kg

   - FR10:10kg

   - FR16:16kg
   
   - FR20:20kg

   - FR30:30kg

   The center of mass coordinate setting range is 0-1000, unit mm.

.. image:: teaching_pendant_software/044.png
   :width: 3in
   :align: center

.. centered:: Figure 3.2-1 Schematic diagram of load setting

.. important:: 
   After the load is installed at the end of the robot, the weight of the end load and the coordinates of the center of mass must be set correctly, otherwise it will affect the drag function of the robot and the use of the collision detection function.

Set tool coordinates
--------------------

Under the menu bar of "Robot Settings" in "Initialize", click "Tool Coordinates" to enter Tool Coordinates Page. Tool coordinates can realize the modification, clearing and application of tool coordinates. There are 15 numbers in the drop-down list of the tool coordinate system. After selecting the corresponding coordinate system (the name of the coordinate system can be customized), the corresponding coordinate value, tool type and installation position will be displayed below (displayed only under the sensor type tool), select Click the "Apply" button after a certain coordinate system, and the currently used tool coordinate system will change to the selected coordinates, as shown in Figure 3.3-1.

Click "Modify" to reset the tool coordinate system of the number according to the prompt. The tool calibration method is divided into four-point method and six-point method. The four-point method only calibrates the tool TCP, that is, the position of the tool center point. Its posture is consistent with the end posture by default. The six-point method adds two points to the four-point method. , used to calibrate the pose of the tool.

.. image:: teaching_pendant_software/027.png
   :width: 3in
   :align: center

.. centered:: Figure 3.3-1 Set tool coordinates

.. image:: teaching_pendant_software/028.png
   :width: 3in
   :align: center

.. centered:: Figure 3.3-2 Set tool coordinates

.. important:: 
   1.After the tool is installed at the end, the tool coordinate system must be calibrated and applied, otherwise the position and attitude of the tool center point will not meet the expected values when the robot executes the motion command.

   2.The tool coordinate system generally uses toolcoord1~toolcoord14, and toolcoord0 is used to indicate that the position center of the tool TCP is at the center of the end flange. When calibrating the tool coordinate system, it is first necessary to apply the tool coordinate system to toolcoord0, and then select other tool coordinate systems for calibration. Calibration and application.
