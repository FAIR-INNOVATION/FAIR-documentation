Robot parameter setting
===========================

Set the installation method
----------------------------

The default installation mode of the robot is horizontal installation. When the installation mode of the robot is changed, the actual installation mode of the robot must be set in the "Initial - Base - Mounting" menu in time to ensure the normal operation of the robot.

The user clicks the "Fixed" option under the "Initial - Base - Mounting" menu to enter the robot fixed installation mode setting page, selects "Upright Installation", "Inverted Installation" or "Side Installation", and clicks the "Apply" button to complete the robot installation mode setting.

.. image:: teaching_pendant_software/025.png
   :width: 6in
   :align: center

.. centered:: Figure 3.1-1 Fixed mounting

Considering the more flexible and rich robot deployment scenarios, we provide a free installation function. Users click the "Free" tab under the "Initial - Base - Mounting" menu to enter the robot free installation mode setting page. Manually adjust the "Base Tilt" and "Base Rotation" angles, and the 3D model will display the installation effect accordingly. After modification, click the "Apply" button to complete the robot installation mode setting.

.. image:: teaching_pendant_software/026.png
   :width: 6in
   :align: center
   
.. centered:: Figure 3.1-2 360 degree free mounting

.. important::
   After the installation of the robot is completed, the installation method of the robot must be set correctly, otherwise it will affect the use of the robot's dragging function and collision detection function.

Set end load
--------------------

Under the menu bar of "Initial - Base - Payload", click "End Payload" to enter End payload page .

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

.. image:: base/016.png
   :width: 4in
   :align: center

.. centered:: Figure 3.2-1 Schematic diagram of load setting

.. important:: 
   After the load is installed at the end of the robot, the weight of the end load and the coordinates of the center of mass must be set correctly, otherwise it will affect the drag function of the robot and the use of the collision detection function.

Set tool coordinates
--------------------

In the menu bar of "Initial - Base - Coordinate", click "TCP" to enter the tool coordinate page.

Tool coordinates can be modified, cleared and applied. In the drop-down list of tool coordinate systems, after selecting the corresponding coordinate system(the coordinate system name can be customized), the corresponding coordinate value, tool type and installation location (only displayed under sensor type tools) will be displayed below. After selecting a coordinate system, click the "Apply" button, and the currently used tool coordinate system will become the selected coordinate, as shown below.

.. image:: base/001.png
   :width: 4in
   :align: center

.. centered:: Figure 3.3-1 Set tool coordinates

Under QNX:

- There are 15 tool coordinate systems.

Under Linux:

- There are 20 tool coordinate systems.

Click "Modify" to reset the tool coordinate system of the number according to the prompt. The tool calibration method is divided into four-point method and six-point method. The four-point method only calibrates the tool TCP, that is, the position of the tool center point. Its posture is consistent with the end posture by default. The six-point method adds two points to the four-point method. , used to calibrate the pose of the tool.

.. image:: base/002.png
   :width: 4in
   :align: center

.. centered:: Figure 3.3-2 Set tool coordinates

.. important:: 
   1. After the tool is installed at the end, the tool coordinate system must be calibrated and applied, otherwise the position and attitude of the tool center point will not meet the expected values when the robot executes the motion command.

   2. The tool coordinate system generally uses toolcoord1~toolcoord14, and toolcoord0 is used to indicate that the position center of the tool TCP is at the center of the end flange. When calibrating the tool coordinate system, it is first necessary to apply the tool coordinate system to toolcoord0, and then select other tool coordinate systems for calibration. Calibration and application.
