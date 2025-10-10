Robot parameter setting
===========================

Set the installation method
----------------------------

The default installation mode of the robot is horizontal installation. When the installation mode of the robot is changed, the actual installation mode of the robot must be set in the "Initial - Base - Mounting" menu in time to ensure the normal operation of the robot.

To accommodate more flexible and diverse robot deployment scenarios, we offer a free installation feature. Users can click the 'Free Installation' tab under the "Initial - Base - Mounting" menu to access the robot free installation settings page. Manually adjust the "Base Tilt" and "Base Rotation" angles, and the 3D model will display the corresponding installation effect. After making changes, click the "Apply" button to complete the robot installation setup.

.. image:: teaching_pendant_software/026.png
   :width: 6in
   :align: center
   
.. centered:: Figure 3.1-1 Robot installation

.. important::
   After the installation of the robot is completed, the installation method of the robot must be set correctly, otherwise it will affect the use of the robot's dragging function and collision detection function.

Set end load
--------------------

In the "Initial - Base - Payload" section, select the identification type for "Trajectory Identification" to enter the end effector load setting interface.

.. note:: 
   .. image:: base/071.png
      :width: 0.75in
      :height: 0.75in
      :align: left

   name: **Apply button**
   
   effect: Click the application load number to view the corresponding load weight and center of gravity coordinates.

.. note:: 
   .. image:: base/072.png
      :width: 0.75in
      :height: 0.75in
      :align: left

   name: **Modify button**
   
   effect: Click to open/close the motion recognition interface.

.. note:: 
   .. image:: base/073.png
      :width: 0.75in
      :height: 0.75in
      :align: left

   name: **Rename button**
   
   effect: Rename load name

.. note:: 
   .. image:: base/074.png
      :width: 0.75in
      :height: 0.75in
      :align: left

   name: **Clear button**
   
   effect: Clear current load information (set load weight and center of gravity coordinates to 0).

When configuring the end load, you can directly input the mass of the end tool and the corresponding center of mass coordinates X, Y, and Z, then click the "Apply" button to complete the setup.

At the same time, you can click the "Edit" button to open the "Motion Identification" interface for automatic load identification, which will be applied after the identification process is completed.

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

Enter the tool coordinates page by going to "Initial Settings" -> "Basic" -> "Tool Coordinates".

.. note:: 
   .. image:: base/071.png
      :width: 0.75in
      :height: 0.75in
      :align: left

   name: **Apply button**
   
   effect: Click to apply the tool coordinate system

.. note:: 
   .. image:: base/072.png
      :width: 0.75in
      :height: 0.75in
      :align: left

   name: **Modify button**
   
   effect: Click to open/close the coordinate system calibration interface

.. note:: 
   .. image:: base/073.png
      :width: 0.75in
      :height: 0.75in
      :align: left

   name: **Rename button**
   
   effect: Rename Tool Coordinate System Name

.. note:: 
   .. image:: base/074.png
      :width: 0.75in
      :height: 0.75in
      :align: left

   name: **Clear button**
   
   effect: Clear current tool coordinate information

Tool coordinates can be modified, cleared and applied. In the drop-down list of tool coordinate systems, after selecting the corresponding coordinate system(the coordinate system name can be customized), the corresponding coordinate value, tool type and installation location (only displayed under sensor type tools) will be displayed below. After selecting a coordinate system, click the "Apply" button, and the currently used tool coordinate system will become the selected coordinate, as shown below.

.. image:: base/001.png
   :width: 4in
   :align: center

.. centered:: Figure 3.3-1 Set tool coordinates

Click "Modify" to reset the tool coordinate system of the number according to the prompt. The tool calibration method is divided into four-point method and six-point method. The four-point method only calibrates the tool TCP, that is, the position of the tool center point. Its posture is consistent with the end posture by default. The six-point method adds two points to the four-point method. , used to calibrate the pose of the tool.

.. image:: base/002.png
   :width: 4in
   :align: center

.. centered:: Figure 3.3-2 Tool Frame Calibration

.. important:: 
   1. After the tool is installed at the end, the tool coordinate system must be calibrated and applied, otherwise the position and attitude of the tool center point will not meet the expected values when the robot executes the motion command.

   2. The tool coordinate system generally uses toolcoord1~toolcoord19, and toolcoord0 is used to indicate that the position center of the tool TCP is at the center of the end flange. When calibrating the tool coordinate system, it is first necessary to apply the tool coordinate system to toolcoord0, and then select other tool coordinate systems for calibration. Calibration and application.
