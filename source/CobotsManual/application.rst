Application
===============

.. toctree:: 
   :maxdepth: 6

Robot packaging
------------------------

In the menu bar of "Application - Tool App", click the "Robot packing" button to enter the robot's one-click packaging interface.

.. important:: 
   Before operating the packaging function, please confirm the surrounding environment and status of the robot to prevent collisions.
   
   If it is shipped from the factory, go to System Settings - General Settings and restore the factory settings before leaving the factory.

**Step1**:Move the robot to zero point before moving to the packing point.

**Step2**:Click the "Move to Zero" button to confirm that the robot's mechanical zero point is correct and that the gaps in the orange circles in the figure are aligned with each joint.

**Step3**:Click the "Move to Packing Point" button, and the robot will move to the packaging point according to the angles of each axis of the packaging process.

.. image:: application/001.png
   :width: 4in
   :align: center

.. centered:: Figure 14.1‑1 Robot one-click packaging

System Upgrade
-----------------

Preparation
~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Before upgrading, check and confirm the current software version in *System Settings > About*.
2. Download the software upgrade package from the Farobot documentation *Downloads > Robot Software Downloads* for the corresponding version. After extraction, the package includes the software upgrade file ``software.tar.gz``.

Important Notes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. **Data Backup**: It is recommended to back up data before upgrading (refer to Section 3.2.1) to avoid data loss due to upgrade failures.
2. **Version Restrictions**:

.. centered:: Table 14.1-1 Version Upgrade Restrictions

.. list-table::
   :widths: 50 50
   :header-rows: 0
   :align: center

   * - **Current Version** 
     - **Maximum Upgradable Version**

   * - <v3.6.1
     - v3.6.1

   * - v3.6.1-v3.6.4
     - v3.6.5

   * - v3.6.5-v3.6.8
     - v3.6.9

   * - v3.6.9 - v3.7.4
     - v3.7.5

   * - v3.7.5
     - v3.7.6

   * - ≥ v3.7.6
     - No restrictions

3. **Cache Clearance**: After each upgrade (especially for cross-version upgrades), clear the browser cache to ensure proper system operation.

Procedure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**System Upgrade**:

1. Navigate to *Auxiliary Apps > Tool Apps* and enter the Software Upgrade section.

.. image:: application/002.png
   :width: 3in
   :align: center

.. centered:: Figure 14.2‑1 System Upgrade Interface

1. Click *Choose File* and select the downloaded 'software.tar.gz' package.

.. important:: 
   The upgrade package must be named 'software.tar.gz'. If the filename differs, the upgrade will fail. Rename the file if necessary.

3. Click *Upload Upgrade Package* to start the upgrade. A progress bar will display the status.

4. When the progress reaches 100%, a message will appear: *"Upgrade successful. Please restart the control box."*

.. image:: application/082.png
   :width: 3in
   :align: center

.. centered:: Figure 14.2‑2 Software Upgrade Success

5. After rebooting the control box, verify the version in *About*.

**Firmware Upgrade**: After the robot enters BOOT mode, upload the upgrade package, select the target slave devices (control box slave, drive slaves 1-6, end-effector slave), and monitor the upgrade status.

.. image:: application/003.png
   :width: 3in
   :align: center

.. centered:: Figure 14.2‑3 Firmware Upgrade

**Slave Configuration File Upgrade**: With the robot disabled, upload the upgrade file, select the target slave devices (control box slave, drive slaves 1-6, end-effector slave), and monitor the upgrade status.

.. image:: application/004.png
   :width: 3in
   :align: center

.. centered:: Figure 14.2‑4 Slave Configuration File Upgrade

Data backup
------------------------

In the menu bar of "Application - Tool App", click "Data backup" to enter the data backup interface, as shown in 3.9-5.

The backup package data includes tool coordinate system data, system configuration files, teaching point data, user programs, template programs and user configuration files. When the user needs to move the relevant data of this robot to another robot, he can use this The function is realized quickly.

.. image:: application/005.png
   :width: 3in
   :align: center

.. centered:: Figure 14.3‑1 Data backup interface

For this function, the following will provide a detailed description of the dynamics configuration, installation method and backup package import related modules.

Backup Package Import Verification Function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add verification function when importing backup packages, and compare key parameters between the backup package and the imported robot. The specific parameters are shown in table below. If these parameters are not set accurately, there will be certain security risks. Only when they are completely consistent, the backup package can be imported normally. If there is inconsistency, an error message will be displayed, as shown below. At this point, it is necessary to check whether the key parameters imported into the robot are consistent with the backup package.

five key parameters to compare table:

.. list-table::
   :widths: 15 40 100
   :header-rows: 0
   :class: sheet-center

   * - **Serial Number**
     - **Key Parameter**
     - **Specific Definition**

   * - 1
     - ROBOT_TYPE
     - Robot Model

   * - 2
     - INSTALL_POS
     - Installation Method

   * - 3
     - INSTALL_YANGLE
     - Base Tilt

   * - 4
     - INSTALL_ZANGLE
     - Base Rotation

   * - 5
     - NEW_TEACH_ENABLE
     - Dynamics Configuration

.. image:: application/064.png
   :width: 6in
   :align: center

.. centered:: Figure 14.3‑2 When the key parameters are inconsistent, the interface will prompt an error

10s data record
------------------------

In the menu bar of "Application - Tool App", click "Data Recording" to enter the 10s data recording function interface.

First, select the record type, which is divided into default parameter record and optional parameter record. The default parameter record is the data automatically set and recorded by the system, and the optional parameter record user can choose the parameter data to be recorded. The maximum number of parameters is 15. After selecting the parameter list, select the record parameter and click the "Move Right" button to configure the parameter into the parameter list. Click "Start Recording" to start recording data, click "Stop Recording" to stop recording data, and click "Download Data" to download the data of the last 10 seconds.

.. image:: application/006.png
   :width: 3in
   :align: center

.. centered:: Figure 14.4‑1 10s data record

Teach point configuration
------------------------------------

In the menu bar of "Application - Tool App", click "Points Config." to enter the teaching point configuration interface.

Before using the button box or other IO signals to record the teaching point function, the user first configures the teaching point name prefix, the upper limit of the number and the teaching method. The name prefix supports two modes: custom prefix and current program name as the prefix. For example, customize the name prefix "P", number upper limit "3", teaching method "robot teaching", record the current end (tool) points of the robot in sequence: P1, P2, P3, and record again will overwrite the previous record points.

.. image:: application/007.png
   :width: 3in
   :align: center

.. centered:: Figure 14.5‑1 Teach point configuration

End-point dot automatic overwrite update Lua program function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

End-point dot configuration
+++++++++++++++++++++++++++++++

1. Click on Auxiliary Applications - Tool Applications - Teaching Point Configuration to access the teaching point configuration page.

.. image:: application/057.png
   :width: 6in
   :align: center

.. centered:: Figure 14.5‑2 Teaching Point Configuration Page

2. Enable the end-point dot function and click on settings. You can use the switch to select the Lua programs that need to be updated for specific positions. 
3. The configuration is complete, with the end-tip dot name prefixed as "test", the numbering limit set to 10, and all Lua programs selected for enabling updates. Close the webApp, and the function remains active.
   
End-button dot automatic update Lua program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++

1. Click the robot end-point dot button.

.. image:: application/058.png
   :width: 3in
   :align: center

.. centered:: Figure 14.5‑3 end-point dot button

2. At this point, the end-tip LED flashes: with the purple light flashing (start) -> blue light on (dot recording and updating in Lua) -> green light on (dot recording completed), and the position information corresponding to the selected Lua program's name is synchronized and updated.

.. image:: application/059.png
   :width: 4in
   :align: center

.. centered:: Figure 14.5‑3 End-tip dot recording and updating Lua program LED changes

3. When the dot recording fails, the end-tip LED flashes: Purple light flashing (start) -> Red light flashing (dot recording failed) -> Green light on (returning to normal).

.. image:: application/060.png
   :width: 4in
   :align: center

.. centered:: Figure 14.5‑4 LED changes when end-tip dot recording fails

Function usage example
+++++++++++++++++++++++++++

1. Click on Auxiliary Applications - Tool Applications - Teaching Point Configuration, customize the prefix to "test", set the number limit to 5, select Robot Teaching as the teaching method, enable the end-point dot function, and click on settings.
2. Activate the Lua program "program1" that requires position updates.

.. image:: application/061.png
   :width: 6in
   :align: center

.. centered:: Figure 14.5‑5 Teaching Point Configuration

3. As shown below, it depicts the "program1" program and its current running trajectory.

.. image:: application/062.png
   :width: 6in
   :align: center

.. centered:: Figure 14.5‑6 Program1 program and its current running trajectory

4. Switch the page to manual mode, move the robot to a new position, click the end-tip dot button, and wait for the end-tip LED to complete its flashing sequence: Purple light flashing (start) -> Blue light on (dot recording and updating in Lua) -> Green light on (dot recording completed), at which point the recorded position is labeled as test1.
5. Repeat step 4 to record positions "test2," "test3," "test4," and "test5," completing the recording of 5 points. At this stage, the positions for the "program1" program have been synchronized and updated.
6. Re-run the "program1" program. The motion trajectory will have been updated, and the updated motion trajectory is shown below.

.. image:: application/063.png
   :width: 3in
   :align: center

.. centered:: Figure 14.5‑7 Updated running trajectory

Work origin
------------------------

In the menu bar of "Application - Tool App", click "Home point" to enter the working origin configuration function interface.

This page displays the name and joint position information of the work origin. The work origin is named pHome. Click "Set" to use the current robot pose as the work origin. Click "Move to this point" to move the robot to the work origin. In addition, the configurable option of moving to the origin of the work is added in the DI configuration, and the configurable option of reaching the origin of the work is added in the DO configuration.

.. image:: application/008.png
   :width: 3in
   :align: center

.. centered:: Figure 14.6‑1 Work origin

Terminal LED configuration
--------------------------------------

In the menu bar of "Application - Tool App", click "End-LED" to enter the end LED color configuration function interface.

The configurable LED colors are green, blue and white cyan. Users can configure the LED colors of automatic mode, manual mode and drag mode according to their needs. Different modes cannot be configured with the same color.

.. image:: application/009.png
   :width: 3in
   :align: center

.. centered:: Figure 14.7‑1 Terminal LED configuration

Peripheral protocol
------------------------

In the menu bar of "Application - Tool App", click "Peripheral protocol" to enter the peripheral protocol configuration function interface.

This page is the configuration page for the peripheral protocol, and the user can configure the protocol according to the currently used peripheral.

.. image:: application/010.png
   :width: 3in
   :align: center

.. centered:: Figure 14.8‑1 Peripheral Protocol Configuration

Add a lua interface for reading and writing registers based on Modbus-rtu communication in the program teaching, input register address 0x1000, the number of registers is 50, a total of 100 bytes of data content; hold the register address 0x2000, the number of registers is 50, a total of 100 bytes data content.

::

   ModbusRegRead(fun_code, reg_add, reg_num): read register;

   fun_code: function code, 0x03-holding register, 0x04-input register

   reg_add: register address

   reg_num: number of registers

::

   ModbusRegWrite(fun_code, reg_add, reg_num, reg_value): write register;

   fun_code function code, 0x06-single register, 0x10-multiple registers

   reg_add: register address

   reg_num: number of registers

   reg_value: byte array

::

   ModbusRegGetData (reg_num): Get register data;

   reg_num: number of registers

   Return value description:

   reg_value: array variable

Program sample screenshot:

.. image:: application/011.png
   :width: 3in
   :align: center

.. centered:: Figure 14.8‑2 Modbus-rtu communication lua program example

Main program configuration
------------------------------------------------

In the menu bar of "Application - Tool App", click "Main program" to enter the main program configuration function interface.

The configuration of the main program can be used in conjunction with the DI configuration of the main program startup. The configured main program needs to be trial run first to ensure safety. After configuring the corresponding DI in the robot settings to start the main program signal function, the user can control the DI signal to run the main program. .

.. image:: application/012.png
   :width: 3in
   :align: center

.. centered:: Figure 14.9‑1 Main program configuration

Drag lock
------------------------

In the menu bar of "Application - Tool App", click "Drag locking" to enter the drag teaching lock configuration function interface.

For drag teaching, the function of locking degrees of freedom is added. When the drag teaching function switch is set to the enabled state, the parameters of each degree of freedom will take effect when the user drags the robot. For example, when the parameter is set to X:10, Y:0, Z:10, RX:10, RY:10, RZ:10, dragging the robot in the drag mode can restrict the robot to move only in the Y direction, if necessary Keep the posture of the robot unchanged while dragging, and only move in the X, Y, and Z directions. You can set X, Y, and Z to 0, and RX, RY, and RZ to 10.

.. image:: application/013.png
   :width: 3in
   :align: center

.. centered:: Figure 14.10‑1 Drag teach lock configuration
   
Force Sensor-Assisted Dragging with Normal Collision Protection Triggering
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Overview
+++++++++++++++++++++++

Currently, FR robots cannot trigger collision protection during force sensor-assisted dragging. This enhancement enables collision protection during force sensor-assisted dragging to improve robot safety and reduce operational risks.

Collision Protection
++++++++++++++++++++++++++++++++++++++++++++++

**Step1**: Click "Auxiliary Applications" -> "Tool Applications" -> "Drag Lock" to enter the force sensor-assisted lock configuration interface. Set both "Status Switch" and "Collision Detection" to ON, as shown below.

.. image:: application/080.png
   :width: 4in
   :align: center

.. centered:: Figure 14.10‑2 Configuring Force Sensor-Assisted Lock

**Step2**: Drag the robot. During robot movement, apply external force to the joints to trigger collision protection. The web interface will display the error "Force Sensor-Assisted Dragging Collision Fault", and provides quick recovery/disable options for force sensor-assisted dragging via the web interface, as shown. Click "Recover" to clear the error and resume force sensor-assisted dragging; click "Disable" to clear the error and keep force sensor-assisted dragging disabled.

.. image:: application/081.png
   :width: 3in
   :align: center

.. centered:: Figure 14.10‑2 Collision Triggering During Force Sensor-Assisted Dragging

.. note:: During force sensor-assisted dragging, the robot itself is in a stopped state. During dragging, there is a difference between joint torque commands and feedback. It is recommended to set the collision level to level 7 or higher. Setting the collision level too low may cause false collision errors during dragging.

Smart Tool
------------------------

In the menu bar of "Application - Tool App", click "Smart Tool" to enter the Smart Tool configuration function interface.

Configure the A-E keys and IO keys in sequence. After the Smart Tool configuration is completed, the task manager internally maintains the function corresponding to each button. When a button is detected to be pressed, the function corresponding to the button is automatically executed.

A~E key function:

-  **Movement instructions**:When selecting PTP, LIN, or ARC motion instructions, you need to enter the corresponding point speed. After the configuration is successful, a new relevant motion instruction is added to the teaching program. When configuring the ARC motion instruction, you need to configure the PTP/LIN instruction first.
  
-  **DO output**:When "DO Output" is selected, a drop-down box is displayed to select output DO0 - DO7 options.
  
.. image:: application/014.png
   :width: 3in
   :align: center

.. centered:: Figure 14.11‑5 Smart Tool Configuration (A~E key)

IO key function:

-  **IO signal configuration**: The drop-down box can select DO0⁓DO7 options, CO0⁓CO7 options, End-DO0, End-DO1 and extended IO (Aux-DO0⁓Aux-DO127);

-  **Combined instructions**: After selecting "IO Signal", the "Welding options" and "Point Speed" configuration items are displayed under specific conditions, and different program instructions are generated.

.. important::
   -  When the IO signal is configured as DO0~DO7 or CO0~CO7 ("Arcing" is not configured), the program adds 'SetDO'; at this time, "Welding options" and "Point speed" are hidden.
   -  When the IO signal is configured as End-DO0, End-DO1, the program adds 'SetToolDO';at this time, "Welding options" and "Point speed" are hidden.
   -  When the IO signal is configured as extended IO ("Welder starting arc" is not configured) , the program adds 'SetAuxDO'; at this time, "Welding options" and "Point speed" are hidden.
   -  When the IO signal is configured as CO0~CO7 (configuration "Arcing"), when "Welder starting arc" is "None", the program adds 'SetDO'; at this time, "Welding options" and "Point speed" are hidden.
   -  When the IO signal is configured as extended IO (configuration "Welder starting arc"), " When the welding machine selection is "None", the program adds 'SetAuxDO'; at this time, the "Welding options" and "Point Speed" are hidden.
   -  When the IO signal is configured as CO0~CO7 (configuration "Arcing") or extended IO (configuration "Welder starting arc"), when "welding machine selection" is "welding", press the program for the first time to add 'ARCStart', and the second time The program adds 'ARCEnd', the third time the program adds 'ARCStart', the fourth time the program adds 'ARCEnd', and the above operations are repeated alternately; at this time, the "Welding options" and "Point Speed" are hidden.
   -  When the IO signal is configured as CO0~CO7 (configuration "Arcing") or expanded IO (configured "Welder starting arc"), and the "welding machine selection" is "LIN + Welding", press the program for the first time to add 'LIN' and 'ARCStart'. The second program adds 'LIN' and 'ARCEnd', the third program adds 'LIN' and 'ARCStart', the fourth program adds 'LIN' and 'ARCEnd', and the above operations are repeated alternately; at this time, "Welding options" and "Point Speed" are displayed.
   -  When the IO signal is configured as CO0~CO7 (configuration "Arcing") or extended IO (configuration "Welder starting arc"), and the "welding machine selection" is "LIN + Welding + Swing", press the program for the first time to add 'LIN', 'ARCStart' and 'WeaveStart', the second program adds 'LIN', 'ARCEnd' and 'WeaveEnd', the third program adds 'LIN', 'ARCStart' and 'WeaveStart', the fourth program adds 'LIN', 'ARCEnd' and 'WeaveEnd', and the above operations are repeated alternately; at this time, the "Welding options" and "Point Speed".

.. image:: application/015.png
   :width: 4in
   :align: center

.. centered:: Figure 14.11‑6 Smart Tool Configuration (IO key)

.. Force sensor assisted drag function settings
.. ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. Under "DI Configuration" in "Robot Settings", click on the different DI drop-down boxes to configure auxiliary drag on, auxiliary off and auxiliary drag on/off

.. - The control box DI function is configured with a force sensor drag function, and the force sensor can be dragged directly through the control box DI input.
.. - The DI function at the end of the robot is equipped with a force sensor drag function, and the force sensor can be dragged through the end DI input.
.. - Based on TPD trajectory recording in robot drag mode, TPD trajectory recording in force sensor-assisted drag mode is added to achieve smoother TPD trajectory dragging.

.. .. image:: application/271.png
..    :width: 4in
..    :align: center

.. .. centered:: Figure 4.9-35 Force sensor assisted drag function settings

.. .. important:: When the force sensor drag-on state is detected, the robot switches to the force sensor drag state; when the force sensor drag-off DI state is detected, the force sensor drag state is turned off;

SmartTool + Force Sensor Combination
------------------------------------------------

In the "Initial - Peripherals - End Tool" menu bar, click "Adapted device" to enter the terminal peripheral configuration interface.

Select "Extended IO Device" as the device type, and the extended IO device configuration information is divided into manufacturer, type, software version and mounting location. Different manufacturers correspond to different types.The current manufacturers are NSR and FR.

Users can configure corresponding device information according to specific production needs. After successful configuration, the device information table is displayed. If the user needs to change the configuration, he can first select the corresponding number, click the "Clear" button to clear the corresponding information, and reconfigure the device information according to needs. 

.. important:: Before clicking Clear Configuration, the corresponding device should be in an inactive state.

.. image:: application/016.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑1 NSR interface

.. image:: application/017.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑2 FR interface

NSR
~~~~~~~~~

The corresponding type of NSR are: SmartTool.

1. Hardware installation

1)Disassemble the SmartTool handle, take out the middle tooling, and install it at the end of the robot.

.. image:: application/018.png
   :width: 3in
   :align: center

.. centered:: Figure 14.12‑3 Install the tooling in the middle of the SmartTool handle

2)After the tooling is installed, splice the SmartTool handle. After the splicing is successful, connect the connecting cable to the end of the robot.

.. image:: application/019.png
   :width: 3in
   :align: center

.. centered:: Figure 14.12‑4 SmartTool handle installed successfully

2. Device information configuration

.. important:: Please ensure that your SmartTool handle has been fixedly installed on the end of the robot and properly connected to the end of the robot.

1)Click the Smart Tool function menu in the auxiliary application to enter this function configuration page. Customize the functions of each button on the end handle according to your needs, including (New Program, Save Program, PTP, Lin, ARC, Weaving Start, Weaving end and IO port);

.. image:: application/020.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑5 SmartTool handle button function configuration interface

2)After the SmartTool handle button function configuration is completed, configure the manufacturer of the extended IO device as "NSR", select the "Type", "Software Version" and "Hang Position" information, and click the "Configure" button.

.. image:: application/021.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑6 NSR device information configuration interface

3)After successfully configuring the device information, view the table data.

3. Application

After the device information is configured successfully, open the "Teaching Simulation - Program Teaching" interface and create a new "testSmartTool.lua" program. Press the SmartTool handle buttons as needed (key function configuration example: A button - PTP, B button - LIN, C button - ARC, D button - create a new program, E button - save the program, IO button - CO0 ), at this time the robot receives feedback and performs corresponding operations on the program. The teaching program is as shown below:

.. image:: application/022.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑7 testSmartTool.lua program that presses the A key

.. image:: application/023.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑8 testSmartTool.lua program that presses the B key

.. image:: application/024.png
   :width: 4in
   :align: center

.. image:: application/025.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑9 testSmartTool.lua program that presses the C key

.. image:: application/026.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑10 testSmartTool.lua program that presses the D key

.. image:: application/027.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑11 testSmartTool.lua program that presses the E key

.. image:: application/028.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑12 testSmartTool.lua program that presses the IO key

FR
~~~~~~~~~

The corresponding type of FR is "SmartTool" and is used in combination with force sensors. The collaborative robot can be adapted to three force sensors of XJC, NSR and GZCX. When using different sensors, you only need to load the corresponding communication protocol, as follows :

- SmartTool + XJC-6F-D82(XJC).
- SmartTool + NSR-FT Sensor A(NSR).
- SmartTool + GZCX-6F-75A(GZCX).

1. Hardware installation

1)Install the SmartTool handle on the end of the robot and connect it correctly to the end of the robot (refer to NSR's hardware installation for detailed installation).

2)After the SmartTool handle is installed, install the force sensor (taking Hong Kong Zhichuangxin as an example) at the end of the SmartTool handle, and connect the connecting cable to the SmartTool handle.

.. image:: application/029.png
   :width: 3in
   :align: center

.. centered:: Figure 14.12‑13 GZCX force sensor is installed at the end of SmartTool handle

2. Device Configuration

.. important:: Please make sure that your SmartTool handle has been fixedly installed on the end of the robot and is correctly connected to the end of the robot, and that the force sensor has been fixedly installed on the end of the SmartTool handle and is correctly connected to the SmartTool handle.

1) Configure the SmartTool handle (refer to NSR's SmartTool button function configuration);

2) After the SmartTool handle button function configuration is completed, configure the manufacturer of the extended IO device as "FR", select the "Type", "Software Version" and "Hang Position" information, and click the "Configure" button;

.. image:: application/030.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑14 FR device information configuration interface

3) After successfully configuring the device information, select the configured force sensor and click the "Activate" button to activate the force sensor. After successful activation, click the "Zero Point Correction" button to clear the force sensor and view the table data;

.. image:: application/031.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑15 Force sensor zero calibration

4) According to the current end installation, configure the load data on the "End Load" interface, and configure the tool coordinate data, tool type and installation location on the "Tool Coordinates" interface.

.. image:: application/032.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑16 "End load" configuration

.. image:: application/033.png
   :width: 4in
   :align: center

.. centered:: Figure 14.12‑17 "Tool coordinates" configuration

3. Application

After the device information is successfully configured, the SmartTool button function and the force sensor function can be independently implemented, such as measuring the size and force direction of the force and auxiliary drag locking based on the force sensor.

.. image:: application/034.png
   :width: 6in
   :align: center

.. centered:: Figure 14.12‑18 Measure the magnitude and direction of force

Welding Expert
------------------------

Click the menu bar of "Welding expert" in "Application" to enter the function interface of welding expert library.

Linear Welding
~~~~~~~~~~~~~~~~~~

Click "Linear Welding" under "Weldment Shape" to enter the direct welding guidance interface. On the basis of the configuration of the basic settings of the robot, we can quickly generate a welding teaching program through a few simple steps. It mainly includes the following five steps. Due to the mutual exclusion between functions, the actual steps to generate a welding teaching program are less than five steps.

Step 1, whether to use the extended axis, if the extended axis is used, the related coordinate system of the extended axis needs to be configured and the extended axis should be enabled.

.. image:: application/040.png
   :width: 3in
   :align: center

.. centered:: Figure 14.14‑1 Extended axis configuration

Step 2: Calibrate the starting point, the starting point safety point, the end point, and the ending point safety point. If the extended axis is selected in the first step, the extended axis movement function will be loaded to cooperate with the calibration of relevant points.

.. image:: application/041.png
   :width: 3in
   :align: center

.. centered:: Figure 14.14‑2 Calibration related points

Step 3, choose whether laser is needed, if yes, edit the parameters of the laser positioning command.

.. image:: application/042.png
   :width: 3in
   :align: center

.. centered:: Figure 14.14‑3 Laser positioning configuration

Step 4: Select whether weaving welding is required, and if weaving welding is required, you need to edit the relevant parameters of weaving welding.

.. image:: application/043.png
   :width: 3in
   :align: center

.. centered:: Figure 14.14‑4 Weaving configuration

Step 5, name the program, and automatically open the program in the program teaching interface.

.. image:: application/044.png
   :width: 3in
   :align: center

.. centered:: Figure 14.14‑5 Save program

Arc Welding
~~~~~~~~~~~~~~~~~~

Click "Arc Welding" under "Weldment Shape" to enter the arc welding guidance interface. On the basis of the configuration of the basic settings of the robot, we can quickly generate a welding teaching program through two simple steps. It mainly includes the following two steps.

Step 1: Calibrate the starting point, the starting point safety point, the arc transition point, the end point and the end point safety point.

.. image:: application/045.png
   :width: 3in
   :align: center

.. centered:: Figure 14.14‑6 Calibration point

Step 2: Name the program and automatically open the program in the program teaching interface.

.. image:: application/046.png
   :width: 3in
   :align: center

.. centered:: Figure 14.14‑7 Save program

Multi-layer welding
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When the weld leg size is greater than 10mm, the multi-layer multi-pass welding function is usually adopted. This function can configure the welding program templated, add the arc tracking function to the first welding process of multi-layer multi-pass welding, and correct the weld deviation in the subsequent multi-pass linear welding process, so as to improve the weld quality.

The operation process of arc tracking multi-layer and multi-pass welding function is as follows:

1) Set the tool coordinate system and fill in the tool size and attitude of the welding gun.

.. note::
   The values on the interface are examples only, and the actual tool status shall prevail.

.. image:: application/047.png
   :width: 6in
   :align: center

.. centered:: Figure 14.14-8 Sets the tool coordinate system

2) Click on "Application", select "Welding Expert", and select "Multi-layer welding" in the "Weldment Shape" category.

.. image:: application/048.png
   :width: 6in
   :align: center

.. centered:: Figure 14.14-9 Open the multi-layer welding interface

3) To use the arc tracing function, be sure to turn on the "First Layer Weld Swing Function" switch and configure the corresponding swing parameters.

.. image:: application/049.png
   :width: 6in
   :align: center

.. centered:: Figure 14.14-10 Turn on the first layer welding swing function

4) Click the "Configure" button to edit the swing parameters.

.. note::
   If arc tracking is required to compensate left and right, only the "triangle wave swing" and "sine wave swing" types can be selected, the swing frequency shall not be less than 0.5Hz, the swing amplitude shall not be less than 3mm, the waiting time for the swing left and right shall be consistent, and the swing azimuth angle shall be 0.

.. image:: application/050.png
   :width: 6in
   :align: center

.. centered:: Figure 14.14-11 Configure the swing parameters

5) Turn on the "Arc Tracking Function" switch, edit the corresponding up-down and left-right compensation parameters, and then click "Next" to enter the multi-layer multi-pass welding setting page.

.. note::
   The arc tracking parameters are configured according to the actual welding situation, refer to the "Arc Tracking Function Operation Manual" or contact relevant technicians.

.. image:: application/051.png
   :width: 6in
   :align: center

.. centered:: Figure 14.14-12  Configure arc tracing parameters

6) Here the "welds" is the welding start position; "X+ point" is a point in the X+ direction of the welding point relative to the custom offset coordinate system; "Z+point" is a point in the Z+ direction of the welding point relative to the custom offset coordinate system; The "Safety Point" is the transition point from the completion of the previous weld to the start of the next weld. After the teaching and setting is complete, click "Next" to select the relevant location of the weld end point.

.. image:: application/052.png
   :width: 6in
   :align: center

.. centered:: Figure 14.14-13 Multi-layer welding line start point position setting

7) 6.Select "Straight Point", where "Weld Point" is the end position of the weld; "X+ point" is a point in the X+ direction of the custom offset coordinate system relative to the "weld point"; The Z+ point is a point in the Z+ direction of the custom offset coordinate system relative to the Weld Point. After the teaching and setting is completed, click "Next" to set the multi-layer and multi-pass welding parameters.

.. image:: application/053.png
   :width: 6in
   :align: center

.. centered:: Figure 14.14-14 Multi-layer weld line end point position setting

8) On this page, you can set the number of multi-layer welds and their distribution locations. Click the "On/Off" box in the parameter table to select the corresponding value of the active multi-layer multi-pass weld, and fill in the "X", "Z" and "B" columns with the desired corresponding offset position and angle in the custom coordinate system. After the setup is complete, click the "Compelete" button to go to the next step.

.. image:: application/054.png
   :width: 6in
   :align: center

.. centered:: Figure 14.14-15 Multi-layer welding parameter setting

9) At this point, all parameters have been configured, enter the name of the program you want to save, and click the "Save" button to automatically produce the corresponding multi-layer multi-pass welding program.

.. image:: application/055.png
   :width: 6in
   :align: center

.. centered:: Figure 14.14-16 Multi-layer welding program generation

10) Click the "Open Program" button to read the LUA program saved in the previous step, as shown in the following figure.

.. image:: application/056.png
   :width: 6in
   :align: center

.. centered:: Figure 14.14-17 Example of an arc tracing multi-layer welding procedure

G-code to Robot Trajectory Planning Function
-----------------------------------------------------------------------------------

Function Overview
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The G-code to robot trajectory planning function converts paths (lines, arcs, full circles, splines) generated by CAD software into G-code files with ".gcode" extension. Spline paths in G-code are composed of multiple small linear segments. The generated G-code files can be imported on the web interface to convert into LUA files.

Function specifications:

(1) The web interface only accepts G-code files with ".gcode" extension. After successful conversion, a LUA file with the same name will be generated. Conversion will fail if a LUA file with the same name already exists.

(2) Currently supports conversion of G0 (rapid move), G1 (linear interpolation), G2 (clockwise arc), and G3 (counterclockwise arc) commands. G0 corresponds to MoveJ, G1 to MoveL, G2/G3 arcs to MoveC, and full circle G2/G3 to Circle commands.

(3) Currently only supports conversion of arcs and circles in XY plane.

(4) Spindle speed S in G-code corresponds to speed in MoveJ (units: rpm to mm/min). Feedrate F corresponds to speed in MoveL, MoveC, and Circle (units: mm/min). G-code speed settings cannot exceed robot's maximum speed.

(5) When executing converted LUA files, set speed percentage to 100% in web interface.

Operation Procedure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Robot pose calculation during path execution is shown below.

.. image:: application/083.png
   :width: 6in
   :align: center

.. centered:: Figure 14.15-1 Robot Pose Calculation Diagram

Where P-xyz represents the reference pose teaching point, and O-xy is the CAD coordinate system. The robot's pose at starting point A is the reference pose. The poses at intermediate point B and endpoint C are calculated based on the angle between Z-axis and CAD plane, and the angle between Z-axis projection and path tangent.

Operation steps:

**Step 1**: Use CAD software with CAM functionality to convert machining paths to G-code, then verify toolpath correctness using simulators like NC Viewer.

**Step 2**: Before conversion, calibrate tool and workpiece coordinate systems. Note workpiece CS must match CAD machine CS.

.. image:: application/084.png
   :width: 6in
   :align: center

.. centered:: Figure 14.15-2 Tool and Workpiece CS Calibration Interface

**Step 3**: Record a reference pose teaching point after calibration. Robot poses along path will be calculated based on this reference.

**Step 4**: Navigate to G-code conversion interface via "Auxiliary Apps" > "Tool Apps" > "G-code Conversion".

.. image:: application/085.png
   :width: 6in
   :align: center

.. centered:: Figure 14.15-3 G-code Conversion Interface

**Step 5**: Click "Select File" to choose G-code file (.gcode only). Select reference pose point from Step 2 - the interface will display current tool/workpiece CS. Click "Convert" - success/failure messages will appear if LUA file already exists.

.. image:: application/086.png
   :width: 6in
   :align: center

.. centered:: Figure 14.15-4 Successful Conversion Interface

.. image:: application/087.png
   :width: 6in
   :align: center

.. centered:: Figure 14.15-5 Failed Conversion Interface

**Step 6**: Open converted LUA file via "Teach Programming" > "Program Editing". Switch robot to auto mode and click start to execute G-code path.

.. image:: application/088.png
   :width: 6in
   :align: center

.. centered:: Figure 14.15-6 Executing Converted LUA File