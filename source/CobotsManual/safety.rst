Safety
===============

.. toctree:: 
   :maxdepth: 6

Stop mode
--------------

Click "Initial" -> "Safety" in the menu bar, and then click the "Stop mode" submenu to enter the configuration interface, set the safe stop mode parameter function.

.. image:: safety/001.png
   :width: 4in
   :align: center

.. centered:: Figure 7.1-1 Safe Stop Mode Configuration

Safe speed
--------------

Click "Initial" -> "Safety" in the menu bar, and then click the "Safe speed" submenu to enter the configuration interface, set the safe speed.

.. note:: TCP manual speed is less than 250mm/s.

.. image:: safety/002.png
   :width: 4in
   :align: center

.. centered:: Figure 7.2-1 Safe manual speed configuration

I/O safety
--------------

Click "Initial" -> "Safety" in the menu bar, and then click the "I/O safety" submenu to enter the configuration interface.

The HMI provides the setting of the safety status of 16 digital inputs and 16 digital outputs, which can be set to valid or invalid status. When the controller determines that it is in a safe state, the 16 digital inputs and 16 digital outputs are set to a safe state.

.. image:: safety/003.png
   :width: 4in
   :align: center

.. centered:: Figure 7.3-1 DIO safety status configuration

On Linux:
   The I/O safety function is provided in "DIO Safety". The safety function is dual-channel DI or DO. When a safety DI signal is detected or the safety status flag is triggered, DO is output.

.. image:: safety/004.png
   :width: 4in
   :align: center

.. centered:: Figure 7.3-2 DIO safety function configuration

Emergency stop
---------------------

Click "Initial" -> "Safety" in the menu bar, and then click the "Emergency stop" submenu to enter the configuration interface.

Emergency stop types 0, 1a, 1b, 2 can be set, stop time limit can be set, and stop distance limit can be set.

 - Send the control box board through the controller, and the emergency stop type 0 control box board directly cuts off the power;

 - Emergency stop type 1a is to cut off the power supply of the main body after deceleration stop;

 - Emergency stop type 1b is to not cut off the power supply of the main body after deceleration stop, and the main body is disabled;

.. image:: safety/005.png
   :width: 4in
   :align: center

.. centered:: Figure 7.4-1 Emergency stop configuration

Safe Stop Recovery Optional Auto Enable Function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Overview
+++++++++++++++++++++

After experiencing a Category 1b emergency stop, the robot provides two modes for user selection: Manual Enable and Auto Enable. When Manual Enable is selected, the user needs to change the robot's operation mode to Automatic after releasing the emergency stop button, and then manually click the enable button to enable the robot. When Auto Enable is selected, the robot will automatically enable itself after the user releases the emergency stop button.

Operation Procedure
+++++++++++++++++++++++++++

**Step1**: Click the "Initial Setup" -> "Safety" -> "Emergency Stop" button. Select "Category 1b" for the "Stop Type", and set the "Stop Time Limit" and "Stop Distance Limit" parameters according to actual needs. The "Enable Strategy After E-Stop Reset" can be selected as "Manual Enable" or "Auto Enable", as shown in Figure 2-1.

.. image:: safety/025.png
   :width: 6in
   :align: center

.. centered:: Figure 7.4-2 Enable Strategy Setting

**Step2**: When "Auto Enable" is selected, the robot will automatically enable itself after the user releases the emergency stop button. When "Manual Enable" is selected, the user needs to manually click the enable button in Automatic mode after releasing the emergency stop button to enable the robot, as shown in Figure 2-2.

.. image:: safety/026.png
   :width: 6in
   :align: center

.. centered:: Figure 7.4-3 Manual Enable Operation

Protective stop
---------------------

Click "Initial" -> "Safety" in the menu bar, and then click the "Protective stop" submenu to enter the configuration interface.

Protective stop type 0, 1, 2. Protective stop type 0: the control box board directly cuts off the power. Protective stop type 1: the control box board first notifies the controller to control the robot to stop and then the controller feeds back to cut off the power of the control box board. Protective stop type 2: the control box board notifies the controller to control the robot to stop.

.. image:: safety/006.png
   :width: 4in
   :align: center

.. centered:: Figure 7.5-1 Protective shutdown configuration

.. important::
   The safety data status flag and control box carrier board fault feedback are obtained through the Web terminal and the controller status feedback. When the flag bit is 1, the safety data status is abnormal in the WebAPP alarm status. After the control box carrier board fault is obtained, the specific error information is displayed in the WebAPP alarm status according to the error code.

.. image:: safety/007.png
   :width: 4in
   :align: center

.. centered:: Figure 7.5-2 WebAPP alarm status 

Safety plane
---------------------------------

Click "Initial" -> "Safety" in the menu bar, and then click the "Safety plane" submenu to enter the configuration interface.

-  **Safety Plane Configuration**:Click the enable button to enable the corresponding security plane. When the security plane is not configured with a security range, an error will be prompted. Click the drop-down box, select the security plane you want to set, and automatically bring out the security distance (you can not set it, the default value is 0), and then click the "Setting" button to set it successfully.
  
.. image:: safety/008.png
   :width: 4in
   :align: center

.. centered:: Figure 7.6-1 Safety Plane Configuration

-  **SSafety Plane Reference Point Configuration**:After selecting a security plane, four reference points can be set. The first three points are plane reference points, which are used to confirm the plane of the safety wall set. The fourth point is the safety range reference point, which is used to confirm the safety range of the set safety wall.

.. important::
   If the reference point is set successfully, the green light will be on. Otherwise, the yellow light is on. It turns green until the reference point is set successfully. When the four reference points are all set successfully, the safety range can be calculated, and the safety range parameter point status will return to the default after the calculation is successful.

.. image:: safety/009.png
   :width: 4in
   :align: center

.. centered:: Figure 7.6-2 Safe range reference point setting

-  Apply effects: The successfully configured security plane is enabled. Drag the robot, if the TCP at the end of the robot is within the set safety range, the system is normal. If it is outside the set safety range, an error will be prompted.

.. image:: safety/010.png
   :width: 6in
   :align: center

.. centered:: Figure 7.6-3 The effect picture after the security range is set successfully

Daemon
---------------------------------

Click "Initial" -> "Safety" in the menu bar, and then click the "Daemon" submenu to enter the configuration interface.

The user clicks the "function enabled" button to open or close the setting of the daemon. Select "Unexpected Situation" and "Background Program", and click the "Settings" button to configure the parameters of the unexpected situation handling logic.

Enable the security background program and set the unexpected scene and background program. When the user starts to run the program and the unexpected situation matches the set unexpected situation, the robot will execute the corresponding background program to play a role of security protection.

.. image:: safety/011.png
   :width: 4in
   :align: center

.. centered:: Figure 7.7-1 Daemon

Direction limit (Only used in Linux systems)
---------------------------------------------

Click "Initial" -> "Safety" in the menu bar, and then click the "Direction limit" submenu to enter the configuration interface.

Tool direction limit is a protective function that acts on the Cartesian space of the robot tool end to limit the range of motion of the robot end posture, including function enablement settings, reference tool direction settings, and maximum offset angle settings. The maximum offset angle defines the maximum angle limit between the Z axis of the Cartesian coordinate system of the tool end and the reference tool direction, which can usually be understood as a conical space.

.. image:: safety/012.png
   :width: 4in
   :align: center

.. centered:: Figure 7.8-1 Direction limit

Robot limit (Only used in Linux systems)
---------------------------------------------

Click "Initial" -> "Safety" in the menu bar, and then click the "Robot limit" submenu to enter the configuration interface.

Robot limits include momentum and power, where the momentum limit is used to limit the robot's maximum momentum, and the power limit is used to limit the mechanical work done by the robot.

.. image:: safety/013.png
   :width: 4in
   :align: center

.. centered:: Figure 7.9-1 Robot limit

Power detection (Only used in QX systems)
---------------------------------------------

Click "Initial" -> "Safety" in the menu bar, and then click the "Power detection" submenu to enter the configuration interface.

When acting directly on the current loop of the robot (only with the command servoJT), it is used to limit the work done by the robot. When it is detected that the integral of the robot speed and torque exceeds the limit, power protection is performed.

.. image:: safety/014.png
   :width: 4in
   :align: center

.. centered:: Figure 7.10-1 Power detection

Motion Configuration
---------------------------------------------

T-Shaped Velocity Optimization + Blending Smoothing Function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Overview
++++++++++++++++++++++

Performing blending between two trajectory segments can avoid frequent start-stop issues caused by complete stops, thereby improving the robot's motion efficiency.

This function mainly applies to blending between PTP-PTP, LIN-LIN, ARC-ARC, LIN-ARC, and ARC-LIN commands. Blending between other commands is not effective.

Operation Process
++++++++++++++++++++++

Since the operation methods for each command are similar, this manual uses PTP-PTP blending as an example to explain the operation method. This function can be implemented in two ways: using Lua commands or using the motion configuration switch.

Using Lua Commands
*****************************

**Step 1**: Select the teaching points for the PTP function. This manual uses "A0" to "A5" as the names of the teaching points.

**Step 2**: Click "Teaching Program" -> "Program Programming," select the "Point-to-Point" command under "Motion Commands," choose the teaching point in the "Command Edit" section, set the debugging speed, select "Acceleration Smoothing Mode" for motion protection, and set the "Smooth Transition" parameter at the points where smoothing is required.

.. image:: safety/020.png
   :width: 6in
   :align: center

.. centered:: Figure 7.11-1 Blending Command Settings for Acceleration Smoothing PTP

**Step 3**: Generate and run the Lua program to implement PTP-PTP blending. This method only applies the optimized T-shaped velocity to commands between `AccSmoothStart()` and `AccSmoothEnd()`, while using the original T-shaped velocity for other commands.

.. image:: safety/021.png
   :width: 4in
   :align: center

.. centered:: Figure 7.11-2 Typical Program for PTP-PTP Blending Using Lua Commands

Using Motion Configuration Switch
***********************************

**Step 1**: Click "Initial Settings" -> "Safety" -> "Motion Configuration," and turn on the "Acceleration Smoothing Mode" switch.

.. image:: safety/022.png
   :width: 6in
   :align: center

.. centered:: Figure 7.11-3 Acceleration Smoothing Mode Configuration Switch Settings

**Step 2**: Select the teaching points for the PTP-PTP function. This manual uses "A0" to "A5" as the names of the teaching points.

**Step 3**: Click "Teaching Program" -> "Program Programming," select the "Point-to-Point" command under "Motion Commands," choose the teaching point in the "Command Edit" section, set the debugging speed, select "None" for motion protection, and set the "Smooth Transition" parameter at the points where smoothing is required.

.. image:: safety/023.png
   :width: 6in
   :align: center

.. centered:: Figure 7.11-4 Blending Command Settings for Regular PTP

**Step 4**: Generate and run the Lua program to implement PTP-PTP blending. The typical program is the same as a regular PTP program. This method applies the optimized T-shaped velocity to all commands.

.. image:: safety/024.png
   :width: 4in
   :align: center

.. centered:: Figure 7.11-5 Typical Program for PTP-PTP Blending Using Configuration Switch

FIR Adaptive Parameter Function + FIR Pause/Resume Function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Overview
++++++++++++++++++++++

The robot's time-optimal mode parameter adaptive configuration function eliminates the need to manually debug and configure parameters. This function adaptively configures the parameters of the time-optimal mode based on the robot's current operating state, improving debugging efficiency.

Operation Process
++++++++++++++++++++++

The usage of basic robot motion commands (PTP, LIN, and ARC) is similar. This example uses the time-optimal mode PTP motion command as the primary example.

**Step 1**: On the robot's web control interface, navigate to "Initial Settings" -> "Safety" -> "Motion Configuration" to enter the "Motion Configuration" interface.

.. image:: safety/015.png
   :width: 6in
   :align: center

.. centered:: Figure 7.11-6 Motion Configuration Interface

**Step 2**: In the "Motion Configuration" interface, click the "Time-Optimal Mode" switch to enter the "Time-Optimal Mode" interface.

.. image:: safety/016.png
   :width: 3in
   :align: center

.. centered:: Figure 7.11-7 Time-Optimal Mode Interface

.. note:: In the "Parameter Configuration" section of the "Time-Optimal Mode" interface, the "Adjustment Coefficient" can be set from -100 to 100, representing a scaling ratio to control the time-optimal degree of motion commands. The default value is 1.

**Step 3**: Determine the teaching points for the PTP motion. This example uses "A0" to "A5" as the names of the teaching points.

**Step 4**: On the robot's web control interface, navigate to "Teaching Program" -> "Program Programming" to enter the "Motion Commands" interface.

.. image:: safety/017.png
   :width: 2in
   :align: center

.. centered:: Figure 7.11-8 Motion Commands Interface

**Step 5**: In the "Motion Commands" interface, click "Point-to-Point" to enter the "PTP" command editing interface. Select the teaching point from the "Point Name" dropdown, set the desired speed ratio in the "Debugging Speed" section, choose "Stop" in the "At This Point" section, select "No" in the "Offset" dropdown, and choose "None" in the "Motion Protection" section. Then, click "Add."

.. image:: safety/018.png
   :width: 6in
   :align: center

.. centered:: Figure 7.11-9 PTP Motion Command Editing Interface

**Step 6**: In the "PTP" motion command editing interface, click "Apply" to automatically generate the corresponding Lua program.

.. image:: safety/019.png
   :width: 4in
   :align: center

.. centered:: Figure 7.11-10 Typical Time-Optimal Mode PTP Motion Lua Program

.. note:: 
   The typical time-optimal mode PTP motion Lua program is the same as a regular PTP motion Lua program, except that the "Time-Optimal Mode" function is enabled in Step 2.

   When the "Time-Optimal Mode" function switch is enabled, all basic robot motion commands (PTP, LIN, and ARC) operate in time-optimal mode. Disabling the switch restores the commands to their basic state.
   The "Acceleration Smoothing Mode" function switch cannot be enabled simultaneously in this interface.