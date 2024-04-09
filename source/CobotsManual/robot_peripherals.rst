Peripheral
=============

.. toctree:: 
  :maxdepth: 5

Gripper Peripheral Configuration
------------------------------------

Gripper program teaching steps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step1**: Select the "End Peripheral Configuration" button in the user peripheral configuration interface, and select "Gripper Device" for the device type. The configuration information of the gripper is divided into gripper manufacturer, gripper type, software version and mounting location. Specific production requirements to configure the corresponding jaw information. If the user needs to change the configuration, first select the corresponding gripper number, click the "Clear" button to clear the corresponding button, and reconfigure according to the needs;

.. figure:: robot_peripherals/001.png
   :align: center
   :width: 3in

.. centered:: Figure 5.1-1 Gripper Jaw Configuration

.. important:: 
	The corresponding gripper should be inactive before clicking Clear Configuration.

**Step2**: After the gripper configuration is completed, the user can view the corresponding gripper information in the gripper information table at the bottom of the page. If configuration errors are found, click the "Reset" button to reconfigure the grippers;

.. figure:: robot_peripherals/002.png
   :align: center
   :width: 3in

.. centered:: Figure 5.1-2 Gripper configuration information

**Step3**: Select the configured gripper and click the "Reset" button. After the page pops up and the command is successfully sent, click the "Activate" button to check the activation status in the gripper information table to determine whether the activation is successful;

.. important::
	When the gripper is activated, the gripper must not have a gripping object

**Step4**: Select the "Gripper" command in the program teaching command interface. In the gripper command interface, the user can select the number of the gripper to be controlled (the gripper that has been configured and activated), and set the corresponding opening and closing state, opening and closing speed, and the maximum opening and closing torque that have been waiting for the gripper to move. time. After completing the settings, click Add Application. Additionally, gripper activation and reset commands can be added to deactivate/reset the gripper while running a program.

.. figure:: robot_peripherals/003.png
   :align: center
   :width: 3in

.. centered:: Figure 5.1-3 Gripper Command Edit

Gripper program teaching
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - PTP(template2,100,-1,0)
     - #Waiting for pinch point
   * - 2
     - PTP(template1,100,-1,0)
     - #Pinch point
   * - 3
     - MoveGripper(1,255,255,0,1000,0)
     - #Clamping jaws closed
   * - 4
     - PTP(template2,100, -1,0)
     - /
   * - 5
     - PTP(template3,100, -1,0)
     - #Waiting for placement point
   * - 6
     - PTP(template3,100, -1,0)
     - #Placement point
   * - 7
     - MoveGripper(1,0,255,0,1000,0)
     - #Clamping jaws open

Spray gun peripheral configuration
-------------------------------------

Spray gun peripheral configuration steps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step1**: Select the "Spray Gun Configuration" button in the user peripheral configuration interface, and the user can quickly configure the DO required for spraying through the one-key configuration button of the spraying function (the default configuration DO10 is spraying start and stop, and DO11 is spraying cleaning gun). Users can also customize DO according to their own needs in the "IO Configuration" interface;

.. important:: 
	Before using the spraying function, it is necessary to establish the corresponding tool coordinate system first, and apply the established tool coordinate system during program teaching.

**Step2**: After the configuration is complete, click the four buttons "Start Spraying", "Stop Spraying", "Start Cleaning the Gun" and "Stop Cleaning the Gun" to debug the spray gun;

.. figure:: robot_peripherals/005.png
   :align: center
   :width: 3in

.. centered:: Figure 5.2-1 Gun Configuration

**Step3**: Select the "spray" command on the program teaching command interface. According to the specific program teaching requirements, add and apply four commands "start spraying", "stop spraying", "start cleaning the gun" and "stop cleaning the gun" in the corresponding places.

.. figure:: robot_peripherals/006.png
   :align: center
   :width: 3in

.. centered:: Figure 5.2-2 Spray Gun Command Editing

Spray program teaching
~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - Lin(template1,100,-1,0,0)
     - #Start spraying point
   * - 2
     - SprayStart()
     - #Start painting
   * - 3
     - Lin(template2,100,-1,0,0)
     - #Spray path
   * - 4
     - Lin(template3,100,-1,0,0)
     - #Stop spraying point
   * - 5
     - SprayStop()
     - #Stop spraying
   * - 6
     - Lin(template4,100,-1,0,0)
     - #Gun cleaning point
   * - 7
     - PowerCleanStart()
     - #Start to clean the gun
   * - 8
     - WaitTime(5000)
     - #Cleaning time ms
   * - 9
     - PowerCleanStop()
     - #Stop gun cleaning

Peripheral configuration of welding machine
---------------------------------------------

Peripheral configuration of welding machine
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step1**: In the user peripherals configuration interface, select the "welding machine configuration" button, and users can configure the IO buttons of the welding machine by selecting the IO type and "Controller I/O" to quickly configure the DI and DO required by the welding machine. The default configuration is as follows:

.. list-table:: 
   :widths: 15 40
   :header-rows: 1
   :align: center

   * - IO signal
     - Function

   * - DI12
     - Arc start success signal

   * - DO9
     - Air supply signal

   * - DO10
     - Arc starting signal

   * - DO11
     - Jog wire feeding

   * - DO12
     - Reverse wire feeding

   * - DO13
     - JOB selection 1

   * - DO14
     - JOB selection 2

   * - DO15
     - JOB selection 3

Users can also customize the configuration in the "IO Configuration" interface according to their own needs; or select "Extended I/O". 

Extended configuration IO:

- configure the "Welder preparation" and "Arc start success" of the extended DI;
- the "Welder starting arc", "Gas detection", "Forward" and "Reverse" of the DO Send wire.

.. figure:: robot_peripherals/008.png
   :align: center
   :width: 3in

.. centered:: Figure 5.3-1 I/O configuration(Controller I/O)

.. figure:: robot_peripherals/065.png
   :align: center
   :width: 3in

.. centered:: Figure 5.3-2 I/O configuration(Extended I/O)

.. important:: 
	Before using the welding machine function, it is necessary to establish the corresponding tool coordinate system first, and apply the established tool coordinate system during program teaching. The welder function is often used in conjunction with a laser tracking sensor.

**Step2**: After the configuration is complete, select the number, set the waiting time, and click the six buttons of "arc end", "arc start", "gas", "gas off", "forward wire feed" and "reverse wire feed" to proceed welding machine debugging;

.. figure:: robot_peripherals/008.png
   :align: center
   :width: 3in

.. centered:: Figure 5.3-3 Welding machine configuration

**Step3**: Select the "Weld" command on the program teaching command interface. According to the specific program teaching requirements, add and apply the "End arc", "Arc", "Gas OFF", "Gas ON", "Stop forward", "Forward", "Stop reverse" and "Reverse" instructions in the corresponding places.

.. figure:: robot_peripherals/009.png
   :align: center
   :width: 3in

.. centered:: Figure 5.3-4 Welding Machine Command Editing

Welding program teaching
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   
   * - 1
     - ARCEnd(0,0,10000)
     - End arc

   * - 2
     - ARCStart(0,0,10000)
     - Arc

   * - 3
     - SetAspirated(0,0)
     - Gas OFF

   * - 4
     - SetAspirated(0,1)
     - Gas ON

   * - 5
     - SetForwardWireFeed(0,0)
     - Stop forward

   * - 6
     - SetForwardWireFeed(0,1)
     - Forward

   * - 7
     - SetReverseWireFeed(0,0)
     - Stop reverse

   * - 8
     - SetReverseWireFeed(0,1)
     - Reverse

Arc interruption parameter configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Arc interruption parameter configuration can obtain and set arc interruption detection enable and confirmation duration.

Welding interruption and resumption parameter configuration can obtain and set the welding interruption and reconnection enable, the weld bead overlap distance, the speed of returning to the arc starting point from the current position, and the operating mode.

.. figure:: robot_peripherals/062.png
   :align: center
   :width: 3in

.. centered:: Figure 5.3-5 Arc interruption parameter configuration

When a welding interruption occurs, a warning prompts that the current welding has been interrupted, and "exit interruption" and "interruption recovery" operations can be performed.

Sensor Peripheral Configuration
---------------------------------

Sensor Peripheral Configuration Steps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step1**: Select the "Sensor Configuration" button in the user peripheral configuration interface. This section takes the end of the robot as an example.

- The user first sets the maximum difference. The maximum difference of the sensor scanning deviation points is recommended to be set to 4 by default. Data processing is based on actual use. The scene selects raw data or YZ data.
- The controller IP defaults to 192.168.57.2, the sensor IP can be configured on the same network segment, the port is 5020, and the sampling period is recommended to be 25. The communication protocol is currently adapted to the communication protocols of Ruiniu, Chuangxiang and Quanshi, just load the corresponding protocol.
- After the loading is complete, the sensor can be tested by pressing the "Sensor On" and "Sensor Off" buttons.

.. figure:: robot_peripherals/011.png
   :align: center
   :width: 3in

.. centered:: Figure 5.4-1 Laser tracking sensor IP configuration

.. important:: 
	1. Before using the sensor function, it is necessary to establish the corresponding tool coordinate system first, and apply the established tool coordinate system during program teaching. The welder function is usually used with sensors.
	2. The maximum difference of the sensor is the maximum deviation between the laser scanning weld position at the previous moment and the current moment, the range is [0~10], the unit is mm, and the recommended value is 4.

**Step2**: Calibration sensor reference point.

In the tool coordinate system setting function, we calibrate the sensor type tool and use the six-point method to configure the sensor coordinate system.

- Select a fixed point in the robot workspace, move the sensor center point to the selected point from three different angles, and set points 1, 2, and 3 respectively.
- Move the sensor center point vertically directly above the selected point and record point 4.
- Move the sensor center point from a fixed point to a point in the X-axis direction of the sensor coordinate system, and set point 5. 
- Return to the fixed point, move vertically upwards, and move the sensor center point from the fixed point to a point in the Z-axis direction of the sensor coordinate system, and set it as point 6.
- Click Calculate to get the pose of the sensor tool, and click Apply to complete. If you do not want to apply the results, click the "Cancel" button to recalibrate.

.. figure:: robot_peripherals/012.png
   :align: center
   :width: 3in

.. centered:: Figure 5.4-2 Reference Point Configuration - Six Point Method

**Eight point method**: In the tool coordinate system setting function, we calibrate the sensor type tool, use the eight-point method to configure the sensor coordinate system, select the eight-point method.

- Move the sensor laser line to coincide with the calibration line on the calibration board, and keep the sensor and the calibration line as close as possible Closer distance and the calibration point is recognized, record point 1.
- Move about 20mm in the -y/+y direction, adjust the robot so that the laser recognizes the calibration point, record point 2.
- Move about 20mm in the -x/+x direction, and adjust the robot so that the laser Recognize the calibration point, record point 3.
- Move about 20mm in the -y/+y direction, adjust the robot so that the laser recognizes the calibration point, record point 4.
- Move about 5mm in the -rx direction, adjust the robot so that the laser recognizes the calibration point, and record the point 5.
- Move about 5mm in the -ry direction, adjust the robot so that the laser recognizes the calibration point, record point 6.
- move about 5mm in the -rz direction, adjust the robot so that the sensor recognizes the calibration point, record point 7.
- Move about 5mm in the -rz direction, and adjust the robot so that Laser recognition to the calibration point, record point 8.
- Click Calculate to get the sensor pose, click Apply to complete. If you do not want to apply the results, click the "Cancel" button to recalibrate.

.. figure:: robot_peripherals/013.png
   :align: center
   :width: 3in

.. centered:: Figure 5.4-3 Reference Point Configuration - Eight Points Method

**Five point method**: In the tool coordinate system setting function, we calibrate the sensor type tool and use the five-point method to configure the sensor coordinate system.

- First, determine a fixed point, align the end of the tool with this point, record point 1.
- And then adjust the posture of the robot so that the laser recognizes Record fixed points, respectively record points 2 to 5, and note that the attitude change needs to be as large as possible.
- Click Calculate to get the sensor pose, click Apply to complete. If you feel that the accuracy is not as expected, click the "Cancel" button to recalibrate.

.. figure:: robot_peripherals/014.png
   :align: center
   :width: 3in

.. centered:: Figure 5.4-4 Reference Point Configuration - Five Points Method

.. figure:: robot_peripherals/071.png
   :align: center
   :width: 3in

.. centered:: Figure 5.4-5 Five Points Method - Sensor output value

.. figure:: robot_peripherals/072.png
   :align: center
   :width: 3in

.. centered:: Figure 5.4-6 Five Points Method - Accuracy

Laser sensor tracking function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Command description: Select the "Laser" command on the program teaching command interface. It integrates laser-related instructions. According to the specific program teaching requirements, add instructions in the corresponding places. Refer to the program example below.

.. figure:: robot_peripherals/015.png
   :align: center
   :width: 3in

.. centered:: Figure 5.4-5 Laser Tracking Command Editing

program example: 

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - LTLaserOn(2)
     - #Turn on the sensor
   * - 2
     - PTP(template1,100,-1,0)
     - #Sensor starting point
   * - 3
     - LTSearchStart(1,20,100,10000,2)
     - #Start searching
   * - 4
     - LTSearchStop()
     - #Stop searching
   * - 5
     - Lin(seamPos,20,-1,0,0,0)
     - #Start of weld
   * - 6
     - LTTrackOn(2)
     - #Laser tracking
   * - 7
     - ARCStart(0,10000)
     - #Arc striking of welder
   * - 8
     - Lin(SeamEnd11,10-1,0,0)
     - #End of weld
   * - 9
     - ARCEnd(0,10000)
     - #Arc extinguishing of welder
   * - 10
     - LTTrackOff()
     - #Sensor tracking off
   * - 11
     - LTLaserOff()
     - #Turn off the sensor

Laser sensor trajectory reproduction function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Command description: Select the "LT-Rec" command on the program teaching command interface. This instruction is mainly used for obtaining the starting point and end point of the laser recognition path and reappearing the trajectory. According to the specific program teaching requirements, add instructions in the corresponding places. Refer to the program example below.

.. figure:: robot_peripherals/017.png
   :align: center
   :width: 3in

.. centered:: Figure 5.4-6 Editing of track reappearance command

Program example:

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - PTP(template1,100,-1,0)
     - #Move to starting point
   * - 2
     - LaserSensorRecord(2,0,30)
     - #Sensor start recording
   * - 3
     - Lin(template2,100,-1,0,0)
     - #Move to the end
   * - 4
     - LaserSensorRecord(0,0,30)
     - #Stop recording
   * - 5
     - pos={}
     - #Initialize array
   * - 6
     - pos=GetWeldTrackingRecordStartPos(0,30)
     - #Start point for obtaining laser record
   * - 7
     - If type(pos) == “table” then
     - #Judge data type
   * - 8
     - LaserPTP(#pos,pos)
     - #Move to the starting point of the laser track
   * - 9
     - end
     - /
   * - 10
     - LaserSensorRecord(3,0,30)
     - #Set Recurrence Track
   * - 11
     - MoveLTR()
     - #Start reproducing track
   * - 12
     - LaserSensorRecord(0,0,30)
     - #End

Extended Axis Peripheral Configuration
-----------------------------------------

Select the "Extended Axis" button in the user peripheral configuration interface to enter the extended axis interface, and select the combination method to configure the corresponding extended axis peripherals. Combination methods are divided into: Controller + PLC (UDP) and Controller + Servo drive (485).

Controller + PLC（UDP）
~~~~~~~~~~~~~~~~~~~~~~~~~~

Configuration steps
++++++++++++++++++++++++

**Step1**：First configure the expansion axis UDP communication. Set parameters such as IP address, port number, communication cycle, packet loss detection cycle, and number of packet losses. The reconnection cycle and the number of reconnections can only be configured after the automatic reconnection switch is turned on when communication is interrupted.

- IP address: Custom ip address
- Port number: defined according to actual situation
- Communication cycle: defined according to actual situation, unit ms
- Packet loss detection communication cycle: 10 ~ 1000 ms
- Number of packet losses: 1 ~ 100
- Communication interruption confirmation time: 0 ~ 500 ms
- Automatic reconnection after communication interruption: On/Off
- Reconnection period: 1 ~ 1000 ms
- Number of reconnections: 1 ~ 100

.. figure:: robot_peripherals/021.png
   :align: center
   :width: 3in

.. centered:: Figure 5.5-1 Extended axis communication configuration

.. important:: 
   1. After setting the communication disconnection confirmation time, when the communication abnormality exceeds this time period, the communication disconnection will be confirmed and an error will be reported.
   2. After the UDP communication is disconnected, a UDP disconnection error is triggered (can be reset). You can click the clear warning message button and the UDP communication is re-established.

**Step2**: Select the extended axis number 1, and click the "Parameter Configuration" button to enter the right interface. Set the axis type, axis direction, running speed, acceleration, forward limit, reverse limit, lead, encoder resolution, starting point offset, manufacturer, model and mode, and click Configure to complete the configuration.

.. figure:: robot_peripherals/019.png
   :align: center
   :width: 3in

.. centered:: Figure 5.5-2 Extended axis parameter configuration
	
.. important:: 
	Before using the extended axis function, it is necessary to establish the corresponding extended axis coordinate system, and apply the established tool coordinate system during program teaching. The extended axis function is mainly used in conjunction with the welder function and the laser tracking sensor function.

**Step3**: Click the "Zero Setting" button to enter the zero setting pop-up window, as shown in the picture on the right. Set the zero return method, zero search speed, zero hoop speed and axis direction, click the "Setting" button, the extended axis will start to return to zero, the zero return status will be displayed in the blank space below the axis direction, when "zero return completed" appears The prompt indicates that the zero point of the extension axis is set successfully.

.. figure:: robot_peripherals/020.png
   :align: center
   :width: 3in

.. centered:: Figure 5.5-3 Extended axis zero point setting

**Step4**: Select the number of the extended axis whose parameters have been configured, click "Servo Enable", set the running speed, acceleration and the maximum distance of a single run, and test the extended axis for forward rotation and reverse rotation.

.. figure:: robot_peripherals/021.png
   :align: center
   :width: 3in

.. centered:: Figure 5.5-4 Extended Axis Test

**Step5**: The extension axis is usually used in conjunction with the laser sensor. At this time, the laser sensor is usually installed externally. The sensor reference point configuration needs to be calibrated by the three-point method instead of the six-point method used before. Align the center of the tool with the middle point of the bottom of the right cross-section (the side close to the camera), set point 1, align the center of the tool with the middle point of the bottom of the other cross-section, which is the middle point of the left cross-section, set point 2, and set the center of the tool with Move the point to the middle point of the upper edge of the cross-section on the right side of the sensor, set point 3, calculate and save, and click Apply to complete the three-point calibration.

.. figure:: robot_peripherals/022.png
   :align: center
   :width: 3in

.. centered:: Figure 5.5-5 Three-point sensor calibration

**Step6**: Select the "EAxis" command on the program teaching command interface. According to the specific program teaching requirements, add instructions in the corresponding places.

.. figure:: robot_peripherals/023.png
   :align: center
   :width: 3in

.. centered:: Figure 5.5-6 Extended axis command editing

Extended axis with laser tracking welding teaching program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - EXT_AXIS_PTP(1,1,laserstart)
     - #External axis movement laser sensor starting point
   * - 2
     - PTP(laserstart,10,-1,0)
     - #Starting point of robot motion laser sensor
   * - 3
     - LTSearchStart(3,20,10,10000)
     - #Start searching
   * - 4
     - LTSearchStop()
     - #Stop searching
   * - 5
     - EXT_AXIS_PTP(1,1,seamPos)
     - #Start point of external axis movement weld
   * - 6
     - Lin(seamPos,20,-1,0,0,0)
     - #Start point of robot moving weld
   * - 7
     - LTTrackOn()
     - #Laser tracking
   * - 8
     - ARCStart(0,10000)
     - #Arc striking of welder
   * - 9
     - EXT_AXIS_PTP(1,1,laserend)
     - #End point of external axis movement weld
   * - 10
     - Lin( laserend,10,-1,0,0)
     - #End point of robot moving weld
   * - 11
     - ARCEnd(0,10000)
     - #Arc extinguishing of welder
   * - 12
     - LTTrackOff
     - #Laser tracking off

Controller + Servo drive (485)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Hardware cable connection
++++++++++++++++++++++++++++++

Figure 5.5-7 shows the schematic diagram of the electrical interface of the control box of FR robot. Before using RS485 communication to control the servo extension shaft, please connect the RS485 communication interface of the servo driver to the RS485 communication interface on the robot control box.

.. figure:: robot_peripherals/067.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-7 Schematic diagram of electrical interface of FR robot easy manufacturing control box

Take Dynatec servo driver FD100-750C as an example, Figure 5.5-8 is the schematic diagram of the driver panel terminal, Figure 5.5-9 is the definition of the X3A-IN terminal of FD100-750C, when the robot is configured to communicate with the FD100-750C servo extension shaft, Connect the 485-A0 and 485-B0 terminals on the control box to pins 4 and 5 of the X3A-IN terminal of the driver.

.. note:: 
  Please note: you can see a "485" mark on the servo driver panel, the terminal is not open for user use, do not connect your RS485 communication cable to this terminal. 

At the same time, if multiple servo drivers are connected and the driver is the last one in the link, turn on the DIP switch (DIP switch 2) of the RS485 communication interrupt resistance on the panel.

.. figure:: robot_peripherals/068.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-8 FD100-750C driver panel

.. figure:: robot_peripherals/069.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-9 X3A-IN terminal definition of FD100-750C

Communication configuration
++++++++++++++++++++++++++++++

**Step1**: After ensuring that your RS485 communication cable is correctly connected, the robot and servo extension shaft are powered on, open the robot WebApp, click "Initialize", " Peripheral Config" and "Ext axis" successively to open the "Extension axis Configuration" page,as shown in Figure 5.5-10.

.. figure:: robot_peripherals/073.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-10 RS485 expansion axis configuration

**Step2**: On the "Expansion Axis Configuration" page, select the Combination as "Controller + Servo driver(485)" and ID as "1".

.. note:: 
  Please note: When connecting multiple servos, this ID is used to distinguish different servos, we will mention this ID many times later.

The Servo manufacturer is "Dynatec", select the model and the software version, fill in the corresponding Resolution of the servo drive, here is 131072, Fill in the mechanical transmission ratio according to your mechanism model, here is 15.45; Click the "Configure" button.

**Step3**: At this point, we have completed the 485 communication configuration between the robot and the servo drive. You can view the real-time status information of the servo in the "Servo status bar" on the right of the WebApp (Figure 5.5-11).

.. figure:: robot_peripherals/074.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-11 Servo status bar

Now you can perform a certain motion test on the expansion shaft equipment, please follow the following test operations under the premise of ensuring safety.

Servo control mode and enablement
**********************************************

As shown in Figure 5.5-12, in the "Servo drive debugging" bar, select the corresponding servo ID and click the "Set" button on the right; Select the control mode as "Location mode" and click the "Configure" button on the right.

.. note:: 
  Please note: After switching the control mode, the servo needs to be disabled and then enabled to make the servo control mode switch take effect.

**Location Mode** : You can enter a certain motion speed and target position parameters, the servo will move to the target position at the set speed, after moving to the target position, the servo will stop moving.

**Speed Mode** : You can enter a certain target speed, the servo will keep moving according to the target speed set by you until you set the target speed to 0 or enable the servo motor down;

- Click the "Enable" button. In the "Servo Status Bar" on the right of the WebApp, you can observe that the "Enable" status is on, indicating that the servo drive is enabled.
- Click the "Disable" button in the "Servo Drive Debugging" bar, the "Enable" status light in the "Servo status bar" is off, and the servo enable is turned off.

.. figure:: robot_peripherals/075.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-12 Servo driver debugging page

Servo homing
**********************************************

As shown in Figure 5.5-13, the Zero return mode is selected as "current position return to zero", the Zero seeking speed is 5mm/s, and the Zero point gripper speed is 1mm/s.

Click the "Set" button to complete the operation of the current position homing. In the "servo status bar", it can be observed that the current "servo position" is 0.

.. important::
  Please read this manual completely, and then select the homing mode as "negative limit homing" or "positive limit homing" for homing test.

.. figure:: robot_peripherals/076.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-13 Servo homing debugging

Servo motion
**********************************************

Before the actual control of the servo motor movement, please first understand the servo motor "Location mode" and "speed mode", once again remind you:

**Location Mode** : You can enter a certain motion speed and target position parameters, the servo will move to the target position at the set speed, after moving to the target position, the servo will stop moving.

**Speed Mode** : You can enter a certain target speed, the servo will keep moving according to the target speed set by you until you set the target speed to 0 or enable the servo motor down;

As shown in Figure 5.5-14, when the control mode of the expansion axis is switched, the display of "Current control mode" will automatically switch

.. note::
  Please note: After the control mode is switched, the servo must be unenabled and then enabled, so that the control mode switch of the servo will take effect.

If your servo is not currently in Location Mode, switch your servo to Location mode. Enter "target position" as 50mm, running speed as 5mm/s, under the condition of confirming safety, click "Set" button, then the servo motor will move according to the parameters set by you, you can observe the servo position and speed in real time in the "servo status bar".

.. figure:: robot_peripherals/077.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-14 Servo motion debugging (Location mode)

Change the control mode of the servo to "Speed mode", click "Disable", then click "Enable", then the servo switches to speed mode.

.. note:: 
  Please note: when the servo motor is moving, the servo motor can only be stopped by setting the target speed to 0.

Enter the target speed of 5mm/s, click the "Set" button, the servo motor will keep moving at 5mm/s, you can also observe the servo position and speed in the "servo status bar" in real time.

.. figure:: robot_peripherals/078.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-15 Servo motion debugging (speed mode)

Extended axis programming
++++++++++++++++++++++++++++++

**Step1**: Create a new user program "testServo.lua", and select "Peripheral Command".

.. figure:: robot_peripherals/079.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-16 Open peripheral Commands

**Step2**: Click "Eaxis" to open the interface of adding extension axis commands.

.. figure:: robot_peripherals/080.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-17 Opens the Add extension axis commands page

**Step3**: select the Combination s "Controller + servo drive (485)" on the Add extension axis commands page, set the control mode as "Location mode", and click the "Add" button on the right.

.. figure:: robot_peripherals/081.png
   :align: center
   :width: 3in

.. centered:: Figure 5.5-18 Sets the control mode for the extension axis

**Step4**: Flip the Add extension axis commands interface to the bottom, click the "Apply" button, and close the Add extension axis commands interface.

.. figure:: robot_peripherals/082.png
   :align: center
   :width: 3in

.. centered:: Figure 5.5-19 Apply the extension axis commands

**Step5**: At this time, a set of commands to switch the servo control mode will appear in the program "testServo.lua". You can switch the robot to the automatic mode and execute the program.

.. figure:: robot_peripherals/083.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-20 Setup servo control mode program

**Step6**: How to control servo motion through user program? Also open the interface of adding extension axis commands , find the parameter configuration bar, take the Location mode as an example, enter the target position and running speed, and click "Add" button; Flip to the bottom of the Add Extension Axis commands screen, click the "Apply" button, and close the Add extension axis commands screen.

.. figure:: robot_peripherals/084.png
   :align: center
   :width: 3in

.. centered:: Figure 5.5-21 Add Location mode motion commands

**Step7**: Servo motion commands have been added to the "testServo.lua" program: "AuxServoSetTargetPos,50,5 (1)".

  The meanings of the three parameters in the commands function are:

  - 1: Servo number is 1;

  - 50: Target position;

  - 5: Target speed;

.. figure:: robot_peripherals/085.png
   :align: center
   :width: 6in

.. centered:: Figure 5.5-22 Location mode servo motion program

**Step8**: Switch the robot to automatic mode and run the program, at which time your servo will move at a speed of 5mm/s to a position of 50mm.

**Step9**: At this point, we have completed the preliminary configuration and testing of the RS485 control servo extension shaft, and you can write the program of the combination of robot motion and servo motion according to the actual situation.

Example of cooperative motion program between extension axis and robot:

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - AuxServoSetTargetPos(1,50,5)
     - #The extended axis moves to the reset point
   * - 2
     - if(GetDI(8,0) == 1) then
     - #If the CI0 input is valid
   * - 3
     - AuxServoSetTargetPos(1,50,5)
     - #Expansion shaft movement to 50mm
   * - 4
     - PTP(testptp1,100,-1,0)
     - #Robot moves to point testptp1
   * - 5
     - else if(GetDI(9,0) == 1) then
     - #If the CI1 input is valid
   * - 6
     - AuxServoSetTargetPos(1,150,5)
     - #Expansion shaft movement to 150mm
   * - 7
     - PTP(testptp2,100,-1,0)
     - #Robot moves to point testptp2
   * - 8
     - else
     - #If both CI0 and CI1 inputs are invalid
   * - 9
     - AuxServoSetTargetPos(1,300,5)
     - #The expansion shaft moves to 300mm
   * - 10
     - PTP(testptp3,100,-1,0)
     - #Robot moves to testptp3
   * - 11
     - end
     - 

Summary
++++++++++++++++++++++++++++++

.. important:: 
  In summary, there are the following points to note when configuring the RS485 communication between the cooperative robot and the servo expansion axis:
  
  1. Correctly connect the RS485 communication cable between the cooperative robot and the servo driver;
  
  2. Correctly select the control mode of the servo extension axis;
  
  3. After switching the control mode, you must first disable, and then enable the servo, the control mode switch can take effect.

Conveyor Tracking Configuration
-----------------------------------

Conveyor Tracking Configuration Steps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step1**: Select the "Conveyor Belt Tracking" button in the user peripheral configuration interface to enter the conveyor belt tracking configuration interface, click the "Configure Conveyor Belt IO" button to quickly configure the IO required for the conveyor belt function, and then configure the corresponding parameters according to the actual use of the function. Here, there is no visual Take the tracking and grabbing function as an example, you need to configure the conveyor belt encoder channel, resolution, lead, visual matching, select No, and click Configure.

.. figure:: robot_peripherals/025.png
   :align: center
   :width: 3in

.. centered:: Figure 5.6-1 Conveyor configuration

**Step2**: Next, set the grab point compensation value, which is the compensation distance in the three directions of X, Y, and Z, which can be set according to the actual situation during the debugging process.

.. figure:: robot_peripherals/026.png
   :align: center
   :width: 3in

.. centered:: Figure 5.6-2 Conveyor Grab Point Compensation Configuration

**Step3**: Turn on the conveyor belt, move the calibrated object to the defined point A, and stop the conveyor belt. Move the robot, align the sharp point of the calibration rod at the end of the robot with the sharp point of the object to be calibrated, click the start point A button, a dialog box will pop up, displaying the current encoder value and robot pose, and click Calibrate to complete the start point A calibration.

.. figure:: robot_peripherals/027.png
   :align: center
   :width: 3in

.. centered:: Figure 5.6-3 Starting Point A Configuration

**Step4**: Click the reference point button to enter the reference point calibration. When recording the reference point, record the height and attitude of the robot when it is grasping. Every time it tracks, it will track and grasp with the height and attitude area of the recorded reference point. It can be different from the AB point. Click Calibrate to complete the reference point calibration.

.. figure:: robot_peripherals/028.png
   :align: center
   :width: 3in

.. centered:: Figure 5.6-4 Reference point configuration

**Step5**: Turn on the conveyor belt, move the calibrated object to the defined point B, and stop the conveyor belt. Move the robot, align the sharp point of the calibration rod at the end of the robot with the sharp point of the object to be calibrated, click the end point B button, a dialog box will pop up, displaying the current encoder value and robot pose, click the calibration to complete the end point B calibration.

.. figure:: robot_peripherals/029.png
   :align: center
   :width: 3in

.. centered:: Figure 5.6-5 Terminal B configuration

Conveyor belt tracking teaching program
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - PTP(conveyorstart,30,-1,0)
     - #Robot grabbing starting point
   * - 2
     - While(1) do
     - #Loop Grab
   * - 3
     - ConveyorIODetect(10000)
     - #IO real-time detection of objects
   * - 4
     - ConveyorGetTrackData(1)
     - #Object position acquisition
   * - 5
     - ConveyorTrackStart(1)
     - #Conveyor tracking start
   * - 6
     - Lin(cvrCatchPoint,10,-1,0,0)
     - #Robot reaches the grab point
   * - 7
     - MoveGripper(1,255,255,0,10000)
     - #Gripper claw grabs objects
   * - 8
     - Lin(cvrRaisePoint,10,-1,0,0)
     - #Robot lifting
   * - 9
     - ConveyorTrackEnd()
     - #End of belt tracking
   * - 10
     - PTP(conveyorraise,30,-1,0)
     - #Robot arrives at holding point
   * - 11
     - PTP(conveyorend,30,-1,0)
     - #Robot reaches the placement point
   * - 12
     - MoveGripper(1,0,255,0,10000)
     - #Gripper release
   * - 13
     - PTP(conveyorstart,50,-1,0)
     - #The robot returns to the starting point again and waits for the next capture
   * - 14
     - end
     - #End
  
Attitude Adaptive Configuration
----------------------------------

Attitude adaptive configuration steps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step1**: Select the "Tracking attitude configuration" button in the user peripherals configuration interface to enter the attitude adjustment configuration interface, select the plate type and the actual working direction of the robot, adjust the robot attitude, and set the attitude point A, attitude point B and attitude point C respectively, usually A is the attitude point of the plane, B is the attitude point of the rising edge, and C is the attitude point of the falling edge.

.. figure:: robot_peripherals/031.png
   :align: center
   :width: 3in

.. centered:: Figure 5.7-1 Attitude Adjustment Configuration

.. important:: 
	The attitude change between A posture and B posture, A posture and C posture is as small as possible under the condition that the application requirements are met. The posture adaptive function is an auxiliary application function, usually used in conjunction with seam tracking.

**Step2**: Select the "Adjust" command on the program teaching command interface. According to the specific program teaching requirements, add instructions in the corresponding places.

.. figure:: robot_peripherals/032.png
   :align: center
   :width: 3in

.. centered:: Figure 5.7-2 Attitude Adjustment Command Edit

Attitude self-adaptive with extended axis and laser tracking welding teaching program
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - EXT_AXIS_PTP(1,1,laserstart)
     - #External axis movement laser sensor starting point
   * - 2
     - PTP(laserstart,10,-1,0)
     - #Starting point of robot motion laser sensor
   * - 3
     - LTSearchStart(3,20,10,10000)
     - #Start searching
   * - 4
     - LTSearchStop()
     - #Stop searching
   * - 5
     - EXT_AXIS_PTP(1,1,seamPos)
     - #Start point of external axis movement weld
   * - 6
     - Lin(seamPos,20,-1,0,0,0)
     - #Start point of robot moving weld
   * - 7
     - LTTrackOn()
     - #Laser tracking
   * - 8
     - ARCStart(0,10000)
     - #Arc striking of welder
   * - 9
     - PostureAdjustOn(0,PosA,PosC,PosB,1000)
     - #Attitude adaptive adjustment on
   * - 10
     - EXT_AXIS_PTP(1,1,laserend)
     - #End point of external axis movement weld
   * - 11
     - Lin( laserend,10,-1,0,0)
     - #End point of robot moving weld
   * - 12
     - ARCEnd(0,10000)
     - #Arc extinguishing of welder
   * - 13
     - PostureAdjustOff(0)
     - #Attitude adaptive adjustment off
   * - 14
     - LTTrackOff
     - #Laser tracking off

Force/Torque Sensor Peripheral Configuration
-----------------------------------------------

Force/Torque Sensor Configuration Steps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step1**: Select the "End Peripheral Configuration" button in the user peripheral configuration interface, and select "Force Sensor Device" for the device type. The force sensor configuration information is divided into manufacturer, type, software version and mounting location. Configure the corresponding force sensor information. If the user needs to change the configuration, he can first select the corresponding number, click the "Clear" button to clear the corresponding information, and reconfigure according to the needs;

.. figure:: robot_peripherals/034.png
   :align: center
   :width: 3in

.. centered:: Figure 5.8-1 Force/Torque Sensor Configuration

.. important:: 
	The corresponding sensor should be inactive before clicking Clear Configuration.

**Step2**: After the configuration of the force sensor is completed, the user can view the corresponding force sensor information in the information table at the bottom of the page. If a configuration error is found, the user can click the "Reset" button to reconfigure.

.. figure:: robot_peripherals/035.png
   :align: center
   :width: 3in

.. centered:: Figure 5.8-2 Force/Torque Sensor Configuration Information

**Step3**: Select the configured force sensor number and click the "Reset" button. After the page pops up and the command is sent successfully, click the "Activate" button to check the activation status in the force sensor information table to determine whether the activation is successful; in addition, the force sensor will There is an initial value, and the user can select "zero point correction" and "zero point removal" according to the usage requirements. The force sensor zero point correction needs to ensure that the force sensor is horizontal and vertical, and the robot is not equipped with a load.

**Step4**: After the configuration of the force sensor is completed, the sensor type tool coordinate system needs to be configured, and the value of the sensor tool coordinate system can be directly input and applied according to the distance between the sensor and the center of the end tool.

Force/Torque Sensor Load Identification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Select "Force/Torque Sensor Load" in the robot configuration interface to configure.

Specific attitude recognition: clear the end load data, configure the force sensor, establish the sensor coordinate system, adjust the end attitude of the robot to be vertically downward, perform "zero point correction" and install the end load. First, select the corresponding sensor tool coordinate system, adjust the robot so that the sensor and tool are vertically downward, record data, and calculate the quality. Next, adjust the three different postures of the robot, record three sets of data respectively, calculate the center of mass, and click Apply after confirming that it is correct.

**Dynamic identification**: After clearing the end load data and configuring the force sensor, establish the sensor coordinate system, adjust the end posture of the robot to be vertically downward, perform "zero point correction" and install the end load. Click "Identification On", drag the robot to move, and then click "Identification Off", the load result can be automatically applied to the robot.

**Automatic Zero Calibration**: After the sensor records the initial position, it can be automatically zeroed.

.. figure:: robot_peripherals/036.png
   :align: center
   :width: 3in

.. centered:: Figure 5.8-3 Force/Torque Sensor Load Identification

Force/Torque Sensor Assisted Drag
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After the sensor is configured, it can be used with the sensor to better assist the dragging robot. When using it for the first time, you can configure it according to the data in the picture on the right. After the application is completed, you don’t need to enter the drag mode at this time, and you can directly drag the end force sensor to control the robot to move in a fixed posture. (The data in the figure below is a reference standard.)

.. figure:: robot_peripherals/037.png
   :align: center
   :width: 3in

.. centered:: Figure 5.8-4 Force/Torque Sensor Drag Lock

**Adaptive selection**: Turn it on when assembly is required. After turning it on, dragging becomes heavier;

**Inertia Parameter**: Adjust the feel during dragging. The minimum value is 0. It needs to be operated with caution under the guidance of technical personnel.

**Damping parameters**:

-  Translation direction: It is recommended to set the parameter between [100-200];

-  Rotation direction: It is recommended to set the parameters between [3-10], among which the RZ direction setting range is [0.1-5];

-  Effect: When dragging with the help of a sensor, increasing the damping will make it difficult to drag, and reducing the damping will make it too easy to drag the robot (it is recommended not to be too small);

-  The overall range of damping parameters: translation XYZ: [100-1000]; rotation RX, RY: [3-50], RZ: [2-10];

-  The maximum drag force is 50 and the maximum drag speed is 180.

**Stiffness parameter**: All are set to 0;

**Drag force threshold**: Translation XYZ is [5-10]; rotation RX, RY, RZ is [0.5-5];

.. important:: 
  Locking is achieved by increasing the force threshold in the translation direction XYZ or rotation direction RX, RY, RZ.

Force/Torque Sensor Collision Detection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Command description: "FT_Guard" command is a collision detection command. Select the corresponding sensor coordinate system, check the effective torque direction detection, set the current value, the maximum collision threshold and the minimum collision threshold. The normal range of the collision detection condition is (current value-minimum threshold, current value+maximum threshold), set "Open" and "Close" commands are added to the program.

.. figure:: robot_peripherals/038.png
   :align: center
   :width: 3in

.. centered:: Figure 5.8-5 FT_Guard Command Edit

Program example:

.. list-table:: 
   :widths: 15 15 70
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - FT_Guard(1,1,1,1,1,0,0,0,5,0,0,0,0,0,10,0,0,0,0,0,5,0,0,0,0,0)
     - #Force/moment collision detection on                                  
   * - 2
     - PTP(template1,100,-1,0)
     - #Motion command
   * - 3
     - FT_Guard(0,1,1,1,1,0,0,0,5,0,0,0,0,0,10,0,0,0,0,0,5,0,0,0,0,0)
     - #Force/moment collision detection off

Force/Torque Sensor Force Control Motion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Instruction description: The "FT_Control" instruction is a force control motion instruction, which can make the robot move near the set force, and is often used in grinding scenes. Select the corresponding sensor coordinate system, check the effective torque direction detection, set the detection threshold, and the PID proportional coefficient in each direction (generally set p to 0.001), set the maximum adjustment distance (corresponding to X, Y, Z) and maximum adjustment angle (corresponding to RX, RY, RZ), add the "open" and "close" instructions to the program.

.. figure:: robot_peripherals/040.png
   :align: center
   :width: 3in

.. centered:: Figure 5.8-6 FT_Control Command Edit

Program example:

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - FT_Control(1,11,1,0,1,0,0,0,10,0,5,0,0,0,0.001,0,0,0,0,0,0,0,0,10,5)
     - #Force/torque motion control ON
   * - 2
     - Lin(template3,100,-1,0,0)
     - #Motion command
   * - 3
     - FT_Control(0,11,1,0,1,0,0,0,10,0,5,0,0,0,0.001,0,0,0,0,0,0,0,10,5)
     - #Force/torque motion control off 

Force/Torque Transducer Screw Insertion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Instruction description: The "FT_Spiral" instruction is a spiral line exploration and insertion, which is generally used for the shaft hole assembly action of a cylindrical shaft. Before running the action, you need to drag the end of the robot to the approximate position of the hole. According to the current scene, set the parameters of the command and add it to the program. After running, the robot will explore in a spiral motion.

.. figure:: robot_peripherals/042.png
   :align: center
   :width: 3in

.. centered:: Figure 5.8-7 FT_Spiral Command Edit

Program example:

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - FT_Control(1,10,0,0,1,0,0,0,0,0,5,0,0,0,0.0005,0,0,0,0,0,0,10,0)
     - #Force/torque motion control ON
   * - 2
     - FT_SpiralSearch(0,0.7,0,60000,5)
     - #Spiral insertion
   * - 3
     - FT_Control(0,10,0,0,1,0,0,0,0,0,5,0,0,0,0.0005,0,0,0,0,0,0,10,0)
     - #Force/torque motion control off 

Force/Torque Transducer Rotary Insertion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Instruction description: The "FT_Rot" instruction is a rotation exploration insertion, which is generally used to undertake the helical insertion action, and is used for the shaft hole assembly of the key shaft. Before running the action, you need to move the end of the robot to the hole found by the helical exploration or the fully aligned teaching hole. According to the current scene, set the parameters of the command and add it to the program. After running, the robot will slowly Spin to explore.

.. figure:: robot_peripherals/044.png
   :align: center
   :width: 3in

.. centered:: Figure 5.8-8 FT_Rot Command Edit

Program example:

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - FT_Control(1,10,0,0,1,0,0,0,0,0,5,0,0,0,0.0005,0,0,0,0,0,0,10,0)
     - #Force/torque motion control ON
   * - 2
     - FT_RotInsertion(0,3,0,5,1,0,1)
     - #Rotate Insert
   * - 3
     - FT_Control(0,10,0,0,1,0,0,0,0,0,5,0,0,0,0.0005,0,0,0,0,0,0,10,0)
     - #Force/torque motion control off 

Force/Torque Transducer Straight Insertion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Instruction description: "FT_Lin" instruction is rotation exploration insertion, generally used to undertake helical insertion action or rotation insertion action, and is used for shaft hole assembly of key shaft. Before running the action, you need to move the end of the robot to the hole found by the helical exploration, rotate the end of the insertion action or the fully aligned teaching hole, set the parameters of the command according to the current scene, add it to the program, and run After that, the robot will move in a straight line in the set direction.

.. figure:: robot_peripherals/046.png
   :align: center
   :width: 3in

.. centered:: Figure 5.8-9 FT_Lin Command Edit

Program example:

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - FT_Control(1,10,0,0,1,0,0,0,0,0,5,0,0,0,0.0005,0,0,0,0,0,0,10,0)
     - #Force/torque motion control ON
   * - 2
     - FT_LinInsertion(0,50,1,0,100,1)
     - #Line insertion
   * - 3
     - FT_Control(0,10,0,0,1,0,0,0,0,0,5,0,0,0,0.0005,0,0,0,0,0,0,10,0)
     - #Force/torque motion control off

Force/Torque Sensor Surface Orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Instruction description: The "FT_FindSurface" instruction is for surface positioning, and is generally used to find the surface of an object. According to the current scene, set the corresponding coordinate system, moving direction, moving axis, exploring linear speed, exploring linear acceleration, maximum exploring distance, action termination force threshold and other parameters, add them to the program, run the program, the action starts to execute, and the end of the robot starts to slow down Move in the direction of the surface.

.. figure:: robot_peripherals/048.png
   :align: center
   :width: 3in

.. centered:: Figure 5.8-10 FT_FindSurface Command Edit

Program example:

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - PTP(1,30,-1,0)
     - #Initial position
   * - 2
     - FT_FindSurface(0,1,3,1,0,100,5)
     - #Plane positioning

Force/Torque Transducer Centering
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Instruction description: The "FT_CalCenter" instruction is for center positioning, and is generally used to find the middle plane surface of two surfaces. According to the current scene, set the corresponding coordinate system, moving direction, moving axis, exploring linear speed, exploring linear acceleration, maximum exploring distance, action termination force threshold and other parameters, find the A plane and B plane respectively, add them to the program, and run the program. The action starts to execute, and the robot slowly moves towards the direction of surface A. After positioning on surface A, the robot slowly moves towards the direction of surface B. After positioning on surface B, the position of the center plane can be calculated.

.. figure:: robot_peripherals/050.png
   :align: center
   :width: 3in

.. centered:: Figure 5.8-11 FT_CalCenter Command Edit

Program example:

.. list-table:: 
   :widths: 1 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - PTP(1,30,-1,0)
     - #Initial position
   * - 2
     - FT_CalCenterStart()
     - #Surface positioning start
   * - 3
     - FT_Control(1,10,0,0,1,0,0,0,0,0,-10,0,0,0,0.00001,0,0,0,0,0,0,100,0)
     - #Force/torque motion control ON
   * - 4
     - FT_FindSurface(1,2,2,10,0,200,5)
     - #Positioning plane A
   * - 5
     - FT_Control(0,10,0,0,1,0,0,0,0,0,-10,0,0,0,0.00001,0,0,0,0,0,0,100,0)
     - #Force/torque motion control off
   * - 6
     - PTP (1,30, - 1,0) -- initial position
     - #Initial position
   * - 7
     - FT_Control(1,10,0,0,1,0,0,0,0,0,-10,0,0,0,0.00001,0,0,0,0,0,0,100,0)
     - #Force/torque motion control ON
   * - 8
     - FT_FindSurface(1,1,2,20,0,200,5)
     - #Positioning plane B
   * - 9
     - FT_Control(0,10,0,0,1,0,0,0,0,0,-10,0,0,0,0.00001,0,0,0,0,0,0,100,0)
     - #Force/torque motion control off
   * - 10
     - pos = {}
     - #Acquire Cartesian pose of positioning center
   * - 11
     - pos = FT_CalCenterEnd()
     - #Acquire Cartesian pose of positioning center
   * - 12
     - MoveCart(pos,GetActualTCPNum(),GetActualWObjNum(),30,10,100,-1,0)
     - #Move to the center of positioning

Force/Torque Sensor Tap Force Detection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Instruction description: The "FT_Click" command is click force detection, which is used to detect a click force, and is usually used in conjunction with the surface positioning action. After setting the parameters, add it to the program, run the program, and the end starts to move toward the target along the Z direction of the tool coordinate system. When the force in the positive Z direction reaches the value of the click force, the click force detection is completed.

.. figure:: robot_peripherals/052.png
   :align: center
   :width: 3in

.. centered:: Figure 5.8-12 FT_Click Command Edit

Program example:

.. list-table:: 
   :widths: 15 40 100
   :header-rows: 1

   * - S/N
     - Instruction format
     - notes
   * - 1
     - PTP(1,30,-1,0)
     - #Initial position
   * - 2
     - FT_Click(0,5,5,0,100,0)
     - #Spot force detection

Extended IO device peripheral configuration
--------------------------------------------

Extended IO device configuration steps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step1**: Select the "End Peripheral Configuration" button in the user peripheral configuration interface, and select "Extended IO Device" for the device type. The configuration information of the extended IO device is divided into manufacturer, type, software version and mounting location. Users can select according to specific production needs. To configure the corresponding device information. If the user needs to change the configuration, he can first select the corresponding number, click the "Clear" button to clear the corresponding information, and reconfigure according to the needs;

.. figure:: robot_peripherals/054.png
   :align: center
   :width: 3in

.. centered:: Figure 5.9-1 Extended IO device configuration

.. important:: 
	The corresponding device should be inactive before clicking Clear Configuration.

**Step2**: After the configuration of the extended IO device is completed, the user can click the "Smart Tool" function menu in the auxiliary application to enter the function configuration page, and the user can customize the functions of each button on the end handle, including (new program, hold program, PTP , Lin, ARC, start of weaving, end of weaving, IO port).

.. figure:: robot_peripherals/055.png
   :align: center
   :width: 3in

.. centered:: Figure 5.9-2 Extended IO device function configuration

Palletizing system configuration
---------------------------------------

Palletizing system configuration steps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step1**: Select the "Palletizing System Configuration" button in the user peripheral configuration interface. For the first time use, you need to create a recipe first. Click "Create Recipe", enter the name of the recipe, click "Create", and click "Start Configuration" after the creation is successful. Enter the palletizing configuration page.

.. figure:: robot_peripherals/056.png
   :align: center
   :width: 3in

.. centered:: Figure 5.10-1 Palletizing recipe configuration

**Step2**:  Click "Configure" in the workpiece configuration bar to enter the workpiece configuration pop-up window, set the "length", "width", "height" of the workpiece and the grabbing point of the workpiece, click "confirm configuration" to complete the workpiece information setting.

.. figure:: robot_peripherals/057.png
   :align: center
   :width: 3in

.. centered:: Figure 5.10-2 Palletizing workpiece configuration

**Step3**: Click "Configure" in the tray configuration bar to enter the tray configuration pop-up window, set the tray "front", "side" and "height", then set the station and station transition point, click "confirm configuration" to complete the tray information setting.

.. figure:: robot_peripherals/058.png
   :align: center
   :width: 3in

.. centered:: Figure 5.10-3 Palletizing pallet configuration

**Step4**: Click "Configure" in the size configuration column of the palletizing equipment to enter the size configuration pop-up window. Set the devices "X", "Y", "Z", and "Angle", and click "Confirm Configuration" to complete the size configuration information setting of the palletizing equipment.

.. important:: 
   X, Y and Z are the absolute values of the upper right corner or upper left corner of the tray relative to the robot's coordinate system. Angle is the rotation angle of the robot during installation, and it is recommended to set it to 0 during installation.

.. figure:: robot_peripherals/066.png
   :align: center
   :width: 3in

.. centered:: Figure 5.10‑4 Size configuration of palletizing equipment

**Step5**: Click "Configure" in the pattern configuration column to enter the mode configuration pop-up window.

   **Pattern B on/off**: on: Can switch pattern A/B, Configure B mode of palletizing each layer mode; off: Mode B cannot be switched, and mode B of each layer mode of palletizing cannot be configured;

   **Pattern A/B switching**: Select mode A: Add the workpiece as mode A, and the workpiece serial numbers are A1, A2..., and the transparency of the workpiece cannot be adjusted; select mode B: Add the workpiece as mode B, and the workpiece serial numbers are B1, B2..., and you can turn on / Turn off the "Display Mode A Configuration" display mode A artifact;

   **Display pattern A on/off**: on: Adjust the transparency of the mode B artifacts to see whether the A/B mode configuration effect is reasonable. At this time, only the mode B artifacts can be selected, added, batch added, deleted and deleted; off: Unable to set mode B artifact transparency;

.. important:: 
   When configuring workpieces, if there is a collision between workpieces, the background color of the workpiece will turn red, and the above operations cannot be performed at this time. If you need to operate, please configure the workpiece to be collision-free.

When configuring workpieces, first set the workpiece interval. The frame on the right simulates the placement of workpieces on the right pallet. You can add them individually or in batches. Then set the number of palletizing layers and the mode of each layer, and click "Confirm Configuration" to complete the mode information setting.

.. important:: 
   Stacking direction: Taking the right pallet as an example, the lower right corner is the farthest point. From the lower right corner, a row of workpieces is placed vertically or horizontally, and then the workpieces are placed horizontally or vertically in the upper row, and so on (the web page has been marked Palletizing direction, please check carefully).
   
   The left pallet places workpieces mirroring the right pallet pattern.

.. figure:: robot_peripherals/059.png
   :align: center
   :width: 6in

.. centered:: Figure 5.10‑5 Palletizing pattern A configuration

.. figure:: robot_peripherals/070.png
   :align: center
   :width: 6in

.. centered:: Figure 5.10‑6 Palletizing pattern B configuration

**Step6**: Click "Advanced Configuration" in the teaching program generation column to enter the advanced configuration pop-up window. At this time, configure the "Material Lifting Height", "First Offset Distance", "Second Offset Distance" and "Suction Waiting Time".

   **Material Lifting Height**: The user defines the lifting height after the material is successfully retrieved from the grabbing point;

   **First/Second Offset Distance**: User-defined configuration of the offset distance for tilted stacking of the robot to the target point;
   
   **Suction Waiting Time**: The user can configure the waiting time for material suction, monitor the negative pressure arrival signal after suction, and repeat the suction action when it is not in place;

   **Smooth transition**: Turn on the smooth transition button to configure parameters related to palletizing/depalletizing PTP smoothing time and LIN smoothing radius.

   - PTP smoothing time: No smooth transition time/Level 1 (200ms)/Level 2 (400ms)/Level 3 (600ms)/Level 4 (800ms)/Level 5 (1000ms)

   - LIN smooth radius: No smooth transition radius/Level 1 (200mm)/Level 2 (400mm)/Level 3 (600mm)/Level 4 (800mm)/Level 5 (1000mm)

.. figure:: robot_peripherals/063.png
   :align: center
   :width: 6in

.. centered:: Figure 5.10-7 Advanced palletizing configuration

**Step7**: Click "Generate Program" to open the "Palletizing Monitoring Page", where you can display and view the "Generation Information", "Alarm Information" and "Palletizing Program".

.. figure:: robot_peripherals/060.png
   :align: center
   :width: 6in

.. centered:: Figure 5.10-8 Palletizing system monitoring

**Step8**: After an error is reported in the middle of the palletizing running program, the program stops. After the user clears the error first, select the palletizing program to run again. At this time, the "Last Program Interruption" pop-up box will pop up. Click the "Continue" button to continue running, and click "Restart" button to restart the program.

.. figure:: robot_peripherals/064.png
   :align: center
   :width: 6in

.. centered:: Figure 5.10-9 Palletizing program continues

Polishing equipment configuration
------------------------------------

Polishing equipment configuration step
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step1**: Enter the Polishing equipment configuration page, configure the communication information, you need to configure the IP address, port, sampling period and communication protocol. After the configuration is successful, it will be automatically displayed next time.

**Step2**: After completing the communication configuration, the communication can be established by loading/unloading the grinding equipment.

**Step3**: Device functions. Operations such as device enabling, error clearing, and force sensor zeroing can be performed.

**Step4**: Parameter configuration. The rotation speed, contact force, reach distance and control mode of the polishing equipment can be set. After successful setting, the corresponding data and status can be displayed in the "Polish" status feedback column on the right.

.. figure:: robot_peripherals/061.png
   :align: center
   :width: 6in

.. centered:: Figure 5.11-1 Polishing equipment configuration page