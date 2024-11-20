Custom protocol slave commands
================================================

.. toctree:: 
   :maxdepth: 6

Overview
-------------------

In order to facilitate PLC motion control of the robot via different industrial bus protocols (CC-Link, Profinet, Ethernet/IP and EtherCAT), the following functions are realised with the addition of Hilscher board modules to the integrated mini control box:

1) CC-Link slave protocol support;
2) Profinet slave protocol support;
3) Ethernet/IP slave protocol support;
4) EtherCAT slave protocol support;

Environment Configuration
------------------------------

Hardware environment setup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Install the Hilscher board into the integrated mini control box as shown.

.. image:: custom_protocol_slave/001.png
   :width: 4in
   :align: center

.. centered:: Figure 17.2-1 Hilscher board installation

.. image:: custom_protocol_slave/002.png
   :width: 4in
   :align: center

.. centered:: Figure 17.2-2 Hilscher board network port

2. The robot control box and PLC wiring is shown below.

.. image:: custom_protocol_slave/003.png
   :width: 4in
   :align: center

.. centered:: Figure 17.2-3 Control Box & Mitsubishi PLC Wiring Diagrams

.. image:: custom_protocol_slave/004.png
   :width: 4in
   :align: center

.. centered:: Figure 17.2-4 Control Box & Siemens PLC Wiring Diagrams

.. image:: custom_protocol_slave/005.png
   :width: 4in
   :align: center

.. centered:: Figure 17.2-5 Control Box & Omron PLC Wiring Diagrams

.. image:: custom_protocol_slave/006.png
   :width: 4in
   :align: center

.. centered:: Figure 17.2-6 Control Box & Omron PLC Wiring Diagrams

.. note:: 
      1: Robot control box (board input network port);
      2: Switch;
      3: PC;
      4: Mitsubishi PLC (CC-link network port);
      5: Siemens PLC (Profinet network port);
      6: Omron PLC (Ethernet/IP network port);
      7: Omron PLC (EtherCAT network port);

.. important:: When the protocol is switched to EtherCAT bus, the board's network port needs to be distinguished as EtherCAT_IN and EtherCAT_OUT. At this time, the PLC's EtherCAT network port is directly connected to the board's EtherCAT_IN through a network cable.

Software environment setup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Browser IP input 192.168.58.2, account for admin, password for 123, click ‘Login’, enter the robot control box Web interface.

.. image:: custom_protocol_slave/007.png
   :width: 6in
   :align: center

.. centered:: Figure 17.2-7 Web Login Interface

2. Enter the Tools App -> System Upgrade, select the software.tar.gz file and upload.

.. image:: custom_protocol_slave/008.png
   :width: 4in
   :align: center

.. centered:: Figure 17.2-8 Upgrade software

.. note:: QNX control box web version needs 3.7.6 and above, Linux control box web version needs 3.7.4 and above.

3. Go to Peripherals->Remote Control, select ‘Profinet control’ for Control mode, select ‘Hilscher’ for Manufacturer, select ‘4ms’ for Cycle Period, and click the ‘Set’ button.

.. image:: custom_protocol_slave/009.png
   :width: 4in
   :align: center

.. centered:: Figure 17.2-9 Interface configuration

4. Click ‘Local Mode’ in the upper right corner -> Switch Remote Mode.

.. image:: custom_protocol_slave/010.png
   :width: 4in
   :align: center

.. centered:: Figure 17.2-10 Switch remote mode

5. Select the controller slave protocol and click the ‘Set’ button.

.. image:: custom_protocol_slave/011.png
   :width: 6in
   :align: center

.. centered:: Figure 17.2-11 Configure the communication protocol

.. note:: Switching different protocols requires restarting the control box before configuring the protocols.

PLC Environment construction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The test environment built to implement the slave commands for each protocol is shown in the table below, which includes the PLC model, firmware version and test software used in each protocol.

.. list-table:: 
   :widths: 100 100 100 100 100
   :header-rows: 1
   :align: center

   * - Protocol
     - Brand
     - Type
     - Firmware
     - Software
  
   * - Profinet
     - Siemens
     - CPU 1515-2 PN
     - 6ES75152AM020AB0
     - TIA Portal V17
  
   * - CC-link
     - Mitsubishi
     - FX5S-30TR/DS
     - 30MR/ES V1.3
     - GX Works3 V1.097B
  
   * - Ethernet/IP
     - Omron
     - MX102-1100
     - V1.3
     - Sysmac Studio V1.50
  
   * - EtherCAT
     - Omron
     - MX102-1100
     - V1.3
     - Sysmac Studio V1.50

Siemens Profinet
++++++++++++++++++++++++++++++++++

1. GSD file (XML file) importing

Open Siemens programming software TIA Portal V17, create a new PLC project, select ‘Devices and Networks’, and select ‘Hardware Catalogue’ on the right side to add PLC module by double clicking 6ES7 515-2AM02-0AB0 to add PLC module.

.. image:: custom_protocol_slave/012.png
   :width: 6in
   :align: center

In the TIA PORTAL software, select Options-> Manage Generic Station Description File (GSD) in the menu bar to install or remove a GSD file that has already been installed.

.. image:: custom_protocol_slave/013.png
   :width: 6in
   :align: center

As an example, to install the Herschel GSD file, select ‘Manage Generic Station Description File (GSD)’ as above, and the ‘Manage Generic Station Description File’ window will appear.

Select the folder where you want to install the GSD file from the ‘Source Path’, select one or more files to install from the list of displayed GSD files, and click the ‘Install’ button. Click the Install button as shown in the following figure.

.. image:: custom_protocol_slave/014.png
   :width: 6in
   :align: center

After successful installation, you can find the device with the installed GSD file in the hardware catalogue, other field devices, as shown in the figure below.

.. image:: custom_protocol_slave/015.png
   :width: 4in
   :align: center

2. Executable programme

Open the project ‘QNXtest’.

.. image:: custom_protocol_slave/016.png
   :width: 6in
   :align: center

Compiler: Double-click on the left side of the project tree to enter ‘Devices and Networks’, right-click on the ‘PLC_1’ module, select Compile from the drop-down menu, and then select ‘Hardware and Software (Changes Only)’ from the stand-alone menu. Hardware and Software (Changes Only)’. After the compilation is completed, it will prompt ‘Compilation complete’ at the bottom of the software view.

.. image:: custom_protocol_slave/017.png
   :width: 6in
   :align: center

.. image:: custom_protocol_slave/018.png
   :width: 6in
   :align: center

Download the programme to the device: Double click on the left side of the project tree to enter the ‘Device and Network’, right click on the ‘PLC_1’ module, and select ‘Download to Device’ from the drop-down menu. ‘Download to Device’, “Hardware and Software (change only)”.

.. image:: custom_protocol_slave/019.png
   :width: 6in
   :align: center

Search and download devices: After the pop-up window, configure the PG/PC interface type as shown in the following figure, click Start Search, select the device that needs to download the programme, and click Download.

.. image:: custom_protocol_slave/020.png
   :width: 6in
   :align: center

.. image:: custom_protocol_slave/021.png
   :width: 6in
   :align: center

Mitsubishi CC-link
++++++++++++++++++++++++++++++++++

1. CC-Link IEF Basic Setup

Enable CC-link: Select ‘Ethernet Port’ in the left menu bar, set the ip address of PLC and make sure it is in the same network segment as the address of Huexun card. Click ‘CC-link IEF Basic’ and select ‘Use’.

.. image:: custom_protocol_slave/022.png
   :width: 6in
   :align: center

CC-Link Network Configuration Settings: Also in CC-Link IEF Basic Settings, select ‘Network Configuration Settings’, and choose Hueyoson CIFX Digital I/O module. Drag and drop the module to the bottom left of the view to complete the hardware configuration.

.. image:: custom_protocol_slave/023.png
   :width: 6in
   :align: center

CC-Link Refresh Settings: Also in CC-Link IEF Basic Settings, click Refresh Settings to customise the transmission settings: 256 bytes receive, 256 bytes transmit.

.. image:: custom_protocol_slave/024.png
   :width: 6in
   :align: center

2. Program Download

After opening the test programme, click ‘Online’->‘Write to Programmable Controller’ to enter the download interface.

.. image:: custom_protocol_slave/025.png
   :width: 6in
   :align: center

After opening the download interface, click ‘Parameter+Programme’ on the top left, then click ‘Execute’ on the bottom right corner to download, wait for the download to complete.

.. image:: custom_protocol_slave/026.png
   :width: 6in
   :align: center

HMI setting (CC-link emulation)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. After logging into the HMI interface, enable ‘Enable Task’ to establish the communication connection between PLC and controller.

.. image:: custom_protocol_slave/027.png
   :width: 6in
   :align: center

2. Click ‘01_MC_EnableRobot’ interface and then click ‘EnableRobot’ to enable the robot, and click ‘Reset’ to reset if there is any error during the process.

.. image:: custom_protocol_slave/028.png
   :width: 6in
   :align: center

3. Click ‘02_MC_ToolData’ to enter the tool information interface, enter the parameters on the left and click WriteToolData to write the tool information; on the right, click ReadToolData to read the existing tool information.
   
.. image:: custom_protocol_slave/029.png
   :width: 6in
   :align: center

4. Click ‘03_MC_FrameData’ to enter the interface of workpiece information. On the left side, after inputting parameters, click WriteFrameData to write workpiece information; on the right side, click ReadFrameData to read existing workpiece information.
   
.. image:: custom_protocol_slave/030.png
   :width: 6in
   :align: center

5. Click ‘04_MC_LoadData’ to enter the load information interface, enter the parameters on the left and click WriteLoadData to write load information; on the right, click ReadLoadData to read the existing load information.
   
.. image:: custom_protocol_slave/031.png
   :width: 6in
   :align: center

6. Click ‘05_MC_RobotReferenceDynamics’ to enter the interface of Maximum Velocity and Maximum Acceleration of Robot, enter the parameters on the left side and then click WriteRobotRefD to write the information of Maximum Velocity and Maximum Acceleration; click ReadRobotRefD on the right side to read the information of Maximum Velocity and Maximum Acceleration. On the right side, click ReadRobotRefD to read the max speed and max acceleration information.
   
.. image:: custom_protocol_slave/032.png
   :width: 6in
   :align: center

7. Click ‘06_MC_Robot DefaultDynamics’ to enter the interface of robot default speed and default acceleration, enter the parameters on the left side and click WriteRobotDefD to write the default speed and default acceleration information; on the right side, click ReadRobotDefD to read the default speed and default acceleration information. on the left side, and then click WriteRobotDefD to write the default speed and default acceleration information; on the right side, click ReadRobotDefD to read the information.
   
.. image:: custom_protocol_slave/033.png
   :width: 6in
   :align: center

8. Click ‘07_MC_RobotSwLimits’ to enter the coordinate limit interface. On the left side, input the maximum limit and minimum limit parameter values and click WriteRobotSwLimits to write the limit parameter information; on the right side, click ReadRobotSwLimits to read the existing limit parameter information. parameter information.
   
.. image:: custom_protocol_slave/034.png
   :width: 6in
   :align: center

9. Click ‘08_MC_ReadActualPosition’ to enter the read actual position interface, click ReadPosition to read the existing position information.
   
.. image:: custom_protocol_slave/035.png
   :width: 6in
   :align: center

10. Click ‘09_MC_MoveLinearAbsolute’ to enter the Linear Motion interface, input the coordinate parameter and click MoveLinearAbsolute to make the robot move linearly at the target position.
   
.. image:: custom_protocol_slave/036.png
   :width: 6in
   :align: center

11. Click ‘10_MC_MoveAxesAbsolute’ to enter the interface of axis coordinate movement, input the coordinate parameter and click MoveAxesAbsolute to make the robot move to the target position with the input axis coordinate as the end point.
   
.. image:: custom_protocol_slave/037.png
   :width: 6in
   :align: center

12. Click ‘11_MC_MoveDirectAbsolute’ to enter the direct motion interface, input the coordinate parameter and click MoveDirectAbsolute to make the robot move directly to the target position with the input parameter as the end point.
   
.. image:: custom_protocol_slave/038.png
   :width: 6in
   :align: center

13. Click ‘12_MC_Groups’ to enter the direct motion interface, in which, clicking GroupInterrupt can interrupt the movement of the robot in the process of movement, and clicking GroupContinue can make the robot continue to move to the target position. Click GroupStop to stop (end) the ongoing position movement. If an alarm or error is triggered during the process, click GroupReset to reset the robot to the error.
   
.. image:: custom_protocol_slave/039.png
   :width: 6in
   :align: center

14. Click ‘13_MC_PositionConversion’ to enter the position conversion interface, XtoJ1 can be converted from Cartesian position to joint angle, and J1toX can be converted from joint angle to Cartesian position.
   
.. image:: custom_protocol_slave/040.png
   :width: 6in
   :align: center

15. Click ‘14_MC_GroupJog’ to enter the interface of robot jogging, after the configuration is finished, drop down the axes to select the axis you need to jog, and then select the rotation direction of the axis. Click JogMove to move. MC_ChangeSpeedOverride on the right side can adjust the moving speed of the robot arm.
   
.. image:: custom_protocol_slave/041.png
   :width: 6in
   :align: center

HMI setting (Profinet emulation)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. After opening the programme, click on ‘HMI_1[ktp700 Basic PN]’ in the project tree, and then click on ‘Online’→‘Simulation’→‘Start’ in the menu bar. Click ‘Online’→‘Simulation’→‘Start’ in the menu bar. Wait for the software to compile and simulate.

2. The function after emulation is the same as the content of the Velcro screen (CC-link). You can refer to the above content to set up.
   
.. image:: custom_protocol_slave/042.png
   :width: 6in
   :align: center   

.. image:: custom_protocol_slave/043.png
   :width: 6in
   :align: center

Appendice
-------------------

Instruction List
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: 
   :widths: 20 80
   :header-rows: 1
   :align: center

   * - Command code
     - Command description

   * - 0x1000
     - Robot enablement

   * - 0x1001
     - Reset all error

   * - 0x1002
     - Robot stops moving

   * - 0x1003
     - Read actual position

   * - 0x1004
     - Set robot speed

   * - 0x1005
     - Resume robot motion

   * - 0x1006
     - Robot pauses motion

   * - 0x1007
     - Calculate the Cartesian position from the joint position

   * - 0x1008
     - Calculate joint position from Cartesian position

   * - 0x2000
     - Write tool information

   * - 0x2001
     - Read tool information

   * - 0x2002
     - Write workpiece information

   * - 0x2003
     - Read workpiece information

   * - 0x2004
     - Write load information

   * - 0x2005
     - Read load information

   * - 0x2006
     - Write reference dynamic information

   * - 0x2007
     - Read reference dynamic information

   * - 0x2008
     - Write default dynamic information

   * - 0x2009
     - Read default dynamic information

   * - 0x2010
     - Write soft limit information

   * - 0x2011
     - Read soft limit information

   * - 0x3000
     - MoveAxes (based on joint angle)

   * - 0x3001
     - MoveLinear

   * - 0x3002
     - MoveDirect (based on Cartesian coordinate system)

   * - 0x3003
     - jog motion

   * - 0x3004
     - jog stop