Version V3.8.6
-----------------

Date: 2025-09-19

- **New Robot Impedance Control Function**:
    Path: Teach Program -> Program Programming -> F/T Command.

    Description: Through real-time detection of external forces, the robot actively complies with the external force and deviates from the motion trajectory when the set threshold is reached. It returns to the motion trajectory when the external force drops below the threshold, enabling better human-robot interaction.

- **New Torque Detection Function Before Dragging**:
    Path: Initial Setup -> Basic -> Collision Level.

    Description: Before entering drag mode, the difference between the torque command and feedback for each joint is calculated. If the load or installation method is incorrectly configured, causing the difference to exceed the threshold, entry into drag mode is prevented to avoid robot失控.

- **New Custom Oscillation Welding Function**:
    Path: Teach Program -> Program Programming -> Weave Command.

    Description: Users can design their own oscillation welding patterns to execute oscillation welding.

- **ModbusTCP**:
    Path: Teach Program -> Program Programming -> ModbusTCP.

    Description: Optimized the ModbusTCP master station timeout detection function.

- **Pneumatic Suction Cup**:
    Path: Initial Setup -> Peripherals -> Array Suction Cup.

    Description: Optimized the web page adaptation function for pneumatic suction cups.

- **Wire Feeding Search Position Fault Error**:

    Description: Optimized WebAPP wire feeding search position fault error reporting, which can be reset.

- **Configurable IO Function**:

    Description: Added configurable IO function for FR3C-FR3MT. Configurable inputs are CI0-CI4, and configurable outputs are CO0-CO4.

Version V3.8.5
-----------------

Date: 2025-08-19

- **Socket Network Debugging**:
    Path: Teach Program -> Program Programming -> Socket Network Debugging.

    Description: Allows adding, configuring, and deleting socket connection configurations, supporting up to 4 socket connections. Command modules can generate teach programming for opening connections, closing connections, sending data, and receiving data.

- **CAD to G-code to Robot Trajectory Planning Function**:
    Path: Auxiliary Applications -> Tool Applications -> G-code Conversion.

    Description: Generate G-code files for machining trajectories like lines, arcs, circles, and splines in CAM software such as Solidworks and FUSION360, then import the G-code files into the Web端.

- **Added Actual Physical Speed Function for Linear, Arc, and Full Circle Motion Commands**:
    Path: Teach Program -> Program Programming, Initial Setup -> Peripherals -> Welding Handle.

    Description: Allows direct definition of the actual running physical speed for the current motion command.

- **Optimized PTP Speed Adjustment Function**:

    Description: Ensures a smooth speed adjustment process and reduces speed fluctuations at the衔接 points of speed adjustments.

- **Robot Model Configuration**:

    Description: Added FR30L robot model configuration option.

- **Control Box Adaptation**:

    Description: Added control box adaptation for ETMP03E board (EtherCAT protocol).

Version V3.8.4.1
-----------------

Date: 2025-07-30

- **Control Box Adaptation for Ethernet Pass-Through Module and Suction Cup/Gripper Control Function**:
    Path: Initial Setup -> Peripherals -> Array Suction Cup.

    Description: Enables control of array suction cups (up to 20) via an Ethernet to 485 pass-through module, based on the page and peripheral open protocol.

- **Robot Look-ahead Trajectory Planning (Constant Velocity Look-ahead)**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: Added a constant velocity switch option. When enabled, the robot performs look-ahead motion at constant velocity.

- **Optimized Control Box Slave Mode Function**:
    Path: Initial Setup -> Peripherals -> Board Communication, Remote Mode.

    Description: Added control box adaptation for Ji Yuan ETMP03E board (EtherCAT protocol).

- **Laser Search Position Point Acquisition Function**:
    Path: Initial Setup -> Peripherals -> Line Laser Sensor.

    Description: After successful laser seam search, the position data stored in "seamPos" can be obtained using SDK or TCP protocol commands.

- **FR5 and FR10 Speed Opening Function**:

    Description: Increased FR5 linear speed from 1.0m/s to 1.7m/s; increased FR10 linear speed from 1.5m/s to 2.0m/s.

- **Robot Model Configuration**:

    Description: Added FR5L long-reach robot model configuration option.

- **Optimized Software Upgrade**:

    Description: Shortened upgrade time, compatible with downgrading from high versions to any lower version (3.7.6-3.8.4).

- **Optimized Factory Reset**:

    Description: Factory reset preserves robot model, stiffness, dynamics settings, button box version, etc.

Version V3.8.4
-----------------

Date: 2025-07-18

- **Implementation of Extension Axis Blending Smoothing Function**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: Achieves smooth motion between extension axis commands, improving work efficiency.

- **Optimized Control Box Slave Mode Function**:
    Path: Initial Setup -> Peripherals -> Board Communication, Remote Mode.

    Description: Added board IP configuration, interface LUA command issuance, and added self-start function for the controller slave protocol in remote mode.

- **Control Box N2L Board QX Adaptation (EtherCAT)**:
    Description: FaRo Industrial real-time bus communication board, miniPCIe board, supporting EtherCAT protocol communication.

- **Robot JOG Motion**:
    Description: Added robot JOG motion CO status output function.

Version V3.8.3
-----------------

Date: 2025-06-27

- **Control Box Slave Mode Function**:
    Path: Initial Setup -> Peripherals -> Board Communication.

    Description: Added board communication module configuration in the robot peripherals interface, enabling interaction with the robot slave station via EIP, CClink, PN protocols based on domestic expansion boards.

- **Added Collision Detection Function for Force Sensor Assisted Dragging in Manual Mode**:
    Path: Auxiliary Applications -> Tool Applications -> Drag Lock -> Force Sensor Assisted Lock.

    Description: When the force sensor assisted dragging function is enabled and the robot is in manual mode, the collision detection function is also triggered to avoid damage to the end equipment.

- **Added T-shaped Velocity Profile Trajectory Planning + Blending Smoothing Function**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: Achieves smooth motion between various motion commands, improving work efficiency.

- **WebApp Status Query Interface Function Optimization**:
    Path: Status Information -> Status Query.

    Description: Optimized status query parameters to 6, maximum waveform time to 30s, added data view display with copy capability, and added rename icon function.

- **FR Full Series Robot Drag Force Optimization**:
    Path: Initial Setup -> Basic -> Joint -> Friction Compensation -> Drag Force Compensation.

    Description: Makes dragging more energy-saving by providing compensation torque during the dragging process.

- **Web System Log Storage Function Optimization**:
    Description: Fixed system log file abnormality issues, set default log retention days to 7 days.

Version V3.8.2
-----------------

Date: 2025-05-29

- **End Lua Open Protocol Adaptation for Welding Handle Function**:
    Path: Initial Setup -> Peripherals -> Welding Handle.

    Description: Enables adaptation of the smarttool welding handle using the open protocol, with support for configuring all parameters.

- **Controller Peripheral Open Protocol Adds Welding Machine Open Protocol**:
    Path: Initial Setup -> Peripherals -> Welding Machine.

    Description: Enables communication with the welding machine for control via the controller peripheral open protocol (ModbusTCP).

- **Added LUA Program Pause Function**:
    Path: Teach Program -> Program Programming.

    Description: During teach program execution, pause can be clicked on any line, including wait-type instructions and communication-type instructions, without occupying timeout time when paused.

- **Added T-shaped Velocity Profiling + Blending Function**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: After enabling the T-shaped velocity profiling switch, the velocity curve becomes smooth. Currently supports blending types (PTP-PTP, LIN-LIN, ARC-ARC, LIN-ARC, ARC-LIN).

- **FIR Adaptive Parameter Function + FIR Pause/Resume Function**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming, Initial Setup -> Safety -> Motion Configuration.

    Description: Added global FIR enable and adaptive parameter switch to the FIR function. When enabled, running ordinary PTP/LIN/ARC instructions automatically converts them to FIR planning (cannot use blend radius).

- **Modbus RTU Function Optimization**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming, Teach Program -> Program Programming -> Modbus RTU Settings.

    Description: Allows configuring ModbusRTU master station, baud rate, parity mode, station number, etc., in WebApp. Enables monitoring of real-time ModbusRTU register values.

- **Joint Soft Limit Protection Function**:
    Path: Initial Setup -> Joint -> Soft Limits.

    Description: In drag mode, when a joint approaches the soft limit set in the interface, damping force is applied. After removing the external torque, the joint can return within the soft limit.

- **Robot Oscillation Roll Angle Function**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: Supports the robot end tool customizing the oscillation angle around the Rx direction of the oscillation coordinate system during oscillation motion.

- **Process Parameter Gradual Change Welding Function**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: Enables gradual change of travel speed, current, and voltage during the welding process.

- **Analog Arc Tracking Function**:
    Path: Initial Setup -> Basic -> I/O Settings -> AI.

    Description: Directly uses control box DI, DO, AI, AO to connect with the welding machine IO ports, achieving arc tracking based on analog current feedback.

- **FRCap Plugin System + Palletizing Plugin Package Function**:

    Description: Adapted FRCap plugin system and palletizing plugin package for QX x86 version. FRCap can interact with the robot controller through provided official interfaces, or customers can write custom interface commands and processing logic based on actual needs for personalized development.

- **Robot Model and Configuration**:

    Description: Added FR3-C robot model and configuration.

Version V3.8.1
-----------------

Date: 2025-04-14

- **T-shaped Velocity Function Optimization**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming, Initial Setup -> Basic Module.

    Description: Added trapezoidal velocity optimization mode in motion configuration, optimizing jerk during start/stop phases and increasing acceleration during the acceleration phase. Mainly applied to working conditions prone to collision reports during start/stop, and conditions with significant residual vibration during start/stop.

- **Conveyor Tracking Function Optimization**:
    Path: Auxiliary Applications -> Process Package -> Conveyor Tracking.

    Description: Added tracking inspection motion function in conveyor tracking motion mode, eliminating the need for motion teaching in the workpiece coordinate system, allowing custom trigger delay distance.

- **Added Impedance Callback Function for Axis Interference Zone**:
    Path: Initial Setup -> Safety -> Interference Zone.

    Description: Under force sensor assisted dragging, entering the interference zone can have an impedance callback effect.

- **Welding Handle Adaptation Function**:
    Description: Added welding handle adaptation: A welding handle installed on the robot end can be used for welding program writing and controlling robot start, stop program, manual/auto switching, dragging, etc.

- **WEB Interface Added Joint Limit Ring Display**:
    Description: Enabling the limit ring display function allows real-time display of the joint position.

- **SDK New Functions**:
    Description: SDK added functions for downloading controller logs, all data sources, and data backup packages.

- **Collision Pre/Post Joint Information Recording Function**:
    Description: Records joint position, velocity, acceleration, and torque information before and after a collision, facilitating direct analysis of the cause of collisions at the customer site.

Version V3.8.0
-----------------

Date: 2025-03-03

- **Offset and Amplitude Monotonic Gradual Change Arc Tracking Function**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: The oscillation width among oscillation parameters can gradually change during a welding motion, transitioning from the segment start amplitude parameter to the segment end amplitude parameter. The welding center can actively offset from the seam center.

- **Custom Collision Detection Torque Threshold Function**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: Allows setting the collision detection torque threshold during operation. The joint torque threshold or TCP side torque threshold can be selected as needed. After a collision, joint speed, acceleration, torque, and other information are printed to the log file.

- **Real-time Look-ahead Trajectory Planning Method Function**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: Achieves smooth connection of multiple trajectory points through fitting, with running speed adaptively adjusted according to path curvature.

- **FR Full Series Model Load Curve Drawing**:
    Description: Updated load curve charts for the FR full series models.

- **Reduced Mode Speed Switching Logic Optimization**:
    Description: Optimized and resolved the issue where occasionally the speed was far lower than the reduced mode speed when the reduced mode was triggered.

- **Teach Point Optimization**:
    Description: Optimized the prompt information added to teach points.

- **CNC Function Package Function Based on FOCAS**:
    Description: Added CNC function based on FOCAS.

- **Slave Command Adaptation Board**:
    Description: Slave command adaptation for EnTalk miniPCIe board (Profinet protocol, Ethernet/IP protocol, CC-Link IEF Basic protocol), adaptation for CIFX 9OE-RE/F/PNS miniPCIe board (Profinet protocol, Ethernet/IP protocol, Ethercat protocol, CC-Link IEF Basic protocol).

- **Checkpoint Timestamp Feedback Function**:
    Description: Servo J motion can receive timestamp results, including command number and issue, enqueue, dequeue, execution timestamps.

- **Laser Sensor Adaptation Controller Peripheral Open Protocol**:
    Description: Added laser peripheral open protocol communication function.

Version V3.7.8
-----------------

Date: 2025-01-20

- **Extension Axis Plus Laser Data No Transformation Fixed Point Tracking Function**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming, Initial Setup -> Peripherals -> Line Laser Sensor.

    Description: Mainly for real-time fixed-point tracking of positioners, compensating the workpiece deviation recorded by the laser at the robot tool end when the extension axis is moving.

- **Robot CI Configuration Addition and Optimization Function**:
    Path: Initial Setup -> Basic -> I/O Settings -> DI, Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: 1. Configurable CI input added the function of switching robot manual/auto mode via high/low level. Configure the CI port as "Manual/Auto Switch (High/Low Level)". When the input signal of this port is valid, the robot automatically switches to auto mode; when invalid, it switches to manual mode. 2. After opening a LUA program in WebApp, starting the LUA program via configurable CI automatically runs the currently opened LUA program, not the last saved program.

- **WebApp Status Query Function Optimization**:
    Path: Status Information -> Status Query.

    Description: Changed the front-end and back-end data interaction method from front-end polling request to WebSocket active data sending, reducing CPU usage.

- **Control Box Firmware DO Power-on High/Low Level Configurable Function**:
    Path: Initial Setup -> Basic -> I/O Settings -> DO.

    Description: Added "Control Box DO Output During Power-on" configuration. Before the robot is enabled, the control box DO can be configured to the required high/low level state according to specific usage scenarios.

- **Real-time Look-ahead Trajectory Planning Method Function**:
    Path: Teach Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: Added "TrajectoryLA" instruction.

- **Strategy Function After Collision Based on Force Sensor**:
    Path: Initial Setup -> Joint -> Collision Level.

    Description: Added "Safe Speed" parameter to the collision rebound mode.

- **Automatic Lifting Function After Force Sensor Collision**:
    Description: Implemented automatic lifting function of the robot after collision detection based on the force sensor, with the ability to limit the speed during the lifting process.

- **FR Full Series Model Load Curve Drawing**:
    Description: Updated load curve charts for the FR full series models.

Version V3.7.7
-----------------

Date: 2024-12-30

- **Gripper Status Data Real-time Monitoring Function**:
    Path: Initial Setup -> Peripherals -> Gripper -> Status Monitoring.

    Description: Displays real-time gripper status information such as running speed, torque, and position.

- **Controller Fault Data Collection Function**:
    Path: System Settings -> General Settings -> Fault Data.

    Description: When collisions, command point errors, or other faults occur during robot operation, the controller automatically records the robot's status information (position, speed, etc.) for 15 seconds before and after the fault. WebApp can export fault information in .csv format to help troubleshoot and analyze the cause of the fault.

- **Singularity Avoidance and Traversal Function Based on Force Sensor Assisted Dragging**:
    Path: Auxiliary Applications -> Tool Applications -> Drag Lock -> Force Sensor Assisted Lock.

    Description: 1. In the force sensor assisted dragging interface, when the strategy is set to avoidance, a virtual force is generated when dragging the robot close to a singularity. 2. When the strategy is set to traversal, it switches to drag mode when approaching a singularity, and switches back to force sensor assisted dragging when moving away from the singularity.

- **Old Dynamics Adaptation for 360° Free Installation and Backup Package Import Adaptation Dynamics Function**:
    Path: Initial Setup -> Basic -> Installation, Auxiliary Applications -> Tool Applications -> Data Backup.

    Description: Added verification of robot type, installation method, installation angle, and dynamics configuration type during backup package import. If these parameters are inconsistent, import is prohibited with a prompt.

- **Singularity Traversal Function in Auto Mode**:
    Path: Teach Program -> Peripherals -> LIN/ARC Command.

    Description: Added "Singularity Traversal" motion protection configuration.

- **Function to Automatically Update Corresponding Point LUA Program After Recording Point with End Button**:
    Path: Teach Program -> Program Programming -> Teach Point.

    Description: Added function to automatically update the corresponding point LUA program after recording a point with the end button.

- **Status Query Interface Optimization**:
    Path: Status Information -> Status Query.

    Description: Optimized the UI and interaction of the status query interface.

- **WebApp, Teach Pendant Interface Optimization**:
    Description: WebApp and teach pendant added Russian and Traditional Chinese display interfaces.

- **WebApp Display Interface Optimization Function**:
    Description: Added software version and robot model display in the lower left corner of the WebApp interface.

- **Extension Axis Plus Laser Tracking Function**:
    Description: 1. Robot and external axis move synchronously, the laser achieves synchronous tracking in the external axis coordinate system. 2. Robot and external axis move asynchronously, the laser can track in the robot base coordinate system or the external axis coordinate system.

- **Tool Point Dragging Function Based on Force Sensor**:
    Description: In the web interface, set the FT reference coordinate system to a custom coordinate system. After enabling force sensor assisted dragging, the robot moves along the set tool coordinate system.

- **Robot CNDE Function**:
    Description: Clients can use CNDE to obtain robot status feedback and send control data to the robot (UDP communication). The status feedback cycle is configurable (1-200ms). The content of robot status feedback data and client input control data can be freely configured.

Version V3.7.6
-----------------

Date: 2024-11-18

- **Initial Setup Page Layout Optimization**:
    Path: Initial Setup.

    Description: Optimized the tool setting interface, load configuration interface, and display of some function icons.

- **WebApp Peripheral Extension Axis Configuration Function Interaction Optimization**:
    Path: Initial Setup -> Peripherals -> Extension Axis.

    Description: Optimized the layout and interaction of the extension axis interface.

- **System Log Function Optimization**:
    Path: Status Information -> System Log.

    Description: Added log pagination display and detailed log operation category distinction.

- **XY Direction Horizontal Constant Force Grinding Function**:
    Path: Teach Program -> Program Programming -> Force Control Set -> F/T_Control Command.

    Description: Added "Grinding Disc Radius" parameter, allowing repeated linear/curved motion along the workpiece surface.

- **Laser Point Acquisition Motion Function**:
    Path: Auxiliary Applications -> Tool Applications -> Intersection Point Generation.

    Description: Added the use of LUA script instructions for the three-point and four-point search to find intersection point coordinate function.

- **External Axis Configuration - Two-DOF Cart Function**:
    Path: Initial Setup -> Peripherals -> Extension Axis.

    Description: Added "Two-DOF Cart" extension axis solution. The robot communicates with the PLC via UDP, and then the PLC controls the two-DOF cart via EtherCat.

- **TCP Calibration Method Function Based on Flat Plate Tool**:
    Path: Initial Setup -> Coordinate System -> Tool Coordinate.

    Description: Added "Flat Plate Tool Calibration" coordinate system calibration method.

- **Robot Background Program Function**:
    Path: Teach Program -> Program Programming.

    Description: Background programs can normally obtain I/O interface data, including system I/O, modbus, and extended I/O.

- **Robot Trajectory Automatic Singularity Avoidance Function**:
    Path: Teach Program -> Program Programming -> Linear Lin/Arc Arc Command.

    Description: Added "Singularity Avoidance" motion protection configuration.

- **Rotary Gripper End Adaptation Function**:
    Path: Initial Setup -> Peripherals -> Gripper.

    Description: Added related function code configuration for rotary grippers.

- **Custom Protocol Slave Command**:
    Path: Remote Mode.

    Description: Added "Controller Slave Protocol" configuration.

- **UDP Communication Wire Feeding Search Position Function**:

    Description: The robot can control wire feeding search start/stop and obtain the wire feeding search success signal via UDP extended IO.

- **Backup Package Function Optimization**:

    Description: Supports import and use of old version data packages (QX 3.6.1 and later version data packages).

- **Factory Reset Function Optimization**: Added file verification, increased system factory reset stability.

    Description: Added file verification, increased system factory reset stability.

- **Upgrade Function Optimization**:

    Description: QX3.6.9 and later versions can be directly upgraded to QX3.7.6, retaining current version user data after upgrade.

- **Page Downgrade Function**:

    Description: WebApp page supports downgrade, can be downgraded to any version after QX3.6.9, retaining current version user data after downgrade.

Version V3.7.5
-----------------

Date: 2024-09-30

- **Wrap Angle Pose Transition Angular Speed Adjustable Function**:
    Path: Teach Program -> Program Programming -> Linear Lin Command.

    Description: Added "Transition Point Angular Speed Adjustable" motion protection configuration.

- **Arc Tracking Multi-layer Multi-pass Welding Function**:
    Path: Auxiliary Applications -> Process Package -> Welding Expert Library -> Multi-layer Multi-pass Welding.

    Description: Added "First Layer Welding Oscillation Function" and "Arc Tracking Function" configurations.

- **485 Extension Axis Configuration Function**:
    Path: Initial Setup -> Peripherals -> Extension Axis.

    Description: Added "Acceleration and Emergency Stop" configuration.

- **Robot Tool TCP Automatic Calibration Function (Self-made Fiber Optic Sensor Fixture)**:
    Path: Initial Setup -> Basic -> Tool Coordinate.

    Description: Added "Photoelectric Automatic Calibration" coordinate system calibration.

- **Teach Pendant Multi-language Setting Function**:
    Path: Login Page.

    Description: Added "Language Switch" configuration.

Version V3.7.4
-----------------

Date: 2024-08-09

- **Software Function Based on Lua End Open Protocol Adaptation (Gripper Part)**:
    Path: Initial Setup -> Peripherals -> Gripper.

    Description: Added "End Protocol Enable" configuration.

- **Oblique Zigzag Oscillation Function**:
    Path: Teach Program -> Program Programming -> Oscillation Weave Command.

    Description: Added "Oscillation Direction Azimuth Angle" parameter configuration.

- **Robot Welding Process Package Optimization Function**:
    Path: Initial Setup -> Peripherals -> Welding Machine -> Welding Machine Configuration.

    Description: Added "Welding Process Parameters" configuration.

- **Lin Command Joint Overspeed Handling Function**:
    Path: Teach Program -> Program Programming -> Linear Lin Command.

    Description: Added "Joint Overspeed Protection" motion protection configuration.

Version V3.7.3
-----------------

Date: 2024-06-28

- **Modbus Slave Control Robot Function**:
    Path: Teach Program -> Program Programming -> ModbusTCP.

    Description: Added "Slave Controller" configuration.

- **Emergency Stop Type Function**:
    Path: Initial Setup -> Safety -> Emergency Stop.

    Description: Added "Category 1a, Category 2a, Category 2" stop types to the stop type.

Version V3.7.2
-----------------

Date: 2024-06-07

- **Configuration Function Based on Lua End Open Protocol**:
    Path: Initial Setup -> Peripherals -> End Tool -> Open Protocol.

    Description: Added "Open Protocol" configuration.

- **Motion AO Control Command Function**:
    Path: Teach Program -> Program Programming.

    Description: Added "Motion AO Command".

- **Six-Axis Force and Joint Impedance Hybrid Dragging Function**:
    Path: Auxiliary Applications -> Tool Applications -> Drag Lock.

    Description: Added "Six-Axis Force and Joint Impedance Hybrid Dragging".

- **Post-Collision Response Strategy Function**:
    Path: Initial Setup -> Basic -> Joint -> Collision Level.

    Description: Added "Collision Strategy" configuration.

- **Robot First Activation Function**:
    Path: Login Settings.

    Description: Added robot first activation verification function.

Version V3.7.1
-----------------

Date: 2024-05-10

- **Web Interface Lock Function**:
    Path: System Settings -> Custom Information.

    Description: Added "Web Interface Lock Screen" configuration.

- **Search Position Three-Point and Four-Point Intersection Coordinate Function**:
    Path: Auxiliary Applications -> Tool Applications -> Intersection Point Generation.

    Description: Added "Search Position Three-Point and Four-Point Intersection Coordinate Function".

- **Segment Welding Motion Posture Optimization Function**:
    Path: Teach Program -> Program Programming -> Segment Welding Segment Command.

    Description: Added "Segment Welding Mode" configuration.

- **Virtual Wall Function Based on Force Sensor**:
    Path: Initial Setup -> Peripherals -> Force Sensor, Auxiliary Applications -> Tool Applications -> Drag Lock.

    Description: Added "Force Sensor" related configuration parameters to the end peripheral configuration. Added "Inertia Coefficient" configuration to force sensor assisted lock.

- **SmartTool + Force Sensor Combination New Functions**:
    Path: Initial Setup -> Peripherals -> Welding Handle.

    Description: Added key function configuration.