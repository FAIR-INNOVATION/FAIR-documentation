Version V3.8.4
-----------------

Date: 2025-07-18
    
- **Extended Axis Blending Smoothing Function Implementation**: 
    Path: Teach Program -> Program Editing, Graphical Programming, Node Graph Programming.
  
    Description: Achieves smooth motion between extended axis commands, improving work efficiency.
    
- **Controller Slave Mode Optimization**: 
    Path: Initial Setup -> Peripherals -> Board Communication, Remote Mode.
  
    Description: Added board IP configuration, LUA command interface, and enabled controller slave protocol auto-start in remote mode.
    
- **Controller N2L Board QX Adaptation (EtherCAT)**: 
    Description: FAIRINO Industrial real-time communication board, miniPCIe card supporting EtherCAT protocol communication.
    
- **Robot JOG Motion**: 
    Description: Added CO status output function for robot JOG motion.

Version V3.8.3
-----------------

Date: 2025-06-27

- **Control Box Slave Mode Function**:
    Path: Initial Setup -> Peripherals -> Board Communication.

    Description: Added board communication module configuration in the robot peripheral interface. Based on domestic expansion boards, it enables EIP, CClink, and PN protocols for slave interaction with the robot.

- **Added Collision Detection for Force Sensor-Assisted Dragging in Manual Mode**:
    Path: Auxiliary Applications -> Tool Applications -> Drag Lock -> Force Sensor-Assisted Lock.

    Description: When the force sensor-assisted dragging function is enabled, collision detection is also triggered in manual mode to prevent damage to end equipment.

- **Added T-Shaped Velocity Profile Trajectory Planning + Blending Smoothing Function**:
    Path: Teach Programming -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: Achieves smooth motion between movement commands, improving work efficiency.

- **WebApp Status Query Interface Optimization**:
    Path: Status Information -> Status Query.

    Description: Optimized status query parameters to 6, with waveform display time up to 30s. Added data view display with copy functionality and renamed icon names.

- **Drag Force Optimization for FR Series Robots**:
    Path: Initial Setup -> Basics -> Joints -> Friction Compensation -> Drag Force Compensation.

    Description: Makes dragging more effortless by providing compensation torque during the dragging process.

- **Web System Log Storage Optimization**:
    Description: Fixed abnormal system log file issues. Default log retention period set to 7 days.

Version V3.8.2
-----------------

Date: 2025-05-29
    
- **End Lua Open Protocol Supports Welding Handle Function**: 
    Path: Initial Settings -> Peripherals -> Lua End Open Protocol Configuration -> Welding Handle.
  
    Description: Supports adapting to Jasic or SmartTool welding handles using open protocol, with all parameters configurable.
    
- **Controller Peripheral Open Protocol Adds Welding Machine Protocol**: 
    Path: Initial Settings -> Peripherals -> Welding Machine Configuration.
  
    Description: Enables communication with welding machines via controller peripheral open protocol (ModbusTCP) for welding machine control.

- **Added LUA Program Pause Function**: 
    Path: Teaching Program -> Program Programming.
  
    Description: During program execution, users can pause at any line including wait commands and communication commands, without counting toward timeout duration.    

- **Added T-Shaped Velocity Optimization + Blending Function**: 
    Path: Teaching Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: When enabled, provides smoother velocity curves. Currently supports blending between (PTP-PTP, LIN-LIN, ARC-ARC, LIN-ARC, ARC-LIN).
    
- **FIR Adaptive Parameter Function + FIR Pause/Resume Function**: 
    Path: Teaching Program -> Program Programming, Graphical Programming, Node Diagram Programming; Initial Settings -> Safety -> Motion Configuration.

    Description: Adds global FIR and adaptive parameter switches. When enabled, regular PTP/LIN/ARC commands automatically use FIR planning (cannot include blend radius).
    
- **Modbus RTU Function Optimization**: 
    Path: Teaching Program -> Program Programming, Graphical Programming, Node Diagram Programming; Teaching Program -> Program Programming -> Modbus RTU Settings.

    Description: Allows configuring ModbusRTU master station, baud rate, parity, station number etc. via WebApp. Enables real-time monitoring of ModbusRTU register values.
    
- **Joint Soft Limit Protection Function**: 
    Path: Initial Settings -> Joints -> Soft Limits.

    Description: In drag mode, provides damping force when joints approach configured soft limits, with automatic return to safe range after releasing external torque.
    
- **Robot Swing Tilt Angle Function**: 
    Path: Teaching Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: Supports custom swing angles around Rx axis of swing coordinate system during swing motion.
    
- **Gradual Process Parameter Welding Function**: 
    Path: Teaching Program -> Program Programming, Graphical Programming, Node Diagram Programming.

    Description: Enables gradual changes in travel speed and current/voltage during welding process.
    
- **Analog Arc Tracking Function**: 
    Path: Initial Settings -> Basic -> I/O Settings -> AI Configuration.

    Description: Direct connection between control box DI、DO/AI、AO and welder IO ports enables arc tracking based on analog current feedback.
    
- **FRCap Plugin System + Palletizing Plugin Package**: 
  
    Description: Supports QX x86 version FRCap plugin system and palletizing plugin package. FRCap can interact with robot controller through official interfaces, or customers can develop custom interfaces and logic as needed.

- **Robot Model and Type**: 

    Description: Add FR3C robot model and configuration settings.

Version V3.8.1
-----------------

Date: 2025-04-14
    
- **T-shaped Velocity Function Optimization**: 
    Path: Teaching Program -> Program Programming, Graphical Programming, Node Graph Programming, Initial Setup -> Basic Module.
  
    Description: Added trapezoidal velocity optimization mode in motion configuration, optimizing jerk during start/stop phases and increasing acceleration during acceleration phase. Mainly applied to scenarios prone to collision alarms during start/stop or with noticeable residual vibrations.
    
- **Conveyor Tracking Function Optimization**: 
    Path: Initial Setup -> Tracking -> Conveyor.
  
    Description: Added follow-up motion function in conveyor tracking mode, enabling motion teaching without workpiece coordinate system and allowing custom trigger delay distance.

- **Added Impedance Callback Function for Axis Interference Zone**:
    Path: Auxiliary Applications -> Tool Applications -> Interference Zone -> Interference Zone Configuration.
  
    Description: Provides impedance callback effect when entering interference zone during force sensor-assisted dragging.

- **Welding Handle Adaptation Function**:
    Description: Added welding handle adaptation: Robots with mounted welding handles can now program welding sequences and control robot start/stop, manual/auto switching, dragging, etc.

- **Added Joint Limit Ring Display on WEB Interface**:
    Description: Enables real-time display of joint positions when limit ring display function is activated.
    
- **New SDK Functions**:
    Description: SDK now supports downloading controller logs, all data sources, and data backup packages.
    
- **Joint Information Recording Before/After Collision**:
    Description: Records joint position, velocity, acceleration and torque information before/after collisions, facilitating direct analysis of collision causes in field applications.

Version V3.8.0
-----------------

Date: 2025-03-03

- **Offset and swing monotonous gradual arc tracking function**:
    Path: Teaching program -> Program programming, graphical programming, node diagram programming.

    Description: The swing width of the swing parameter can be gradually changed in a section of welding movement, and the swing parameter at the beginning of the section gradually transitions to the swing parameter at the end of the section, and the welding center can actively offset the weld center.

- **Custom collision detection torque threshold function**:
    Path: Teaching program -> Program programming, graphical programming, node diagram programming.

    Description: The collision detection torque threshold can be set during operation. You can select the joint torque threshold or TCP side torque threshold as needed, and print the joint speed, acceleration, torque and other information to the log file after the collision.

- **Real-time forward trajectory planning method function**:
    Path: Teaching program -> Program programming, graphical programming, node diagram programming.

    Description: Realize smooth connection of multi-track point fitting, and adaptively adjust the running speed with the path curvature.

- **Load curve drawing of FR full series models**:
    Description: Update the load curve chart of FR full series models.

- **Reduction mode speed switching logic optimization**:
    Description: When the reduction mode is triggered, there will occasionally be movement with a speed far lower than the reduction mode speed. This problem is optimized and solved.

- **Teaching point optimization**:
    Description: Optimize the teaching point adding prompt information.

- **FOCAS-based CNC Function Package**:
    Description: Adds FOCAS CNC functions.

- **Slave Command Adaptation Board**:
    Description: Adapts slave commands to EnTalk miniPCIe board (Profinet protocol, Ethernet/IP protocol, CC-Link protocol) and CIFX 9OE-RE/F/PNS miniPCIe board (Profinet protocol, Ethernet/IP protocol, Ethercat protocol, CC-Link protocol).

- **Checkpoint Timestamp Feedback Function**:
    Description: Servo J motion can receive timestamp results, including command number and timestamps for issuance, enqueueing, dequeueing, and execution.

- **Laser Sensor Adaptation Controller Peripheral Open Protocol**:
    Description: Adds laser peripheral open protocol communication functionality.

Version V3.7.8
-----------------

Date: 2025-01-20

- **Extension axis plus laser data non-transformation fixed-point tracking function**:
    Path: Teaching program -> Program programming, graphical programming, node diagram programming, initial settings -> Peripherals -> Sensors.

    Description: Mainly for real-time fixed-point tracking of positioners, so that the workpiece deviation recorded by the laser during the movement of the extension axis is compensated at the end of the robot tool.

- **Robot CI configuration new and optimized functions**:
    Path: Initial settings -> Basics -> I/O settings -> DI configuration, teaching program -> Program programming, graphical programming, node diagram programming.

    Description: 1. CI configurable input adds high and low level switching robot hand automatic mode function. Configure the CI port as "hand automatic switching (high and low level)". When the input signal of this port is valid, the robot automatically switches to automatic mode. When the input signal of this port is invalid, the robot automatically switches to manual mode. 2. After opening the LUA program in WebApp, the currently opened LUA program will be automatically run when starting the LUA program through the configurable CI, instead of the last saved program.

- **WebApp status query function optimization**:
    Path: Status Information -> Status Query.

    Note: The front-end and back-end data interaction method changes the front-end polling request method to the websocket active data sending method to reduce CPU usage.

- **Control box firmware DO power-on high and low effective configurable function**:
    Path: Initial Settings -> Basics -> I/O Settings -> DO Configuration.

    Note: Added "Control box DO output during power-on" configuration. Before the robot is enabled, the control box DO can be configured to the required high and low level states according to the specific usage scenario.

- **Real-time forward trajectory planning method function**:
    Path: Teaching program -> Program programming, graphical programming, node diagram programming.

    Description: Added "TrajectoryLA" instruction.

- **Strategy function based on force sensor collision**:
    Path: Initial settings -> Joints -> Collision level.

    Description: Added "safe speed" parameter in collision rebound mode.

- **Automatic lifting function after force sensor collision**:
    Description: After realizing collision detection based on force sensor, the robot automatically lifts and can limit the speed of the lifting process.

- **Load curve drawing of FR full series models**:
    Description: Update the load curve chart of FR full series models.

Version V3.7.7
-----------------

Date: 2024-12-30

- **Real-time monitoring function of gripper status data**:
    Path: Initial Settings -> Peripherals -> Open Protocol -> Status Monitoring.

    Description: Real-time display of gripper running speed, torque, position and other status information.

- **Controller fault data collection function**:
    Path: System Settings -> General Settings -> Fault Data.

    Description: When a collision, instruction point error or other faults occur during the operation of the robot, the controller automatically records the robot position, speed and other status information 15s before and after the fault occurs. WebApp can export fault information in .csv format to help troubleshoot and analyze the cause of the fault.

- **Singularity avoidance and crossing function based on force sensor assisted drag**:
    Path: Auxiliary Application -> Tool Application -> Drag Lock -> Force Sensor Assisted Lock.

    Description: 1. In the force sensor assisted drag interface, select the strategy of avoidance. When the robot is dragged close to the singularity, a virtual force will be generated. 2. When the strategy is selected as crossing, it will switch to drag mode when approaching the singularity, and switch to force sensor assisted drag when away from the singularity.

- **Old dynamics adaptation 360° free installation and backup package import adaptation dynamics function**:
    Path: Initial Settings -> Basics -> Installation -> Free Installation, Auxiliary Application -> Tool Application -> Data Backup.

    Description: When importing the backup package, add verification of robot type, installation method, installation angle and dynamic configuration type. When the above parameters are inconsistent, prompt to prohibit import.

- **Singularity crossing function in automatic mode**:
    Path: Teaching Program -> Peripherals -> LIN/ARC instructions.

    Description: Added "singularity crossing" motion protection configuration.
    
- **Lua program function that automatically updates the corresponding point after the end button records the point**:
    Path: Auxiliary application -> Tool application -> Teaching point configuration.

    Description: Added the Lua program function that automatically updates the corresponding point after the end button records the point.

- **Status query interface optimization**:
    Path: Status information -> Status query.

    Description: Status query interface UI and interaction optimization.

- **WebApp, teach pendant interface optimization**:
    Description: WebApp, teach pendant added Russian and Traditional Chinese display interfaces.

- **WebApp display interface optimization function**:
    Description: Added software version and robot model display in the lower left corner of the WebApp interface.

- **Extended axis plus laser tracking function**:
    Description: 1. The robot moves synchronously with the external axis, and the laser realizes synchronous tracking in the external axis coordinate system. 2. The robot moves asynchronously with the external axis, and the laser can track in the robot base coordinate system or the external axis coordinate system.

- **Tool point drag function based on force sensor**:
    Description: In the web interface, set the FT reference coordinate system to a custom coordinate system. After turning on the force sensor-assisted dragging, the robot moves along the set tool coordinate system.

- **Robot CNDE function**:
    Description: The client can obtain robot status feedback and send control data to the robot through CNDE (UDP communication). The status feedback cycle is configurable (1-200ms), and the robot status feedback data content and the client input control robot data content can be freely configured.

Version V3.7.6
-----------------

Date: 2024-11-18

- **Initial setting page layout optimization**:
    Path: Initial setting.

    Description: Optimize the tool setting interface, load configuration interface and some function icon display.

- **WebApp peripheral extension axis configuration function interaction optimization**:
    Path: Initial setting -> Peripheral -> Extension axis.

    Description: Extension axis interface layout and interaction optimization.

- **System log function optimization**:
    Path: Status information -> System log.

    Description: Added log paging display and log detailed operation category distinction.

- **XY direction horizontal constant force grinding function**:
    Path: Teaching program -> Programming -> Force control set -> F/T_Control instruction.

    Description: Add "grinding disc radius" parameter, which can perform repeated straight line/curve motion along the workpiece surface.

- **Laser point motion function**:
    Path: Initial setting -> Peripheral -> Tracking -> Sensor.

    Description: The function of finding the coordinates of the intersection of three and four points is added with the use of lua script instructions.

- **External axis configuration-two-degree-of-freedom trolley function**:
    Path: Initial settings -> Coordinate system -> Extended axis -> Extended axis coordinate system.

    Description: Add the "two-degree-of-freedom trolley" extended axis solution, the robot and PLC communicate with UDP, and then the PLC controls the two-degree-of-freedom trolley through EtherCat.

- **TCP calibration method based on flat tool function**:
    Path: Initial settings -> Coordinate system -> Tool -> Tool coordinate system.

    Description: Add the "flat tool calibration" coordinate system calibration method.

- **Robot background program function**:
    Path: Teaching program -> Program programming.

    Description: The background program can normally obtain I/O interface data, including system I/O, modbus, and extended I/O.

- **Robot trajectory automatically avoids singular points function**:
    Path: Teaching program -> Program programming -> Linear Lin/Arc instructions.

    Description: Added "Singularity Avoidance" motion protection configuration.

- **Rotating gripper end adaptation function**:
    Path: Initial Settings -> Peripherals -> Open Protocol.

    Description: Added rotating gripper related function code configuration.

- **Custom Protocol Slave Instructions**:
    Path: Remote Mode.

    Description: Added "Controller Slave Protocol" configuration.

- **UDP Communication Wire Positioning Function**:

    Description: The robot can use UDP to expand IO to start and stop the welding wire positioning and obtain the welding wire positioning success signal.

- **Backup package function optimization**:

    Description: Support the import and use of old version data packets (QX 3.6.1 and later version data packets).

- **Restore Factory Settings Function Optimization**: Added file verification to increase the system restore factory stability.

    Description: Added file verification to increase the system restore factory stability.

- **Upgrade function optimization**:

    Description: QX3.6.9 and later versions can be directly upgraded to QX3.7.6, and the current version user data will be retained after the upgrade.

- **Page downgrade function**:

    Description: WebApp pages support downgrades, which can be downgraded to any version of QX3.6.9 and later, and the current version user data will be retained after the downgrade.

Version V3.7.5
-----------------

Date: 2024-09-30

- **Adjustable angular velocity function of wrap angle attitude transition**:
    Path: Teaching program -> Programming -> Linear Lin instruction.

    Description: Added "Adjustable angular velocity of transition point" motion protection configuration.

- **Arc tracking multi-layer multi-pass welding function**:
    Path: Auxiliary application -> Welding expert library -> Weldment shape -> Multi-layer multi-pass welding.

    Description: Added "First layer welding swing function" and "Arc tracking function" configuration.

- **485 extended axis configuration function**:
    Path: Initial settings -> Peripherals -> Extended axis -> Extended axis configuration.

    Description: Added "acceleration and emergency stop" configuration.

- **Robot tool TCP automatic calibration function (self-made fiber optic sensor tooling)**:
    Path: Initial settings -> Coordinate system -> Tool -> Tool coordinate system.

    Description: Added "photoelectric automatic calibration" coordinate system calibration.

- **Teach pendant multi-language setting function**:
    Path: Login page.

    Description: Added "language switching" configuration.

Version V3.7.4
-----------------

Date: 2024-08-09

- **Based on lua terminal open protocol software function (gripper part)**:
    Path: Initial settings -> Peripherals -> Terminal tools -> Open protocol -> Lua terminal development protocol configuration.

    Description: Added "Terminal protocol enable" configuration.

- **Oblique sawtooth swing function**:
    Path: Teaching program -> Program programming -> Swing Weave instruction.

    Description: Added "Swing direction azimuth" parameter configuration.

- **Robot welding process package optimization function**:
    Path: Initial settings -> Peripherals -> Welding machine -> Welding machine configuration.

    Description: Added "Welding process parameters" configuration.

- **Lin instruction joint overspeed processing function**:
    Path: Teaching Program -> Programming -> Linear Lin Instruction.

    Description: Added "Joint Overspeed Protection" motion protection configuration.

Version V3.7.3
-----------------

Date: 2024-06-28

- **Modbus slave control robot function**:
    Path: Teaching Program -> Programming -> ModbusTCP.

    Description: Added "Slave Controller" configuration.

- **Emergency Stop Type Function**:
    Path: Initial Settings -> Safety -> Emergency Stop.

    Description: Added "Class 1a, Class 2a, Class 2" stop types.

Version V3.7.2
-----------------

Date: 2024-06-07

- **Lua terminal development protocol configuration function**:
    Path: Initial Settings -> Peripherals -> Terminal Tools -> Open Protocol.

    Description: Added "Development Protocol" configuration.

- **Motion AO control instruction function**:
    Path: Teaching program -> Programming.

    Description: Added "Motion AO instruction".

- **Six-dimensional force and joint impedance mixed drag function**:
    Path: Auxiliary application -> Tool application -> Drag lock.

    Description: Added "Six-dimensional force and joint impedance mixed drag".

- **Post-collision response strategy function**:
    Path: Initial settings -> Basics -> Joints -> Collision level.

    Description: Added "Collision strategy" configuration.

- **Robot first activation function**:
    Path: Login settings.

    Description: Added robot first activation verification function.

Version V3.7.1
-----------------

Date: 2024-05-10

- **Web interface lock function**:
    Path: System settings -> Custom information.

    Description: Added "web interface lock screen" configuration.

- **Three-point and four-point intersection coordinate search function**:
    Path: Initial Settings -> Peripherals -> Tracking -> Sensors.

    Description: Added "Three-point and Four-point Intersection Point Coordinates Function".

- **Segment Welding Motion Posture Optimization Function**:
    Path: Teaching Program -> Programming -> Segment Welding Segment Instructions.

    Description: Added "Segment Welding Mode" configuration.

- **Virtual Wall Function Based on Force Sensor**:
    Path: Initial Settings -> Peripherals -> End Tools -> End Peripherals, Auxiliary Applications -> Tool Applications -> Drag Lock.

    Description: Added "Force Sensor" related configuration parameters to the end peripheral configuration, and "Inertia Coefficient" configuration to the force sensor auxiliary lock.

- **SmartTool+Force Sensor Combination New Function**:
    Path: Auxiliary Applications -> Tool Applications -> Smart Tool.

    Description: Added key function configuration.