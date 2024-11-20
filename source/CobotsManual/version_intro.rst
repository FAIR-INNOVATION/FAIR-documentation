Version V3.7.6
-----------------

Date: 2024-11-18

- **Initial settings page layout optimization**: Initial settings -> Optimize tool setting interface, load configuration interface and some function icon display;

- **WebApp peripheral extension axis configuration function interaction optimization**: Initial settings -> Peripherals -> Extension axis interface layout and interaction optimization;

- **System log function optimization**: Status information -> System log -> Added log paging display and log detailed operation category distinction;

- **XY direction horizontal constant force grinding function**: Teaching program -> Programming -> Force control set -> F/T_Control command adds "grinding disc radius" parameter, which can be repeated linear/curved motion along the workpiece surface;

- **Laser point movement function**: Initial settings -> Peripherals -> Tracking -> Sensor -> Three-point and four-point position search intersection coordinate function adds lua script command usage;

- **External axis configuration-two-degree-of-freedom trolley function**: Initial settings -> Coordinate system -> Extended axis -> Extended axis coordinate system -> Added "two-degree-of-freedom trolley" extended axis solution, UDP communication between the robot and PLC, and then the PLC controls the two-degree-of-freedom trolley through EtherCat;

- **TCP calibration method based on flat tool function**: Initial settings -> Coordinate system -> Tool -> Tool coordinate system -> Added "flat tool calibration" coordinate system calibration method;

- **Robot background program function**: Teaching program -> Program programming -> Background program can normally obtain I/O interface data, including system I/O, modbus, and extended I/O;

- **Robot trajectory automatic avoidance of singularity function**: Teaching program -> Program programming -> Linear Lin/Arc instructions add "singularity avoidance" motion protection configuration;

- **Rotating gripper end adaptation function**: Initial settings -> Peripherals -> Open protocol -> Added rotating gripper related function code configuration;

- **Customized protocol slave station command**: Remote mode -> Added "controller slave station protocol" configuration;

- **UDP communication welding wire positioning function**: The robot can use UDP to expand IO to start and stop the welding wire positioning and obtain the welding wire positioning success signal;

- **Backup package function optimization**: Support the import and use of old version data packets (QNX 3.6.1 and later version data packets);

- **Factory reset function optimization**: Added file verification to increase the stability of system factory reset;

- **Upgrade function optimization**: QNX3.6.9 and later versions can be directly upgraded to QNX3.7.6, and the current version user data will be retained after the upgrade;

- **Page downgrade function**: WebApp page supports downgrade, which can be downgraded to any version of QNX3.6.9 and later, and the current version user data will be retained after the downgrade;
  
Version V3.7.5
-----------------

Date: 2024-09-30

- **Angle attitude transition angular velocity adjustable function**: Teaching program -> Program programming -> Added "adjustable transition point angular velocity" motion protection configuration to the linear Lin instruction;

- **Arc tracking multi-layer multi-pass welding function**: Auxiliary application -> Welding expert library -> Weldment shape -> Added "first layer welding swing function" and "arc tracking function" configuration to multi-layer multi-pass welding;

- **485 extended axis configuration function**: Initial settings -> Peripherals -> Extended axis -> Added "acceleration and emergency stop" configuration to the extended axis configuration;

- **Robot tool TCP automatic calibration function (self-made fiber optic sensor tooling)**: Initial settings -> Coordinate system -> Tool -> Added "photoelectric automatic calibration" coordinate system calibration to the tool coordinate system;

- **Teach pendant multi-language setting function**: Login page -> Added "language switching" configuration;

Version V3.7.4
-----------------

Date: 2024-08-09

- **Software function based on lua terminal open protocol (gripper part)**: Initial settings -> Peripherals -> Terminal tool -> Open protocol -> Lua terminal development protocol configuration adds "terminal protocol enable" configuration;

- **Oblique sawtooth swing function**: Teaching program -> Program programming -> Weave instruction adds "Swing direction azimuth" parameter configuration;

- **Robot welding process package optimization function**: Initial settings -> Peripherals -> Welding machine -> Welding machine configuration adds "Welding process parameters" configuration;

- **Lin instruction joint overspeed processing function**: Teaching program -> Program programming -> Linear Lin instruction adds "Joint overspeed protection" motion protection configuration;

Version V3.7.3
-----------------

Date: 2024-06-28

- **Modbus slave station control robot function**: Teaching program -> Program programming -> ModbusTCP adds "Slave station controller" configuration;

- **Emergency stop type function**: Initial settings -> Safety -> Emergency stop -> Stop type adds "Class 1a, Class 2a, Class 2" stop type;

Version V3.7.2
-----------------

Date: 2024-06-07

- **Lua terminal development protocol configuration function**: Initial settings -> Peripherals -> Terminal tools -> Open protocol -> Added "Development protocol" configuration;

- **Motion AO control instruction function**: Teaching program -> Program programming -> Added "MoveAO instruction";

- **Six-dimensional force and joint impedance mixed drag function**: Auxiliary application -> Tool application -> Drag lock -> Added "Six-dimensional force and joint impedance mixed drag";

- **Post-collision response strategy function**: Initial settings -> Basics -> Joints -> Collision level -> Added "Collision strategy" configuration;

- **Robot first activation function**: Login settings -> Added robot first activation verification function;

Version V3.7.1
-----------------

Date: 2024-05-10

- **Web interface lock function**: System settings -> Custom information -> Added "web interface lock screen" configuration;

- **Function of finding three or four points to find the coordinates of the intersection**: Initial settings -> Peripherals -> Tracking -> Sensors -> Added "Function of finding three or four points to find the coordinates of the intersection";

- **Segment welding motion posture optimization function**: Teaching program -> Program programming -> Segment welding instruction added "Segment welding mode" configuration;

- **Virtual wall function based on force sensor**: Initial settings -> Peripherals -> End tool -> End peripheral configuration added "Force sensor" related configuration parameters, Auxiliary application -> Tool application -> Drag lock -> Force sensor auxiliary lock added "Inertia coefficient" configuration;

- **SmartTool+force sensor combination new function**: Auxiliary application -> Tool application -> Smart Tool added key function configuration.