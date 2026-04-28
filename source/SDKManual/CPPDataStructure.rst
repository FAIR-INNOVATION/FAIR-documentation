Data structure specification
==============================

.. toctree:: 
    :maxdepth: 5

Interface call return value type
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    typedef  int errno_t;

Joint position data type
+++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint position data type
    */
    typedef  struct
    {
        double jPos[6];   /* Six joint positions, unit: deg */
    }JointPos;

Cartesian spatial location data type
++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Cartesian spatial location data type
    */
    typedef struct
    {
        double x;    /* X-axis coordinate, unit: mm  */
        double y;    /* Y-axis coordinate, unit: mm  */
        double z;    /* Z-axis coordinate, unit: mm  */
    } DescTran;

Euler Angle attitude data type
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Euler Angle attitude data type
    */
    typedef struct
    {
        double rx;   /* Rotation Angle about fixed axis X, unit: deg  */
        double ry;   /* Rotation Angle about fixed axis y, unit: deg  */
        double rz;   /* Rotation Angle about fixed axis Z, unit: deg  */
    } Rpy;

Cartesian space pose data type
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    *@brief Cartesian space pose type
    */
    typedef struct
    {
        DescTran tran;      /* Cartesian position  */
        Rpy rpy;            /* Cartesian space attitude  */
    } DescPose;

Extension axis position data type
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Extension axis position data type
    */
    typedef  struct
    {
        double ePos[4];   /* Position of four expansion shafts, unit: mm */
    }ExaxisPos;

Torque sensor data type
+++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief The force component and torque component of the force sensor
    */
    typedef struct
    {
        double fx;  /* Component of force along the x axis, unit: N  */
        double fy;  /* Component of force along the y axis, unit: N  */
        double fz;  /* Component of force along the z axis, unit: N  */
        double tx;  /* Component of torque about the X-axis, unit: Nm */
        double ty;  /* Component of torque about the Y-axis, unit: Nm */
        double tz;  /* Component of torque about the Z-axis, unit: Nm */
    } ForceTorque;

Spiral parameter data type
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Spiral parameter data type
    */
    typedef  struct
    {
        int    circle_num;           /* Coil number  */
        float  circle_angle;         /* Spiral Angle  */
        float  rad_init;             /* Initial radius of spiral, unit: mm  */
        float  rad_add;              /* Radius increment  */
        float  rotaxis_add;          /* Increment in the direction of the axis of rotation  */
        int rot_direction;  /* Rotation direction, 0- clockwise, 1- counterclockwise  */
        int velAccMode;      /* Velocity acceleration parameter mode: 0-constant angular velocity; 1-constant linear velocity */
    }SpiralParam;

feedback packet of robot controller state
+++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionchanged:: C++ SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
     * @brief  feedback packet of robot controller state
     */
    typedef struct _ROBOT_STATE_PKG
    {
      uint16_t frame_head;   // Frame header, preset to 0x5A5A 
      uint8_t frame_cnt;    // Frame count, cyclic count 0-255 
      uint16_t data_len;    // Length of data content 
      uint8_t program_state;  // Program running status, 1-stopped; 2-running; 3-paused;
      uint8_t robot_state;   // Robot motion status, 1-stopped; 2-running; 3-paused; 4-dragging 
      int   main_code;    // Main fault code 
      int   sub_code;    // Sub fault code 
      uint8_t robot_mode;   // Robot mode, 1-manual mode; 0-automatic mode; 
      double  jt_cur_pos[6];  // Current joint positions of 6 axes, unit deg 
      double  tl_cur_pos[6];  // Current tool position
            // tl_cur_pos[0], position along x-axis, unit mm,
            // tl_cur_pos[1], position along y-axis, unit mm,
            // tl_cur_pos[2], position along z-axis, unit mm,
            // tl_cur_pos[3], rotation angle around fixed X-axis, unit deg
            // tl_cur_pos[4], rotation angle around fixed Y-axis, unit deg
            // tl_cur_pos[5], rotation angle around fixed Z-axis, unit deg 
      double  flange_cur_pos[6]; // Current end flange position
            // flange_cur_pos[0], position along x-axis, unit mm,
            // flange_cur_pos[1], position along y-axis, unit mm,
            // flange_cur_pos[2], position along z-axis, unit mm,
            // flange_cur_pos[3], rotation angle around fixed X-axis, unit deg
            // flange_cur_pos[4], rotation angle around fixed Y-axis, unit deg
            // flange_cur_pos[5], rotation angle around fixed Z-axis, unit deg
      double  actual_qd[6];   // Current velocities of 6 joints, unit deg/s 
      double  actual_qdd[6];   // Current accelerations of 6 joints, unit deg/s^2 
      double  target_TCP_CmpSpeed[2]; 
            // target_TCP_CmpSpeed[0], TCP composite command speed (position), unit mm/s
            // target_TCP_CmpSpeed[1], TCP composite command speed (orientation), unit deg/s */
      double  target_TCP_Speed[6];  // TCP command speed
              // target_TCP_Speed[0], speed along x-axis, unit mm/s,
              // target_TCP_Speed[1], speed along y-axis, unit mm/s,
              // target_TCP_Speed[2], speed along z-axis, unit mm/s,
              // target_TCP_Speed[3], angular velocity around fixed X-axis, unit deg/s
              // target_TCP_Speed[4], angular velocity around fixed Y-axis, unit deg/s
              // target_TCP_Speed[5], angular velocity around fixed Z-axis, unit deg/s */
      double  actual_TCP_CmpSpeed[2]; 
              // actual_TCP_CmpSpeed[0], TCP composite actual speed (position), unit mm/s
              // actual_TCP_CmpSpeed[1], TCP composite actual speed (orientation), unit deg/s 
      double  actual_TCP_Speed[6];  // TCP actual speed
              // actual_TCP_Speed[0], speed along x-axis, unit mm/s,
              // actual_TCP_Speed[1], speed along y-axis, unit mm/s,
              // actual_TCP_Speed[2], speed along z-axis, unit mm/s,
              // actual_TCP_Speed[3], angular velocity around fixed X-axis, unit deg/s
              // actual_TCP_Speed[4], angular velocity around fixed Y-axis, unit deg/s
              // actual_TCP_Speed[5], angular velocity around fixed Z-axis, unit deg/s 
      double  jt_cur_tor[6];   // Current torques of 6 axes, unit N·m 
      int   tool;        // Applied tool coordinate system number 
      int   user;        // Applied workpiece coordinate system number 
      uint8_t cl_dgt_output_h;  // Control box digital IO output 15-8 
      uint8_t cl_dgt_output_l;  // Control box digital IO output 7-0 
      uint8_t tl_dgt_output_l;  // Tool digital IO output 7-0, only bit0-bit1 valid 
      uint8_t cl_dgt_input_h;   // Control box digital IO input 15-8 
      uint8_t cl_dgt_input_l;   // Control box digital IO input 7-0 
      uint8_t tl_dgt_input_l;   // Tool digital IO input 7-0, only bit0-bit1 valid 
      uint16_t cl_analog_input[2]; // cl_analog_input[0], control box analog input 0
                    //cl_analog_input[1], control box analog input 1 
      uint16_t tl_anglog_input;  // Tool analog input 
      double  ft_sensor_raw_data[6]; // Force/torque sensor raw data
              // ft_sensor_raw_data[0], force along x-axis, unit N
              // ft_sensor_raw_data[1], force along y-axis, unit N
              // ft_sensor_raw_data[2], force along z-axis, unit N
              // ft_sensor_raw_data[3], torque around x-axis, unit Nm
              // ft_sensor_raw_data[4], torque around y-axis, unit Nm
              // ft_sensor_raw_data[5], torque around z-axis, unit Nm 
      double  ft_sensor_data[6];   // Force/torque sensor data,
              // ft_sensor_data[0], force along x-axis, unit N
              // ft_sensor_data[1], force along y-axis, unit N
              // ft_sensor_data[2], force along z-axis, unit N
              // ft_sensor_data[3], torque around x-axis, unit Nm
              // ft_sensor_data[4], torque around y-axis, unit Nm
              // ft_sensor_data[5], torque around z-axis, unit Nm 
      uint8_t ft_sensor_active;  // Force/torque sensor activation status, 0-reset, 1-activated 
      uint8_t EmergencyStop;   // Emergency stop flag, 0-emergency stop not pressed, 1-emergency stop pressed 
      int   motion_done;    // Motion completion signal, 1-completed, 0-not completed 
      uint8_t gripper_motiondone; // Gripper motion completion signal, 1-completed, 0-not completed 
      int   mc_queue_len;    // Motion command queue length 
      uint8_t collisionState;   // Collision detection, 1-collision, 0-no collision 
      int   trajectory_pnum;  // Trajectory point number 
      uint8_t safety_stop0_state; // Safety stop signal SI0 
      uint8_t safety_stop1_state; // Safety stop signal SI1 
      uint8_t gripper_fault_id;  // Faulty gripper number 
      uint16_t gripper_fault;   // Gripper fault 
      uint16_t gripper_active;   // Gripper activation status 
      uint8_t gripper_position;  // Gripper position 
      int8_t  gripper_speed;   // Gripper speed 
      int8_t  gripper_current;  // Gripper current 
      int   gripper_temp;    // Gripper temperature 
      int   gripper_voltage;  // Gripper voltage 
      robot_aux_state aux_state;  // 485 extension axis status
      EXT_AXIS_STATUS extAxisStatus[4]; // UDP extension axis status 
      uint16_t extDIState[8];    // Extension DI input
      uint16_t extDOState[8];    // Extension DO output
      uint16_t extAIState[4];    // Extension AI input
      uint16_t extAOState[4];    // Extension AO output
      int rbtEnableState;      // Robot enable status
      double  jointDriverTorque[6];    // Robot joint driver torque
      double  jointDriverTemperature[6];  // Robot joint driver temperature
      RobotTime robotTime;      // Robot system time
      int softwareUpgradeState;   // Robot software upgrade status
      uint16_t endLuaErrCode;    // End Lua running status
      uint16_t cl_analog_output[2]; // Control box analog output
      uint16_t tl_analog_output;   // Tool analog output
      float gripperRotNum;      // Current rotation count of rotating gripper
      uint8_t gripperRotSpeed;    // Current rotation speed percentage of rotating gripper
      uint8_t gripperRotTorque;   // Current rotation torque percentage of rotating gripper
      WELDING_BREAKOFF_STATE weldingBreakOffState; // Welding interruption status
      double jt_tgt_tor[6];         // Joint command torque
      int smartToolState;          // SmartTool handle button status
      float wideVoltageCtrlBoxTemp;     // Wide voltage control box temperature
      uint16_t wideVoltageCtrlBoxFanCurrent;// Wide voltage control box fan current (mA)
      double toolCoord[6];    // Current tool coordinate system values; x,y,z,rx,ry,rz
      double wobjCoord[6];    // Current workpiece coordinate system values; x,y,z,rx,ry,rz
      double extoolCoord[6];   // Current external tool coordinate system values; x,y,z,rx,ry,rz
      double exAxisCoord[6];   // Current extension axis coordinate system values; x,y,z,rx,ry,rz
      double load;        // Load mass
      double loadCog[3];     // Load center of gravity
      double lastServoTarget[6]; // Last ServoJ target position in the queue
      int servoJCmdNum;      // ServoJ command count
      double targetJointPos[6];  // 6 joints command position, unit °
      double targetJointVel[6];  // 6 joints command velocity, unit °/s
      double targetJointAcc[6];  // 6 joints command acceleration, unit °/s2
      double targetJointCurrent[6]; // 6 joints command current, unit A
      double actualJointCurrent[6]; // 6 joints current current, unit A
      double actualTCPForce[6];  // Robot end-effector torque Nm; x,y,z,rx,ry,rz
      double targetTCPPos[6];   // Robot TCP command position mm; x,y,z,rx,ry,rz
      uint8_t collisionLevel[6]; // Robot collision level
      double speedScaleManual;  // Manual mode global speed percentage
      double speedScaleAuto;   // Automatic mode global speed percentage
      int luaLineNum;       // Current Lua program running line number
      uint8_t abnomalStop;    // 0-no abnormality; 1-abnormality present
      char currentLuaFileName[256]; // Name of currently running Lua program
      uint8_t programTotalLine;  // Total lines of Lua program
      uint8_t safetyBoxSingal[6]; // Robot button box button status
      double weldVoltage;     // Welding voltage V
      double weldCurrent;     // Welding current
      double weldTrackVel;    // Seam tracking speed mm/s
      uint8_t tpdException;    // TPD trajectory load count exceeded, 0-not exceeded, 1-exceeded 
      uint8_t alarmRebootRobot;  // Warning, 1-release emergency stop button and power cycle the control box, 2-joint communication abnormality, power cycle the control box
      uint8_t modbusMasterConnect; // bit0-bit7 correspond to ModbusTCP master 0-7 connection status 0-not connected 1-connected
      uint8_t modbusSlaveConnect;  // ModbusTCP slave connection status 0-not connected; 1-connected
      uint8_t btnBoxStopSignal;   // Button box emergency stop signal, 0-emergency stop released; 1-emergency stop pressed
      uint8_t dragAlarm;      // Drag warning, currently in automatic mode, 0-no alarm, 1-alarm, 2-position feedback abnormality, no switching
      uint8_t safetyDoorAlarm;   // Safety door warning; 0-safety door closed; 1-safety door open
      uint8_t safetyPlaneAlarm;   // Entering safety wall warning; 0-not entering safety wall; 1-entered safety wall
      uint8_t motonAlarm;      // Motion warning
      uint8_t interfaceAlarm;    // Entering interference area warning
      int udpCmdState;       // Port 20007 UDP communication connection status
      uint8_t weldReadyState;    // Welder ready status
      uint8_t alarmCheckEmergStopBtn; // 0-normal; 1-communication abnormality, check if emergency stop button is released
      uint8_t tsTmCmdComError;   // 0-normal; 1-torque command communication failure
      uint8_t tsTmStateComError;  // 0-normal; 1-torque status communication failure
      int ctrlBoxError;       // Control box error
      uint8_t safetyDataState;   // Safety data status flag, 0-normal, 1-abnormal
      uint8_t forceSensorErrState; // Force sensor connection timeout fault; bit0-bit1 correspond to force sensor ID1-ID2
      uint8_t ctrlOpenLuaErrCode[4]; // 4 controller peripheral protocol error codes (500 error code)
      uint8_t strangePosFlag; // Currently in singular posture flag; 0-normal; 1-singular posture
      uint8_t alarm;      // Warning
      uint8_t driverAlarm;   // Driver alarm axis number
      uint8_t aliveSlaveNumError; // Active slave count error, 0: normal; 1: count error
      uint8_t slaveComError[8];  // Slave error, 0: normal; 1: slave offline; 2: slave status inconsistent with set value; 3: slave not configured; 4: slave configuration error; 5: slave initialization error; 6: slave mailbox communication initialization error
      uint8_t cmdPointError;   // Command point error
      uint8_t IOError;      // IO error
      uint8_t gripperError;    // Gripper error
      uint8_t fileError;     // File error
      uint8_t paraError;     // Parameter error
      uint8_t exaxisOutLimitError;  // External axis soft limit exceeded error
      uint8_t driverComError[6];   // Driver communication fault
      uint8_t driverError;      // Driver communication fault axis number
      uint8_t outSoftLimitError;   // Soft limit exceeded fault
      char axleGenComData[130];   // Robot end-effector transparent transmission feedback data
      uint8_t socketConnTimeout;   // Socket connection timeout, bit0-bit4: socketID 1-4
      uint8_t socketReadTimeout;   // Socket read timeout, bit0-bit4: socketID 1-4
      uint8_t tsWebStateComErr;   // Web-torque communication failure; 0-normal; 1-failed
      uint16_t check_sum;     // Checksum
    }ROBOT_STATE_PKG;

Robot Status Feedback Configuration Enumeration Type
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    enum class RobotState
    {
        ProgramState = 3,           // Program running status, 1-stopped; 2-running; 3-paused
        RobotState = 4,             // Robot motion status, 1-stopped; 2-running; 3-paused; 4-dragging
        MainCode = 5,               // Main fault code
        SubCode = 6,                // Sub fault code
        RobotMode = 7,              // Robot mode, 1-manual mode; 0-automatic mode
        JointCurPos = 8,            // Current joint positions of 6 axes, unit deg
        ToolCurPos = 9,             // Current tool position: [0] along x-axis (mm), [1] along y-axis (mm), [2] along z-axis (mm), [3] rotation around fixed X-axis (deg), [4] around fixed Y-axis (deg), [5] around fixed Z-axis (deg)
        FlangeCurPos = 10,          // Current end flange position: [0] along x-axis (mm), [1] along y-axis (mm), [2] along z-axis (mm), [3] rotation around fixed X-axis (deg), [4] around fixed Y-axis (deg), [5] around fixed Z-axis (deg)
        ActualJointVel = 11,        // Current velocities of 6 joints, unit deg/s
        ActualJointAcc = 12,        // Current accelerations of 6 joints, unit deg/s^2
        TargetTCPCmpSpeed = 13,     // TCP composite command speed: [0] position (mm/s), [1] orientation (deg/s)
        TargetTCPSpeed = 14,        // TCP command speed: [0] along x-axis (mm/s), [1] along y-axis (mm/s), [2] along z-axis (mm/s), [3] angular velocity around X-axis (deg/s), [4] around Y-axis (deg/s), [5] around Z-axis (deg/s)
        ActualTCPCmpSpeed = 15,     // TCP composite actual speed: [0] position (mm/s), [1] orientation (deg/s)
        ActualTCPSpeed = 16,        // TCP actual speed: [0] along x-axis (mm/s), [1] along y-axis (mm/s), [2] along z-axis (mm/s), [3] angular velocity around X-axis (deg/s), [4] around Y-axis (deg/s), [5] around Z-axis (deg/s)
        ActualJointTorque = 17,     // Current torques of 6 axes, unit N·m
        Tool = 18,                  // Applied tool coordinate system number
        User = 19,                  // Applied workpiece coordinate system number
        ClDgtOutputH = 20,          // Control box digital IO output 15-8
        ClDgtOutputL = 21,          // Control box digital IO output 7-0
        TlDgtOutputL = 22,          // Tool digital IO output 7-0, only bit0-bit1 valid
        ClDgtInputH = 23,           // Control box digital IO input 15-8
        ClDgtInputL = 24,           // Control box digital IO input 7-0
        TlDgtInputL = 25,           // Tool digital IO input 7-0, only bit0-bit1 valid
        ClAnalogInput = 26,         // Control box analog input: [0] channel 0, [1] channel 1
        TlAnalogInput = 27,         // Tool analog input
        FtSensorRawData = 28,       // Force/torque sensor raw data: [0] force along x-axis (N), [1] along y-axis (N), [2] along z-axis (N), [3] torque around x-axis (Nm), [4] around y-axis (Nm), [5] around z-axis (Nm)
        FtSensorData = 29,          // Force/torque sensor data (processed): [0] force along x-axis (N), [1] along y-axis (N), [2] along z-axis (N), [3] torque around x-axis (Nm), [4] around y-axis (Nm), [5] around z-axis (Nm)
        FtSensorActive = 30,        // Force/torque sensor activation status, 0-reset, 1-activated
        EmergencyStop = 31,         // Emergency stop flag, 0-emergency stop not pressed, 1-emergency stop pressed
        MotionDone = 32,            // Motion completion signal, 1-completed, 0-not completed
        GripperMotiondone = 33,     // Gripper motion completion signal, 1-completed, 0-not completed
        McQueueLen = 34,            // Motion command queue length
        CollisionState = 35,        // Collision detection, 1-collision, 0-no collision
        TrajectoryPnum = 36,        // Trajectory point number
        SafetyStop0State = 37,      // Safety stop signal SI0
        SafetyStop1State = 38,      // Safety stop signal SI1
        GripperFaultId = 39,        // Faulty gripper number
        GripperFault = 40,          // Gripper fault
        GripperActive = 41,         // Gripper activation status
        GripperPosition = 42,       // Gripper position
        GripperSpeed = 43,          // Gripper speed
        GripperCurrent = 44,        // Gripper current
        GripperTemp = 45,           // Gripper temperature
        GripperVoltage = 46,        // Gripper voltage
        AuxState = 47,              // 485 extension axis status
        ExtAxisStatus = 48,         // UDP extension axis status (4 axes)
        ExtDIState = 49,            // Extension DI input (8)
        ExtDOState = 50,            // Extension DO output (8)
        ExtAIState = 51,            // Extension AI input (4)
        ExtAOState = 52,            // Extension AO output (4)
        RbtEnableState = 53,        // Robot enable status
        JointDriverTorque = 54,     // Robot joint driver torque (6 joints)
        JointDriverTemperature = 55,// Robot joint driver temperature (6 joints)
        RobotTime = 56,             // Robot system time
        SoftwareUpgradeState = 57,  // Robot software upgrade status
        EndLuaErrCode = 58,         // End Lua running status
        ClAnalogOutput = 59,        // Control box analog output (2)
        TlAnalogOutput = 60,        // Tool analog output
        GripperRotNum = 61,         // Current rotation count of rotating gripper
        GripperRotSpeed = 62,       // Current rotation speed percentage of rotating gripper
        GripperRotTorque = 63,      // Current rotation torque percentage of rotating gripper
        WeldingBreakOffState = 64,  // Welding interruption status
        TargetJointTorque = 65,     // Joint command torque (6 joints)
        SmartToolState = 66,        // SmartTool handle button status
        WideVoltageCtrlBoxTemp = 67,// Wide voltage control box temperature
        WideVoltageCtrlBoxFanCurrent = 68, // Wide voltage control box fan current (mA)
        ToolCoord = 69,             // Current tool coordinate system values: x,y,z,rx,ry,rz
        WobjCoord = 70,             // Current workpiece coordinate system values: x,y,z,rx,ry,rz
        ExtoolCoord = 71,           // Current external tool coordinate system values: x,y,z,rx,ry,rz
        ExAxisCoord = 72,           // Current extension axis coordinate system values: x,y,z,rx,ry,rz
        Load = 73,                  // Load mass
        LoadCog = 74,               // Load center of gravity: x,y,z
        LastServoTarget = 75,       // Last ServoJ target position in the queue (6 joints)
        ServoJCmdNum = 76,          // ServoJ command count
        TargetJointPos = 77,        // 6 joints command position, unit °
        TargetJointVel = 78,        // 6 joints command velocity, unit °/s
        TargetJointAcc = 79,        // 6 joints command acceleration, unit °/s²
        TargetJointCurrent = 80,    // 6 joints command current, unit A
        ActualJointCurrent = 81,    // 6 joints current current, unit A
        ActualTCPForce = 82,        // Robot end-effector torque: x,y,z,rx,ry,rz, unit Nm
        TargetTCPPos = 83,          // Robot TCP command position: x,y,z,rx,ry,rz, unit mm
        CollisionLevel = 84,        // Robot collision level (6)
        SpeedScaleManual = 85,      // Manual mode global speed percentage
        SpeedScaleAuto = 86,        // Automatic mode global speed percentage
        LuaLineNum = 87,            // Current Lua program running line number
        AbnomalStop = 88,           // 0-no abnormality; 1-abnormality present
        CurrentLuaFileName = 89,    // Name of currently running Lua program
        ProgramTotalLine = 90,      // Total lines of Lua program
        SafetyBoxSingal = 91,       // Robot button box button status (6)
        WeldVoltage = 92,           // Welding voltage V
        WeldCurrent = 93,           // Welding current
        WeldTrackVel = 94,          // Seam tracking speed mm/s
        TpdException = 95,          // TPD trajectory load count exceeded, 0-not exceeded, 1-exceeded
        AlarmRebootRobot = 96,      // Warning: 1-release emergency stop and power cycle, 2-joint communication abnormality, power cycle
        ModbusMasterConnect = 97,   // bit0-7 correspond to ModbusTCP master 0-7 connection status, 0-not connected, 1-connected
        ModbusSlaveConnect = 98,    // ModbusTCP slave connection status, 0-not connected, 1-connected
        BtnBoxStopSignal = 99,      // Button box emergency stop signal, 0-emergency stop released, 1-emergency stop pressed
        DragAlarm = 100,            // Drag warning: 0-no alarm, 1-alarm, 2-position feedback abnormality, no switching
        SafetyDoorAlarm = 101,      // Safety door warning: 0-closed, 1-open
        SafetyPlaneAlarm = 102,     // Safety wall warning: 0-not entered, 1-entered
        MotonAlarm = 103,           // Motion warning
        InterfaceAlarm = 104,       // Entering interference area warning
        UdpCmdState = 105,          // Port 20007 UDP communication connection status
        WeldReadyState = 106,       // Welder ready status
        AlarmCheckEmergStopBtn = 107, // 0-normal; 1-communication abnormality, check emergency stop button
        TsTmCmdComError = 108,      // 0-normal; 1-torque command communication failure
        TsTmStateComError = 109,    // 0-normal; 1-torque status communication failure
        CtrlBoxError = 110,         // Control box error
        SafetyDataState = 111,      // Safety data status, 0-normal, 1-abnormal
        ForceSensorErrState = 112,  // Force sensor connection timeout, bit0-bit1 correspond to ID1-ID2
        CtrlOpenLuaErrCode = 113,   // 4 controller peripheral protocol error codes (500 error code)
        StrangePosFlag = 114,       // Singular posture flag: 0-normal, 1-singular posture
        Alarm = 115,                // Warning
        DriverAlarm = 116,          // Driver alarm axis number
        AliveSlaveNumError = 117,   // Active slave count error: 0-normal, 1-count error
        SlaveComError = 118,        // Slave error: 0-normal, 1-offline, 2-state inconsistent, 3-not configured, 4-configuration error, 5-initialization error, 6-mailbox communication initialization error
        CmdPointError = 119,        // Command point error
        IOError = 120,              // IO error
        GripperError = 121,         // Gripper error
        FileError = 122,            // File error
        ParaError = 123,            // Parameter error
        ExaxisOutLimitError = 124,  // External axis soft limit exceeded error
        DriverComError = 125,       // Driver communication fault (6 axes)
        DriverError = 126,          // Driver communication fault axis number
        OutSoftLimitError = 127,    // Soft limit exceeded fault
        AxleGenComData = 128,       // Robot end-effector transparent transmission feedback data
        SocketConnTimeout = 129,    // Socket connection timeout, bit0-bit4 correspond to socketID 1-4
        SocketReadTimeout = 130,    // Socket read timeout, bit0-bit4 correspond to socketID 1-4
        TsWebStateComErr = 131      // Web-torque communication failure: 0-normal, 1-failed
    };