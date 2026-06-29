Data Structure Description
===================================================

.. toctree:: 
    :maxdepth: 5

Joint position data type
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Joint position data type 
    */  
    struct JointPos
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jPos; /* Six joint positions in deg */
    }

Cartesian space position data type
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Cartesian spatial position datatype.
    */
    struct DescTran
    {
        public double x; /* x-axis coordinate in mm */
        public double y; /* y-axis coordinate in mm */
        public double z; /* z-axis coordinate in mm */
    }

Euler Angle Attitude data type
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Euler Angle Attitude data type.
    */
    struct Rpy
    {
    public double rx; /* Angle of rotation around fixed axis X in deg */
    public double ry; /* Angle of rotation around fixed axis Y in degrees */
    public double rz; /* Angle of rotation about fixed axis Z in degrees */
    }

Cartesian space position data type
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    *@brief Cartesian space position type.
    */
    struct DescPose
    {
        public DescTran tran; /* Cartesian space position */
        public Rpy rpy; /* Cartesian space pose */
    }

Extended axis position data type
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended axis position datatype.
    */
    struct ExaxisPos
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public double[] ePos; /* Four extended axis positions in mm */
    }

Torque sensor data type
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Force components and moment components of a force sensor.
    */
    struct ForceTorque
    { public double fx; /* Force component along x-axis, in N */
        public double fx; /* Force component along x-axis in N */
        public double fy; /* Force component along y-axis in N */
        public double fz; /* Component of force along z-axis in N */
        public double tx; /* Component of moment around x-axis, unit Nm */ 
        public double ty; /* Component of moment around y-axis, in Nm */
        public double tz; /* Moment component around z-axis, in Nm */ 
    }

Helix parameter data type
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Spiral parameter data types
    */
    public struct SpiralParam
    {
        public int circle_num;           /* Number of spiral turns */
        public float circle_angle;       /* Spiral inclination angle */
        public float rad_init;           /* Initial spiral radius, unit: mm */
        public float rad_add;            /* Radius increment */
        public float rotaxis_add;        /* Rotation axis direction increment */
        public uint rot_direction;       /* Rotation direction, 0-clockwise, 1-counterclockwise */
        public int velAccMode;           // Velocity/acceleration parameter mode: 0-constant angular velocity; 1-constant linear velocity

        public SpiralParam(int num, float angle, float initRad, float addRad, float axisAdd, uint direction, int mode)
        {
            circle_num = num;
            circle_angle = angle;
            rad_init = initRad;
            rad_add = addRad;
            rotaxis_add = axisAdd;
            rot_direction = direction;
            velAccMode = mode;
        }
    }

Extended axis state type
++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Extended axis state type
    */
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct ROBOT_AUX_STATE
    {
        public byte servoId; // servo drive ID number
        public int servoErrCode; //servo drive error code
        public int servoState; //servo drive state
        public double servoPos; //Servo current position
        public float servoVel; //Servo current speed
        public float servoTorque; //Servo current torque
    }

Welding interrupt status
+++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct WELDING_BREAKOFF_STATE
    {
        public byte breakOffState;  // Welding interrupt status
        public byte weldArcState;   // Welding arc interrupted state
    }

Robot Status Feedback Structure Type
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.4  Web-3.8.3
    
.. code-block:: c#
    :linenos:

    /**
    * @brief  Robot status feedback structure type
    */
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public class ROBOT_STATE_PKG
    {
        public UInt16 frame_head;           // Frame header 0x5A5A
        public byte frame_cnt;              // Frame count
        public UInt16 data_len;             // Data length 5
        public byte program_state;          // Program running status, 1-stopped; 2-running; 3-paused;
        public byte robot_state;            // Robot motion status, 1-stopped; 2-running; 3-paused; 4-dragging
        public int main_code;               // Main fault code
        public int sub_code;                // Sub fault code
        public byte robot_mode;             // Robot mode, 1-manual mode; 0-automatic mode;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jt_cur_pos;         // Current joint positions of 6 axes, unit deg
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] tl_cur_pos;         // Current tool position
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] flange_cur_pos;     // Current end flange position
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] actual_qd;          // Current velocities of 6 joints, unit deg/s
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] actual_qdd;         // Current accelerations of 6 joints, unit deg/s^2
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public double[] target_TCP_CmpSpeed;// TCP composite command speed (position, orientation)
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] target_TCP_Speed;   // TCP command speed
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public double[] actual_TCP_CmpSpeed;// TCP composite actual speed
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] actual_TCP_Speed;   // TCP actual speed
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jt_cur_tor;         // Current torques of 6 axes, unit N·m

        public int tool;                    // Applied tool coordinate system number
        public int user;                    // Applied workpiece coordinate system number
        public byte cl_dgt_output_h;        // Control box digital IO output 15-8
        public byte cl_dgt_output_l;        // Control box digital IO output 7-0
        public byte tl_dgt_output_l;        // Tool digital IO output 7-0, only bit0-bit1 valid
        public byte cl_dgt_input_h;         // Control box digital IO input 15-8
        public byte cl_dgt_input_l;         // Control box digital IO input 7-0
        public byte tl_dgt_input_l;         // Tool digital IO input 7-0, only bit0-bit1 valid

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public UInt16[] cl_analog_input;        // Control box analog input
        public UInt16 tl_anglog_input;          // Tool analog input                            

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] ft_sensor_raw_data; // Force/torque sensor raw data
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] ft_sensor_data;     // Force/torque sensor data
        public byte ft_sensor_active;       // Force/torque sensor activation status, 0-reset, 1-activated

        public byte EmergencyStop;          // Emergency stop flag, 0-not pressed, 1-pressed
        public int motion_done;             // Motion completion signal, 1-completed, 0-not completed
        public byte gripper_motiondone;     // Gripper motion complete signal, 0-not complete, 1-complete (no object detected), 2-motion complete (object detected)
        public int mc_queue_len;            // Motion command queue length
        public byte collisionState;         // Collision detection, 1-collision, 0-no collision
        public int trajectory_pnum;         // Trajectory point number
        public byte safety_stop0_state;     // Safety stop signal SI0
        public byte safety_stop1_state;     // Safety stop signal SI1
        public byte gripper_fault_id;       // Faulty gripper number
        public UInt16 gripper_fault;     /* Gripper fault 0-no fault 1-485 timeout 2-command error 3-workpiece dropped Other-gripper fault code */
        public UInt16 gripper_active;    /* Gripper activation status */
        public byte gripper_position;       // Gripper position
        public byte gripper_speed;       /* Gripper speed */
        public byte gripper_current;     /* Gripper current */
        public int gripper_temp;            // Gripper temperature
        public int gripper_voltage;         // Gripper voltage

        public ROBOT_AUX_STATE auxState;   // 485 extension axis status

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public EXT_AXIS_STATUS[] extAxisStatus; // UDP extension axis status

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public UInt16[] extDIState;        // Extension DI input
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public UInt16[] extDOState;        // Extension DO output
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public UInt16[] extAIState;        // Extension AI input
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public UInt16[] extAOState;        // Extension AO output

        public int rbtEnableState;          // Robot enable status

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jointDriverTorque;      // Robot joint driver torque
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jointDriverTemperature; // Robot joint driver temperature

        public ROBOT_TIME robotTime;        // Robot system time
        public int softwareUpgradeState;    // Robot software upgrade status
        public UInt16 endLuaErrCode;    // End Lua running status 

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public  UInt16[] cl_analog_output;  // Control box analog output
        public UInt16 tl_analog_output;     // Tool analog output

        public float gripperRotNum;         // Current rotation count of rotating gripper
        public byte gripperRotSpeed;        // Current rotation speed percentage of rotating gripper
        public byte gripperRotTorque;       // Current rotation torque percentage of rotating gripper

        public WELDING_BREAKOFF_STATE weldingBreakOffState; // Welding interruption status

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jt_tgt_tor;         // Joint command torque
        public int smartToolState;          // SmartTool handle button status
        public float wideVoltageCtrlBoxTemp; // Wide voltage control box temperature
        public UInt16 wideVoltageCtrlBoxFanVel;   // Wide voltage control box fan current (mA)

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] toolCoord;          // Current tool coordinate system values; x,y,z,rx,ry,rz
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] wobjCoord;          // Current workpiece coordinate system values; x,y,z,rx,ry,rz
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] extoolCoord;        // Current external tool coordinate system values; x,y,z,rx,ry,rz
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] exAxisCoord;        // Current extension axis coordinate system values; x,y,z,rx,ry,rz

        public double load;                 // Load mass
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] loadCog;            // Load center of gravity
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] lastServoTarget;    // Last ServoJ target position in the queue
        public int servoJCmdNum;            // ServoJ command count

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] targetJointPos;     // 6 joints command position, unit °
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] targetJointVel;     // 6 joints command velocity, unit °/s
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] targetJointAcc;     // 6 joints command acceleration, unit °/s²
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] targetJointCurrent; // 6 joints command current, unit A
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] actualJointCurrent; // 6 joints current current, unit A
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] actualTCPForce;     // Robot end-effector torque Nm; x,y,z,rx,ry,rz
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] targetTCPPos;       // Robot TCP command position mm; x,y,z,rx,ry,rz
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public byte[] collisionLevel;       // Robot collision level

        public double speedScaleManual;     // Manual mode global speed percentage
        public double speedScaleAuto;       // Automatic mode global speed percentage
        public int luaLineNum;              // Current Lua program running line number
        public byte abnomalStop;            // 0-no abnormality; 1-abnormality present

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256)]
        public byte[] currentLuaFileName;   // Name of currently running Lua program
        public byte programTotalLine;       // Total lines of Lua program
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public byte[] safetyBoxSingal;      // Robot button box button status

        public double weldVoltage;          // Welding voltage V
        public double weldCurrent;          // Welding current
        public double weldTrackVel;         // Seam tracking speed mm/s

        public byte tpdException;           // TPD trajectory load count exceeded, 0-not exceeded, 1-exceeded
        public byte alarmRebootRobot;       // Warning, 1-release emergency stop button and power cycle the control box, 2-joint communication abnormality, power cycle the control box
        public byte modbusMasterConnect;    // bit0-bit7 correspond to ModbusTCP master 0-7 connection status 0-not connected 1-connected
        public byte modbusSlaveConnect;     // ModbusTCP slave connection status 0-not connected; 1-connected
        public byte btnBoxStopSignal;       // Button box emergency stop signal, 0-emergency stop released; 1-emergency stop pressed
        public byte dragAlarm;              // Drag warning, currently in automatic mode, 0-no alarm, 1-alarm, 2-position feedback abnormality, no switching
        public byte safetyDoorAlarm;        // Safety door warning; 0-safety door closed; 1-safety door open
        public byte safetyPlaneAlarm;       // Entering safety wall warning; 0-not entering safety wall; 1-entered safety wall
        public byte motonAlarm;             // Motion warning
        public byte interfaceAlarm;         // Entering interference area warning
        public int udpCmdState;             // Port 20007 UDP communication connection status
        public byte weldReadyState;         // Welder ready status
        public byte alarmCheckEmergStopBtn; // 0-normal; 1-communication abnormality, check if emergency stop button is released
        public byte tsTmCmdComError;        // 0-normal; 1-torque command communication failure
        public byte tsTmStateComError;      // 0-normal; 1-torque status communication failure
        public int ctrlBoxError;            // Control box error
        public byte safetyDataState;        // Safety data status flag, 0-normal, 1-abnormal
        public byte forceSensorErrState;    // Force sensor connection timeout fault; bit0-bit1 correspond to force sensor ID1-ID2

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public byte[] ctrlOpenLuaErrCode;   // 4 controller peripheral protocol error codes (500 error code)

        public byte strangePosFlag;         // Currently in singular posture flag; 0-normal; 1-singular posture
        public byte alarm;                  // Warning
        public byte driverAlarm;            // Driver alarm axis number
        public byte aliveSlaveNumError;     // Active slave count error, 0: normal; 1: count error

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public byte[] slaveComError;        // Slave error, 0: normal; 1: slave offline; 2: slave status inconsistent with set value; 3: slave not configured; 4: slave configuration error; 5: slave initialization error; 6: slave mailbox communication initialization error

        public byte cmdPointError;          // Command point error
        public byte IOError;                // IO error
        public byte gripperError;           // Gripper error
        public byte fileError;              // File error
        public byte paraError;              // Parameter error
        public byte exaxisOutLimitError;    // External axis soft limit exceeded error

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public byte[] driverComError;       // Driver communication fault
        public byte driverError;            // Driver communication fault axis number
        public byte outSoftLimitError;      // Soft limit exceeded fault

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 130)]
        public byte[] axleGenComData;       // Robot end-effector transparent transmission feedback data

        public byte socketConnTimeout;     // Socket connection timeout flag
        public byte socketReadTimeout;     // Socket read timeout flag
        public byte tsWebStateComErr;      // ts_web_state_com_err
        public byte exaxisCoordID;         // Extension axis coordinate system number
        public UInt16 check_sum;         /* Checksum */                 

        // Constructor: initialize all array fields
        public ROBOT_STATE_PKG()
        {
            jt_cur_pos = new double[6];
            tl_cur_pos = new double[6];
            flange_cur_pos = new double[6];
            actual_qd = new double[6];
            actual_qdd = new double[6];
            target_TCP_CmpSpeed = new double[2];
            target_TCP_Speed = new double[6];
            actual_TCP_CmpSpeed = new double[2];
            actual_TCP_Speed = new double[6];
            jt_cur_tor = new double[6];
            cl_analog_input = new ushort[2];
            ft_sensor_raw_data = new double[6];
            ft_sensor_data = new double[6];
            extAxisStatus = new EXT_AXIS_STATUS[4];
            extDIState = new ushort[8];
            extDOState = new ushort[8];
            extAIState = new ushort[4];
            extAOState = new ushort[4];
            jointDriverTorque = new double[6];
            jointDriverTemperature = new double[6];
            cl_analog_output = new ushort[2];
            jt_tgt_tor = new double[6];
            toolCoord = new double[6];
            wobjCoord = new double[6];
            extoolCoord = new double[6];
            exAxisCoord = new double[6];
            loadCog = new double[3];
            lastServoTarget = new double[6];
            targetJointPos = new double[6];
            targetJointVel = new double[6];
            targetJointAcc = new double[6];
            targetJointCurrent = new double[6];
            actualJointCurrent = new double[6];
            actualTCPForce = new double[6];
            targetTCPPos = new double[6];
            collisionLevel = new byte[6];
            currentLuaFileName = new byte[256];
            safetyBoxSingal = new byte[6];
            ctrlOpenLuaErrCode = new byte[4];
            slaveComError = new byte[8];
            driverComError = new byte[6];
            axleGenComData = new byte[130];
        }
    }

Robot Configurable Status Enumeration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief  Robot configurable status enumeration, range 3~131
    */
    public enum RobotState
    {
        ProgramState = 3,
        RobotState = 4,
        MainCode = 5,
        SubCode = 6,
        RobotMode = 7,
        JointCurPos = 8,
        ToolCurPos = 9,
        FlangeCurPos = 10,
        ActualJointVel = 11,
        ActualJointAcc = 12,
        TargetTCPCmpSpeed = 13,
        TargetTCPSpeed = 14,
        ActualTCPCmpSpeed = 15,
        ActualTCPSpeed = 16,
        ActualJointTorque = 17,
        Tool = 18,
        User = 19,
        ClDgtOutputH = 20,
        ClDgtOutputL = 21,
        TlDgtOutputL = 22,
        ClDgtInputH = 23,
        ClDgtInputL = 24,
        TlDgtInputL = 25,
        ClAnalogInput = 26,
        TlAnglogInput = 27,
        FtSensorRawData = 28,
        FtSensorData = 29,
        FtSensorActive = 30,
        EmergencyStop = 31,
        MotionDone = 32,
        GripperMotiondone = 33,
        McQueueLen = 34,
        CollisionState = 35,
        TrajectoryPnum = 36,
        SafetyStop0State = 37,
        SafetyStop1State = 38,
        GripperFaultId = 39,
        GripperFault = 40,
        GripperActive = 41,
        GripperPosition = 42,
        GripperSpeed = 43,
        GripperCurrent = 44,
        GripperTemp = 45,
        GripperVoltage = 46,
        AuxState = 47,
        ExtAxisStatus = 48,
        ExtDIState = 49,
        ExtDOState = 50,
        ExtAIState = 51,
        ExtAOState = 52,
        RbtEnableState = 53,
        JointDriverTorque = 54,
        JointDriverTemperature = 55,
        RobotTime = 56,
        SoftwareUpgradeState = 57,
        EndLuaErrCode = 58,
        ClAnalogOutput = 59,
        TlAnalogOutput = 60,
        GripperRotNum = 61,
        GripperRotSpeed = 62,
        GripperRotTorque = 63,
        WeldingBreakOffState = 64,
        TargetJointTorque = 65,
        SmartToolState = 66,
        WideVoltageCtrlBoxTemp = 67,
        WideVoltageCtrlBoxFanCurrent = 68,
        ToolCoord = 69,
        WobjCoord = 70,
        ExtoolCoord = 71,
        ExAxisCoord = 72,
        Load = 73,
        LoadCog = 74,
        LastServoTarget = 75,
        ServoJCmdNum = 76,
        TargetJointPos = 77,
        TargetJointVel = 78,
        TargetJointAcc = 79,
        TargetJointCurrent = 80,
        ActualJointCurrent = 81,
        ActualTCPForce = 82,
        TargetTCPPos = 83,
        CollisionLevel = 84,
        SpeedScaleManual = 85,
        SpeedScaleAuto = 86,
        LuaLineNum = 87,
        AbnomalStop = 88,
        CurrentLuaFileName = 89,
        ProgramTotalLine = 90,
        SafetyBoxSingal = 91,
        WeldVoltage = 92,
        WeldCurrent = 93,
        WeldTrackVel = 94,
        TpdException = 95,
        AlarmRebootRobot = 96,
        ModbusMasterConnect = 97,
        ModbusSlaveConnect = 98,
        BtnBoxStopSignal = 99,
        DragAlarm = 100,
        SafetyDoorAlarm = 101,
        SafetyPlaneAlarm = 102,
        MotonAlarm = 103,
        InterfaceAlarm = 104,
        UdpCmdState = 105,
        WeldReadyState = 106,
        AlarmCheckEmergStopBtn = 107,
        TsTmCmdComError = 108,
        TsTmStateComError = 109,
        CtrlBoxError = 110,
        SafetyDataState = 111,
        ForceSensorErrState = 112,
        CtrlOpenLuaErrCode = 113,
        StrangePosFlag = 114,
        Alarm = 115,
        DriverAlarm = 116,
        AliveSlaveNumError = 117,
        SlaveComError = 118,
        CmdPointError = 119,
        IOError = 120,
        GripperError = 121,
        FileError = 122,
        ParaError = 123,
        ExaxisOutLimitError = 124,
        DriverComError = 125,
        DriverError = 126,
        OutSoftLimitError = 127,
        AxleGenComData = 128,
        SocketConnTimeout = 129,     // Socket connection timeout, bit0-bit4: socketID 1-4
        SocketReadTimeout = 130,     // Socket read timeout, bit0-bit4: socketID 1-4
        TsWebStateComErr = 131     // Web-torque communication failure; 0-normal; 1-failed
    }