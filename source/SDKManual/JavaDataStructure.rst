Data Structure Description
============================================

.. toctree:: 
    :maxdepth: 5

Joint Position Data Type
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Joint position data type 
    */  
    public class JointPos
    {
      double J1;
      double J2;
      double J3;
      double J4;
      double J5;
      double J6;

      public JointPos(double j1, double j2, double j3, double j4, double j5, double j6)
      {
        J1 = j1;
        J2 = j2;
        J3 = j3;
        J4 = j4;
        J5 = j5;
        J6 = j6;
      }

      public JointPos()
      {

      }
    }

Cartesian Space Position Data Type
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Cartesian space position data type
    */
    public class DescTran
    {
      public double x = 0.0;    /* X-axis coordinate, unit: mm */
      public double y = 0.0;    /* Y-axis coordinate, unit: mm */
      public double z = 0.0;    /* Z-axis coordinate, unit: mm */
      public DescTran(double posX, double posY, double posZ)
      {
        x = posX;
        y = posY;
        z = posZ;
      }

      public DescTran()
      {

      }

    }

Euler Angle Attitude Data Type
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Euler angle attitude data type
    */
    public class Rpy
    {
      public double rx = 0.0;   /* Rotation angle around fixed X-axis, unit: deg */
      public double ry = 0.0;   /* Rotation angle around fixed Y-axis, unit: deg */
      public double rz = 0.0;   /* Rotation angle around fixed Z-axis, unit: deg */
      public Rpy(double rotateX, double rotateY, double rotateZ)
      {
        rx = rotateX;
        ry = rotateY;
        rz = rotateZ;
      }
    }

Cartesian Space Pose Data Type
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    *@brief Cartesian space pose type
    */
    public class DescPose
    {
      public DescTran tran = new DescTran(0.0, 0.0, 0.0);      /* Cartesian space position */
      public Rpy rpy = new Rpy(0.0, 0.0, 0.0);			       /* Cartesian space attitude */

      public DescPose()
      {

      }

      public DescPose(DescTran descTran, Rpy rotateRpy)
      {
        tran = descTran;
        rpy = rotateRpy;
      }

      public DescPose(double tranX, double tranY, double tranZ, double rX, double ry, double rz)
      {
        tran.x = tranX;
        tran.y = tranY;
        tran.z = tranZ;
        rpy.rx = rX;
        rpy.ry = ry;
        rpy.rz = rz;
      }

      public String toString()
      {
        return String.valueOf(tran.x) + "," + String.valueOf(tran.y) + "," + String.valueOf(tran.z) + "," + String.valueOf(rpy.rx) + "," + String.valueOf(rpy.ry) + "," + String.valueOf(rpy.rz);
      }
    }

Extended Axis Position Data Type
++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
.. code-block:: Java
    :linenos:

    /**
    * @brief Extended axis position data type
    */
    public class ExaxisPos
    {
      public double axis1 = 0.0;
      public double axis2 = 0.0;
      public double axis3 = 0.0;
      public double axis4 = 0.0;

      public ExaxisPos()
      {

      }
      public ExaxisPos(double[] exaxisPos)
      {
        axis1 = exaxisPos[0];
        axis2 = exaxisPos[1];
        axis3 = exaxisPos[2];
        axis4 = exaxisPos[3];
      }

      public ExaxisPos(double pos1, double pos2, double pos3, double pos4)
      {
        axis1 = pos1;
        axis2 = pos2;
        axis3 = pos3;
        axis4 = pos4;
      }
    }

Force Torque Sensor Data Type
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Force component and torque component of force sensor
    */
    public class ForceTorque
    {
      public double fx;  /* Force component along X-axis, unit: N */
      public double fy;  /* Force component along Y-axis, unit: N */
      public double fz;  /* Force component along Z-axis, unit: N */
      public double tx;  /* Torque component around X-axis, unit: Nm */
      public double ty;  /* Torque component around Y-axis, unit: Nm */
      public double tz;  /* Torque component around Z-axis, unit: Nm */
      public ForceTorque(double fX, double fY, double fZ, double tX, double tY, double tZ)
      {
        fx = fX;
        fy = fY;
        fz = fZ;
        tx = tX;
        ty = tY;
        tz = tZ;
      }
    }

Spiral Parameter Data Type
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Spiral parameter data type
    */
    public class SpiralParam
    {
        public int circle_num;           /* Number of spiral turns  */
        public double circle_angle;         /* Spiral pitch angle  */
        public double rad_init;             /* Initial spiral radius, in mm  */
        public double rad_add;              /* Radius increment  */
        public double rotaxis_add;          /* Rotation axis increment  */
        public int rot_direction;  /* Rotation direction: 0-clockwise, 1-counterclockwise  */
        public int velAccMode;     /* Velocity acceleration parameter mode: 0-constant angular velocity; 1- Constant linear velocity */
        public SpiralParam(int circleNum, double circleAngle, double radInit, double radAdd, double rotaxisAdd, int rotDirection, int vel_AccMode)
        {
            circle_num = circleNum;
            circle_angle = circleAngle;
            rad_init = radInit;
            rad_add = radAdd;
            rotaxis_add = rotaxisAdd;
            rot_direction = rotDirection;
            velAccMode=vel_AccMode;
        }
    }

Extended Axis Status Type
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Extended axis status type
    */
    public class EXT_AXIS_STATUS
    {
     public double pos = 0;        // Extended axis position
     public double vel = 0;        // Extended axis velocity
     public int errorCode = 0;     // Extended axis error code
     public int ready = 0;        // Servo ready
     public int inPos = 0;        // Servo in position
     public int alarm = 0;        // Servo alarm
     public int flerr = 0;        // Following error
     public int nlimit = 0;       // Negative limit reached
     public int pLimit = 0;       // Positive limit reached
     public int mdbsOffLine = 0;  // Driver 485 bus offline
     public int mdbsTimeout = 0;  // Communication timeout between control card and control box via 485
     public int homingStatus = 0; // Extended axis homing status
    }

Sensor Type
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Sensor type
    */
    public class DeviceConfig
    {
      int company = 0;          // Manufacturer
      int device = 0;           // Type/device number
      int softwareVersion = 0;  // Software version
      int bus = 0;              // Mounting location

      public DeviceConfig()
      {

      }

      public DeviceConfig(int company, int device, int softwareVersion, int bus)
      {
        this.company = company;
        this.device = device;
        this.softwareVersion = softwareVersion;
        this.bus = bus;
      }
    }

485 Extended Axis Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief 485 extended axis configuration
    */
    public class Axis485Param
    {
      int servoCompany;           // Servo driver manufacturer, 1-DynaTech
      int servoModel;             // Servo driver model, 1-FD100-750C
      int servoSoftVersion;       // Servo driver software version, 1-V1.0
      int servoResolution;        // Encoder resolution
      double axisMechTransRatio;  // Mechanical transmission ratio

      public Axis485Param(int company, int model, int softVersion, int resolution, double mechTransRatio)
      {
        servoCompany = company;
        servoModel = model;
        servoSoftVersion = softVersion;
        servoResolution = resolution;
        axisMechTransRatio = mechTransRatio;
      }

      public Axis485Param()
      {

      }
    }

Servo Controller Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Servo controller status
    */
    public class ROBOT_AUX_STATE
    {
      public int servoId = 0;           // Servo driver ID
      public int servoErrCode = 0;       // Servo driver error code
      public int servoState = 0;         // Servo driver status
      public double servoPos = 0;        // Current servo position
      public float servoVel = 0;         // Current servo velocity
      public float servoTorque = 0;      // Current servo torque
    }

Welding Breakoff Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Welding breakoff status
    */
    public class WELDING_BREAKOFF_STATE
    {
      public int breakOffState = 0;  // Welding breakoff status
      public int weldArcState = 0;   // Welding arc breakoff status
    }

UDP Extended Axis Communication Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extended axis communication parameters
    */
    public class UDP_EXT_AXIS_PARAM
    {
      public String ip = "192.168.58.88"; // IP address
      public int port = 2021;            // Port number
      public int period = 2;             // Communication period (ms, default is 2, do not modify this parameter)
      public int lossPkgTime = 50;       // Packet loss detection time (ms)
      public int lossPkgNum = 2;         // Number of packet losses
      public int disconnectTime = 100;   // Communication disconnection confirmation duration
      public int reconnectEnable = 0;    // Communication disconnection auto-reconnect enable 0-disable 1-enable
      public int reconnectPeriod = 100;  // Reconnection interval (ms)
      public int reconnectNum = 3;       // Number of reconnection attempts
      public int selfConnect = 0;       // Whether to automatically establish connection after power restart; 0-do not establish connection; 1-establish connection
    }

Robot State Feedback Structure Type
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Robot status feedback structure type
    */
    public class ROBOT_STATE_PKG {
        public int frame_head;                      // Frame header
        public int frame_cnt;                       // Frame count
        public int data_len;                        // Data length
        public int program_state;                   // Program status - 1-stopped; 2-running; 3-paused
        public int robot_state;                     // Robot motion status - 1-stopped; 2-running; 3-paused; 4-dragging
        public int main_code;                       // Main fault code
        public int sub_code;                        // Sub fault code
        public int robot_mode;                      // Robot mode - 1-manual mode; 0-automatic mode
        public double[] jt_cur_pos = new double[6]; // Current joint positions of 6 axes, unit deg
        public double[] tl_cur_pos = new double[6]; // Current tool position - [x,y,z,rx,ry,rz]
        public double[] flange_cur_pos = new double[6]; // Current end flange position - [x,y,z,rx,ry,rz]
        public double[] actual_qd = new double[6];  // Current velocities of 6 joints, unit deg/s
        public double[] actual_qdd = new double[6]; // Current accelerations of 6 joints, unit deg/s^2
        public double[] target_TCP_CmpSpeed = new double[2]; // TCP composite command speed - [position mm/s, orientation deg/s]
        public double[] target_TCP_Speed = new double[6]; // TCP command speed - [vx,vy,vz,wx,wy,wz]
        public double[] actual_TCP_CmpSpeed = new double[2]; // TCP composite actual speed - [position mm/s, orientation deg/s]
        public double[] actual_TCP_Speed = new double[6]; // TCP actual speed - [vx,vy,vz,wx,wy,wz]
        public double[] jt_cur_tor = new double[6]; // Current joint torque
        public int tool;                            // Tool ID
        public int user;                            // Workpiece ID
        public int cl_dgt_output_h;                 // Control cabinet digital output high byte
        public int cl_dgt_output_l;                 // Control cabinet digital output low byte
        public int tl_dgt_output_l;                 // Tool digital output low byte
        public int cl_dgt_input_h;                  // Control cabinet digital input high byte
        public int cl_dgt_input_l;                  // Control cabinet digital input low byte
        public int tl_dgt_input_l;                  // Tool digital input low byte
        public int[] cl_analog_input = new int[2];  // Control cabinet analog input
        public int tl_anglog_input;                 // Tool analog input
        public double[] ft_sensor_raw_data = new double[6]; // Force sensor raw data
        public double[] ft_sensor_data = new double[6]; // Force sensor data
        public int ft_sensor_active;                // Force sensor activation status
        public int EmergencyStop;                   // Emergency stop status
        public int motion_done;                     // Motion completed
        public int gripper_motiondone;              // Gripper motion completed
        public int mc_queue_len;                    // Motion queue length
        public int collisionState;                  // Collision status
        public int trajectory_pnum;                 // Trajectory point sequence number
        public int safety_stop0_state;              // Safety stop 0 status
        public int safety_stop1_state;              // Safety stop 1 status
        public int gripper_fault_id;                // Gripper fault ID
        public int gripper_fault;                   // Gripper fault
        public int gripper_active;                  // Gripper activation
        public int gripper_position;                // Gripper position
        public int gripper_speed;                   // Gripper speed
        public int gripper_current;                 // Gripper current
        public int gripper_temp;                    // Gripper temperature
        public int gripper_voltage;                 // Gripper voltage
        public AuxState aux_state = new AuxState(); // Internal auxiliary axis status
        public EXT_AXIS_STATUS[] extAxisStatus = new EXT_AXIS_STATUS[4]; // Extension axis status array
        public short[] extDIState = new short[8];   // Extended I/O
        public short[] extDOState = new short[8];   // Extended I/O
        public short[] extAIState = new short[4];   // Extended I/O
        public short[] extAOState = new short[4];   // Extended I/O
        public int rbtEnableState;                  // Robot enable status
        public double[] jointDriverTorque = new double[6]; // Joint driver torque
        public double[] jointDriverTemperature = new double[6]; // Joint driver temperature
        public ROBOT_TIME robotTime = new ROBOT_TIME(); // Robot time object
        public int softwareUpgradeState;            // Software upgrade status
        public int endLuaErrCode;                   // End Lua error code
        public int[] cl_analog_output = new int[2]; // Control cabinet analog output
        public int tl_analog_output;                // Tool analog output
        public float gripperRotNum;                 // Rotating gripper rotation count
        public int gripperRotSpeed;                 // Rotating gripper speed
        public int gripperRotTorque;                // Rotating gripper torque
        public WELDING_BREAKOFF_STATE weldingBreakOffState = new WELDING_BREAKOFF_STATE(); // Welding interruption status
        public double[] jt_tgt_tor = new double[6]; // Target joint torque
        public int smartToolState;                  // Smart tool status
        public float wideVoltageCtrlBoxTemp;        // Wide voltage control box temperature
        public int wideVoltageCtrlBoxFanCurrent;    // Wide voltage control box fan current
        public double[] toolCoord = new double[6];  // Tool coordinate system
        public double[] wobjCoord = new double[6];  // Workpiece coordinate system
        public double[] extoolCoord = new double[6]; // External tool coordinate system
        public double[] exAxisCoord = new double[6]; // Extension axis coordinate system
        public double load;                         // Payload
        public double[] loadCog = new double[3];    // Load center of gravity
        public double[] lastServoTarget = new double[6]; // Last servo J target position
        public int servoJCmdNum;                    // Servo J command count
        public double[] targetJointPos = new double[6]; // Target joint position
        public double[] targetJointVel = new double[6]; // Target joint velocity
        public double[] targetJointAcc = new double[6]; // Target joint acceleration
        public double[] targetJointCurrent = new double[6]; // Target joint current
        public double[] actualJointCurrent = new double[6]; // Actual joint current
        public double[] actualTCPForce = new double[6]; // Actual TCP force
        public double[] targetTCPPos = new double[6]; // Target TCP position
        public int[] collisionLevel = new int[6];   // Collision level
        public double speedScaleManual;              // Manual speed scale
        public double speedScaleAuto;                // Automatic speed scale
        public int luaLineNum;                       // Lua line number
        public int abnomalStop;                      // Abnormal stop
        public String currentLuaFileName;            // Current Lua file name
        public int programTotalLine;                 // Total program lines
        public int[] safetyBoxSingal = new int[6];   // Safety box signal
        public double weldVoltage;                   // Welding voltage
        public double weldCurrent;                   // Welding current
        public double weldTrackVel;                  // Welding tracking speed
        public int tpdException;                     // TPD exception
        public int alarmRebootRobot;                 // Alarm reboot robot
        public int modbusMasterConnect;              // Modbus master connection
        public int modbusSlaveConnect;               // Modbus slave connection
        public int btnBoxStopSignal;                 // Button box stop signal
        public int dragAlarm;                        // Drag alarm
        public int safetyDoorAlarm;                  // Safety door alarm
        public int safetyPlaneAlarm;                 // Safety plane alarm
        public int motonAlarm;                       // Motion alarm
        public int interfaceAlarm;                   // Interference alarm
        public int udpCmdState;                      // UDP command status
        public int weldReadyState;                   // Welding ready status
        public int alarmCheckEmergStopBtn;           // Alarm check emergency stop button
        public int tsTmCmdComError;                  // Command communication error
        public int tsTmStateComError;                // Status communication error
        public int ctrlBoxError;                     // Control box error
        public int safetyDataState;                  // Safety data status
        public int forceSensorErrState;              // Force sensor error status
        public int[] ctrlOpenLuaErrCode = new int[4]; // Control open Lua error code
        public int strangePosFlag;                   // Singular position flag
        public int alarm;                            // Alarm
        public int driverAlarm;                      // Driver alarm
        public int aliveSlaveNumError;               // Alive slave count error
        public int[] slaveComError = new int[8];     // Slave communication error
        public int cmdPointError;                    // Command point error
        public int IOError;                          // IO error
        public int gripperError;                     // Gripper error
        public int fileError;                        // File error
        public int paraError;                        // Parameter error
        public int exaxisOutLimitError;              // Extension axis soft limit exceeded error
        public int[] driverComError = new int[6];    // Driver communication error
        public int driverError;                      // Driver error
        public int outSoftLimitError;                // Soft limit exceeded error
        public byte[] axleGenComData = new byte[130]; // General axis communication data
        public int check_sum;                        // Checksum
        public int socketConnTimeout;                // Socket connection timeout
        public int socketReadTimeout;                // Socket read timeout
        public int tsWebStateComErr;                 // TS Web state communication error
    }

Robot Status Feedback Configuration Result Class
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * Robot status feedback configuration result class, containing status list and period
    */
    public static class StateConfigResult {
      public final List<RobotState> stateList;
      public final int period;
    }

Robot Status Feedback Configuration Enumeration Type
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * Robot status enumeration type
    * Used for real-time status feedback configuration
    */
    public enum RobotState {
        ProgramState,
        RobotState,
        MainCode,
        SubCode,
        RobotMode,
        JointCurPos,
        ToolCurPos,
        FlangeCurPos,
        ActualJointVel,
        ActualJointAcc,
        TargetTCPCmpSpeed,
        TargetTCPSpeed,
        ActualTCPCmpSpeed,
        ActualTCPSpeed,
        ActualJointTorque,
        Tool,
        User,
        ClDgtOutputH,
        ClDgtOutputL,
        TlDgtOutputL,
        ClDgtInputH,
        ClDgtInputL,
        TlDgtInputL,
        ClAnalogInput,
        TlAnglogInput,
        FtSensorRawData,
        FtSensorData,
        FtSensorActive,
        EmergencyStop,
        MotionDone,
        GripperMotiondone,
        McQueueLen,
        CollisionState,
        TrajectoryPnum,
        SafetyStop0State,
        SafetyStop1State,
        GripperFaultId,
        GripperFault,
        GripperActive,
        GripperPosition,
        GripperSpeed,
        GripperCurrent,
        GripperTemp,
        GripperVoltage,
        AuxState,
        ExtAxisStatus,
        ExtDIState,
        ExtDOState,
        ExtAIState,
        ExtAOState,
        RbtEnableState,
        JointDriverTorque,
        JointDriverTemperature,
        RobotTime,
        SoftwareUpgradeState,
        EndLuaErrCode,
        ClAnalogOutput,
        TlAnalogOutput,
        GripperRotNum,
        GripperRotSpeed,
        GripperRotTorque,
        WeldingBreakOffState,
        TargetJointTorque,
        SmartToolState,
        WideVoltageCtrlBoxTemp,
        WideVoltageCtrlBoxFanCurrent,
        ToolCoord,
        WobjCoord,
        ExtoolCoord,
        ExAxisCoord,
        Load,
        LoadCog,
        LastServoTarget,
        ServoJCmdNum,
        TargetJointPos,
        TargetJointVel,
        TargetJointAcc,
        TargetJointCurrent,
        ActualJointCurrent,
        ActualTCPForce,
        TargetTCPPos,
        CollisionLevel,
        SpeedScaleManual,
        SpeedScaleAuto,
        LuaLineNum,
        AbnomalStop,
        CurrentLuaFileName,
        ProgramTotalLine,
        SafetyBoxSingal,
        WeldVoltage,
        WeldCurrent,
        WeldTrackVel,
        TpdException,
        AlarmRebootRobot,
        ModbusMasterConnect,
        ModbusSlaveConnect,
        BtnBoxStopSignal,
        DragAlarm,
        SafetyDoorAlarm,
        SafetyPlaneAlarm,
        MotonAlarm,
        InterfaceAlarm,
        UdpCmdState,
        WeldReadyState,
        AlarmCheckEmergStopBtn,
        TsTmCmdComError,
        TsTmStateComError,
        SocketConnTimeout,
        SocketReadTimeout,
        TsWebStateComErr,
        CtrlBoxError,
        SafetyDataState,
        ForceSensorErrState,
        CtrlOpenLuaErrCode,
        StrangePosFlag,
        Alarm,
        DriverAlarm,
        AliveSlaveNumError,
        SlaveComError,
        CmdPointError,
        IOError,
        GripperError,
        FileError,
        ParaError,
        ExaxisOutLimitError,
        DriverComError,
        DriverError,
        OutSoftLimitError,
        AxleGenComData;
    }