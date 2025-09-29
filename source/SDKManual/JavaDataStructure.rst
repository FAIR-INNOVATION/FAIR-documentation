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
      public int circle_num;              /* Number of spiral circles */
      public double circle_angle;         /* Spiral tilt angle */
      public double rad_init;             /* Initial radius of spiral, unit: mm */
      public double rad_add;              /* Radius increment */
      public double rotaxis_add;          /* Rotation axis direction increment */
      public int rot_direction;           /* Rotation direction, 0-clockwise, 1-counterclockwise */
      public SpiralParam(int circleNum, double circleAngle, double radInit, double radAdd, double rotaxisAdd, int rotDirection)
      {
        circle_num = circleNum;
        circle_angle = circleAngle;
        rad_init = radInit;
        rad_add = radAdd;
        rotaxis_add = rotaxisAdd;
        rot_direction = rotDirection;
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
    * @brief Robot state feedback structure type
    */
    public class ROBOT_STATE_PKG
    {
      public short frame_head = 0;            // Frame header 0x5A5A
      public byte frame_cnt = 0;              // Frame count
      public short data_len = 0;              // Data length
      public int program_state = 0;          // Program running state, 1-stop; 2-run; 3-pause
      public int robot_state = 0;            // Robot motion state, 1-stop; 2-run; 3-pause; 4-drag
      public int main_code = 0;               // Main error code
      public int sub_code = 0;                // Sub error code
      public int robot_mode = 0;             // Robot mode, 0-auto mode; 1-manual mode
      public double[] jt_cur_pos = new double[6];                  // Joint current position
      public double[] tl_cur_pos = new double[6];                  // Tool current pose
      public double[] flange_cur_pos = new double[6];              // End flange current pose
      public double[] actual_qd = new double[6];                   // Robot current joint velocity
      public double[] actual_qdd = new double[6];                  // Robot current joint acceleration
      public double[] target_TCP_CmpSpeed = new double[2];         // Robot TCP composite command velocity
      public double[] target_TCP_Speed = new double[6];            // Robot TCP command velocity
      public double[] actual_TCP_CmpSpeed = new double[2];         // Robot TCP composite actual velocity
      public double[] actual_TCP_Speed = new double[6];            // Robot TCP actual velocity
      public double[] jt_cur_tor = new double[6];                  // Current torque
      public int tool = 0;                        // Tool number
      public int user = 0;                        // Workpiece number
      public int cl_dgt_output_h = 0;            // Digital output 15-8
      public int cl_dgt_output_l = 0;            // Digital output 7-0
      public int tl_dgt_output_l = 0;            // Tool digital output 7-0 (only bit0-bit1 valid)
      public int cl_dgt_input_h = 0;             // Digital input 15-8
      public int cl_dgt_input_l = 0;             // Digital input 7-0
      public int tl_dgt_input_l = 0;             // Tool digital input 7-0 (only bit0-bit1 valid)
      public short[] cl_analog_input = new short[2];          // Control box analog input
      public short tl_anglog_input = 0;                       // Tool analog input
      public double[] ft_sensor_raw_data = new double[6];     // Force/torque sensor raw data
      public double[] ft_sensor_data = new double[6];         // Reference coordinate system force/torque sensor data
      public int ft_sensor_active = 0;           // Force/torque sensor activation state, 0-reset, 1-active
      public int EmergencyStop = 0;              // Emergency stop flag
      public int motion_done = 0;                 // In-position signal
      public int gripper_motiondone = 0;         // Gripper motion completion signal
      public int mc_queue_len = 0;                // Motion queue length
      public int collisionState = 0;             // Collision detection, 1-collision; 0-no collision
      public int trajectory_pnum = 0;             // Trajectory point number
      public int safety_stop0_state = 0;  /* Safety stop signal SI0 */
      public int safety_stop1_state = 0;  /* Safety stop signal SI1 */
      public int gripper_fault_id = 0;    /* Error gripper number */
      public short gripper_fault = 0;      /* Gripper fault */
      public short gripper_active = 0;     /* Gripper activation state */
      public int gripper_position = 0;    /* Gripper position */
      public int gripper_speed = 0;       /* Gripper speed */
      public int gripper_current = 0;     /* Gripper current */
      public int gripper_tmp = 0;          /* Gripper temperature */
      public int gripper_voltage = 0;      /* Gripper voltage */
      public ROBOT_AUX_STATE auxState = new ROBOT_AUX_STATE(); /* 485 extended axis status */
      public EXT_AXIS_STATUS extAxisStatus0 = new EXT_AXIS_STATUS();
      public EXT_AXIS_STATUS extAxisStatus1 = new EXT_AXIS_STATUS();
      public EXT_AXIS_STATUS extAxisStatus2 = new EXT_AXIS_STATUS();
      public EXT_AXIS_STATUS extAxisStatus3 = new EXT_AXIS_STATUS();
      public short[] extDIState = new short[8];        // Extended DI input
      public short[] extDOState = new short[8];        // Extended DO output
      public short[] extAIState = new short[4];        // Extended AI input
      public short[] extAOState = new short[4];        // Extended AO output
      public int rbtEnableState = 0;       // Robot enable state
      public double[] jointDriverTorque = new double[6];       // Joint driver current torque
      public double[] jointDriverTemperature = new double[6];  // Joint driver current temperature
      public ROBOT_TIME robotTime = new ROBOT_TIME();
      public int softwareUpgradeState = 0;   // Robot software upgrade state 0-idle or uploading upgrade package; 1~100: upgrade completion percentage; -1: upgrade software failed; -2: verification failed; -3: version verification failed; -4: decompression failed; -5: user configuration upgrade failed; -6: peripheral configuration upgrade failed; -7: extended axis configuration upgrade failed; -8: robot configuration upgrade failed; -9: DH parameter configuration upgrade failed
      public int endLuaErrCode;              // End LUA running state

      public int[] cl_analog_output = new int[2];  // Control box analog output
      public int tl_analog_output;              // Tool analog output
      public float gripperRotNum;               // Rotary gripper current rotation number
      public int gripperRotSpeed;                // Rotary gripper current rotation speed percentage
      public int gripperRotTorque;	            // Rotary gripper current rotation torque percentage

      public WELDING_BREAKOFF_STATE weldingBreakOffstate = new WELDING_BREAKOFF_STATE(); // Welding breakoff state

      public double[] jt_tgt_tor = new double[6];    // Joint command torque
      public int smartToolState;         // SmartTool handle button state

      public float wideVoltageCtrlBoxTemp;        // Wide voltage control box temperature
      public int wideVoltageCtrlBoxFanVel;   // Wide voltage control box fan speed (mA)

      public double[] toolCoord=new double[6];           // Tool coordinate system
      public double[] wobjCoord=new double[6];		   // Work object coordinate system
      public double[] extoolCoord=new double[6];		   // External tool coordinate system
      public double[] exAxisCoord=new double[6];		   // Extended axis coordinate system
      public double load;                   // Payload mass
      public double[] loadCog=new double[3];             // Payload center of gravity

      public short check_sum = 0;          /* Checksum */

      public ROBOT_STATE_PKG()
      {

      }
    }