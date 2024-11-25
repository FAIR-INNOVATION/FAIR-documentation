Data Structure Description
=====================================

.. toctree:: 
    :maxdepth: 5

Joint Position Data Type
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Joint Position Data Type 
    */  
    public class JointPos
    {
      double J1.
      double J2.
      double J3.
      double J4.
      double J5.
      double J6.

      public JointPos(double j1, double j2, double j3, double j4, double j5, double j6)
      {
        J1 = j1;
        J2 = j2.
        J3 = j3.
        J4 = j4.
        J5 = j5.
        J6 = j6.
      }

      public JointPos()
      {

      }
    }

Cartesian spatial position data types
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * :: @brief Cartesian spatial position data types
    */
    public class DescTran
    {
      public double x = 0.0; /* x-axis coordinates in mm */
      public double y = 0.0; /* y-axis coordinates in mm */
      public double z = 0.0; /* z-axis coordinates in mm */
      public DescTran(double posX, double posY, double posZ)
      {
        x = posX.
        y = posY;
        z = posZ.
      }

      public DescTran()
      {

      }

    }

Eulerian angle pose data type
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Eulerian angle pose data type
    */
    public class Rpy
    {
      public double rx = 0.0; /* Angle of rotation around fixed axis X in deg */
      public double ry = 0.0; /* Angle of rotation around fixed axis Y, unit: deg */
      public double rz = 0.0; /* Angle of rotation around fixed axis Z, unit: deg */
      public Rpy(double rotateX, double rotateY, double rotateZ)
      {
        rx = rotateX.
        ry = rotateY.
        rz = rotateZ.
      }
    }

Cartesian space position data types
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    *@brief Cartesian space position type
    */
    public class DescPose
    {
      public DescTran tran = new DescTran(0.0, 0.0, 0.0); /* Cartesian space position */
      public Rpy rpy = new Rpy(0.0, 0.0, 0.0);			       /* Cartesian space gesture */

      public DescPose()
      {

      }

      public DescPose(DescTran descTran, Rpy rotateRpy)
      {
        tran = descTran.
        rpy = rotateRpy.
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
        return String.valueOf(tran.x) + "," + String.valueOf(tran.y) + "," +String.valueOf(tran.z) + "," +String.valueOf(rpy.rx) + "," +String.valueOf( rpy.ry) + "," +String.valueOf(rpy.rz);
      }
    }

Extended Axis Position Data Type
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Extended Axis Position Data Type
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
        axis2 = pos2.
        axis3 = pos3.
        axis4 = pos4.
      }
    }

Torque Sensor Data Type
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Force components and moment components of force sensors
    */
    public class ForceTorque
    {
      public double fx; /* force component along x-axis, in N */
      public double fy; /* force component along y-axis, in N */
      public double fz; /* Force component along z-axis, unit N */
      public double tx; /* Moment component around x-axis, unit Nm */
      public double ty; /* Moment component around y-axis, unit Nm */
      public double tz; /* Moment component around z-axis, unit Nm */
      public ForceTorque(double fX, double fY, double fZ, double tX, double tY, double tZ)
      {
        fx = fX.
        fy = fY.
        fz = fZ.
        tx = tX.
        ty = tY.
        tz = tZ.
      }
    }

Spiral Parameter Data Type
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Spiral parameter data type
    */
    public class SpiralParam
    {
      public int circle_num; /* number of circles */
      public double circle_angle; /* spiral inclination */
      public double rad_init; /* Spiral initial radius in mm */
      public double rad_add; /* radius increment */
      public double rotaxis_add; /* rotaxis direction increment */
      public int rot_direction; /* rot_direction, 0-clockwise, 1-counterclockwise */
      public SpiralParam(int circleNum, double circleAngle, double radInit, double radAdd, double rotaxisAdd, int rotDirection)
      {
        circle_num = circleNum;
        circle_angle = circleAngle;
        rad_init = radInit;
        rad_add = radAdd;
        rotaxis_add = rotaxisAdd;
        rot_direction = rotDirection.
      }
    }

Extended Axis State Type
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Extended axis state type
    */
    public class EXT_AXIS_STATUS
    {
     public double pos = 0; //extended axis position
     public double vel = 0; //extended axis velocity
     public int errorCode = 0; //extended shaft error code
     public int ready = 0; //servo ready
     public int inPos = 0; //servo in place
     public int alarm = 0; //servo alarm
     public int flerr = 0; //following error
     public int nlimit = 0; //to the negative limit
     public int pLimit = 0; //to the positive limit
     public int mdbsOffLine = 0; // drive 485 bus offline
     public int mdbsTimeout = 0; //control card and control box 485 communication timeout
     public int homingStatus = 0; //extended axis back to zero status
    }

Sensor type
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Sensor type
    */
    public class DeviceConfig
    {
      int company = 0; // vendor
      int device = 0; // type/device number
      int softwareVersion = 0; // software version
      int bus = 0; // mount position

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
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief 485 extended axis configuration
    */
    public class Axis485Param
    {
      int servoCompany; // Servo Drive Manufacturer, 1 - Dynatec
      int servoModel; // servo drive model, 1-FD100-750C
      int servoSoftVersion; // Servo drive software version, 1-V1.0
      int servoResolution; // encoder resolution
      double axisMechTransRatio; // mechanical transmission ratio

      public Axis485Param(int company, int model, int softVersion, int resolution, double mechTransRatio)
      {
        servoCompany = company;
        servoModel = model;
        servoSoftVersion = softVersion;
        servoResolution = resolution;
        axisMechTransRatio = mechTransRatio.
      }

      public Axis485Param()
      {

      }
    }

Servo controller status
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Servo controller status
    */
    public class ROBOT_AUX_STATE
    {
      public int servoId = 0; // servo drive ID number
      public int servoErrCode = 0; // servo drive error code
      public int servoState = 0; // servo drive state
      public double servoPos = 0; // servo current position
      public float servoVel = 0; // servo current speed
      public float servoTorque = 0; // servo current torque 25
    }

Robot Status Feedback Structure Type
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Robot status feedback structure type
    */
    public class ROBOT_STATE_PKG
    {
      public short frame_head = 0; //frame head 0x5A5A
      public byte frame_cnt = 0; //frame counter
      public short data_len = 0; //data length 5
      public int program_state = 0; //program running state, 1-stop; 2-run; 3-pause
      public int robot_state = 0; // robot motion state, 1-stop; 2-run; 3-pause; 4-drag 7
      public int main_code = 0; //main fault code
      public int sub_code = 0; //sub fault code
      public int robot_mode = 0; //robot mode, 0-automatic mode; 1-manual mode 16
      public double[] jt_cur_pos =new double[6]; //current position of joints
      public double[] tl_cur_pos = new double[6]; //tool current position posture
      public double[] flange_cur_pos = new double[6]; //end flange current bit position
      public double[] actual_qd = new double[6]; //robot current joint velocity
      public double[] actual_qdd = new double[6]; //robot current joint acceleration
      public double[] target_TCP_CmpSpeed = new double[2]; //robot TCP synthesis command speed
      public double[] target_TCP_Speed = new double[6]; //robot TCP command speed
      public double[] actual_TCP_CmpSpeed = new double[2]; // robot TCP synthesis actual speed
      public double[] actual_TCP_Speed = new double[6]; // robot TCP actual speed
      public double[] jt_cur_tor = new double[6]; //current torque
      public int tool = 0; //tool number
      public int user = 0; //artifact number
      public int cl_dgt_output_h = 0; // digital output 15-8
      public int cl_dgt_output_l = 0; //digital output 7-0
      public int tl_dgt_output_l = 0; //tool digital output 7-0 (only bit0-bit1 valid)
      public int cl_dgt_input_h = 0; // digital input 15-8
      public int cl_dgt_input_l = 0; //digital input 7-0
      public int tl_dgt_input_l = 0; //tool digital input 7-0 (only bit0-bit1 valid)
      public short[] cl_analog_input = new short[2]; //control box analog input
      public short tl_anglog_input = 0; //tool analog input
      public double[] ft_sensor_raw_data = new double[6]; // force/torque sensor raw data
      public double[] ft_sensor_data = new double[6]; //Force/torque sensor data in reference coordinate system
      public int ft_sensor_active = 0; //Force/torque sensor active, 0-reset, 1-active
      public int EmergencyStop = 0; //Emergency stop flag
      public int motion_done = 0; //in place signal
      public int gripper_motiondone = 0; //gripper motion done signal
      public int mc_queue_len = 0; //motion queue length
      public int collisionState = 0; //collision detection, 1-collision; 0-no collision
      public int trajectory_pnum = 0; //trajectory point number
      public int safety_stop0_state = 0; /* Safety stop signal SI0 */
      public int safety_stop1_state = 0; /* Safety stop signal SI1 */
      public int gripper_fault_id = 0; /* error gripper number */ // + 19 = 567
      public short gripper_fault = 0; /* gripper_fault */
      public short gripper_active = 0; /* Gripper active status */
      public int gripper_position = 0; /* Gripper position */
      public int gripper_speed = 0; /* Gripper speed */
      public int gripper_current = 0; /* Gripper current */
      public int gripper_tmp = 0; /* Gripper temperature */
      public int gripper_voltage = 0; /* gripper_voltage */
      public ROBOT_AUX_STATE auxState = new ROBOT_AUX_STATE(); /* 485 Extended Axis State */
      public EXT_AXIS_STATUS extAxisStatus0 = new EXT_AXIS_STATUS();
      public EXT_AXIS_STATUS extAxisStatus1 = new EXT_AXIS_STATUS();
      public EXT_AXIS_STATUS extAxisStatus2 = new EXT_AXIS_STATUS();
      public EXT_AXIS_STATUS extAxisStatus3 = new EXT_AXIS_STATUS();
      public short[] extDIState = new short[8]; //extended DI inputs
      public short[] extDOState = new short[8]; //extend DO outputs
      public short[] extAIState = new short[4]; //extended AI inputs
      public short[] extAOState = new short[4]; //extended AO outputs
      public int rbtEnableState = 0; //robot enable state--robot enable s
      public double[] jointDriverTorque =new double[6]; // joint driver current torque
      public double[] jointDriverTemperature = new double[6]; // joint driver current temperature
      public ROBOT_TIME robotTime = new ROBOT_TIME();
      public int softwareUpgradeState = 0; //Robot software upgrade state 0-Idle or uploading upgrade package; 1~100: percentage of upgrade completion; -1: upgrade software failure; -2: verification failure; -3: version verification failure; -4: decompression failure; -5: user configuration upgrade failure; -6: peripheral configuration upgrade failure; -7: Failed to upgrade extended axis configuration; -8: Failed to upgrade robot configuration; -9: Failed to upgrade DH parameter configuration
      public int endLuaErrCode; //end LUA running state
      public short check_sum = 0; /* sum check */

      public ROBOT_STATE_PKG()
      {

      }
    }