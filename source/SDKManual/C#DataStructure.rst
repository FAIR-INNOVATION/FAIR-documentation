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
    struct SpiralParam
    { public int circle_num; /* number of circles */
        public int circle_num; /* Spiral circle number */
        public float circle_angle; /* Spiral inclination */
        public float rad_init; /* Initial radius of the spiral in mm */
        public float rad_add; /* Radius increment */
        public float rotaxis_add; /* Rotational axis direction increment */
        public uint rot_direction; /* Rotation direction, 0-clockwise, 1-counterclockwise */
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

Robot State Feedback Structures Types
++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Robot status feedback struct type.
    */
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct ROBOT_STATE_PKG
    {
    public UInt16 frame_head; // frame header 0x5A5A
    public byte frame_cnt; //frame count
    public UInt16 data_len; //data length 5
    public byte program_state; //program running state, 1-stop; 2-run; 3-pause
    public byte robot_state; //robot motion state, 1-stop; 2-run; 3-pause; 4-drag  
    public int main_code; //main fault code
    public int sub_code; //Sub fault code
    public byte robot_mode; //robot mode, 0-automatic mode; 1-manual mode 16

    [MarshalAs(UnmanagedType.ByValArray,SizeConst=6)].
    public double[] jt_cur_pos; //current position of joints
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
    public double[] tl_cur_pos; // current position of tool
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]  
    public double[] flange_cur_pos; //end flange current pose
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)] 
    public double[] actual_qd; //current joint velocity of the robot
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)] 
    public double[] actual_qdd; //Robot's current joint acceleration  
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)] 
    public double[] target_TCP_CmpSpeed; //robot TCP synthesis command speed                         
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)] 
    public double[] target_TCP_Speed; //robot TCP command speed                        
    [MarshalAs(UnmanagedType.ByValArray,SizeConst = 2)]
    public double[] actual_TCP_CmpSpeed; //robot TCP synthesis actual speed                        
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
    public double[] actual_TCP_Speed; //robot TCP actual speed                      
    [MarshalAs(UnmanagedType.ByValArray,SizeConst = 6)] 
    public double[] jt_cur_tor; //current torque         
    public int tool; //tool number
    public int user; //workpiece number
    public byte cl_dgt_output_h; //digital output 15-8
    public byte cl_dgt_output_l; //digital output 7-0
    public byte tl_dgt_output_l; //tool digital output 7-0 (valid only for bit 0 - bit 1)
    public byte cl_dgt_input_h; //digital input 15-8
    public byte cl_dgt_input_l; //digital input 7-0
    public byte tl_dgt_input_l; //tools digital input 7-0 (valid only for bit 0 - bit 1)                    
    [MarshalAs(UnmanagedType.ByValArray,SizeConst=2)](only bit 0)
    public UInt16[] cl_analog_input; //control box analog input
    public UInt16 tl_anglog_input; //tool analog input                              
    [MarshalAs(UnmanagedType.ByValArray,SizeConst = 6)] 
    public double[] ft_sensor_raw_data; // force/torque sensor raw data
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)] 
    public double[] ft_sensor_data; //force/torque sensor data                           
    public byte ft_sensor_active; //Force/Torque Sensor active, 0-reset, 1-active
    public byte EmergencyStop; //Emergency stop flag
    public int motion_done; //In place signal
    public byte gripper_motiondone; //gripper motion completion signal
    public int mc_queue_len; //motion queue length
    public byte collisionState; //Collision detection, 1-collision; 0-no collision
    public int trajectory_pnum; //Trajectory point number
    public byte safety_stop0_state; /* safety stop signal SI0 */
    public byte safety_stop1_state; /* Safety stop signal SI1 */
    public byte gripper_fault_id; /* error gripper id */               
    public UInt16 gripper_fault; /* Gripper fault */ 
    public UInt16 gripper_active; /* gripper_active status */
    public byte gripper_position; /* Gripper position */
    public byte gripper_speed; /* gripper_speed */
    public byte gripper_current; /* Gripper current */
    public int gripper_tmp; /* gripper_temp */
    public int gripper_voltage; /* Gripper voltage */                
    public ROBOT_AUX_STATE auxState; /* 485 Extended Axis State */ 
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] 
    public EXT_AXIS_STATUS[] extAxisStatus; /* UDP Extended Axis Status */
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)] 
    public UInt16[] extDIState;// Extended DI inputs
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)] 
    public UInt16[] extDOState;//extended DO output
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] 
    public UInt16[] extAIState;//extended AI inputs
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] 
    public UInt16[] extAOState;//extended AO Outputs 
    public int rbtEnableState;       //robot enable state
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
    public double[] jointDriverTorque;               //Joint Drive Torque
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
    public double[] jointDriverTemperature;          //Joint Drive Temperature
    public ROBOT_TIME robotTime;     //Robotic system time
    public int softwareUpgradeState; //Software Upgrade Status 0-Idle or uploading upgrade package; 1~100: Upgrade completion percentage; -1: Upgrade software failure; -2: Verification failure; -3: Version verification failure; -4: Decompression failure; -5: User Configuration Upgrade Failure; -6: Peripheral Configuration Upgrade Failure; -7: Expansion Axis Configuration Upgrade Failure; -8: Robot Configuration Upgrade Failure; -9: DH Parameter Configuration Upgrade Failed
    public UInt16 endLuaErrCode;    //End LUA operational status 
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
    public  UInt16[] cl_analog_output;  //Control box analog output
    public UInt16 tl_analog_output;     //Tool analog output
    public float gripperRotNum;           // The current number of turns of the rotating clamp
    public byte gripperRotSpeed;       //Percentage of the current rotation speed of the rotary clamp
    public byte gripperRotTorque;	   //Percentage of the current rotating torque of the rotating clamp     
    public WELDING_BREAKOFF_STATE weldingBreakOffState;//Welding interrupt status    
    public UInt16 check_sum; /* sum check */                  
    }
