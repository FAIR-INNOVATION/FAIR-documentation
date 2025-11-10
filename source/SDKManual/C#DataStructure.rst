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

Robot State Feedback Structures Types
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.4  Web-3.8.3
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Robot status feedback struct type.
    */
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct ROBOT_STATE_PKG
    {
        public UInt16 frame_head;           //Frame header 0x5A5A
        public byte frame_cnt;              //Frame count
        public UInt16 data_len;             //Data length 5
        public byte program_state;          //Program running state, 1-stop; 2-running; 3-pause
        public byte robot_state;            //Robot motion state, 1-stop; 2-running; 3-pause; 4-drag 7
        public int main_code;               //Main fault code
        public int sub_code;                //Sub fault code
        public byte robot_mode;             //Robot mode, 0-auto mode; 1-manual mode 16

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jt_cur_pos;                             //Joint current position
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] tl_cur_pos;                             //Tool current pose
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] flange_cur_pos;                         //End flange current pose
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] actual_qd;                              //Robot current joint velocity
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] actual_qdd;                             //Robot current joint acceleration 16 + 8 * 6 * 5 = 256
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public double[] target_TCP_CmpSpeed;                    //Robot TCP composite command velocity                        
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] target_TCP_Speed;                       //Robot TCP command velocity                        
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public double[] actual_TCP_CmpSpeed;                    //Robot TCP composite actual velocity                     
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] actual_TCP_Speed;                       //Robot TCP actual velocity                     
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jt_cur_tor;                             //Current torque       
        public int tool;                        //Tool number
        public int user;                        //Workpiece number
        public byte cl_dgt_output_h;            //Digital output 15-8
        public byte cl_dgt_output_l;            //Digital output 7-0
        public byte tl_dgt_output_l;            //Tool digital output 7-0 (only bit0-bit1 valid)
        public byte cl_dgt_input_h;             //Digital input 15-8
        public byte cl_dgt_input_l;             //Digital input 7-0
        public byte tl_dgt_input_l;             //Tool digital input 7-0 (only bit0-bit1 valid)                  
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public UInt16[] cl_analog_input;        //Control box analog input
        public UInt16 tl_anglog_input;          //Tool analog input                            
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] ft_sensor_raw_data;     //Force/torque sensor raw data
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] ft_sensor_data;         //Force/torque sensor data                          
        public byte ft_sensor_active;           //Force/torque sensor activation state, 0-reset, 1-active
        public byte EmergencyStop;              //Emergency stop flag
        public int motion_done;                 //In-position signal
        public byte gripper_motiondone;         //Gripper motion complete signal
        public int mc_queue_len;                //Motion queue length
        public byte collisionState;             //Collision detection, 1-collision; 0-no collision
        public int trajectory_pnum;             //Trajectory point number
        public byte safety_stop0_state;  /* Safety stop signal SI0 */
        public byte safety_stop1_state;  /* Safety stop signal SI1 */
        public byte gripper_fault_id;    /* Faulty gripper number */             
        public UInt16 gripper_fault;     /* Gripper fault */
        public UInt16 gripper_active;    /* Gripper activation state */
        public byte gripper_position;    /* Gripper position */
        public byte gripper_speed;       /* Gripper speed */
        public byte gripper_current;     /* Gripper current */
        public int gripper_tmp;          /* Gripper temperature */
        public int gripper_voltage;      /* Gripper voltage */                 
        public ROBOT_AUX_STATE auxState; /* 485 extension axis state */          
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public EXT_AXIS_STATUS[] extAxisStatus;  /* UDP extension axis state */
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public UInt16[] extDIState;        //Extended DI input
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public UInt16[] extDOState;        //Extended DO output
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public UInt16[] extAIState;        //Extended AI input
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public UInt16[] extAOState;        //Extended AO output
        public int rbtEnableState;       //Robot enable state
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jointDriverTorque;               //Joint driver torque
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jointDriverTemperature;          //Joint driver temperature
        public ROBOT_TIME robotTime;     //Robot system time
        public int softwareUpgradeState; //Software upgrade state 0-idle or uploading upgrade package; 1~100: upgrade completion percentage; -1: upgrade software failed; -2: verification failed; -3: version verification failed; -4: decompression failed; -5: user configuration upgrade failed; -6: peripheral configuration upgrade failed; -7: extension axis configuration upgrade failed; -8: robot configuration upgrade failed; -9: DH parameter configuration upgrade failed
        public UInt16 endLuaErrCode;    //End Lua running state 
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public UInt16[] cl_analog_output;  //Control box analog output
        public UInt16 tl_analog_output;     //Tool analog output
        public float gripperRotNum;           //Rotating gripper current rotation turns
        public byte gripperRotSpeed;       //Rotating gripper current rotation speed percentage
        public byte gripperRotTorque;	   //Rotating gripper current rotation torque percentage
        public WELDING_BREAKOFF_STATE weldingBreakOffState;//Welding breakoff state

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jt_tgt_tor;//Joint command torque
        public int smartToolState; //SmartTool handle button state
        public float wideVoltageCtrlBoxTemp;        //Wide voltage control box temperature
        public UInt16 wideVoltageCtrlBoxFanVel;   //Wide voltage control box fan current (mA)

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] toolCoord;         //Tool coordinate system
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] wobjCoord;         //Workpiece coordinate system
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] extoolCoord;        //External tool coordinate system
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] exAxisCoord;          //Extension axis coordinate system
        public double load;                   //Load mass

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] loadCog;           //Load center of gravity
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] lastServoTarget;//Last servoJ target position in queue

        public int servoJCmdNum;// servoJ command count
        public UInt16 check_sum;         /* Checksum */                 
    }
