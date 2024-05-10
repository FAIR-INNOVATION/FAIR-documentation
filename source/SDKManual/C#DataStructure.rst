Data structure specification
==============================

.. toctree:: 
    :maxdepth: 5

Data structure specification
++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Joint position data type
    */  
    struct JointPos
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jPos;   /* Six joint positions, unit: deg */
    }

Cartesian spatial location data type
++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Cartesian spatial location data type
    */
    struct DescTran
    {
        public double x;    /* X-axis coordinate, unit: mm  */
        public double y;    /* Y-axis coordinate, unit: mm  */
        public double z;    /* Z-axis coordinate, unit: mm  */
    }

Euler Angle attitude data type
++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Euler Angle attitude data type
    */
    struct Rpy
    {
        public double rx;   /* Rotation Angle about fixed axis X, unit: deg  */
        public double ry;   /* Rotation Angle about fixed axis y, unit: deg  */
        public double rz;   /* Rotation Angle about fixed axis Z, unit: deg  */
    }

Cartesian space pose data type
++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    *@brief Cartesian space pose type
    */
    struct DescPose
    {
        public DescTran tran;     /* Cartesian position  */
        public Rpy rpy;			/* Cartesian space attitude  */
    }

Extension axis position data type
++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Extension axis position data type
    */ 
    struct ExaxisPos
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public double[] ePos;   /* Position of four expansion shafts, unit: mm */
    }

Torque sensor data type
+++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief The force component and torque component of the force sensor
    */
    struct ForceTorque
    {
        public double fx;  /* Component of force along the x axis, unit: N  */
        public double fy;  /* Component of force along the y axis, unit: N  */
        public double fz;  /* Component of force along the z axis, unit: N  */
        public double tx;  /* Component of torque about the X-axis, unit: Nm */
        public double ty;  /* Component of torque about the Y-axis, unit: Nm */
        public double tz;  /* Component of torque about the Z-axis, unit: Nm */
    }

Spiral parameter data type
+++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Spiral parameter data type
    */ 
    struct SpiralParam
    {
        public int circle_num;      /* Coil number  */
        public float circle_angle;  /* Spiral Angle  */
        public float rad_init;      /* Initial radius of spiral, unit: mm  */
        public float rad_add;       /* Radius increment  */
        public float rotaxis_add;   /* Increment in the direction of the axis of rotation  */
        public uint rot_direction;  /* Rotation direction, 0- clockwise, 1- counterclockwise  */
    }

485 extended axis status  structure 
++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6

.. code-block:: c#
    :linenos:

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct ROBOT_AUX_STATE
    {
        public byte servoId;      //servo ID  
        public int servoErrCode;  //servo error code
        public int servoState;    //servo state
        public double servoPos;   //current position
        public float servoVel;    //current speed
        public float servoTorque;  //current torque
    }

Real-time status structure of the robot
++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6

.. code-block:: c#
    :linenos:

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct ROBOT_STATE_PKG
    {
        public UInt16 frame_head;           //Frame header 0x5A5A
        public byte frame_cnt;              //Frame count
        public UInt16 data_len;             //data length  
        public byte program_state;          //Program running status, 1- stop; 2- Run; 3- Pause
        public byte robot_state;            //Robot motion state, 1- stop; 2- Run; 3- Pause; 4- Drag  
        public int main_code;               //main error code
        public int sub_code;                //sub error code
        public byte robot_mode;             //Robot mode, 0-automatic mode; 1- Manual mode

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jt_cur_pos;        //Current joint position
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] tl_cur_pos;       //Current tool position
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] flange_cur_pos;   //Current pose of end flange
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] actual_qd;        //Robot current joint speed
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] actual_qdd;      //Current joint acceleration of robot  
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public double[] target_TCP_CmpSpeed;  //Robot TCP synthesis command speed                         
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] target_TCP_Speed;     //Robot TCP command speed                        
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public double[] actual_TCP_CmpSpeed;   //Robot TCP synthesizes actual speed                        
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] actual_TCP_Speed;   //Robot TCP actual speed                      
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jt_cur_tor;         //Current torque         
        public int tool;                        //tool number
        public int user;                        //workpiece number
        public byte cl_dgt_output_h;            //Digital output 15-8
        public byte cl_dgt_output_l;            //Digital output 7-0
        public byte tl_dgt_output_l;     //tool digital output7-0
        public byte cl_dgt_input_h;             //Digital input 15-8
        public byte cl_dgt_input_l;             //Digital input 7-0
        public byte tl_dgt_input_l;             //tool Digital input 7-0                    
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public UInt16[] cl_analog_input;        //Control box analog input
        public UInt16 tl_anglog_input;          //Tool analog input                              
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] ft_sensor_raw_data;     //Force/torque sensor raw data
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] ft_sensor_data;   //Force/torque sensor data                           
        public byte ft_sensor_active;           //Force/torque sensor active status, 0-reset, 1-activated
        public byte EmergencyStop;              //Emergency stop sign
        public int motion_done;                 //Position signal
        public byte gripper_motiondone;         //gripper movement complete signal
        public int mc_queue_len;                //Motion queue length
        public byte collisionState;             //Collision detection, 1- collision; 0- No collision
        public int trajectory_pnum;             //Track point number
        public byte safety_stop0_state;  /* Safety stop signal SI0 */
        public byte safety_stop1_state;  /* Safety stop signal SI1 */
        public byte gripper_fault_id;    /* gripper error number */     
        public UInt16 gripper_fault;     /* Gripper fault */
        public UInt16 gripper_active;    /* Gripper active status */
        public byte gripper_position;    /* Gripper position */
        public byte gripper_speed;       /* Gripper speed */
        public byte gripper_current;     /* Gripper current */
        public int gripper_tmp;          /* Gripper temperature */
        public int gripper_voltage;      /* Gripper voltage */             
        public ROBOT_AUX_STATE auxState; /* 485Extended axis state */          
        public UInt16 check_sum;         /* Sum check */                 
    }
