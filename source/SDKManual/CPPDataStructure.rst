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
        unsigned int rot_direction;  /* Rotation direction, 0- clockwise, 1- counterclockwise  */
    }SpiralParam;

feedback packet of robot controller state
+++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionchanged:: C++ SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
     * @brief  feedback packet of robot controller state
     */
    typedef struct _ROBOT_STATE_PKG
    {
        uint16_t frame_head;      /* Frame header, fixed as 0x5A5A */
        uint8_t  frame_cnt;       /* Frame counter, cycles 0-255 */
        uint16_t data_len;        /* Length of data content */
        uint8_t  program_state;   /* Program running state: 1-Stop; 2-Run; 3-Pause; */
        uint8_t  robot_state;     /* Robot motion state: 1-Stop; 2-Run; 3-Pause; 4-Drag */
        int      main_code;       /* Main error code */
        int      sub_code;        /* Sub error code */
        uint8_t  robot_mode;      /* Robot mode: 1-Manual mode; 0-Auto mode; */
        double   jt_cur_pos[6];   /* Current joint positions of 6 axes, unit: deg */
        double   tl_cur_pos[6];   /* Current tool position
                                     tl_cur_pos[0]: Position along x-axis, unit: mm,
                                     tl_cur_pos[1]: Position along y-axis, unit: mm,
                                     tl_cur_pos[2]: Position along z-axis, unit: mm,
                                     tl_cur_pos[3]: Rotation angle around fixed X-axis, unit: deg
                                     tl_cur_pos[4]: Rotation angle around fixed y-axis, unit: deg
                                     tl_cur_pos[5]: Rotation angle around fixed z-axis, unit: deg */
        double   flange_cur_pos[6]; /* Current flange position
                                     flange_cur_pos[0]: Position along x-axis, unit: mm,
                                     flange_cur_pos[1]: Position along y-axis, unit: mm,
                                     flange_cur_pos[2]: Position along z-axis, unit: mm,
                                     flange_cur_pos[3]: Rotation angle around fixed X-axis, unit: deg
                                     flange_cur_pos[4]: Rotation angle around fixed y-axis, unit: deg
                                     flange_cur_pos[5]: Rotation angle around fixed z-axis, unit: deg */
        double   actual_qd[6];      /* Current joint velocities of 6 axes, unit: deg/s */
        double   actual_qdd[6];     /* Current joint accelerations of 6 axes, unit: deg/s^2 */
        double   target_TCP_CmpSpeed[2]; /* target_TCP_CmpSpeed[0]: TCP composite command speed (position), unit: mm/s
                                     target_TCP_CmpSpeed[1]: TCP composite command speed (orientation), unit: deg/s */
        double   target_TCP_Speed[6];   /* TCP command speed
                                     target_TCP_Speed[0]: Velocity along x-axis, unit: mm/s,
                                     target_TCP_Speed[1]: Velocity along y-axis, unit: mm/s,
                                     target_TCP_Speed[2]: Velocity along z-axis, unit: mm/s,
                                     target_TCP_Speed[3]: Angular velocity around fixed X-axis, unit: deg/s
                                     target_TCP_Speed[4]: Angular velocity around fixed y-axis, unit: deg/s
                                     target_TCP_Speed[5]: Angular velocity around fixed z-axis, unit: deg/s */
        double   actual_TCP_CmpSpeed[2];/* actual_TCP_CmpSpeed[0]: TCP composite actual speed (position), unit: mm/s
                                     actual_TCP_CmpSpeed[1]: TCP composite actual speed (orientation), unit: deg/s */
        double   actual_TCP_Speed[6];   /* TCP actual speed
                                     actual_TCP_Speed[0]: Velocity along x-axis, unit: mm/s,
                                     actual_TCP_Speed[1]: Velocity along y-axis, unit: mm/s,
                                     actual_TCP_Speed[2]: Velocity along z-axis, unit: mm/s,
                                     actual_TCP_Speed[3]: Angular velocity around fixed X-axis, unit: deg/s
                                     actual_TCP_Speed[4]: Angular velocity around fixed y-axis, unit: deg/s
                                     actual_TCP_Speed[5]: Angular velocity around fixed z-axis, unit: deg/s */
        double   jt_cur_tor[6];      /* Current torque of 6 axes, unit: N·m */
        int      tool;               /* Applied tool coordinate system number */
        int      user;               /* Applied workpiece coordinate system number */
        uint8_t  cl_dgt_output_h;    /* Control box digital IO output 15-8 */
        uint8_t  cl_dgt_output_l;    /* Control box digital IO output 7-0 */
        uint8_t  tl_dgt_output_l;    /* Tool digital IO output 7-0, only bit0-bit1 valid */
        uint8_t  cl_dgt_input_h;     /* Control box digital IO input 15-8 */
        uint8_t  cl_dgt_input_l;     /* Control box digital IO input 7-0 */
        uint8_t  tl_dgt_input_l;     /* Tool digital IO input 7-0, only bit0-bit1 valid */
        uint16_t cl_analog_input[2]; /* cl_analog_input[0]: Control box analog input 0
                                     cl_analog_input[1]: Control box analog input 1 */
        uint16_t tl_anglog_input;    /* Tool analog input */
        double   ft_sensor_raw_data[6]; /* Force-torque sensor raw data
                                     ft_sensor_raw_data[0]: Force along x-axis, unit: N
                                     ft_sensor_raw_data[1]: Force along y-axis, unit: N
                                     ft_sensor_raw_data[2]: Force along z-axis, unit: N
                                     ft_sensor_raw_data[3]: Torque around x-axis, unit: Nm
                                     ft_sensor_raw_data[4]: Torque around y-axis, unit: Nm
                                     ft_sensor_raw_data[5]: Torque around z-axis, unit: Nm */
        double   ft_sensor_data[6];     /* Force-torque sensor data
                                     ft_sensor_data[0]: Force along x-axis, unit: N
                                     ft_sensor_data[1]: Force along y-axis, unit: N
                                     ft_sensor_data[2]: Force along z-axis, unit: N
                                     ft_sensor_data[3]: Torque around x-axis, unit: Nm
                                     ft_sensor_data[4]: Torque around y-axis, unit: Nm
                                     ft_sensor_data[5]: Torque around z-axis, unit: Nm */
        uint8_t  ft_sensor_active;   /* Force-torque sensor active state: 0-Reset, 1-Active */
        uint8_t  EmergencyStop;      /* Emergency stop flag: 0-Not pressed, 1-Pressed */
        int      motion_done;        /* Motion completion signal: 1-Completed, 0-Not completed */
        uint8_t  gripper_motiondone; /* Gripper motion completion signal: 1-Completed, 0-Not completed */
        int      mc_queue_len;       /* Motion command queue length */
        uint8_t  collisionState;     /* Collision detection: 1-Collision, 0-No collision */
        int      trajectory_pnum;    /* Trajectory point number */
        uint8_t  safety_stop0_state; /* Safety stop signal SI0 */
        uint8_t  safety_stop1_state; /* Safety stop signal SI1 */
        uint8_t  gripper_fault_id;   /* Faulty gripper number */
        uint16_t gripper_fault;      /* Gripper fault */
        uint16_t gripper_active;     /* Gripper active state */
        uint8_t  gripper_position;   /* Gripper position */
        int8_t   gripper_speed;      /* Gripper speed */
        int8_t   gripper_current;    /* Gripper current */
        int      gripper_temp;       /* Gripper temperature */
        int      gripper_voltage;    /* Gripper voltage */
        robot_aux_state aux_state;
        EXT_AXIS_STATUS extAxisStatus[4];  /* UDP extended axis status */
        uint16_t extDIState[8];        //Extended DI input
        uint16_t extDOState[8];        //Extended DO output
        uint16_t extAIState[4];        //Extended AI input
        uint16_t extAOState[4];        //Extended AO output
        int rbtEnableState;            //Robot enable state               
        double   jointDriverTorque[6];        //Robot joint driver torque    
        double   jointDriverTemperature[6];   //Robot joint driver temperature   
        RobotTime robotTime;           //Robot system time                
        int softwareUpgradeState;      //Robot software upgrade state            
        uint16_t endLuaErrCode;        //End LUA running state            
        uint16_t cl_analog_output[2];  //Control box analog output               
        uint16_t tl_analog_output;     //Tool analog output            
        float gripperRotNum;           //Rotary gripper current rotation count 
        uint8_t gripperRotSpeed;       //Rotary gripper current rotation speed percentage
        uint8_t gripperRotTorque;      //Rotary gripper current rotation torque percentage
        WELDING_BREAKOFF_STATE weldingBreakOffState;  //Welding breakoff state
        double jt_tgt_tor[6];          //Joint command torque
        int smartToolState;            //SmartTool handle button state
        uint16_t check_sum;            /* Checksum */
    }ROBOT_STATE_PKG;

Servo controller status
+++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
     * @brief  Servo controller status
     */
    typedef struct ROBOT_AUX_STATE
    {
        uint8_t servoId;	/* servo drive ID, range [1-16], corresponding slave ID */
        int servoErrCode;	/* servo drive fault code */
        int servoState;	    /* servo drive status [decimal number converted to binary, bit0: 0-disable,1-enable;bit1:0-not running,1-running; bit2: 0-attch postive limit, 1-not touch negative limit; bit3:0-attach negative limit,1-not attach negative limit; bit4:0-not locate,1-loacated; bit5:0-not zero,1-have zero */
        double servoPos;	/* servo current position mm or ° */
        float servoVel;	    /* Servo current speed mm/s or °/s */
        float servoTorque;  /* servoTorque Servo current torque Nm */
    } robot_aux_state;
