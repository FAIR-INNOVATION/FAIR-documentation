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
        uint16_t frame_head;                /* head frame, Specified as 0x5A5A */
        uint8_t  frame_cnt;                 /* frame counts, a loop in range 0-255 */
        uint16_t data_len;                  /* length of data content */
        uint8_t  program_state;             /* Program running status, 1-stop; 2-run; 3-suspend */
        uint8_t  robot_state;               /* Robot motion statu, 1-stop; 2-run; 3- suspend; 4-drag */
        int      main_code;                 /* main error code */
        int      sub_code;                  /* sub-error code */
        uint8_t  robot_mode;                /* robot mode, 0-automatic mode，1-manual mode */
        double   jt_cur_pos[6];             /* current position of 6 joints, unit:deg */
        double   tl_cur_pos[6];             /* current position of tool.
                                               tl_cur_pos[0], X-axis coordinate, unit: mm,
                                               tl_cur_pos[1], Y-axis coordinate, unit: mm,
                                               tl_cur_pos[2], Z-axis coordinate, unit: mm,
                                               tl_cur_pos[3], rotation angle about fixed axis X, unit: deg,
                                               tl_cur_pos[4], rotation angle about fixed axis Y, unit: deg,
                                               tl_cur_pos[5], rotation angle about fixed axis Z, unit: deg,
        double   flange_cur_pos[6];         /* current position of end flange.
                                               flange_cur_pos[0], X-axis coordinate, unit: mm,
                                               flange_cur_pos[1], Y-axis coordinate, unit: mm,
                                               flange_cur_pos[2], Z-axis coordinate, unit: mm,
                                               flange_cur_pos[3], rotation angle about fixed axis X, unit: deg,
                                               flange_cur_pos[4], rotation angle about fixed axis Y, unit: deg,
                                               flange_cur_pos[5], rotation angle about fixed axis Z, unit: deg,
        double   actual_qd[6];              /* current angular speed of 6 joints, unit: deg/s */
        double   actual_qdd[6];             /* current angular acceleration of 6 joints , unit: deg/s^2 */
        double   target_TCP_CmpSpeed[2];    /* target_TCP_CmpSpeed[0], TCP composite command speed (position), unit: mm/s
                                               target_TCP_CmpSpeed[1], TCP composite command speed (posture), unit: deg/s  */
        double   target_TCP_Speed[6];       /* TCP command speed. 
                                               target_TCP_Speed[0],X-axis coordinate speed, unit: mm/s,
                                               target_TCP_Speed[1],Y-axis coordinate speed, unit: mm/s,
                                               target_TCP_Speed[2],Z-axis coordinate speed, unit: mm/s,
                                               target_TCP_Speed[3], rotation angle speed about fixed axis X, unit: deg/s,
                                               target_TCP_Speed[4], rotation angle speed about fixed axis Y, unit: deg/s,
                                               target_TCP_Speed[5], rotation angle speed about fixed axis Z, unit: deg/s,
        double   actual_TCP_CmpSpeed[2];    /* actual_TCP_CmpSpeed[0], TCP composite actual speed (position), unit: mm/s
                                               actual_TCP_CmpSpeed[1], TCP composite actual speed (posture), unit: deg/s */
        double   actual_TCP_Speed[6];       /* TCP actual speed. 
                                               actual_TCP_Speed[0] ,X-axis coordinate speed, unit: mm/s,
                                               actual_TCP_Speed[1] ,Y-axis coordinate speed, unit: mm/s,
                                               actual_TCP_Speed[2] ,Z-axis coordinate speed, unit: mm/s,
                                               actual_TCP_Speed[3] , rotation angle speed about fixed axis X, unit: deg/s,
                                               actual_TCP_Speed[4] , rotation angle speed about fixed axis Y, unit: deg/s,
                                               actual_TCP_Speed[5] , rotation angle speed about fixed axis Z, unit: deg/s */
        double   jt_cur_tor[6];             /*current torque of 6 joints, unit: N·m */
        int      tool;                      /* system number of applied tool coordinate */
        int      user;                      /* system number applied orkpiece coordinate */
        uint8_t  cl_dgt_output_h;           /* Control box digital IO output 15-8 */
        uint8_t  cl_dgt_output_l;           /* Control box digital IO output 7-0 */
        uint8_t  tl_dgt_output_l;           /* Tool digital IO output 7-0, only bit0-bit1 valid */
        uint8_t  cl_dgt_input_h;            /* Control box digital IO input 15-8 */
        uint8_t  cl_dgt_input_l;            /* Control box digital IO input 7-0 */
        uint8_t  tl_dgt_input_l;            /* Tool digital IO input 7-0, only bit0-bit1 valid */
        uint16_t cl_analog_input[2];        /* cl_analog_input[0], Control box analog input 0                                           cl_analog_input[1], Control box analog input 1 */
        uint16_t tl_anglog_input;           /* Tool analog input */
        double   ft_sensor_raw_data[6];     /* orque sensor raw data.
                                               ft_sensor_raw_data[0] ,X-axis coordinate force, unit: N,
                                               ft_sensor_raw_data[1] ,Y-axis coordinate force, unit: N,
                                               ft_sensor_raw_data[2] ,Z-axis coordinate force, unit: N,
                                               ft_sensor_raw_data[3] ,X-axis coordinate torque, unit: Nm,
                                               ft_sensor_raw_data[4] ,Y-axis coordinate torque, unit: Nm,
                                               ft_sensor_raw_data[5] ,Z-axis coordinate torque, unit: Nm */
        double   ft_sensor_data[6];         /* Torque sensor data.
                                               ft_sensor_data[0] ,X-axis coordinate force, unit: N,
                                               ft_sensor_data[1] ,Y-axis coordinate force, unit: N,
                                               ft_sensor_data[2] ,Z-axis coordinate force, unit: N,
                                               ft_sensor_data[3] ,X-axis coordinate torque, unit: Nm,
                                               ft_sensor_data[4] ,Y-axis coordinate torque, unit: Nm,
                                               ft_sensor_data[5] ,Z-axis coordinate torque, unit: Nm */
        uint8_t  ft_sensor_active;          /* Torque sensor activation status, 0-reset, 1-activate */
        uint8_t  EmergencyStop;             /* Emergency stop sign, 0-emergency stop is not pressed, 1-emergency stop is pressed */
        int      motion_done;               /* Movement in place signal, 1-in place, 0-not in place */
        uint8_t  gripper_motiondone;        /* gripper movement completion signal, 1-completed, 0-not completed */
        int      mc_queue_len;              /* Motion command queue length */
        uint8_t  collisionState;            /* Collision detection, 1-have collision, 0-no collision */
        int      trajectory_pnum;           /* track point number */
        uint8_t  safety_stop0_state;        /* safety stop signal SI0 */
        uint8_t  safety_stop1_state;        /* safety stop signal SI1 */
        uint8_t  gripper_fault_id;          /* gripper fault id */
        uint16_t gripper_fault;             /* gripper fault */
        uint16_t gripper_active;            /* gripper active state */
        uint8_t  gripper_position;          /* gripper position */
        int8_t   gripper_speed;             /* gripper speed */
        int8_t   gripper_current;           /* gripper current */
        int      gripper_temp;              /* gripper temperature */
        int      gripper_voltage;           /* gripper voltage*/
        robot_aux_state aux_state;
        EXT_AXIS_STATUS extAxisStatus[4];  /* UDPExtended axis state */
        uint16_t extDIState[8];        //Extended DI
        uint16_t extDOState[8];        //Extended DO
        uint16_t extAIState[4];        //Extended AI
        uint16_t extAOState[4];        //Extended AO
        uint16_t check_sum;            /* Sum check */
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
