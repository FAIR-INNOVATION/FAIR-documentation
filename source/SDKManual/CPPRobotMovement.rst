Robot Motion
============

.. toctree:: 
    :maxdepth: 5


JOG Motion
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief JOG motion
    * @param [in] ref 0-Joint JOG, 2-Base coordinate JOG, 4-Tool coordinate JOG, 8-Workpiece coordinate JOG
    * @param [in] nb 1-Joint1 (or x-axis), 2-Joint2 (or y-axis), 3-Joint3 (or z-axis), 4-Joint4 (or rotation about x-axis), 5-Joint5 (or rotation about y-axis), 6-Joint6 (or rotation about z-axis)
    * @param [in] dir 0-Negative direction, 1-Positive direction
    * @param [in] vel Speed percentage [0~100]
    * @param [in] acc Acceleration percentage [0~100]
    * @param [in] max_dis Maximum single JOG angle in [°] or distance in [mm]
    * @return Error code
    */
    errno_t StartJOG(uint8_t ref, uint8_t nb, uint8_t dir, float vel, float acc, float max_dis);

JOG Deceleration Stop
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief JOG deceleration stop
    * @param [in] ref 1-Joint JOG stop, 3-Base coordinate JOG stop, 5-Tool coordinate JOG stop, 9-Workpiece coordinate JOG stop
    * @return Error code
    */
    errno_t StopJOG(uint8_t ref);

JOG Immediate Stop
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief JOG immediate stop
    * @return Error code
    */
    errno_t ImmStopJOG();

Robot JOG Control Example
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestJOG(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        for (int i = 0; i < 6; i++)
        {
            robot.StartJOG(0, i + 1, 0, 20.0, 20.0, 30.0);
            robot.Sleep(1000);
            robot.ImmStopJOG();
            robot.Sleep(1000);
        }
        for (int i = 0; i < 6; i++)
        {
            robot.StartJOG(2, i + 1, 0, 20.0, 20.0, 30.0);
            robot.Sleep(1000);
            robot.ImmStopJOG();
            robot.Sleep(1000);
        }
        for (int i = 0; i < 6; i++)
        {
            robot.StartJOG(4, i + 1, 0, 20.0, 20.0, 30.0);
            robot.Sleep(1000);
            robot.StopJOG(5);
            robot.Sleep(1000);
        }
        for (int i = 0; i < 6; i++)
        {
            robot.StartJOG(8, i + 1, 0, 20.0, 20.0, 30.0);
            robot.Sleep(1000);
            robot.StopJOG(9);
            robot.Sleep(1000);
        }
        robot.CloseRPC();
        return 0;
    }

Joint Space Motion
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint space motion
    * @param [in] joint_pos Target joint position in deg
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool Tool coordinate number [0~14]
    * @param [in] user Workpiece coordinate number [0~14]
    * @param [in] vel Speed percentage [0~100]
    * @param [in] acc Acceleration percentage [0~100] (not yet available)
    * @param [in] ovl Speed scaling factor [0~100]
    * @param [in] epos Extended axis position in mm
    * @param [in] blendT [-1.0]-Move to position (blocking), [0~500.0]-Smoothing time (non-blocking) in ms
    * @param [in] offset_flag 0-No offset, 1-Offset in base/workpiece coordinate, 2-Offset in tool coordinate
    * @param [in] offset_pos Pose offset
    * @return Error code
    */
    errno_t  MoveJ(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos *epos, float blendT, uint8_t offset_flag, DescPose *offset_pos);

Joint Space Motion (Automatic Forward Kinematics Calculation)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint Space Motion (Automatic Forward Kinematics Calculation)
    * @param [in] joint_pos Target joint position, unit: deg
    * @param [in] tool Tool coordinate number, range [0~14]
    * @param [in] user Workpiece coordinate number, range [0~14]
    * @param [in] vel Velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], temporarily not available
    * @param [in] ovl Velocity scaling factor, range [0~100]
    * @param [in] epos Extended axis position, unit: mm
    * @param [in] blendT [-1.0]-Move to position (blocking), [0~500.0]-Smoothing time (non-blocking), unit: ms
    * @param [in] offset_flag 0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos Pose offset value
    * @return Error code
    */
    errno_t MoveJ(JointPos* joint_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos* epos, float blendT, uint8_t offset_flag, DescPose* offset_pos);
   
Cartesian Space Linear Motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Cartesian Space Linear Motion
    * @param [in] joint_pos Target joint positions, unit: deg
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool Tool coordinate system index, range [0~14]
    * @param [in] user Workpiece/user coordinate system index, range [0~14]
    * @param [in] vel Velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100] (currently unavailable)
    * @param [in] ovl Velocity scaling factor [0~100] / physical velocity (mm/s)
    * @param [in] blendR [-1.0]-block until motion completes (blocking), [0~1000.0]-blend radius (non-blocking), unit: mm
    * @param [in] blendMode Transition method; 0-tangent blend; 1-corner blend
    * @param [in] epos Extended axis position, unit: mm
    * @param [in] search 0-no wire search, 1-wire search
    * @param [in] offset_flag 0-no offset, 1-offset in base/workpiece coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos Pose offset value
    * @param [in] oacc Acceleration scaling factor [0-100] / physical acceleration (mm/s²)
    * @param [in] velAccParamMode Velocity/acceleration parameter mode; 0-percentage; 1-physical velocity (mm/s) and acceleration (mm/s²)
    * @param [in] overSpeedStrategy Overspeed handling strategy; 1-standard; 2-stop with error on overspeed; 3-adaptive speed reduction, default 0
    * @param [in] speedPercent Allowable speed reduction threshold percentage [0-100], default 10%
    * @return Error code
    */
    errno_t MoveL(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int blendMode, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos, float oacc = 100.0, int velAccParamMode = 0, int overSpeedStrategy = 0, int speedPercent = 10);

Cartesian Space Linear Motion (Automatic Inverse Kinematics Calculation)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Cartesian Space Linear Motion (Automatic Inverse Kinematics Calculation)
    * @param [in] desc_pos  Target Cartesian pose
    * @param [in] tool Tool coordinate number, range [0~14]
    * @param [in] user Workpiece coordinate number, range [0~14]
    * @param [in] vel Velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], temporarily not available
    * @param [in] ovl Velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-Move to position (blocking), [0~1000.0]-Smoothing radius (non-blocking), unit: mm
    * @param [in] blendMode Transition method; 0-Tangent transition; 1-Corner transition
    * @param [in] epos Extended axis position, unit: mm
    * @param [in] search 0-No wire search, 1-Wire search
    * @param [in] offset_flag 0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos Pose offset value
    * @param [in] config Inverse kinematics joint space configuration, [-1]-Calculate based on current joint position, [0~7]-Solve based on specific joint space configuration
    * @param [in] overSpeedStrategy Overspeed handling strategy, 1-Standard; 2-Error stop when overspeed; 3-Adaptive speed reduction, default is 0
    * @param [in] speedPercent Allowable speed reduction threshold percentage [0-100], default 10%
    * @return Error code
    */
    errno_t MoveL(DescPose* desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int blendMode, ExaxisPos* epos, uint8_t search, uint8_t offset_flag, DescPose* offset_pos, int config = -1, int overSpeedStrategy = 0, int speedPercent = 10);

Cartesian Space Circular Motion
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Cartesian Space Circular Motion
    * @param  [in] joint_pos_p  Path point joint position, unit: deg
    * @param  [in] desc_pos_p   Path point Cartesian pose
    * @param  [in] ptool  Tool coordinate number, range [0~14]
    * @param  [in] puser  Workpiece coordinate number, range [0~14]
    * @param  [in] pvel  Velocity percentage, range [0~100]
    * @param  [in] pacc  Acceleration percentage, range [0~100], temporarily not available
    * @param  [in] epos_p  Extended axis position, unit: mm
    * @param  [in] poffset_flag  0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param  [in] offset_pos_p  Pose offset value
    * @param  [in] joint_pos_t  Target point joint position, unit: deg
    * @param  [in] desc_pos_t   Target point Cartesian pose
    * @param  [in] ttool  Tool coordinate number, range [0~14]
    * @param  [in] tuser  Workpiece coordinate number, range [0~14]
    * @param  [in] tvel  Velocity percentage, range [0~100]
    * @param  [in] tacc  Acceleration percentage, range [0~100], temporarily not available
    * @param  [in] epos_t  Extended axis position, unit: mm
    * @param  [in] toffset_flag  0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param  [in] offset_pos_t  Pose offset value   
    * @param [in] ovl Velocity scaling factor [0~100] / Physical velocity (mm/s)
    * @param [in] blendR [-1.0]-Block until motion completes (blocking), [0~1000.0]-Blend radius (non-blocking), unit: mm
    * @param [in] oacc Acceleration scaling factor [0-100] / Physical acceleration (mm/s²)
    * @param [in] velAccParamMode Velocity/acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) acceleration (mm/s²)
    * @return Error code
    */
    errno_t MoveC(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, uint8_t poffset_flag, DescPose *offset_pos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, uint8_t toffset_flag, DescPose *offset_pos_t, float ovl, float blendR, float oacc = 100.0, int velAccParamMode = 0);

Cartesian Space Circular Motion (Automatic Inverse Kinematics Calculation)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Cartesian Space Circular Motion (Automatic Inverse Kinematics Calculation)
    * @param [in] desc_pos_p  Path point Cartesian pose
    * @param [in] ptool Tool coordinate number, range [0~14]
    * @param [in] puser Workpiece coordinate number, range [0~14]
    * @param [in] pvel Velocity percentage, range [0~100]
    * @param [in] pacc Acceleration percentage, range [0~100], temporarily not available
    * @param [in] epos_p Extended axis position, unit: mm
    * @param [in] poffset_flag 0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos_p Pose offset value
    * @param [in] desc_pos_t  Target point Cartesian pose
    * @param [in] ttool Tool coordinate number, range [0~14]
    * @param [in] tuser Workpiece coordinate number, range [0~14]
    * @param [in] tvel Velocity percentage, range [0~100]
    * @param [in] tacc Acceleration percentage, range [0~100], temporarily not available
    * @param [in] epos_t Extended axis position, unit: mm
    * @param [in] toffset_flag 0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos_t Pose offset value
    * @param [in] ovl Velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-Move to position (blocking), [0~1000.0]-Smoothing radius (non-blocking), unit: mm
    * @param [in] config Inverse kinematics joint space configuration, [-1]-Calculate based on current joint position, [0~7]-Solve based on specific joint space configuration
    * @return Error code
    */
    errno_t MoveC(DescPose* desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos* epos_p, uint8_t poffset_flag, DescPose* offset_pos_p, DescPose* desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos* epos_t, uint8_t toffset_flag, DescPose* offset_pos_t, float ovl, float blendR, int config = -1);

Cartesian Space Full Circle Motion
++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     *@brief  Cartesian Space Full Circle Motion
     *@param  [in] joint_pos_p  Path point 1 joint position, unit: deg
     *@param  [in] desc_pos_p   Path point 1 Cartesian pose
     *@param  [in] ptool  Tool coordinate number, range [1~15]
     *@param  [in] puser  Workpiece coordinate number, range [1~15]
     *@param  [in] pvel  Velocity percentage, range [0~100]
     *@param  [in] pacc  Acceleration percentage, range [0~100], temporarily not available
     *@param  [in] epos_p  Extended axis position, unit: mm
     *@param  [in] joint_pos_t  Path point 2 joint position, unit: deg
     *@param  [in] desc_pos_t   Path point 2 Cartesian pose
     *@param  [in] ttool  Tool coordinate number, range [1~15]
     *@param  [in] tuser  Workpiece coordinate number, range [1~15]
     *@param  [in] tvel  Velocity percentage, range [0~100]
     *@param  [in] tacc  Acceleration percentage, range [0~100], temporarily not available
     *@param  [in] epos_t  Extended axis position, unit: mm
     *@param  [in] ovl Velocity scaling factor [0~100] / Physical velocity (mm/s)
     *@param  [in] offset_flag 0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
     *@param  [in] offset_pos Pose offset value
     *@param  [in] oacc Acceleration scaling factor [0-100] / Physical acceleration (mm/s²)
     *@param  [in] blendR -1: Blocking; 0~1000: Smoothing radius
    * @param  [in] velAccParamMode Velocity/acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) acceleration (mm/s²)
    * @return Error code
    */
    errno_t Circle(JointPos* joint_pos_p, DescPose* desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos* epos_p, JointPos* joint_pos_t, DescPose* desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos* epos_t, float ovl, uint8_t offset_flag, DescPose* offset_pos, double oacc = 100.0, double blendR = -1, int velAccParamMode = 0);

Cartesian Space Full Circle Motion (Automatic Inverse Kinematics Calculation)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Cartesian Space Full Circle Motion (Automatic Inverse Kinematics Calculation)
    * @param [in] desc_pos_p  Path point 1 Cartesian pose
    * @param [in] ptool Tool coordinate number, range [0~14]
    * @param [in] puser Workpiece coordinate number, range [0~14]
    * @param [in] pvel Velocity percentage, range [0~100]
    * @param [in] pacc Acceleration percentage, range [0~100], temporarily not available
    * @param [in] epos_p Extended axis position, unit: mm
    * @param [in] desc_pos_t  Path point 2 Cartesian pose
    * @param [in] ttool Tool coordinate number, range [0~14]
    * @param [in] tuser Workpiece coordinate number, range [0~14]
    * @param [in] tvel Velocity percentage, range [0~100]
    * @param [in] tacc Acceleration percentage, range [0~100], temporarily not available
    * @param [in] epos_t Extended axis position, unit: mm
    * @param [in] ovl Velocity scaling factor, range [0~100]
    * @param [in] offset_flag 0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos Pose offset value
    * @param [in] oacc Acceleration percentage
    * @param [in] blendR -1: Blocking; 0~1000: Smoothing radius
    * @param [in] config Inverse kinematics joint space configuration, [-1]-Calculate based on current joint position, [0~7]-Solve based on specific joint space configuration
    * @return Error code
    */
    errno_t Circle(DescPose* desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos* epos_p, DescPose* desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos* epos_t, float ovl, uint8_t offset_flag, DescPose* offset_pos, double oacc = 100.0, double blendR = -1, int config = -1);
    
Cartesian Space Point-to-Point Motion
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Cartesian Space Point-to-Point Motion
    * @param  [in]  desc_pos  Target Cartesian pose or pose increment
    * @param  [in] tool  Tool coordinate number, range [0~14]
    * @param  [in] user  Workpiece coordinate number, range [0~14]
    * @param  [in] vel  Velocity percentage, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], temporarily not available
    * @param  [in] ovl  Velocity scaling factor, range [0~100]
    * @param  [in] blendT [-1.0]-Move to position (blocking), [0~500.0]-Smoothing time (non-blocking), unit: ms 
    * @param  [in] config  Joint space configuration, [-1]-Calculate based on current joint position, [0~7]-Calculate based on specific joint space configuration, default is -1   
    * @return  Error code
    */
    errno_t  MoveCart(DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendT, int config);

Robot Basic Motion Instruction Code Example
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestMove(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;

        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);

        JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        JointPos j3(-29.777, -84.536, 109.275, -114.075, -86.655, 74.257);
        JointPos j4(-31.154, -95.317, 94.276, -88.079, -89.740, 74.256);
        DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose desc_pos3(-487.434, 154.362, 308.576, 176.600, 0.268, -14.061);
        DescPose desc_pos4(-443.165, 147.881, 480.951, 179.511, -0.775, -15.409);
        DescPose offset_pos(0, 0, 0, 0, 0, 0);
        ExaxisPos epos(0, 0, 0, 0);
        int tool = 0;
        int user = 0;
        float vel = 100.0;
        float acc = 100.0;
        float ovl = 100.0;
        float oacc = 100.0;
        float blendT = 0.0;
        float blendR = 0.0;
        uint8_t flag = 0;
        uint8_t search = 0;
        int blendMode = 0;
        int velAccMode = 0;
        robot.SetSpeed(20);
        rtn = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        printf("movej errcode:%d\n", rtn);
        rtn = robot.MoveL(&j2, &desc_pos2, tool, user, vel, acc, ovl, blendR, blendMode, &epos, search, flag, &offset_pos, oacc, velAccMode);
        printf("movel errcode:%d\n", rtn);
        rtn = robot.MoveC(&j3, &desc_pos3, tool, user, vel, acc, &epos, flag, &offset_pos, &j4, &desc_pos4, tool, user, vel, acc, &epos, flag, &offset_pos, ovl, blendR, oacc, velAccMode);
        printf("movec errcode:%d\n", rtn);
        rtn = robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        printf("movej errcode:%d\n", rtn);
        rtn = robot.Circle(&j3, &desc_pos3, tool, user, vel, acc, &epos, &j1, &desc_pos1, tool, user, vel, acc, &epos, ovl, flag, &offset_pos, oacc, -1, velAccMode);
        printf("circle errcode:%d\n", rtn);
        rtn = robot.MoveCart(&desc_pos4, tool, user, vel, acc, ovl, blendT, -1);
        printf("MoveCart errcode:%d\n", rtn);
        rtn = robot.MoveJ(&j1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        printf("movej errcode:%d\n", rtn);
        rtn = robot.MoveL(&desc_pos2, tool, user, vel, acc, ovl, blendR, blendMode, &epos, search, flag, &offset_pos, -1, velAccMode);
        printf("movel errcode:%d\n", rtn);
        rtn = robot.MoveC(&desc_pos3, tool, user, vel, acc, &epos, flag, &offset_pos, &desc_pos4, tool, user, vel, acc, &epos, flag, &offset_pos, ovl, blendR, -1, velAccMode);
        printf("movec errcode:%d\n", rtn);
        rtn = robot.MoveJ(&j2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        printf("movej errcode:%d\n", rtn);
        rtn = robot.Circle(&desc_pos3, tool, user, vel, acc, &epos, &desc_pos1, tool, user, vel, acc, &epos, ovl, flag, &offset_pos, oacc, blendR, -1, velAccMode);
        printf("circle errcode:%d\n", rtn);
        robot.CloseRPC();
        return 0;
    }

Cartesian Space Spiral Motion
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Cartesian Space Spiral Motion
    * @param  [in] joint_pos  Target joint position, unit: deg
    * @param  [in] desc_pos   Target Cartesian pose
    * @param  [in] tool  Tool coordinate number, range [0~14]
    * @param  [in] user  Workpiece coordinate number, range [0~14]
    * @param  [in] vel  Velocity percentage, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], temporarily not available
    * @param  [in] epos  Extended axis position, unit: mm
    * @param  [in] ovl  Velocity scaling factor, range [0~100]    
    * @param  [in] offset_flag  0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param  [in] offset_pos  Pose offset value
    * @param  [in] spiral_param  Spiral parameters
    * @return  Error code
    */
    errno_t  NewSpiral(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, ExaxisPos *epos, float ovl, uint8_t offset_flag, DescPose *offset_pos, SpiralParam spiral_param);  

Cartesian Space Spiral Motion (Automatic Inverse Kinematics Calculation)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Cartesian Space Spiral Motion (Automatic Inverse Kinematics Calculation)
    * @param [in] desc_pos  Target Cartesian pose
    * @param [in] tool Tool coordinate number, range [0~14]
    * @param [in] user Workpiece coordinate number, range [0~14]
    * @param [in] vel Velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], temporarily not available
    * @param [in] epos Extended axis position, unit: mm
    * @param [in] ovl Velocity scaling factor, range [0~100]
    * @param [in] offset_flag 0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos Pose offset value
    * @param [in] spiral_param Spiral parameters
    * @param [in] config Inverse kinematics joint space configuration, [-1]-Calculate based on current joint position, [0~7]-Solve based on specific joint space configuration
    * @return Error code
    */
    errno_t NewSpiral(DescPose* desc_pos, int tool, int user, float vel, float acc, ExaxisPos* epos, float ovl, uint8_t offset_flag, DescPose* offset_pos, SpiralParam spiral_param, int config = -1);

Spiral Motion Code Example
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestSpiral(void)
    {
      ROBOT_STATE_PKG pkg = {};
      FRRobot robot;
      robot.LoggerInit();
      robot.SetLoggerLevel(1);
      int rtn = robot.RPC("192.168.58.2");
      if (rtn != 0)
      {
        return -1;
      }
      robot.SetReConnectParam(true, 30000, 500);
      JointPos j(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
      DescPose desc_pos(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
      DescPose offset_pos1(50, 0, 0, -30, 0, 0);
      DescPose offset_pos2(50, 0, 0, -5, 0, 0);
      ExaxisPos epos(0, 0, 0, 0);
      SpiralParam sp;
      sp.circle_num = 5;
      sp.circle_angle = 5.0;
      sp.rad_init = 50.0;
      sp.rad_add = 10.0;
      sp.rotaxis_add = 10.0;
      sp.rot_direction = 0;
      int tool = 0;
      int user = 0;
      float vel = 100.0;
      float acc = 100.0;
      float ovl = 100.0;
      float blendT = 0.0;
      uint8_t flag = 2;
      robot.SetSpeed(20);
      rtn = robot.MoveJ(&j, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos1);
      printf("movej errcode:%d\n", rtn);
      rtn = robot.NewSpiral(&desc_pos, tool, user, vel, acc, &epos, ovl, flag, &offset_pos2, sp);
      printf("newspiral errcode:%d\n", rtn);
      robot.CloseRPC();
      return 0;
    }

Servo Motion Start
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Start servo motion, used with ServoJ and ServoCart commands
    * @param [in] comType Command sending type; 0-xmlrpc; 1-UDP (corresponding to robot port 20007)
    * @return Error code
    */
    errno_t ServoMoveStart(int comType = 0);

Servo Motion End
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief End servo motion, used with ServoJ and ServoCart commands
    * @param [in] comType Command sending type; 0-xmlrpc; 1-UDP (corresponding to robot port 20007)
    * @return Error code
    */
    errno_t ServoMoveEnd(int comType = 0);

Joint Space Servo Mode Motion
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint space servo mode motion
    * @param [in] joint_pos Target joint position, unit deg
    * @param [in] axisPos External axis position, unit mm
    * @param [in] acc Acceleration percentage, range [0~100], temporarily not open, default is 0
    * @param [in] vel Velocity percentage, range [0~100], temporarily not open, default is 0
    * @param [in] cmdT Command sending period, unit s, recommended range [0.001~0.0016]
    * @param [in] filterT Filter time, unit s, temporarily not open, default is 0
    * @param [in] gain Proportional amplifier for target position, temporarily not open, default is 0
    * @param [in] id ServoJ command ID, default is 0
    * @param [in] comType Command sending type; 0-xmlrpc; 1-UDP (corresponding to robot port 20007)
    * @return Error code
    */
    errno_t ServoJ(JointPos *joint_pos, ExaxisPos* axisPos, float acc, float vel, float cmdT, float filterT, float gain, int id = 0, int comType = 0);

Joint Space Servo Mode Motion Example Program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestServoJ(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos j(0, 0, 0, 0, 0, 0);
        ExaxisPos epos(0, 0, 0, 0);
        float vel = 0.0;
        float acc = 0.0;
        float cmdT = 0.008;
        float filterT = 0.0;
        float gain = 0.0;
        uint8_t flag = 0;
        int count = 500;
        float dt = 0.1;
        int cmdID = 0;
        int ret = robot.GetActualJointPosDegree(flag, &j);
        if (ret == 0)
        {
            robot.ServoMoveStart();
            while (count)
            {
                robot.ServoJ(&j, &epos, acc, vel, cmdT, filterT, gain, cmdID);
                j.jPos[0] += dt;
                count -= 1;
                robot.WaitMs(cmdT * 1000);
            }
            robot.ServoMoveEnd();
        }
        else
        {
            printf("GetActualJointPosDegree errcode:%d\n", ret);
        }
        robot.CloseRPC();
        return 0;
    }

Code Example for Robot Joint Space Servo Mode Motion Based on UDP Communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    void UDPFrameCallBack(int srcType, int count, int cmdID, int len, std::string content)
    {
        cout << "recv cmd: cmdID:  " << to_string(cmdID) << "  content is " << content << "  count is " << count << endl;;
            return;
    }

    int TestServoJUDP(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        int rtn = 0;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        rtn = robot.SetCmdRpyCallback(UDPFrameCallBack);
        printf("SetCmdRpyCallback rtn is %d\n", rtn);
        rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 50);
        JointPos j(0, -90, 90, 0, 0, 0);
        ExaxisPos epos(0, 0, 0, 0);
        DescPose offset_pos(0, 0, 0, 0, 0, 0);
        while (true)
        {
            robot.MoveJ(&j, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
            float vel = 0.0;
            float acc = 0.0;
            float cmdT = 0.016;
            float filterT = 0.0;
            float gain = 0.0;
            uint8_t flag = 0;
            float dt = 0.1;
            int cmdID = 0;
            int ret = robot.GetActualJointPosDegree(flag, &j);
            if (ret != 0)
            {
                printf("GetActualJointPosDegree errcode:%d\n", ret);
            }
            int comType = 1;
            int count = 300;
            rtn = robot.ServoMoveStart(comType);
            printf("ServoMoveStart rtn is %d\n", rtn);
            while (count)
            {
                rtn = robot.ServoJ(&j, &epos, acc, vel, cmdT, filterT, gain, cmdID, comType);
                printf("ServoJ rtn is %d\n", rtn);
                j.jPos[0] += dt;
                j.jPos[1] += dt;
                j.jPos[2] += dt;
                j.jPos[3] += dt;
                j.jPos[4] += dt;
                j.jPos[5] += dt;
                epos.ePos[0] += dt;
                count -= 1;
                robot.Sleep(15);
            }
            robot.ServoMoveEnd(comType);
            printf("ServoMoveEnd rtn is %d\n", rtn);
            count = 300;
            robot.ServoMoveStart(comType);
            printf("ServoMoveStart rtn is %d\n", rtn);
            while (count)
            {
                robot.ServoJ(&j, &epos, acc, vel, cmdT, filterT, gain, cmdID, comType);
                printf("ServoJ rtn is %d\n", rtn);
                j.jPos[0] -= dt;
                j.jPos[1] -= dt;
                j.jPos[2] -= dt;
                j.jPos[3] -= dt;
                j.jPos[4] -= dt;
                j.jPos[5] -= dt;
                epos.ePos[0] -= dt;
                count -= 1;
                robot.Sleep(15);
            }
            robot.ServoMoveEnd(comType);
            printf("ServoMoveEnd rtn is %d\n", rtn);
        }
        robot.Sleep(4000);
        robot.CloseRPC();
        return 0;
    }

Joint Torque Control Start
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Start joint torque control
    * @param [in] comType Command sending type; 0-xmlrpc; 1-UDP (corresponding to robot port 20007)
    * @return Error code
    */
    errno_t ServoJTStart(int comType = 0);

Joint Torque Control
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint torque control
    * @param [in] torque j1~j6 joint torque, unit Nm
    * @param [in] interval Command period, unit s, range [0.001~0.008]
    * @param [in] checkFlag Detection strategy 0-no restriction; 1-power limitation; 2-velocity limitation; 3-both power and velocity limitation
    * @param [in] jPowerLimit Joint maximum power limit (W)
    * @param [in] jVelLimit Joint maximum velocity (°/s)
    * @param [in] comType Command sending type; 0-xmlrpc; 1-UDP (corresponding to robot port 20007)
    * @return Error code
    */
    errno_t ServoJT(float torque[], double interval, int checkFlag, double jPowerLimit[6], double jVelLimit[6], int comType = 0);

Joint Torque Control End
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief End joint torque control
    * @param [in] comType Command sending type; 0-xmlrpc; 1-UDP (corresponding to robot port 20007)
    * @return Error code
    */
    errno_t ServoJTEnd(int comType = 0);

Joint Torque Control
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint Torque Control 1
    * @param  [in] torque Torque for joints j1~j6, unit: Nm
    * @param  [in] interval Command cycle, unit: s, range: [0.001~0.008]
    * @param  [in] checkFlag Detection strategy 0-No restrictions; 1-Power limit; 2-Velocity limit; 3-Both power and velocity limits
    * @param  [in] jPowerLimit Maximum joint power limit (W)
    * @param  [in] jVelLimit Maximum joint velocity (°/s)
    * @return  Error code
    */
    errno_t ServoJT(float torque[], double interval);

Joint Torque Control End
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint torque control end
    * @return Error code
    */
    errno_t ServoJTEnd();

Joint Torque Control Example
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int TestServoJT(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        robot.DragTeachSwitch(1);
        float torques[] = { 0, 0, 0, 0, 0, 0 };
        robot.GetJointTorques(1, torques);
        int count = 100;
        robot.ServoJTStart(); 
        int error = 0;
        while (count > 0)
        {
            error = robot.ServoJT(torques, 0.001);
            count = count - 1;
            robot.Sleep(1);
        }
        error = robot.ServoJTEnd();
        robot.DragTeachSwitch(0);
        robot.CloseRPC();
        return 0;
    }

Joint Torque Control Code Example with Overspeed Protection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int ServoJTWithSafety(FRRobot* robot)
    {
        robot->ResetAllError();
        robot->Sleep(500);
        float torques[] = { 0, 0, 0, 0, 0, 0 };
        robot->GetJointTorques(1, torques);
        robot->ServoJTStart(); 
        ROBOT_STATE_PKG pkg = {};
        robot->DragTeachSwitch(1);
        int checkFlag = 3;
        //double jPowerLimit[6] = {1, 1, 1, 1, 1, 1}; 
        double jPowerLimit[6] = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };
        double jVelLimit[6] = { 181, 80, 80, 80, 80, 80 };
        int count = 800000;
        int error = 0;
        while (count > 0)
        {
            torques[2] = torques[2] + 0.01;
            error = robot->ServoJT(torques, 0.008, checkFlag, jPowerLimit, jVelLimit); 
            if (error != 0)
            {
                robot->ServoJTEnd();
            }
            printf("ServoJT rtn is %d\n", error);
            count = count - 1;
            robot->Sleep(1);
            robot->GetRobotRealTimeState(&pkg);
            printf("maincode %d, subcode %d\n", pkg.main_code, pkg.sub_code);
        }
        robot->DragTeachSwitch(0);
        error = robot->ServoJTEnd();  
        return 0;
    }

Cartesian Space Servo Mode Motion
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Cartesian space servo mode motion
    * @param [in] mode 0-Absolute motion (base coordinate system), 1-Incremental motion (base coordinate system), 2-Incremental motion (tool coordinate system)
    * @param [in] desc_pos Target Cartesian pose or pose increment
    * @param [in] exaxis Extended axis position
    * @param [in] pos_gain Pose increment proportionality coefficient, only effective in incremental motion, range [0~1]
    * @param [in] acc Acceleration percentage, range [0~100], temporarily not available, default 0
    * @param [in] vel Velocity percentage, range [0~100], temporarily not available, default 0
    * @param [in] cmdT Command transmission period, unit s, recommended range [0.001~0.016]
    * @param [in] filterT Filter time, unit s, temporarily not available, default 0
    * @param [in] gain Proportional amplifier for target position, temporarily not available, default 0
    * @return Error code
    */
    errno_t ServoCart(int mode, DescPose *desc_pose, ExaxisPos exaxis, float pos_gain[6], float acc, float vel, float cmdT, float filterT, float gain);

Cartesian Space Servo Mode Motion Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestServoCart(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        DescPose desc_pos_dt = { 83.00800, 50.525000 , 29.246 , 179.629 , -7.138 , -166.975 };
        ExaxisPos exaxis = { 100.0, 0.0, 0.0, 0.0 };
        float pos_gain[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        int mode = 0;
        float vel = 0.0;
        float acc = 0.0;
        float cmdT = 0.001;
        float filterT = 0.0;
        float gain = 0.0;
        uint8_t flag = 0;
        int count = 5000;
        robot.SetSpeed(20);
        while (count)
        {
            rtn = robot.ServoCart(mode, &desc_pos_dt, exaxis, pos_gain, acc, vel, cmdT, filterT, gain);
            printf("ServoCart rtn is %d\n", rtn);
            count -= 1;
            desc_pos_dt.tran.x += 0.01;
            exaxis.ePos[0] += 0.01;
        }
        robot.CloseRPC();
        return 0;
    }

Spline Motion Start
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Spline motion start
    * @return Error code
    */
    errno_t SplineStart();

Joint Space Spline Motion (Auto Forward Kinematics Calculation)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Joint space spline motion (auto forward kinematics calculation)
    * @param [in] joint_pos Target joint position, unit: deg
    * @param [in] tool Tool coordinate system number, range [0~14]
    * @param [in] user User coordinate system number, range [0~14]
    * @param [in] vel Velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], currently not open
    * @param [in] ovl Velocity scaling factor, range [0~100]
    * @return Error code
    */
    errno_t SplinePTP(JointPos* joint_pos, int tool, int user, float vel, float acc, float ovl);

Spline PTP Motion
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint space spline motion
    * @param [in] joint_pos Target joint position in deg
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool Tool coordinate number [0~14]
    * @param [in] user Workpiece coordinate number [0~14]
    * @param [in] vel Speed percentage [0~100]
    * @param [in] acc Acceleration percentage [0~100] (not yet available)
    * @param [in] ovl Speed scaling factor [0~100]
    * @return Error code
    */
    errno_t SplinePTP(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl);

Spline Motion End
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Spline motion end
    * @return Error code
    */
    errno_t SplineEnd();

Spline Motion Example
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestSpline(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        JointPos j3(-61.954, -84.409, 108.153, -116.316, -91.283, 74.260);
        JointPos j4(-89.575, -80.276, 102.713, -116.302, -91.284, 74.267);
        DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose desc_pos3(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
        DescPose desc_pos4(-104.066, 544.321, 327.023, -177.715, 3.371, -73.818);
        DescPose offset_pos(0, 0, 0, 0, 0, 0);
        ExaxisPos epos(0, 0, 0, 0);
        int tool = 0;
        int user = 0;
        float vel = 100.0;
        float acc = 100.0;
        float ovl = 100.0;
        float blendT = -1.0;
        uint8_t flag = 0;
        robot.SetSpeed(20);
        int err1 = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        printf("movej errcode:%d\n", err1);
        robot.SplineStart();
        robot.SplinePTP(&j1, &desc_pos1, tool, user, vel, acc, ovl);
        robot.SplinePTP(&j2, &desc_pos2, tool, user, vel, acc, ovl);
        robot.SplinePTP(&j3, &desc_pos3, tool, user, vel, acc, ovl);
        robot.SplinePTP(&j4, &desc_pos4, tool, user, vel, acc, ovl);
        robot.SplineEnd();
        err1 = robot.MoveJ(&j1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        printf("movej errcode:%d\n", err1);
        robot.SplineStart();
        robot.SplinePTP(&j1, tool, user, vel, acc, ovl);
        robot.SplinePTP(&j2, tool, user, vel, acc, ovl);
        robot.SplinePTP(&j3, tool, user, vel, acc, ovl);
        robot.SplinePTP(&j4, tool, user, vel, acc, ovl);
        robot.SplineEnd();
        robot.CloseRPC();
        return 0;
    }

New Spline Motion Start
++++++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief New spline motion start
    * @param [in] type 0-Circular transition, 1-Given points as path points
    * @param [in] averageTime Global average transition time in ms (10 ~ ), default 2000
    * @return Error code
    */
    errno_t NewSplineStart(int type, int averageTime=2000);

New Spline Command Point
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief New spline command point
    * @param [in] joint_pos Target joint position in deg
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool Tool coordinate number [0~14]
    * @param [in] user Workpiece coordinate number [0~14]
    * @param [in] vel Speed percentage [0~100]
    * @param [in] acc Acceleration percentage [0~100] (not yet available)
    * @param [in] ovl Speed scaling factor [0~100]
    * @param [in] blendR [-1.0]-Move to position (blocking), [0~1000.0]-Smoothing radius (non-blocking) in mm
    * @param [in] lastFlag Whether it's the last point, 0-No, 1-Yes
    * @return Error code
    */
    errno_t NewSplinePoint(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int lastFlag);

New Spline Command Point (Auto Inverse Kinematics Calculation)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief New spline command point (auto inverse kinematics calculation)
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool Tool coordinate system number, range [0~14]
    * @param [in] user User coordinate system number, range [0~14]
    * @param [in] vel Velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], currently not open
    * @param [in] ovl Velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-Move to position (blocking), [0~1000.0]-Blending radius (non-blocking), unit: mm
    * @param [in] lastFlag Whether it is the last point, 0-No, 1-Yes
    * @param [in] config Inverse kinematics joint space configuration, [-1]-Calculate based on current joint position, [0~7]-Solve according to specific joint space configuration
    * @return Error code
    */
    errno_t NewSplinePoint(DescPose* desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int lastFlag, int config = -1);

New Spline Motion End
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief New spline motion end
    * @return Error code
    */
    errno_t NewSplineEnd();

New Spline Motion Example
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestNewSpline(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        JointPos j3(-61.954, -84.409, 108.153, -116.316, -91.283, 74.260);
        JointPos j4(-89.575, -80.276, 102.713, -116.302, -91.284, 74.267);
        JointPos j5(-95.228, -54.621, 73.691, -112.245, -91.280, 74.268);
        DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose desc_pos3(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
        DescPose desc_pos4(-104.066, 544.321, 327.023, -177.715, 3.371, -73.818);
        DescPose desc_pos5(-33.421, 732.572, 275.103, -177.907, 2.709, -79.482);
        DescPose offset_pos(0, 0, 0, 0, 0, 0);
        ExaxisPos epos(0, 0, 0, 0);
        int tool = 0;
        int user = 0;
        float vel = 100.0;
        float acc = 100.0;
        float ovl = 100.0;
        float blendT = -1.0;
        uint8_t flag = 0;
        robot.SetSpeed(20);
        int err1 = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        printf("movej errcode:%d\n", err1);
        robot.NewSplineStart(1, 2000);
        robot.NewSplinePoint(&j1, &desc_pos1, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(&j2, &desc_pos2, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(&j3, &desc_pos3, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(&j4, &desc_pos4, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(&j5, &desc_pos5, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplineEnd();
        err1 = robot.MoveJ(&j1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        printf("movej errcode:%d\n", err1);
        robot.NewSplineStart(1, 2000);
        robot.NewSplinePoint(&desc_pos1, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(&desc_pos2, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(&desc_pos3, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(&desc_pos4, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(&desc_pos5, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplineEnd();
        robot.CloseRPC();
        return 0;
    }

Stop Motion
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Stop motion
    * @return Error code
    */
    errno_t StopMotion();

Pause Motion
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Pause motion
    * @return Error code
    */
    errno_t PauseMotion();

Resume Motion
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:
    
    /**
    * @brief Resume motion
    * @return Error code
    */
    errno_t ResumeMotion();

Motion Pause, Resume, Stop Example
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestPause(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j5(-95.228, -54.621, 73.691, -112.245, -91.280, 74.268);
        DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos5(-33.421, 732.572, 275.103, -177.907, 2.709, -79.482);
        DescPose offset_pos(0, 0, 0, 0, 0, 0);
        ExaxisPos epos(0, 0, 0, 0);
        int tool = 0;
        int user = 0;
        float vel = 100.0;
        float acc = 100.0;
        float ovl = 100.0;
        float blendT = -1.0;
        uint8_t flag = 0;
        robot.SetSpeed(20);
        rtn = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        rtn = robot.MoveJ(&j5, &desc_pos5, tool, user, vel, acc, ovl, &epos, 1, flag, &offset_pos);
        robot.Sleep(1000);
        robot.PauseMotion();
        robot.Sleep(1000);
        robot.ResumeMotion();
        robot.Sleep(1000);
        robot.StopMotion();
        robot.Sleep(1000);
        robot.CloseRPC();
        return 0;
    }

Point Global Offset Start
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Point global offset start
    * @param [in] flag 0-Offset in base/workpiece coordinate, 2-Offset in tool coordinate
    * @param [in] offset_pos Pose offset
    * @return Error code
    */
    errno_t PointsOffsetEnable(int flag, DescPose *offset_pos);

Point Global Offset End
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Point global offset end
    * @return Error code
    */
    errno_t PointsOffsetDisable();

Point Offset Example
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestOffset(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose offset_pos(0, 0, 0, 0, 0, 0);
        DescPose offset_pos1(0, 0, 50, 0, 0, 0);
        ExaxisPos epos(0, 0, 0, 0);
        int tool = 0;
        int user = 0;
        float vel = 100.0;
        float acc = 100.0;
        float ovl = 100.0;
        float blendT = -1.0;
        uint8_t flag = 0;
        robot.SetSpeed(20);
        robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        robot.Sleep(1000);
        robot.PointsOffsetEnable(0, &offset_pos1);
        robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        robot.PointsOffsetDisable();
        robot.CloseRPC();
        return 0;
    }

Control Box AO Flying Start
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Control box AO flying start
    * @param [in] AONum Control box AO number
    * @param [in] maxTCPSpeed Maximum TCP speed value [1-5000mm/s], default 1000
    * @param [in] maxAOPercent AO percentage corresponding to maximum TCP speed, default 100%
    * @param [in] zeroZoneCmp Dead zone compensation value AO percentage, integer, default 20%, range [0-100]
    * @return Error code
    */
    errno_t MoveAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);

Control Box AO Flying Stop
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0
   
.. code-block:: c++
    :linenos:

    /**
    * @brief Control box AO flying stop
    * @return Error code
    */
    errno_t MoveAOStop();
    
End AO Flying Start
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0
   
.. code-block:: c++
    :linenos:

    /**
    * @brief End AO flying start
    * @param [in] AONum End AO number
    * @param [in] maxTCPSpeed Maximum TCP speed value [1-5000mm/s], default 1000
    * @param [in] maxAOPercent AO percentage corresponding to maximum TCP speed, default 100%
    * @param [in] zeroZoneCmp Dead zone compensation value AO percentage, integer, default 20%, range [0-100]
    * @return Error code
    */
    errno_t MoveToolAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);
    
End AO Flying Stop
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0
   
.. code-block:: c++
    :linenos:

    /**
    * @brief End AO flying stop
    * @return Error code
    */
    errno_t MoveToolAOStop();

AO Flying Example
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestMoveAO(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose offset_pos(0, 0, 0, 0, 0, 0);
        DescPose offset_pos1(0, 0, 50, 0, 0, 0);
        ExaxisPos epos(0, 0, 0, 0);
        int tool = 0;
        int user = 0;
        float vel = 20.0;
        float acc = 20.0;
        float ovl = 100.0;
        float blendT = -1.0;
        uint8_t flag = 0;
        robot.SetSpeed(20);
        robot.MoveAOStart(0, 100, 100, 20);
        robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        robot.MoveAOStop();
        robot.Sleep(1000);
        robot.MoveToolAOStart(0, 100, 100, 20);
        robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
        robot.MoveToolAOStop();
        robot.CloseRPC();
        return 0;
    }

Start Ptp Motion FIR Filter
+++++++++++++++++++++++++++++
.. versionadded:: V3.7.7
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Start Ptp motion FIR filter
    * @param [in] maxAcc Maximum acceleration limit (deg/s2)
    * @param [in] maxJek Unified joint jerk limit (deg/s3), default 1000
    * @return Error code
    */
    errno_t PtpFIRPlanningStart(double maxAcc, double maxJek = 1000);

Close Ptp Motion FIR Filter
+++++++++++++++++++++++++++++
.. versionadded:: V3.7.7

.. code-block:: c++
    :linenos:

    /**
    * @brief Close Ptp motion FIR filter
    * @return Error code
    */
    errno_t PtpFIRPlanningEnd();

Start LIN, ARC Motion FIR Filter
+++++++++++++++++++++++++++++++++++++++++
.. versionadded:: V3.7.7

.. code-block:: c++
    :linenos:

    /**
    * @brief Start LIN, ARC motion FIR filter
    * @param [in] maxAccLin Linear acceleration limit (mm/s2)
    * @param [in] maxAccDeg Angular acceleration limit (deg/s2)
    * @param [in] maxJerkLin Linear jerk limit (mm/s3)
    * @param [in] maxJerkDeg Angular jerk limit (deg/s3)
    * @return Error code
    */
    errno_t LinArcFIRPlanningStart(double maxAccLin, double maxAccDeg, double maxJerkLin, double maxJerkDeg);

Close LIN, ARC Motion FIR Filter
+++++++++++++++++++++++++++++++++++
.. versionadded:: V3.7.7

.. code-block:: c++
    :linenos:

    /**
    * @brief Close LIN, ARC motion FIR filter
    * @return Error code
    */
    errno_t LinArcFIRPlanningEnd();

FIR Filter Example
+++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestFIR(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos startjointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos midjointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        JointPos endjointPos(-29.777, -84.536, 109.275, -114.075, -86.655, 74.257);
        DescPose startdescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose middescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose enddescPose(-487.434, 154.362, 308.576, 176.600, 0.268, -14.061);
        ExaxisPos exaxisPos(0, 0, 0, 0);
        DescPose offdese(0, 0, 0, 0, 0, 0);
        rtn = robot.PtpFIRPlanningStart(1000, 1000);
        cout << "PtpFIRPlanningStart rtn is " << rtn << endl;
        robot.MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.PtpFIRPlanningEnd();
        cout << "PtpFIRPlanningEnd rtn is " << rtn << endl;
        robot.LinArcFIRPlanningStart(1000, 1000, 1000, 1000);
        cout << "LinArcFIRPlanningStart rtn is " << rtn << endl;
        robot.MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
        robot.MoveC(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &endjointPos, &enddescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, -1);
        robot.LinArcFIRPlanningEnd();
        cout << "LinArcFIRPlanningEnd rtn is " << rtn << endl;
        robot.CloseRPC();
        return 0;
    }

Acceleration Smoothing Start
+++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.1-3.8.1

.. code-block:: c++
    :linenos:

    /**
    * @brief Acceleration smoothing start
    * @param [in] saveFlag Whether to save after power off
    * @return Error code
    */
    errno_t AccSmoothStart(bool saveFlag);

Acceleration Smoothing End
+++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.1-3.8.1

.. code-block:: c++
    :linenos:

    /**
    * @brief Acceleration smoothing end
    * @param [in] saveFlag Whether to save after power off
    * @return Error code
    */
    errno_t AccSmoothEnd(bool saveFlag);

Acceleration Smoothing Example
+++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestAccSmooth(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos startjointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos endjointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        DescPose startdescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose enddescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        ExaxisPos exaxisPos(0, 0, 0, 0);
        DescPose offdese(0, 0, 0, 0, 0, 0);
        rtn = robot.AccSmoothStart(0);
        cout << "AccSmoothStart rtn is " << rtn << endl;
        robot.MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        rtn = robot.AccSmoothEnd(0);
        cout << "AccSmoothEnd rtn is " << rtn << endl;
        robot.CloseRPC();
        return 0;
    }

Specified Pose Speed Start
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Specified pose speed start
    * @param [in] ratio Pose speed percentage [0-300]
    * @return Error code
    */
    errno_t AngularSpeedStart(int ratio);

Specified Pose Speed End
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Specified pose speed end
    * @return Error code
    */
    errno_t AngularSpeedEnd();

Robot Specified Pose Speed Example
++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestAngularSpeed(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos startjointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos endjointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        DescPose startdescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose enddescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        ExaxisPos exaxisPos(0, 0, 0, 0);
        DescPose offdese(0, 0, 0, 0, 0, 0);
        rtn = robot.AngularSpeedStart(50);
        cout << "AngularSpeedStart rtn is " << rtn << endl;
        robot.MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        rtn = robot.AngularSpeedEnd();
        cout << "AngularSpeedEnd rtn is " << rtn << endl;
        robot.CloseRPC();
        return 0;
    }

Start Singular Pose Protection
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Start singular pose protection
    * @param [in] protectMode Singular protection mode, 0: Joint mode; 1-Cartesian mode
    * @param [in] minShoulderPos Shoulder singular adjustment range (mm), default 100
    * @param [in] minElbowPos Elbow singular adjustment range (mm), default 50
    * @param [in] minWristPos Wrist singular adjustment range (°), default 10
    * @return Error code
    */
    errno_t SingularAvoidStart(int protectMode, double minShoulderPos, double minElbowPos, double minWristPos);

Stop Singular Pose Protection
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Stop singular pose protection
    * @return Error code
    */
    errno_t SingularAvoidEnd();

Robot Singular Pose Protection Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestAngularSpeed(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos startjointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos endjointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        DescPose startdescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose enddescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        ExaxisPos exaxisPos(0, 0, 0, 0);
        DescPose offdese(0, 0, 0, 0, 0, 0);
        rtn = robot.SingularAvoidStart(2, 10, 5, 5);
        cout << "SingularAvoidStart rtn is " << rtn << endl;
        robot.MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        rtn = robot.SingularAvoidEnd();
        cout << "SingularAvoidEnd rtn is " << rtn << endl;
        robot.CloseRPC();
        return 0;
    }

Clear the motion command queue
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Clear the motion command queue
    * @return error code
    */
    errno_t MotionQueueClear();

Move to Intersecting Line Start Point
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Move to intersecting line start point
    * @param [in] mainPoint Cartesian poses of 6 taught points on the main pipe
    * @param [in] mainExaxisPos Extended axis positions for 6 taught points on the main pipe
    * @param [in] piecePoint Cartesian poses of 6 taught points on the branch pipe
    * @param [in] pieceExaxisPos Extended axis positions for 6 taught points on the branch pipe
    * @param [in] extAxisFlag Whether to enable extended axis; 0-Disable; 1-Enable
    * @param [in] exaxisPos Start point extended axis position
    * @param [in] tool Tool coordinate system number
    * @param [in] wobj Workpiece coordinate system number
    * @param [in] vel Velocity percentage
    * @param [in] acc Acceleration percentage
    * @param [in] ovl Velocity scaling factor
    * @param [in] oacc Acceleration scaling factor
    * @param [in] moveType Motion type; 0-PTP; 1-LIN
    * @param [in] moveDirection Motion direction; 0-Clockwise; 1-Counterclockwise
    * @param [in] offset Offset value
    * @return Error code
    */
    errno_t MoveToIntersectLineStart(DescPose mainPoint[6], ExaxisPos mainExaxisPos[6], DescPose piecePoint[6], ExaxisPos pieceExaxisPos[6], int extAxisFlag, ExaxisPos exaxisPos, int tool, int wobj, double vel, double acc, double ovl, double oacc, int moveType, int moveDirection, DescPose offset);
            
Intersecting Line Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Intersecting line motion
    * @param [in] mainPoint Cartesian poses of 6 taught points on the main pipe
    * @param [in] mainExaxisPos Extended axis positions for 6 taught points on the main pipe
    * @param [in] piecePoint Cartesian poses of 6 taught points on the branch pipe
    * @param [in] pieceExaxisPos Extended axis positions for 6 taught points on the branch pipe
    * @param [in] extAxisFlag Whether to enable extended axis; 0-Disable; 1-Enable
    * @param [in] exaxisPos Start point extended axis positions
    * @param [in] tool Tool coordinate system number
    * @param [in] wobj Workpiece coordinate system number
    * @param [in] vel Velocity percentage
    * @param [in] acc Acceleration percentage
    * @param [in] ovl Velocity scaling factor
    * @param [in] oacc Acceleration scaling factor
    * @param [in] moveDirection Motion direction; 0-Clockwise; 1-Counterclockwise
    * @param [in] offset Offset value
    * @return Error code
    */
    errno_t MoveIntersectLine(DescPose mainPoint[6], ExaxisPos mainExaxisPos[6], DescPose piecePoint[6], ExaxisPos pieceExaxisPos[6], int extAxisFlag, ExaxisPos exaxisPos[4], int tool, int wobj, double vel, double acc, double ovl, double oacc, int moveDirection, DescPose offset);
                
Robot Intersecting Line Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    void TestIntersectLineMove()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(3);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return ;
        }
        robot.SetReConnectParam(true, 30000, 500);
        DescPose mainPoint[6] = {};
        DescPose piecePoint[6] = {};
        ExaxisPos mainExaxisPos[6] = {};
        ExaxisPos pieceExaxisPos[6] = {};
        int extAxisFlag = 1;
        ExaxisPos exaxisPos[4] = {};
        DescPose offset = { 0.0, 2.0 ,30.0, -2.0, 0.0, 0.0 };
        mainPoint[0] = {490.004, -383.194, 402.735, -9.332, -1.528, 69.594};
        mainPoint[1] = {444.950, -407.117, 389.011, -5.546, -2.196, 65.279};
        mainPoint[2] = {445.168, -463.605, 355.759, -1.544, -10.886, 57.104};
        mainPoint[3] = {507.529, -485.385, 343.013, -0.786, -4.834, 61.799};
        mainPoint[4] = {554.390, -442.647, 367.701, -4.761, -10.181, 64.925};
        mainPoint[5] = {532.552, -394.003, 396.467, -13.732, -13.592, 67.411};
        mainExaxisPos[0] = { -29.996, 0.000, 0.000, 0.000 };
        mainExaxisPos[1] = { -29.996, 0.000, 0.000, 0.000 };
        mainExaxisPos[2] = { -29.996, 0.000, 0.000, 0.000 };
        mainExaxisPos[3] = { -29.996, 0.000, 0.000, 0.000 };
        mainExaxisPos[4] = { -29.996, 0.000, 0.000, 0.000 };
        mainExaxisPos[5] = { -29.996, 0.000, 0.000, 0.000 };
        piecePoint[0] = { 505.571, -192.408, 316.759, 38.098, 37.051, 139.447 };
        piecePoint[1] = {533.837, -201.558, 332.340, 34.644, 42.339, 137.748};
        piecePoint[2] = {530.386, -225.085, 373.808, 35.431, 45.111, 137.560};
        piecePoint[3] = {485.646, -229.195, 383.778, 33.870, 45.173, 137.064};
        piecePoint[4] = {460.551, -212.161, 354.256, 28.856, 45.602, 135.930};
        piecePoint[5] = {474.217, -197.124, 324.611, 42.469, 41.133, 148.167};
        pieceExaxisPos[0] = { -29.996, -0.000, 0.000, 0.000 };
        pieceExaxisPos[1] = { -29.996, -0.000, 0.000, 0.000 };
        pieceExaxisPos[2] = { -29.996, -0.000, 0.000, 0.000 };
        pieceExaxisPos[3] = { -29.996, -0.000, 0.000, 0.000 };
        pieceExaxisPos[4] = { -29.996, -0.000, 0.000, 0.000 };
        pieceExaxisPos[5] = { -29.996, -0.000, 0.000, 0.000 };
        exaxisPos[0] = {-29.996, -0.000, 0.000, 0.000};
        exaxisPos[1] = {-44.994, 90.000, 0.000, 0.000};
        exaxisPos[2] = {-59.992, 0.002, 0.000, 0.000};
        exaxisPos[3] = {-44.994, -89.997, 0.000, 0.000};
        int tool = 2;
        int wobj = 0;
        double vel = 100.0;
        double acc = 100.0;
        double ovl = 12.0;
        double oacc = 12.0; 
        int moveType = 1;
        int moveDirection = 1;
        rtn = robot.MoveToIntersectLineStart(mainPoint, mainExaxisPos, piecePoint, pieceExaxisPos, extAxisFlag, exaxisPos[0], tool, wobj, vel, acc, ovl, oacc, moveType, moveDirection, offset);
        printf("MoveToIntersectLineStart rtn is %d\n", rtn);
        rtn = robot.MoveIntersectLine(mainPoint, mainExaxisPos, piecePoint, pieceExaxisPos, extAxisFlag, exaxisPos, tool, wobj, vel, acc, 5.0, 5.0, moveDirection, offset);
        printf("MoveIntersectLine rtn is %d\n", rtn);
        robot.CloseRPC();
        return ;
    }

Stationary Air Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Stationary Air Motion
    * @return Error code
    */
    errno_t MoveStationary();

Stationary Air Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestLaserStationary(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return 0;
        }
        robot.SetReConnectParam(true, 30000, 500);
        rtn = robot.LaserSensorRecordandReplay(0, 10, 1, 0, 0.1, 1, 0, 10, 100);
        printf("LaserSensorRecordandReplay rtn is %d\n", rtn);
        rtn = robot.MoveStationary();
        printf("MoveStationary rtn is %d\n", rtn);
        rtn = robot.LaserSensorRecord1(0, 10);
        printf("LaserSensorRecordandReplay rtn is %d\n", rtn);
        robot.CloseRPC();
        robot.Sleep(9999999);
        return 0;
    }

Wait for In-Place Idle Motion to Complete
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Wait for in-place idle motion to complete
    * @return Error code
    */
    errno_t WaitStationaryMotionDone();

Fixed-Point Swing Start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Start fixed-point swing
    * @param [in] weaveNum Swing number [0-7]
    * @param [in] mode 0-Tool coordinate system; 1-Reference point
    * @param [in] refPoint Reference point Cartesian coordinates [x,y,z,a,b,c]
    * @param [in] weaveTime Swing time [s]
    * @return Error code
    */
    errno_t OriginPointWeaveStart(int weaveNum, int mode, DescPose refPoint, double weaveTime);
    
Fixed-Point Swing End
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief End fixed-point swing
    * @return Error code
    */
    errno_t OriginPointWeaveEnd();
        
Fixed-Point Swing SDK Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestOriginPointWeave()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos j(39.886, -98.580, -124.032, -47.393, 90.000, 40.842);
        ExaxisPos epos(0, 0, 0, 0);
        DescPose offset_pos(0, 0, 0, 0, 0, 0);
        DescPose refPoint = { 400.021,300.022,299.996,179.997,-0.003,-90.956 };
        robot.MoveJ(&j, 1, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.OriginPointWeaveStart(0, 0, refPoint, 3);
        robot.MoveStationary();
        robot.OriginPointWeaveEnd();
        robot.Sleep(2000);
        robot.MoveJ(&j, 1, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.OriginPointWeaveStart(0, 1, refPoint, 3);
        robot.MoveStationary();
        robot.OriginPointWeaveEnd();
        robot.CloseRPC();
        robot.Sleep(1000);
        return 0;
    }

Fixed-Point Swing (Including Laser Sensor and Extension Axis) Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestOriginPointWeave()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos j(39.886, -98.580, -124.032, -47.393, 90.000, 40.842);
        ExaxisPos epos1(0, 0, 0, 0);
        DescPose offset_pos(0, 0, 0, 0, 0, 0);
        ExaxisPos epos2(5, 0.000, 0.000, 0.000);
        DescPose refPoint(400.021, 300.022, 299.996, 179.997, -0.003, -90.956);
        robot.LaserTrackingSensorConfig("192.168.58.20", 5020);
        robot.LaserTrackingSensorSamplePeriod(20);
        robot.LoadPosSensorDriver(101);
        robot.ExtDevLoadUDPDriver();
        rtn = robot.SetExAxisCmdDoneTime(5000.0);
        printf("SetExAxisCmdDoneTime rtn is %d\n" , rtn);
        rtn = robot.ExtAxisServoOn(1, 1);
        printf("ExtAxisServoOn axis id 1 rtn is %d\n" , rtn);
        rtn = robot.ExtAxisServoOn(2, 1);
        printf("ExtAxisServoOn axis id 2 rtn is %d\n" , rtn);
        robot.Sleep(2000);
        robot.ExtAxisSetHoming(1, 0, 10, 2);
        rtn = robot.LaserTrackingLaserOnOff(1, 0);
        printf("LaserTrackingLaserOnOff id 2 rtn is %d\n", rtn);
        robot.LaserTrackingTrackOnOff(1, 4);
        robot.Sleep(200);
        robot.OriginPointWeaveStart(0, 0, refPoint, 10);
        robot.MoveStationary();  
        robot.OriginPointWeaveEnd();
        robot.LaserTrackingTrackOnOff(0, 4);

        robot.Sleep(2000);      
    
        robot.ExtAxisMove(epos1, 100, -1);
        robot.LaserTrackingTrackOnOff(1, 4);
        robot.OriginPointWeaveStart(0, 0, refPoint, 20);
        robot.ExtAxisMove(epos2, 100, -1);
        robot.OriginPointWeaveEnd();
        robot.LaserTrackingTrackOnOff(0, 4);
        robot.CloseRPC();
        robot.Sleep(1000);
        return 0;
    }

Joint Space Velocity Servo Mode Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint space velocity servo mode motion
    * @param [in] joint_pos 6 target joint velocities, unit deg/s
    * @param [in] axisPos 4 external axis velocities, unit deg/s
    * @param [in] acc Acceleration percentage, range [0~100], temporarily not open, default is 0
    * @param [in] vel Velocity percentage, range [0~100], temporarily not open, default is 0
    * @param [in] cmdT Command sending period, unit s, recommended range [0.001~0.0016]
    * @param [in] filterT Filter time, unit s, temporarily not open, default is 0
    * @param [in] gain Proportional amplifier for target position, temporarily not open, default is 0
    * @param [in] id ServoJ command ID, default is 0
    * @param[in] comType Command sending type; 0-xmlrpc; 1-UDP (corresponding to robot port 20007)
    * @return Error code
    */
    errno_t ServoJV(double jointVel[6], double exisVel[4], float acc, float vel, float cmdT, float filterT, float gain, int id = 0, int comType = 0);

Joint Space Velocity Servo Mode Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int ServoJVtest()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        robot.SetReConnectParam(true, 300000, 500);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        double joint_vel[6] = { 10.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        double exis_vel[4] { 0.0, 0.0, 0.0, 0.0 };
        float acc = 0.0f;
        float vel = 0.0f;
        float cmdT = 0.008f;
        float filterT = 0.0f;
        float gain = 0.0f;
        int cnt = 0;
        while (cnt < 200)
        {
            int error = robot.ServoJV(joint_vel, exis_vel, acc, vel, cmdT, filterT, gain);
            printf("ServoJV rtn is %d\n", error);
            cnt++;
        }
        robot.CloseRPC();
        robot.Sleep(1000000);
        return 0;
    }

Joint MIT Control Start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint MIT control start
    * @param [in] comType Command sending type; 0-xmlrpc; 1-UDP (corresponding to robot port 20007)
    * @return Error code
    */
    errno_t ServoMITStart(int comType = 0);

Joint MIT Control End
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint MIT control end
    * @param [in] comType Command sending type; 0-xmlrpc; 1-UDP (corresponding to robot port 20007)
    * @return Error code
    */
    errno_t ServoMITEnd(int comType = 0);

Joint MIT Control
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint MIT control
    * @param [in] posGain j1~j6 joint position gain
    * @param [in] desPos j1~j6 joint desired position, unit: deg
    * @param [in] velGain j1~j6 joint velocity gain
    * @param [in] desVel j1~j6 joint desired velocity, unit: deg/s
    * @param [in] torque_ff j1~j6 feedforward torque, unit: Nm
    * @param [in] interval Command period, unit s, range [0.001~0.008]
    * @param [in] comType Command sending type; 0-xmlrpc; 1-UDP (corresponding to robot port 20007)
    * @return Error code
    */
    errno_t ServoMIT(double posGain[6], double desPos[6], double velGain[6], double desVel[6], double torque_ff[6], double interval, int comType = 0);

Robot Joint MIT Control Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int ServoMITtest()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        robot.SetReConnectParam(true, 30000, 500);
        int rtn = robot.SetCmdRpyCallback(UDPFrameCallBack);
        printf("SetCmdRpyCallback rtn is %d\n", rtn);
        rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        while (true)
        {
            robot.ResetAllError();
            robot.Sleep(500);
            double posGain[6] = { 0.0 };
            double desPos[6] = { 0.0 };
            double velGain[6] = { 0.0 };
            double desVel[6] = { 0.0 };
            double torques[6] = { 0.0 };
            float curTorque[6] = { 0.0 };
            robot.GetJointTorques(1, curTorque);
            for (int i = 0; i < 6; i++)
            {
                torques[i] = curTorque[i];
            }
            robot.ServoMITStart(0);
            ROBOT_STATE_PKG pkg = {};
            robot.DragTeachSwitch(1);
            double intev = 0.008;
            double jPowerLimit[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
            double jVelLimit[6] = { 50, 50, 50, 50, 50, 50 };
            int error = 0;
            while (true)
            {
                torques[5] = 0.02;
                error = robot.ServoMIT(posGain, desPos, velGain, desVel, torques, intev, 0);
                robot.Sleep(1);
                robot.GetRobotRealTimeState(&pkg);
                printf("pkg.jt_cur_pos[5]: %f\n", pkg.jt_cur_pos[5]);
                if (pkg.jt_cur_pos[5] > 30)
                {
                    break;
                }
            }
            while (true)
            {
                torques[5] = -0.02;
                error = robot.ServoMIT(posGain, desPos, velGain, desVel, torques, intev, 0);
                robot.Sleep(1);
                robot.GetRobotRealTimeState(&pkg);
                printf("pkg.jt_cur_pos[5]:%f\n", pkg.jt_cur_pos[5]);
                if (pkg.jt_cur_pos[5] < 0)
                {
                    break;
                }
            }
            robot.DragTeachSwitch(0);
            error = robot.ServoMITEnd(0);
        }
        robot.CloseRPC();
        robot.Sleep(1000000);
        return 0;
    }

Workpiece Coordinate System Point Transformation Start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: V3.9.8
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Start workpiece coordinate system point transformation
    * @param [in] workpieceID Workpiece number [0-14]
    * @return Error code, 0 on success
    */
    errno_t WorkPieceTrsfStart(int workpieceID);
    
Workpiece Coordinate System Point Transformation End
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: V3.9.8

.. code-block:: c++
    :linenos:

    /**
    * @brief End workpiece coordinate system point transformation
    * @return Error code, 0 on success
    */
    errno_t WorkPieceTrsfEnd();
    
Workpiece Coordinate System Transformation Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: V3.9.8

.. code-block:: c++
    :linenos:

    int TestWorkPieceTrsf()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        robot.SetReConnectParam(true, 30000, 500);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return 0;
        }
        JointPos j1(-11.188, -64.165, -107.299, -76.706, 89.590, 92.983);
        DescPose desc1(225.986, 190.694, 394.238, -6.230, -23.797, -98.972);
        JointPos j2(-38.148, -97.408, -133.704, -30.999, 89.584, 92.986);
        DescPose desc2(52.741, 262.917, 30.824, -5.696, -9.864, -126.092);
        JointPos j3(-25.561, -123.131, -85.736, -94.911, 89.582, 93.006);
        DescPose desc3(70.455, 88.410, 45.299, -4.101, 31.775, -113.199);
        JointPos j4(-8.013, -125.881, -79.196, -84.440, 89.564, 93.005);
        DescPose desc4(209.453, -73.895, 56.416, -4.727, 17.523, -95.906);
        JointPos j5(-2.722, -94.518, -119.965, -54.518, 89.563, 93.005);
        DescPose desc5(274.800, 81.106, 102.977, -5.467, -2.980, -90.711);
        JointPos j6(-2.671, -56.234, -138.914, -25.099, 95.355, 92.967);
        DescPose desc6(300.392, 177.281, 300.926, -1.909, -51.894, -89.703);
        JointPos j7(-1.229, -121.184, -63.201, -122.331, 93.045, 93.019);
        DescPose desc7(296.856, -31.294, 215.698, -0.589, 34.594, -88.954);
        ExaxisPos exaxis = {0.0, 0.0, 0.0, 0.0};
        DescPose offset(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        int tool = 1;
        int workpiece = 1;
        double blend = 5.0;
        robot.MoveJ(&j1, &desc1, tool, workpiece, 100, 100, 100, &exaxis, -1, 0, &offset);
        robot.MoveJ(&j2, &desc2, tool, workpiece, 100, 100, 100, &exaxis, blend, 0, &offset);
        robot.MoveL(&j3, &desc3, tool, workpiece, 10, 100, 100, blend, 0, &exaxis, 0, 1, &offset);
        robot.MoveC(&j4, &desc4, tool, workpiece, 100, 100, &exaxis, 0, &offset, &j5, &desc5, tool, workpiece, 100, 100, &exaxis, 0, &offset, 10, blend);
        robot.Circle(&j6, &desc6, tool, workpiece, 100, 100, &exaxis, &j7, &desc7, tool, workpiece, 100, 100, &exaxis, 10, 0, &offset, 100.0, blend);
        rtn = robot.WorkPieceTrsfStart(2);
        printf("WorkPieceTrsfStart rtn is %d\n", rtn);
        robot.MoveJ(&j1, &desc1, tool, workpiece, 100, 100, 100, &exaxis, -1, 0, &offset);
        robot.MoveJ(&j2, &desc2, tool, workpiece, 100, 100, 100, &exaxis, blend, 0, &offset);
        robot.MoveL(&j3, &desc3, tool, workpiece, 10, 100, 100, blend, 0, &exaxis, 0, 1, &offset);
        robot.MoveC(&j4, &desc4, tool, workpiece, 100, 100, &exaxis, 0, &offset, &j5, &desc5, tool, workpiece, 100, 100, &exaxis, 0, &offset, 10, blend);
        robot.Circle(&j6, &desc6, tool, workpiece, 100, 100, &exaxis, &j7, &desc7, tool, workpiece, 100, 100, &exaxis, 10, 0, &offset, 100.0, blend);    
        rtn = robot.WorkPieceTrsfEnd();
        printf("WorkPieceTrsfEnd rtn is %d\n", rtn);
        robot.CloseRPC();
        robot.Sleep(2000);
    }    