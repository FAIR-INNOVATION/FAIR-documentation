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
    * @brief Cartesian space linear motion
    * @param [in] joint_pos Target joint position, unit: deg
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool Tool coordinate system number, range [0~14]
    * @param [in] user User coordinate system number, range [0~14]
    * @param [in] vel Velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], currently not open
    * @param [in] ovl Velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-Move to position (blocking), [0~1000.0]-Blending radius (non-blocking), unit: mm
    * @param [in] blendMode Transition mode; 0-Internal tangent transition; 1-Corner transition
    * @param [in] epos Extended axis position, unit: mm
    * @param [in] search 0-No wire search, 1-Wire search
    * @param [in] offset_flag 0-No offset, 1-Offset in base/work coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos Pose offset
    * @param [in] velAccParamMode Velocity/acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) acceleration (mm/s²)
    * @param [in] overSpeedStrategy Overspeed handling strategy; 1-Standard; 2-Error stop when overspeed; 3-Adaptive speed reduction, default is 0
    * @param [in] speedPercent Allowable speed reduction threshold percentage [0-100], default 10%
    * @return Error code
    */
    errno_t MoveL(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int blendMode, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos, int velAccParamMode = 0, int overSpeedStrategy = 0, int speedPercent = 10);

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
    * @param  [in] ovl  Velocity scaling factor, range [0~100]    
    * @param  [in] blendR [-1.0]-Move to position (blocking), [0~1000.0]-Smoothing radius (non-blocking), unit: mm    
    * @param [in] velAccParamMode Velocity/acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) acceleration (mm/s²)
    * @return Error code
    */
    errno_t MoveC(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, uint8_t poffset_flag, DescPose *offset_pos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, uint8_t toffset_flag, DescPose *offset_pos_t, float ovl, float blendR, int velAccParamMode = 0);

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
     *@param  [in] ovl  Velocity scaling factor, range [0~100]
     *@param  [in] offset_flag  0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
     *@param  [in] offset_pos  Pose offset value
     *@param  [in] oacc Acceleration percentage
     *@param  [in] blendR -1: Blocking; 0~1000: Smoothing radius
    * @param [in] velAccParamMode Velocity/acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) acceleration (mm/s²)
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
      float blendT = 0.0;
      float blendR = 0.0;
      uint8_t flag = 0;
      uint8_t search = 0;
      robot.SetSpeed(20);
      rtn = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
      printf("movej errcode:%d\n", rtn);
      rtn = robot.MoveL(&j2, &desc_pos2, tool, user, vel, acc, ovl, blendR, &epos, search, flag, &offset_pos);
      printf("movel errcode:%d\n", rtn);
      rtn = robot.MoveC(&j3, &desc_pos3, tool, user, vel, acc, &epos, flag, &offset_pos, &j4, &desc_pos4, tool, user, vel, acc, &epos, flag, &offset_pos, ovl, blendR);
      printf("movec errcode:%d\n", rtn);
      rtn = robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
      printf("movej errcode:%d\n", rtn);
      rtn = robot.Circle(&j3, &desc_pos3, tool, user, vel, acc, &epos, &j1, &desc_pos1, tool, user, vel, acc, &epos, ovl, flag, &offset_pos, 100, -1);
      printf("circle errcode:%d\n", rtn);
      rtn = robot.MoveCart(&desc_pos4, tool, user, vel, acc, ovl, blendT, -1);
      printf("MoveCart errcode:%d\n", rtn);
      rtn = robot.MoveJ(&j1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
      printf("movej errcode:%d\n", rtn);
      rtn = robot.MoveL(&desc_pos2, tool, user, vel, acc, ovl, blendR, 0, &epos, search, flag, &offset_pos);
      printf("movel errcode:%d\n", rtn);
      rtn = robot.MoveC(&desc_pos3, tool, user, vel, acc, &epos, flag, &offset_pos, &desc_pos4, tool, user, vel, acc, &epos, flag, &offset_pos, ovl, blendR);
      printf("movec errcode:%d\n", rtn);
      rtn = robot.MoveJ(&j2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
      printf("movej errcode:%d\n", rtn);
      rtn = robot.Circle(&desc_pos3, tool, user, vel, acc, &epos, &desc_pos1, tool, user, vel, acc, &epos, ovl, flag, &offset_pos, 100, -1);
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
    * @brief Servo motion start, used with ServoJ and ServoCart commands
    * @return Error code
    */
    errno_t ServoMoveStart();

Servo Motion End
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Servo motion end, used with ServoJ and ServoCart commands
    * @return Error code
    */
    errno_t ServoMoveEnd();

Joint Space Servo Mode Motion
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint space servo mode motion
    * @param [in] joint_pos Target joint position in deg
    * @param [in] acc Acceleration percentage [0~100] (not yet available, default 0)
    * @param [in] vel Speed percentage [0~100] (not yet available, default 0)
    * @param [in] cmdT Command cycle in seconds, recommended range [0.001~0.0016]
    * @param [in] filterT Filter time in seconds (not yet available, default 0)
    * @param [in] gain Target position proportional amplifier (not yet available, default 0)
    * @param  [in] gain  The target position of the proportional amplifier, temporarily closed, the default is 0
    * @param  [in] id servoJInstruction ID, which defaults to 0
    * @return  error code
    */
    errno_t ServoJ(JointPos *joint_pos, ExaxisPos* axisPos, float acc, float vel, float cmdT, float filterT, float gain, int id = 0);

Joint Space Servo Mode Motion Example
++++++++++++++++++++++++++++++++++++++++++
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
        int ret = robot.GetActualJointPosDegree(flag, &j);
        if (ret == 0)
        {
            robot.ServoMoveStart();
            while (count)
            {
                robot.ServoJ(&j, &epos, acc, vel, cmdT, filterT, gain);
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

Joint Torque Control Start
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint torque control start
    * @return Error code
    */
    errno_t ServoJTStart();

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
    * @param [in] mode 0-Absolute motion (base coordinate), 1-Incremental motion (base coordinate), 2-Incremental motion (tool coordinate)
    * @param [in] desc_pos Target Cartesian pose or pose increment
    * @param [in] pos_gain Pose increment proportional coefficient, only effective in incremental motion, range [0~1]
    * @param [in] acc Acceleration percentage [0~100] (not yet available, default 0)
    * @param [in] vel Speed percentage [0~100] (not yet available, default 0)
    * @param [in] cmdT Command cycle in seconds, recommended range [0.001~0.0016]
    * @param [in] filterT Filter time in seconds (not yet available, default 0)
    * @param [in] gain Target position proportional amplifier (not yet available, default 0)
    * @return Error code
    */
    errno_t ServoCart(int mode, DescPose *desc_pose, float pos_gain[6], float acc, float vel, float cmdT, float filterT, float gain);

Cartesian Space Servo Mode Motion Example
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
        DescPose desc_pos_dt;
        memset(&desc_pos_dt, 0, sizeof(DescPose));
        desc_pos_dt.tran.z = -0.5;
        float pos_gain[6] = { 0.0,0.0,1.0,0.0,0.0,0.0 };
        int mode = 2;
        float vel = 0.0;
        float acc = 0.0;
        float cmdT = 0.008;
        float filterT = 0.0;
        float gain = 0.0;
        uint8_t flag = 0;
        int count = 100;
        robot.SetSpeed(20);
        while (count)
        {
            robot.ServoCart(mode, &desc_pos_dt, pos_gain, acc, vel, cmdT, filterT, gain);
            count -= 1;
            robot.WaitMs(cmdT * 1000);
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