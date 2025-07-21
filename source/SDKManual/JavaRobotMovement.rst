Robot movement
========================

.. toctree::
    :maxdepth: 5


Jog movement
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Jog movement 
    * @param [in] refType 0-joint jog, 2-base coordinate jog, 4-tool coordinate jog, 8-workpiece coordinate jog
    * @param [in] nb 1-joint 1 (or x-axis), 2-joint 2 (or y-axis), 3-joint 3 (or z-axis), 4-joint 4 (or rotation about x-axis), 5-joint 5 (or rotation about y-axis), 6-joint 6 (or rotation about z-axis)
    * @param [in] dir 0-negative direction, 1-positive direction
    * @param [in] vel Speed percentage, [0~100]
    * @param [in] acc Acceleration percentage, [0~100]
    * @param [in] max_dis Maximum angle per jog movement in [°] or distance in [mm]
    * @return Error code 
    */ 
    int StartJOG(int refType, int nb, int dir, double vel, double acc, double max_dis);

Jog deceleration stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Jog deceleration stop
    * @param  [in]  stopType  1-joint jog stop, 3-base coordinate jog stop, 5-tool coordinate jog stop, 9-workpiece coordinate jog stop
    * @return  Error code
    */
    int StopJOG(int stopType);

Jog immediate stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Jog immediate stop
    * @return  Error code
    */
    int ImmStopJOG(); 

Robot jog control code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static  int TestJOG(Robot robot)
    {
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
        return 0;
    }

Joint space movement
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Joint space movement
    * @param  [in] joint_pos  Target joint position in deg
    * @param  [in] desc_pos  Target cartesian pose
    * @param  [in] tool  Tool coordinate number, range [0~14]
    * @param  [in] user  Workpiece coordinate number, range [0~14]
    * @param  [in] vel  Speed percentage, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not currently available
    * @param  [in] ovl  Speed scaling factor, range [0~100]
    * @param  [in] epos  Extended axis position in mm
    * @param  [in] blendT [-1.0]-move to position (blocking), [0~500.0]-smoothing time (non-blocking) in ms
    * @param  [in] offset_flag  0-no offset, 1-offset in base/workpiece coordinate, 2-offset in tool coordinate
    * @param  [in] offset_pos  Pose offset
    * @return  Error code
    */
    int MoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, ExaxisPos epos, double blendT, int offset_flag, DescPose offset_pos);

Cartesian space linear movement
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Java SDK-v1.0.5-3.8.2

.. code-block:: Java
    :linenos:

    /**
    * @brief  Cartesian space linear movement
    * @param  [in] joint_pos  Target joint position in deg
    * @param  [in] desc_pos   Target cartesian pose
    * @param  [in] tool  Tool coordinate number, range [0~14]
    * @param  [in] user  Workpiece coordinate number, range [0~14]
    * @param  [in] vel  Speed percentage, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not currently available
    * @param  [in] ovl  Speed scaling factor, range [0~100]
    * @param  [in] blendR [-1.0]-move to position (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm
    * @param  [in] blendMode Transition mode; 0-tangent transition; 1-corner transition
    * @param  [in] epos  Extended axis position in mm
    * @param  [in] search  0-no wire search, 1-wire search
    * @param  [in] offset_flag  0-no offset, 1-offset in base/workpiece coordinate, 2-offset in tool coordinate
    * @param  [in] offset_pos  Pose offset
    * @param  [in] overSpeedStrategy  Overspeed handling strategy, 1-standard; 2-error stop when overspeed; 3-adaptive speed reduction, default 0
    * @param  [in] speedPercent  Allowed speed reduction threshold percentage [0-100], default 10%
    * @return  Error code
    */   
    int MoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendR, int blendMode,ExaxisPos epos, int search, int offset_flag, DescPose offset_pos, int overSpeedStrategy, int speedPercent);

Cartesian space circular movement
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Cartesian space circular movement
    * @param  [in] joint_pos_p  Path point joint position in deg
    * @param  [in] desc_pos_p   Path point cartesian pose
    * @param  [in] ptool  Tool coordinate number, range [0~14]
    * @param  [in] puser  Workpiece coordinate number, range [0~14]
    * @param  [in] pvel  Speed percentage, range [0~100]
    * @param  [in] pacc  Acceleration percentage, range [0~100], not currently available
    * @param  [in] epos_p  Extended axis position in mm
    * @param  [in] poffset_flag  0-no offset, 1-offset in base/workpiece coordinate, 2-offset in tool coordinate
    * @param  [in] offset_pos_p  Pose offset
    * @param  [in] joint_pos_t  Target point joint position in deg
    * @param  [in] desc_pos_t   Target point cartesian pose
    * @param  [in] ttool  Tool coordinate number, range [0~14]
    * @param  [in] tuser  Workpiece coordinate number, range [0~14]
    * @param  [in] tvel  Speed percentage, range [0~100]
    * @param  [in] tacc  Acceleration percentage, range [0~100], not currently available
    * @param  [in] epos_t  Extended axis position in mm
    * @param  [in] toffset_flag  0-no offset, 1-offset in base/workpiece coordinate, 2-offset in tool coordinate
    * @param  [in] offset_pos_t  Pose offset
    * @param  [in] ovl  Speed scaling factor, range [0~100]
    * @param  [in] blendR [-1.0]-move to position (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm 
    * @return  Error code
    */      
    int MoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, double pvel, double pacc, ExaxisPos epos_p, int poffset_flag, DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, double tvel, double tacc, ExaxisPos epos_t, int toffset_flag, DescPose offset_pos_t, double ovl, double blendR);

Cartesian space full circle movement
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Java SDK-v1.0.6-3.8.3

.. code-block:: Java
    :linenos:

    /**
    * @brief  Cartesian space full circle movement
    * @param  [in] joint_pos_p  Path point 1 joint position in deg
    * @param  [in] desc_pos_p   Path point 1 cartesian pose
    * @param  [in] ptool  Tool coordinate number, range [0~14]
    * @param  [in] puser  Workpiece coordinate number, range [0~14]
    * @param  [in] pvel  Speed percentage, range [0~100]
    * @param  [in] pacc  Acceleration percentage, range [0~100], not currently available
    * @param  [in] epos_p  Extended axis position in mm
    * @param  [in] joint_pos_t  Path point 2 joint position in deg
    * @param  [in] desc_pos_t   Path point 2 cartesian pose
    * @param  [in] ttool  Tool coordinate number, range [0~14]
    * @param  [in] tuser  Workpiece coordinate number, range [0~14]
    * @param  [in] tvel  Speed percentage, range [0~100]
    * @param  [in] tacc  Acceleration percentage, range [0~100], not currently available
    * @param  [in] epos_t  Extended axis position in mm
    * @param  [in] ovl  Speed scaling factor, range [0~100]
    * @param  [in] offset_flag  0-no offset, 1-offset in base/workpiece coordinate, 2-offset in tool coordinate
    * @param  [in] offset_pos  Pose offset
    * @param  [in] oacc Acceleration percentage
    * @param  [in] blendR -1: blocking; 0~1000: smoothing radius
    * @return  Error code
    */
    int Circle(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, double pvel, double pacc, ExaxisPos epos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, double tvel, double tacc, ExaxisPos epos_t, double ovl, int offset_flag, DescPose offset_pos, double oacc, double blendR)

Cartesian space point-to-point movement
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Cartesian space point-to-point movement 
    * @param [in] desc_pos  Target cartesian pose or pose increment
    * @param [in] tool  Tool coordinate number, range [0~14]
    * @param [in] user  Workpiece coordinate number, range [0~14]
    * @param [in] vel  Speed percentage, range [0~100]
    * @param [in] acc  Acceleration percentage, range [0~100], not currently available
    * @param [in] ovl  Speed scaling factor, range [0~100]
    * @param [in] blendT [-1.0]-move to position (blocking), [0~500.0]-smoothing time (non-blocking) in ms
    * @param [in] config  Joint space configuration, [-1]-calculate with reference to current joint position, [0~7]-calculate with reference to specific joint space configuration, default -1
    * @return Error code 
    */ 
    int MoveCart(DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendT, int config);

Basic robot movement command code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestMove(Robot robot)
    {
        int rtn=-1;
        JointPos j1=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2=new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        JointPos j3=new JointPos(-29.777, -84.536, 109.275, -114.075, -86.655, 74.257);
        JointPos j4=new JointPos(-31.154, -95.317, 94.276, -88.079, -89.740, 74.256);
        DescPose desc_pos1=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose desc_pos3=new DescPose(-487.434, 154.362, 308.576, 176.600, 0.268, -14.061);
        DescPose desc_pos4=new DescPose(-443.165, 147.881, 480.951, 179.511, -0.775, -15.409);
        DescPose offset_pos=new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos=new ExaxisPos(0, 0, 0, 0);

        int tool = 0;
        int user = 0;
        double vel = 100.0;
        double acc = 100.0;
        double ovl = 100.0;
        double blendT = 0.0;
        double blendR = 0.0;
        int flag = 0;
        int search = 0;

        robot.SetSpeed(20);

        rtn = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        System.out.println("movej errcode:"+ rtn);

        rtn = robot.MoveL(j2, desc_pos2, tool, user, vel, acc, ovl, blendR, 0,epos, search, flag, offset_pos,0,10);
        System.out.println("movel errcode:"+ rtn);

        rtn = robot.MoveC(j3, desc_pos3, tool, user, vel, acc, epos, flag, offset_pos, j4, desc_pos4, tool, user, vel, acc, epos, flag, offset_pos, ovl, blendR);
        System.out.println("movec errcode:"+ rtn);

        rtn = robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        System.out.println("movej errcode:"+ rtn);

        rtn = robot.Circle(j3, desc_pos3, tool, user, vel, acc, epos, j1, desc_pos1, tool, user, vel, acc, epos, ovl, flag, offset_pos);
        System.out.println("circle errcode:"+ rtn);

        rtn = robot.MoveCart(desc_pos4, tool, user, vel, acc, ovl, blendT, -1);
        System.out.println("MoveCart errcode:"+ rtn);

        return 0;
    }

Cartesian space spiral movement
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Cartesian space spiral movement 
    * @param [in] joint_pos  Target joint position in deg
    * @param [in] desc_pos   Target cartesian pose
    * @param [in] tool  Tool coordinate number, range [0~14]
    * @param [in] user  Workpiece coordinate number, range [0~14]
    * @param [in] vel  Speed percentage, range [0~100]
    * @param [in] acc  Acceleration percentage, range [0~100], not currently available
    * @param [in] epos  Extended axis position in mm
    * @param [in] ovl  Speed scaling factor, range [0~100]
    * @param [in] offset_flag  0-no offset, 1-offset in base/workpiece coordinate, 2-offset in tool coordinate
    * @param [in] offset_pos  Pose offset
    * @return Error code 
    */
    int NewSpiral(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, ExaxisPos epos, double ovl, int offset_flag, DescPose offset_pos, SpiralParam spiral_param);

Spiral movement code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestSpiral(Robot robot)
    {
        int rtn=-1;
        JointPos j=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        DescPose desc_pos=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose offset_pos1=new DescPose(50, 0, 0, -30, 0, 0);
        DescPose offset_pos2=new DescPose(50, 0, 0, -5, 0, 0);
        ExaxisPos epos=new ExaxisPos(0, 0, 0, 0);
        SpiralParam sp=new SpiralParam();
        sp.circle_num = 5;
        sp.circle_angle = 5.0;
        sp.rad_init = 50.0;
        sp.rad_add = 10.0;
        sp.rotaxis_add = 10.0;
        sp.rot_direction = 0;

        int tool = 0;
        int user = 0;
        double vel = 100.0;
        double acc = 100.0;
        double ovl = 100.0;
        double blendT = 0.0;
        int flag = 2;

        robot.SetSpeed(20);

        rtn = robot.MoveJ(j, desc_pos, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos1);
        System.out.println("movej errcode:"+ rtn);

        rtn = robot.NewSpiral(j, desc_pos, tool, user, vel, acc, epos, ovl, flag, offset_pos2, sp);
        System.out.println("newspiral errcode:"+ rtn);

        return 0;
    }

Servo movement start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Servo movement start, used with ServoJ, ServoCart commands
    * @return Error code 
    */ 
    int ServoMoveStart();

Servo movement end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Servo movement end, used with ServoJ, ServoCart commands
    * @return Error code 
    */ 
    int ServoMoveEnd();

Joint space servo mode movement
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Java SDK-v1.0.6-3.8.3

.. code-block:: Java
    :linenos:

    /**
    * @brief  Joint space servo mode movement
    * @param  [in] joint_pos  Target joint position in deg
    * @param  [in] axisPos  External axis position in mm
    * @param  [in] acc  Acceleration percentage, range [0~100], not currently available, default 0
    * @param  [in] vel  Speed percentage, range [0~100], not currently available, default 0
    * @param  [in] cmdT  Command cycle in s, recommended range [0.001~0.0016]
    * @param  [in] filterT Filter time in s, not currently available, default 0
    * @param  [in] gain  Target position proportional amplifier, not currently available, default 0
    * @param  [in] id  servoJ command ID, default 0
    * @return  Error code
    */
    int ServoJ(JointPos joint_pos, ExaxisPos axisPos, double acc, double vel, double cmdT, double filterT, double gain, int id);

Joint space servo mode movement example program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void TestServoJ()
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set reconnection times and interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        JointPos j5 = new JointPos();
        ExaxisPos ePos=new ExaxisPos();
        int ret = robot.GetActualJointPosDegree(j5);
        if (ret == 0)
        {
            int count = 200;
            while (count > 0)
            {
                robot.ServoJ(j5, ePos,100, 100, 0.008, 0, 0);
                j5.J1 += 0.2;//Joint 1 position increase
                count -= 1;
                robot.WaitMs((int)(8));
            }
        }
    }

Joint torque control start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Joint torque control start
    * @return  Error code
    */
    int ServoJTStart()

Joint torque control
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Joint torque control
    * @param  [in] torque j1~j6 joint torque in Nm
    * @param  [in] interval Command cycle in s, range [0.001~0.008]
    * @return  Error code
    */
    int ServoJT(Object[] torque, double interval)

Joint torque control end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Joint torque control end
    * @return  Error code
    */
    int ServoJTEnd()

Joint space servo mode movement example program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestServoJT(Robot robot)
    {

        robot.DragTeachSwitch(1);
        List<Number> joint_toq=new ArrayList<>();
        joint_toq=robot.GetJointTorques(1);

        int count = 100;
        robot.ServoJTStart(); //   #servoJT start
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

Cartesian space servo mode movement
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Cartesian space servo mode movement
    * @param  [in]  mode  0-absolute movement (base coordinate), 1-incremental movement (base coordinate), 2-incremental movement (tool coordinate)
    * @param  [in]  desc_pose  Target cartesian pose or pose increment
    * @param  [in]  pos_gain  Pose increment proportional coefficient, only effective in incremental movement, range [0~1]
    * @param  [in]  acc  Acceleration percentage, range [0~100], not currently available, default 0
    * @param  [in]  vel  Speed percentage, range [0~100], not currently available, default 0
    * @param  [in]  cmdT  Command cycle in s, recommended range [0.001~0.0016]
    * @param  [in]  filterT Filter time in s, not currently available, default 0
    * @param  [in]  gain  Target position proportional amplifier, not currently available, default 0
    * @return  Error code
    */
    int ServoCart(int mode, DescPose desc_pose, Object[] pos_gain, double acc, double vel, double cmdT, double filterT, double gain);

Cartesian space servo mode movement code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestServoCart(Robot robot)
    {
        DescPose desc_pos_dt=new DescPose(0,0,0,0,0,0);

        desc_pos_dt.tran.z = -0.5;
        Object[] pos_gain = { 0.0,0.0,1.0,0.0,0.0,0.0 };
        int mode = 2;
        double vel = 0.0;
        double acc = 0.0;
        double cmdT = 0.008;
        double filterT = 0.0;
        double gain = 0.0;
        int flag = 0;
        int count = 100;

        robot.SetSpeed(20);

        while (count>0)
        {
            robot.ServoCart(mode, desc_pos_dt, pos_gain, acc, vel, cmdT, filterT, gain);
            count -= 1;
            double time=cmdT*1000;
            robot.WaitMs((int)time);
        }

        return 0;
    }

Spline movement start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Spline movement start
    * @return  Error code
    */
    int SplineStart();

Joint movement PTP
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Joint space spline movement
    * @param  [in] joint_pos  Target joint position in deg
    * @param  [in] desc_pos   Target cartesian pose
    * @param  [in] tool  Tool coordinate number, range [0~14]
    * @param  [in] user  Workpiece coordinate number, range [0~14]
    * @param  [in] vel  Speed percentage, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not currently available
    * @param  [in] ovl  Speed scaling factor, range [0~100]
    * @return  Error code
    */
    int SplinePTP(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl);

Spline movement end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Spline movement end
    * @return  Error code
    */
    int SplineEnd(); 

Spline movement code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestSpline(Robot robot)
    {
        JointPos j1=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2=new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        JointPos j3=new JointPos(-61.954, -84.409, 108.153, -116.316, -91.283, 74.260);
        JointPos j4=new JointPos(-89.575, -80.276, 102.713, -116.302, -91.284, 74.267);
        DescPose desc_pos1=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose desc_pos3=new DescPose(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
        DescPose desc_pos4=new DescPose(-104.066, 544.321, 327.023, -177.715, 3.371, -73.818);
        DescPose offset_pos=new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos=new ExaxisPos(0, 0, 0, 0);

        int tool = 0;
        int user = 0;
        double vel = 100.0;
        double acc = 100.0;
        double ovl = 100.0;
        double blendT = -1.0;
        int flag = 0;

        robot.SetSpeed(20);

        int err1 = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        System.out.println("movej errcode:"+ err1);
        robot.SplineStart();
        robot.SplinePTP(j1, desc_pos1, tool, user, vel, acc, ovl);
        robot.SplinePTP(j2, desc_pos2, tool, user, vel, acc, ovl);
        robot.SplinePTP(j3, desc_pos3, tool, user, vel, acc, ovl);
        robot.SplinePTP(j4, desc_pos4, tool, user, vel, acc, ovl);
        robot.SplineEnd();

        return 0;
    }

New spline movement start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief New spline movement start 
    * @param [in] type   0-circular transition, 1-given points as path points
    * @param [in] averageTime  Global average transition time(ms)(10 ~  ), default 2000
    * @return Error code 
    */ 
    int NewSplineStart(int type, int averageTime);
    
New spline command point
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Add spline movement command point 
    * @param [in] joint_pos  Target joint position in deg
    * @param [in] desc_pos   Target cartesian pose
    * @param [in] tool  Tool coordinate number, range [0~14]
    * @param [in] user  Workpiece coordinate number, range [0~14]
    * @param [in] vel  Speed percentage, range [0~100]
    * @param [in] acc  Acceleration percentage, range [0~100], not currently available
    * @param [in] ovl  Speed scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-move to position (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm
    * @param [in] lastFlag Whether it is the last point, 0-no, 1-yes
    * @return Error code 
    */ 
    int NewSplinePoint(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendR, int lastFlag);

New spline movement end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief New spline movement start 
    * @return Error code 
    */ 
    int NewSplineEnd();
    
New spline movement code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestNewSpline(Robot robot)
    {
        JointPos j1=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2=new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        JointPos j3=new JointPos(-61.954, -84.409, 108.153, -116.316, -91.283, 74.260);
        JointPos j4=new JointPos(-89.575, -80.276, 102.713, -116.302, -91.284, 74.267);
        JointPos j5=new JointPos(-95.228, -54.621, 73.691, -112.245, -91.280, 74.268);
        DescPose desc_pos1=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose desc_pos3=new DescPose(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
        DescPose desc_pos4=new DescPose(-104.066, 544.321, 327.023, -177.715, 3.371, -73.818);
        DescPose desc_pos5=new DescPose(-33.421, 732.572, 275.103, -177.907, 2.709, -79.482);
        DescPose offset_pos=new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos=new ExaxisPos(0, 0, 0, 0);

        int tool = 0;
        int user = 0;
        double vel = 100.0;
        double acc = 100.0;
        double ovl = 100.0;
        double blendT = -1.0;
        int flag = 0;

        robot.SetSpeed(20);

        int err1 = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        System.out.println("movej errcode:"+ err1);
        robot.NewSplineStart(1, 2000);
        robot.NewSplinePoint(j1, desc_pos1, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(j2, desc_pos2, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(j3, desc_pos3, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(j4, desc_pos4, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(j5, desc_pos5, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplineEnd();
        return 0;
    }

Stop movement
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Stop movement
    * @return  Error code
    */
    int StopMotion();

Pause movement
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:
    
    /** 
      * @brief Pause movement 
      * @return Error code 
    */  
    int PauseMotion();

Resume movement
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Resume movement 
    * @return Error code 
    */ 
    int ResumeMotion();

Movement pause, resume, stop code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestPause(Robot robot)
    {
        JointPos j1=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j5=new JointPos(-95.228, -54.621, 73.691, -112.245, -91.280, 74.268);
        DescPose desc_pos1=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos5=new DescPose(-33.421, 732.572, 275.103, -177.907, 2.709, -79.482);
        DescPose offset_pos=new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos=new ExaxisPos(0, 0, 0, 0);

        int tool = 0;
        int user = 0;
        double vel = 100.0;
        double acc = 100.0;
        double ovl = 100.0;
        double blendT = -1.0;
        int flag = 0;

        robot.SetSpeed(20);
        int rtn=-1;
        rtn = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        rtn = robot.MoveJ(j5, desc_pos5, tool, user, vel, acc, ovl, epos, 1, flag, offset_pos);
        robot.Sleep(1000);
        robot.PauseMotion();

        robot.Sleep(1000);
        robot.ResumeMotion();

        robot.Sleep(1000);
        robot.StopMotion();

        robot.Sleep(1000);

        return 0;
    }

Point global offset start
+++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Point global offset start
    * @param  [in]  flag  0-offset in base/workpiece coordinate, 2-offset in tool coordinate
    * @param  [in]  offset_pos  Pose offset
    * @return  Error code
    */
    int PointsOffsetEnable(int flag, DescPose offset_pos); 


Point global offset end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Point global offset end
    * @return  Error code
    */
    int PointsOffsetDisable(); 

Point offset code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestOffset(Robot robot)
    {
        JointPos j1=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2=new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);

        DescPose desc_pos1=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

        DescPose offset_pos=new DescPose(0, 0, 0, 0, 0, 0);
        DescPose offset_pos1=new DescPose(0, 0, 50, 0, 0, 0);
        ExaxisPos epos=new ExaxisPos(0, 0, 0, 0);

        int tool = 0;
        int user = 0;
        double vel = 100.0;
        double acc = 100.0;
        double ovl = 100.0;
        double blendT = -1.0;
        int flag = 0;

        robot.SetSpeed(20);

        robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.Sleep(1000);
        robot.PointsOffsetEnable(0, offset_pos1);
        robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.PointsOffsetDisable();

        return 0;
    }

Controller AO flying start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Controller AO flying start
    * @param [in] AONum Controller AO number
    * @param [in] maxTCPSpeed Maximum TCP speed value[1-5000mm/s], default 1000
    * @param [in] maxAOPercent AO percentage corresponding to maximum TCP speed, default 100%
    * @param [in] zeroZoneCmp Dead zone compensation value AO percentage, integer, default 20%, range [0-100]
    * @return Error code
    */
    int MoveAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);

Controller AO flying stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
.. code-block:: Java
    :linenos:

    /**
    * @brief Controller AO flying stop
    * @return Error code
    */
    int MoveAOStop();
    
End effector AO flying start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
.. code-block:: Java
    :linenos:

    /**
    * @brief End effector AO flying start
    * @param [in] AONum End effector AO number
    * @param [in] maxTCPSpeed Maximum TCP speed value[1-5000mm/s], default 1000
    * @param [in] maxAOPercent AO percentage corresponding to maximum TCP speed, default 100%
    * @param [in] zeroZoneCmp Dead zone compensation value AO percentage, integer, default 20%, range [0-100]
    * @return Error code
    */
    int MoveToolAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);
    
End effector AO flying stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
.. code-block:: Java
    :linenos:

    /**
    * @brief End effector AO flying stop
    * @return Error code
    */
    int MoveToolAOStop();

AO flying code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestMoveAO(Robot robot)
    {
        JointPos j1=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2=new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);

        DescPose desc_pos1=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

        DescPose offset_pos=new DescPose(0, 0, 0, 0, 0, 0);
        DescPose offset_pos1=new DescPose(0, 0, 50, 0, 0, 0);
        ExaxisPos epos=new ExaxisPos(0, 0, 0, 0);

        int tool = 0;
        int user = 0;
        double vel = 20.0;
        double acc = 20.0;
        double ovl = 100.0;
        double blendT = -1.0;
        int flag = 0;

        robot.SetSpeed(20);

        robot.MoveAOStart(0, 100, 100, 20);
        robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveAOStop();

        robot.Sleep(1000);

        robot.MoveToolAOStart(0, 100, 100, 20);
        robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveToolAOStop();

        return 0;
    }

Start Ptp movement FIR filtering
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Java SDK-v1.0.5-3.8.2

.. code-block:: Java
    :linenos:

    /**
    * @brief Start Ptp movement FIR filtering
    * @param [in] maxAcc Maximum acceleration limit (deg/s2)
    * @param [in] maxJek Unified joint jerk limit (deg/s3)
    * @return Error code
    */
    int PtpFIRPlanningStart(double maxAcc,double maxJek);

Close Ptp movement FIR filtering
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Close Ptp movement FIR filtering
    * @return Error code
    */
    int PtpFIRPlanningEnd();

Start LIN, ARC movement FIR filtering
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Start LIN, ARC movement FIR filtering
    * @param [in] maxAccLin Linear acceleration limit (mm/s2)
    * @param [in] maxAccDeg Angular acceleration limit (deg/s2)
    * @param [in] maxJerkLin Linear jerk limit (mm/s3)
    * @param [in] maxJerkDeg Angular jerk limit (deg/s3)
    * @return Error code
    */
    int LinArcFIRPlanningStart(double maxAccLin, double maxAccDeg, double maxJerkLin, double maxJerkDeg);

Close LIN, ARC movement FIR filtering
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Close LIN, ARC movement FIR filtering
    * @return Error code
    */
    int LinArcFIRPlanningEnd();

FIR filtering code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestFIR(Robot robot)
    {
        JointPos startjointPos=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos midjointPos=new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        JointPos endjointPos=new JointPos(-29.777, -84.536, 109.275, -114.075, -86.655, 74.257);

        DescPose startdescPose=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose middescPose=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose enddescPose=new DescPose(-487.434, 154.362, 308.576, 176.600, 0.268, -14.061);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        int rtn = robot.PtpFIRPlanningStart(1000, 1000);
        robot.MoveJ(startjointPos, startdescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveJ(endjointPos, enddescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.PtpFIRPlanningEnd();

        robot.LinArcFIRPlanningStart(1000, 1000, 1000, 1000);
        robot.MoveL(startjointPos, startdescPose, 0, 0, 100, 100, 100, -1, 0,exaxisPos, 0, 0, offdese, 1, 1);
        robot.MoveC(midjointPos, middescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, 100, -1);
        robot.LinArcFIRPlanningEnd();
        return 0;
    }

Acceleration smoothing enable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.4-3.8.1
.. code-block:: Java
    :linenos:

    /**
     * @brief Acceleration smoothing enable
     * @param [in] saveFlag Whether to save after power off
     * @return  Error code
     */
    public int AccSmoothStart(boolean saveFlag)

Acceleration smoothing disable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.4-3.8.1
.. code-block:: Java
    :linenos:

    /**
     * @brief Acceleration smoothing disable
     * @param [in] saveFlag Whether to save after power off
     * @return  Error code
     */
    public int AccSmoothEnd(boolean saveFlag)

Acceleration smoothing code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestAccSmooth(Robot robot)
    {
        JointPos startjointPos=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos endjointPos=new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);

        DescPose startdescPose=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose enddescPose=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0,0,0,0,0,0);
        int rtn = robot.AccSmoothStart(false);
        robot.MoveJ(startjointPos, startdescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveJ(endjointPos, enddescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        rtn = robot.AccSmoothEnd(false);

        robot.CloseRPC();
        return 0;
    }

Specified pose speed enable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Specified pose speed enable
     * @param [in] ratio Pose speed percentage [0-300]
     * @return  Error code
     */
    int AngularSpeedStart(int ratio)

Specified pose speed disable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Specified pose speed disable
     * @return  Error code
     */
    int AngularSpeedEnd();

Robot specified pose speed code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestAngularSpeed(Robot robot)
    {
        JointPos startjointPos=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos endjointPos=new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);

        DescPose startdescPose=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose enddescPose=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);
        int rtn = robot.AngularSpeedStart(50);
        robot.MoveJ(startjointPos, startdescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveJ(endjointPos, enddescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        rtn = robot.AngularSpeedEnd();

        return 0;
    }

Start singular pose protection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Start singular pose protection
    * @param  [in]  protectMode Singular protection mode, 0: joint mode; 1-cartesian mode
    * @param  [in]  minShoulderPos Shoulder singular adjustment range(mm), default 100
    * @param  [in]  minElbowPos Elbow singular adjustment range(mm), default 50
    * @param  [in]  minWristPos Wrist singular adjustment range(°), default 10
    * @return  Error code
    */
    int SingularAvoidStart(int protectMode, double minShoulderPos, double minElbowPos, double minWristPos);

Stop singular pose protection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Stop singular pose protection
    * @return  Error code
    */
    int SingularAvoidEnd();

Robot singular pose protection code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestAngularSpeed(Robot robot)
    {
        JointPos startjointPos=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos endjointPos=new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);

        DescPose startdescPose=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose enddescPose=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);
        int rtn = robot.AngularSpeedStart(50);
        robot.MoveJ(startjointPos, startdescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveJ(endjointPos, enddescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        rtn = robot.AngularSpeedEnd();

        return 0;
    }