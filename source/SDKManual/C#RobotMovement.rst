Movement
=================

.. toctree:: 
    :maxdepth: 5


Jog point movement
+++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Jog point movement
    * @param  [in]  refType 0- node movement, 2- base coordinate system, 4- tool coordinate system, 8- workpiece coordinate system
    * @param  [in]  nb 1-joint 1(or axis x), 2-joint 2(or axis y), 3-joint 3(or axis z), 4-joint 4(or rotation about axis x), 5-joint 5(or rotation about axis y), 6-joint 6(or rotation about axis z)
    * @param  [in]  dir 0-negative correlation, 1-positive correlation
    * @param  [in]  vel The percentage of velocity,[0~100]
    * @param  [in]  acc The percentage of acceleration, [0~100]
    * @param  [in]  max_dis Maximum Angle of single click, unit: [°] or distance, unit: [mm]
    * @return  Error code
    */
    int StartJOG(byte refType, byte nb, byte dir, float vel, float acc, float max_dis); 

Jog point dynamic deceleration stop
++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Jog point dynamic deceleration stop
    * @param  [in]  ref  1- point stop, 3- point stop in base coordinate system, 5- point stop in tool coordinate system, 9- point stop in workpiece coordinate system
    * @return  Error code
    */
    errno_t  StopJOG(uint8_t ref);

The jog stops immediately
+++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief The jog stops immediately
    * @param [in] stopType Type of jog stop: 1-joint jog stop, 3-jog stop in the base coordinate system, 5-jog stop in the tool coordinate system, 9-jog stop in the workpiece coordinate system 
    * @return  Error code
    */
    int StopJOG(byte stopType); 

Code example
+++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnJOG_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2"); 

        robot.SetSpeed(35);
        robot.StartJOG(0, 1, 0, 15, 20.0f, 30.0f);   //For single-joint motion, StartJOG is a non-blocking command. Receiving other motion commands (including StartJOG) while in motion is discarded
        Thread.Sleep(1000);
        robot.StopJOG(1);  //Robot single axis point deceleration stop
        //robot.ImmStopJOG();  //The single axis of the robot stops immediately
        robot.StartJOG(0, 2, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(0, 3, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(0, 4, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(0, 5, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(0, 6, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();

        robot.StartJOG(2, 1, 0, 15, 20.0f, 30.0f);   //Point in the base coordinate system
        Thread.Sleep(1000);
        robot.StopJOG(3);  
        //robot.ImmStopJOG();  
        robot.StartJOG(2, 2, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(2, 3, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(2, 4, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(2, 5, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(2, 6, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();

        robot.StartJOG(4, 1, 0, 15, 20.0f, 30.0f);   //Point in the tool coordinate system
        Thread.Sleep(1000);
        robot.StopJOG(5);  
        //robot.ImmStopJOG();  
        robot.StartJOG(4, 2, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(4, 3, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(4, 4, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(4, 5, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(4, 6, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();

        robot.StartJOG(8, 1, 0, 15, 20.0f, 30.0f);   //Point in the workpiece coordinate system
        Thread.Sleep(1000);
        robot.StopJOG(9);  
        //robot.ImmStopJOG();  
        robot.StartJOG(8, 2, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(8, 3, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(8, 4, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(8, 5, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(8, 6, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
    }

Joint space motion
+++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Joint space motion
    * @param  [in] joint_pos  Target joint location, unit: deg
    * @param  [in] desc_pos   Target Cartesian position
    * @param  [in] tool  Tool coordinate number, range [0~14]
    * @param  [in] user  Workpiece coordinate number, range [0~14]
    * @param  [in] vel  Percentage of speed, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] ovl  Velocity scaling factor, range[0~100]
    * @param  [in] epos  Position of expansion shaft, unit: mm
    * @param  [in] blendT [-1.0]- movement in place (blocking), [0~500.0]- smoothing time (non-blocking), in ms
    * @param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos  The pose offset
    * @return  Error code
    */
    int MoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos epos, float blendT, byte offset_flag, DescPose offset_pos); 

Rectilinear motion in Cartesian space
++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Rectilinear motion in Cartesian space
    * @param  [in] joint_pos  Target joint location, unit: deg
    * @param  [in] desc_pos   Target Cartesian position
    * @param  [in] tool  Tool coordinate number, range [0~14]
    * @param  [in] user  Workpiece coordinate number, range [0~14]
    * @param  [in] vel  Percentage of speed, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] ovl  Velocity scaling factor, range[0~100]
    * @param  [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm    
    * @param  [in] epos  Position of expansion shaft, unit: mm
    * @param  [in] search  0- no wire seeking, 1- wire seeking
    * @param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos  The pose offset
    * @return  Error code
    */   
    int MoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, byte search, byte offset_flag, DescPose offset_pos); 

Circular arc motion in Cartesian space
++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Circular arc motion in Cartesian space
    * @param  [in] joint_pos_p  Waypoint joint position, unit: deg
    * @param  [in] desc_pos_p   Waypoint Cartesian position
    * @param  [in] ptool  Tool coordinate number, range [0~14]
    * @param  [in] puser  Workpiece coordinate number, range [0~14]
    * @param  [in] pvel  Percentage of speed, range [0~100]
    * @param  [in] pacc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] epos_p  Position of expansion shaft, unit: mm
    * @param  [in] poffset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos_p  The pose offset
    * @param  [in] joint_pos_t  Target joint position, unit: deg
    * @param  [in] desc_pos_t   Target point Cartesian position
    * @param  [in] ttool  Tool coordinate number, range [0~14]
    * @param  [in] tuser  Workpiece coordinate number, range [0~14]
    * @param  [in] tvel  Percentage of speed, range [0~100]
    * @param  [in] tacc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] epos_t  Position of expansion shaft, unit: mm
    * @param  [in] toffset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos_t  The pose offset   
    * @param  [in] ovl  Velocity scaling factor, range[0~100]    
    * @param  [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm    
    * @return  Error code
    */      
    int MoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos epos_p, byte poffset_flag, DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos epos_t, byte toffset_flag, DescPose offset_pos_t, float ovl, float blendR); 

Circular motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Circular motion in Cartesian space
    * @param  [in] joint_pos_p  Path point 1 joint position, unit: deg
    * @param  [in] desc_pos_p   Waypoint 1 Cartesian position
    * @param  [in] ptool  Tool coordinate number, range [0~14]
    * @param  [in] puser  Workpiece coordinate number, range [0~14]
    * @param  [in] pvel  Percentage of speed, range [0~100]
    * @param  [in] pacc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] epos_p  Position of expansion shaft, unit: mm
    * @param  [in] joint_pos_t  Joint position at waypoint 2, unit: deg
    * @param  [in] desc_pos_t   Waypoint 2 Cartesian position
    * @param  [in] ttool  Tool coordinate number, range [0~14]
    * @param  [in] tuser  Workpiece coordinate number, range [0~14]
    * @param  [in] tvel  Percentage of speed, range [0~100]
    * @param  [in] tacc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] epos_t  Position of expansion shaft, unit: mm
    * @param  [in] ovl  Velocity scaling factor, range[0~100]   
    * @param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos  The pose offset     
    * @return  Error code
    */      
    int Circle(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos epos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos epos_t, float ovl, byte offset_flag, DescPose offset_pos);

Code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnMovetest_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2"); 

        JointPos j1, j2, j3, j4;
        DescPose desc_pos1, desc_pos2, desc_pos3, desc_pos4, offset_pos;
        ExaxisPos epos;

        j1 = new JointPos(-58.982, -90.717, 127.647, -129.041, -87.989, -0.062);
        desc_pos1 = new DescPose(-437.039, 411.064, 426.189, -177.886, 2.007, 31.155);

        j2 = new JointPos(-58.978, -76.817, 112.494, -127.348, -89.145, -0.063);
        desc_pos2 = new DescPose(-525.55, 562.3, 417.199, -178.325, 0.847, 31.109);

        j3 = new JointPos(-71.746, -87.177, 123.953, -126.25, -89.429, -0.089);
        desc_pos3 = new DescPose(-345.155, 535.733, 421.269, 179.475, 0.571, 18.332);

        ExaxisPos ePos = new ExaxisPos(0, 0, 0, 0);
        DescPose offset = new DescPose();

        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = 0.0f;
        float blendR = 0.0f;
        byte flag = 0;
        byte search = 0;

        robot.SetSpeed(20);
        int err = -1;
        err = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, ePos, blendT, flag, offset);
        Console.WriteLine($"movej errcode:  {err}");

        Thread.Sleep(1000);
        err = robot.MoveL(j2, desc_pos2, tool, user, vel, acc, ovl, blendR, ePos, search, flag, offset);
        Console.WriteLine($"moveL errcode:  {err}");

        Thread.Sleep(1000);
        err = robot.MoveL(j1, desc_pos1, tool, user, vel, acc, ovl, blendR, ePos, search, flag, offset);
        Console.WriteLine($"moveL errcode:  {err}");

        Thread.Sleep(1000);
        err = robot.MoveC(j2, desc_pos2, tool, user, vel, acc, ePos, flag, offset, j3, desc_pos3, tool, user, vel, acc, ePos, flag, offset, ovl, blendR);
        Console.WriteLine($"circle errcode:  {err}");

        Thread.Sleep(1000);
        err = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, ePos, blendT, flag, offset);
        Console.WriteLine($"movej errcode:  {err}");

        Thread.Sleep(1000);
        err = robot.Circle(j2, desc_pos2, tool, user, vel, acc, ePos, j3, desc_pos3, tool, user, vel, acc, ePos, ovl, flag, offset);
        Console.WriteLine($"circle errcode:  {err}");
    }

Spiral motion in Cartesian space
+++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Spiral motion in Cartesian space
    * @param  [in] joint_pos  Target joint location, unit: deg
    * @param  [in] desc_pos   Target Cartesian position
    * @param  [in] tool  Tool coordinate number, range [0~14]
    * @param  [in] user  Workpiece coordinate number, range [0~14]
    * @param  [in] vel  Percentage of speed, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] epos  Position of expansion shaft, unit: mm
    * @param  [in] ovl  Velocity scaling factor, range[0~100]    
    * @param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos  The pose offset
    * @param  [in] spiral_param  Spiral parameter
    * @return  Error code
    */
    int NewSpiral(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, ExaxisPos epos, float ovl, byte offset_flag, DescPose offset_pos, SpiralParam spiral_param); 

Code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnDescSpiral_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");
        JointPos j;
        DescPose desc_pos;
        DescPose offset_pos1 = new DescPose(0, 0, 0, 0, 0, 0);
        DescPose offset_pos2 = new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        SpiralParam sp;

        j = new JointPos(-58.982, -90.717, 127.647, -129.041, -87.989, -0.062);
        desc_pos = new DescPose(-437.039, 411.064, 426.189, -177.886, 2.007, 31.155);

        offset_pos1.tran.x = 50.0;
        offset_pos1.rpy.rx = -30.0;
        offset_pos2.tran.x = 50.0;
        offset_pos2.rpy.rx = -5.0;

        sp.circle_num = 5;
        sp.circle_angle = 1.0f;
        sp.rad_init = 10.0f;
        sp.rad_add = 40.0f;
        sp.rotaxis_add = 10.0f;
        sp.rot_direction = 0;

        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = 0.0f;
        byte flag = 2;

        robot.SetSpeed(20);
        int ret = robot.GetForwardKin(j, ref desc_pos);  //The forward kinematic interface can be used to solve Cartesian space coordinates with only joint positions
        if (ret == 0)
        {
            int err = -1;
            err = robot.MoveJ(j, desc_pos, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos1);
            Console.WriteLine($"movej errcode:  {err}");

            err = robot.NewSpiral(j, desc_pos, tool, user, vel, acc, epos, ovl, flag, offset_pos2, sp);
            Console.WriteLine($"newspiral errcode:  {err}");
        }
        else
        {
            Console.WriteLine($"GetForwardKin errcode: {ret}");
        }
    }

Servo motion start
+++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Servo motion start，used with ServoJ and ServoCart commands
    * @return Error code 
    */ 
    int ServoMoveStart();

Servo motion end
+++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Servo motion end，used with ServoJ and ServoCart commands
    * @return Error code 
    */ 
    int ServoMoveEnd();

Joint space servo mode motion
+++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Joint space servo mode motion
    * @param  [in] joint_pos  Target joint location, unit: deg
    * @param  [in] acc  Acceleration percentage range[0~100], not open yet, default: 0
    * @param  [in] vel  The value ranges from 0 to 100. The value is not available. The default value is 0
    * @param  [in] cmdT Instruction delivery period, unit: s, recommended range [0.001~0.0016]
    * @param  [in] filterT Filtering time (unit: s), temporarily disabled. The default value is 0
    * @param  [in] gain  The proportional amplifier at the target position, not yet open, defaults to 0
    * @return  Error code
    */
    int ServoJ(JointPos joint_pos, float acc, float vel, float cmdT, float filterT, float gain);

Code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnJointServoMove_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        JointPos j = new JointPos(0, 0, 0, 0, 0, 0);

        float vel = 0.0f;
        float acc = 0.0f;
        float cmdT = 0.008f;
        float filterT = 0.0f;
        float gain = 0.0f;
        byte flag = 0;
        int count = 200;
        float dt = 0.1f;
        int ret = robot.GetActualJointPosDegree(flag, ref j);
        if (ret == 0)
        {
            while (count > 0)
            {
                robot.ServoJ(j, acc, vel, cmdT, filterT, gain);
                j.jPos[0] += dt;
                count -= 1;
                robot.WaitMs((int)(cmdT * 1000));
            }
        }
        else
        {
            Console.WriteLine($"GetActualJointPosDegree errcode:  {ret}");
        }
    }

Cartesian space servo mode motion
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Cartesian space servo mode motion
    * @param  [in]  mode  0- absolute motion (base coordinates), 1- incremental motion (base coordinates), 2- incremental motion (tool coordinates)
    * @param  [in]  desc_pos  Target Cartesian pose or pose increment
    * @param  [in]  pos_gain  Proportional coefficient of pose increment, effective only for incremental motion, range [0~1]
    * @param  [in] acc  Acceleration percentage range[0~100], not open yet, default: 0
    * @param  [in] vel  The value ranges from 0 to 100. The value is not available. The default value is 0
    * @param  [in] cmdT Instruction delivery period, unit: s, recommended range [0.001~0.0016]
    * @param  [in] filterT Filtering time (unit: s), temporarily disabled. The default value is 0
    * @param  [in] gain  The proportional amplifier at the target position, not yet open, defaults to 0
    * @return  Error code
    */
    int ServoCart(int mode, DescPose desc_pos, double[] pos_gain, float acc, float vel, float cmdT, float filterT, float gain);

Code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnDescServoMove_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        DescPose desc_pos_dt = new DescPose(0, 0, 0, 0, 0, 0);
        desc_pos_dt.tran.z = -0.5;
        double[] pos_gain = new double[6]{ 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
        int mode = 2;
        float vel = 0.0f;
        float acc = 0.0f;
        float cmdT = 0.008f;
        float filterT = 0.0f;
        float gain = 0.0f;
        int flag = 0;
        int count = 500;

        robot.SetSpeed(20);
        while (count > 0)
        {
            robot.ServoCart(mode, desc_pos_dt, pos_gain, acc, vel, cmdT, filterT, gain);
            count -= 1;
            robot.WaitMs((int)(cmdT * 1000));
        }
    }

Point to point motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Point to point motion in Cartesian space
    * @param  [in]  desc_pos  Target Cartesian pose or pose increment
    * @param  [in] tool  Tool coordinate number, range [0~14]
    * @param  [in] user  Workpiece coordinate number, range [0~14]
    * @param  [in] vel  Percentage of speed, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] ovl  Velocity scaling factor, range[0~100]
    * @param  [in] blendT [-1.0]- movement in place (blocking), [0~500.0]- smoothing time (non-blocking), in ms
    * @param  [in] config  Joint space configuration, [-1]- refer to the current joint position, [0~7]- refer to the specific joint space configuration, the default is -1 
    * @return  Error code
    */
    int MoveCart(DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendT, int config);

Code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnDescPTPMove_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        DescPose desc_pos1, desc_pos2, desc_pos3;
        desc_pos1 = new DescPose(-437.039, 411.064, 426.189, -177.886, 2.007, 31.155);
        desc_pos2 = new DescPose(-525.55, 562.3, 417.199, -178.325, 0.847, 31.109);
        desc_pos3 = new DescPose(-345.155, 535.733, 421.269, 179.475, 0.571, 18.332);

        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = -1.0f;
        float blendT1 = 0.0f;
        int config = -1;

        robot.SetSpeed(20);
        robot.MoveCart(desc_pos1, tool, user, vel, acc, ovl, blendT, config);
        robot.MoveCart(desc_pos2, tool, user, vel, acc, ovl, blendT, config);
        robot.MoveCart(desc_pos3, tool, user, vel, acc, ovl, blendT1, config);
    }

The spline motion begins
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  The spline motion begins
    * @return  Error code
    */
    int SplineStart();

Spline motion PTP
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Joint space spline movement
    * @param  [in] joint_pos  Target joint location, unit: deg
    * @param  [in] desc_pos   Target Cartesian position
    * @param  [in] tool  Tool coordinate number, range [0~14]
    * @param  [in] user  Workpiece coordinate number, range [0~14]
    * @param  [in] vel  Percentage of speed, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] ovl  Velocity scaling factor, range[0~100]   
    * @return  Error code
    */
    int SplinePTP(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl);

The spline movement ends
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  The spline movement is complete
    * @return  Error code
    */
    int SplineEnd();

Code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnSplineMove_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        JointPos j1, j2, j3, j4;
        DescPose desc_pos1, desc_pos2, desc_pos3, desc_pos4, offset_pos;
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);

        j1 = new JointPos(-58.982, -90.717, 127.647, -129.041, -87.989, -0.062);
        desc_pos1 = new DescPose(-437.039, 411.064, 426.189, -177.886, 2.007, 31.155);

        j2 = new JointPos(-58.978, -76.817, 112.494, -127.348, -89.145, -0.063);
        desc_pos2 = new DescPose(-525.55, 562.3, 417.199, -178.325, 0.847, 31.109);

        j3 = new JointPos(-49.129, -68.49, 103.297, -128.898, -91.478, -0.062);
        desc_pos3 = new DescPose(-680.308, 547.378, 399.189, -175.909, -1.479, 40.827);

        j4 = new JointPos(-56.126, -54.093, 80.686, -121.655, -91.428, -0.064);
        desc_pos4 = new DescPose(-719.201, 790.816, 389.118, -174.939, -1.428, 33.809);

        offset_pos = new DescPose(0, 0, 0, 0, 0, 0);

        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = -1.0f;
        byte flag = 0;
        robot.SetSpeed(20);
        int err = -1;
        err = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Console.WriteLine($"movej errcode:  {err}");
                
        robot.SplineStart();
        robot.SplinePTP(j1, desc_pos1, tool, user, vel, acc, ovl);
        robot.SplinePTP(j2, desc_pos2, tool, user, vel, acc, ovl);
        robot.SplinePTP(j3, desc_pos3, tool, user, vel, acc, ovl);
        robot.SplinePTP(j4, desc_pos4, tool, user, vel, acc, ovl);
        robot.SplineEnd();
    }

The new spline motion begins
+++++++++++++++++++++++++++++++++++++++++++

.. versionchanged:: C# SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief The new spline motion begins
    * @param [in] type  0-Arc transition，1-The given point is a path point
    * @param [in] averageTime global average connection time (ms) (10 ~ ), default 2000
    * @return Error code 
    */ 
    int NewSplineStart(int type, int averageTime=2000);

Add spline motion instruction points
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Add spline motion instruction points 
    * @param [in] joint_pos Target joint location, unit: deg
    * @param [in] desc_pos Target Cartesian position 
    * @param [in] tool Tool coordinate number, range [0~14]
    * @param [in] user Workpiece coordinate number, range[0~14] 
    * @param [in] vel Percentage of speed, range [0~100] 
    * @param [in] acc Acceleration percentage, range [0~100], not open for now
    * @param [in] ovl Velocity scaling factor, range[0~100]
    * @param [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm
    * @param [in] lastFlag  Whether it's the last point，0-No，1-yes
    * @return Error code 
    */ 
    int NewSplinePoint(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int lastFlag);

New spline motion end
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief New spline motion begins
    * @return Error code 
    */ 
    int NewSplineEnd();

Termination motion
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Termination motion
    * @return  Error code
    */
    int StopMotion();

Pause motion
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Pause motion
    * @return Error code 
    */  
    int PauseMotion();

Resume motion
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Resume motion
    * @return Error code
    */ 
    int ResumeMotion();

The whole point shift begins
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  The whole point shift begins
    * @param  [in]  flag 0- offset in base coordinate system/workpiece coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos  The pose offset
    * @return  Error code
    */
    int PointsOffsetEnable(int flag, DescPose offset_pos); 

The whole point shift ends
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  The whole point shift ends
    * @return  Error code
    */
    int PointsOffsetDisable();

Code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnPointOffect_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        JointPos j1, j2;
        DescPose desc_pos1, desc_pos2, offset_pos, offset_pos1;
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);

        j1 = new JointPos(-58.982, -90.717, 127.647, -129.041, -87.989, -0.062);
        desc_pos1 = new DescPose(-437.039, 411.064, 426.189, -177.886, 2.007, 31.155);

        j2 = new JointPos(-58.978, -76.817, 112.494, -127.348, -89.145, -0.063);
        desc_pos2 = new DescPose(-525.55, 562.3, 417.199, -178.325, 0.847, 31.109);

        offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        offset_pos1 = new DescPose(50.0, 50.0, 50.0, 5.0, 5.0, 5.0);

        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = -1.0f;
        float blendR = 0.0f;
        byte flag = 0;
        int type = 0;

        robot.SetSpeed(20);

        robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Thread.Sleep(1000);
        robot.PointsOffsetEnable(type, offset_pos1);
        robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.PointsOffsetDisable();
    }
    
Set control box AO when the robot moves start
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Set control box AO when the robot moves Start
    * @param [in] AONum Control box AO num
    * @param [in] maxTCPSpeed the maximum TCP speed[1-5000mm/s]，default 1000
    * @param [in] maxAOPercent the AO percentage corresponding to the maximum TCP speed, default 100%
    * @param [in] zeroZoneCmp dead zone compensation value AO percentage, integer, default is 20, range [0-100]
    * @return error code
    */
    int MoveAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);

Set control box AO when the robot moves stop
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7
   
.. code-block:: c#
    :linenos:

    /**
    * @brief Set control box AO when the robot moves stop
    * @return error code
    */
    int MoveAOStop();
    
Set tool AO when the robot moves start
+++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7
   
.. code-block:: c#
    :linenos:

    /**
    * @brief Set tool AO when the robot moves start
    * @param [in] AONum tool AO num
    * @param [in] maxTCPSpeed the maximum TCP speed[1-5000mm/s]，default 1000
    * @param [in] maxAOPercent the AO percentage corresponding to the maximum TCP speed, default 100%
    * @param [in] zeroZoneCmp dead zone compensation value AO percentage, integer, default is 20, range [0-100]
    * @return error code
    */
    int MoveToolAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);
    
Set tool AO when the robot moves stop
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7
   
.. code-block:: c#
    :linenos:

    /**
    * @brief Set tool AO when the robot moves stop
    * @return error code
    */
    int MoveToolAOStop();

Code example
************
.. code-block:: c#
    :linenos:

    private void btnMoveAO_Click(object sender, EventArgs e)
    {
        DescPose startdescPose = new DescPose();
        JointPos startjointPos = new JointPos();
        DescPose enddescPose = new DescPose();
        JointPos endjointPos = new JointPos();
        DescPose CPose = new DescPose();
        JointPos CJPos = new JointPos();
        DescPose DPose = new DescPose();
        JointPos DJPos = new JointPos();            
        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
        int rtn = robot.MoveToolAOStart(0, 100, 80, 1);
        //int rtn = robot.MoveAOStart(0, 100, 80, 1);
        Console.WriteLine(rtn);

        rtn = robot.MoveL(startjointPos, startdescPose, 0, 0, 100, 100, 100, 0, exaxisPos, 0, 0, offdese);
        //robot.MoveJ(startjointPos, startdescPose, 0, 0, 100, 100, 100, exaxisPos, 0, 0, offdese);
        //robot.MoveC(startjointPos, startdescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, 100, 0);
        //robot.Circle(startjointPos, startdescPose, 0, 0, 100, 100, exaxisPos, endjointPos, enddescPose, 0, 0, 100, 100, exaxisPos, 100, 0, offdese);
        //robot.SplineStart();
        //robot.SplinePTP(startjointPos, startdescPose, 0, 0, 100, 100, 100);
        //robot.SplinePTP(endjointPos, enddescPose, 0, 0, 100, 100, 100);
        //robot.SplinePTP(CJPos, CPose, 0, 0, 100, 100, 100);
        //robot.SplinePTP(DJPos, DPose, 0, 0, 100, 100, 100);
        //robot.SplineEnd();

        //robot.NewSplineStart(0, 5000);
        //robot.NewSplinePoint(startjointPos, startdescPose, 0, 0, 100, 100, 100, 5, 0);
        //robot.NewSplinePoint(endjointPos, enddescPose, 0, 0, 100, 100, 100, 5, 0);
        //robot.NewSplinePoint(CJPos, CPose, 0, 0, 100, 100, 100, 5, 0);
        //robot.NewSplinePoint(DJPos, DPose, 0, 0, 100, 100, 100, 5, 1);
        //robot.NewSplineEnd();
        //int count = 1000;
        //while (count > 0)
        //{
        //    robot.ServoJ(startjointPos, 0, 0, 0.008f, 0, 0);
        //    startjointPos.jPos[0] += 0.01;//0 joint position increase
        //    count -= 1;
        //}
        rtn = robot.MoveToolAOStop();
        //rtn = robot.MoveAOStop();
        Console.WriteLine(rtn);
    }

