RobotMovement
===============================================

.. toctree:: 
    :maxdepth: 5


jog point and click
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief jog point and click 
    * @param [in] refType pointing type: 0 - joint pointing, 2 - pointing in base coordinate system, 4 - pointing in tool coordinate system, 8 - pointing in workpiece coordinate system 
    * @param [in] nb 1-joint 1 (or x-axis), 2-joint 2 (or y-axis), 3-joint 3 (or z-axis), 4-joint 4 (or rotation about x-axis), 5-joint 5 (or rotation about y-axis), 6-joint 6 (or rotation about z-axis)
    * @param [in] dir 0-negative direction, 1-positive direction 
    * @param [in] vel velocity percentage, [0~100] 
    * @param [in] acc Acceleration percentage, [0~100] 
    * @param [in] max_dis The maximum angle of a single tap in [°] or distance in [mm]. 
    * @return error code 
    */ 
    int StartJOG(byte refType, byte nb, byte dir, float vel, float acc, float max_dis);

jog tap to decelerate and stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief jog tap to decelerate and stop.
    * @param [in] ref 1-joint stopping, 3-stopping in base coordinate system, 5-stopping in tool coordinate system, 9-stopping in workpiece coordinate system
    * @return error code
    */
    int StopJOG(byte stopType).

jog tapping stops immediately
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief jogging stops instantly
    * @return error code
    */
    int ImmStopJOG(); 

code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnJOG_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2"); 

        robot.SetSpeed(35);
        robot.StartJOG(0, 1, 0, 15, 20.0f, 30.0f); //single-joint motion, StartJOG is a non-blocking command, receiving other motion commands (including StartJOG) in the motion state will be discarded
        Thread.Sleep(1000);
        robot.StopJOG(1); //Robot single-axis tap deceleration stops
        //robot.ImmStopJOG(); //robot single-axis pointing stops immediately
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

        robot.StartJOG(2, 1, 0, 15, 20.0f, 30.0f); //point motion in base coordinate system
        Thread.Sleep(1000);
        robot.StopJOG(3); //Robot single-axis pointing deceleration stops
        //robot.ImmStopJOG(); //robot single-axis pointing stops immediately
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

        robot.StartJOG(4, 1, 0, 15, 20.0f, 30.0f); //point motion in tool coordinate system
        Thread.Sleep(1000);
        robot.StopJOG(5); //Robot single-axis pointing deceleration stops
        //robot.ImmStopJOG(); //robot single-axis pointing stops immediately
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

        robot.StartJOG(8, 1, 0, 15, 20.0f, 30.0f); //point motion in the workpiece coordinate system
        Thread.Sleep(1000);
        robot.StopJOG(9); //Robot single-axis pointing deceleration stops
        //robot.ImmStopJOG(); //robot single-axis pointing stops immediately
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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Joint space movement
    * @param [in] joint_pos Target joint position, in deg.
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool tool coordinate number in the range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] epos Extended axis position in mm
    * @param [in] blendT [-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in ms
    * @param [in] offset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos Bit position offset
    * @return error code
    */
    int MoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos epos, float blendT, byte offset_flag, DescPose offset_pos); 

Cartesian linear motion in space
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Cartesian space linear motion
    * @param [in] joint_pos Target joint position in deg.
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool tool coordinate number in the range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm  
    * @param [in] epos Extended axis position in mm
    * @param [in] search 0-no wire seek, 1-wire seek
    * @param [in] offset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos Bit position offset
    * @param [in] overSpeedStrategy Overspeed handling strategy, 1-standard; 2-over speed error stop; 3-adaptive speed reduction, default 0
    * @param [in] speedPercent Percentage of allowed speed reduction threshold [0-100], default 10%
    * @return error code
    */  
    int MoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, byte search, byte offset_flag, DescPose offset_pos, int overSpeedStrategy = 0, int speedPercent = 10); 

Circular motion in Cartesian space
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Cartesian space circular motion
    * @param [in] joint_pos_p Path point joint position in deg.
    * @param [in] desc_pos_p Path point Cartesian pose
    * @param [in] ptool tool coordinate number, range [0~14]
    * @param [in] puser Workpiece coordinate number, range [0~14].
    * @param [in] pvel speed percentage, range [0~100]
    * @param [in] pacc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos_p Extended axis position in mm
    * @param [in] poffset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos_p bit position offset
    * @param [in] joint_pos_t Joint position of target point, in deg.
    * @param [in] desc_pos_t Target point Cartesian pose
    * @param [in] ttool tool coordinate number, in the range [0 to 14].
    * @param [in] tuser Workpiece coordinate number, range [0~14].
    * @param [in] tvel speed percentage, range [0~100]
    * @param [in] tacc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos_t Extended axis position in mm
    * @param [in] toffset_flag 0 - no offset, 1 - offset in base/work coordinate system, 2 - offset in tool coordinate system
    * @param [in] offset_pos_t bit position offset   
    * @param [in] ovl velocity scaling factor, range [0~100]    
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm  
    * @return error code
    */  
    int MoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos epos_p, byte poffset_flag, DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos epos_t, byte toffset_flag, DescPose offset_pos_t, float ovl, float blendR). 

Whole circle motion in Cartesian space
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Cartesian space rectilinear motion
    * @param [in] joint_pos_p Path point 1 joint position in deg.
    * @param [in] desc_pos_p Path point 1 Cartesian pose
    * @param [in] ptool tool coordinate number, range [0~14]
    * @param [in] puser Workpiece coordinate number, range [0~14].
    * @param [in] pvel speed percentage, range [0~100]
    * @param [in] pacc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos_p Extended axis position in mm
    * @param [in] joint_pos_t Path point 2 joint position in deg.
    * @param [in] desc_pos_t Path point 2 Cartesian pose
    * @param [in] ttool tool coordinate number, in the range [0 to 14].
    * @param [in] tuser Workpiece coordinate number, range [0~14].
    * @param [in] tvel speed percentage, range [0~100]
    * @param [in] tacc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos_t Extended axis position in mm
    * @param [in] ovl velocity scaling factor, range [0~100]   
    * @param [in] offset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos Bit position offset     
    * @return error code
    */  
    int Circle(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos epos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos epos_t, float ovl, byte offset_flag, DescPose offset_pos).

code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnMovetest_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2"); 

        JointPos j1, j2, j3, j4.
        DescPose desc_pos1, desc_pos2, desc_pos3, desc_pos4, offset_pos;
        ExaxisPos epos.

        j1 = new JointPos(-58.982, -90.717, 127.647, -129.041, -87.989, -0.062);
        desc_pos1 = new DescPose(-437.039, 411.064, 426.189, -177.886, 2.007, 31.155);

        j2 = new JointPos(-58.978, -76.817, 112.494, -127.348, -89.145, -0.063);
        desc_pos2 = new DescPose(-525.55, 562.3, 417.199, -178.325, 0.847, 31.109);

        j3 = new JointPos(-71.746, -87.177, 123.953, -126.25, -89.429, -0.089);
        desc_pos3 = new DescPose(-345.155, 535.733, 421.269, 179.475, 0.571, 18.332);

        ExaxisPos ePos = new ExaxisPos(0, 0, 0, 0, 0);
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
        Console.WriteLine($"movej errcode: {err}");

        Thread.Sleep(1000);
        err = robot.MoveL(j2, desc_pos2, tool, user, vel, acc, ovl, blendR, ePos, search, flag, offset);
        Console.WriteLine($"moveL errcode: {err}");

        Thread.Sleep(1000);
        err = robot.MoveL(j1, desc_pos1, tool, user, vel, acc, ovl, blendR, ePos, search, flag, offset);
        Console.WriteLine($"moveL errcode: {err}");

        Thread.Sleep(1000);
        err = robot.MoveC(j2, desc_pos2, tool, user, vel, acc, ePos, flag, offset, j3, desc_pos3, tool, user, vel, acc, ePos, flag, offset, ovl, blendR);
        Console.WriteLine($"circle errcode: {err}");

        Thread.Sleep(1000);
        err = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, ePos, blendT, flag, offset);
        Console.WriteLine($"movej errcode: {err}");

        Thread.Sleep(1000);
        err = robot.Circle(j2, desc_pos2, tool, user, vel, acc, ePos, j3, desc_pos3, tool, user, vel, acc, ePos, ovl, flag, offset);
        Console.WriteLine($"circle errcode: {err}");
    }

Spiral motion in Cartesian space
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Cartesian space spiral motion 
    * @param [in] joint_pos Target joint position, in deg 
    * @param [in] desc_pos Target Cartesian pose 
    * @param [in] tool tool coordinate number in the range [0~14]. 
    * @param [in] user Workpiece coordinate number, range [0~14]. 
    * @param [in] vel velocity percentage, range [0~100] 
    * @param [in] acc Acceleration percentage, range [0~100], not open yet. 
    * @param [in] epos Extended axis position in mm 
    * @param [in] ovl velocity scaling factor, range [0~100] 
    * @param [in] offset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system 
    * @param [in] offset_pos Bit position offset 
    * @param [in] spiral_param spiral_param 
    * @return error code 
    */
    int NewSpiral(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, ExaxisPos epos, float ovl, byte offset_flag, DescPose offset_pos, SpiralParam spiral_param); 

code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnDescSpiral_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");
        JointPos j.
        DescPose desc_pos.
        DescPose offset_pos1 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        DescPose offset_pos2 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        SpiralParam sp.

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
        int ret = robot.GetForwardKin(j, ref desc_pos); // with only joint positions, use the positive kinematics interface to solve for Cartesian space coordinates
        if (ret == 0)
        {
            int err = -1;
            err = robot.MoveJ(j, desc_pos, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos1);
            Console.WriteLine($"movej errcode: {err}");

            err = robot.NewSpiral(j, desc_pos, tool, user, vel, acc, epos, ovl, flag, offset_pos2, sp);
            Console.WriteLine($"newspiral errcode: {err}");
        }
        else
        {
            Console.WriteLine($"GetForwardKin errcode: {ret}");
        }
    }

Start of servo motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Servo motion start, used in conjunction with ServoJ, ServoCart instruction
    * @return error code 
    */ 
    int ServoMoveStart();

End of servo motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief End of servo motion, used in conjunction with ServoJ, ServoCart commands
    * @return error code 
    */ 
    int ServoMoveEnd().

Joint space servo mode motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Joint space servo mode motion
    * @param [in] joint_pos Target joint position, in deg.
    * @param [in] acc Acceleration percentage, range [0~100], not open yet, default is 0.
    * @param [in] vel speed percentage, range [0~100], not open yet, default 0
    * @param [in] cmdT command send cycle, unit s, recommended range [0.001~0.0016].
    * @param [in] filterT filter time, unit s, not open, default 0
    * @param [in] gain Proportional amplifier for target position, not open yet, default 0
    * @return error code
    */
    int ServoJ(JointPos joint_pos, float acc, float vel, float cmdT, float filterT, float gain);

code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnJointServoMove_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        JointPos j = new JointPos(0, 0, 0, 0, 0, 0, 0);

        float vel = 0.0f;
        float acc = 0.0f;
        float cmdT = 0.008f;
        float filterT = 0.0f;
        float gain = 0.0f;
        byte flag = 0;
        int count = 200.
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
            Console.WriteLine($"GetActualJointPosDegree errcode: {ret}");
        }
    }

Servo-mode motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Servo-mode motion in Cartesian space
    * @param [in] mode 0-absolute motion (base coordinate system), 1-incremental motion (base coordinate system), 2-incremental motion (tool coordinate system)
    * @param [in] desc_pos target Cartesian position or position increment
    * @param [in] pos_gain Positional incremental scale factor, valid only for incremental motion, range [0~1].
    * @param [in] acc Acceleration percentage, range [0~100], not open yet, default is 0.
    * @param [in] vel speed percentage, range [0~100], not open yet, default 0
    * @param [in] cmdT command send cycle, unit s, recommended range [0.001~0.0016].
    * @param [in] filterT filter time, unit s, not open, default 0
    * @param [in] gain Proportional amplifier for target position, not open yet, default 0
    * @return error code
    */
    int ServoCart(int mode, DescPose desc_pos, double[] pos_gain, float acc, float vel, float cmdT, float filterT, float gain);

code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnDescServoMove_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        DescPose desc_pos_dt = new DescPose(0, 0, 0, 0, 0, 0, 0);
        desc_pos_dt.tran.z = -0.5;
        double[] pos_gain = new double[6]{ 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
        int mode = 2;
        float vel = 0.0f;
        float acc = 0.0f;
        float cmdT = 0.008f;
        float filterT = 0.0f;
        float gain = 0.0f;
        int flag = 0;
        int count = 500.

        robot.SetSpeed(20);
        while (count > 0)
        {
            robot.ServoCart(mode, desc_pos_dt, pos_gain, acc, vel, cmdT, filterT, gain);
            count -= 1;
            robot.WaitMs((int)(cmdT * 1000));
        }
    }

Point-to-point motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Cartesian space point-to-point motion 
    * @param [in] desc_pos Cartesian position of the target in the base coordinate system. 
    * @param [in] tool tool coordinate number in the range [0~14]. 
    * @param [in] user Workpiece coordinate number, range [0~14]. 
    * @param [in] vel velocity percentage, range [0~100] 
    * @param [in] acc Acceleration percentage, range [0~100], not open yet. 
    * @param [in] ovl velocity scaling factor, range [0~100] 
    * @param [in] blendT [-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in ms 
    * @param [in] config Joint space configuration, [-1] - solve with reference to current joint position, [0~7] - solve with reference to specific joint space configuration, default is -1. 
    * @return error code 
    */ 
    int MoveCart(DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendT, int config);

code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnDescPTPMove_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        DescPose desc_pos1, desc_pos2, desc_pos3.
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

Start of spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Spline motion begins
    * @return error code
    */
    int SplineStart();

Sample motion PTP
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Joint space spline motion
    * @param [in] joint_pos Target joint position, in deg.
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool tool coordinate number in the range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0~100]   
    * @return error code
    */
    int SplinePTP(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl);

End of spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief End of spline motion
    * @return error code
    */
    int SplineEnd(); 

code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnSplineMove_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        JointPos j1, j2, j3, j4.
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

        offset_pos = new DescPose(0, 0, 0, 0, 0, 0, 0);

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
        Console.WriteLine($"movej errcode: {err}");
                
        robot.SplineStart();
        robot.SplinePTP(j1, desc_pos1, tool, user, vel, acc, ovl);
        robot.SplinePTP(j2, desc_pos2, tool, user, vel, acc, ovl);
        robot.SplinePTP(j3, desc_pos3, tool, user, vel, acc, ovl);
        robot.SplinePTP(j4, desc_pos4, tool, user, vel, acc, ovl);
        robot.SplineEnd();
    }

New Sample Campaign Begins
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief New spline campaign begins 
    * @param [in] type 0-circular transition, 1-given point position as path point
    * @param [in] averageTime global average articulation time (ms) (10 ~ ), default 2000
    * @return error code 
    */ 
    int NewSplineStart(int type, int averageTime=2000);
    
new spline command point
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Adding spline motion command points 
    * @param [in] joint_pos Target joint position, in deg 
    * @param [in] desc_pos Target Cartesian pose 
    * @param [in] tool tool coordinate number in the range [0~14]. 
    * @param [in] user Workpiece coordinate number, range [0~14]. 
    * @param [in] vel velocity percentage, range [0~100] 
    * @param [in] acc Acceleration percentage, range [0~100], not open yet. 
    * @param [in] ovl velocity scaling factor, range [0~100] 
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm
    * @param [in] lastFlag whether it is the last point, 0-no, 1-yes
    * @return error code 
    */ 
    int NewSplinePoint(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int lastFlag);;

End of new spline movement
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief New spline campaign begins 
    * @return error code 
    */ 
    int NewSplineEnd();
    
Ending the campaign
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief end of campaign
    * @return error code
    */
    int StopMotion();

pause
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
    
    /** 
      * @brief Suspension of movement 
      * @return error code 
    */  
    int PauseMotion();

Resumption of movement
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Recovery Movement 
    * @return error code 
    */ 
    int ResumeMotion();

Overall shift in points begins
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Points of overall offset start
    * @param [in] flag 0 - offset in base/workpiece coordinate system, 2 - offset in tool coordinate system
    * @param [in] offset_pos Bit position offset
    * @return error code
    */
    int PointsOffsetEnable(int flag, DescPose offset_pos); 


Overall point shift ends
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief End of overall point shift
    * @return error code
    */
    int PointsOffsetDisable(); 

code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnPointOffect_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        JointPos j1, j2.
        DescPose desc_pos1, desc_pos2, offset_pos, offset_pos1;
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);

        j1 = new JointPos(-58.982, -90.717, 127.647, -129.041, -87.989, -0.062);
        desc_pos1 = new DescPose(-437.039, 411.064, 426.189, -177.886, 2.007, 31.155);

        j2 = new JointPos(-58.978, -76.817, 112.494, -127.348, -89.145, -0.063);
        desc_pos2 = new DescPose(-525.55, 562.3, 417.199, -178.325, 0.847, 31.109);

        offset_pos = new DescPose(0, 0, 0, 0, 0, 0, 0);
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

Control box AO fly shooting start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief control box AO fly shooting start
    * @param [in] AONum Control box AO number
    * @param [in] maxTCPSpeed maxTCPSpeed value [1-5000mm/s], default 1000
    * @param [in] maxAOPercent The percentage of AO corresponding to the maximum TCP speed value, default 100%.
    * @param [in] zeroZoneCmp dead zone compensation value AO percentage, shaped, default 20%, range [0-100]
    * @return error code
    */
    int MoveAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);

Control Box AO Flying Racket Stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7
   
.. code-block:: c#
    :linenos:

    /**
    * @brief Control box AO flyswatter stops
    * @return error code
    */
    int MoveAOStop();
    
End AO Fly Shot Begins
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7
   
.. code-block:: c#
    :linenos:

    /**
    * @brief End of AO Fly Shot Begins *
    * @param [in] AONum End AO number
    * @param [in] maxTCPSpeed maxTCPSpeed value [1-5000mm/s], default 1000
    * @param [in] maxAOPercent The percentage of AO corresponding to the maximum TCP speed value, default 100%.
    * @param [in] zeroZoneCmp dead zone compensation value AO percentage, shaped, default 20%, range [0-100]
    * @return error code
    */
    int MoveToolAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);
    
End AO Flying Racket Stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7
   
.. code-block:: c#
    :linenos:

    /**
    * @brief End AO fly-tap stops
    * @return error code
    */
    int MoveToolAOStop();

code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0, 0);
        int rtn = robot.MoveToolAOStart(0, 100, 80, 1);
        //int rtn = robot.MoveAOStart(0, 100, 80, 1);
        Console.WriteLine(rtn);

        rtn = robot.MoveL(startjointPos, startdescPose, 0, 0, 100, 100, 100, 100, 0, exaxisPos, 0, 0, offdese);
        //robot.MoveJ(startjointPos, startdescPose, 0, 0, 100, 100, 100, exaxisPos, 0, 0, offdese);
        //robot.MoveC(startjointPos, startdescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, 100, 0);
        //robot.Circle(startjointPos, startdescPose, 0, 0, 100, 100, exaxisPos, endjointPos, enddescPose, 0, 0, 100, 100, 100, exaxisPos, 100, 0, offdese);
        //robot.SplineStart();
        //robot.SplinePTP(startjointPos, startdescPose, 0, 0, 100, 100, 100);
        //robot.SplinePTP(endjointPos, enddescPose, 0, 0, 100, 100, 100);
        //robot.SplinePTP(CJPos, CPose, 0, 0, 100, 100, 100);
        //robot.SplinePTP(DJPos, DPose, 0, 0, 100, 100, 100);
        //robot.SplineEnd();

        //robot.NewSplineStart(0, 5000);
        //robot.NewSplinePoint(startjointPos, startdescPose, 0, 0, 100, 100, 100, 100, 5, 0);
        //robot.NewSplinePoint(endjointPos, enddescPose, 0, 0, 100, 100, 100, 100, 5, 0);
        //robot.NewSplinePoint(CJPos, CPose, 0, 0, 100, 100, 100, 100, 5, 0);
        //robot.NewSplinePoint(DJPos, DPose, 0, 0, 100, 100, 100, 100, 5, 1);
        //robot.NewSplineEnd();
        //int count = 1000;
        //while (count > 0)
        //{
        // robot.ServoJ(startjointPos, 0, 0, 0.008f, 0, 0);
        // startjointPos.jPos[0] += 0.01;//0 joint position increase
        // count -= 1;
        //}
        rtn = robot.MoveToolAOStop();
        //rtn = robot.MoveAOStop();
        Console.WriteLine(rtn);
    }

Starting odd position protection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * :: @brief begins odd-position protection.
    * @param [in] protectMode singular protection mode, 0: articulated mode; 1 - Cartesian mode
    * @param [in] minShoulderPos Shoulder singularity adjustment range (mm), default 100
    * @param [in] minElbowPos elbow singularity adjustment range (mm), default 50
    * @param [in] minWristPos wrist singularity adjustment range (°), default 10
    * @return error code
    */
    int SingularAvoidStart(int protectMode, double minShoulderPos, double minElbowPos, double minWristPos);;

Stop odd position protection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * :: @brief stops odd-position protection.
    * @return error code
    */
    int SingularAvoidEnd().

code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9
    
.. code-block:: c#
    :linenos:

    private void btnTestSingularAvoidEArc_Click(object sender, EventArgs e)
    {
        DescPose startdescPose = new DescPose(-352.437, -88.350, 226.471, 177.222, 4.924, 86.631);
        JointPos startjointPos = new JointPos(-3.463, -84.308, 105.579, -108.475, -85.087, -0.334);

        DescPose middescPose = new DescPose(-518.339, -23.706, 207.899, -178.420, 0.171, 71.697);
        JointPos midjointPos = new JointPos(-8.587, -51.805, 64.914, -104.695, -90.099, 9.718);

        DescPose enddescPose = new DescPose(-273.934, 323.003, 227.224, 176.398, 2.783, 66.064);
        JointPos endjointPos = new JointPos(-63.460, -71.228, 88.068, -102.291, -90.149, -39.605);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);

        robot.MoveL(startjointPos, startdescPose, 0, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        robot.SingularAvoidStart(1, 100, 50, 10);
        robot.MoveC(midjointPos, middescPose, 0, 0, 50, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, 100, -1) ;
        robot.SingularAvoidEnd();
    }