robotics
=====================================

.. toctree:: 
    :maxdepth: 5

jog point and click
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief jog point and click 
    * @param [in] refType 0 - joint pointing, 2 - pointing in base coordinate system, 4 - pointing in tool coordinate system, 8 - pointing in workpiece coordinate system
    * @param [in] nb 1-joint 1 (or x-axis), 2-joint 2 (or y-axis), 3-joint 3 (or z-axis), 4-joint 4 (or rotation around x-axis), 5-joint 5 (or rotation around y-axis), 6-joint 6 (or rotation around z-axis)
    * @param [in] dir 0-negative direction, 1-positive direction
    * @param [in] vel velocity percentage, [0~100]
    * @param [in] acc Acceleration percentage, [0~100]
    * @param [in] max_dis The maximum angle of a single tap in [°] or distance in [mm].
    * @return error code 
    */ 
    int StartJOG(int refType, int nb, int dir, double vel, double acc, double max_dis);

jog tap to decelerate and stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief jog jog jog jog jog jog jog jog jog jog jog jog jog jog jog stop
    * @param [in] stopType 1-joint stop, 3-base coordinate system stop, 5-tool coordinate system stop, 9-workpiece coordinate system stop
    * @return error code
    */
    int StopJOG(int stopType).

Immediate stop for jog taps
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief jogging stops instantly
    * @return error code
    */
    int ImmStopJOG(); 

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
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
        robot.StartJOG(0, 1, 0, 30, 100, 90);//joint point movement
        robot.Sleep(3000);
        robot.StopJOG(1);//Tap to decelerate and stop
        robot.StartJOG(0, 1, 0, 30, 100, 90);//joint point movement
        robot.Sleep(3000);
        robot.ImmStopJOG();//tap to stop immediately
    } 

Joint space motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Joint space movement
    * @param [in] joint_pos Target joint position, in deg.
    * @param [in] desc_pos target Cartesian position
    * @param [in] tool tool coordinate number, range [0~14].
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
    int MoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, ExaxisPos epos, double blendT, int offset_ flag, DescPose offset_pos);

Cartesian linear motion in space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Cartesian space linear motion
    * @param [in] joint_pos Target joint position, in deg.
    * @param [in] desc_pos target Cartesian position
    * @param [in] tool tool coordinate number, range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0 to 100].
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm
    * @param [in] epos Extended axis position in mm
    * @param [in] search 0-no wire seek, 1-wire seek
    * @param [in] offset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos Bit position offset
    * @param [in] overSpeedStrategy Overspeed handling strategy, 1-standard; 2-over speed error stop; 3-adaptive speed reduction, default is 0
    * @param [in] speedPercent Percentage of allowable speed reduction threshold [0-100], default 10%
    * @return error code
    */  
    int MoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendR, ExaxisPos epos, int search, int offset_flag, DescPose offset_pos, int overSpeedStrategy, int speedPercent).

Circular motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Cartesian space circular motion
    * @param [in] joint_pos_p Path point joint position in deg.
    * @param [in] desc_pos_p Path point Cartesian pose
    * @param [in] ptool tool coordinate number, in the range [0~14].
    * @param [in] puser Workpiece coordinate number, range [0~14].
    * @param [in] pvel speed percentage, range [0~100]
    * @param [in] pacc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos_p Extended axis position in mm
    * @param [in] poffset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos_p bit position offset
    * @param [in] joint_pos_t Joint position of target point, in deg.
    * @param [in] desc_pos_t Destination point Cartesian pose
    * @param [in] ttool tool coordinate number, in the range [0~14].
    * @param [in] tuser Workpiece coordinate number, range [0~14].
    * @param [in] tvel speed percentage, range [0~100]
    * @param [in] tacc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos_t Extended axis position in mm
    * @param [in] toffset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos_t bit position offset
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm 
    * @return error code
    */  
    int MoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, double pvel, double pacc, ExaxisPos epos_p, int poffset_flag, DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, double tvel, double tacc, ExaxisPos epos_t, int toffset_flag, DescPose offset_pos_t, double ovl, double blendR).

Whole circle motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Cartesian spatial rectilinear motion
    * @param [in] joint_pos_p Path point 1 joint position in deg.
    * @param [in] desc_pos_p Path point 1 Cartesian poses
    * @param [in] ptool tool coordinate number, in the range [0~14].
    * @param [in] puser Workpiece coordinate number, range [0~14].
    * @param [in] pvel speed percentage, range [0~100]
    * @param [in] pacc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos_p Extended axis position in mm
    * @param [in] joint_pos_t Path point 2 joint position in deg.
    * @param [in] desc_pos_t Path point 2 Cartesian poses
    * @param [in] ttool tool coordinate number, in the range [0~14].
    * @param [in] tuser Workpiece coordinate number, range [0~14].
    * @param [in] tvel speed percentage, range [0~100]
    * @param [in] tacc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos_t Extended axis position in mm
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] offset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos Bit position offset
    * @return error code
    */  
    int Circle(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, double pvel, double pacc, ExaxisPos epos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, double tvel, double tacc, ExaxisPos epos_t, double ovl, int offset_flag, DescPose offset_pos);

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection successful");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        DescPose middescPoseCir1=new DescPose(-435.414,-342.926,309.205,-171.382,-4.513,171.520);

        JointPos midjointPosCir1=new JointPos(26.804,-79.866,106.642,-125.433,-85.562,-54.721);

        DescPose enddescPoseCir1=new DescPose(-524.862,-217.402,308.459,-171.425,-4.810,156.088);

        JointPos endjointPosCir1=new JointPos(11.399,-78.055,104.603,-125.421,-85.770,-54.721);

        DescPose middescPoseCir2=new DescPose(-482.691,-587.899,318.594,-171.001,-4.999,-172.996);

        JointPos midjointPosCir2=new JointPos(42.314,-53.600,67.296,-112.969,-85.533,-54.721);

        DescPose enddescPoseCir2=new DescPose(-403.942,-489.061,317.038,-163.189,-10.425,-175.627);

        JointPos endjointPosCir2=new JointPos(39.959,-70.616,96.679,-134.243,-82.276,-54.721);

        DescPose middescPoseMoveC=new DescPose(-435.414,-342.926,309.205,-171.382,-4.513,171.520);

        JointPos midjointPosMoveC=new JointPos(26.804,-79.866,106.642,-125.433,-85.562,-54.721);

        DescPose enddescPoseMoveC=new DescPose(-524.862,-217.402,308.459,-171.425,-4.810,156.088);

        JointPos endjointPosmoveC=new JointPos(11.399,-78.055,104.603,-125.421,-85.770,-54.721);

        DescPose middescPoseCir3=new DescPose(-435.414,-342.926,309.205,-171.382,-4.513,171.520);

        JointPos midjointPosCir3=new JointPos(26.804,-79.866,106.642,-125.433,-85.562,-54.721);

        DescPose enddescPoseCir3=new DescPose(-569.505,-405.378,357.596,-172.862,-10.939,171.108);

        JointPos endjointPosCir3=new JointPos(27.138,-63.750,78.586,-117.861,-90.588,-54.721);

        DescPose middescPoseCir4=new DescPose(-482.691,-587.899,318.594,-171.001,-4.999,-172.996);

        JointPos midjointPosCir4=new JointPos(42.314,-53.600,67.296,-112.969,-85.533,-54.721);

        DescPose enddescPoseCir4=new DescPose(-569.505,-405.378,357.596,-172.862,-10.939,171.108);

        JointPos endjointPosCir4=new JointPos(27.138,-63.750,78.586,-117.861,-90.588,-54.721);

        DescPose startdescPose =new DescPose(-569.505, -405.378, 357.596, -172.862, -10.939, 171.108);
        JointPos startjointPos = new JointPos(27.138, -63.750, 78.586, -117.861, -90.588, -54.721);

        DescPose linedescPose =new DescPose(-403.942, -489.061, 317.038, -163.189, -10.425, -175.627);
        JointPos linejointPos = new JointPos(39.959, -70.616, 96.679, -134.243, -82.276, -54.721);

        ExaxisPos exaxisPos =new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

        robot.MoveJ(startjointPos,startdescPose,3,0,100,100,100,exaxisPos,-1,0,offdese);
        robot.Circle(midjointPosCir1,middescPoseCir1,3,0,100,100,exaxisPos,endjointPosCir1,enddescPoseCir1,3,0,100,100,exaxisPos,100,-1,offdese);
        robot.Circle(midjointPosCir2,middescPoseCir2,3,0,100,100,exaxisPos,endjointPosCir2,enddescPoseCir2,3,0,100,100,exaxisPos,100,-1,offdese);
        robot.MoveC(midjointPosMoveC,middescPoseMoveC,3,0,100,100,exaxisPos,0,offdese,endjointPosmoveC,enddescPoseMoveC,3,0,100,100,exaxisPos,0,offdese,100,20);
        robot.Circle(midjointPosCir3,middescPoseCir3,3,0,100,100,exaxisPos,endjointPosCir3,enddescPoseCir3,3,0,100,100,exaxisPos,100,-1,offdese);
        robot.MoveL(linejointPos,linedescPose,3,0,100,100,100,-1,0,exaxisPos,0,0,offdese,0,10);
        robot.Circle(midjointPosCir4,middescPoseCir4,3,0,100,100,exaxisPos,endjointPosCir4,enddescPoseCir4,3,0,100,100,exaxisPos,100,-1,offdese);
    } 

Spiral motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Cartesian space spiral motion 
    * @param [in] joint_pos Target joint position, in deg.
    * @param [in] desc_pos target Cartesian position
    * @param [in] tool tool coordinate number, range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos Extended axis position in mm
    * @param [in] ovl velocity scaling factor, range [0 to 100].
    * @param [in] offset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos Bit position offset
    * @return error code 
    */
    int NewSpiral(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, ExaxisPos epos, double ovl, int offset_flag, DescPose offset_pos, SpiralParam spiral_param);

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
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
        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();
        JointPos JP1=new JointPos(117.408,-86.777,81.499,-87.788,-92.964,92.959);
        DescPose DP1 =new DescPose(327.359,-420.973,518.377,-177.199,3.209,114.449);
        SpiralParam param = new SpiralParam(5,10.0,30.0,10.0,5.0,0);//spiral
        robot.NewSpiral(JP1, DP1, 0, 0, 50, 100, epos, 100, 0, offset_pos, param);
    }

Start of servo motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Servo motion start, used in conjunction with ServoJ, ServoCart instruction
    * @return error code 
    */ 
    int ServoMoveStart();

End of servo motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief End of servo motion, used in conjunction with ServoJ, ServoCart commands
    * @return error code 
    */ 
    int ServoMoveEnd();

Joint space servo mode motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Joint space servo mode motion
    * @param [in] joint_pos Target joint position, in deg.
    * @param [in] axisPos External axis position in mm.
    * @param [in] acc Acceleration percentage, range [0~100], not open, default is 0.
    * @param [in] vel speed percentage, range [0~100], not open yet, default 0
    * @param [in] cmdT Command down cycle, unit s, recommended range [0.001~0.0016]
    * @param [in] filterT Filter time in s, not open, default is 0.
    * @param [in] gain Proportional amplifier for target position, not open yet, default is 0
    * @return error code
    */
    int ServoJ(JointPos joint_pos, ExaxisPos axisPos, double acc, double vel, double cmdT, double filterT, double gain);

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
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
            int count = 200.
            while (count > 0)
            {
                robot.ServoJ(j5, ePos,100, 100, 0.008, 0, 0);
                j5.J1 += 0.2;//1 joint position increase
                count -= 1;
                robot.WaitMs((int)(8));
            }
        }
    }


Servo-mode motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Servo-mode motion in Cartesian space
    * @param [in] mode 0-absolute motion (base coordinate system), 1-incremental motion (base coordinate system), 2-incremental motion (tool coordinate system)
    * @param [in] desc_pose target Cartesian pose or pose increment
    * @param [in] pos_gain Positional incremental scale factor, valid only for incremental motion, range [0~1].
    * @param [in] acc Acceleration percentage, range [0~100], not open, default is 0.
    * @param [in] vel speed percentage, range [0~100], not open yet, default 0
    * @param [in] cmdT Command down cycle, unit s, recommended range [0.001~0.0016].
    * @param [in] filterT Filter time in s, not open, default is 0.
    * @param [in] gain Proportional amplifier for target position, not open yet, default is 0
    * @return error code
    */
    int ServoCart(int mode, DescPose desc_pose, Object[] pos_gain, double acc, double vel, double cmdT, double filterT, double gain);

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
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
        DescPose desc_pos_dt = new DescPose(0, 0, 0, 0, 0, 0, 0);
        desc_pos_dt.tran.z = -0.5;
        Object[] pos_gain = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };// z-axis gain only
        int mode = 2;//incremental motion in tool coordinate system
        float vel = 0.0f;
        float acc = 0.0f;
        float cmdT = 0.008f;
        float filterT = 0.0f;
        float gain = 0.0f;
        int count = 200.

        robot.SetSpeed(20);

        while (count > 0)
        {
            robot.ServoCart(mode, desc_pos_dt, pos_gain, acc, vel, cmdT, filterT, gain);
            count -= 1;
            robot.WaitMs((int)(cmdT * 1000));
        }
    }

Point-to-point motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Cartesian space point-to-point motion 
    * @param [in] desc_pos target Cartesian position or position increment
    * @param [in] tool tool coordinate number, range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] blendT [-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in ms
    * @param [in] config Joint space configuration, [-1] - solve with reference to current joint position, [0~7] - solve with reference to specific joint space configuration, default is -1.
    * @return error code 
    */ 
    int MoveCart(DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendT, int config); int MoveCart(DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendT, int config).

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
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
        DescPose DP2=new DescPose(-63.512,-529.698,517.946,-178.192,3.07,69.554);
        robot.MoveCart(DP2, 0, 0, 30.0, 100.0, 100.0, -1.0, -1);
    }

Start of spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Spline motion begins
    * @return error code
    */
    int SplineStart();

Joint space spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Joint space spline motion
    * @param [in] joint_pos Target joint position, in deg.
    * @param [in] desc_pos target Cartesian position
    * @param [in] tool tool coordinate number, range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @return error code
    */
    int SplinePTP(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl);

End of spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief End of spline movement
    * @return error code
    */
    int SplineEnd(). 

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
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
        DescPose desc_p1, desc_p2, desc_p3, desc_p4;//Cartesian spatial position and pose
        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);
        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        desc_p3 = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);
        desc_p4 = new DescPose(0, 0, 0, 0, 0, 0, 0);

        desc_p1.tran.x = -104.846;
        desc_p1.tran.y = 309.573;
        desc_p1.tran.z = 336.647;
        desc_p1.rpy.rx = 179.681;
        desc_p1.rpy.ry = -0.419;
        desc_p1.rpy.rz = -92.692;

        desc_p2.tran.x = -318.287;
        desc_p2.tran.y = 158.502;
        desc_p2.tran.z = 346.184;
        desc_p2.rpy.rx = 179.602;
        desc_p2.rpy.ry = 1.081;
        desc_p2.rpy.rz = -46.342;

        desc_p3.tran.x = -352.414;
        desc_p3.tran.y = 24.059;
        desc_p3.tran.z = 395.376;
        desc_p3.rpy.rx = 179.755;
        desc_p3.rpy.ry = -1.045;
        desc_p3.rpy.rz = -23.877;

        desc_p4.tran.x = 195.474;
        desc_p4.tran.y = 423.278;
        desc_p4.tran.z = 228.509;
        desc_p4.rpy.rx = -179.199;
        desc_p4.rpy.ry = -0.567;
        desc_p4.rpy.rz = -130.209;

        JointPos j1 = new JointPos();
        JointPos j2 = new JointPos();
        JointPos j3 = new JointPos();
        JointPos j4 = new JointPos();
        robot.GetInverseKin(0, desc_p1, -1, j1);//inverse kinematic solution
        robot.GetInverseKin(0, desc_p2, -1, j2);
        robot.GetInverseKin(0, desc_p3, -1, j3);
        robot.GetInverseKin(0, desc_p4, -1, j4);
        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();
        robot.MoveJ(j1, desc_p1,4, 0, 100, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.SplineStart();
        robot.SplinePTP(j4, desc_p4, 0, 0, 100, 100, 100);
        robot.SplinePTP(j1, desc_p1, 0, 0, 100, 100, 100);
        robot.SplinePTP(j2, desc_p2, 0, 0, 100, 100, 100);
        robot.SplinePTP(j3, desc_p3, 0, 0, 100, 100, 100);
        robot.SplineEnd();
    }

New spline movement begins
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief New spline campaign begins 
    * @param [in] type 0-circular transition, 1-given point position as a path point
    * @param [in] averageTime global average time (ms) (10 ~ ), default 2000
    * @return error code 
    */ 
    int NewSplineStart(int type, int averageTime).
    
new spline command point
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Adding spline motion command points 
    * @param [in] joint_pos Target joint position, in deg.
    * @param [in] desc_pos target Cartesian position
    * @param [in] tool tool coordinate number, range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0 to 100].
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm
    * @param [in] lastFlag whether it is the last point, 0 - no, 1 - yes
    * @return error code 
    */ 
    int NewSplinePoint(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendR, int lastFlag);;

End of new spline movement
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief New spline campaign begins 
    * @return error code 
    */ 
    int NewSplineEnd();
    
Termination campaigns
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief end of campaign
    * @return error code
    */
    int StopMotion().

pause
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:
    
    /** 
    * @brief pause campaign 
    * @return error code 
    */  
    int PauseMotion().

Resumption of movement
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Recovery Movement 
    * @return error code 
    */ 
    int ResumeMotion().

Overall shift in points begins
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Points of overall offset started
    * @param [in] flag 0 - offset in base/workpiece coordinate system, 2 - offset in tool coordinate system
    * @param [in] offset_pos Bit position offset
    * @return error code
    */
    int PointsOffsetEnable(int flag, DescPose offset_pos); 


Overall offset of points ends
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief End of overall point shift
    * @return error code
    */
    int PointsOffsetDisable(); 

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
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
        DescPose desc_p1 =new DescPose(-104.846, 309.573, 336.647, 179.681, -0.419, -92.692);
        DescPose desc_p2=new DescPose(-194.846, 309.573, 336.647, 179.681,-0.419, -92.692;);
        DescPose desc_p3=new DescPose(-254.846, 259.573,336.647, 179.681, -0.419, -92.692;);
        DescPose desc_p4=new DescPose(-304.846,259.573, 336.647, 179.681, -0.419, -92.692;);
        JointPos j1 = new JointPos();
        JointPos j2 = new JointPos();
        JointPos j3 = new JointPos();
        JointPos j4 = new JointPos();
        robot.GetInverseKin(0, desc_p1, -1, j1);//inverse kinematic solution
        robot.GetInverseKin(0, desc_p2, -1, j2);
        robot.GetInverseKin(0, desc_p3, -1, j3);
        robot.GetInverseKin(0, desc_p4, -1, j4);
        robot.MoveCart(desc_p1, 0, 0, 100.0, 100.0, 100.0, 100.0, -1.0, -1);
        robot.NewSplineStart(0, 5000);//new spline start
        robot.NewSplinePoint(j1, desc_p1, 0, 0, 100, 100, 100, 100, 50, 0);//new spline command point
        robot.NewSplinePoint(j2, desc_p2, 0, 0, 100, 100, 100, 100, 50, 0);
        robot.NewSplinePoint(j3, desc_p3, 0, 0, 100, 100, 100, 100, 50, 0);
        robot.NewSplinePoint(j4, desc_p4, 0, 0, 100, 100, 100, 100, 50, 1);
        robot.NewSplineEnd();//new spline end

        DescPose off = new DescPose(0, 0, 100, 0, 0, 0, 0);
        robot.PointsOffsetEnable(0, off);
        robot.MoveL(j1, desc_p1,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
        robot.PointsOffsetDisable(); robot.
    }

Control box AO fly shooting start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Control box AO fly shooting start *
    * @param [in] AONum Control box AO number
    * @param [in] maxTCPSpeed Maximum TCP speed value [1-5000mm/s], default 1000
    * @param [in] maxAOPercent The percentage of AO corresponding to the maximum TCP speed value, default 100%.
    * @param [in] zeroZoneCmp dead zone compensation value AO percentage, shaped, default 20%, range [0-100]
    * @return error code
    */
    int MoveAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);

Control Box AO Flying Racket Stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Control Box AO Fly Rap Stop
    * @return error code
    */
    int MoveAOStop();
    
End AO Fly Shot Starts
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
.. code-block:: Java
    :linenos:

    /**
    * @brief End of AO Fly Shot Begins *
    * @param [in] AONum End AO number
    * @param [in] maxTCPSpeed Maximum TCP speed value [1-5000mm/s], default 1000
    * @param [in] maxAOPercent The percentage of AO corresponding to the maximum TCP speed value, default 100%.
    * @param [in] zeroZoneCmp dead zone compensation value AO percentage, shaped, default 20%, range [0-100]
    * @return error code
    */
    int MoveToolAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);

End AO Fly Tap Stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
.. code-block:: Java
    :linenos:

    /**
    * @brief End AO fly tapping stops
    * @return error code
    */
    int MoveToolAOStop();

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
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
        robot.MoveToolAOStart(0, 100, 80, 1);//end AO flytap start
        //robot.MoveAOStart(0, 100, 80, 1);//control box AO flytap
        DescPose desc_p1, desc_p2.

        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0, 0);

        JointPos j1 = new JointPos(-81.684,-106.159,-74.447,-86.33,94.725,41.639);
        JointPos j2 = new JointPos(-102.804,-106.159,-74.449,-86.328,94.715,41.639);

        robot.GetForwardKin(j1,desc_p1);
        robot.GetForwardKin(j2,desc_p2);

        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();
        robot.MoveL(j1, desc_p1,0, 0, 30, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
        robot.MoveL(j2, desc_p2,0, 0, 30, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
        robot.MoveToolAOStop();
        //robot.MoveAOStop();
    }

Start Ptp motion FIR filtering
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.1-3.7.8

.. code-block:: Java
    :linenos:

    /**
    * @brief Start Ptp motion FIR filtering
    * @param [in] maxAcc Maximum acceleration extreme (deg/s2)
    * @param [in] maxJek Maximum joint jerk (deg/s3)
    * @return error code
    */
    int PtpFIRPlanningStart(double maxAcc,double maxJek);

Turn off Ptp Motion FIR Filtering
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.1-3.7.8

.. code-block:: Java
    :linenos:

    /**
    * @brief Turn off Ptp Motion FIR Filtering
    * @return error code
    */
    int PtpFIRPlanningEnd();

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.1-3.7.8

.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
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
        DescPose startdescPose=new DescPose(-569.710, -132.595, 395.147, 178.418, -1.893, 171.051);
        JointPos startjointPos=new JointPos(-2.334, -79.300, 108.196, -120.594, -91.790, -83.386);

        DescPose enddescPose=new DescPose(-366.397, -572.427, 418.339, -178.972, 1.829, -142.970);
        JointPos endjointPos=new JointPos(43.651, -70.284, 91.057, -109.075, -88.768, -83.382);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        robot.PtpFIRPlanningStart(1000);
        robot.MoveJ(startjointPos, startdescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveJ(endjointPos, enddescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.PtpFIRPlanningEnd();

        robot.MoveJ(startjointPos, startdescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveJ(endjointPos, enddescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
    }

Started LIN, ARC motion FIR filtering
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.1-3.7.8

.. code-block:: Java
    :linenos:

    /**
    * @brief Started LIN, ARC motion FIR filtering
    * @param [in] maxAccLin Linear Acceleration Extreme (mm/s2)
    * @param [in] maxAccDeg Angular Acceleration Extreme (deg/s2)
    * @param [in] maxJerkLin Linear Acceleration Extreme (mm/s3)
    * @param [in] maxJerkDeg angular plus acceleration pole (deg/s3)
    * @return error code
    */
    int LinArcFIRPlanningStart(double maxAccLin, double maxAccDeg, double maxJerkLin, double maxJerkDeg);

Turn off LIN, ARC motion FIR filtering
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.1-3.7.8

.. code-block:: Java
    :linenos:

    /**
    * @brief Turn off LIN, ARC motion FIR filtering
    * @return error code
    */
    int LinArcFIRPlanningEnd();

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
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
        DescPose startdescPose=new DescPose(-366.397, -572.427, 418.339, -178.972, 1.829, -142.970);
        JointPos startjointPos=new JointPos(43.651, -70.284, 91.057, -109.075, -88.768, -83.382);

        DescPose middescPose=new DescPose(-569.710, -132.595, 395.147, 178.418, -1.893, 171.051);
        JointPos midjointPos=new JointPos(-2.334, -79.300, 108.196, -120.594, -91.790, -83.386);

        DescPose enddescPose=new DescPose(-608.420, 610.692, 314.930, -176.438, -1.756, 117.333);
        JointPos endjointPos=new JointPos(-56.153, -46.964, 68.015, -113.200, -86.661, -83.479);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        robot.LinArcFIRPlanningStart(1000, 1000, 1000, 1000);
        robot.MoveL(startjointPos, startdescPose, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        robot.MoveC(midjointPos, middescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, 100, -1);
        robot.LinArcFIRPlanningEnd();
    }

Acceleration Smoothing Enable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.4-3.8.1
.. code-block:: Java
    :linenos:

    /**
     * @brief Enable acceleration smoothing
     * @param [in] saveFlag Save after power-off
     * @return  Error code
     */
    public int AccSmoothStart(boolean saveFlag)

Acceleration Smoothing Disable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.4-3.8.1
.. code-block:: Java
    :linenos:

    /**
     * @brief Disable acceleration smoothing
     * @param [in] saveFlag Save after power-off
     * @return  Error code
     */
    public int AccSmoothEnd(boolean saveFlag)

Code Example
+++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set reconnect attempts and interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("RPC connection fail");
            return ;
        }

        JointPos JP1 = new JointPos(88.927,-85.834,80.289,-85.561,-91.388,108.718);
        DescPose DP1 =new DescPose(88.739,-527.617,514.939,-179.039,1.494,70.209);

        JointPos JP2 =new JointPos(27.036,-83.909,80.284,-85.579,-90.027,108.604);
        DescPose DP2 = new DescPose(-433.125,-334.428,497.139,-179.723,-0.745,8.437);

        JointPos JP3 =new JointPos(60.219,-94.324,62.906,-62.005,-87.159,108.598);
        DescPose DP3 =new DescPose(-112.215,-409.323,686.497,176.217,2.338,41.625);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        int error = robot.AccSmoothStart(false);

        System.out.println("AccSmoothStart return:"+error);
        //MoveJ
        robot.MoveJ(JP1, DP1, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveJ(JP2, DP2, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveJ(JP1, DP1, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveJ(JP2, DP2, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);

        error = robot.AccSmoothEnd(false);
    }