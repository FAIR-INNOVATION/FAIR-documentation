Robot trajectory playback  
==============================

.. toctree::  
    :maxdepth: 5  

Set TPD trajectory recording parameters  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Set TPD trajectory recording parameters  
    * @param  [in] type  Recording data type, 1-Joint position  
    * @param  [in] name  Trajectory file name  
    * @param  [in] period_ms  Data sampling period, fixed value 2ms/4ms/8ms  
    * @param  [in] di_choose  DI selection, bit0~bit7 correspond to controller DI0~DI7, bit8~bit9 correspond to end DI0~DI1, 0-Not selected, 1-Selected  
    * @param  [in] do_choose  DO selection, bit0~bit7 correspond to controller DO0~DO7, bit8~bit9 correspond to end DO0~DO1, 0-Not selected, 1-Selected  
    * @return  Error code  
    */  
    int SetTPDParam(int type, String name, int period_ms, int di_choose, int do_choose);  

Start TPD trajectory recording  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Start TPD trajectory recording  
    * @param  [in] type  Recording data type, 1-Joint position  
    * @param  [in] name  Trajectory file name  
    * @param  [in] period_ms  Data sampling period, fixed value 2ms/4ms/8ms  
    * @param  [in] di_choose  DI selection, bit0~bit7 correspond to controller DI0~DI7, bit8~bit9 correspond to end DI0~DI1, 0-Not selected, 1-Selected  
    * @param  [in] do_choose  DO selection, bit0~bit7 correspond to controller DO0~DO7, bit8~bit9 correspond to end DO0~DO1, 0-Not selected, 1-Selected  
    * @return  Error code  
    */  
    int SetTPDStart(int type, String name, int period_ms, int di_choose, int do_choose);  

Stop TPD trajectory recording  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: java  
    :linenos:  

    /**  
    * @brief  Stop TPD trajectory recording  
    * @return  Error code  
    */  
    int SetWebTPDStop();  

Delete TPD trajectory recording  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Delete TPD trajectory recording  
    * @param  [in] name  Trajectory file name  
    * @return  Error code  
    */  
    int SetTPDDelete(string name);  

TPD trajectory preloading  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Trajectory preloading  
    * @param  [in] name  Trajectory file name  
    * @return  Error code  
    */  
    int LoadTPD(String name);  

TPD trajectory playback 
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Trajectory playback  
    * @param  [in] name  Trajectory file name  
    * @param  [in] blend 0-No smoothing, 1-Smoothing  
    * @param  [in] ovl  Speed scaling percentage, range [0~100]  
    * @return  Error code  
    */  
    int MoveTPD(String name, int blend, double ovl);  

Get TPD starting pose  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get trajectory starting pose  
    * @param [in] name  Trajectory file name (without extension)  
    * @param [out] desc_pose  Retrieved trajectory starting pose  
    * @return Error code  
    */  
    int GetTPDStartPose(String name, DescPose desc_pose);  

Robot TPD trajectory recording code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestTPD(Robot robot)  
    {  
        int type = 1;  
        String name = "tpd2025";  
        int period_ms = 4;  
        int di_choose = 0;  
        int do_choose = 0;  

        robot.SetTPDParam(type, name, period_ms, di_choose, do_choose);  

        robot.Mode(1);  
        robot.Sleep(1000);  
        robot.DragTeachSwitch(1);  
        robot.SetTPDStart(type, name, period_ms, di_choose, do_choose);  
        robot.Sleep(10000);  
        robot.SetWebTPDStop();  
        robot.DragTeachSwitch(0);  

        double ovl = 100.0;  
        int blend = 0;  

        DescPose start_pose =new DescPose() {};  

        int rtn = robot.LoadTPD(name);  
        System.out.println("LoadTPD rtn is:"+ rtn);  

        robot.GetTPDStartPose(name, start_pose);  
        robot.MoveCart(start_pose, 0, 0, 100, 100, ovl, -1, -1);  
        robot.Sleep(1000);  

        rtn = robot.MoveTPD(name, blend, ovl);  
        System.out.println("MoveTPD rtn is: "+ rtn);  
        robot.Sleep(5000);  

        robot.SetTPDDelete(name);  
        return 0;  
    }  

Trajectory preprocessing  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief External trajectory file preprocessing  
    * @param [in] name Trajectory file name  
    * @param [in] ovl Speed scaling percentage, range [0~100]  
    * @param [in] opt 1-Control point (default)  
    * @return Error code  
    */  
    int LoadTrajectoryJ(String name, double ovl, int opt);  

Trajectory playback  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief External trajectory file playback  
    * @return Error code  
    */  
    int MoveTrajectoryJ();  

Get trajectory starting pose  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get trajectory starting pose  
    * @param [in] name Trajectory file name  
    * @param [out] desc_pose Retrieved trajectory starting pose  
    * @return Error code  
    */  
    int GetTrajectoryStartPose(String name, DescPose desc_pose);  

Get trajectory point number  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get trajectory point number  
    * @return  Error code  
    */  
    public int GetTrajectoryPointNum(int pnum)  

Set trajectory playback speed  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /*
    * @brief  Set trajectory playback speed  
    * @param  ovl Speed percentage
    * @param  mode Mode; 0-decruise mode; 1-direct switching
    * @return  Error code
    */
    public int SetTrajectoryJSpeed(double ovl, int mode)

Set force/torque during trajectory playback  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Set force/torque during trajectory playback  
    * @param  [in] ft Force and torque in three directions (unit: N and Nm)  
    * @return  Error code  
    */  
    public int SetTrajectoryJForceTorque(ForceTorque ft)  

Set x-direction force during trajectory playback  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set x-direction force during trajectory playback  
    * @param [in] fx x-direction force (unit: N)  
    * @return Error code  
    */  
    int SetTrajectoryJForceFx(double fx);  

Set y-direction force during trajectory playback  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set y-direction force during trajectory playback  
    * @param [in] fy y-direction force (unit: N)  
    * @return Error code  
    */  
    int SetTrajectoryJForceFy(double fy);  

Set z-direction force during trajectory playback  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set z-direction force during trajectory playback  
    * @param [in] fz z-direction force (unit: N)  
    * @return Error code  
    */  
    int SetTrajectoryJForceFz(double fz);  

Set x-axis torque during trajectory playback  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set x-axis torque during trajectory playback  
    * @param [in] tx x-axis torque (unit: Nm)  
    * @return Error code  
    */  
    int SetTrajectoryJTorqueTx(double tx);  

Set y-axis torque during trajectory playback  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set y-axis torque during trajectory playback  
    * @param [in] ty y-axis torque (unit: Nm)  
    * @return Error code  
    */  
    int SetTrajectoryJTorqueTy(double ty);  

Set z-axis torque during trajectory playback 
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set z-axis torque during trajectory playback  
    * @param [in] tz z-axis torque (unit: Nm)  
    * @return Error code  
    */  
    int SetTrajectoryJTorqueTz(double tz);  

Upload trajectory J file  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Upload trajectory J file  
    * @param [in] filePath Full path of trajectory file (e.g., C://test/testJ.txt)  
    * @return Error code  
    */  
    int TrajectoryJUpLoad(String filePath);  

Delete trajectory J file  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Delete trajectory J file  
    * @param [in] fileName File name (e.g., testJ.txt)  
    * @return Error code  
    */  
    int TrajectoryJDelete(String fileName);  

Robot trajectory J file playback code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestTraj(Robot robot)  
    {  
        int rtn = robot.TrajectoryJUpLoad("D://zUP/horse.txt");  
        System.out.println("Upload TrajectoryJ A :"+ rtn);  

        String traj_file_name = "horse.txt";  
        rtn = robot.LoadTrajectoryJ(traj_file_name, 100, 1);  
        System.out.println("LoadTrajectoryJ:"+traj_file_name+", rtn is:"+ rtn);  

        DescPose traj_start_pose=new DescPose(0,0,0,0,0,0);  
        rtn = robot.GetTrajectoryStartPose(traj_file_name, traj_start_pose);  

        robot.Sleep(1000);  

        ExaxisPos epos=new ExaxisPos(0,0,0,0);  
        DescPose po=new DescPose(0,0,0,0,0,0);  
        robot.SetSpeed(50);  
        robot.MoveCart(traj_start_pose, 0, 0, 100, 100, 100, -1, -1);  

        int traj_num = 0;  
        rtn = robot.GetTrajectoryPointNum(traj_num);  

        rtn = robot.SetTrajectoryJSpeed(50.0);  
        System.out.println("SetTrajectoryJSpeed is:"+ rtn);  

        ForceTorque traj_force=new ForceTorque(0,0,0,0,0,0);  
        traj_force.fx = 10;  
        rtn = robot.SetTrajectoryJForceTorque(traj_force);  
        System.out.println("SetTrajectoryJForceTorque rtn is: "+ rtn);  

        rtn = robot.SetTrajectoryJForceFx(10.0);  
        System.out.println("SetTrajectoryJForceFx rtn is:"+ rtn);  

        rtn = robot.SetTrajectoryJForceFy(0.0);  
        System.out.println("SetTrajectoryJForceFy rtn is:"+ rtn);  

        rtn = robot.SetTrajectoryJForceFz(0.0);  
        System.out.println("SetTrajectoryJForceFz rtn is: "+ rtn);  

        rtn = robot.SetTrajectoryJTorqueTx(10.0);  
        System.out.println("SetTrajectoryJTorqueTx rtn is: "+ rtn);  

        rtn = robot.SetTrajectoryJTorqueTy(10.0);  
        System.out.println("SetTrajectoryJTorqueTy rtn is:"+ rtn);  

        rtn = robot.SetTrajectoryJTorqueTz(10.0);  
        System.out.println("SetTrajectoryJTorqueTz rtn is:"+ rtn);  

        rtn = robot.MoveTrajectoryJ();  
        System.out.println("MoveTrajectoryJ rtn is: "+ rtn);  

        return 0;  
    }  

Code example for setting the speed during robot trajectory execution
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: Java
    :linenos:

    public int TestSetTrajectoryJSpeed(Robot robot) {
        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
        int rtn;

        robot.SetReconnectParam(true, 30000, 500);
        rtn = robot.TrajectoryJUpLoad("D://zUP/horse.txt");
        System.out.printf("Upload TrajectoryJ A %d%n", rtn);
        String trajFileName = "horse.txt";
        rtn = robot.LoadTrajectoryJ(trajFileName, 100, 1);
        System.out.printf("LoadTrajectoryJ %s, rtn is: %d%n", trajFileName, rtn);
        DescPose trajStartPose = new DescPose();
        rtn = robot.GetTrajectoryStartPose(trajFileName, trajStartPose);
        System.out.printf("GetTrajectoryStartPose is: %d%n", rtn);
        System.out.printf("desc_pos:%f,%f,%f,%f,%f,%f%n", trajStartPose.tran.x, trajStartPose.tran.y, trajStartPose.tran.z, trajStartPose.rpy.rx, trajStartPose.rpy.ry, trajStartPose.rpy.rz);
        robot.Sleep(1000);
        robot.SetSpeed(50);
        robot.MoveCart(trajStartPose, 0, 0, 100, 100, 100, -1, -1);
        rtn = robot.GetTrajectoryPointNum(0);
        pkg = robot.GetRobotRealTimeState();
        int trajNum = pkg.trajectory_pnum;
        System.out.printf("GetTrajectoryPointNum rtn is: %d, traj num is: %d%n", rtn, trajNum);

        rtn = robot.MoveTrajectoryJ();
        System.out.printf("MoveTrajectoryJ rtn is: %d%n", rtn);

        robot.Sleep(1000);

        pkg = robot.GetRobotRealTimeState();
        int trajspeedMode = 1;
        while (pkg.motion_done == 0)
        {
            pkg = robot.GetRobotRealTimeState();

            rtn = robot.SetTrajectoryJSpeed(10.0, trajspeedMode);
            System.out.printf("SetTrajectoryJSpeed is: %d%n", rtn);

            robot.Sleep(1000);

            rtn = robot.SetTrajectoryJSpeed(80.0, trajspeedMode);
            System.out.printf("SetTrajectoryJSpeed is: %d%n", rtn);

            robot.Sleep(1000);
        }

        return 0;
    }

Trajectory preprocessing (lookahead)  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.3-3.8.0  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Trajectory preprocessing (lookahead)  
    * @param [in] name  Trajectory file name  
    * @param [in] mode  Sampling mode: 0-No sampling; 1-Equal interval sampling; 2-Equal error limit sampling  
    * @param [in] errorLim  Error limit (effective when using linear fitting)  
    * @param [in] type  Smoothing method: 0-Bezier smoothing  
    * @param [in] precision  Smoothing precision (effective when using Bezier smoothing)  
    * @param [in] vamx  Maximum speed setting (mm/s)  
    * @param [in] amax  Maximum acceleration setting (mm/s²)  
    * @param [in] jmax  Maximum jerk setting (mm/s³)  
    * @return Error code  
    */  
    int LoadTrajectoryLA(String name, int mode, double errorLim, int type, double precision, double vamx, double amax, double jmax);  

Trajectory playback (lookahead)  
+++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.3-3.8.0  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Trajectory playback (lookahead)  
    * @return Error code  
    */  
    int MoveTrajectoryLA();  

Trajectory playback (lookahead) code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestLoadTrajLA(Robot robot)  
    {  
        int rtn = robot.TrajectoryJUpLoad("D://zUP/traj.txt");  

        String traj_file_name = "/fruser/traj/traj.txt";  
        rtn = robot.LoadTrajectoryLA(traj_file_name, 1, 2, 0, 2, 100, 200, 1000);  

        DescPose traj_start_pose=new DescPose(0,0,0,0,0,0);  
        rtn = robot.GetTrajectoryStartPose(traj_file_name, traj_start_pose);  

        robot.Sleep(1000);  
        robot.SetSpeed(50);  
        robot.MoveCart(traj_start_pose, 0, 0, 100, 100, 100, -1, -1);  

        rtn = robot.MoveTrajectoryLA();  

        robot.CloseRPC();  
        return 0;  
    }  

Move to TPD Trajectory Recording Start Point
++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Move to TPD trajectory recording start point
    * @param name Trajectory file name
    * @param moveType Motion type; 0-PTP; 1-LIN
    * @param ovl Speed scaling percentage, range [0~100]
    * @return Error code
    */
    public int MoveToTPDStart(string name, int moveType, double ovl)

SDK Code Example for Moving to TPD Trajectory Recording Start Point
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void testTPDmove (Robot robot)
    {
        int rtn = 0;
        int type = 1;
        String name = "tpd2025";
        int period_ms = 4;
        int di_choose = 0;
        int do_choose = 0;

        robot.SetTPDParam(type, name, period_ms, di_choose, do_choose);

        robot.Mode(1);
        robot.Sleep(1000);
        robot.DragTeachSwitch(1);
        robot.SetTPDStart(type, name, period_ms, di_choose, do_choose);
        robot.Sleep(3000);
        robot.SetWebTPDStop();
        robot.DragTeachSwitch(0);

        robot.Sleep(1000);
        double ovl = 100.0;
        int blend = 0;
        DescPose start_pose = new DescPose();
        rtn = robot.LoadTPD(name);
        System.out.printf("LoadTPD rtn is: %d\n", rtn);

        robot.GetTPDStartPose(name, start_pose);
        System.out.printf("start pose, xyz is: %f %f %f. rpy is: %f %f %f \n", start_pose.tran.x, start_pose.tran.y, start_pose.tran.z, start_pose.rpy.rx, start_pose.rpy.ry, start_pose.rpy.rz);
        //robot.MoveCart(&start_pose, 0, 0, 100, 100, ovl, -1, -1);
        //robot.Sleep(1000);

        rtn = robot.MoveToTPDStart(name, 0, 100);
        System.out.printf("MoveToTPDStart rtn is: %d\n", rtn);

        rtn = robot.MoveTPD(name, blend, ovl);
        System.out.printf("MoveTPD rtn is: %d\n", rtn);

        robot.Sleep(5000);

        robot.SetTPDDelete(name);

        return ;
    }