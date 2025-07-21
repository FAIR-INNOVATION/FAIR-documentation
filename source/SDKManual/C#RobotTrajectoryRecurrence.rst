Robot trajectory reproduction
===============================================

.. toctree:: 
    :maxdepth: 5


Set the parameters for TPD trajectory recording
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Set the TPD trajectory recording parameters
    * @param [in] type records data type, 1- joint position
    * @param [in] name Trajectory file name
    * @param [in] period_ms data sampling period, fixed value 2ms, 4ms or 8ms
    * @param [in] di_choose DI selection,bit0 to bit7 correspond to control box DI0 to DI7, bit8 to bit9 correspond to end DI0 to DI1, 0- no selection, 1- selection
    * @param [in] do_choose DO selection,bit0 to bit7 correspond to control box DO0 to DO7, bit8 to bit9 correspond to end DO0 to DO1, 0- do not select, 1- select
    * @return error code
    * /
    int SetTPDParam(int type, string name, int period_ms, UInt16 di_choose, UInt16 do_choose);

Start TPD trajectory recording
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Starts TPD trajectory recording
    * @param [in] type records data type, 1- joint position
    * @param [in] name Trajectory file name
    * @param [in] period_ms data sampling period, fixed value 2ms, 4ms or 8ms
    * @param [in] di_choose DI selection,bit0 to bit7 correspond to control box DI0 to DI7, bit8 to bit9 correspond to end DI0 to DI1, 0- no selection, 1- selection
    * @param [in] do_choose DO selection,bit0 to bit7 correspond to control box DO0 to DO7, bit8 to bit9 correspond to end DO0 to DO1, 0- do not select, 1- select
    * @return error code
    * /
    int SetTPDStart(int type, string name, int period_ms, UInt16 di_choose, UInt16 do_choose);

Stop TPD trajectory recording
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Stop TPD trajectory recording
    * @return error code
    * /
    int SetWebTPDStop();

Delete the TPD trajectory record
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Delete TPD trajectory records
    * @param [in] name Trajectory file name
    * @return error code
    * /
    int SetTPDDelete(string name);

TPD trajectory preloading
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Trajectory Preloading
    * @param [in] name Trajectory file name
    * @return error code
    * /
    int LoadTPD(string name);

Obtain the starting pose of the TPD trajectory
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Obtain the starting pose of the trajectory
    * @param [in] name Trajectory file name
    * @param [out] desc_pose trajectory starting pose
    * @return error code
    * /
    int GetTPDStartPose(string name, ref DescPose desc_pose);

TPD trajectory reproduction
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Trajectory Reproduction
    * @param [in] name Trajectory file name
    * @param [in] blend 0- not smooth, 1- smooth
    * @param [in] ovl speed scaling percentage, range [0 to 100]
    * @return error code
    * /
    int MoveTPD(string name, byte blend, float ovl);

A sample code for robot TPD trajectory recording
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnTPDMove_Click(object sender, EventArgs e)
    {
        int type = 1;
        string name = "tpd2025";
        int period_ms = 4;
        ushort di_choose = 0;
        ushort do_choose = 0;

        robot.SetTPDParam(type, name, period_ms, di_choose, do_choose);

        robot.Mode(1);
        Thread.Sleep(1000);
        robot.DragTeachSwitch(1);
        robot.SetTPDStart(type, name, period_ms, di_choose, do_choose);
        Thread.Sleep(10000);
        robot.SetWebTPDStop();
        robot.DragTeachSwitch(0);

        float ovl = 100.0f;
        byte blend = 0;

        DescPose start_pose = new DescPose();

        int rtn = robot.LoadTPD(name);
        Console.WriteLine("LoadTPD rtn is: {0}\n", rtn);

        robot.GetTPDStartPose(name, ref start_pose);
        Console.WriteLine("start pose, xyz is: {0} {1} {2}. rpy is: {3} {4} {5} \n",
            start_pose.tran.x, start_pose.tran.y, start_pose.tran.z,
            start_pose.rpy.rx, start_pose.rpy.ry, start_pose.rpy.rz);
        robot.MoveCart(start_pose, 0, 0, 100, 100, ovl, -1, -1);
        Thread.Sleep(1000);

        rtn = robot.MoveTPD(name, blend, ovl);
        Console.WriteLine("MoveTPD rtn is: {0}\n", rtn);
        Thread.Sleep(5000);

        robot.SetTPDDelete(name);
    }

Preprocessing of external trajectory files
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief External Trajectory File Preprocessing
    * @param [in] name Trajectory file name
    * @param [in] ovl speed scaling percentage, range [0 to 100]
    * @param [in] opt 1- Control point, default is 1
    * @return error code
    * /
    int LoadTrajectoryJ(string name, float ovl, int opt);

External trajectory file trajectory reproduction
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief External Trajectory file trajectory reproduction
    * @return error code
    * /
    int MoveTrajectoryJ();

Obtain the starting position of the trajectory file's trajectory
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Get the starting position of the trajectory file trajectory
    * @param [in] name Trajectory file name
    * @param [out] desc_pose trajectory starting pose
    * @return error code
    * /
    int GetTrajectoryStartPose(string name, ref DescPose desc_pose);

Obtain the trajectory point number in the trajectory file
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Get the trajectory point number
    * @param [out] pnum trajectory point number
    * @return error code
    * /
    int GetTrajectoryPointNum(ref int pnum);

Set the running speed of the trajectory file trajectory
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Set the running speed of the trajectory file trajectory
    * @param [in] ovl speed percentage
    * @return error code
    * /
    int SetTrajectoryJSpeed(double ovl);

Set the force and torque during the trajectory file's trajectory operation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Set the force and torque during the trajectory operation of the trajectory file
    * @param [in] ft forces and torques in three directions, units N and Nm
    * @return error code
    * /
    int SetTrajectoryJForceTorque(ForceTorque ft);

Set the force along the x direction during the trajectory's operation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Set the force along the x direction during the trajectory operation
    * @param [in] fx the force along the x direction, unit N
    * @return error code
    * /
    int SetTrajectoryJForceFx(double fx);

Set the force along the y direction during the trajectory's operation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Set the force along the y direction during the trajectory operation
    * @param [in] fy the force along the y direction, unit N
    * @return error code
    * /
    int SetTrajectoryJForceFy(double fy);

Set the force along the z direction during the trajectory's operation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Set the force along the z direction during the trajectory operation
    * @param [in] fz force along the z direction, unit N
    * @return error code
    * /
    int SetTrajectoryJForceFz(double fz);

Set the torque around the X-axis during the trajectory's operation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Set the torque around the X-axis during the trajectory operation
    * @param [in] tx Torque around the X-axis, in Nm
    * @return error code
    * /
    int SetTrajectoryJTorqueTx(double tx);

Set the torque around the Y-axis during the trajectory's operation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Set the torque around the Y-axis during the trajectory operation
    * @param [in] ty Torque around the Y-axis, in Nm
    * @return error code
    * /
    int SetTrajectoryJTorqueTy(double ty);

Set the torque around the Z-axis during the trajectory's operation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Set the torque around the Z-axis during the trajectory operation
    * @param [in] tz the torque around the Z-axis, in Nm
    * @return error code
    * /
    int SetTrajectoryJTorqueTz(double tz);

Upload the trajectory J file
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Upload the trajectory J file
    * @param [in] filePath upload the full path name of the trajectory file C://test/testJ.txt
    * @return error code
    * /
    int TrajectoryJUpLoad(string filePath);

Delete the trajectory J file
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Delete the trajectory J file
    * @param [in] fileName file name testJ.txt
    * @return error code
    * /
    int TrajectoryJDelete(string fileName);

Sample code for reproducing robot trajectory J files
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button33_Click(object sender, EventArgs e)
    {
        int rtn = robot.TrajectoryJUpLoad("D://zUP/spray_traj1.txt");
        Console.WriteLine("Upload TrajectoryJ A {0}\n", rtn);

        string traj_file_name = "/fruser/traj/spray_traj1.txt";
        rtn = robot.LoadTrajectoryJ(traj_file_name, 100, 1);
        Console.WriteLine("LoadTrajectoryJ {0}, rtn is: {1}\n", traj_file_name, rtn);

        DescPose traj_start_pose = new DescPose();
        rtn = robot.GetTrajectoryStartPose(traj_file_name, ref traj_start_pose);
        Console.WriteLine("GetTrajectoryStartPose is: {0}\n", rtn);
        Console.WriteLine("desc_pos:{0},{1},{2},{3},{4},{5}\n",
            traj_start_pose.tran.x, traj_start_pose.tran.y, traj_start_pose.tran.z,
            traj_start_pose.rpy.rx, traj_start_pose.rpy.ry, traj_start_pose.rpy.rz);

        Thread.Sleep(1000);

        robot.SetSpeed(50);
        robot.MoveCart(traj_start_pose, 0, 0, 100, 100, 100, -1, -1);

        int traj_num = 0;
        rtn = robot.GetTrajectoryPointNum(ref traj_num);
        Console.WriteLine("GetTrajectoryStartPose rtn is: {0}, traj num is: {1}\n", rtn, traj_num);

        rtn = robot.SetTrajectoryJSpeed(50.0f);
        Console.WriteLine("SetTrajectoryJSpeed is: {0}\n", rtn);

        ForceTorque traj_force = new ForceTorque();
        traj_force.fx = 10;
        rtn = robot.SetTrajectoryJForceTorque(traj_force);
        Console.WriteLine("SetTrajectoryJForceTorque rtn is: {0}\n", rtn);

        rtn = robot.SetTrajectoryJForceFx(10.0f);
        Console.WriteLine("SetTrajectoryJForceFx rtn is: {0}\n", rtn);

        rtn = robot.SetTrajectoryJForceFy(0.0f);
        Console.WriteLine("SetTrajectoryJForceFy rtn is: {0}\n", rtn);

        rtn = robot.SetTrajectoryJForceFz(0.0f);
        Console.WriteLine("SetTrajectoryJForceFz rtn is: {0}\n", rtn);

        rtn = robot.SetTrajectoryJTorqueTx(10.0f);
        Console.WriteLine("SetTrajectoryJTorqueTx rtn is: {0}\n", rtn);

        rtn = robot.SetTrajectoryJTorqueTy(10.0f);
        Console.WriteLine("SetTrajectoryJTorqueTy rtn is: {0}\n", rtn);

        rtn = robot.SetTrajectoryJTorqueTz(10.0f);
        Console.WriteLine("SetTrajectoryJTorqueTz rtn is: {0}\n", rtn);

        rtn = robot.MoveTrajectoryJ();
        Console.WriteLine("MoveTrajectoryJ rtn is: {0}\n", rtn);
    }

Trajectory preprocessing (trajectory forward-looking)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Trajectory Preprocessing (Trajectory Preview)
    * @param [in] name Trajectory file name
    * @param [in] mode Sampling mode, 0- no sampling; 1- Equal data interval sampling; 2- Equal error limiting sampling
    * @param [in] errorLim error limit, effective with linear fitting
    * @param [in] type smoothing method, 0-Bezier smoothing
    * @param [in] precision Smoothing accuracy, effective when using Bezier smoothing
    * @param [in] The maximum speed set by vamx, mm/s
    * @param [in] The maximum acceleration set by amax, mm/s2
    * @param [in] The maximum acceleration set by jmax, mm/s3
    * @return error code
    * /
    int LoadTrajectoryLA(string name, int mode, double errorLim, int type, double precision, double vamx, double amax,  double jmax);

Trajectory reproduction (Trajectory Foresight)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Trajectory Reproduction (Trajectory Preview)
    * @return error code
    * /
    int MoveTrajectoryLA();

Trajectory reproduction (trajectory forward-looking) code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button8_Click(object sender, EventArgs e)
    {
        int rtn = 0;

        string nameA = "/fruser/traj/A.txt";
        string nameB = "/fruser/traj/B.txt";

        rtn = robot.LoadTrajectoryLA(nameB, 0, 0, 0, 1, 100.0, 100.0, 1000.0);    // 直线拟合
        Console.WriteLine($"LoadTrajectoryLA rtn is {rtn}");

        DescPose startPos = new DescPose(0, 0, 0, 0, 0, 0);
        robot.GetTrajectoryStartPose(nameA, ref startPos);

        //
        robot.MoveCart(startPos, 1, 0, (float)100.0, (float)100.0, (float)100.0, -1, -1);

        rtn = robot.MoveTrajectoryLA();
        Console.WriteLine($"MoveTrajectoryLA rtn is {rtn}");
    }
