Robot trajectory replication
==============================
.. toctree:: 
    :maxdepth: 5

Setting up track recording parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief Setting track logging parameters
    * @param [in] type Record data type, 1-joint position
    * @param [in] name track file name
    * @param [in] period_ms data sampling period, fixed value 2ms or 4ms or 8ms
    * @param [in] di_choose DI choice,bit0~bit7 corresponds to control box DI0~DI7,bit8~bit9 corresponds to end DI0~DI1,0-no choice,1-choice
    * @param [in] do_choose DO selection, bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0-no choice, 1-choice
    * @return error code
    */
    int SetTPDParam(int type, string name, int period_ms, UInt16 di_choose, UInt16 do_choose);

Start Track Recording
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief start track record
    * @param [in] type Record data type, 1-joint position
    * @param [in] name track file name
    * @param [in] period_ms data sampling period, fixed value 2ms or 4ms or 8ms
    * @param [in] di_choose DI choice,bit0~bit7 corresponds to control box DI0~DI7,bit8~bit9 corresponds to end DI0~DI1,0-no choice,1-choice
    * @param [in] do_choose DO selection, bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0-no choice, 1-choice
    * @return error code
    */
    int SetTPDStart(int type, string name, int period_ms, UInt16 di_choose, UInt16 do_choose); 

Stop Track Recording
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief stop track recording
    * @return error code
    */
    int SetWebTPDStop(); 

Deleting track records
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Delete track record
    * @param [in] name track file name
    * @return error code
    */  
    int SetTPDDelete(string name). 

code example
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnTCPRecord_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        int type = 1;
        string name = "tpd2023";
        int period_ms = 2;
        UInt16 di_choose = 0;
        UInt16 do_choose = 0;

        robot.SetTPDParam(type, name, period_ms, di_choose, do_choose);

        robot.Mode(1);
        Thread.Sleep(1000);
        robot.DragTeachSwitch(1);
        robot.SetTPDStart(type, name, period_ms, di_choose, do_choose);
        Thread.Sleep(10000);
        robot.SetWebTPDStop();
        robot.DragTeachSwitch(0);

        //robot.SetTPDDelete(name);
    }

Trajectory preloading
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Trajectory preloading
    * @param [in] name track file name
    * @return error code
    */  
    int LoadTPD(string name).

Get the start position of the trajectory
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the trajectory start position. 
    * @param [in] name track file name
    * @param [out] desc_pose trajectory start position 
    * @return error code 
    */ 
    int GetTPDStartPose(string name, ref DescPose desc_pose); 

Trajectory Reproduction
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief Trajectory replication
    * @param [in] name track file name
    * @param [in] blend 0-unsmoothed, 1-smoothed
    * @param [in] ovl speed scaling percentage, range [0~100]
    * @return error code
    */
    int MoveTPD(string name, byte blend, float ovl). 

code example
++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnTPDMove_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        string name = "tpd2023";
        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = -1.0f;
        int config = -1;
        byte blend = 1;

        DescPose desc_pose = new DescPose();
        robot.GetTPDStartPose(name, ref desc_pose);
        Console.WriteLine($"GetTPDStartPose:{desc_pose.tran.x},{desc_pose.tran.y},{desc_pose.tran.z},{desc_pose.rpy.rx},{desc_pose.rpy.ry} ,{desc_pose.rpy.rz}").
        robot.SetTrajectoryJSpeed(100.0f);

        robot.LoadTPD(name);
        robot.MoveCart(desc_pose, tool, user, vel, acc, ovl, blendT, config);
        robot.MoveTPD(name, blend, 100.0f);
    }

External track file preprocessing
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief External track file preprocessing 
    * @param [in] name track file name  
    * @param [in] ovl speed scaling percentage, range [0~100] 
    * @param [in] opt 1-control point, default 1 
    * @return error code 
    */ 
    int LoadTrajectoryJ(string name, float ovl, int opt); 

External track file track reproduction
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief External trajectory file trajectory replication  
    * @return error code 
    */
    int MoveTrajectoryJ().

Get track file track start position
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get track file track start position 
    * @param [in] name track file name  
    * @param [out] desc_pose trajectory start position  
    * @return error code 
    */ 
    int GetTrajectoryStartPose(string name, ref DescPose desc_pose); 

Get track file track point number
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get track point number   
    * @param [out] pnum track point number  
    * @return error code 
    */  
    int GetTrajectoryPointNum(ref int pnum).

Setting Track File Track Run Speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting track file track run speeds   
    * @param [in] ovl speed percentage  
    * @return error code 
    */  
    int SetTrajectoryJSpeed(double ovl).

Setting forces and moments in trajectory file trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting forces and moments in trajectory file trajectory runs  
    * @param [in] ft Force and torque in three directions, in N and Nm
    * @return error code 
    */
    int SetTrajectoryJForceTorque(ForceTorque ft). 

Setting the force along the x-direction in the trajectory run
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief sets the force along the x-direction in the trajectory run  
    * @param [in] fx Force along x direction in N
    * @return error code 
    */
    int SetTrajectoryJForceFx(double fx).

Setting the force along the y-direction in the trajectory run
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting the force along the y-direction in the trajectory run  
    * @param [in] fy Force along y direction in N
    * @return error code 
    */
    int SetTrajectoryJForceFy(double fy).

Setting the force along the z-direction in a trajectory run
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting the force along the z-direction in a trajectory run  
    * @param [in] fz Force along the z-direction in N
    * @return error code 
    */
    int SetTrajectoryJForceFz(double fz).

Setting the torque around the x-axis in a trajectory run
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting the torque around the x-axis in the trajectory run  
    * @param [in] tx Torque around x-axis in Nm
    * @return error code 
    */
    int SetTrajectoryJTorqueTx(double tx).

Setting the torque around the y-axis in trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting the torque around the y-axis for a trajectory run  
    * @param [in] ty Torque around y-axis in Nm
    * @return error code 
    */
    int SetTrajectoryJTorqueTy(double ty).

Setting the torque around the z-axis in trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Sets the torque around the z-axis for the trajectory run.  
    * @param [in] tz Torque around z-axis in Nm
    * @return error code 
    */
    int SetTrajectoryJTorqueTz(double tz).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnTrajectory_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");
        string name = "/fruser/traj/trajHelix_aima_1.txt";
        int rtn = -1;

        rtn = robot.LoadTrajectoryJ(name, 100, 1);
        Console.WriteLine($"LoadTrajectoryJ:{rtn}");

        DescPose desc_pos2 = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);
        rtn = robot.GetTrajectoryStartPose(name, ref desc_pos2);
        Console.WriteLine($"GetTrajectoryStartPose:{desc_pos2.tran.x},{desc_pos2.tran.y},{desc_pos2.tran.z},{desc_pos2.rpy.rx},{desc_pos2. rpy.ry},{desc_pos2.rpy.rz}").

        int tool = 1;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = -1.0f;
        float blendT1 = 0.0f;
        int config = -1;
        robot.MoveCart(desc_pos2, tool, user, vel, acc, ovl, blendT, config);

        rtn = robot.SetTrajectoryJSpeed(20);
        Console.WriteLine($"SetTrajectoryJSpeed: rtn {rtn}");

        rtn = robot.MoveTrajectoryJ();
        Console.WriteLine($"MoveTrajectoryJ:{rtn}");

        int pnum = -1;
        rtn = robot.GetTrajectoryPointNum(ref pnum);
        Console.WriteLine($"GetTrajectoryPointNum: rtn {rtn} num {pnum}");

        rtn = robot.SetTrajectoryJSpeed(100);
        Console.WriteLine($"SetTrajectoryJSpeed: rtn {rtn}");

        ForceTorque ft = new ForceTorque(1, 1, 1, 1, 1, 1, 1, 1);
        rtn = robot.SetTrajectoryJForceTorque(ft);
        Console.WriteLine($"SetTrajectoryJForceTorque: rtn {rtn}");

        rtn = robot.SetTrajectoryJForceFx(1.0);
        Console.WriteLine($"SetTrajectoryJForceFx: rtn {rtn}");
        rtn = robot.SetTrajectoryJForceFy(1.0);
        Console.WriteLine($"SetTrajectoryJForceFx: rtn {rtn}");
        rtn = robot.SetTrajectoryJForceFz(1.0);
        Console.WriteLine($"SetTrajectoryJForceFx: rtn {rtn}");
        rtn = robot.SetTrajectoryJTorqueTx(1.0);
        Console.WriteLine($"SetTrajectoryJForceFx: rtn {rtn}");
        rtn = robot.SetTrajectoryJTorqueTy(1.0);
        Console.WriteLine($"SetTrajectoryJForceFx: rtn {rtn}");
        rtn = robot.SetTrajectoryJTorqueTz(1.0);
        Console.WriteLine($"SetTrajectoryJForceFx: rtn {rtn}");
    }