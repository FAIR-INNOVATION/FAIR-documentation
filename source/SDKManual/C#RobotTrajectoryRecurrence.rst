Trajectory recurrence
==============================

.. toctree:: 
    :maxdepth: 5

Set track recording parameters
++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set track recording parameters
    * @param  [in] type  Record data type, 1- joint position
    * @param  [in] name  Track file name
    * @param  [in] period_ms  Data sampling period, fixed value 2ms or 4ms or 8ms
    * @param  [in] di_choose  DI Select,bit0 to bit7 corresponds to control box DI0 to DI7, bit8 to bit9 corresponds to end DI0 to DI1, 0- do not select, 1- select
    * @param  [in] do_choose  DO select,bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0- do not select, 1- select
    * @return  Error code
    */
    int SetTPDParam(int type, string name, int period_ms, UInt16 di_choose, UInt16 do_choose);

Start track recording
++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Start track recording
    * @param  [in] type  Record data type, 1- joint position
    * @param  [in] name  Track file name
    * @param  [in] period_ms  Data sampling period, fixed value 2ms or 4ms or 8ms
    * @param  [in] di_choose  DI Select,bit0 to bit7 corresponds to control box DI0 to DI7, bit8 to bit9 corresponds to end DI0 to DI1, 0- do not select, 1- select
    * @param  [in] do_choose  DO select,bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0- do not select, 1- select
    * @return  Error code
    */
    int SetTPDStart(int type, string name, int period_ms, UInt16 di_choose, UInt16 do_choose); 

Stop track recording
++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Stop track recording
    * @return  Error code
    */
    int SetWebTPDStop(); 

Delete track record
++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Delete track record
    * @param  [in] name  Track file name
    * @return  Error code
    */   
    int SetTPDDelete(string name); 

Code example
++++++++++++++
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
++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Trajectory preloading
    * @param  [in] name  Track file name
    * @return  Error code
    */      
    int LoadTPD(string name);

Get the trajectory start position and attitude
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the trajectory start pose 
    * @param [in] name  trajectory file name
    * @param [out] desc_pose trajectory start position and attitude
    * @return Error code 
    */ 
    int GetTPDStartPose(string name, ref DescPose desc_pose); 

Trajectory recurrence
++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Trajectory recurrence
    * @param  [in] name  Track file name
    * @param  [in] blend 0- not smooth, 1- smooth
    * @param  [in] ovl  Speed scaling percentage, range [0~100]
    * @return  Error code
    */
    int MoveTPD(string name, byte blend, float ovl); 

Code example
++++++++++++++++++
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
        Console.WriteLine($"GetTPDStartPose:{desc_pose.tran.x},{desc_pose.tran.y},{desc_pose.tran.z},{desc_pose.rpy.rx},{desc_pose.rpy.ry},{desc_pose.rpy.rz}");
        robot.SetTrajectoryJSpeed(100.0f);

        robot.LoadTPD(name);
        robot.MoveCart(desc_pose, tool, user, vel, acc, ovl, blendT, config);
        robot.MoveTPD(name, blend, 100.0f);
    }

External trajectory file preprocessing
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief External trajectory file preprocessing
    * @param [in] name trajectory file name 
    * @param [in] ovl Percentage speed scaling，range[0~100] 
    * @param [in] opt controls the default value is 1
    * @return Error code
    */ 
    int LoadTrajectoryJ(string name, float ovl, int opt); 

Reproduces the external trajectory file
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Reproduces the external trajectory file 
    * @return Error code
    */
    int MoveTrajectoryJ();

Gets trajectory file start position and attitude
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Gets trajectory file start position and attitude
    * @param [in] name trajectory file name  
    * @param [out] desc_pose trajectory file start position and attitude 
    * @return Error code 
    */ 
    int GetTrajectoryStartPose(string name, ref DescPose desc_pose); 

Gets the trajectory file point number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Gets the trajectory file point number   
    * @param [out] pnum trajectory file point number  
    * @return Error code 
    */  
    int GetTrajectoryPointNum(ref int pnum); 

Set the trajectory file motion move velocity
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set the trajectory file motion move velocity   
    * @param [in] ovl Percentage of velocity  
    * @return Error code 
    */  
    int SetTrajectoryJSpeed(double ovl);

Set the forces and torques in trajectory file running
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set the forces and torques in trajectory file running  
    * @param [in] ft Force and torque in three directions  (N,Nm)
    * @return Error code 
    */
    int SetTrajectoryJForceTorque(ForceTorque ft); 

Sets the force in the x direction of the trajectory
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Sets the force in the x direction of the trajectory  
    * @param [in] fx  the force in the x direction (N)
    * @return Error code
    */
    int SetTrajectoryJForceFx(double fx);

Sets the force in the y direction of the trajectory
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Sets the force in the y direction of the trajectory
    * @param [in] fy  the force in the y direction (N)
    * @return Error code 
    */
    int SetTrajectoryJForceFy(double fy);

Sets the force in the z direction of the trajectory
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Sets the force in the z direction of the trajectory
    * @param [in] fz the force in the z direction (N)
    * @return Error code 
    */
    int SetTrajectoryJForceFz(double fz);

Sets the torque around the x axis during the trajectory run
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Sets the torque around the x axis during the trajectory run  
    * @param [in] tx  the torque around the x axis (Nm)
    * @return Error code 
    */
    int SetTrajectoryJTorqueTx(double tx);

Sets the torque around the y axis during the trajectory run
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Sets the torque around the y axis during the trajectory run  
    * @param [in] ty  the torque around the x axis (Nm)
    * @return Error code 
    */
    int SetTrajectoryJTorqueTy(double ty);

Sets the torque around the z axis during the trajectory run
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Sets the torque around the z axis during the trajectory run  
    * @param [in] tz  the torque around the z axis (Nm)
    * @return Error code 
    */
    int SetTrajectoryJTorqueTz(double tz);

Code example
++++++++++++++++++
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

        DescPose desc_pos2 = new DescPose(0, 0, 0, 0, 0, 0);
        rtn = robot.GetTrajectoryStartPose(name, ref desc_pos2);
        Console.WriteLine($"GetTrajectoryStartPose:{desc_pos2.tran.x},{desc_pos2.tran.y},{desc_pos2.tran.z},{desc_pos2.rpy.rx},{desc_pos2.rpy.ry},{desc_pos2.rpy.rz}");

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
        Console.WriteLine($"SetTrajectoryJSpeed: rtn  {rtn}");

        rtn = robot.MoveTrajectoryJ();
        Console.WriteLine($"MoveTrajectoryJ:{rtn}");

        int pnum = -1;
        rtn = robot.GetTrajectoryPointNum(ref pnum);
        Console.WriteLine($"GetTrajectoryPointNum: rtn  {rtn}    num {pnum}");

        rtn = robot.SetTrajectoryJSpeed(100);
        Console.WriteLine($"SetTrajectoryJSpeed: rtn  {rtn}");

        ForceTorque ft = new ForceTorque(1, 1, 1, 1, 1, 1);
        rtn = robot.SetTrajectoryJForceTorque(ft);
        Console.WriteLine($"SetTrajectoryJForceTorque: rtn  {rtn}");

        rtn = robot.SetTrajectoryJForceFx(1.0);
        Console.WriteLine($"SetTrajectoryJForceFx: rtn  {rtn}");
        rtn = robot.SetTrajectoryJForceFy(1.0);
        Console.WriteLine($"SetTrajectoryJForceFx: rtn  {rtn}");
        rtn = robot.SetTrajectoryJForceFz(1.0);
        Console.WriteLine($"SetTrajectoryJForceFx: rtn  {rtn}");
        rtn = robot.SetTrajectoryJTorqueTx(1.0);
        Console.WriteLine($"SetTrajectoryJForceFx: rtn  {rtn}");
        rtn = robot.SetTrajectoryJTorqueTy(1.0);
        Console.WriteLine($"SetTrajectoryJForceFx: rtn  {rtn}");
        rtn = robot.SetTrajectoryJTorqueTz(1.0);
        Console.WriteLine($"SetTrajectoryJForceFx: rtn  {rtn}");
    }
