Robot trajectory replication
=====================================

.. toctree:: 
    :maxdepth: 5

Setting Track Recording Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting track logging parameters
    * @param [in] type Record data type, 1-joint position
    * @param [in] name Track file name
    * @param [in] period_ms data sampling period, fixed value 2ms or 4ms or 8ms
    * @param [in] di_choose DI selection,bit0~bit7 corresponds to control box DI0~DI7, bit8~bit9 corresponds to end DI0~DI1, 0-no choice, 1-choice
    * @param [in] do_choose DO selection, bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0-no choice, 1-choice
    * @return error code
    */
    int SetTPDParam(int type, String name, int period_ms, int di_choose, int do_choose);

Start Track Recording
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief start track record
    * @param [in] type Record data type, 1-joint position
    * @param [in] name Track file name
    * @param [in] period_ms data sampling period, fixed value 2ms or 4ms or 8ms
    * @param [in] di_choose DI choice,bit0~bit7 corresponds to control box DI0~DI7, bit8~bit9 corresponds to end DI0~DI1, 0-no choice, 1-choice
    * @param [in] do_choose DO selection, bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0-no choice, 1-choice
    * @return error code
    */
    int SetTPDStart(int type, String name, int period_ms, int di_choose, int do_choose);

Stop Track Recording
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Stop Track Recording
    * @return error code
    */
    int SetWebTPDStop(); 

Deleting track records
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Delete track record
    * @param [in] name Track file name
    * @return error code
    */  
    int SetTPDDelete(string name). 

code example
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
        int type = 1;
        String name = "tpd_2024";
        int period_ms = 2;
        int di_choose = 0;
        int do_choose = 0;

        robot.SetTPDDelete(name);//delete the track record

        robot.SetTPDParam(type, name, period_ms, di_choose, do_choose);//set track record parameters

        robot.Mode(1);
        robot.Sleep(1000);
        robot.DragTeachSwitch(1);
        robot.SetTPDStart(type, name, period_ms, di_choose, do_choose);//start track recording
        robot.Sleep(10000);
        robot.SetWebTPDStop();//stop track recording
        robot.DragTeachSwitch(0);
    }

Trajectory preloading
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Trajectory preloading
    * @param [in] name Track file name
    * @return error code
    */  
    int LoadTPD(String name).

Get the starting position of the trajectory
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get the starting position of the trajectory. 
    * @param [in] name track file name, no file extension required
    * @param [out] desc_pose The starting position of the acquired trajectory.
    * @return error code 
    */ 
    int GetTPDStartPose(String name, DescPose desc_pose); 

Trajectory Reproduction
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Trajectory replication
    * @param [in] name Track file name
    * @param [in] blend 0-unsmoothed, 1-smoothed
    * @param [in] ovl speed scaling percentage, range [0~100]
    * @return error code
    */
    int MoveTPD(String name, int blend, double ovl). 

Setting the speed of the trajectory in operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Sets the speed of the trajectory as it runs.
    * @param [in] ovl speed percentage
    * @return error code
    */
    int SetTrajectoryJSpeed(double ovl). 

code example
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
        String name = "tpd_2024";
        int tool = 0;
        int user = 0;
        double vel = 30.0;
        double acc = 100.0;
        double ovl = 100.0;
        double blendT = -1.0;
        int config = -1;
        byte blend = 1;

        DescPose desc_pose = new DescPose();
        robot.GetTPDStartPose(name, desc_pose);
        robot.SetTrajectoryJSpeed(100.0);

        robot.LoadTPD(name);
        robot.MoveCart(desc_pose, tool, user, vel, acc, ovl, blendT, config);
        robot.MoveTPD(name, blend, 80.0);
    }

External track file preprocessing
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief External track file preprocessing 
    * @param [in] name Track file name  
    * @param [in] ovl speed scaling percentage, range [0~100]
    * @param [in] opt 1-control point, default 1 
    * @return error code 
    */ 
    int LoadTrajectoryJ(String name, double ovl, int opt); 

External Trace File Trace Replication
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief External track file track replication  
    * @return error code 
    */
    int MoveTrajectoryJ().

Get the trajectory file trajectory start position
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get the starting position of the trajectory. 
    * @param [in] name Track file name  
    * @param [out] desc_pose The starting position of the acquired trajectory.
    * @return error code 
    */ 
    int GetTrajectoryStartPose(String name, DescPose desc_pose); 

Setting forces and moments in trajectory file trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting forces and moments in trajectory file trajectory runs  
    * @param [in] ft Force and torque in three directions, in N and Nm
    * @return error code 
    */
    int SetTrajectoryJForceTorque(ForceTorque ft). 

Setting the force along the x-direction in the trajectory run
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set the force along the x-direction in the trajectory run.  
    * @param [in] fx Force along the x-direction, in N
    * @return error code 
    */
    int SetTrajectoryJForceFx(double fx).

Setting the force along the y-direction in the trajectory run
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting the force along the y-direction in a trajectory run.
    * @param [in] fy Force along y direction in N
    * @return error code 
    */
    int SetTrajectoryJForceFy(double fy).

Setting the force along the z-direction in a trajectory run
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting the force along the z-direction in a trajectory run  
    * @param [in] fz Force along the z-direction in N
    * @return error code 
    */
    int SetTrajectoryJForceFz(double fz).

Setting the torque around the x-axis in a trajectory run
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Sets the torque around the x-axis for the trajectory run.  
    * @param [in] tx Torque around x-axis in Nm
    * @return error code 
    */
    int SetTrajectoryJTorqueTx(double tx).

Setting the torque around the y-axis in trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Sets the torque around the y-axis for the trajectory run.  
    * @param [in] ty Torque around y-axis in Nm
    * @return error code 
    */
    int SetTrajectoryJTorqueTy(double ty).

Setting the torque around the z-axis in trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Sets the torque around the z-axis for the trajectory run.  
    * @param [in] tz Torque around z-axis in Nm
    * @return error code 
    */
    int SetTrajectoryJTorqueTz(double tz).

code example
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

        ForceTorque tor = new ForceTorque(10.0,10.0, 10.0, 10.0, 10.0, 10.0, 10.0);
        robot.SetTrajectoryJForceTorque(tor);

        robot.SetTrajectoryJForceFx(2.0);
        robot.SetTrajectoryJForceFy(2.0);
        robot.SetTrajectoryJForceFz(2.0);
        robot.SetTrajectoryJTorqueTx(2.0);
        robot.SetTrajectoryJTorqueTy(2.0);
        robot.SetTrajectoryJTorqueTz(2.0);


        robot.LoadTrajectoryJ("/fruser/traj/test1011002.txt", 20, 1);
        DescPose startPos = new DescPose();
        robot.GetTrajectoryStartPose("/fruser/traj/test1011002.txt", startPos);
        robot.MoveCart(startPos, 0, 0, 40, 100.0, 100.0, -1.0, -1);

        ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
        System.out.println("Trajectory point num is " + pkg.trajectory_pnum);
        robot.SetTrajectoryJSpeed(40);
        robot.MoveTrajectoryJ();
    }


