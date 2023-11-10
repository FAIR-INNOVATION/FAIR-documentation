Common Settings
==========================

.. toctree:: 
    :maxdepth: 5

Set global speed
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set global speed
    * @param  [in]  vel  Percentage of velocity, range[0~100]
    * @return  Error code
    */
    int SetSpeed(int vel); 

Set the value of a system variable
+++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the value of a system variable
    * @param  [in]  id  Variable number, range[1~20]
    * @param  [in]  value Variable value
    * @return  Error code
    */
    int SetSysVarValue(int id, double value);

Set tool coordinate reference point - six point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set tool coordinate reference point - six point method 
    * @param [in] point_num point number,range[1~6] 
    * @return Error code
    */ 
    int SetToolPoint(int point_num); 

Calculation tool coordinate - six point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Calculation tool coordinate - six point method
    * @param [out] tcp_pose tool coordinate
    * @return Error code 
    */ 
    int ComputeTool(ref DescPose tcp_pose); 

Set tool coordinate reference point - four point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set tool coordinate reference point - four point method
    * @param [in] point_num point number,range[1~4] 
    * @return Error code 
    */ 
    int SetTcp4RefPoint(int point_num); 

Calculation tool coordinate - four point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Calculation tool coordinate - four point method
    * @param [out] tcp_pose tool coordinate
    * @return Error code
    */ 
    int ComputeTcp4(ref DescPose tcp_pose); 

Set tool coordinate system
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set tool coordinate system
    * @param  [in] id Frame number, range[1~15]
    * @param  [in] coord  Tool center position relative to end flange center position
    * @param  [in] type  0- tool coordinates, 1- sensor coordinates
    * @param  [in] install Installation position, 0- robot end, 1- robot outside
    * @return  Error code
    */
    int SetToolCoord(int id, DescPose coord, int type, int install);

Set the tool coordinate list
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the tool coordinate list
    * @param  [in] id Frame number, range[1~15]
    * @param  [in] coord  Tool center position relative to end flange center position
    * @param  [in] type  0- tool coordinates, 1- sensor coordinates
    * @param  [in] install Installation position, 0- robot end, 1- robot outside
    * @return  Error code
    */
    int SetToolList(int id, DescPose coord, int type, int install);

Set the external tool coordinate reference point - three point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set the external tool coordinate reference point - three point method 
    * @param [in] point_num point number,range[1~3] 
    * @return Error code 
    */ 
    int SetExTCPPoint(int point_num); 

Calculation external tool coordinate - three point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Calculation external tool coordinate - three point method
    * @param [out] tcp_pose external tool coordinate
    * @return Error code
    */ 
    int ComputeExTCF(ref DescPose tcp_pose);

Set the external tool coordinate system
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the external tool coordinate system
    * @param  [in] id Frame number, range[1~15]
    * @param  [in] etcp  Tool center position relative to end flange center position
    * @param  [in] etool  To be determined
    * @return  Error code
    */
    int SetExToolCoord(int id, DescPose etcp, DescPose etool);

Set the list of external tool coordinate systems
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the list of external tool coordinate systems
    * @param  [in] id Frame number, range[1~15]
    * @param  [in] etcp  Tool center position relative to end flange center position
    * @param  [in] etool  To be determined
    * @return  Error code
    */
    int SetExToolList(int id, DescPose etcp, DescPose etool);

Set the workpiece coordinate reference point - three-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set the workpiece coordinate reference point - three-point method
    * @param [in] point_num point number,range[1~3]  
    * @return Error code
    */ 
    int SetWObjCoordPoint(int point_num); 

Calculation workpiece coordinate - three point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Calculation workpiece coordinate - three point method
    * @param [in] method Calculation method 0: origin - X-axis - Z-axis 1: origin - X-axis -xy plane 
    * @param [out] wobj_pose workpiece coordinate
    * @return Error code 
    */ 
    int ComputeWObjCoord(int method, ref DescPose wobj_pose);

Set the workpiece coordinate system
++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the workpiece coordinate system
    * @param  [in] id Frame number, range[1~15]
    * @param  [in] coord  Tool center position relative to end flange center position
    * @return  Error code
    */    
    int SetWObjCoord(int id, DescPose coord);

Set the list of work coordinate systems
++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the list of work coordinate systems
    * @param  [in] id Frame number, range[1~15]
    * @param  [in] coord  Tool center position relative to end flange center position
    * @return  Error code
    */    
    int SetWObjList(int id, DescPose coord); 

Set the end load weight
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the end load weight
    * @param  [in] weight  Load weight, unit: kg
    * @return  Error code
    */
    int SetLoadWeight(float weight);

Set the end-load centroid coordinates
+++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the end-load centroid coordinates
    * @param  [in] coord Centroid coordinates, unit: mm
    * @return  Error code
    */
    int SetLoadCoord(DescTran coord); 

Set the robot installation mode
+++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the robot installation mode
    * @param  [in] install  Installation mode: 0- formal installation, 1- side installation, 2- inverted installation
    * @return  Error code
    */
    int SetRobotInstallPos(byte install);  

Set the robot installation Angle
+++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the robot installation Angle, free installation
    * @param  [in] yangle  Angle of inclination
    * @param  [in] zangle  Angle of rotation
    * @return  Error code
    */
    int SetRobotInstallAngle(double yangle, double zangle);


Wait for the specified time
+++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Wait for the specified time
    * @param  [in]  t_ms  unit: ms
    * @return  Error code
    */
    int WaitMs(int t_ms);

Code example
+++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnCommonSets_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        int i;
        double value = 0;
        int id;
        int type;
        int install;

        DescTran coord = new DescTran();
        DescPose t_coord, etcp, etool, w_coord;
        t_coord = new DescPose();
        etcp = new DescPose();
        w_coord = new DescPose();

        robot.SetSpeed(20);

        for (i = 1; i < 21; i++)
        {
            robot.SetSysVarValue(i, (float)(i + 0.5));
            robot.WaitMs(100);
        }

        for (i = 1; i < 21; i++)
        {
            robot.GetSysVarValue(i, ref value);
            Console.WriteLine($"sys value : {value}");
        }

        robot.SetLoadWeight((float)2.5);
        coord.x = 3.0;
        coord.y = 4.0;
        coord.z = 5.0;
        robot.SetLoadCoord(coord);
                
        id = 3;
        t_coord.tran.x = 1.0;
        t_coord.tran.y = 2.0;
        t_coord.tran.z = 300.0;
        t_coord.rpy.rx = 4.0;
        t_coord.rpy.ry = 5.0;
        t_coord.rpy.rz = 6.0;
        type = 0;
        install = 0;

        int rtn1 = -1;
        int rtn2 = -1;
        rtn1 = robot.SetToolCoord(id, t_coord, type, install);
        rtn2 = robot.SetToolList(id, t_coord, type, install);
        Console.WriteLine($"set tool coord result {rtn1}, set tool list rtn{rtn2}");
            
        etcp.tran.x = 1.0;
        etcp.tran.y = 2.0;
        etcp.tran.z = 3.0;
        etcp.rpy.rx = 4.0;
        etcp.rpy.ry = 5.0;
        etcp.rpy.rz = 6.0;
        etool.tran.x = 11.0;
        etool.tran.y = 22.0;
        etool.tran.z = 330.0;
        etool.rpy.rx = 44.0;
        etool.rpy.ry = 55.0;
        etool.rpy.rz = 66.0;
        id = 5;
        robot.SetExToolCoord(id, etcp, etool);
        robot.SetExToolList(id, etcp, etool);

        w_coord.tran.x = 110.0;
        w_coord.tran.y = 12.0;
        w_coord.tran.z = 13.0;
        w_coord.rpy.rx = 14.0;
        w_coord.rpy.ry = 15.0;
        w_coord.rpy.rz = 16.0;
        id = 12;
        robot.SetWObjCoord(id, w_coord);
        //robot.SetWObjList(id, w_coord);

        double yangle = 0, zangle = 0;
        robot.SetRobotInstallPos(1);
        robot.SetRobotInstallAngle(15.0, 25.0);
        Thread.Sleep(1000);
        robot.GetRobotInstallAngle(ref yangle, ref zangle);
        Console.WriteLine($"yangle  {yangle}   zangle  {zangle}");
        robot.SetRobotInstallAngle(10.0, 10.0);
        Thread.Sleep(1000);
        robot.GetRobotInstallAngle(ref yangle, ref zangle);
        Console.WriteLine($"yangle  {yangle}   zangle  {zangle}");
    }