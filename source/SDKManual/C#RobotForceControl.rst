Machine Manpower Control
====================================

.. toctree:: 
    :maxdepth: 5

Force Sensor Configuration
++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     /**
     * @brief Configuring force sensors
     * @param [in] company Force Sensor Manufacturer, 17 - Quintessence Technologies
     * @param [in] device device number, not used, default is 0
     * @param [in] softvesion software version number, not used, the default is 0
     * @param [in] bus device hangs on the end of the bus position, do not use, the default is 0
     * @return Error code
     */
     int FT_SetConfig(int company, int device, int softvesion, int bus); 

Get the force transducer configuration 
++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     /** 
     * @brief Get force sensor configuration 
     * @param [out] deviceID force sensor number 
     * @param [out] company Force Sensor Manufacturer,, Force Sensor Manufacturer, 17-Kunwei Technology, 19-Aerospace 11th Academy, 20-ATI Sensors, 21-Zhongke MiDot, 22-Weihang Minxin
     * @param [out] device device number, Kunwei (0-KWR75B), Aisino Eleven (0-MCS6A-200-4), ATI (0-AXIA80 -M8), Zhongke MiDot (0-MST2010), Weihang Minxin (0-WHC6L-YB-10A) 
     * @param [out] softvesion software version number, not used, the default is 0 
     * @return Error code 
     */ 
     int FT_GetConfig(ref int deviceID, ref int company, ref int device, ref int softvesion); 

Force sensor activation
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     /**
     * @brief Force sensor activation
     * @param [in] act 0-reset, 1-activate
     * @return Error code.
     */
     int FT_Activate(byte act). 

Force Transducer Zeroing
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Force sensor zeroing
    * @param [in] act 0-remove zero, 1-zero correction
    * @return Error code
    */
    int FT_SetZero(byte act). 

Set the force transducer reference coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the force sensor reference coordinate system.
    * @param [in] ref 0-tool coordinate system, 1-base coordinate system
    * @return Error code.
    */
    int FT_SetRCS(byte type). 

Set the force transducer lower load weight
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the load weight under the force sensor.
    * @param [in] weight load weight kg
    * @return Error code.
    */
    int SetForceSensorPayLoad(double weight);

Set the force sensor payload center of mass
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the center of mass of a load under a force sensor.
    * @param [in] x load center of mass x mm 
    * @param [in] y load center of mass y mm
    * @param [in] z load center of mass z mm
    * @return Error code
    */
    int SetForceSensorPayLoadCog(double x, double y, double z);

Get the force sensor pay load weight
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the load weight under the force sensor.
    * @param [in] weight load weight kg
    * @return Error code.
    */
    int GetForceSensorPayLoad(ref double weight).

Get force sensor payload center of mass
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the center of mass of the load under the force transducer.
    * @param [out] x load center of mass x mm 
    * @param [out] y load center of mass y mm
    * @param [out] z load center of mass z mm
    * @return Error code
    */
    int GetForceSensorPayLoadCog(ref double x, ref double y, ref double z);

Automatic zeroing of the force sensor.
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Force sensor auto-zero
    * @param [out] weight Sensor mass kg 
    * @param [out] pos sensor center of mass mm
    * @return Error code
    */
    int ForceSensorAutoComputeLoad(ref double weight, ref DescTran pos);

Get force/torque data in reference coordinate system.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get force/torque data in reference coordinate system.
    * @param [out] ft force/torque, fx,fy,fz,tx,ty,tz
    * @return Error code
    */    
    int FT_GetForceTorqueRCS(byte flag, ref ForceTorque ft); 

Get force sensor raw force/torque data
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get force sensor raw force/torque data.
    * @param [out] ft force/torque, fx,fy,fz,tx,ty,tz
    * @return Error code.
    */    
    int FT_GetForceTorqueOrigin(byte flag, ref ForceTorque ft); 

Force Transducer Configuration and Auto-Zero Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     private void button54_Click(object sender, EventArgs e)
    {
        int company = 24;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int index = 1;

        robot.FT_SetConfig(company, device, softversion, bus);
        Thread.Sleep(1000);
        robot.FT_GetConfig(ref company, ref device, ref softversion, ref bus);
        Console.WriteLine($"FT config:{company},{device},{softversion},{bus}");
        Thread.Sleep(1000);

        robot.FT_Activate(0);
        Thread.Sleep(1000);
        robot.FT_Activate(1);
        Thread.Sleep(1000);

        Thread.Sleep(1000);
        robot.FT_SetZero(0);
        Thread.Sleep(1000);

        ForceTorque ft = new ForceTorque(0, 0, 0, 0, 0, 0);
        robot.FT_GetForceTorqueOrigin(0, ref ft);
        Console.WriteLine($"ft origin:{ft.fx},{ft.fy},{ft.fz},{ft.tx},{ft.ty},{ft.tz}");
        robot.FT_SetZero(1);
        Thread.Sleep(1000);

        DescPose ftCoord = new DescPose(0, 0, 0, 0, 0, 0);
        robot.FT_SetRCS(0, ftCoord);

        robot.SetForceSensorPayLoad(0.824);
        robot.SetForceSensorPayLoadCog(0.778, 2.554, 48.765);
        double weight = 0;
        double x = 0, y = 0, z = 0;
        robot.GetForceSensorPayLoad(ref weight);
        robot.GetForceSensorPayLoadCog(ref x, ref y, ref z);
        Console.WriteLine($"the FT load is {weight}, {x} {y} {z}");

        robot.SetForceSensorPayLoad(0);
        robot.SetForceSensorPayLoadCog(0, 0, 0);

        double computeWeight = 0;
        DescTran tran = new DescTran(0, 0, 0);
        robot.ForceSensorAutoComputeLoad(ref weight, ref tran);
        Console.WriteLine($"the result is weight {weight} pos is {tran.x} {tran.y} {tran.z}");

    }

Load Weight Recognition Record
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Load weight recognition record.
    * @param [in] id Sensor coordinate system number in the range [1~14].
    * @return Error code.
    */
    int FT_PdIdenRecord(int id).

Load weight recognition calculation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Load Weight Recognition Calculation
    * @param [out] weight Weight of load in kg.
    * @return Error code
    */    
    int FT_PdIdenCompute(ref double weight).

Load center of mass identification record
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Load center-of-mass identification record.
    * @param [in] id Sensor coordinate system number, range [1~14].
    * @param [in] index point number, range [1~3].
    * @return Error code
    */
    int FT_PdCogIdenRecord(int id, int index). 

Load center of mass identification calculation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Load center of mass identification calculation.
    * @param [out] cog load center of mass in mm.
    * @return Error code.
    */    
    int FT_PdCogIdenCompute(ref DescTran cog);

Force Transducer Load Recognition Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     private void btnFTPdCog_Click(object sender, EventArgs e)
    {
        int company = 24, device = 0, softversion = 0, bus = 1;

        robot.FT_SetConfig(company, device, softversion, bus);
        Thread.Sleep(1000);
        robot.FT_GetConfig(ref company, ref device, ref softversion, ref bus);
        Console.WriteLine($"FT config: {company}, {device}, {softversion}, {bus}");
        Thread.Sleep(1000);

        robot.FT_Activate(0);
        Thread.Sleep(1000);
        robot.FT_Activate(1);
        Thread.Sleep(1000);

        Thread.Sleep(1000);
        robot.FT_SetZero(0);
        Thread.Sleep(1000);

        ForceTorque ft = new ForceTorque(0,0,0,0,0,0);
        robot.FT_GetForceTorqueOrigin(0, ref ft);
        Console.WriteLine($"ft origin: {ft.fx}, {ft.fy}, {ft.fz}, {ft.tx}, {ft.ty}, {ft.tz}");
        robot.FT_SetZero(1);
        Thread.Sleep(1000);

        DescPose tcoord = new DescPose(0, 0, 35.0, 0, 0, 0);
        robot.SetToolCoord(10, tcoord, 1, 0, 0, 0);

        robot.FT_PdIdenRecord(10);
        Thread.Sleep(1000);

        double weight = 0.0f;
        robot.FT_PdIdenCompute(ref weight);
        Console.WriteLine($"payload weight: {weight}");

        DescPose desc_p1 = new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_p2 = new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose desc_p3 = new DescPose(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);

        robot.MoveCart( desc_p1, 0, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        Thread.Sleep(1000);
        robot.FT_PdCogIdenRecord(10, 1);
        robot.MoveCart( desc_p2, 0, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        Thread.Sleep(1000);
        robot.FT_PdCogIdenRecord(10, 2);
        robot.MoveCart( desc_p3, 0, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        Thread.Sleep(1000);
        robot.FT_PdCogIdenRecord(10, 3);

        DescTran cog = new DescTran(0,0,0);
        robot.FT_PdCogIdenCompute(ref cog);
        Console.WriteLine($"cog: {cog.x}, {cog.y}, {cog.z}");
    }

Collision Guard
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief collision guarding
    * @param [in] flag 0- turn off collision guarding, 1- turn on collision guarding
    * @param [in] sensor_id Force sensor number.
    * @param [in] select Whether to detect collision in six degrees of freedom, 0-no detection, 1-detection.
    * @param [in] ft collision force/torque, fx,fy,fz,tx,ty,tz
    * @param [in] max_threshold max_threshold
    * @param [in] min_threshold min_threshold
    * @note Force/torque detection range: (ft-min_threshold, ft+max_threshold)
    * @return Error code
    */    
    int FT_Guard(int flag, int sensor_id, int[] select, ForceTorque ft, double[] max_threshold, double[] min_threshold); 

Collision Guard Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     private void btnFTGuard_Click(object sender, EventArgs e)
    {
        int company = 24, device = 0, softversion = 0, bus = 1;

        robot.FT_SetConfig(company, device, softversion, bus);
        Thread.Sleep(1000);
        robot.FT_GetConfig(ref company, ref device, ref softversion, ref bus);
        Console.WriteLine($"FT config: {company}, {device}, {softversion}, {bus}");
        Thread.Sleep(1000);

        robot.FT_Activate(0);
        Thread.Sleep(1000);
        robot.FT_Activate(1);
        Thread.Sleep(1000);

        Thread.Sleep(1000);
        robot.FT_SetZero(0);
        Thread.Sleep(1000);

        byte sensor_id = 1;
        int[] select = { 1, 1, 1, 1, 1, 1 };
        double[] max_threshold = { 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f };
        double[] min_threshold = { 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f };

        ForceTorque ft = new ForceTorque();
        DescPose desc_p1 = new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_p2 = new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose desc_p3 = new DescPose(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);

        robot.FT_Guard(1, sensor_id, select,  ft, max_threshold, min_threshold);
        robot.MoveCart( desc_p1, 0, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        robot.MoveCart( desc_p2, 0, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        robot.MoveCart( desc_p3, 0, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);

        robot.FT_Guard(0, sensor_id, select, ft, max_threshold, min_threshold);
    }

Constant force control
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    /**
    * @brief  Constant Force Control
    * @param  [in] flag 0-Disable constant force control, 1-Enable constant force control
    * @param  [in] sensor_id Force sensor ID
    * @param  [in] select  Select whether to detect collision for the six degrees of freedom, 0-Do not detect, 1-Detect
    * @param  [in] ft  Collision force/torque, fx,fy,fz,tx,ty,tz
    * @param  [in] ft_pid Force PID parameters, torque PID parameters
    * @param  [in] adj_sign Adaptive start/stop control, 0-Disable, 1-Enable
    * @param  [in] ILC_sign ILC start/stop control, 0-Stop, 1-Training, 2-Operation
    * @param  [in] max_dis Maximum adjustment distance, unit mm
    * @param  [in] max_ang Maximum adjustment angle, unit deg
    * @param  [in] M rx, ry mass parameters [0.1-10], default 2
    * @param  [in] B rx, ry damping parameters [0.1-50], default 8
    * @param  [in] threshold rx, ry activation thresholds [0-10], default 0.2
    * @param  [in] adjustCoeff rx, ry torque adjustment coefficients [0-1], default 1
    * @param  [in] polishRadio Polishing radius, unit mm
    * @param  [in] filter_Sign Filter enable flag 0-Off; 1-On, default off
    * @param  [in] posAdapt_sign Pose compliance enable flag 0-Off; 1-On, default off
    * @param  [in] isNoBlock Blocking flag, 0-Blocking; 1-Non-blocking
    * @return  Error code
    */
    public int FT_Control(byte flag, int sensor_id, byte[] select, ForceTorque ft, float[] ft_pid,byte adj_sign, byte ILC_sign, float max_dis, float max_ang,double[] M, double[] B, double[] threshold, double[] adjustCoeff,double polishRadio, int filter_Sign, int posAdapt_sign, int isNoBlock)

Constant force control with damping code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    public void TestFTControlWithAdjustCoeff()
    {
        int rtn;
        int sensor_id = 10;
        byte[] select = new byte[6] { 0, 0, 1, 0, 0, 0 };
        float[] ft_pid = new float[6] { 0.0008f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        byte adj_sign = 0;
        byte ILC_sign = 0;
        float max_dis = 1000.0f;
        float max_ang = 20.0f;
        ForceTorque ft = new ForceTorque();
        ft.fz = -10.0f;
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        JointPos j1 = new JointPos(80.765f, -98.795f, 106.548f, -97.734f, -89.999f, 94.842f);
        JointPos j2 = new JointPos(43.067f, -84.429f, 92.620f, -98.175f, -90.011f, 57.144f);
        DescPose desc_p1 = new DescPose(5.009f, -547.463f, 262.053f, -179.999f, -0.019f, 75.923f);
        DescPose desc_p2 = new DescPose(-347.966f, -547.463f, 262.048f, -180.000f, -0.019f, 75.923f);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        double[] M = new double[2] { 2.0, 2.0 };
        double[] B = new double[2] { 15.0, 15.0 };
        double[] threshold = new double[2] { 1.0, 1.0 };
        double[] adjustCoeff = new double[2] { 1.0, 0.8 };
        double polishRadio = 0.0;
        int filter_Sign = 0;
        int posAdapt_sign = 1;
        int isNoBlock = 0;
        while (true)
        {
            rtn = robot.FT_Control(1, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, threshold, adjustCoeff, 0, 0, 1, 0);
            Console.WriteLine($"FT_Control start rtn is {rtn}");
            robot.MoveL(j1, desc_p1, 1, 0, 100, 100, 100, -1, 0, epos, 0, 0, offset_pos, 0, 0, 10);
            robot.MoveL(j2, desc_p2, 1, 0, 100, 100, 100, -1, 0, epos, 0, 0, offset_pos, 0, 0, 10);
            rtn = robot.FT_Control(0, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, threshold, adjustCoeff, 0, 0, 1, 0);
            Console.WriteLine($"FT_Control end rtn is {rtn}");
        }
    }

Rotational Insertion
+++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Rotational Insertion
    * @param [in] rcs Reference coordinate system, 0 - Tool coordinate system, 1 - Base coordinate system
    * @param [in] angVelRot Rotational angular velocity, unit deg/s
    * @param [in] ft Force/Torque threshold, fx,fy,fz,tx,ty,tz, range [0~100]
    * @param [in] max_angle Maximum rotation angle, unit deg
    * @param [in] orn Force/Torque direction, 1 - Along Z-axis direction, 2 - Around Z-axis direction
    * @param [in] max_angAcc Maximum rotational acceleration, unit deg/s^2, currently unused, default is 0
    * @param [in] rotorn Rotation direction, 1 - Clockwise, 2 - Counterclockwise
    * @param [in] strategy Processing strategy for undetected force/torque, 0 - Error; 1 - Warning, continue motion
    * @return Error code
    */
    public int FT_RotInsertion(int rcs, double angVelRot, double ft, double max_angle, int orn, double max_angAcc, int rotorn, int strategy)

Spiral Search
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief  Spiral search
    * @param  [in] rcs Reference coordinate system, 0-tool coordinate system, 1-base coordinate system
    * @param  [in] dr Radius feed per revolution
    * @param  [in] ft Force/torque threshold, fx,fy,fz,tx,ty,tz, range [0~100]
    * @param  [in] max_t_ms Maximum search time, unit ms
    * @param  [in] max_vel Maximum linear velocity, unit mm/s
    * @param  [in] strategy Handling strategy when no force/torque is detected, 0-error; 1-warning, continue motion
    * @return  Error code
    */
    public int FT_SpiralSearch(int rcs, float dr, float ft, float max_t_ms, float max_vel, int strategy = 0)

Linear Insertion
+++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief  Linear insertion
    * @param  [in] rcs Reference coordinate system, 0-tool coordinate system, 1-base coordinate system
    * @param  [in] ft  Force/torque threshold, fx,fy,fz,tx,ty,tz, range [0~100]
    * @param  [in] lin_v Linear speed, unit mm/s
    * @param  [in] lin_a Linear acceleration, unit mm/s^2, not used temporarily
    * @param  [in] max_dis Maximum insertion distance, unit mm
    * @param  [in] linorn  Insertion direction, 0-negative direction, 1-positive direction
    * @param  [in] strategy Handling strategy when no force/torque is detected, 0-error; 1-warning, continue motion
    * @return  Error code
    */
    public int FT_LinInsertion(int rcs, float ft, float lin_v, float lin_a, float max_dis, byte linorn, int strategy=0)

Force Sensor Rotational Insertion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    public void TestRotInsert()
    {
        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
        int rtn;

        float forceInsertion = 5.0f; // Force or torque threshold (0~100), unit N or Nm
        int angleMax = 300; // Maximum rotation angle, unit °
        byte orn = 1; // Force direction, 1-fz, 2-mz
        float angAccmax = 0; // Maximum rotational angular acceleration, unit °/s^2, not used temporarily
        byte status = 1;  // Constant force control enable flag, 0-off, 1-on
        int sensor_num = 11; // Force sensor number
        float[] gain = { 0.0001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };  // Maximum threshold
        byte adj_sign = 0;  // Adaptive start/stop status, 0-off, 1-on
        byte ILC_sign = 0;  // ILC control start/stop status, 0-stop, 1-training, 2-operational
        float max_dis = 1000.0f;  // Maximum adjustment distance
        float max_ang = 20.0f;  // Maximum adjustment angle
        ForceTorque ft = new ForceTorque();
        int rcs = 0;  // Reference coordinate system, 0-tool coordinate system, 1-base coordinate system
        float angVelRot = 1.0f;  // Rotational angular velocity, unit °/s
        byte rotorn = 1; // Rotation direction, 1-clockwise, 2-counterclockwise
        JointPos j1 = new JointPos(100.968, -108.678, 126.166, -106.630, -93.253, 19.584);
        DescPose desc_p1 = new DescPose(159.473, -316.570, 334.560, -179.718, -3.352, 171.400);
        ExaxisPos epos = new ExaxisPos(0.0f, 0.0f, 0.0f, 0.0f);
        DescPose offset_pos = new DescPose(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        robot.MoveL(j1, desc_p1, 2, 0, 100.0f, 180.0f, 100.0f, -1.0f, 0, epos, (byte)0, (byte)1, offset_pos);

        byte[] select3 = { 0, 0, 1, 0, 0, 0 };
        ft.fz = -5.0f;
        gain[0] = 0.0001f;
        status = 1;
        robot.FT_Control(status, sensor_num, select3, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
        rtn = robot.FT_LinInsertion(rcs, 10, 1, 1, 100, 1);
        Console.WriteLine("FT_LinInsertion rtn is " + rtn);
        robot.FT_Control(0, sensor_num, select3, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

        ft.fz = -30.0f;
        robot.FT_Control(1, sensor_num, select3, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
        rtn = robot.FT_RotInsertion(rcs, angVelRot, forceInsertion, angleMax, orn, angAccmax, rotorn, 0);
        Console.WriteLine("FT_RotInsertion rtn is " + rtn);
        robot.FT_Control(0, sensor_num, select3, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

        rtn = robot.FT_LinInsertion(0, 40, 3, 0, 100, 1);
        Console.WriteLine("FT_LinInsertion retract rtn is " + rtn);

        Thread.Sleep(1000);
        robot.GetRobotRealTimeState(ref pkg);
        Console.WriteLine("robot errcode " + pkg.main_code + "  " + pkg.sub_code);
    }
    
Robot Force Sensor Rotational Insertion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    public void TestMove()
    {
        int rtn;
        JointPos j1 = new JointPos(-11.904f, -99.669f, 117.473f, -108.616f, -91.726f, 74.256f);
        JointPos j2 = new JointPos(-45.615f, -106.172f, 124.296f, -107.151f, -91.282f, 74.255f);
        JointPos j3 = new JointPos(-29.777f, -84.536f, 109.275f, -114.075f, -86.655f, 74.257f);
        JointPos j4 = new JointPos(-31.154f, -95.317f, 94.276f, -88.079f, -89.740f, 74.256f);
        DescPose desc_pos1 = new DescPose(-419.524f, -13.000f, 351.569f, -178.118f, 0.314f, 3.833f);
        DescPose desc_pos2 = new DescPose(-321.222f, 185.189f, 335.520f, -179.030f, -1.284f, -29.869f);
        DescPose desc_pos3 = new DescPose(-487.434f, 154.362f, 308.576f, 176.600f, 0.268f, -14.061f);
        DescPose desc_pos4 = new DescPose(-443.165f, 147.881f, 480.951f, 179.511f, -0.775f, -15.409f);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float oacc = 100.0f;
        float blendT = 0.0f;
        float blendR = 0.0f;
        byte flag = 0;
        byte search = 0;
        int blendMode = 0;
        int velAccMode = 0;
        robot.SetSpeed(20);
        rtn = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Console.WriteLine($"movej errcode:{rtn}");
        rtn = robot.MoveL(j2, desc_pos2, tool, user, vel, acc, ovl, blendR, blendMode, epos, search, flag, offset_pos, oacc, velAccMode,0,10);
        Console.WriteLine($"movel errcode:{rtn}");
        rtn = robot.MoveC(j3, desc_pos3, tool, user, vel, acc, epos, flag, offset_pos,j4, desc_pos4, tool, user, vel, acc, epos, flag, offset_pos, ovl, blendR, oacc, velAccMode);
        Console.WriteLine($"movec errcode:{rtn}");
        rtn = robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Console.WriteLine($"movej errcode:{rtn}");
        rtn = robot.Circle(j3, desc_pos3, tool, user, vel, acc, epos,j1, desc_pos1, tool, user, vel, acc, epos,ovl, flag, offset_pos, oacc, -1, velAccMode);
        Console.WriteLine($"circle errcode:{rtn}");
        rtn = robot.MoveCart(desc_pos4, tool, user, vel, acc, ovl, blendT, -1);
        Console.WriteLine($"MoveCart errcode:{rtn}");
        rtn = robot.MoveJ(j1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Console.WriteLine($"movej errcode:{rtn}");
        rtn = robot.MoveL(desc_pos2, tool, user, vel, acc, ovl, blendR, blendMode, epos, search, flag, offset_pos, -1, velAccMode);
        Console.WriteLine($"movel errcode:{rtn}");
        rtn = robot.MoveC(desc_pos3, tool, user, vel, acc, epos, flag, offset_pos,desc_pos4, tool, user, vel, acc, epos, flag, offset_pos,ovl, blendR, -1, velAccMode);
        Console.WriteLine($"movec errcode:{rtn}");
        rtn = robot.MoveJ(j2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Console.WriteLine($"movej errcode:{rtn}");
        rtn = robot.Circle(desc_pos3, tool, user, vel, acc, epos, desc_pos1, tool, user, vel, acc, epos,ovl, flag, offset_pos, oacc, blendR, -1, velAccMode);
        Console.WriteLine($"circle errcode:{rtn}");
    }

Flex control on
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Smooth control on
    * @param [in] p Position adjustment coefficient or softening coefficient.
    * @param [in] force Soft opening force threshold in N
    * @return Error Code
    */    
    int FT_ComplianceStart(float p, float force);

Flex control off
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Soft control off
    * @return Error code
    */    
    int FT_ComplianceStop(); 

Sample Flex Control Code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     private void btnComplience_Click(object sender, EventArgs e)
    {
        int company = 24, device = 0, softversion = 0, bus = 1;
        robot.FT_SetConfig(company, device, softversion, bus);
        Thread.Sleep(1000);
        robot.FT_GetConfig(ref company, ref device, ref softversion, ref bus);
        Console.WriteLine($"FT config: {company}, {device}, {softversion}, {bus}");
        Thread.Sleep(1000);

        robot.FT_Activate(0);
        Thread.Sleep(1000);
        robot.FT_Activate(1);
        Thread.Sleep(1000);

        robot.FT_SetZero(0);
        Thread.Sleep(1000);

        byte flag = 1;
        int sensor_id = 1;
        int[] select = { 1, 1, 1, 0, 0, 0 };
        double[] ft_pid = { 0.0005f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        byte adj_sign = 0, ILC_sign = 0;
        float max_dis = 100.0f, max_ang = 0.0f;

        ForceTorque ft = new ForceTorque { fx = -10.0, fy = -10.0, fz = -10.0 };
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);

        JointPos j1 = new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2 = new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        DescPose desc_p1 = new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_p2 = new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

        robot.FT_Control(flag, (byte)sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);
        float p = 0.00005f;
        float force = 30.0f;
        int rtn = robot.FT_ComplianceStart(p, force);
        Console.WriteLine($"FT_ComplianceStart rtn is {rtn}");

        int count = 5;
        while (count-- > 0)
        {
        robot.MoveL(j1, desc_p1, 0, 0, 100.0f, 180.0f, 100.0f, -1.0f, epos, 0, 1, offset_pos);
        robot.MoveL(j2, desc_p2, 0, 0, 100.0f, 180.0f, 100.0f, -1.0f, epos, 0, 0, offset_pos);
        }

        robot.FT_ComplianceStop();
        Console.WriteLine($"FT_ComplianceStop rtn is {rtn}");

        flag = 0;
        robot.FT_Control(flag, (byte)sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);
    }

Load recognition initialization
++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Load recognition initialization.
    * @return Error code
    */
    int LoadIdentifyDynFilterInit();

Load identification variable initialization
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Initialization of load recognition variables.
    * @return Error code
    */
    int LoadIdentifyDynVarInit();

Load Identification Main Program
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Load recognition main program.
    * @param [in] joint_torque Joint torque.
    * @param [in] joint_pos Joint position.
    * @param [in] t Sampling period.
    * @return Error code
    */
    int LoadIdentifyMain(double[] joint_torque, double[] joint_pos, double t);

Get the load identification result
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Get load recognition results
    * @param [in] gain Gravity term coefficient double[6], centrifugal term coefficient double[6].
    * @param [out] weight load weight
    * @param [out] cog load center of mass
    * @return error code
    */
    int LoadIdentifyGetResult(double[] gain, ref double weight, ref DescTran cog);

Robot Load Identification Code Example
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button74_Click(object sender, EventArgs e)
    {
        int rtn;
        int retval = 0;

        retval = robot.LoadIdentifyDynFilterInit();
        Console.WriteLine("LoadIdentifyDynFilterInit retval is: " + retval);

        retval = robot.LoadIdentifyDynVarInit();
        Console.WriteLine("LoadIdentifyDynVarInit retval is: " + retval);

        JointPos posJ = new JointPos(0,0,0,0,0,0);
        DescPose posDec = new DescPose(0, 0, 0, 0, 0, 0);
        double[] joint_toq = new double[6] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        robot.GetActualJointPosDegree(0, ref posJ);
        posJ.jPos[1] = posJ.jPos[1] + 10;
        robot.GetJointTorques(0, joint_toq);
        joint_toq[1] = joint_toq[1] + 2;

        double[] tmpTorque = new double[6] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        for (int i = 0; i < 6; i++)
        {
            tmpTorque[i] = joint_toq[i];
        }

        retval = robot.LoadIdentifyMain(tmpTorque, posJ.jPos, 1);
        Console.WriteLine("LoadIdentifyMain retval is: " + retval);

        double[] gain = new double[12] { 0, 0.05, 0, 0, 0, 0, 0, 0.02, 0, 0, 0, 0 };
        double weight = 0;
        DescTran load_pos = new DescTran(0, 0, 0);
        retval = robot.LoadIdentifyGetResult(gain, ref weight, ref load_pos);
        Console.WriteLine("LoadIdentifyGetResult retval is: {0}; weight is {1} cog is {2} {3} {4}", retval, weight, load_pos.x, load_pos.y, load_pos.z);
    }

Force sensor assisted drag
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.4  Web-3.8.3
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Force sensor assisted drag
    * @param [in] status control status, 0-off; 1-on
    * @param [in] asaptiveFlag Adaptive on flag, 0-off; 1-on
    * @param [in] interfereDragFlag Interference area drag flag, 0-off; 1-on
    * @param [in] ingularityConstraintsFlag singularity strategy, 0-avoidance; 1-crossing
    * @param [in] forceCollisionFlag Robot collision detection flag during auxiliary drag; 0-off; 1-on
    * @param [in] M inertia coefficient
    * @param [in] B Damping coefficient
    * @param [in] K stiffness coefficient
    * @param [in] F Drag six-dimensional force threshold
    * @param [in] Fmax Maximum drag force limit Nm
    * @param [in] Vmax Maximum joint speed limit °/s
    * @return Error code
    */
    int EndForceDragControl(int status, int asaptiveFlag, int interfereDragFlag,int ingularityConstraintsFlag,int forceCollisionFlag, double[] M , double[] B, double[] K, double[] F, double Fmax, double Vmax);
    
Get the state of the force sensor drag switch
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get force sensor drag switch state.
    * @param [out] dragState force sensor assist drag control state, 0-off; 1-on
    * @param [out] sixDimensionalDragState sixDimensionalForceAssistedDragState, 0-off; 1-on
    * @return ErrorCode
    */
    int GetForceAndTorqueDragState(ref int dragState, ref int sixDimensionalDragState);

The force sensor is automatically turned on after the error is cleared
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief The force sensor is automatically turned on after an error is cleared.
    * @param [in] status Control status, 0-off, 1-on.
    * @return Error code
    */
    int SetForceSensorDragAutoFlag(int status);

Force Sensor Assisted Drag Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     private void button61_Click(object sender, EventArgs e)
    {
        robot.SetForceSensorDragAutoFlag(1);
        double[] M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
        double[] B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
        double[] K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        double[] F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };

        robot.EndForceDragControl(1, 0, 0, 0, M, B, K, F, 50, 100);
        robot.WaitMs(5000);

        int dragState = 0;
        int sixDimensionalDragState = 0;
        robot.GetForceAndTorqueDragState(ref dragState, ref sixDimensionalDragState);
        Console.WriteLine($"the drag state is {dragState} {sixDimensionalDragState}");

        robot.EndForceDragControl(0, 0, 0, 0, M, B, K, F, 50, 100);
    }

Setting up the six-dimensional force and joint impedance hybrid drag switch and parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting up a six-dimensional force and joint impedance hybrid drag switch and parameters.
    * @param [in] status Control status, 0-off, 1-on.
    * @param [in] impedanceFlag impedance on flag, 0-off; 1-on
    * @param [in] lamdeDain drag gain
    * @param [in] KGain Stiffness Gain
    * @param [in] BGain damping gain
    * @param [in] dragMaxTcpVel Drag End Max Linear Velocity Limit
    * @param [in] dragMaxTcpOriVel Maximum angular velocity limit at drag end
    * @return Error code
    */
    int ForceAndJointImpedanceStartStop(int status, impedanceFlag, double[] lamdeDain, double[] KGain, double[] BGain, double dragMaxTcpVel, double dragMaxTcpOriVel);

Force Sensor Assisted Drag Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     private void button62_Click(object sender, EventArgs e)
    {
        robot.DragTeachSwitch(1);
        double[] lambdaGain = { 3.0, 2.0, 2.0, 2.0, 2.0, 3.0 };
        double[] kGain = { 0, 0, 0, 0, 0, 0 };
        double[] bGain = { 150, 150, 150, 5.0, 5.0, 1.0 };
        int rtn = robot.ForceAndJointImpedanceStartStop(1, 0, lambdaGain, kGain, bGain, 1000, 180);
        Console.WriteLine($"ForceAndJointImpedanceStartStop rtn is {rtn}");
        Thread.Sleep(5000); 
        robot.DragTeachSwitch(0);
        rtn = robot.ForceAndJointImpedanceStartStop(0, 0, lambdaGain, kGain, bGain, 1000, 180);
        Console.WriteLine($"ForceAndJointImpedanceStartStop rtn is {rtn}");
    }

Setting up the Wire Seek Expansion IO Port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting up a wire-seeking extended IO port.
    * @param searchDoneDINum Successful DO port (0-127) for wire seek.
    * @param searchStartDONum The start/stop control DO port (0-127).
    * @return Error Code
    */
    int SetWireSearchExtDIONum(int searchDoneDINum, int searchStartDONum);

Impedance Control Start/Stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Impedance Control Start/Stop
    * @param [in] status 0: disable; 1-enable
    * @param [in] workSpace 0-joint space; 1-Cartesian space
    * @param [in] forceThreshold Trigger force threshold (N)
    * @param [in] m Mass parameter
    * @param [in] b Damping parameter
    * @param [in] k Stiffness parameter
    * @param [in] maxV Max linear velocity (mm/s)
    * @param [in] maxVA Max linear acceleration (mm/s2)
    * @param [in] maxW Max angular velocity (°/s)
    * @param [in] maxWA Max angular acceleration (°/s2)
    * @return Error code
    */
    public int ImpedanceControlStartStop(int status, int workSpace, double[] forceThreshold, double[] m, double[] b, double[] k, double maxV, double maxVA, double maxW, double maxWA)

Robot Impedance Control Start/Stop Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    public void TestImpedanceControl()
    { 
        int[] ctrl = new int[20];
        int state;
        int pressValue;
        int error;
        int rtn;
        JointPos j1 = new JointPos(102.622, -135.990, 120.769, -73.950, -90.848, 35.507);
        JointPos j2 = new JointPos(93.674, -80.062, 82.947, -92.199, -90.967, 26.559);
        DescPose desc_pos1 = new DescPose(136.552, -149.799, 449.532, 179.817, -1.172, 157.123);
        DescPose desc_pos2 = new DescPose(136.540, -561.048, 449.542, 179.819, -1.172, 157.122);
    
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
    
        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 200.0f;
        float ovl = 100.0f;
        float blendT = -1.0f;
        float blendR = -1.0f;
    
        byte flag = 0;
    
        byte search = 0;
        robot.SetSpeed(20);
        int company = 17;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        robot.FT_SetConfig(company, device, softversion, bus);
        Thread.Sleep(1000);
        robot.FT_GetConfig(ref company, ref device, ref softversion, ref bus);
        Console.WriteLine($"FT config:{company},{device},{softversion},{bus}");
        Thread.Sleep(1000);
    
        robot.FT_Activate(0);
        Thread.Sleep(1000);
        robot.FT_Activate(1);
        Thread.Sleep(1000);
        Thread.Sleep(1000);
        robot.FT_SetZero(0);
        Thread.Sleep(1000);
        robot.FT_SetZero(1);
        Thread.Sleep(1000);
    
        double[] forceThreshold = new double[] { 30, 30, 30, 5, 5, 5 };
        double[] m = new double[] { 0.1, 0.1, 0.1, 0.02, 0.02, 0.02 };
        double[] b = new double[] { 1, 1, 1, 0.08, 0.08, 0.08 };
        double[] k = new double[] { 0, 0, 0, 0, 0, 0 };
        rtn = robot.ImpedanceControlStartStop(1, 1, forceThreshold, m, b, k, 1000, 500, 100, 100);
        Console.WriteLine($"ImpedanceControlStartStop errcode:{rtn}");
        rtn = robot.MoveL(desc_pos1, tool, user, vel, acc, ovl, blendR, 0, epos, search, flag, offset_pos, -1, 0);
        rtn = robot.MoveL(desc_pos2, tool, user, vel, acc, ovl, blendR, 0, epos, search, flag, offset_pos, -1, 0);
        rtn = robot.MoveL(desc_pos1, tool, user, vel, acc, ovl, blendR, 0, epos, search, flag, offset_pos, -1, 0);
        rtn = robot.MoveL(desc_pos2, tool, user, vel, acc, ovl, blendR, 0, epos, search, flag, offset_pos, -1, 0);
        rtn = robot.MoveL(desc_pos1, tool, user, vel, acc, ovl, blendR, 0, epos, search, flag, offset_pos, -1, 0);
        rtn = robot.MoveL(desc_pos2, tool, user, vel, acc, ovl, blendR, 0, epos, search, flag, offset_pos, -1, 0);
        Console.WriteLine($"movel errcode:{rtn}");
        robot.ImpedanceControlStartStop(0, 1, forceThreshold, m, b, k, 1000, 500, 100, 100);
    }

Enable torque compensation function and compensation coefficient
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Enable torque compensation function and compensation coefficient
    * @param [in] status Switch, 0-Disable; 1-Enable
    * @param [in] torqueCoeff J1-J6 torque compensation coefficient [0-1]
    * @return Error code
    */
    public int SetCoderCompenParams(int status, double[] torqueCoeff)

Surface Positioning
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief  Surface positioning
    * @param  [in] rcs Reference coordinate system, 0-tool coordinate system, 1-base coordinate system
    * @param  [in] dir  Movement direction, 1-positive direction, 2-negative direction
    * @param  [in] axis Movement axis, 1-x axis, 2-y axis, 3-z axis
    * @param  [in] lin_v Search linear speed, unit mm/s
    * @param  [in] lin_a Search linear acceleration, unit mm/s^2, not used temporarily, default 0
    * @param  [in] max_dis Maximum search distance, unit mm
    * @param  [in] ft  Motion termination force/torque threshold, fx,fy,fz,tx,ty,tz
    * @param  [in] strategy Handling strategy when no force/torque is detected, 0-error; 1-warning, continue motion
    * @return  Error code
    */
    public int FT_FindSurface(int rcs, byte dir, byte axis, float lin_v, float lin_a, float max_dis, float ft, int stragety = 0)

Calculate Mid-Plane Position Start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief  Calculate mid-plane position start
    * @return  Error code
    */
    public int FT_CalCenterStart()

Calculate Mid-Plane Position End
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief  Calculate mid-plane position end
    * @param  [out] pos Mid-plane pose
    * @return  Error code
    */
    public int FT_CalCenterEnd(ref DescPose pos)

Surface Positioning Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    private void button59_Click(object sender, EventArgs e)
    {
        int company = 22;
        int device = 0;
        int softversion = 0;
        int bus = 1;

        robot.FT_SetConfig(company, device, softversion, bus);
        Thread.Sleep(1000);
        robot.FT_GetConfig(ref company, ref device, ref softversion, ref bus);
        Console.WriteLine("FT config:" + company + "," + device + "," + softversion + "," + bus);
        Thread.Sleep(1000);

        robot.FT_Activate(0);
        Thread.Sleep(1000);
        robot.FT_Activate(1);
        Thread.Sleep(1000);

        Thread.Sleep(1000);
        robot.FT_SetZero(0);
        Thread.Sleep(1000);

        int rcs = 0;
        byte dir = 1;
        byte axis = 1;
        float lin_v = 15.0f;
        float lin_a = 0.0f;
        float maxdis = 500.0f;
        float ft_goal = 2.0f;
        DescPose desc_pos = new DescPose(-419.524f, -13.000f, 351.569f, -178.118f, 0.314f, 3.833f);
        DescPose xcenter = new DescPose(0, 0, 0, 0, 0, 0);
        DescPose ycenter = new DescPose(0, 0, 0, 0, 0, 0);

        ForceTorque ft = new ForceTorque();

        ft.fx = -2.0f;

        robot.MoveCart(desc_pos, 1, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);

        robot.FT_CalCenterStart();
        robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
        robot.MoveCart(desc_pos, 1, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        robot.WaitMs(1000);

        dir = 2;
        robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
        robot.FT_CalCenterEnd(ref xcenter);
        Console.WriteLine("xcenter:" + xcenter.tran.x + "," + xcenter.tran.y + "," + xcenter.tran.z + "," + xcenter.rpy.rx + "," + xcenter.rpy.ry + "," + xcenter.rpy.rz);
        robot.MoveCart(xcenter, 1, 0, 60.0f, 50.0f, 50.0f, -1.0f, -1);

        robot.FT_CalCenterStart();
        dir = 1;
        axis = 2;
        lin_v = 6.0f;
        maxdis = 150.0f;
        robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
        robot.MoveCart(desc_pos, 1, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        robot.WaitMs(1000);

        dir = 2;
        robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
        robot.FT_CalCenterEnd(ref ycenter);
        Console.WriteLine("ycenter:" + ycenter.tran.x + "," + ycenter.tran.y + "," + ycenter.tran.z + "," + ycenter.rpy.rx + "," + ycenter.rpy.ry + "," + ycenter.rpy.rz);
        robot.MoveCart(ycenter, 1, 0, 60.0f, 50.0f, 50.0f, 0.0f, -1);

    }

Set Real-Time Weave Offset
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set real-time weave offset
    * @param [in] offset Real-time offset [mm, °]
    * @return  Error code
    */
    public int SetWeaveOffsetRT(DescPose offset)

Real-Time Weave Speed and Offset Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:   

    public void TestWeaveSpeedAndOffset()
    {
        Console.WriteLine("============================================================");
        Console.WriteLine("  Weave Speed and Offset Test");
        Console.WriteLine("============================================================");

        if (robot == null)
        {
            Console.WriteLine("ERROR: Robot not connected!");
            return;
        }

        int rtn;
        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);

        JointPos j1 = new JointPos(5.027, -84.331, -75.139, -103.690, 86.379, 20.794);
        DescPose d1 = new DescPose(324.752, -83.339, 366.314, -172.321, -0.936, -106.047);

        JointPos j2 = new JointPos(-35.335, -117.598, -57.174, -95.234, 90.001, -19.560);
        DescPose d2 = new DescPose(324.999, -355.439, 260.000, 179.995, 0.003, -105.775);

        JointPos j3 = new JointPos(59.787, -117.594, -57.183, -95.222, 90.006, 75.562);
        DescPose d3 = new DescPose(324.998, 355.441, 260.002, 179.995, 0.003, -105.775);

        // ---- Step 1: MoveJ to start point ----
        Console.WriteLine("\nStep 1: MoveJ to start point");
        rtn = robot.MoveJ(j1, d1, 1, 0, 100, 100, 50, epos, -1, 0, offset_pos);
        Console.WriteLine("  MoveJ(j1) rtn={0}", rtn);
        Thread.Sleep(500);

        // ---- Step 2: MoveJ to weave entry ----
        Console.WriteLine("\nStep 2: MoveJ to weave entry point");
        rtn = robot.MoveJ(j2, d2, 1, 0, 100, 100, 50, epos, -1, 0, offset_pos);
        Console.WriteLine("  MoveJ(j2) rtn={0}", rtn);
        Thread.Sleep(500);

        // ---- Step 3: WeaveStart, launch weave MoveL thread ----
        Console.WriteLine("\nStep 3: WeaveStart + MoveL in background thread");
        robot.WeaveStart(0);

        bool weaveRunning = true;
        Thread weaveThread = new Thread(() =>
        {
            rtn = robot.MoveL(j3, d3, 1, 0, 100, 100, 5, -1, 0, epos, 0, 0, offset_pos, 5, 0, 0, 10);
            Console.WriteLine("  MoveL(weave) thread finished, rtn={0}", rtn);
            weaveRunning = false;
        });
        weaveThread.IsBackground = true;
        weaveThread.Start();
        Thread.Sleep(500);  // Wait for motion to start

        // ---- Step 4: Speed test (main thread, weave MoveL in background) ----
        Console.WriteLine("\nStep 4: SetSpeed test during weaving");
        int[] speedValues = { 20, 50, 80, 30, 60, 10 };
        foreach (int speed in speedValues)
        {
            if (!weaveRunning) break;
            rtn = robot.SetSpeedInstant(speed);
            robot.GetRobotRealTimeState(ref pkg);
            Console.WriteLine("  SetSpeed({0}) -> rtn={1}, TCP_CmpSpeed={2}", speed, rtn, pkg.target_TCP_CmpSpeed);
            Thread.Sleep(5000);
        }


        Thread.Sleep(5000);
        // ---- Step 5: SetWeaveOffsetRT offset test (main thread, weave MoveL in background) ----
        Console.WriteLine("\nStep 5: SetWeaveOffsetRT test (50 iterations, delta=0.1)");
        double accumOffset = 0.0;
        for (int i = 0; i < 50 && weaveRunning; i++)
        {
            accumOffset += 0.1;
            DescPose weaveOffset = new DescPose(0, 0, accumOffset, 0, 0, 0);
            rtn = robot.SetWeaveOffsetRT(weaveOffset);
            robot.GetRobotRealTimeState(ref pkg);
            Console.WriteLine("  [{0}/50] SetWeaveOffsetRT(x={1:F1}) -> rtn={2}, TCP_pos=({3:F2},{4:F2},{5:F2})",
                i + 1, accumOffset, rtn,
                pkg.tl_cur_pos[0], pkg.tl_cur_pos[1], pkg.tl_cur_pos[2]);
            Thread.Sleep(100);
        }

        // ---- Step 6: Wait for weave MoveL, then WeaveEnd ----
        Console.WriteLine("\nStep 6: Wait for weave MoveL, then WeaveEnd");
        weaveThread.Join();
        robot.WeaveEnd(0);
        Thread.Sleep(500);

        // ---- Step 7: MoveL back to start ----
        Console.WriteLine("\nStep 7: MoveL back to start");
        rtn = robot.MoveL(j1, d1, 1, 0, 100, 100, 50, -1, 0, epos, 0, 0, offset_pos, 50, 0, 0, 10);
        Console.WriteLine("  MoveL(back) rtn={0}", rtn);

        robot.GetRobotRealTimeState(ref pkg);
        Console.WriteLine("\n  Final robot state: main_code={0}, sub_code={1}", pkg.main_code, pkg.sub_code);
        Console.WriteLine("============================================================");
        Console.WriteLine("  Weave Speed and Offset Test Complete");
        Console.WriteLine("============================================================");
    }