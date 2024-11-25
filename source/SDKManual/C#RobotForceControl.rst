Machine Human Control
===============================================

.. toctree:: 
    :maxdepth: 5


Force Sensor Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Configure the force sensor
    * @param [in] company Force Sensor Manufacturer, 17 - Quintiq Technologies
    * @param [in] device device number, not used, default is 0
    * @param [in] softvesion software version number, do not use, the default is 0
    * @param [in] bus device hangs on the end of the bus position, do not use, the default is 0
    * @return Error code
    */
    int FT_SetConfig(int company, int device, int softvesion, int bus); 

Get the force transducer configuration 
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Force sensor activation
    * @param [in] act 0-reset, 1-activate
    * @return Error code.
    */
    int FT_Activate(byte act). 

Force Transducer Zeroing
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Force sensor zeroing
    * @param [in] act 0-remove zero, 1-zero correction
    * @return Error code
    */
    int FT_SetZero(byte act); 

Code example
+++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnFT_Click(object sender, EventArgs e)
    {     
       Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        int company = 17;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int index = 1;
        byte act = 0;

        robot.FT_SetConfig(company, device, softversion, bus);
        Thread.Sleep(1000);
        company = 0;
        robot.FT_GetConfig(ref company, ref device, ref softversion, ref bus);
        Console.WriteLine($"FT config : {company}, {device}, {softversion}, {bus}");
        Thread.Sleep(1000);

        robot.FT_Activate(act);
        Thread.Sleep(1000);
        act = 1;
        robot.FT_Activate(act);
        Thread.Sleep(1000);

        robot.SetLoadWeight(0.0f);
        Thread.Sleep(1000);
        DescTran coord = new DescTran(0, 0, 0);
                
        robot.SetLoadCoord(coord);
        Thread.Sleep(1000);
        robot.FT_SetZero(0);// 0 Remove zero point 1 Correct zero point
        Thread.Sleep(1000);

        ForceTorque ft = new ForceTorque(0, 0, 0, 0, 0, 0);
        int rtn = robot.FT_GetForceTorqueOrigin(1, ref ft);
        Console.WriteLine($"ft origin : {ft.fx}, {ft.fy}, { ft.fz}, { ft.tx}, { ft.ty}, { ft.tz}    rtn   {rtn}");
        rtn = robot.FT_SetZero(1);// zero correction
        //Console.WriteLine($"set zero rtn {rtn}");

        Thread.Sleep(2000);
        rtn = robot.FT_GetForceTorqueOrigin(1, ref ft);
        Console.WriteLine($"ft rcs : {ft.fx}, {ft.fy}, {ft.fz}, {ft.tx}, {ft.ty}, {ft.tz}  rtn  {rtn}");

        robot.FT_GetForceTorqueRCS(1, ref ft);
        Console.WriteLine($"FT_GetForceTorqueRCS rcs : {ft.fx}, {ft.fy}, {ft.fz}, {ft.tx}, {ft.ty}, {ft.tz}");
    }

Set the force sensor reference coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the force sensor reference coordinate system.
    * @param [in] ref 0-tool coordinate system, 1-base coordinate system
    * @return Error code.
    */
    int FT_SetRCS(byte type). 

Load Weight Recognition Record
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Load weight identification record.
    * @param [in] id Sensor coordinate system number in the range [1~14].
    * @return Error code.
    */
    int FT_PdIdenRecord(int id).

Load weight recognition calculation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Load weight recognition calculation.
    * @param [out] weight Weight of the load in kg.
    * @return Error code.
    */   
    int FT_PdIdenCompute(ref double weight).

Load center of mass identification record
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Load center-of-mass identification logging
    * @param [in] id Sensor coordinate system number, range [1~14].
    * @param [in] index point number, range [1~3].
    * @return Error code
    */
    int FT_PdCogIdenRecord(int id, int index). 

Load center of mass identification calculation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Load center of mass identification calculation.
    * @param [out] cog load center of mass in mm.
    * @return Error code.
    */   
    int FT_PdCogIdenCompute(ref DescTran cog);

Code Samples
+++++++++++++++
.. code-block:: c#
    :linenos:

      private void btnFTPdCog_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        double weight = 0.1;
        int rtn = -1;
        DescPose tcoord, desc_p1, desc_p2, desc_p3;
        tcoord = new DescPose(0, 0, 0, 0, 0, 0);
        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
        desc_p3 = new DescPose(0, 0, 0, 0, 0, 0);

        robot.FT_SetRCS(0);
        Thread.Sleep(1000);

        tcoord.tran.z = 35.0;
        robot.SetToolCoord(10, tcoord, 1, 0);
        Thread.Sleep(1000);
        robot.FT_PdIdenRecord(10);
        Thread.Sleep(1000);
        robot.FT_PdIdenCompute(ref weight);
        Console.WriteLine($"payload weight : {weight}");

        desc_p1.tran.x = -47.805;
        desc_p1.tran.y = -362.266;
        desc_p1.tran.z = 317.754;
        desc_p1.rpy.rx = -179.496;
        desc_p1.rpy.ry = -0.255;
        desc_p1.rpy.rz = 34.948;

        desc_p2.tran.x = -77.805;
        desc_p2.tran.y = -312.266;
        desc_p2.tran.z = 317.754;
        desc_p2.rpy.rx = -179.496;
        desc_p2.rpy.ry = -0.255;
        desc_p2.rpy.rz = 34.948;

        desc_p3.tran.x = -167.805;
        desc_p3.tran.y = -312.266;
        desc_p3.tran.z = 387.754;
        desc_p3.rpy.rx = -179.496;
        desc_p3.rpy.ry = -0.255;
        desc_p3.rpy.rz = 34.948;

        rtn = robot.MoveCart(desc_p1, 0, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        Console.WriteLine($"MoveCart rtn  {rtn}");
        Thread.Sleep(1000);
        robot.FT_PdCogIdenRecord(10, 1);
        robot.MoveCart(desc_p2, 0, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        Thread.Sleep(1000);
        robot.FT_PdCogIdenRecord(10, 2);
        robot.MoveCart(desc_p3, 0, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        Thread.Sleep(1000);
        robot.FT_PdCogIdenRecord(10, 3);
        Thread.Sleep(1000);
        DescTran cog = new DescTran(0, 0, 0);

        robot.FT_PdCogIdenCompute(ref cog);
        Console.WriteLine($"cog : {cog.x}, {cog.y}, {cog.z}");
    }

Get force/torque data in reference coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get force/torque data in reference coordinate system.
    * @param [out] ft force/torque, fx,fy,fz,tx,ty,tz
    * @return Error code
    */   
    int FT_GetForceTorqueRCS(byte flag, ref ForceTorque ft); 

Get force sensor raw force/torque data
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the raw force/torque data from the force sensor.
    * @param [out] ft force/torque, fx,fy,fz,tx,ty,tz
    * @return Error code.
    */   
    int FT_GetForceTorqueOrigin(byte flag, ref ForceTorque ft); 

Collision guard
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

Code Samples
+++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnFTGuard_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        byte flag = 1;
        byte sensor_id = 1;
        int[] select = new int[6]{ 1, 0, 0, 0, 0, 0 };//只启用x轴碰撞守护
        double[] max_threshold = new double[6]{ 5.0f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f };
        double[] min_threshold = new double[6]{ 3.0f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f };

        ForceTorque ft = new ForceTorque(0, 0, 0, 0, 0, 0);
        DescPose desc_p1, desc_p2, desc_p3;
        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
        desc_p3 = new DescPose(0, 0, 0, 0, 0, 0);

        desc_p1.tran.x = 1.299;
        desc_p1.tran.y = -719.159;
        desc_p1.tran.z = 141.314;
        desc_p1.rpy.rx = 177.999;
        desc_p1.rpy.ry = -0.715;
        desc_p1.rpy.rz = -161.937;

        desc_p2.tran.x = 245.047;
        desc_p2.tran.y = -675.509;
        desc_p2.tran.z = 139.538;
        desc_p2.rpy.rx = 177.987;
        desc_p2.rpy.ry = -0.129;
        desc_p2.rpy.rz = -142.238;

        desc_p3.tran.x = 157.233;
        desc_p3.tran.y = -550.088;
        desc_p3.tran.z = 112.485;
        desc_p3.rpy.rx = -176.579;
        desc_p3.rpy.ry = -2.819;
        desc_p3.rpy.rz = -148.415;
        robot.SetSpeed(5);

        int rtn =  robot.FT_Guard(flag, sensor_id, select, ft, max_threshold, min_threshold);
        Console.WriteLine($"FT_Guard start rtn {rtn}");
        robot.MoveCart(desc_p1, 1, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        robot.MoveCart(desc_p2, 1, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        robot.MoveCart(desc_p3, 1, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
        flag = 0;
        rtn = robot.FT_Guard(flag, sensor_id, select, ft, max_threshold, min_threshold);
        Console.WriteLine($"FT_Guard end rtn {rtn}");
    }

Constant force control
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Constant force control
    * @param [in] flag 0- turn off constant force control, 1- turn on constant force control
    * @param [in] sensor_id force sensor number
    * @param [in] select Selects whether or not to detect collisions in the six degrees of freedom, 0-does not detect, 1-detects.
    * @param [in] ft collision force/torque, fx,fy,fz,tx,ty,tz
    * @param [in] ft_pid force pid parameter, torque pid parameter
    * @param [in] adj_sign Adaptive start/stop control, 0-off, 1-on
    * @param [in] ILC_sign ILC start-stop control, 0-stop, 1-training, 2-practice
    * @param [in] Maximum adjustment distance in mm
    * @param [in] Max. adjustment angle in deg.
    * @return Error code
    */   
    int FT_Control(int flag, int sensor_id, int[] select, ForceTorque ft, double[] ft_pid, int adj_sign, int ILC_sign, double max_dis, double max_ang);   

Code example
+++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnFTConttol_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        byte flag = 1;
        byte sensor_id = 1;
        int[] select = new int[6]{ 0, 0, 1, 0, 0, 0 };
        double[] ft_pid = new double[6]{ 0.0005f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        byte adj_sign = 0;
        byte ILC_sign = 0;
        float max_dis = 100.0f;
        float max_ang = 0.0f;

        ForceTorque ft = new ForceTorque(0, 0, 0, 0 ,0 ,0);
        DescPose desc_p1, desc_p2, offset_pos;
        JointPos j1, j2;
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
        offset_pos = new DescPose(0, 0, 0, 0, 0, 0);

        j2 = new JointPos(0, 0, 0, 0, 0, 0);
        j1 = new JointPos(0, 0, 0, 0, 0, 0);

        desc_p1.tran.x = 1.299;
        desc_p1.tran.y = -719.159;
        desc_p1.tran.z = 141.314;
        desc_p1.rpy.rx = 177.999;
        desc_p1.rpy.ry = -0.715;
        desc_p1.rpy.rz = -161.937;

        desc_p2.tran.x = 245.047;
        desc_p2.tran.y = -675.509;
        desc_p2.tran.z = 139.538;
        desc_p2.rpy.rx = 177.987;
        desc_p2.rpy.ry = -0.129;
        desc_p2.rpy.rz = -142.238;
        ft.fz = -10.0;

        robot.GetInverseKin(0, desc_p1, -1, ref j1);
        robot.GetInverseKin(0, desc_p2, -1, ref j2);

        robot.MoveJ(j1, desc_p1, 1, 0, 100.0f, 180.0f, 100.0f, epos, -1.0f, 0, offset_pos);
        int rtn = robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);
        Console.WriteLine($"FT_Control start rtn {rtn}");

        robot.MoveL(j2, desc_p2, 1, 0, 100.0f, 180.0f, 20.0f, -1.0f, epos, 0, 0, offset_pos);
        flag = 0;
        rtn = robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);
        Console.WriteLine($"FT_Control end rtn {rtn}");
    }

Smooth control on
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Softening control on
    * @param [in] p Position adjustment coefficient or softening coefficient.
    * @param [in] force Soft opening force threshold in N
    * @return Error code.
    */   
    int FT_ComplianceStart(float p, float force).

Flex control off
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Smooth control off
    * @return Error code
    */   
    int FT_ComplianceStop(); 

Code example
+++++++++++++++
.. code-block:: c#
    :linenos:


    private void btnComplience_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        byte flag = 1;
        int sensor_id = 1;
        int[] select = new int[6]{ 1, 1, 1, 0, 0, 0 };
        double[] ft_pid = new double[6] { 0.0005f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        byte adj_sign = 0;
        byte ILC_sign = 0;
        float max_dis = 100.0f;
        float max_ang = 0.0f;

        ForceTorque ft = new ForceTorque(0, 0, 0, 0, 0, 0);
        DescPose desc_p1, desc_p2, offset_pos;
        JointPos j1, j2;

        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
        offset_pos = new DescPose(0, 0, 0, 0, 0, 0);

        j2 = new JointPos(0, 0, 0, 0, 0, 0);
        j1 = new JointPos(0, 0, 0, 0, 0, 0);

        desc_p1.tran.x = 1.299;
        desc_p1.tran.y = -719.159;
        desc_p1.tran.z = 141.314;
        desc_p1.rpy.rx = 177.999;
        desc_p1.rpy.ry = -0.715;
        desc_p1.rpy.rz = -161.937;

        desc_p2.tran.x = 245.047;
        desc_p2.tran.y = -675.509;
        desc_p2.tran.z = 139.538;
        desc_p2.rpy.rx = 177.987;
        desc_p2.rpy.ry = -0.129;
        desc_p2.rpy.rz = -142.238;
        ft.fz = -10.0;

        robot.GetInverseKin(0, desc_p1, -1, ref j1);
        robot.GetInverseKin(0, desc_p2, -1, ref j2);

        ft.fx = -10.0;
        ft.fy = -10.0;
        ft.fz = -10.0;
        robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);
        float p = 0.00005f;
        float force = 30.0f;
        int rtn = robot.FT_ComplianceStart(p, force);
        Console.WriteLine($"FT_ComplianceStart rtn {rtn}");
        int count = 15;
        while (count > 0)
        {
            robot.MoveL(j1, desc_p1, 1, 0, 100.0f, 180.0f, 100.0f, -1.0f, epos, 0, 1, offset_pos);
            robot.MoveL(j2, desc_p2, 1, 0, 100.0f, 180.0f, 100.0f, -1.0f, epos, 0, 0, offset_pos);
            count -= 1;
        }
        rtn = robot.FT_ComplianceStop();
        Console.WriteLine($"FT_ComplianceStop rtn {rtn}");
        flag = 0;
        robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);
    }

Load recognition initialization
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Load Recognition Initialization
    * @return Error code
    */
    int LoadIdentifyDynFilterInit();

Initialize load identifying variables
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Initialization of load recognition variables.
    * @return Error code
    */
    int LoadIdentifyDynVarInit();

Load identification main program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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