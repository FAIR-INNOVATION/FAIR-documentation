human control of machinery
=====================================

.. toctree:: 
    :maxdepth: 5

Force Sensor Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Configuring force sensors
    * @param [in] config company:force sensor manufacturers, 17-Kunwei Technology, 19-Aerospace 11th Academy, 20-ATI Sensors, 21-Zhongke MiDot, 22-Weihang Minxin
    * @param [in] config device: device number, Kunwei (0-KWR75B), Aisino Eleven (0-MCS6A-200-4), ATI (0-AXIA80-M8), Zhongke MiDot (0-MST2010), Weihang Minxin (0-WHC6L-YB-10A)
    * @param [in] config softvesion: software version number, not used yet, default is 0.
    * @param [in] config bus: the device hangs on the end bus position, not used for the time being, the default is 0
    * @return error code
    */
    int FT_SetConfig(DeviceConfig config). 

Get Force Sensor Configuration 
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get force sensor configuration 
    * @param [out] config company:force sensor manufacturers, 17-Kunwei Technology, 19-Aerospace 11th Academy, 20-ATI Sensors, 21-Zhongke MiDot, 22-Weihang Minxin
    * @param [out] config device: device number, Kunwei (0-KWR75B), Aisino Eleven (0-MCS6A-200-4), ATI (0-AXIA80-M8), Zhongke MiDot (0-MST2010), Weihang Minxin (0-WHC6L-YB-10A)
    * @param [out] config softvesion: software version number, not used yet, default is 0.
    * @param [out] config bus: the device hangs on the end bus position, not used for the time being, the default is 0
    * @return error code 
    */ 
    int FT_GetConfig(DeviceConfig config). 

Force sensor activation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Force sensor activation
    * @param [in] act 0-reset, 1-activate
    * @return error code
    */
    int FT_Activate(int act). 

Force Sensor Zeroing
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Force Transducer Zeroing
    * @param [in] act 0-zero removal, 1-zero correction
    * @return error code
    */
    int FT_SetZero(int act). 

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
        DeviceConfig config = new DeviceConfig();
        config.company = 24;
        config.device = 0;
        config.softwareVersion = 0;
        config.bus = 0;

        robot.FT_SetConfig(config);
        robot.Sleep(1000);
        config.company = 0;
        robot.FT_GetConfig(config);
        System.out.println("FT config : " + config.company + ", " + config.device + ", " + config.softwareVersion + ", " + config.bus);

        robot.FT_Activate(0); //reset
        robot.Sleep(2000);

        robot.FT_Activate(1); //activate
        robot.Sleep(2000);

        robot.FT_SetZero(0);//0 to remove the zero point
        robot.Sleep(2000);

        robot.FT_SetZero(1);//1 zero correction
    }

Setting the force transducer reference coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the force transducer reference coordinate system
    * @param [in] type 0-tool coordinate system, 1-base coordinate system, 2-free coordinate system
    * @param [in] coord free coordinate system value
    * @return error code
    */
    int FT_SetRCS(int type, DescPose coord). 

Load weight identification records
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Load weight identification records
    * @param [in] id Sensor coordinate system number in the range [1 to 14].
    * @return error code
    */
    int FT_PdIdenRecord(int id).

Load weight identification calculation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Load weight identification calculations
    * @return List[0]: error code; List[1] : double weight load weight in kg
    */  
    List<Number> FT_PdIdenCompute().

Load center of mass identification records
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Load center of mass identification records
    * @param [in] id Sensor coordinate system number in the range [1 to 14].
    * @param [in] index point number in the range [1~3].
    * @return error code
    */
    int FT_PdCogIdenRecord(int id, int index); 

Load center of mass identification calculation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Load center of mass identification calculations
    * @param [out] cog load center of mass in mm
    * @return error code
    */  
    int FT_PdCogIdenCompute(DescTran cog).

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
        DeviceConfig config = new DeviceConfig();
        config.company = 24;
        config.device = 0;
        config.softwareVersion = 0;
        config.bus = 0;

        robot.FT_SetConfig(config);
        robot.Sleep(1000);
        DescPose tcoord, desc_p1, desc_p2, desc_p3;
        tcoord = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);
        desc_p1 = new DescPose(-14.404,-455.283,319.847,-172.935,25.141,-68.097);
        desc_p2 = new DescPose(-107.999,-599.174,285.939,153.472,12.686,-71.284);
        desc_p3 = new DescPose(6.586,-704.897,309.638,178.909,-27.759,-70.479);

        DescPose coord = new DescPose(0, 0 ,0, 1, 0, 0);
        robot.FT_SetRCS(0, coord);
        robot.Sleep(1000);

        tcoord.tran.z = 35.0;
        robot.SetToolCoord(8, tcoord, 1, 0);
        robot.Sleep(1000);
        robot.FT_PdIdenRecord(10).
        robot.Sleep(1000);
        List<Number> rtnArray = robot.FT_PdIdenCompute();
        System.out.println("payload weight : " + rtnArray.get(1));

        robot.MoveCart(desc_p1, 0, 0, 20.0f, 100.0f, 100.0f, -1.0f, -1);
        robot.Sleep(1000);
        robot.FT_PdCogIdenRecord(2, 1);
        robot.MoveCart(desc_p2, 0, 0, 20.0f, 100.0f, 100.0f, -1.0f, -1);
        robot.Sleep(1000);
        robot.FT_PdCogIdenRecord(2, 2);
        robot.MoveCart(desc_p3, 0, 0, 20.0f, 100.0f, 100.0f, -1.0f, -1);
        robot.Sleep(1000);
        robot.FT_PdCogIdenRecord(2, 3);
        robot.Sleep(1000);

        DescTran rtnCog = new DescTran();
        robot.FT_PdCogIdenCompute(rtnCog);
        System.out.println("cog : " + rtnCog.x + ", " + rtnCog.y + ", " + rtnCog.z);
    }


Obtaining force/torque data in the reference coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Obtain force/torque data in reference coordinate system.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] ft force/torque, fx,fy,fz,tx,ty,tz
    * @return error code
    */  
    int FT_GetForceTorqueRCS(int flag, ForceTorque ft); 

Obtaining Force Sensor Raw Force/Torque Data
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Obtain raw force/torque data from force transducers
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] ft force/torque, fx,fy,fz,tx,ty,tz
    * @return error code
    */  
    int FT_GetForceTorqueOrigin(int flag, ForceTorque ft); 

Collision Guard
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Collision Guard
    * @param [in] flag 0- turn off collision guarding, 1- turn on collision guarding
    * @param [in] sensor_id force sensor number
    * @param [in] select Select whether to detect collisions in the six degrees of freedom, 0-no detection, 1-detection
    * @param [in] ft collision force/torque, fx,fy,fz,tx,ty,tz
    * @param [in] max_threshold max_threshold
    * @param [in] min_threshold minimum_threshold
    * @note Force/torque detection range: (ft-min_threshold, ft+max_threshold)
    * @return error code
    */  
    int FT_Guard(int flag, int sensor_id, Object[] select, ForceTorque ft, Object[] max_threshold, Object[] min_threshold); 

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
        byte flag = 1;
        byte sensor_id = 8;
        Object[] select = { 1, 0, 0, 0, 0, 0, 0 };//enable only x-axis collision guarding
        Object[] max_threshold = { 5.0, 0.01, 0.01, 0.01, 0.01, 0.01 };
        Object[] min_threshold = { 3.0, 0.01, 0.01, 0.01, 0.01, 0.01 };

        ForceTorque ft = new ForceTorque(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        DescPose desc_p1, desc_p2, desc_p3.
        desc_p1 = new DescPose(-14.404,-455.283,319.847,-172.935,25.141,-68.097);
        desc_p2 = new DescPose(-107.999,-599.174,285.939,153.472,12.686,-71.284);
        desc_p3 = new DescPose(6.586,-704.897,309.638,178.909,-27.759,-70.479);

        int rtn = robot.FT_Guard(flag, sensor_id, select, ft, max_threshold, min_threshold);
        System.out.println("FT_Guard start rtn {rtn}");
        robot.MoveCart(desc_p1, 0, 0, 20, 100.0f, 100.0f, -1.0f, -1);
        robot.MoveCart(desc_p2, 0, 0, 20, 100.0f, 100.0f, -1.0f, -1);
        robot.MoveCart(desc_p3, 0, 0, 20, 100.0f, 100.0f, -1.0f, -1);
    }

constant force control
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Constant Force Control
    * @param [in] flag 0-constant force control off, 1-constant force control on
    * @param [in] sensor_id force sensor number
    * @param [in] select Select whether to detect collisions in the six degrees of freedom, 0-no detection, 1-detection
    * @param [in] ft collision force/torque, fx,fy,fz,tx,ty,tz
    * @param [in] ft_pid force pid parameter, torque pid parameter
    * @param [in] adj_sign Adaptive start-stop control, 0-off, 1-on
    * @param [in] ILC_sign ILC start/stop control, 0-stop, 1-training, 2-practice
    * @param [in] max_dis Maximum adjustment distance in mm
    * @param [in] max_ang Maximum adjustment angle in deg
    * @param [in] filter_Sign filter_on flag 0-off; 1-on, default off
    * @param [in] posAdapt_sign posAdapt_sign Attitude compliance on flag 0-Off; 1-On, default off
    * @param [in] isNoBlock blocking flag, 0-blocking; 1-non-blocking
    * @return error code
    */  
    int FT_Control(int flag, int sensor_id, Object[] select, ForceTorque ft, Object[] ft_pid, int adj_sign, int ILC_sign, double max_dis, double max_ang , int filter_Sign, int posAdapt_sign, int isNoBlock).   

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
        byte flag = 1;
        byte sensor_id = 8;
        Object[] select = { 0,0,1,0,0,0,0 };
        Object[] ft_pid = { 0.0005, 0.0, 0.0, 0.0, 0.0, 0.0 };
        byte adj_sign = 0;
        byte ILC_sign = 0;
        float max_dis = 100.0f;
        float max_ang = 0.0f;
        ForceTorque ft = new ForceTorque(0, 0, -10, 0 ,0 ,0 ,0);

        JointPos j1=new JointPos(-21.724,-136.814,-59.518,-68.853,89.245,-66.35);
        DescPose desc_p1 = new DescPose(703.996,-391.695,240.708,-178.756,-4.709,-45.447);

        JointPos j2=new JointPos(0.079,-130.285,-71.029,-72.115,88.945,-62.736);
        DescPose desc_p2 = new DescPose(738.755,-102.812,226.704,177.488,2.566,-27.209);

        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0, 0);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);

        //Joint space movement
        robot.MoveL(j1, desc_p1, 0, 0, 40.0f, 180.0f, 20.0f, -1.0f, epos, 0, 0, offset_pos, 0, 100);
        int rtn = robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
        System.out.println("FT_Control start rtn " + rtn);

        robot.MoveL(j2, desc_p2, 0, 0, 10.0f, 180.0f, 20.0f, -1.0f, epos, 0, 0, offset_pos, 0, 100);
        flag = 0;
        rtn = robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0 ,0);
        System.out.println("FT_Control end rtn " + rtn);
    }

Soft control on
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Soft control on
    * @param [in] p Position adjustment factor or softness factor
    * @param [in] force Soft opening force threshold in N
    * @return error code
    */  
    int FT_ComplianceStart(double p, double force).

Soft Control Off
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Soft control off
    * @return error code
    */  
    int FT_ComplianceStop(); 

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
            System.out.println("rpc connection failed");
            return ;
        }
        byte flag = 1;
        int sensor_id = 8;
        Object[] select = { 1, 1, 1, 0, 0, 0 };
        Object[] ft_pid = { 0.0005, 0.0, 0.0, 0.0, 0.0, 0.0 };
        int adj_sign = 0;
        int ILC_sign = 0;
        double max_dis = 100.0;
        double max_ang = 0.0;

        ForceTorque ft = new ForceTorque(-10.0, -10.0, -10.0, 0.0, 0.0, 0.0, 0.0);
        DescPose desc_p1, desc_p2, offset_pos.
        JointPos j1.
        j1=new JointPos(-21.724, -136.814, -59.518, -68.853, 89.245, -66.359);

        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0, 0);
        desc_p1 = new DescPose(703.996, -391.695, 240.708, -178.756, -4.709, -45.447);
        offset_pos = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);

        ft.fx = -10.0;
        ft.fy = -10.0;
        ft.fz = -10.0;
        robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang,0,0,0);
        float p = 0.00005f;
        float force = 10.0f;
        int rtn = robot.FT_ComplianceStart(p, force);
        System.out.println("FT_ComplianceStart rtn " + rtn);

        robot.MoveL(j1, desc_p1, 0, 0, 20.0, 180.0, 100.0, -1.0, epos, 0, 1, offset_pos, 0, 100);

        rtn = robot.FT_ComplianceStop();
        System.out.println("FT_ComplianceStop rtn " + rtn);
        flag = 0;
        robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang,0,0,0);
    }

Load recognition initialization
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Load recognition initialization
    * @return error code
    */
    int LoadIdentifyDynFilterInit();

Initialization of load recognition variables
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Load recognition variable initialization
    * @return error code
    */
    int LoadIdentifyDynVarInit();

Load Recognition Main Program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Load Recognition Main Program
    * @param [in] joint_torque Joint torque
    * @param [in] joint_pos Joint position
    * @param [in] t Sampling period
    * @return error code
    */
    int LoadIdentifyMain(Object[] joint_torque, Object[] joint_pos, double t);

Getting Load Recognition Results
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get load recognition results
    * @param [in] gain
    * @return List[0]: error code; List[1] : double weight load weight; List[2]: x load center of mass X mm; List[3] : y load center of mass Y mm; List[2]: z load center of mass Z mm
    */
    List<Number> LoadIdentifyGetResult(Object[] gain);

Get force sensor drag switch status
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get force sensor drag switch status
    * @return List[0]:errorCode; List[1] : dragState force sensor assisted drag control state, 0-off; 1-on; List[1] : sixDimensionalDragState sixDimensionalForceAssistedDragControlState, 0-off; 1-on
    */
    List<Integer> GetForceAndTorqueDragState();

Force Sensor Assisted Drag
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Force sensor assisted drag
    * @param [in] status Control status, 0-off; 1-on
    * @param [in] asaptiveFlag Adaptive on flag, 0-off; 1-on
    * @param [in] interfereDragFlag interference area drag flag, 0-off; 1-on
    * @param [in] M coefficient of inertia
    * @param [in] B Damping factor
    * @param [in] K Stiffness factor
    * @param [in] F Drag six-dimensional force thresholds
    * @param [in] Fmax Maximum towing force limit Nm
    * @param [in] Vmax Maximum joint speed limit °/s
    * @return error code
    */
    int EndForceDragControl(int status, int asaptiveFlag, int interfereDragFlag, Object[] M, Object[] B, Object[] K, Object[] F, double Fmax, double Vmax);

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
        List<Integer> rtnArray = robot.GetForceAndTorqueDragState();
        System.out.println("The drag state is " + rtnArray.get(1) + " ForceAndJointImpedance state " + rtnArray.get(2));

        robot.Sleep(1000);
        Object[] M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
        Object[] B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
        Object[] K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        Object[] F = { 10.0, 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
        robot.EndForceDragControl(1, 0, 0, M, B, K, F, 50, 100);

        rtnArray = robot.GetForceAndTorqueDragState();
        System.out.println("The drag state is " + rtnArray.get(1) + " ForceAndJointImpedance state " + rtnArray.get(2));

        robot.Sleep(1000 * 10);
        robot.EndForceDragControl(0, 0, 0, M, B, K, F, 50, 100);

        rtnArray = robot.GetForceAndTorqueDragState();
        System.out.println("The drag state is " + rtnArray.get(1) + " ForceAndJointImpedance state " + rtnArray.get(2));
    }

Setting up hybrid drag switches and parameters for six-dimensional force and joint impedance
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting up six-dimensional force and joint impedance hybrid drag switches and parameters
    * @param [in] status Control status, 0-off; 1-on
    * @param [in] impedanceFlag impedance on flag, 0- off; 1- on
    * @param [in] lamdeGain drag gain
    * @param [in] KGain Stiffness gain
    * @param [in] BGain Damping Gain
    * @param [in] dragMaxTcpVel Maximum line speed limit at the end of the drag
    * @param [in] dragMaxTcpOriVel Maximum angular velocity limit at end of drag
    * @return error code
    */
    int ForceAndJointImpedanceStartStop(int status, int impedanceFlag, Object[] lamdeGain, Object[] KGain, Object[] BGain, double dragMaxTcpVel, double dragMaxTcpOriVel);

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
        robot.DragTeachSwitch(1);
        Object[] lamdeDain = { 3.0, 2.0, 2.0, 2.0, 2.0, 2.0, 3.0 };
        Object[] KGain = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        Object[] BGain = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
        robot.ForceAndJointImpedanceStartStop(1, 0, lamdeDain, KGain, BGain, 1000.0, 180.0);

        List<Integer> rtnArray = robot.GetForceAndTorqueDragState();
        System.out.println("The drag state is " + rtnArray.get(1) + " ForceAndJointImpedance state " + rtnArray.get(2));
    }

Setting the load weight under the force transducer
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the load weight under the force transducer
    * @param [in] weight load weight kg
    * @return error code
    */
    int SetForceSensorPayLoad(double weight);

Setting the load center of mass under the force transducer
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the center of mass of the load under the force transducer.
    * @param [in] cog load center of mass mm
    * @return error code
    */
    int SetForceSensorPayLoadCog(DescTran cog).

Getting the load weight under the force transducer
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get load weight under force transducer
    * @return List[0]: error code; List[1] : weight load weight kg
    */
    List<Number> GetForceSensorPayLoad();

Obtaining the center of mass of the load under the force transducer
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Getting the center of mass of the load under the force transducer.
    * @param [out] cog load center of mass mm
    * @return error code
    */
    int GetForceSensorPayLoadCog(DescTran cog).
    
code example
+++++++++++++++
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
        robot.SetForceSensorPayLoad(1.34);
        DescTran cog = new DescTran(0.778, 2.554, 48.765);
        robot.SetForceSensorPayLoadCog(cog);
        double weight = 0;

        List<Number> rtnArrays = robot.GetForceSensorPayLoad();
        DescTran getCog = new DescTran(0.0, 0.0, 0.0);
        robot.GetForceSensorPayLoadCog(getCog);
        System.out.println("the FT load is " + rtnArrays.get(1) + " cog is " + getCog.x + " " + getCog.y + " " + getCog.z);
    }

Automatic zeroing of force sensors
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Automatic zeroing of force sensors
    * @param [in] massCenter Sensor mass (kg) and center of mass (mm)
    * @return error code
    */
    int ForceSensorAutoComputeLoad(MassCenter massCenter).