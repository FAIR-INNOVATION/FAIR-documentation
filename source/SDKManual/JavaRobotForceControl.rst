Robot Force Control
========================

.. toctree:: 
    :maxdepth: 5

Configure Force Sensor
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Configure force sensor
    * @param  config company: Force sensor manufacturer, 17-Kunwei Technology, 19-Aerospace 11th Academy, 20-ATI Sensor, 21-Zhongke Midian, 22-Weihang Minxin, 23-NBIT, 24-Xinjingcheng (XJC), 26-NSR
    * @param  config device: Device number, Kunwei (0-KWR75B), Aerospace 11th Academy (0-MCS6A-200-4), ATI (0-AXIA80-M8), Zhongke Midian (0-MST2010), Weihang Minxin (0-WHC6L-YB-10A), NBIT (0-XLH93003ACS), Xinjingcheng XJC (0-XJC-6F-D82), NSR (0-NSR-FTSensorA)
    * @param  config softvesion: Software version number, not currently used, default is 0
    * @param  config bus: Device bus location, not currently used, default is 0
    * @return  Error code
    */
    int FT_SetConfig(DeviceConfig config); 

Get Force Sensor Configuration 
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get force sensor configuration 
    * @param [out] config company: Force sensor manufacturer, 17-Kunwei Technology, 19-Aerospace 11th Academy, 20-ATI Sensor, 21-Zhongke Midian, 22-Weihang Minxin
    * @param [out] config device: Device number, Kunwei (0-KWR75B), Aerospace 11th Academy (0-MCS6A-200-4), ATI (0-AXIA80-M8), Zhongke Midian (0-MST2010), Weihang Minxin (0-WHC6L-YB-10A)
    * @param [out] config softvesion: Software version number, not currently used, default is 0
    * @param [out] config bus: Device bus location, not currently used, default is 0
    * @return Error code 
    */ 
    int FT_GetConfig(DeviceConfig config); 

Activate Force Sensor
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Activate force sensor
    * @param  [in] act  0-Reset, 1-Activate
    * @return  Error code
    */
    int FT_Activate(int act); 

Zero Calibration for Force Sensor
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Zero calibration for force sensor
    * @param  [in] act  0-Remove zero point, 1-Zero calibration
    * @return  Error code
    */
    int FT_SetZero(int act); 

Set Force Sensor Reference Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Set force sensor reference coordinate system
    * @param  [in] type  0-Tool coordinate system, 1-Base coordinate system, 2-Free coordinate system
    * @param  [in] coord  Free coordinate system value
    * @return  Error code
    */
    int FT_SetRCS(int type, DescPose coord);

Set Load Weight Under Force Sensor
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set load weight under force sensor
    * @param [in] weight Load weight in kg
    * @return Error code
    */
    int SetForceSensorPayLoad(double weight);

Set Load Center of Gravity Under Force Sensor
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set load center of gravity under force sensor
    * @param [in] cog Load center of gravity in mm
    * @return Error code
    */
    int SetForceSensorPayLoadCog(DescTran cog);

Get Load Weight Under Force Sensor
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get load weight under force sensor
    * @return List[0]: Error code; List[1]: weight Load weight in kg
    */
    List<Number> GetForceSensorPayLoad();

Get Load Center of Gravity Under Force Sensor
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get load center of gravity under force sensor
    * @param [out] cog Load center of gravity in mm
    * @return Error code
    */
    int GetForceSensorPayLoadCog(DescTran cog);

Automatic Zero Calibration for Force Sensor
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Automatic zero calibration for force sensor
    * @param [in] massCenter Sensor mass (kg) and center of gravity (mm)
    * @return Error code
    */
    int ForceSensorAutoComputeLoad(MassCenter massCenter);

Get Force/Torque Data in Reference Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get force/torque data in reference coordinate system
    * @param  [in] flag 0-Blocking, 1-Non-blocking
    * @param  [out] ft  Force/torque, fx,fy,fz,tx,ty,tz
    * @return  Error code
    */   
    int FT_GetForceTorqueRCS(int flag, ForceTorque ft); 

Get Raw Force/Torque Data from Force Sensor
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get raw force/torque data from force sensor
    * @param  [in] flag 0-Blocking, 1-Non-blocking
    * @param  [out] ft  Force/torque, fx,fy,fz,tx,ty,tz
    * @return  Error code
    */   
    int FT_GetForceTorqueOrigin(int flag, ForceTorque ft); 

Force Sensor Configuration and Automatic Zero Calibration Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestFTInit(Robot robot)
    {
        DescTran tr1=new DescTran(0,0,0);
        robot.SetForceSensorPayload(0);
        robot.SetForceSensorPayloadCog(tr1);

        int company = 24;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int index = 1;

        DeviceConfig con=new DeviceConfig(company,device,softversion,bus);
        robot.FT_SetConfig(con);
        robot.Sleep(1000);
        robot.FT_GetConfig(con);
        robot.Sleep(1000);

        robot.FT_Activate(0);
        robot.Sleep(1000);
        robot.FT_Activate(1);
        robot.Sleep(1000);

        robot.Sleep(1000);
        robot.FT_SetZero(0);
        robot.Sleep(1000);

        ForceTorque ft=new ForceTorque(0,0,0,0,0,0);
        robot.FT_GetForceTorqueOrigin(0, ft);
        robot.FT_SetZero(1);
        robot.Sleep(1000);

        DescPose ftCoord = new DescPose();
        robot.FT_SetRCS(0, ftCoord);

        robot.SetForceSensorPayload(0.824);

        DescTran tr=new DescTran(0.778, 2.554, 48.765);
        robot.SetForceSensorPayloadCog(tr);
        List<Number> weight = new ArrayList<>();
        double x = 0, y = 0, z = 0;
        weight=robot.GetForceSensorPayload();
        robot.GetForceSensorPayloadCog(tr);
        tr.x=0;
        tr.y=0;
        tr.z=0;
        robot.SetForceSensorPayload(0);
        robot.SetForceSensorPayloadCog(tr);

        double computeWeight = 0;
        DescTran tran = new DescTran();
        MassCenter mass=new MassCenter();
        mass.weight=weight.get(1).doubleValue();
        mass.cog=tran;
        robot.ForceSensorAutoComputeLoad(mass);
        return 0;
    }

Load Weight Identification Record
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Load weight identification record
    * @param  [in] id  Sensor coordinate system number, range [1~14]
    * @return  Error code
    */
    int FT_PdIdenRecord(int id);

Load Weight Identification Calculation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Load weight identification calculation
    * @return  List[0]: Error code; List[1]: double weight Load weight in kg
    */   
    List<Number> FT_PdIdenCompute();

Load Center of Gravity Identification Record
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Load center of gravity identification record
    * @param  [in] id  Sensor coordinate system number, range [1~14]
    * @param  [in] index Point number, range [1~3]
    * @return  Error code
    */
    int FT_PdCogIdenRecord(int id, int index); 

Load Center of Gravity Identification Calculation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Load center of gravity identification calculation
    * @param  [out] cog  Load center of gravity in mm
    * @return  Error code
    */   
    int FT_PdCogIdenCompute(DescTran cog);

Force Sensor Load Identification Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestFTLoadCompute(Robot robot)
    {
        DescTran tr1=new DescTran(0,0,0);
        robot.SetForceSensorPayload(0);
        robot.SetForceSensorPayloadCog(tr1);

        int company = 24;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int index = 1;

        DeviceConfig con=new DeviceConfig(company, device, softversion, bus);
        robot.FT_SetConfig(con);
        robot.Sleep(1000);
        robot.FT_GetConfig(con);
        robot.Sleep(1000);

        robot.FT_Activate(0);
        robot.Sleep(1000);
        robot.FT_Activate(1);
        robot.Sleep(1000);

        robot.Sleep(1000);
        robot.FT_SetZero(0);
        robot.Sleep(1000);

        ForceTorque ft=new ForceTorque(0,0,0,0,0,0);
        robot.FT_GetForceTorqueOrigin(0, ft);
        robot.FT_SetZero(1);
        robot.Sleep(1000);

        DescPose tcoord = new DescPose();
        tcoord.tran.z = 35.0;
        robot.SetToolCoord(10, tcoord, 1, 0, 0, 0);

        robot.FT_PdIdenRecord(10);
        robot.Sleep(1000);

        List<Number> weight =new ArrayList<>();
        weight=robot.FT_PdIdenCompute();

        DescPose desc_p1=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_p2=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose desc_p3=new DescPose(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);

        robot.MoveCart(desc_p1, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
        robot.Sleep(1000);
        robot.FT_PdCogIdenRecord(10, 1);
        robot.MoveCart(desc_p2, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
        robot.Sleep(1000);
        robot.FT_PdCogIdenRecord(10, 2);
        robot.MoveCart(desc_p3, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
        robot.Sleep(1000);
        robot.FT_PdCogIdenRecord(10, 3);
        robot.Sleep(1000);
        DescTran cog=new DescTran(0,0,0);
        robot.FT_PdCogIdenCompute(cog);

        robot.CloseRPC();
        return 0;
    }

Collision Guard
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Collision guard
    * @param  [in] flag 0-Disable collision guard, 1-Enable collision guard
    * @param  [in] sensor_id Force sensor number
    * @param  [in] select  Select which degrees of freedom to detect collision, 0-Disable, 1-Enable
    * @param  [in] ft  Collision force/torque, fx,fy,fz,tx,ty,tz
    * @param  [in] max_threshold Maximum threshold
    * @param  [in] min_threshold Minimum threshold
    * @note   Force/torque detection range: (ft-min_threshold, ft+max_threshold)
    * @return  Error code
    */   
    int FT_Guard(int flag, int sensor_id, Object[] select, ForceTorque ft, Object[] max_threshold, Object[] min_threshold); 

Collision Guard Example
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set reconnection times and interval
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
        Object[] select = { 1, 0, 0, 0, 0, 0 };//Only enable collision guard on x-axis
        Object[] max_threshold = { 5.0, 0.01, 0.01, 0.01, 0.01, 0.01 };
        Object[] min_threshold = { 3.0, 0.01, 0.01, 0.01, 0.01, 0.01 };

        ForceTorque ft = new ForceTorque(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        DescPose  desc_p1, desc_p2, desc_p3;
        desc_p1 = new DescPose(-14.404,-455.283,319.847,-172.935,25.141,-68.097);
        desc_p2 = new DescPose(-107.999,-599.174,285.939,153.472,12.686,-71.284);
        desc_p3 = new DescPose(6.586,-704.897,309.638,178.909,-27.759,-70.479);

        int rtn =  robot.FT_Guard(flag, sensor_id, select, ft, max_threshold, min_threshold);
        System.out.println("FT_Guard start rtn {rtn}");
        robot.MoveCart(desc_p1, 0, 0, 20, 100.0f, 100.0f, -1.0f, -1);
        robot.MoveCart(desc_p2, 0, 0, 20, 100.0f, 100.0f, -1.0f, -1);
        robot.MoveCart(desc_p3, 0, 0, 20, 100.0f, 100.0f, -1.0f, -1);
    }

Constant Force Control
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Constant force control
    * @param  [in] flag 0-Disable constant force control, 1-Enable constant force control
    * @param  [in] sensor_id Force sensor number
    * @param  [in] select  Select which degrees of freedom to detect, 0-Disable, 1-Enable
    * @param  [in] ft  Force/torque, fx,fy,fz,tx,ty,tz
    * @param  [in] ft_pid Force PID parameters, torque PID parameters
    * @param  [in] adj_sign Adaptive start/stop control, 0-Disable, 1-Enable
    * @param  [in] ILC_sign ILC start/stop control, 0-Stop, 1-Training, 2-Operation
    * @param  [in] max_dis Maximum adjustment distance in mm
    * @param  [in] max_ang Maximum adjustment angle in deg
    * @param  [in] filter_Sign Filter enable flag, 0-Disable, 1-Enable, default is disabled
    * @param  [in] posAdapt_sign Posture compliance enable flag, 0-Disable, 1-Enable, default is disabled
    * @param  [in] isNoBlock Blocking flag, 0-Blocking, 1-Non-blocking
    * @return  Error code
    */   
    int FT_Control(int flag, int sensor_id, Object[] select, ForceTorque ft, Object[] ft_pid, int adj_sign, int ILC_sign, double max_dis, double max_ang, int filter_Sign, int posAdapt_sign, int isNoBlock);   

Constant Force Control
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Constant Force Control
    * @param  flag 0-Disable constant force control, 1-Enable constant force control
    * @param  sensor_id Force sensor ID
    * @param  select  Collision detection for six degrees of freedom, 0-Disable, 1-Enable
    * @param  ft  Collision force/torque, fx, fy, fz, tx, ty, tz
    * @param  ft_pid Force PID parameters, Torque PID parameters
    * @param  adj_sign Adaptive start/stop control, 0-Disable, 1-Enable
    * @param  ILC_sign ILC start/stop control: 0-Stop, 1-Training, 2-Operational
    * @param  max_dis Maximum adjustment distance, unit mm
    * @param  max_ang Maximum adjustment angle, unit deg
    * @param  M Mass parameter
    * @param  B Damping parameter
    * @param  polishRadio Polishing radius, in mm
    * @param  filter_Sign Filter enable flag: 0-Off; 1-On (default off)
    * @param  posAdapt_sign Pose adaptation enable flag: 0-Off; 1-On (default off)
    * @param  isNoBlock Blocking flag: 0-Blocking; 1-Non-blocking
    * @return Error code
    */
    public int FT_Control(int flag, int sensor_id, int[] select, ForceTorque ft, double[] ft_pid, int adj_sign, int ILC_sign, double max_dis, double max_ang, double[] M, double[] B, double polishRadio, int filter_Sign, int posAdapt_sign, int isNoBlock)

Example Code for Constant Force Control with Damping
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void TestFTControlWithDamping(Robot robot)
    {
        int sensor_id = 10;
        int[] select = { 0,0,1,0,0,0 };
        double[] ft_pid = { 0.0008, 0.0, 0.0, 0.0, 0.0, 0.0 };
        int adj_sign = 0;
        int ILC_sign = 0;
        double max_dis = 100.0;
        double max_ang = 20;
        ForceTorque ft = new ForceTorque(0, 0, 0, 0, 0, 0);
        ft.fz = -10.0;
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        JointPos j1 = new JointPos(-118.985, -86.882, -118.139, -65.019, 90.002, 54.951);
        JointPos j2 = new JointPos(-77.055, -77.218, -126.219, -66.591, 90.028, 96.881);
        DescPose desc_p1 = new DescPose(-300.856, -332.618, 309.240, 179.976, -0.031, 96.065);
        DescPose desc_p2 = new DescPose(-16.399, -383.760, 309.312, 179.975, -0.031, 96.064);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        double[] M = {2.0, 2.0};
        double[] B = {8.0, 8.0};
        double polishRadio;
        int filter_Sign;
        int posAdapt_sign;
        int isNoBlock;
        DescPose ftCoord = new DescPose();
        robot.FT_SetRCS(2, ftCoord);
        int rtn = robot.FT_Control(1, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, 0, 0, 1, 0);
        System.out.printf("FT_Control start rtn is %d\n", rtn);
        rtn = robot.MoveL(j1, desc_p1, 0, 0, 100.0, 100.0, 20.0, -1.0, 0, epos, 0, 0, offset_pos, 0, 0, 10);
        rtn = robot.MoveL(j2, desc_p2, 0, 0, 100.0, 100.0, 20.0, -1.0, 0, epos, 0, 0, offset_pos, 0, 0, 10);
        rtn = robot.FT_Control(1, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, 0, 0, 1, 0);
        System.out.printf("FT_Control end rtn is %d\n", rtn);
        robot.CloseRPC();
    }

Constant Force Control Example
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestFTControl(Robot robot)
    {
        DescTran tr1=new DescTran(0,0,0);
        robot.SetForceSensorPayload(0);
        robot.SetForceSensorPayloadCog(tr1);

        int company = 24;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int index = 1;
        DeviceConfig con=new DeviceConfig(company, device, softversion, bus);

        robot.FT_SetConfig(con);
        robot.Sleep(1000);
        robot.FT_GetConfig(con);
        robot.Sleep(1000);

        robot.FT_Activate(0);
        robot.Sleep(1000);
        robot.FT_Activate(1);
        robot.Sleep(1000);

        robot.Sleep(1000);
        robot.FT_SetZero(0);
        robot.Sleep(1000);

        int sensor_id = 1;
        Object[] select =new Object[] { 0,0,1,0,0,0 };
        Object[] ft_pid =new Object[]{ 0.0005,0.0,0.0,0.0,0.0,0.0 };
        int adj_sign = 0;
        int ILC_sign = 0;
        double max_dis = 100.0;
        double max_ang = 0.0;

        ForceTorque ft=new ForceTorque(0,0,0,0,0,0);
        ExaxisPos epos=new ExaxisPos(0, 0, 0, 0);
        JointPos j1=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2=new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        DescPose desc_p1=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_p2=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose offset_pos=new DescPose(0, 0, 0, 0, 0, 0);

        ft.fz = -10.0;

        int rtn = robot.MoveJ(j1, desc_p1, 0, 0, 100.0, 180.0, 100.0, epos, -1.0, 0, offset_pos);
        robot.FT_Control(1, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang,0,0,0);
        rtn = robot.MoveJ(j2, desc_p2, 0, 0, 100.0, 180.0, 100.0, epos, -1.0, 0, offset_pos);
        robot.FT_Control(0, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang,0,0,0);
        return 0;
    }

Compliance Control Start
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Compliance control start
    * @param  [in] p Position adjustment coefficient or compliance coefficient
    * @param  [in] force Compliance activation force threshold in N
    * @return  Error code
    */   
    int FT_ComplianceStart(double p, double force);

Compliance Control Stop
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Compliance control stop
    * @return  Error code
    */   
    int FT_ComplianceStop(); 

Compliance Control Example
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestCompliance(Robot robot)
    {
        DescTran tr1=new DescTran(0,0,0);
        robot.SetForceSensorPayload(0);
        robot.SetForceSensorPayloadCog(tr1);

        int company = 24;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int index = 1;

        DeviceConfig con=new DeviceConfig(company, device, softversion, bus);
        robot.FT_SetConfig(con);
        robot.Sleep(1000);
        robot.FT_GetConfig(con);

        robot.Sleep(1000);

        robot.FT_Activate(0);
        robot.Sleep(1000);
        robot.FT_Activate(1);
        robot.Sleep(1000);

        robot.Sleep(1000);
        robot.FT_SetZero(0);
        robot.Sleep(1000);

        int flag = 1;
        int sensor_id = 1;
        Object[] select =new Object[] { 1,1,1,0,0,0 };
        Object[] ft_pid =new Object[] { 0.0005,0.0,0.0,0.0,0.0,0.0 };
        int adj_sign = 0;
        int ILC_sign = 0;
        double max_dis = 100.0;
        double max_ang = 0.0;

        ForceTorque ft=new ForceTorque(0,0,0,0,0,0);
        DescPose  offset_pos=new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos=new ExaxisPos(0, 0, 0, 0);


        JointPos j1=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2=new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        DescPose desc_p1=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_p2=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

        ft.fx = -10.0;
        ft.fy = -10.0;
        ft.fz = -10.0;
        robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
        double p = 0.00005;
        double force = 30.0;
        int rtn = robot.FT_ComplianceStart(p, force);

        int count = 15;
        while (count>0)
        {
            robot.MoveL(j1, desc_p1, 0, 0, 100.0, 180.0, 100.0, -1.0,0, epos, 0, 1, offset_pos,0,10);
            robot.MoveL(j2, desc_p2, 0, 0, 100.0, 180.0, 100.0, -1.0,0, epos, 0, 0, offset_pos,0,10);
            count -= 1;
        }
        robot.FT_ComplianceStop();
        flag = 0;
        robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

        robot.CloseRPC();
        return 0;
    }

Load Identification Initialization
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Load identification initialization
    * @return Error code
    */
    int LoadIdentifyDynFilterInit();

Load Identification Variable Initialization
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Load identification variable initialization
    * @return Error code
    */
    int LoadIdentifyDynVarInit();

Load Identification Main Program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Load identification main program
    * @param [in] joint_torque Joint torque
    * @param [in] joint_pos Joint position
    * @param [in] t Sampling period
    * @return Error code
    */
    int LoadIdentifyMain(Object[] joint_torque, Object[] joint_pos, double t);

Get Load Identification Result
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get load identification result
    * @param [in] gain
    * @return List[0]: Error code; List[1]: double weight Load weight; List[2]: x Load center of gravity X in mm; List[3]: y Load center of gravity Y in mm; List[2]: z Load center of gravity Z in mm
    */
    List<Number> LoadIdentifyGetResult(Object[] gain);

Robot Load Identification Example
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestIdentify(Robot robot)
    {
        int retval = 0;

        retval = robot.LoadIdentifyDynFilterInit();

        retval = robot.LoadIdentifyDynVarInit();

        JointPos posJ = new JointPos(0,0,0,0,0,0);
        DescPose posDec = new DescPose(0,0,0,0,0,0);
        List<Number> joint_toq=new ArrayList<>();
        robot.GetActualJointPosDegree( posJ);
        posJ.J2 = posJ.J2 + 10;
        joint_toq=robot.GetJointTorques(0);

        Object[] gain =new Object[] { 0,0.05,0,0,0,0,0,0.02,0,0,0,0 };
        double weight = 0;
        DescTran load_pos=new DescTran(0,0,0);
        List<Number> num=new ArrayList<>();
        num = robot.LoadIdentifyGetResult(gain);

        robot.CloseRPC();
        return 0;

    }

Force Sensor Assisted Dragging
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Java SDK-v1.0.6-3.8.3

.. code-block:: Java
    :linenos:

    /**
    * @brief Force sensor assisted dragging
    * @param [in] status Control status, 0-Disable; 1-Enable
    * @param [in] asaptiveFlag Adaptive enable flag, 0-Disable; 1-Enable
    * @param [in] interfereDragFlag Interference zone dragging flag, 0-Disable; 1-Enable
    * @param [in] ingularityConstraintsFlag Singularity strategy, 0-Avoid; 1-Traverse
    * @param [in] forceCollisionFlag Robot collision detection flag during assisted dragging; 0-Disable; 1-Enable
    * @param [in] M Inertia coefficient
    * @param [in] B Damping coefficient
    * @param [in] K Stiffness coefficient
    * @param [in] F Dragging six-dimensional force threshold
    * @param [in] Fmax Maximum dragging force limit in Nm
    * @param [in] Vmax Maximum joint speed limit in °/s
    * @return Error code
    */
    int EndForceDragControl(int status, int asaptiveFlag, int interfereDragFlag,int ingularityConstraintsFlag, int forceCollisionFlag, Object[] M, Object[] B, Object[] K, Object[] F, double Fmax, double Vmax)

Get Force Sensor Dragging Switch Status
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get force sensor dragging switch status
    * @return List[0]: Error code; List[1]: dragState Force sensor assisted dragging control status, 0-Disable; 1-Enable; List[1]: sixDimensionalDragState Six-dimensional force assisted dragging control status, 0-Disable; 1-Enable
    */
    List<Integer> GetForceAndTorqueDragState();

Force Sensor Auto-Enable After Error Clearance
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Force sensor auto-enable after error clearance
    * @param [in] status Control status, 0-Disable; 1-Enable
    * @return Error code
    */
    int SetForceSensorDragAutoFlag(int status)

Force Sensor Assisted Dragging Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestEndForceDragCtrl(Robot robot)
    {
        DescTran tr1=new DescTran(0,0,0);
        robot.SetForceSensorPayload(0);
        robot.SetForceSensorPayloadCog(tr1);

        robot.SetForceSensorDragAutoFlag(1);

        Object[] M =new Object[] { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
        Object[] B =new Object[] { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
        Object[] K =new Object[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        Object[] F =new Object[] { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
        robot.EndForceDragControl(1, 0, 0, 0, M, B, K, F, 50, 100);

        robot.Sleep(10000);

        int dragState = 0;
        int sixDimensionalDragState = 0;
        List<Integer> state=new ArrayList<>();
        state=robot.GetForceAndTorqueDragState();

        robot.EndForceDragControl(0, 0, 0, 0, M, B, K, F, 50, 100);
        return 0;
    }

Set Six-Dimensional Force and Joint Impedance Hybrid Dragging Switch and Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set six-dimensional force and joint impedance hybrid dragging switch and parameters
    * @param [in] status Control status, 0-Disable; 1-Enable
    * @param [in] impedanceFlag Impedance enable flag, 0-Disable; 1-Enable
    * @param [in] lamdeGain Dragging gain
    * @param [in] KGain Stiffness gain
    * @param [in] BGain Damping gain
    * @param [in] dragMaxTcpVel Maximum end-effector linear velocity limit during dragging
    * @param [in] dragMaxTcpOriVel Maximum end-effector angular velocity limit during dragging
    * @return Error code
    */
    int ForceAndJointImpedanceStartStop(int status, int impedanceFlag, Object[] lamdeGain, Object[] KGain, Object[] BGain, double dragMaxTcpVel, double dragMaxTcpOriVel);

Six-Dimensional Force and Joint Impedance Hybrid Dragging Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestForceAndJointImpedance(Robot robot)
    {
        robot.DragTeachSwitch(1);
        Object[] lamdeDain =new Object[] { 3.0, 2.0, 2.0, 2.0, 2.0, 3.0 };
        Object[] KGain = new Object[]{ 0, 0, 0, 0, 0, 0 };
        Object[] BGain =new Object[] { 150, 150, 150, 5.0, 5.0, 1.0 };
        int rtn = robot.ForceAndJointImpedanceStartStop(1, 0, lamdeDain, KGain, BGain, 1000.0, 180.0);

        robot.Sleep(10000);

        robot.DragTeachSwitch(0);
        rtn = robot.ForceAndJointImpedanceStartStop(0, 0, lamdeDain, KGain, BGain, 1000.0, 180.0);

        robot.CloseRPC();
        return 0;
    }

Example Program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    private static void TestUDPWireSearch(Robot robot)
    {
        UDPComParam param = new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10,0);
        robot.ExtDevSetUDPComParam(param);//UDP extended axis communication

        robot.SetWireSearchExtDIONum(0, 0);

        int rtn0, rtn1, rtn2 = 0;
        ExaxisPos exaxisPos = new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offdese = new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        DescPose descStart = new DescPose(-158.767, -510.596, 271.709, -179.427, -0.745, -137.349);
        JointPos jointStart = new JointPos(61.667, -79.848, 108.639, -119.682, -89.700, -70.985);

        DescPose descEnd = new DescPose(0.332, -516.427, 270.688, 178.165, 0.017, -119.989);
        JointPos jointEnd = new JointPos(79.021, -81.839, 110.752, -118.298, -91.729, -70.981);

        robot.MoveL(jointStart, descStart, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);
        robot.MoveL(jointEnd, descEnd, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);

        DescPose descREF0A = new DescPose(-66.106, -560.746, 270.381, 176.479, -0.126, -126.745);
        JointPos jointREF0A = new JointPos(73.531, -75.588, 102.941, -116.250, -93.347, -69.689);

        DescPose descREF0B = new DescPose(-66.109, -528.440, 270.407, 176.479, -0.129, -126.744);
        JointPos jointREF0B = new JointPos(72.534, -79.625, 108.046, -117.379, -93.366, -70.687);

        DescPose descREF1A = new DescPose(72.975, -473.242, 270.399, 176.479, -0.129, -126.744);
        JointPos jointREF1A = new JointPos(87.169, -86.509, 115.710, -117.341, -92.993, -56.034);

        DescPose descREF1B = new DescPose(31.355, -473.238, 270.405, 176.480, -0.130, -126.745);
        JointPos jointREF1B = new JointPos(82.117, -87.146, 116.470, -117.737, -93.145, -61.090);

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF0A, descREF0A, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);  //Starting point
        robot.MoveL(jointREF0B, descREF0B, 1, 0, 10, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);  //Direction point
        rtn1 = robot.WireSearchWait("REF0");
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF1A, descREF1A, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);  //Starting point
        robot.MoveL(jointREF1B, descREF1B, 1, 0, 10, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);  //Direction point
        rtn1 = robot.WireSearchWait("REF1");
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF0A, descREF0A, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);  //Starting point
        robot.MoveL(jointREF0B, descREF0B, 1, 0, 10, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);  //Direction point
        rtn1 = robot.WireSearchWait("RES0");
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF1A, descREF1A, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);  //Starting point
        robot.MoveL(jointREF1B, descREF1B, 1, 0, 10, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);  //Direction point
        
        rtn1 = robot.WireSearchWait("RES1");
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        String[] varNameRef = {"REF0", "REF1", "#", "#", "#", "#"};
        String[] varNameRes = {"RES0", "RES1", "#", "#", "#", "#"};
        int offectFlag = 0;
        //DescPose offectPos = new DescPose(0, 0, 0, 0, 0, 0);
        DescOffset offset = new DescOffset();
        rtn0 = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes, offset);
        robot.PointsOffsetEnable(0, offset.offset);
        robot.MoveL(jointStart, descStart, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);
        robot.MoveL(jointEnd, descEnd, 1, 0, 100, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);
        robot.PointsOffsetDisable();
    }

Impedance Control Start/Stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
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
.. code-block:: Java
    :linenos:

    public static int TestImpedanceControl(Robot robot)
    {
        JointPos j1=new JointPos(102.622, -135.990, 120.769, -73.950, -90.848, 35.507);
        JointPos j2=new JointPos(93.674, -80.062, 82.947, -92.199, -90.967, 26.559);
        DescPose desc_pos1=new DescPose(136.552, -149.799, 449.532, 179.817, -1.172, 157.123);
        DescPose desc_pos2=new DescPose(136.540, -561.048, 449.542, 179.819, -1.172, 157.122);
        DescPose offset_pos=new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos=new ExaxisPos(0, 0, 0, 0);
        int tool = 0;
        int user = 0;
        double vel = 100.0;
        double acc = 200.0;
        double ovl = 100.0;
        double blendT = -1.0;
        double blendR = -1.0;
        int flag = 0;
        int search = 0;
        robot.SetSpeed(20);
        int company = 17;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        DeviceConfig con=new DeviceConfig(company, device, softversion, bus);
        robot.FT_SetConfig(con);
        robot.Sleep(1000);
        robot.FT_GetConfig(con);
        System.out.println("FT config:"+con.company+","+con.device+","+con.softwareVersion+"con"+ con.bus);
        robot.Sleep(1000);
        robot.FT_Activate(0);
        robot.Sleep(1000);
        robot.FT_Activate(1);
        robot.Sleep(1000);
        robot.Sleep(1000);
        robot.FT_SetZero(0);
        robot.Sleep(1000);
        robot.FT_SetZero(1);
        robot.Sleep(1000);
        double[] forceThreshold = { 30,30,30,5,5,5 };
        double[] m= { 0.1,0.1,0.1,0.02,0.02,0.02 };
        double[] b = { 1,1,1,0.08,0.08,0.08 };
        double[] k = { 0,0,0,0,0,0 };
        int rtn = robot.ImpedanceControlStartStop(1, 1, forceThreshold, m, b, k, 1000, 500, 100, 100);
        System.out.println("ImpedanceControlStartStop errcode:"+ rtn);
        rtn = robot.MoveL(desc_pos1, tool, user, vel, acc, ovl, blendR, 0, epos, search, flag, offset_pos, -1,0,-1, 1);
        rtn = robot.MoveL(desc_pos2, tool, user, vel, acc, ovl, blendR, 0, epos, search, flag, offset_pos, -1,0,-1, 1);
        rtn = robot.MoveL(desc_pos1, tool, user, vel, acc, ovl, blendR, 0, epos, search, flag, offset_pos, -1,0,-1, 1);
        rtn = robot.MoveL(desc_pos2, tool, user, vel, acc, ovl, blendR, 0, epos, search, flag, offset_pos, -1,0,-1, 1);
        rtn = robot.MoveL(desc_pos1, tool, user, vel, acc, ovl, blendR, 0, epos, search, flag, offset_pos, -1,0,-1, 1);
        rtn = robot.MoveL(desc_pos2, tool, user, vel, acc, ovl, blendR, 0, epos, search, flag, offset_pos, -1,0,-1, 1);
        System.out.println("movel errcode:"+ rtn);
        robot.ImpedanceControlStartStop(0, 1, forceThreshold, m, b, k, 1000, 500, 100, 100);
        robot.CloseRPC();
        return 0;
    }
