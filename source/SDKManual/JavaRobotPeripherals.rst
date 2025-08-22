Robot peripherals
========================

.. toctree:: 
    :maxdepth: 5

Configure gripper
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Configure gripper
    * @param  [in] config .company  Gripper manufacturer, 1-Robotiq, 2-Huiling, 3-Tianji, 4-Dahuan, 5-Zhixing
    * @param  [in] config .device  Device number, Robotiq(0-2F-85 series), Huiling(0-NK series,1-Z-EFG-100), Tianji(0-TEG-110), Dahuan(0-PGI-140), Zhixing(0-CTPM2F20)
    * @param  [in] config .softvesion  Software version, not currently used, default 0
    * @param  [in] config .bus  Device bus position at end, not currently used, default 0
    * @return  Error code
    */
    int SetGripperConfig(DeviceConfig config);

Get gripper configuration
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get gripper configuration
    * @param  [out] config .company  Gripper manufacturer, 1-Robotiq, 2-Huiling, 3-Tianji, 4-Dahuan, 5-Zhixing
    * @param  [out] config .device  Device number, Robotiq(0-2F-85 series), Huiling(0-NK series,1-Z-EFG-100), Tianji(0-TEG-110), Dahuan(0-PGI-140), Zhixing(0-CTPM2F20)
    * @param  [out] config .softvesion  Software version, not currently used, default 0
    * @param  [out] config .bus  Device bus position at end, not currently used, default 0
    * @return  Error code
    */
    int GetGripperConfig(DeviceConfig config);

Activate gripper
++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Activate gripper
    * @param  [in] index  Gripper number
    * @param  [in] act  0-reset, 1-activate
    * @return  Error code
    */
    int ActGripper(int index, int act); 

Control gripper
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Control gripper
    * @param  [in] index  Gripper number
    * @param  [in] pos  Position percentage, range [0~100]
    * @param  [in] vel  Speed percentage, range [0~100]
    * @param  [in] force  Torque percentage, range [0~100]
    * @param  [in] max_time  Maximum wait time, range [0~30000], unit ms
    * @param  [in] block  0-blocking, 1-non-blocking
    * @param  [in] type Gripper type, 0-parallel gripper; 1-rotary gripper
    * @param  [in] rotNum Rotation turns
    * @param  [in] rotVel Rotation speed percentage [0-100]
    * @param  [in] rotTorque Rotation torque percentage [0-100]
    * @return Error code
    */
    int MoveGripper(int index, int pos, int vel, int force, int max_time, int block, int type, double rotNum, int rotVel, int rotTorque); 

Get gripper motion status
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get gripper motion status
    * @return List[0]:Error code; List[1] : fault  0-no error, 1-error; List[2]: staus  0-motion not completed, 1-motion completed
    */
    List<Integer> GetGripperMotionDone(); 

Get gripper activation status
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get gripper activation status
    * @return  List[0]:Error code; List[1] : fault  0-no error, 1-error; List[2]: status  bit0~bit15 correspond to gripper numbers 0~15, bit=0 not activated, bit=1 activated
    */
    List<Number> GetGripperActivateStatus()

Get gripper position
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get gripper position
    * @return  List[0]:Error code; List[1] : fault  0-no error, 1-error; List[2]: position  Position percentage, range 0~100%
    */
    List<Number> GetGripperCurPosition()

Get gripper speed
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get gripper speed
    * @return  List[0]:Error code; List[1] : fault  0-no error, 1-error; List[2]: speed  Speed percentage, range 0~100%
    */
    List<Number> GetGripperCurSpeed()

Get gripper current
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get gripper current
    * @return  List[0]:Error code; List[1] : fault  0-no error, 1-error; List[2]: current  Current percentage, range 0~100%
    */
    List<Number> GetGripperCurCurrent()

Get gripper voltage
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get gripper voltage
    * @return List[0]:Error code; List[1] : fault  0-no error, 1-error; List[2]:voltage  Voltage, unit 0.1V
    */
    List<Number> GetGripperVoltage()

Get gripper temperature
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get gripper temperature
    * @return List[0]:Error code; List[1] : fault  0-no error, 1-error; List[2]:temp  Temperature, unit ℃
    */
    List<Number> GetGripperTemp()

Calculate pre-grasp point - vision
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Calculate pre-grasp point - vision 
    * @param [in] desc_pos  Grasp point cartesian pose
    * @param [in] zlength   z-axis offset
    * @param [in] zangle    Rotation offset around z-axis
    * @param [out] pre_pos  Pre-grasp point
    * @return Error code 
    */ 
    int ComputePrePick(DescPose desc_pos, double zlength, double zangle, DescPose pre_pos);

Calculate retreat point - vision
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Calculate retreat point - vision 
    * @param [in] desc_pos  Grasp point cartesian pose
    * @param [in] zlength   z-axis offset 
    * @param [in] zangle    Rotation offset around z-axis
    * @param [out] post_poss Retreat point
    * @return Error code 
    */ 
    int ComputePostPick(DescPose desc_pos, double zlength, double zangle, DescPose post_pos);

Robot gripper operation code example
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestGripper(Robot robot)
    {
        int company = 4;
        int device = 0;
        int softversion = 0;
        int bus = 2;
        int index = 2;
        int act = 0;
        int max_time = 30000;
        int block = 0;

        int current_pos = 0;
        int current = 0;
        int voltage = 0;
        int temp = 0;
        int speed = 0;

        DeviceConfig cnn=new DeviceConfig(company,device,softversion,bus);
        robot.SetGripperConfig(cnn);
        robot.GetGripperConfig(cnn);

        robot.ActGripper(index, act);
        robot.Sleep(1000);
        act = 1;
        robot.ActGripper(index, act);
        robot.Sleep(1000);

        robot.MoveGripper(index, 100, 50, 50, max_time, block, 0, 0, 0, 0);
        robot.Sleep(1000);
        robot.MoveGripper(index, 0, 50, 0, max_time, block, 0, 0, 0, 0);

        List<Integer> stat=new ArrayList<>();
        stat=robot.GetGripperMotionDone();

        List<Number> list=new ArrayList<>();
        list=robot.GetGripperActivateStatus();

        list=robot.GetGripperCurPosition();

        list=robot.GetGripperCurCurrent();

        list=robot.GetGripperVoltage();

        list=robot.GetGripperTemp();

        list=robot.GetGripperCurSpeed();

        int retval = 0;
        DescPose prepick_pose = new DescPose(){};
        DescPose postpick_pose = new DescPose(){};

        DescPose p1Desc=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose p2Desc=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

        retval = robot.ComputePrePick(p1Desc, 10, 0, prepick_pose);

        retval = robot.ComputePostPick(p2Desc, -10, 0, postpick_pose);
        return 0;
    }

Get rotary gripper rotation turns
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get rotary gripper rotation turns
    * @return List[0]:Error code List[1]: 0-no error, 1-error List[2]:Rotation turns
    */
    List<Number> GetGripperRotNum(); 

Get rotary gripper rotation speed percentage
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get rotary gripper rotation speed percentage
    * @return List[0]:Error code List[1]: 0-no error, 1-error List[2]:Rotation speed percentage
    */
    List<Number> GetGripperRotSpeed(); 

Get rotary gripper rotation torque percentage
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get rotary gripper rotation torque percentage
    * @return List[0]:Error code List[1]: 0-no error, 1-error List[2]:Rotation torque percentage
    */
    List<Number> GetGripperRotTorque(); 

Code example for getting rotary gripper status
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestRotGripperState(Robot robot)
    {
        int fault = 0;
        List<Number> rotNum=new ArrayList<>();
        List<Number> rotSpeed=new ArrayList<>();
        List<Number> rotTorque=new ArrayList<>();

        rotNum=robot.GetGripperRotNum();
        rotSpeed=robot.GetGripperRotSpeed();
        rotTorque=robot.GetGripperRotTorque();
        System.out.println("gripper rot num :"+rotNum.get(2)+ ", gripper rotSpeed :"+rotSpeed.get(2)+",gripper rotTorque : "+rotTorque.get(2));

        return 0;
    }

Conveyor start/stop
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Conveyor start/stop
    * @param  [in] status Status, 1-start, 0-stop
    * @return  Error code
    */
    int ConveyorStartEnd(int status);

Record IO detection point
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Record IO detection point
    * @return  Error code
    */
    int ConveyorPointIORecord();

Record point A
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Record point A
    * @return  Error code
    */
    int ConveyorPointARecord(); 

Record reference point
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Record reference point
    * @return  Error code
    */
    int ConveyorRefPointRecord();

Record point B
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Record point B
    * @return Error code
    */
    int ConveyorPointBRecord(); 

Conveyor workpiece IO detection
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Conveyor workpiece IO detection
    * @param [in] max_t Maximum detection time, unit ms
    * @return Error code 
    */ 
    int ConveyorIODetect(int max_t);

Get object current position
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get object current position
    * @param [in] mode 1-tracking grasp, 2-tracking motion, 3-TPD tracking
    * @return Error code 
    */ 
    int ConveyorGetTrackData(int mode);

Conveyor tracking start
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Conveyor tracking start
    * @param [in] status Status, 1-start, 0-stop
    * @return Error code 
    */ 
    int ConveyorTrackStart(int status);

Conveyor tracking stop
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Conveyor tracking stop
    * @return Error code 
    */ 
    int ConveyorTrackEnd();

Conveyor parameter configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Java SDK-v1.0.4-3.8.1

.. code-block:: Java
    :linenos:

    /**
    * @brief  Conveyor parameter configuration
    * @param [in] encChannel Encoder channel 1~2
    * @param [in] resolution Pulses per encoder revolution
    * @param [in] lead Conveyor travel distance per encoder revolution
    * @param [in] wpAxis Workpiece coordinate system number For tracking motion function, set to 0 for tracking grasp and TPD tracking
    * @param [in] vision Whether vision is configured  0 no  1 yes
    * @param [in] speedRadio Speed ratio  For conveyor tracking grasp option (1-100)  Default 1 for other options
    * @param [in] followType Tracking motion type, 0-tracking motion; 1-chasing inspection motion
    * @param [in] startDis Required for chasing grasp, tracking start distance, -1: auto calculate (automatically chase after workpiece reaches below robot), unit mm, default 0
    * @param [in] endDis Required for chasing grasp, tracking end distance, unit mm, default 100
    * @return Error code
    */
    int ConveyorSetParam(int encChannel, int resolution, double lead, int wpAxis, int vision, double speedRadio, int followType, int startDis, int endDis); 

Set conveyor grasp point compensation
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set conveyor grasp point compensation
    * @param [in] cmp Compensation position double[3]{x, y, z}
    * @return Error code 
    */ 
    int ConveyorCatchPointComp(Object[] cmp);

Conveyor linear motion
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Linear motion
    * @param [in] name Motion point description
    * @param [in] tool Tool coordinate number, range [0~14]
    * @param [in] wobj Workpiece coordinate number, range [0~14]
    * @param [in] vel Speed percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not currently available
    * @param [in] ovl Speed scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-move to position (blocking), [0~1000.0]-smoothing radius (non-blocking), unit mm
    * @return Error code 
    */ 
    int ConveyorTrackMoveL(String name, int tool, int wobj, double vel, double acc, double ovl, double blendR);   

Conveyor communication input detection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.4-3.8.1

.. code-block:: Java
    :linenos:

    /** 
    * @brief Conveyor communication input detection
    * @param [in] timeout Wait timeout time ms
    * @return Error code
    */
    int ConveyorComDetect(int timeout);

Conveyor communication input detection trigger
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.4-3.8.1

.. code-block:: Java
    :linenos:

    /** 
    * @brief Conveyor communication input detection trigger
    * @param [in] timeout Wait timeout time ms
    * @return Error code
    */
    int ConveyorComDetectTrigger();

Robot conveyor operation example program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestConveyor(Robot robot)
    {
        int retval = 0;

        retval = robot.ConveyorStartEnd(1);

        retval = robot.ConveyorPointIORecord();

        retval = robot.ConveyorPointARecord();

        retval = robot.ConveyorRefPointRecord();

        retval = robot.ConveyorPointBRecord();

        retval = robot.ConveyorStartEnd(0);

        retval = 0;

        retval = robot.ConveyorSetParam(1,10000,200,0,0,20,0,0,100);

        Object[] cmp = new Object[]{ 0.0, 0.0, 0.0 };
        retval = robot.ConveyorCatchPointComp(cmp);

        int index = 1;
        int max_time = 30000;
        int block = 0;
        retval = 0;

        DescPose p1Desc=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose p2Desc=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);


        retval = robot.MoveCart(p1Desc, 1, 0, 100.0, 100.0, 100.0, -1.0, -1);

        retval = robot.WaitMs(1);

        retval = robot.ConveyorTrackStart(1);

        retval = robot.ConveyorTrackMoveL("cvrCatchPoint", 1, 0, 100, 100, 100, -1.0);

        retval = robot.MoveGripper(index, 51, 40, 30, max_time, block, 0, 0, 0, 0);

        retval = robot.ConveyorTrackMoveL("cvrRaisePoint", 1, 0, 100, 100, 100, -1.0);

        retval = robot.ConveyorTrackEnd();

        robot.MoveCart(p2Desc, 1, 0, 100.0, 100.0, 100.0, -1.0, -1);

        retval = robot.MoveGripper(index, 100, 40, 10, max_time, block, 0, 0, 0, 0);

        return 0;
    }

End sensor configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief End sensor configuration
    * @param [in] config idCompany Manufacturer, 18-JUNKONG; 25-HUIDE
    * @param [in] config idDevice Type, 0-JUNKONG/RYR6T.V1.0
    * @param [in] config idSoftware Software version, 0-J1.0/HuiDe1.0(not currently available)
    * @param [in] config idBus Mount position, 1-end port 1; 2-end port 2...8-end port 8(not currently available)
    * @return Error code
    */
    int AxleSensorConfig(DeviceConfig config);

Get end sensor configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get end sensor configuration
    * @param [out] config idCompany Manufacturer, 18-JUNKONG; 25-HUIDE
    * @param [out] config idDevice Type, 0-JUNKONG/RYR6T.V1.0
    * @return Error code
    */
    int AxleSensorConfigGet(DeviceConfig config);

End sensor activation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief End sensor activation
    * @param [in] actFlag 0-reset; 1-activate
    * @return Error code
    */
    int AxleSensorActivate(int actFlag);

End sensor register write
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief End sensor register write
    * @param [in] devAddr  Device address number 0-255
    * @param [in] regHAddr Register address high 8 bits
    * @param [in] regLAddr Register address low 8 bits
    * @param [in] regNum  Register count 0-255
    * @param [in] data1 Write register value 1
    * @param [in] data2 Write register value 2
    * @param [in] isNoBlock 0-blocking; 1-non-blocking
    * @return Error code
    */
    int AxleSensorRegWrite(int devAddr, int regHAddr, int regLAddr, int regNum, int data1, int data2, int isNoBlock);

End sensor code example
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestAxleSensor(Robot robot)
    {
        DeviceConfig con=new DeviceConfig(18,0,0,1);
        robot.AxleSensorConfig(con);
        int company = -1;
        int type = -1;
        robot.AxleSensorConfigGet(con);

        int rtn = robot.AxleSensorActivate(1);

        robot.Sleep(1000);

        rtn = robot.AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0);
        return 0;
    }

Get robot peripheral protocol
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get robot peripheral protocol
    * @return List[0]:Error code; List[1] : int protocol Robot peripheral protocol number 4096-extension axis control card; 4097-ModbusSlave; 4098-ModbusMaster 
    */
    List<Integer> GetExDevProtocol();

Set robot peripheral protocol
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set robot peripheral protocol
    * @param [in] protocol Robot peripheral protocol number 4096-extension axis control card; 4097-ModbusSlave; 4098-ModbusMaster
    * @return Error code 
    */
    int SetExDevProtocol(int protocol);

Set robot peripheral protocol example program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestExDevProtocol(Robot robot)
    {
        int protocol = 4096;
        int rtn = robot.SetExDevProtocol(protocol);
        List<Integer> integer=new ArrayList<>();
        integer = robot.GetExDevProtocol();

        return 0;
    }

Get end communication parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get end communication parameters
    * @param [out] param End communication parameters
    * @return Error code 
    */
    int GetAxleCommunicationParam(AxleComParam param)

Set end communication parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set end communication parameters
    * @param [in] param End communication parameters
    * @return Error code 
    */
    int SetAxleCommunicationParam(AxleComParam param)

Set end file transfer type
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set end file transfer type
    * @param [in] type 1-MCU upgrade file; 2-LUA file
    * @return  Error code
    */
    public int SetAxleFileType(int type)

Set enable end LUA execution
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set enable end LUA execution
    * @param [in] enable 0-disable; 1-enable
    * @return  Error code
    */
    public int SetAxleLuaEnable(int enable)

End LUA file error recovery
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief End LUA file error recovery
    * @param [in] status 0-no recovery; 1-recover
    * @return  Error code
    */
    public int SetRecoverAxleLuaErr(int status)

Get end LUA execution enable status
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get end LUA execution enable status
    * @param [out] status[0]: 0-disabled; 1-enabled
    * @return  Error code
    */
    int GetAxleLuaEnableStatus(int[] status)

Set end LUA end device enable type
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set end LUA end device enable type
    * @param forceSensorEnable Force sensor enable status, 0-disable; 1-enable
    * @param gripperEnable Gripper enable status, 0-disable; 1-enable
    * @param IOEnable IO device enable status, 0-disable; 1-enable
    * @return  Error code
    */
    public int SetAxleLuaEnableDeviceType(int forceSensorEnable, int gripperEnable, int IOEnable)

Get end LUA end device enable type
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Get end LUA end device enable type
     * @param enable enable[0]:forceSensorEnable Force sensor enable status, 0-disable; 1-enable
     * @param enable enable[1]:gripperEnable Gripper enable status, 0-disable; 1-enable
     * @param enable enable[2]:IOEnable IO device enable status, 0-disable; 1-enable
     * @return  Error code
     */
    public int GetAxleLuaEnableDeviceType(int[] enable)

Get currently configured end devices
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Get currently configured end devices
     * @param forceSensorEnable Force sensor enable device number 0-disabled; 1-enabled
     * @param gripperEnable Gripper enable device number, 0-disabled; 1-enabled
     * @param IODeviceEnable IO device enable device number, 0-disabled; 1-enabled
     * @return  Error code
     */
    public int GetAxleLuaEnableDevice(int[] forceSensorEnable, int[] gripperEnable, int[] IODeviceEnable)

Set enable gripper action control function
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Set enable gripper action control function
     * @param id Gripper device number
     * @param func func[0]-gripper enable; func[1]-gripper initialization; 2-position set; 3-speed set; 4-torque set; 6-read gripper status; 7-read initialization status; 8-read error code; 9-read position; 10-read speed; 11-read torque
     * @return  Error code
     */
    public int SetAxleLuaGripperFunc(int id, int[] func)

Get enable gripper action control function
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Get enable gripper action control function
     * @param id Gripper device number
     * @param func func[0]-gripper enable; func[1]-gripper initialization; 2-position set; 3-speed set; 4-torque set; 6-read gripper status; 7-read initialization status; 8-read error code; 9-read position; 10-read speed; 11-read torque
     * @return  Error code
     */
    public int GetAxleLuaGripperFunc(int id, int[] func)

Robot Ethercat slave file write
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Robot Ethercat slave file write
     * @param type Slave file type, 1-upgrade slave file; 2-upgrade slave configuration file
     * @param slaveID Slave number
     * @param fileName Upload file name
     * @return  Error code
     */
    public int SlaveFileWrite(int type, int slaveID, String fileName)

Upload end Lua open protocol file
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Upload end Lua open protocol file
     * @param filePath Local lua file path name ".../AXLE_LUA_End_DaHuan.lua"
     * @return Error code
     */
    public int AxleLuaUpload(String filePath)

Robot Ethercat slave enter boot mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Robot Ethercat slave enter boot mode
     * @return  Error code
     */
    public int SetSysServoBootMode()

Robot end LUA file operation code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestAxleLua(Robot robot)
    {
        robot.AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan.lua");

        AxleComParam param=new AxleComParam(7, 8, 1, 0, 5, 3, 1);
        robot.SetAxleCommunicationParam(param);

        robot.GetAxleCommunicationParam(param);

        robot.SetAxleLuaEnable(1);
        int[] luaEnableStatus = new int[]{0};
        robot.GetAxleLuaEnableStatus(luaEnableStatus);
        robot.SetAxleLuaEnableDeviceType(0, 1, 0);

        int forceEnable = 0;
        int gripperEnable = 0;
        int ioEnable = 0;
        int [] enable=new int[]{0,0,0};
        robot.GetAxleLuaEnableDeviceType(enable);

        int[] func = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
        robot.SetAxleLuaGripperFunc(1, func);
        int[] getFunc = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        robot.GetAxleLuaGripperFunc(1, getFunc);
        int[] getforceEnable = { 0,0,0,0,0,0,0,0};
        int[] getgripperEnable = { 0,0,0,0,0,0,0,0};
        int[] getioEnable = { 0,0,0,0,0,0,0,0};
        robot.GetAxleLuaEnableDevice(getforceEnable, getgripperEnable, getioEnable);
        for (int i = 0; i < 8; i++)
        {
            System.out.println(getforceEnable[i]);
        }
        System.out.println("getgripperEnable status : ");
        for (int i = 0; i < 8; i++)
        {
            System.out.println(getgripperEnable[i]);
        }
        System.out.println("getioEnable status : ");
        for (int i = 0; i < 8; i++)
        {
            System.out.println(getioEnable[i]);
        }
        robot.ActGripper(1, 0);
        robot.Sleep(2000);
        robot.ActGripper(1, 1);
        robot.Sleep(2000);
        robot.MoveGripper(1, 90, 10, 100, 50000, 0, 0, 0, 0, 0);
        int pos = 0;
        while (true)
        {
            ROBOT_STATE_PKG pkg=new ROBOT_STATE_PKG();
            pkg=robot.GetRobotRealTimeState();
            System.out.println("gripper pos is:"+pkg.gripper_position);
            robot.Sleep(100);
        }

    }

Get SmartTool button status
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.4-3.8.1

.. code-block:: Java
    :linenos:

    /**
    * @brief Get SmartTool button status
    * @param [out] state SmartTool handle button status;(bit0:0-communication normal; 1-communication lost; bit1-undo operation; bit2-clear program; bit3-A key; bit4-B key; bit5-C key; bit6-D key; bit7-E key; bit8-IO key; bit9-manual/auto; bit10-start)
    * @return Error code
    */
    int GetSmarttoolBtnState(int[] state)

SmartTool button code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args) 
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true, 100, 500);//Set reconnection times and interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn == 0) {
            System.out.println("rpc connection success");
        } else {
            System.out.println("rpc connection fail");
            return;
        }

        int[] state = {0};
        while (true)
        {
            robot.GetSmarttoolBtnState(state);

            String binaryString = String.format("%32s", Integer.toBinaryString(state[0])).replace(' ', '0');
            System.out.println("GetSmarttoolBtnState:"+binaryString);
            robot.Sleep(100);
        }
    }

Upload Open Protocol Lua File
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.8-3.8.5

.. code-block:: Java
    :linenos:

    /**
    * @brief Upload Open Protocol Lua File
    * @param  filePath Local open protocol lua file path name
    * @return Error code
    */
    public int OpenLuaUpload(String filePath)


Get Slave Board Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.8-3.8.5

.. code-block:: Java
    :linenos:

    /**
    * @brief  Get Slave Board Parameters
    * @param  type  0-Ethercat, 1-CClink, 3-Ethercat, 4-EIP
    * @param  version  Protocol version
    * @param  connState  0-Disconnected 1-Connected
    * @return  Error code
    */
    public int GetFieldBusConfig(int[] type, int[] version, int[] connState)

Write Slave DO
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.8-3.8.5

.. code-block:: Java
    :linenos:

    /**
    * @brief  Write Slave DO
    * @param   DOIndex  DO number
    * @param   wirteNum  Number to write
    * @param   status Value to write, max 8
    * @return  Error code
    */
    public int FieldBusSlaveWriteDO(int DOIndex, int wirteNum, int[] status)

Write Slave AO
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.8-3.8.5

.. code-block:: Java
    :linenos:

    /**
    * @brief  Write Slave AO
    * @param  AOIndex  AO number
    * @param  wirteNum  Number to write
    * @param  status Value to write, max 8
    * @return  Error code
    */
    public int FieldBusSlaveWriteAO(int AOIndex, int wirteNum, int[] status)

Read Slave DI
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.8-3.8.5

.. code-block:: Java
    :linenos:

    /**
    * @brief  Read Slave DI
    * @param  DOIndex  DI number
    * @param  readNum  Number to read
    * @param  status Read value, max 8
    * @return  Error code
    */
    public int FieldBusSlaveReadDI(int DOIndex, int readNum, int[] status)

Read Slave AI
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.8-3.8.5

.. code-block:: Java
    :linenos:

    /**
    * @brief  Read Slave AI
    * @param  AIIndex  AI number
    * @param  readNum  Number to read
    * @param  status Read value, max 8
    * @return  Error code
    */
    public int FieldBusSlaveReadAI(int AIIndex, int readNum, double[] status)

Wait for Extended DI Input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.8-3.8.5

.. code-block:: Java
    :linenos:

    /**
    * @brief Wait for Extended DI Input
    * @param  DIIndex DI number
    * @param  status 0-Low level; 1-High level
    * @param  waitMs Max waiting time (ms)
    * @return Error code
    */
    public int FieldBusSlaveWaitDI(int DIIndex, int status, int waitMs)

Wait for Extended AI Input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.8-3.8.5

.. code-block:: Java
    :linenos:

    /**
    * @brief Wait for Extended AI Input
    * @param  AIIndex AI number
    * @param  waitType 0-Greater than; 1-Less than
    * @param  value AI value
    * @param  waitMs Max waiting time (ms)
    * @return Error code
    */
    public int FieldBusSlaveWaitAI(int AIIndex, int waitType, double value, int waitMs)

Slave Mode Related Interface Command Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void testFieldBusBoard(Robot robot)
    {
        //Upload and load open protocol file
        robot.OpenLuaUpload("D://zUP/1111/CtrlDev_field.lua");
        robot.Sleep(2000);
        robot.SetCtrlOpenLUAName(3, "CtrlDev_field.lua");
        robot.UnloadCtrlOpenLUA(3);
        robot.LoadCtrlOpenLUA(3);
        robot.Sleep(8000);
        int[] type=new int[1];
        int[] version=new int[1];
        int[] connState=new int[1];
        //Get protocol type, software version, and PLC connection status of the slave board
        robot.GetFieldBusConfig(type, version, connState);
        System.out.println("type is: "+type[0]+", version is : "+version[0]+", connState is : "+connState[0]);
        //Write DO0 = 1, DO1 = 0, DO2 = 1
        int[] ctrl =new int[8];
        ctrl[0] = 1;
        ctrl[1] = 0;
        ctrl[2] = 1;
        robot.FieldBusSlaveWriteDO(0, 3, ctrl);
        //Write AO2 = 0x1000
        int[] ctrlAO =new int[8];
        ctrlAO[0] = 0x1000;
        robot.FieldBusSlaveWriteAO(2, 1, ctrlAO);
        int[] DI=new int[4];
        double[] AI=new double[3];
        //Loop monitor DI0~DI3 AI0~AI2
        for (int i = 0; i < 100; i++)
        {
            robot.FieldBusSlaveReadDI(0, 4, DI);
            System.out.println("DI0 is: "+DI[0]+", DI1 is: "+DI[1]+",DI2 is: "+DI[2]+",DI3 is: "+DI[3]);
            robot.FieldBusSlaveReadAI(0, 3, AI);
            System.out.println("AI0 is: "+AI[0]+ ",AI1 is: "+AI[1]+",AI2 is: "+AI[2]);
            robot.Sleep(10);
        }
        //Wait for DI0 to be 1, wait time 100ms, and print result
        int ret = robot.FieldBusSlaveWaitDI(0, 1, 100);
        System.out.println("FieldBusSlaveWaitDI result is: "+ ret);
        //Wait for AI0 to be greater than 400, wait time 100ms, and print result
        ret = robot.FieldBusSlaveWaitAI(0,0,400.00,100);
        System.out.println("FieldBusSlaveWaitAI result is: "+ ret);
        robot.CloseRPC();
    }

Control Array Sucker
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.8-3.8.5

.. code-block:: Java
    :linenos:

    /**
    * @brief Control Array Sucker
    * @param  slaveID Slave ID
    * @param  len Length
    * @param  ctrlValue Control value 1-Suction at max vacuum; 2-Suction at set vacuum; 3-Stop suction
    * @return Error code
    */
    public int SetSuckerCtrl(int slaveID, int len, int[] ctrlValue)

Get Array Sucker Status
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.8-3.8.5

.. code-block:: Java
    :linenos:

    /**
    * @brief Get Array Sucker Status
    * @param  slaveID Slave ID
    * @param  state Adsorption state 0-Release object; 1-Workpiece detected and adsorbed successfully; 2-No object adsorbed; 3-Object detached
    * @param  pressValue Current vacuum degree Unit kpa
    * @param  error Sucker current error code
    * @return Error code
    */
    public int GetSuckerState(int slaveID, int[] state, int[] pressValue, int[] error)

Wait for Sucker Status
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.8-3.8.5

.. code-block:: Java
    :linenos:

    /**
    * @brief Wait for Sucker Status
    * @param  slaveID Slave ID
    * @param  state Adsorption state 0-Release object; 1-Workpiece detected and adsorbed successfully; 2-No object adsorbed; 3-Object detached
    * @param  ms Max waiting time
    * @return Error code
    */
    public int WaitSuckerState(int slaveID, int state, int ms)

Array Sucker Control Command Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void testSucker(Robot robot)
    {
        //Upload and load open protocol file
        robot.OpenLuaUpload("C：//project/peripheralSDK/CtrlDev_sucker.lua");
        robot.Sleep(2000);
        robot.UnloadCtrlOpenLUA(1);
        robot.LoadCtrlOpenLUA(1);
        robot.Sleep(1000);
        //Control sucker in broadcast mode, adsorb with maximum capacity
        int[] ctrl = {1};
        robot.SetSuckerCtrl(0, 1, ctrl);
        int[] state=new int[1];
        int[] pressVlaue=new int[1];
        int[] error=new int[1];
        //Loop monitor status of sucker 1 and sucker 12
        for (int i = 0; i < 100; i++)
        {
            robot.GetSuckerState(1, state,pressVlaue, error);
            System.out.println("sucker1 state is:"+state[0]+",pressVlaue is:"+pressVlaue[0]+",error num is"+error[0]);
            robot.GetSuckerState(12, state, pressVlaue, error);
            System.out.println("sucker12 state is :"+state[0]+", pressVlaue is:"+pressVlaue[0]+",error num is:"+error[0]);
            robot.Sleep(100);
        }
        //Wait for sucker 1 to be in adsorbed state, wait time 100ms
        int ret = robot.WaitSuckerState(1, 1, 100);
        System.out.println("WaitSuckerState result is:"+ ret);
        //Unicast mode to turn off sucker 1 and 12
        ctrl[0] = 3;
        robot.SetSuckerCtrl(1, 1, ctrl);
        robot.SetSuckerCtrl(12, 1, ctrl);
        robot.CloseRPC();
    }