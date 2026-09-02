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
    * @brief  Get gripper motion status(only defined for the end-effector open protocol; for adapted devices, the obtained motion status is a pass-through value)
    * @return List[0]:Error code; List[1] : fault  0-no error, other-error; List[2]: staus  0-motion not completed, 1-motion completed with no object detected, 2-motion completed with object detected
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

Conveyor Belt In-Place Tracking Parameter Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Configure conveyor belt in-place tracking parameters
    * @param  [in] trackMode 0-time; 1-distance; 2-time and distance, either condition satisfied
    * @param  [in] trackTime Tracking time, unit s
    * @param  [in] trackDis Tracking distance
    * @return  Error code
    */
    public int SetStationaryTrackPara(int trackMode, double trackTime, int trackDis)

Conveyor Belt In-Place Tracking Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestStationaryTrack(Robot robot)
    {
        System.out.println("\n========== Conveyor Stationary Tracking Test ==========");

        int rtn;

        JointPos j1 = new JointPos(-35.146, -102.684, 120.805, -100.401, -90.295, 150.105);
        DescPose d1 = new DescPose(-121.814, -348.341, 209.978, -173.152, -3.585, -5.446);

        ExaxisPos ex = new ExaxisPos(0, 0, 0, 0);
        DescPose zeroOff = new DescPose(0, 0, 0, 0, 0, 0);

        int tool = 1;
        int workpiece = 1;

        rtn = robot.ConveyorSetParam(0, 10000, 200, 0, 0, 10,0,0,0);


        robot.MoveJ(j1, d1, tool, workpiece, 100, 100, 100, ex, -1, 0, zeroOff);

        // Step 1: SetDO control signal
        System.out.println("--- Step 1: SetDO(6,1) ---");
        rtn = robot.SetDO(6, 1, 0, 0);
        System.out.println("  SetDO(6,1) rtn=" + rtn);

        // Step 2: Conveyor tracking start
        System.out.println("--- Step 2: ConveyorTrackStart(2) ---");
        rtn = robot.ConveyorTrackStart(2);
        System.out.println("  ConveyorTrackStart(2) rtn=" + rtn);

        // Step 3: Workpiece IO detection
        System.out.println("--- Step 3: ConveyorIODetect(10000) ---");
        rtn = robot.ConveyorIODetect(10000);
        System.out.println("  ConveyorIODetect(10000) rtn=" + rtn);

        // Step 4: Get tracking data
        System.out.println("--- Step 4: ConveyorGetTrackData(2) ---");
        rtn = robot.ConveyorGetTrackData(2);
        System.out.println("  ConveyorGetTrackData(2) rtn=" + rtn);

        // Step 5: Stationary tracking parameter configuration (time mode, 200s, distance 5)
        System.out.println("--- Step 5: SetStationaryTrackPara(0,200,5) ---");
        rtn = robot.SetStationaryTrackPara(0, 5, 5);
        System.out.println("  SetStationaryTrackPara(0,200,5) rtn=" + rtn);

        // Step 6: Execute stationary tracking motion
        System.out.println("--- Step 6: MoveStationary() ---");
        rtn = robot.MoveStationary();
        robot.WaitStationaryMotionDone();
        System.out.println("  MoveStationary() rtn=" + rtn);

        // Step 7: Conveyor tracking end
        System.out.println("--- Step 7: ConveyorTrackEnd() ---");
        rtn = robot.ConveyorTrackEnd();
        System.out.println("  ConveyorTrackEnd() rtn=" + rtn);

        // Step 8: SetDO turn off signal
        System.out.println("--- Step 8: SetDO(6,0) ---");
        rtn = robot.SetDO(6, 0, 0, 0);
        System.out.println("  SetDO(6,0) rtn=" + rtn);

        System.out.println("\n========== Stationary Tracking Test Complete ==========");
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

Set the enabled device types for the end-effector LUA
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set the enabled device types for the end-effector LUA
    * @param forceSensorEnable Force sensor enable status, 0-disabled; 1-enabled
    * @param gripperEnable Gripper enable status, 0-disabled; 1-enabled
    * @param IOEnable IO device enable status, 0-disabled; 1-enabled
    * @param dexhandEnable Dexterous hand enable status, 0-disabled; 1-enabled
    * @return  Error code
    */
    public int SetAxleLuaEnableDeviceType(int forceSensorEnable, int gripperEnable, int IOEnable, int dexhandEnable)

Get the enabled device types for the end-effector LUA
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Get the enabled device types for the end-effector LUA
     * @param enable enable[0]:forceSensorEnable Force sensor enable status, 0-disabled; 1-enabled
     * @param enable enable[1]:gripperEnable Gripper enable status, 0-disabled; 1-enabled
     * @param enable enable[2]:IOEnable IO device enable status, 0-disabled; 1-enabled
     * @param enable enable[3]:dexhandEnable Dexterous hand enable status, 0-disabled; 1-enabled
     * @return  Error code
     */
    public int GetAxleLuaEnableDeviceType(int[] enable)

Get the currently configured end-effector devices
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Get the currently configured end-effector devices
     * @param forceSensorEnable Force sensor enabled device number, 0-disabled; 1-enabled
     * @param gripperEnable Gripper enabled device number, 0-disabled; 1-enabled
     * @param IODeviceEnable IO device enabled device number, 0-disabled; 1-enabled
     * @param dexhandEnable Dexterous hand enable status, 0-disabled; 1-enabled
     * @return  Error code
     */
    public int GetAxleLuaEnableDevice(int[] forceSensorEnable, int[] gripperEnable, int[] IODeviceEnable, int[] dexhandEnable)


Set the enabled gripper action control functions
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Set the enabled gripper action control functions
     * @param id Gripper device number
     * @param func func[0]-gripper enable; func[1]-gripper initialization; func[2]-position setting; func[3]-speed setting; func[4]-torque setting; func[6]-read gripper status;
        func[7]-read initialization status; func[8]-read fault code; func[9]-read position; func[10]-read speed; func[11]-read torque; func[12]-set rotation count for rotary gripper;
        func[13]-set rotation speed for rotary gripper; func[14]-set rotation torque for rotary gripper; func[15]-read rotary gripper status; func[16]-read rotary gripper initialization status;
        func[17]-read rotary gripper rotation count; func[18]-read rotary gripper speed; func[19]-read rotary gripper torque; func[20]-multi-axis synchronous motion setting; func[21]-fault clear command;
        func[22]-single-axis running status; func[23]-all-axis running status;
     * @return  Error code
     */
    public int SetAxleLuaGripperFunc(int id, int[] func)

Get the enabled gripper action control functions
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Get the enabled gripper action control functions
     * @param id Gripper device number
     * @param func func[0]-gripper enable; func[1]-gripper initialization; func[2]-position setting; func[3]-speed setting; func[4]-torque setting; func[6]-read gripper status;
        func[7]-read initialization status; func[8]-read fault code; func[9]-read position; func[10]-read speed; func[11]-read torque; func[12]-set rotation count for rotary gripper;
        func[13]-set rotation speed for rotary gripper; func[14]-set rotation torque for rotary gripper; func[15]-read rotary gripper status; func[16]-read rotary gripper initialization status;
        func[17]-read rotary gripper rotation count; func[18]-read rotary gripper speed; func[19]-read rotary gripper torque; func[20]-multi-axis synchronous motion setting; func[21]-fault clear command;
        func[22]-single-axis running status; func[23]-all-axis running status;
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

    /*
    * @brief  Write Slave AO
    * @param  AOIndex AO number
    * @param  writeNum Number of values to write
    * @param  status Array of values to write (up to 8 values), AO0~AO15 are integer, AO16~AO31 are floating point
    * @return  Error code
    */
    public int FieldBusSlaveWriteAO(int AOIndex, int writeNum, double[] status)

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

Laser Peripheral On/Off Function
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Laser peripheral on/off function
    * @param [in] OnOff 0-off 1-on
    * @param [in] weldId Weld seam ID, default is 0
    * @return Error code
    */
   public int LaserTrackingLaserOnOff(int OnOff, int weldId)

Laser Tracking Start/Stop Function
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Laser tracking start/stop function
    * @param [in] OnOff 0-stop 1-start
    * @param [in] coordId Laser peripheral tool coordinate system number
    * @return Error code
    */
   public int LaserTrackingTrackOnOff(int OnOff, int coordId)

Laser Positioning - Fixed Direction
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Laser positioning - fixed direction
    * @param [in] direction 0-x+ 1-x- 2-y+ 3-y- 4-z+ 5-z-
    * @param [in] vel Speed in %
    * @param [in] distance Maximum positioning distance in mm
    * @param [in] timeout Positioning timeout in ms
    * @param [in] posSensorNum Laser calibrated tool coordinate number
    * @return Error code
    */
   public int LaserTrackingSearchStart_xyz(int direction, int vel, int distance, int timeout, int posSensorNum)

Laser Positioning - Arbitrary Direction
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Laser positioning - arbitrary direction
    * @param [in] directionPoint XYZ coordinates of the positioning input point
    * @param [in] vel Speed in %
    * @param [in] distance Maximum positioning distance in mm
    * @param [in] timeout Positioning timeout in ms
    * @param [in] posSensorNum Laser calibrated tool coordinate number
    * @return Error code
    */
   public int LaserTrackingSearchStart_point(DescTran directionPoint, int vel, int distance, int timeout, int posSensorNum)

Laser Positioning Stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Laser positioning stop
    * @return Error code
    */
   public int LaserTrackingSearchStop()

Laser IP Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Laser IP configuration
    * @param [in] ip IP address of the laser peripheral
    * @param [in] port Port number of the laser peripheral
    * @return Error code
    */
   public int LaserTrackingSensorConfig(String ip, int port)

Laser Peripheral Sampling Period Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Laser peripheral sampling period configuration
    * @param [in] period Laser peripheral sampling period in ms
    * @return Error code
    */
   public int LaserTrackingSensorSamplePeriod(int period)

Laser Peripheral Driver Loading
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Laser peripheral driver loading
    * @param [in] type Laser peripheral driver protocol type 101-Ruiniu 102-Chuangxiang 103-Quanshi 104-Tongzhou 105-Aotai
    * @return Error code
    */
   public int LoadPosSensorDriver(int type)

Laser Peripheral Driver Unloading
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Laser peripheral driver unloading
    * @return Error code
    */
   public int UnLoadPosSensorDriver()

Laser Weld Seam Trajectory Recording
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Laser weld seam trajectory recording
    * @param [in] status 0-stop recording 1-real-time tracking 2-start recording
    * @param [in] delayTime Delay time in ms
    * @return Error code
    */
   public int LaserSensorRecord1(int status, int delayTime)

Laser Weld Seam Trajectory Replay
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Laser weld seam trajectory replay
    * @param [in] delayTime Delay time in ms
    * @param [in] speed Speed in %
    * @return Error code
    */
   public int LaserSensorReplay(int delayTime, double speed)

Laser Tracking Replay
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Laser tracking replay
    * @return Error code
    */
   public int MoveLTR()

Laser Weld Seam Trajectory Recording and Replay
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

    /**
    * @brief Laser Weld Seam Trajectory Replay
    * @param delayMode Mode 0-Delay Time 1-Delay Distance
    * @param delayTime Delay time in milliseconds (ms)
    * @param delayDisExAxisNum Extended Axis Number
    * @param delayDis Delay distance in millimeters (mm)
    * @param sensitivePara Compensation Sensitivity Coefficient
    * @param trackMode Fixed-point Tracking Type. 0-Extended Axis Asynchronous Motion; 1-Robot
    * @param triggerMode Fixed-point Tracking Trigger Method. 0-Tracking Duration; 1-IO
    * @param runTime Robot Fixed-point Tracking Duration in seconds (s)
    * @param speed Speed in percentage (%)
    * @return Error Code
    */
    public int LaserSensorRecordandReplay(int delayMode, int delayTime, int delayDisExAxisNum, double delayDis, double sensitivePara, int trackMode, int triggerMode, double runTime, double speed)

Move to Laser Record Start Point
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Move to laser record start point
    * @param [in] moveType 0-PTP 1-LIN
    * @param [in] ovl Speed in %
    * @return Error code
    */
   public int MoveToLaserRecordStart(int moveType, double ovl)

Move to Laser Record End Point
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Move to laser record end point
    * @param [in] moveType 0-PTP 1-LIN
    * @param [in] ovl Speed in %
    * @return Error code
    */
   public int MoveToLaserRecordEnd(int moveType, double ovl)

Move to Laser Sensor Positioning Point
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Move to laser sensor positioning point
    * @param [in] moveFlag Motion type: 0-PTP; 1-LIN
    * @param [in] ovl Speed scaling factor, 0-100
    * @param [in] dataFlag Weld seam cache data selection: 0-execute planning data; 1-execute recorded data
    * @param [in] plateType Plate type: 0-corrugated plate; 1-corrugated cardboard; 2-fence plate; 3-oil drum; 4-corrugated shell steel
    * @param [in] trackOffectType Laser sensor offset type: 0-no offset; 1-base coordinate system offset; 2-tool coordinate system offset; 3-laser sensor raw data offset
    * @param [in] offset Offset value
    * @return Error code
    */
   public int MoveToLaserSeamPos(int moveFlag, double ovl, int dataFlag, int plateType, int trackOffectType, DescPose offset)

Get Laser Sensor Positioning Point Coordinate Information
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
   :linenos:

   /**
    * @brief Get laser sensor positioning point coordinate information
    * @param [in] trackOffectType Laser sensor offset type: 0-no offset; 1-base coordinate system offset; 2-tool coordinate system offset; 3-laser sensor raw data offset
    * @param [in] offset Offset value
    * @param [out] jPos Joint position [°]
    * @param [out] descPos Cartesian position [mm]
    * @param [out] tool Tool coordinate system
    * @param [out] user Workpiece coordinate system
    * @param [out] exaxis Extended axis position [mm]
    * @return Error code
    */
   public int GetLaserSeamPos(int trackOffectType, DescPose offset, JointPos jPos, DescPose descPos, int[] tool, int[] user, ExaxisPos exaxis)

Laser Peripheral Sensor Parameter Configuration and Debugging Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: java
   :linenos:

   public static void testLaserConfig(Robot robot)
   {
       robot.LaserTrackingSensorConfig("192.168.58.20", 5020);

       robot.LaserTrackingSensorSamplePeriod(20);

       robot.LoadPosSensorDriver(101);
       robot.LaserTrackingLaserOnOff(0,0);

       robot.Sleep(3000);

       robot.LaserTrackingLaserOnOff(1, 0);

       robot.CloseRPC();
   }

Laser Trajectory Scanning and Trajectory Replay Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: java
   :linenos:

   public static void testLaserRecordAndReplay(Robot robot)
   {
       // Upload and load open protocol file
       robot.OpenLuaUpload("D://zUP/CtrlDev_laser_ruiniu-0117.lua");
       robot.Sleep(2000);
       robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua");
       robot.UnloadCtrlOpenLUA(0);
       robot.LoadCtrlOpenLUA(0);
       robot.Sleep(8000);

       for (int i=0;i<10;++i){
           JointPos startjointPos=new JointPos(56.205, -117.951, 141.872, -118.149, -94.217, -122.176);
           DescPose startdescPose=new DescPose(-97.552, -282.855, 26.675, 174.182, -1.338, -91.707);
           ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
           DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);
           robot.MoveL(startjointPos, startdescPose, 1, 0, 100, 100, 100, -1, 0,exaxisPos, 0, 0, offdese, 0,1, 1);

           robot.LaserSensorRecord1(2, 10);

           JointPos endjointPos=new JointPos(68.809, -87.100, 121.120, -127.233, -95.038, -109.555);
           DescPose enddescPose=new DescPose(-103.555, -464.234, 13.076, 174.179, -1.344, -91.709);
           robot.MoveL(endjointPos, enddescPose, 1, 0, 50, 100, 100, -1,0, exaxisPos, 0, 0, offdese, 0,1, 1);

           robot.LaserSensorRecord1(0, 10);

           robot.MoveToLaserRecordStart(1, 30);

           robot.LaserSensorReplay(10, 100);

           robot.MoveLTR();

           robot.LaserSensorRecord1(0, 10);
       }

       robot.CloseRPC();
   }

Laser Positioning and Real-time Tracking Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: java
   :linenos:

   public static void testLasertrack(Robot robot)
   {
       // Upload and load open protocol file
       robot.OpenLuaUpload("D://zUP/CtrlDev_laser_ruiniu-0117.lua");
       robot.Sleep(2000);
       robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua");
       robot.UnloadCtrlOpenLUA(0);
       robot.LoadCtrlOpenLUA(0);
       robot.Sleep(8000);
       for(int i=0;i<10;++i){
           JointPos startjointPos=new JointPos(56.205, -117.951, 141.872, -118.149, -94.217, -122.176);
           DescPose startdescPose=new DescPose(-97.552, -282.855, 26.675, 174.182, -1.338, -91.707);
           ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
           DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);
           DescTran directionPoint=new DescTran();
           robot.MoveL(startjointPos, startdescPose, 1, 0, 100, 100, 100, -1, 0,exaxisPos, 0, 0, offdese, 0,1, 1);

           robot.LaserTrackingSearchStart_xyz(3, 100, 300, 1000, 3);
           robot.LaserTrackingSearchStop();

           //robot.GetRobotTeachingPoint(name, data);
           robot.MoveToLaserSeamPos(1, 30, 0, 0, 0, offdese);
           //printf("%f, %f, %f,%f, %f, %f,%f, %f, %f,%f, %f, %f\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11]);

           robot.LaserTrackingTrackOnOff(1, 3);
           //robot.LaserTrackingTrackOn(3);
           JointPos endjointPos=new JointPos(68.809,-87.100,121.120,-127.233,-95.038,-109.555);
           DescPose enddescPose=new DescPose(-103.555,-464.234,13.076,174.179,-1.344,-91.709);
           robot.MoveL(endjointPos, enddescPose, 1, 0, 20, 100, 100, -1, 0,exaxisPos, 0, 0, offdese, 0,1, 1);

           robot.LaserTrackingTrackOnOff(0, 3);
           System.out.println("Current iteration: "+(i+1));
       }
       robot.CloseRPC();
   }

Extended Axis and Robot Synchronized Laser Tracking Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: java
   :linenos:

   public static void testLasertrackandExitAxis(Robot robot)
   {
       ExaxisPos startexaxisPos =new ExaxisPos( 0,0,0,0 );
       ExaxisPos seamexaxisPos = new ExaxisPos(-10,0,0,0 );
       ExaxisPos endexaxisPos = new ExaxisPos(-30, 0, 0, 0);
       DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0 );
       JointPos seamjointPos=new JointPos(0, 0, 0, 0, 0, 0);
       DescPose seamdescPose=new DescPose(0, 0, 0, 0, 0, 0);

       for(int i =0;i<10;++i) {
           // Move to the starting point for positioning
           JointPos startjointPos = new JointPos(58.337, -119.628, 146.037, -116.358, -92.224, -117.654);
           DescPose startdescPose = new DescPose(-53.375, -255.363, 0.919, 178.054, 1.077, -94.026);
           robot.ExtAxisSyncMoveJ(startjointPos, startdescPose, 1, 0, 100, 100, 100, startexaxisPos, -1, 0, offdese);

           System.out.println("11111");
           // Start positioning along -y direction
           int ret = robot.LaserTrackingSearchStart_xyz(3, 100, 300, 1000, 2);
           robot.LaserTrackingSearchStop();
           System.out.println("2222");
           int[] tool = new int[1];
           int[] user = new int[1];
           robot.GetLaserSeamPos(0, offdese, seamjointPos, seamdescPose, tool, user, startexaxisPos);
           System.out.println(seamjointPos.J1 + ", " + seamjointPos.J2 + ", " +
                   seamjointPos.J3 + ", " + seamjointPos.J4 + ", " +
                   seamjointPos.J5 + ", " + seamjointPos.J6 + ", " +
                   seamdescPose.tran.x + ", " + seamdescPose.tran.y + ", " +
                   seamdescPose.tran.z + ", " + seamdescPose.rpy.rx + ", " +
                   seamdescPose.rpy.ry + ", " + seamdescPose.rpy.rz);
           // If positioning is successful
           if (ret == 0) {
               // Robot and extended axis synchronously move to the positioning point
               robot.ExtAxisSyncMoveJ(seamjointPos, seamdescPose, 1, 0, 100, 100, 100, seamexaxisPos, -1, 0, offdese);

               // Start laser tracking along the positioning point and synchronously move with extended axis
               System.out.println("3333");
               robot.LaserTrackingTrackOnOff(1, 2);
               JointPos endjointPos = new JointPos(70.580, -90.918, 126.593, -125.154, -92.162, -105.403);
               DescPose enddescPose = new DescPose(-53.375, -419.020, 0.920, 178.054, 1.076, -94.026);
               robot.ExtAxisSyncMoveL(endjointPos, enddescPose, 1, 0, 20, 100, 100, -1, endexaxisPos, 0, offdese);
               ;
               // Stop tracking
               robot.LaserTrackingTrackOnOff(0, 2);
               System.out.println("44444");
           }
           System.out.println("Current run count: "+i);
       }
       robot.CloseRPC();
   }

End-Effector Transparent Transmission Function Enable/Disable SDK Interface
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Enable end-effector general transparent transmission function
    * @param enable, 0-disable, 1-enable
    * @return Error code
    */
    public int SetAxleGenComEnable(int mode)

End-Effector Transparent Transmission Function Non-Periodic Data Transmission and Reception SDK Interface
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief End-effector sends non-periodic data and waits for response
    * @param lenSnd Length of data to send
    * @param sndBuff Data to send
    * @param lenRcv Length of data to receive
    * @param [out] rcvData Response data
    * @return Error code
    */
    public int SndRcvAxleGenComCmdData(int lenSnd, int[] sndBuff, int lenRcv, int[] rcvData)
    
Code Example for Non-Periodic Data Communication of DIO Health Care Moxibustion Head Based on End-Effector Transparent Transmission Function
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void testAxleGenCom(Robot robot) {
    int[] led_on = {0xAB, 0xBA, 0x12, 0x01, 0x01, 0x79};
    int[] led_off = {0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78};
    int[] version = {0xAB, 0xBA, 0x11, 0x00, 0x76};
    int[] state = {0xAB, 0xBA, 0x1B, 0x01, 0xAA, 0x2B};

    int[] rcvdata = new int[16];
    int ret = 0;
    int cnt = 1;

    JointPos p1Joint = new JointPos(88.708, -86.178, 140.989, -141.825, -89.162, -49.879);
    DescPose p1Desc = new DescPose(188.007, -377.850, 260.207, 178.715, 2.823, -131.466);

    JointPos p2Joint = new JointPos(112.131, -75.554, 126.989, -139.027, -88.044, -26.477);
    DescPose p2Desc = new DescPose(368.003, -377.848, 260.211, 178.715, 2.823, -131.465);

    ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
    DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

    // Enable end-effector transparent transmission function
    robot.SetAxleGenComEnable(1);
    robot.SetAxleLuaEnable(1);

    while (cnt <= 10000) {
        // Read version number
        ret = robot.SndRcvAxleGenComCmdData(5, version, 10, rcvdata);
        if (ret == 0) {
            System.out.printf(" hard version : %d,hard code:%d, soft version:%d %d, soft code:%d \n",
                    rcvdata[4], rcvdata[5], rcvdata[6], rcvdata[7], rcvdata[8]);
        } else {
            System.out.println("SndRcvAxleGenComCmdData version fail: " + ret);
            break;
        }
        robot.Sleep(1000);

        // Read moxibustion head presence status
        ret = robot.SndRcvAxleGenComCmdData(6, state, 6, rcvdata);
        if (ret == 0) {
            System.out.printf(" state : %d \n", rcvdata[4]);
        }
        robot.Sleep(1000);

        // Turn on moxibustion head laser
        ret = robot.SndRcvAxleGenComCmdData(6, led_on, 6, rcvdata);
        if (ret == 0) {
            System.out.printf("led on rcv data is: %d, %d, %d, %d, %d, %d\n",
                    rcvdata[0], rcvdata[1], rcvdata[2], rcvdata[3], rcvdata[4], rcvdata[5]);
        }
        robot.MoveJ(p1Joint, p1Desc, 0, 0, 100.0, 100.0, 100.0, exaxisPos, -1.0, 0, offdese);
        robot.Sleep(4000);

        // Turn off moxibustion head laser
        ret = robot.SndRcvAxleGenComCmdData(6, led_off, 6, rcvdata);
        if (ret == 0) {
            System.out.printf("led off rcv data is: %d, %d, %d, %d, %d, %d \n",
                    rcvdata[0], rcvdata[1], rcvdata[2], rcvdata[3], rcvdata[4], rcvdata[5]);
        }
        robot.MoveJ(p2Joint, p2Desc, 0, 0, 100.0, 100.0, 100.0, exaxisPos, -1.0, 0, offdese);
        robot.Sleep(1000);

        System.out.println("***********************complete No. " + cnt + " SDK test*****************************");
        cnt++;
    }
    }  
    
Download Open Protocol Lua File
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Download open protocol Lua file
    * @param fileName Open protocol file name "CtrlDev_XXX.lua"
    * @param savePath Path to save the open protocol file
    * @return Error code
    */
    public int OpenLuaDownload(string fileName, string savePath)

Delete Open Protocol Lua File
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: Java
    :linenos:

    /**
    * @brief Delete open protocol Lua file
    * @param [in] fileName Name of the open protocol Lua file to delete "CtrlDev_XXX.lua"
    * @return Error code
    */
    public int OpenLuaDelete(string fileName)
        
Delete All Open Protocol Lua Files
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: Java
    :linenos:

    /**
    * @brief Delete all open protocol Lua files
    * @return Error code
    */
    public int AllOpenLuaDelete()

Code Example for Controller Peripheral Open Protocol Upload, Download, and Delete
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: Java
    :linenos:

    public static int TestCtrlOpenLuaOperate(Robot robot) {
        int rtn;
        rtn = robot.OpenLuaUpload("D://zUP/openlua/CtrlDev_WELDING_A.lua");
        System.out.println("OpenLuaUpload rtn is " + rtn);
        rtn = robot.OpenLuaUpload("D://zUP/openlua/CtrlDev_SWDPOLISH.lua");
        System.out.println("OpenLuaUpload rtn is " + rtn);
        rtn = robot.OpenLuaDownload("CtrlDev_WELDING_A.lua", "D://zDOWN/");
        System.out.println("OpenLuaDownload rtn is " + rtn);
        rtn = robot.OpenLuaDownload("CtrlDev_SWDPOLISH.lua", "D://zDOWN/");
        System.out.println("OpenLuaDownload rtn is " + rtn);

        rtn = robot.SetCtrlOpenLUAName(0, "CtrlDev_WELDING_A.lua");
        System.out.println("SetCtrlOpenLUAName rtn is " + rtn);
        rtn = robot.SetCtrlOpenLUAName(1, "CtrlDev_SWDPOLISH.lua");
        System.out.println("SetCtrlOpenLUAName rtn is " + rtn);

        String[] names = new String[4];
        rtn = robot.GetCtrlOpenLUAName(names);
        System.out.println("GetCtrlOpenLUAName rtn is " + rtn + ", names: " +
                names[0] + ", " + names[1] + ", " + names[2] + ", " + names[3]);

        rtn = robot.LoadCtrlOpenLUA(1);
        System.out.println("LoadCtrlOpenLUA rtn is " + rtn);
        robot.Sleep(2000);
        rtn = robot.UnloadCtrlOpenLUA(1);
        System.out.println("UnloadCtrlOpenLUA rtn is " + rtn);

        rtn = robot.OpenLuaDelete("CtrlDev_WELDING_A.lua");
        System.out.println("OpenLuaDelete rtn is " + rtn);
        rtn = robot.AllOpenLuaDelete();
        System.out.println("AllOpenLuaDelete rtn is " + rtn);

        robot.Sleep(1000);
        return 0;
    }

Control Dexterous Hand Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: Java
    :linenos:

    /**
    * @brief Control dexterous hand motion
    * @param idstart Starting slave station number
    * @param slaveNum Number of slaves
    * @param pos Position array, length 16, range (-360~360)
    * @param speed Speed percentage array, length 16, range [0~100]
    * @param force Torque percentage array, length 16, range [0~100]
    * @param max_time Maximum wait time, range [0~30000], unit ms
    * @return Error code
    */
    public int SetDexterousHandsMove(int idstart, int slaveNum, double[] pos, int[] speed, int[] force, int max_time) 
        
Control Dexterous Hand Reset and Activation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: Java
    :linenos:

    /**
    * @brief Control dexterous hand reset and activation
    * @param id Slave station number
    * @param act 0-reset 1-activate
    * @return Error code
    */
    public int SetDexterousHandsAct(int id, int act)
            
Clear Dexterous Hand Error
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: Java
    :linenos:

    /**
    * @brief Clear dexterous hand error
    * @return Error code
    */
    public int ClearDexterousHandsError()
                
Set Enabled Dexterous Hand Action Control Functions
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: Java
    :linenos:

    /**
    * @brief Set enabled dexterous hand action control functions
    * @param id Dexterous hand slave station number
    * @param func Function array, length 32, Bit0-grip trigger, Bit1-gripper initialization, Bit2-position setting, Bit3-speed setting, Bit4-torque setting, Bit6-read gripper status, Bit7-read initialization status, Bit8-read fault code, Bit9-read position, Bit10-read speed, Bit11-read torque, Bit12-rotation count setting, Bit13-rotation speed setting, Bit14-rotation torque setting, Bit15-read rotary gripper status, Bit16-read rotary initialization status, Bit17-read rotation count, Bit18-read rotation speed, Bit19-read rotation torque, Bit20-multi-axis synchronous motion setting, Bit21-fault clear command, Bit22-single-axis running status, Bit23-all-axis running status
    * @return Error code
    */
    public int SetDexterousHandsFunc(int id, int[] func)
                    
Get Enabled Dexterous Hand Action Control Functions
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: Java
    :linenos:

    /**
    * @brief Get enabled dexterous hand action control functions
    * @param id Dexterous hand device number
    * @param func Output parameter array, length 32, Bit0-grip trigger, Bit1-gripper initialization, Bit2-position setting, Bit3-speed setting, Bit4-torque setting, Bit6-read gripper status, Bit7-read initialization status, Bit8-read fault code, Bit9-read position, Bit10-read speed, Bit11-read torque, Bit12-rotation count setting, Bit13-rotation speed setting, Bit14-rotation torque setting, Bit15-read rotary gripper status, Bit16-read rotary initialization status, Bit17-read rotation count, Bit18-read rotation speed, Bit19-read rotation torque, Bit20-multi-axis synchronous motion setting, Bit21-fault clear command, Bit22-single-axis running status, Bit23-all-axis running status
    * @return Error code
    */
    public int GetDexterousHandsFunc(int id, int[] func)
                    
End-Effector Dexterous Hand Configuration and Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: Java
    :linenos:

    public static int TestDexterousHands(Robot robot) {
        int id = 1;
        int slaveNum = 4;
        int max_time = 8000;
        int[] speed = new int[16]; 
        int[] force = new int[16]; 

        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        for (int i = 0; i < 16; i++) {
            force[i] = (i < 4) ? 50 : 0;
        }

        final double[] pos = new double[16];

        JointPos j1 = new JointPos(-91.876, -85.920, 109.279, -86.239, -96.664, -28.563);
        JointPos j2 = new JointPos(-40.954, -85.920, 109.279, -86.239, -96.664, -28.563);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);

        int ret = robot.ClearDexterousHandsError();
        System.out.println("ClearDexterousHandsError -> " + ret);

        int[] setFunc = new int[32];
        setFunc[2] = 1;
        setFunc[4] = 1;
        setFunc[9] = 1;
        setFunc[10] = 1;
        setFunc[11] = 1;
        setFunc[22] = 1;

        ret = robot.SetDexterousHandsFunc(id, setFunc);

        int[] getFunc = new int[32];
        ret = robot.GetDexterousHandsFunc(id, getFunc);
        System.out.println("GetDexterousHandsFunc -> " + ret);
        if (ret == 0) {
            for (int i = 0; i < getFunc.length; i++) {
                System.out.print("  [" + i + "]=" + getFunc[i]);
                if ((i + 1) % 8 == 0) {
                    System.out.println();
                } else if (i < getFunc.length - 1) {
                    System.out.print(", ");
                }
            }
            if (getFunc.length % 8 != 0) {
                System.out.println();
            }
        }

        ret = robot.SetDexterousHandsAct(id, 1);
        if (ret != 0) {
            return ret;
        }

        setPositions(pos, 20, 20, 20, 20);
        ret = robot.SetDexterousHandsMove(id, slaveNum, pos, speed, force, max_time);
        robot.Sleep(5000);
        
        for (int iteration = 1; iteration <= 10; iteration++) {
            robot.MoveJ(j1, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);

            setPositions(pos, 10, 10, 10, 10);
            ret = robot.SetDexterousHandsMove(id, slaveNum, pos, speed, force, max_time);
            robot.Sleep(1000);

            robot.MoveJ(j2, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
            setPositions(pos, 50, 50, 50, 50);
            ret = robot.SetDexterousHandsMove(id, slaveNum, pos, speed, force, max_time);
            robot.Sleep(1000);
        }
        return 0;
    }    