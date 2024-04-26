Others
=================

.. toctree:: 
    :maxdepth: 5

The conveyor starts and stops
++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief The conveyor starts and stops
    * @param [in] status 1-starts ，0-stops
    * @return Error code
    */ 
    int ConveyorStartEnd(byte status); 

Record the IO detection point
++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Record the IO detection point
    * @return Error code 
    */ 
    int ConveyorPointIORecord();

Record the A point
++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Record the A point 
    * @return Error code 
    */ 
    int ConveyorPointARecord(); 

Record reference point
++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Record reference point 
    * @return Error code 
    */ 
    int ConveyorRefPointRecord(); 

Record the B point
++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Record the B point
    * @return Error code
    */ 
    int ConveyorPointBRecord(); 

Conveyor belt workpiece IO detection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Conveyor belt workpiece IO detection 
    * @param [in] max_t Maximum detection time (ms)
    * @return Error code 
    */ 
    int ConveyorIODetect(int max_t); 

Gets the current position of the workpiece
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Gets the current position of the workpiece 
    * @param [in] mode 1- tracking grab, 2- tracking motion, 3-TPD tracking
    * @return Error code 
    */ 
    int ConveyorGetTrackData(int mode);

Conveyor tracking start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Conveyor tracking start 
    * @param [in] status 1-start，0-stop
    * @return Error code 
    */
    int ConveyorTrackStart(byte status);

Conveyor tracking stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Conveyor tracking stop 
    * @return Error code 
    */
    int ConveyorTrackEnd();

Conveyor parameter configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Conveyor parameter configuration 
    * @param [in] encChannel Encoder channel 1~2
    * @param [in] resolution The number of pulses per turn of the encoder
    * @param [in] lead Encoder on one turn of the conveyor belt travel distance
    * @param [in] wpAxis Workpiece coordinate system number Select the workpiece coordinate system number for the tracking motion function, and set the tracking capture and TPD tracking to 0
    * @param [in] vision Does it match vision? 0- No, 1- Yes
    * @param [in] speedRadio for conveyor tracking Grab options (1-100) Other options default to 1
    * @return Error code 
    */
    int ConveyorSetParam(int encChannel, int resolution, double lead, int wpAxis, int vision, double speedRadio);

Set the conveyor grab point compensation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set the conveyor grab point compensation 
    * @param [in] cmp point compensation double[3]{x, y, z}
    * @return Error code 
    */
    int ConveyorCatchPointComp(double[] cmp);

The conveyor belt tracks linear motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief The conveyor belt tracks linear motion 
    * @param [in] name Motion point name
    * @param [in] tool Tool coordinate number, range[0~14] 
    * @param [in] wobj Workpiece coordinate number, range[0~14] 
    * @param [in] vel Percentage of speed, range[0~100] 
    * @param [in] acc Acceleration percentage, range[0~100],not open for now 
    * @param [in] ovl Velocity scaling factor, range[0~100] 
    * @param [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm  
    * @return Error code  
    */
    int ConveyorTrackMoveL(string name, int tool, int wobj, float vel, float acc, float ovl, float blendR);

Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnConvert_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");
        DescPose pos1 = new DescPose(0, 0, 0, 0 ,0 ,0);
        DescPose pos2 = new DescPose(0, 0, 0, 0, 0, 0);

        pos1.tran.x = -351.175;
        pos1.tran.y = 3.389;
        pos1.tran.z = 431.172;
        pos1.rpy.rx = -179.111;
        pos1.rpy.ry = -0.241;
        pos1.rpy.rz = 90.388;

        pos2.tran.x = -333.654;
        pos2.tran.y = -229.003;
        pos2.tran.z = 404.335;
        pos2.rpy.rx = -179.139;
        pos2.rpy.ry = -0.779;
        pos2.rpy.rz = 91.269;
        int rtn = -1;

        double[] cmp = new double[3] { 0, 9.99, 0};
        rtn = robot.ConveyorCatchPointComp(cmp);
        if(rtn != 0)
        {
            return;
        }
        Console.WriteLine($"ConveyorCatchPointComp: rtn  {rtn}");

        rtn = robot.MoveCart(pos1, 0, 0, 100.0f, 180.0f, 100.0f, -1.0f, -1);
        Console.WriteLine($"MoveCart: rtn  {rtn}");

        rtn = robot.ConveyorIODetect(10000);
        Console.WriteLine($"ConveyorIODetect: rtn  {rtn}");

        robot.ConveyorGetTrackData(1);
        rtn = robot.ConveyorTrackStart(1);
        Console.WriteLine($"ConveyorTrackStart: rtn  {rtn}");

        rtn = robot.ConveyorTrackMoveL("cvrCatchPoint", 0, 0, 100.0f, 0.0f, 100.0f, -1.0f, 0, 0);
        Console.WriteLine($"ConveyorTrackMoveL: rtn  {rtn}");

        rtn = robot.MoveGripper(1, 59, 43, 21, 30000, 0);
        Console.WriteLine($"MoveGripper: rtn  {rtn}");

        rtn = robot.ConveyorTrackMoveL("cvrRaisePoint", 0, 0, 100.0f, 0.0f, 100.0f, -1.0f, 0, 0);
        Console.WriteLine($"ConveyorTrackMoveL: rtn  {rtn}");

        rtn = robot.ConveyorTrackEnd();
        Console.WriteLine($"ConveyorTrackEnd: rtn  {rtn}");

        rtn = robot.MoveCart(pos2, 0, 0, 100.0f, 180.0f, 100.0f, -1.0f, -1);
        Console.WriteLine($"MoveCart: rtn  {rtn}");

        rtn = robot.MoveGripper(1, 100, 43, 21, 30000, 0);
        Console.WriteLine($"MoveGripper: rtn  {rtn}");
    }

Obtaining an SSH Key
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Obtaining an SSH Key 
    * @param [out] keygen key
    * @return Error code 
    */
    int GetSSHKeygen(ref string keygen);

Calculates the MD5 value of the file in the specified path
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Calculates the MD5 value of the file in the specified path 
    * @param [in] file_path For example,"/fruser/traj/trajHelix_aima_1.txt"
    * @param [out] md5 MD5 value
    * @return Error code
    */
    int ComputeFileMD5(string file_path, ref string md5);

Get robot emergency stop state
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get robot emergency stop state 
    * @param [out] state emergency stop state，0-not emergency stop，1-emergency stop
    * @return Error code 
    */
    int GetRobotEmergencyStopState(ref byte state);

Get the communication status between SDK and robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the communication status between SDK and robot 
    * @param [out] state Communication status, 0- normal, 1-abnormal
    * @return Error code 
    */
    int GetSDKComState(ref int state)

Obtain a safe stop signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Obtain a safe stop signal 
    * @param [out] si0_state signal SI0，0- invalid, 1- valid
    * @param [out] si1_state signal SI1，0- invalid, 1- valid
    * @return Error code 
    */
    int GetSafetyStopState(ref byte si0_state, ref byte si1_state)

Obtain the compensation value of robot DH parameter
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Obtain the compensation value of robot DH parameter 
    * @param [out] dhCompensation The compensation value of robot DH parameter(mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]
    * @return Error code 
    */
    int GetDHCompensation(ref double[] dhCompensation)

Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnTestOthers_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");
        int rtn = -1;
        double[] dhCompensation = new double[6]{0,0,0,0,0,0};
        rtn = robot.GetDHCompensation(ref dhCompensation);
        Console.WriteLine($"GetDHCompensation:  rtn :{rtn}    {dhCompensation[0]}  {dhCompensation[1]}  {dhCompensation[2]}  {dhCompensation[3]}  {dhCompensation[4]}  {dhCompensation[5]}");
        string ssh = "";
        rtn = robot.GetSSHKeygen(ref ssh);
        Console.WriteLine($"GetSSHKeygen:  ssh {ssh}  rtn  {rtn}");
        string file_path = "/fruser/test.txt";
        string md5 = "";
        robot.ComputeFileMD5(file_path, ref md5);

        byte state = 255;
        rtn = robot.GetRobotEmergencyStopState(ref state);
        Console.WriteLine($"GetRobotEmergencyStopState:  rtn  {rtn}   state {state}");

        int comState = -1;
        rtn = robot.GetSDKComState(ref comState);
        Console.WriteLine($"GetSDKComState:  rtn  {rtn}   state  {comState}");

        byte si0_state = 255;
        byte si1_state = 255;

        rtn = robot.GetSafetyStopState(ref si0_state, ref si1_state);
        Console.WriteLine($"GetSafetyStopState:  rtn  {rtn}   si0_state  {si0_state}   si1_state  {si1_state}");
    }

UpLoad Point Table
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Upload the point table from the local computer to the robot controller
    * @param [in] pointTableFilePath The absolute path of the point table on the local computer is C://test/pointTabl e1.db
    * @return Error code 
    */
    int PointTableUpLoad(string pointTableFilePath)

DownLoad Point Table
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Download the point table from the robot controller to the local computer 
    * @param [in] pointTableName point table name：pointTable1.db
    * @param [in] saveFilePath save path： C://test/
    * @return Error code 
    */
    int PointTableDownLoad(string pointTableName, string saveFilePath);

Point table update Lua program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Point table update Lua program
    * @param [in] pointTableName point table name："pointTable1.db", When the point table is empty, that is, an empty string, the lua program is updated to the original program that did not apply the point table
    * @param [in] luaFileName Lua program name   "test.lua"
    * @param [out] errorStr error string  
    * @return Error code 
    */
    int PointTableUpdateLua(string pointTableName, string luaFileName, ref string errorStr);

Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnUpload_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");
        int rtn = -1;
        rtn = robot.PointTableUpLoad("C://point_table_test.db");
        Thread.Sleep(2000);
        rtn = robot.PointTableDownLoad("point_table_test.db", "D://zDOWN/");
        string errorStr = "";
        rtn = robot.PointTableUpdateLua("point_table_test.db", "test.lua", ref errorStr);
        Console.WriteLine($"PointTableSwitch rtn  is {rtn}" + errorStr);
        rtn = robot.ProgramLoad("/fruser/test.lua");
        rtn = robot.ProgramRun();
    }

Initialize log parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Initialize log parameters
    * @param [in] logType：Output mode，DIRECT-direct output；BUFFER-buffered output；ASYNC-asynchronous output
    * @param [in] logLevel：Filter the log level value，ERROR、WARNING、INFO and DEBUG
    * @param [in] filePath: File save path，“D://Log/”
    * @param [in] saveFileNum：Number of saved files,The files that exceed the number of files to be saved and the number of days to be saved will be deleted
    * @param [in] saveDays: Number of days to save the file,The files that exceed the number of files to be saved and the number of days to be saved will be deleted
    * @return Error Code
    */
    public int LoggerInit(FrLogType logType = FrLogType.DIRECT, FrLogLevel logLevel = FrLogLevel.ERROR, string filePath = "", int saveFileNum = 10, int saveDays = 10);

Set the log filtering level
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Set the log filtering level;
    * @param [in] logLevel: Filter the log level value，ERROR、WARNING、INFO and DEBUG
    * @return Error code
    */
    public int SetLoggerLevel(FrLogLevel logLevel);


Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.5

.. code-block:: c#
    :linenos:

    private void btnTestLog_Click(object sender, EventArgs e)
    {
        robot = new Robot();
        robot.RPC("192.168.58.2"); 
        string path = "D://log/";
        robot.LoggerInit(FrLogType.ASYNC, FrLogLevel.DEBUG, path, 5, 5);
        robot.SetLoggerLevel(FrLogLevel.INFO);
    }

Set the robot peripheral protocol
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set the robot peripheral protocol
    * @param [in] protocol Robot peripheral protocol number 4096-Extended axis control card; 4097-ModbusSlave; 4098-ModbusMaster
    * @return error code
    */
    int SetExDevProtocol(int protocol);

Get the robot peripheral protocol
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the robot peripheral protocol
    * @param [out] protocol Robot peripheral protocol number 4096-Extended axis control card; 4097-ModbusSlave; 4098-ModbusMaster
    * @return error code
    */
    int GetExDevProtocol(ref int protocol);

Code Example
++++++++++++++

.. versionadded:: C# SDK-v1.0.6

.. code-block:: c#
    :linenos:

    private void btnSetProto_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        int protocol = 4098;//ModbusMaster 
        robot.SetExDevProtocol(protocol);

        robot.GetExDevProtocol(ref protocol);
    }