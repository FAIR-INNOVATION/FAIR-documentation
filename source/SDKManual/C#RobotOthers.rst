Other interfaces
================================

.. toctree:: 
    :maxdepth: 5

Belt start and stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Drive belt start, stop 
    * @param [in] status status, 1-start, 0-stop
    * @return error code 
    */ 
    int ConveyorStartEnd(byte status). 

Record IO detection points
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Record IO detection points 
    * @return error code 
    */ 
    int ConveyorPointIORecord(). 

Record point A
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Record point A 
    * @return error code 
    */ 
    int ConveyorPointARecord().

Recording reference points
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Record reference points 
    * @return error code 
    */ 
    int ConveyorRefPointRecord(). 

Record point B
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Record point B 
    * @return error code 
    */ 
    int ConveyorPointBRecord().

Conveyor workpiece IO inspection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Conveyorised Workpiece IO Inspection 
    * @param [in] max_t Maximum detection time in ms
    * @return error code 
    */ 
    int ConveyorIODetect(int max_t).

Get the current position of the object
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the current position of the object 
    * @param [in] mode 1-tracking grab, 2-tracking motion, 3-TPD tracking
    * @return error code 
    */ 
    int ConveyorGetTrackData(int mode).

Drive belt tracking started
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Drivetrain tracking begins * 
    * @param [in] status status, 1-start, 0-stop
    * @return error code 
    */
    int ConveyorTrackStart(byte status).

Belt tracking stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Drive belt tracking stop 
    * @return error code 
    */
    int ConveyorTrackEnd().

Drive Belt Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Drive Belt Parameter Configuration 
    * @param [in] encChannel encoder channel 1~2
    * @param [in] resolution Number of pulses for one revolution of the encoder
    * @param [in] lead Distance travelled by the encoder for one conveyor revolution
    * @param [in] wpAxis Workpiece coordinate system number Select the workpiece coordinate system number for the tracking motion function, and set the tracking gripping and TPD tracking to 0.
    * @param [in] vision whether or not to match vision 0-no, 1-match
    * @param [in] speedRadio speedRatio: for conveyor belt tracking capture option (1-100) other options default to 1
    * @return error code 
    */
    int ConveyorSetParam(int encChannel, int resolution, double lead, int wpAxis, int vision, double speedRadio);

Setting the drive belt gripping point compensation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting drive belt grab point compensation 
    * @param [in] cmp compensation position double[3]{x, y, z}
    * @return error code 
    */
    int ConveyorCatchPointComp(double[] cmp).

Conveyor tracking linear motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Conveyor tracking linear motion 
    * @param [in] name Movement point name
    * @param [in] tool tool coordinate number in the range [0~14]. 
    * @param [in] wobj Workpiece coordinate number in the range [0~14]. 
    * @param [in] vel velocity percentage, range [0~100] 
    * @param [in] acc Acceleration percentage, range [0~100], not open yet. 
    * @param [in] ovl velocity scaling factor, range [0~100] 
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm  
    * @return error code 
    */
    int ConveyorTrackMoveL(string name, int tool, int wobj, float vel, float acc, float ovl, float blendR);

code example
+++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnConvert_Click(object sender, EventArgs e)
        {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");
        DescPose pos1 = new DescPose(0, 0, 0, 0, 0 , 0 ,0 ,0);
        DescPose pos2 = new DescPose(0, 0, 0, 0, 0, 0, 0);

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
        if(rtn ! = 0)
        {
            return;
        }
        Console.WriteLine($"ConveyorCatchPointComp: rtn {rtn}");

        rtn = robot.MoveCart(pos1, 0, 0, 100.0f, 180.0f, 100.0f, -1.0f, -1);
        Console.WriteLine($"MoveCart: rtn {rtn}");

        rtn = robot.ConveyorIODetect(10000);
        Console.WriteLine($"ConveyorIODetect: rtn {rtn}");

        robot.ConveyorGetTrackData(1);
        rtn = robot.ConveyorTrackStart(1);
        Console.WriteLine($"ConveyorTrackStart: rtn {rtn}");

        rtn = robot.ConveyorTrackMoveL("cvrCatchPoint", 0, 0, 100.0f, 0.0f, 100.0f, -1.0f, 0, 0);
        Console.WriteLine($"ConveyorTrackMoveL: rtn {rtn}");

        rtn = robot.MoveGripper(1, 59, 43, 21, 30000, 0);
        Console.WriteLine($"MoveGripper: rtn {rtn}");

        rtn = robot.ConveyorTrackMoveL("cvrRaisePoint", 0, 0, 100.0f, 0.0f, 100.0f, -1.0f, 0, 0);
        Console.WriteLine($"ConveyorTrackMoveL: rtn {rtn}");

        rtn = robot.ConveyorTrackEnd();
        Console.WriteLine($"ConveyorTrackEnd: rtn {rtn}");

        rtn = robot.MoveCart(pos2, 0, 0, 100.0f, 180.0f, 100.0f, -1.0f, -1);
        Console.WriteLine($"MoveCart: rtn {rtn}");

        rtn = robot.MoveGripper(1, 100, 43, 21, 30000, 0);
        Console.WriteLine($"MoveGripper: rtn {rtn}");
    }

Get SSH public key
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Getting SSH public key 
    * @param [out] keygen public key
    * @return error code 
    */
    int GetSSHKeygen(ref string keygen).

Calculate the MD5 value of a file in a specified path
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Calculates the MD5 value of a file in a specified path. 
    * @param [in] file_path file path contains the file name, the default Traj folder path is: "/fruser/traj/", such as "/fruser/traj/trajHelix_aima_1.txt".
    * @param [out] md5 file MD5 value
    * @return error code 
    */
    int ComputeFileMD5(string file_path, ref string md5);

Getting robot emergency stop status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the robot emergency stop status 
    * @param [out] state Emergency stop state, 0-non-emergency stop, 1-emergency stop
    * @return error code 
    */
    int GetRobotEmergencyStopState(ref byte state).

Get the communication status of the SDK with the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the status of the SDK's communication with the robot. 
    * @param [out] state communication state, 0-normal communication, 1-abnormal communication
    * @return error code 
    */
    int GetSDKComState(ref int state)

Obtaining a safety stop signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Getting a safety stop signal 
    * @param [out] si0_state safety stop signal SI0, 0-invalid, 1-valid
    * @param [out] si1_state safety stop signal SI1, 0-invalid, 1-valid
    * @return error code 
    */
    int GetSafetyStopState(ref byte si0_state, ref byte si1_state)

Getting the robot DH parameter compensation value
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Getting a safety stop signal 
    * @param [out] dhCompensation Robot DH parameter compensation value (mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]
    * @return error code 
    */
    int GetDHCompensation(ref double[] dhCompensation)

code example
+++++++++++++
.. code-block:: c#
    :linenos:

    private void btnTestOthers_Click(object sender, EventArgs e)
        {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");
        int rtn = -1;
        double[] dhCompensation = new double[6]{0,0,0,0,0,0,0};
        rtn = robot.GetDHCompensation(ref dhCompensation);
        Console.WriteLine($"GetDHCompensation: rtn :{rtn} {dhCompensation[0]} {dhCompensation[1]} {dhCompensation[2]} {dhCompensation[3]} { dhCompensation[4]} {dhCompensation[5]}");
        string ssh = "";
        rtn = robot.GetSSHKeygen(ref ssh);
        Console.WriteLine($"GetSSHKeygen: ssh {ssh} rtn {rtn}");
        string file_path = "/fruser/test.txt";
        string md5 = "";
        robot.ComputeFileMD5(file_path, ref md5);

        byte state = 255;
        rtn = robot.GetRobotEmergencyStopState(ref state);
        Console.WriteLine($"GetRobotEmergencyStopState: rtn {rtn} state {state}");

        int comState = -1;
        rtn = robot.GetSDKComState(ref comState);
        Console.WriteLine($"GetSDKComState: rtn {rtn} state {comState}");

        byte si0_state = 255.
        byte si1_state = 255;

        rtn = robot.GetSafetyStopState(ref si0_state, ref si1_state);
        Console.WriteLine($"GetSafetyStopState: rtn {rtn} si0_state {si0_state} si1_state {si1_state}");
    }

Upload Points List
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Point table uploaded from local computer to robot controller 
    * @param [in] pointTableFilePath The absolute path to the point table on the local computer C:///test/pointTabl e1.db
    * @return error code 
    */
    int PointTableUpLoad(string pointTableFilePath);;

Download Points List
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Point table downloaded from robot controller to local computer 
    * @param [in] pointTableName Name of the point table in the controller: pointTable1.db
    * @param [in] saveFilePath Path where the point list is downloaded to the computer C://test/
    * @return error code 
    */
    int PointTableDownLoad(string pointTableName, string saveFilePath);;

Points Table Update Lua Program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Updates points in a lua program with a given table of points.
    * @param [in] pointTableName Name of the point table in the controller: "pointTable1.db", when the point table is empty, i.e., "", it means that the lua program will be updated to the initial program without applying the point table.
    * @param [in] luaFileName Name of the lua file to be updated "test.lua"
    * @param [out] errorStr point table update lua error message  
    * @return error code 
    */
    int PointTableUpdateLua(string pointTableName, string luaFileName, ref string errorStr);;

code example
++++++++++++++++++++++++++++++++++++
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
        Console.WriteLine($"PointTableSwitch rtn is {rtn}" + errorStr);
        rtn = robot.ProgramLoad("/fruser/test.lua");
        rtn = robot.ProgramRun();
    }

Initialising logging parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Initialise log parameters
    * @param [in] logType: output mode, DIRECT-direct output; BUFFER-buffered output; ASYNC-asynchronous output
    * @param [in] logLevel: log filter level, ERROR-Error; WARNING-Warning; INFO-Information; DEBUG-Debugging
    * @param [in] filePath: file save path, e.g. "D:///Log/".
    * @param [in] saveFileNum: the number of files to be saved; files that exceed both the number of files to be saved and the number of days to be saved will be deleted.
    * @param [in] saveDays: the number of days to save a file, files that exceed both the number of files to save and the number of days to save a file will be deleted
    * @return error code
    */
    public int LoggerInit(FrLogType logType = FrLogType.DIRECT, FrLogLevel logLevel = FrLogLevel.ERROR, string filePath = "", int saveFileNum = 10, int saveDays = 10);

Setting the log filter level
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:

    /**
    * :: @brief sets the level of log filtering.
    * @param [in] logLevel: log filter level, ERROR-Error; WARNING-Warning; INFO-Information; DEBUG-Debugging
    * @return error code
    */
    public int SetLoggerLevel(FrLogLevel logLevel);


code example
++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:

    private void btnTestLog_Click(object sender, EventArgs e)
    {
        robot = new Robot();//instantiate the robot object
        robot.RPC("192.168.58.2"); //establish connection with control box
        string path = "D://log/";
        robot.LoggerInit(FrLogType.ASYNC, FrLogLevel.DEBUG, path, 5, 5);
        robot.SetLoggerLevel(FrLogLevel.INFO);
    }

Setting up robot peripheral protocols
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting up robot peripheral protocols
    * @param [in] protocol Robot peripheral protocol number 4096-Extended Axis Control Card; 4097-ModbusSlave; 4098-ModbusMaster
    * @return error code 
    */
    int SetExDevProtocol(int protocol).

Obtaining Robot Peripheral Protocols
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Get robot peripheral protocols
    * @param [out] protocol Robot peripheral protocol number 4096-Extended Axis Control Card; 4097-ModbusSlave; 4098-ModbusMaster
    * @return error code 
    */
    int GetExDevProtocol(ref int protocol).

code example
+++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    private void btnSetProto_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        int protocol = 4098;//ModbusMaster 
        robot.SetExDevProtocol(protocol);

        robot.GetExDevProtocol(ref protocol);
        Console.Writeline("protocol is" + protocol);
    }