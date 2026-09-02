Robot Basics
===================================================

.. toctree:: 
    :maxdepth: 5

Instantiating the Robot
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Robot interface class constructor
    */
    Robot(); 

Establishing Communication with the Controller
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Establish communication with the robot controller
    * @param  [in] ip  Controller IP address, default is 192.168.58.2
    * @return Error code
    */
    int RPC(string ip);

Disconnect from the robot
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Disconnect from the robot controller 
    * @return Error code 
    */ 
    int CloseRPC(); 

Query the SDK version number
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Query the SDK version number 
    * @param [out] version SDK version number 
    * @return Error code 
    */  
    int GetSDKVersion(ref string version);

Get the controller IP address
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the controller IP
    * @param  [out] ip  Controller IP
    * @return  Error code
    */
    int GetControllerIP(ref string ip);

Control the robot to enter or exit drag teaching mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Control the robot to enter or exit drag teaching mode
    * @param  [in] state 0-exit drag teaching mode, 1-enter drag teaching mode
    * @return  Error code
    */
    int DragTeachSwitch(byte state);

Check if the robot is in drag mode
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Check if the robot is in drag teach mode
    * @param  [out] state 0-not in drag teach mode, 1-in drag teach mode
    * @return Error code
    */
    int IsInDragTeach(ref byte state);

Control robot enable or disable
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Control robot enable or disable; robot is automatically enabled by default after power-on
    * @param  [in] state  0-disable, 1-enable
    * @return  Error code
    */
    int RobotEnable(byte state); 

Control robot manual/automatic mode switching
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Control robot manual/automatic mode switching
    * @param [in] mode 0-automatic mode, 1-manual mode
    * @return Error code
    */
    int Mode(int mode);

Shut down the robot operating system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Shut down the robot operating system
    * @return Error code
    */
    int ShutDownRobotOS();

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button6_Click(object sender, EventArgse)
    {   
        int rtn = robot.ShutDownRobotOS();
        Console.WriteLine($"ShutDownRobotOS rtn is {rtn}");
    }

Set robot communication reconnection parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set robot communication reconnection parameters
    * @param [in] enable Whether to enable true-enable, false-disable
    * @param [in] times Reconnect times
    * @param [in] period Reconnect time interval (milliseconds)
    */
    void SetReconnectParam(bool enable, int times, int period);

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnStandard_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true, 100, 20000);//Disconnection reconnection parameters
        robot.RPC("192.168.58.2"); 

        string ip = "";
        string version = "";
        byte state = 0;

        robot.GetSDKVersion(ref version);
        Console.WriteLine($"SDK version : {version}");
        robot.GetControllerIP(ref ip);
        Console.WriteLine($"controller ip : {ip}");

        robot.Mode(1);
        Thread.Sleep(1000);
        robot.DragTeachSwitch(1);
        int rtn = robot.IsInDragTeach(ref state);
        Console.WriteLine($"drag state : {state}");
        Thread.Sleep(3000);
        robot.DragTeachSwitch(0);
        Thread.Sleep(1000);
        robot.IsInDragTeach(ref state);
        Console.WriteLine($"drag state: {state}");
        Thread.Sleep(3000);
        robot.RobotEnable(0);
        Thread.Sleep(3000);
        robot.RobotEnable(1);

        robot.Mode(0);
        Thread.Sleep(1000);
        robot.Mode(1);
        
        rtn = robot.HiSpeedManualSwitch(1);
        Console.WriteLine($"change high speed mode {rtn}");
        Thread.Sleep(10000);

        rtn = robot.HiSpeedManualSwitch(0);
        Console.WriteLine($"change low speed mode {rtn}");
        Thread.Sleep(1000);
    }

Initialize log parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Initialize log parameters
    * @param [in] logType: Output mode, DIRECT-direct output; BUFFER-buffered output; ASYNC-asynchronous output
    * @param [in] logLevel: Log filtering level, ERROR-error; WARNING-warning; INFO-information; DEBUG-debug
    * @param [in] filePath: File save path, e.g., "D://Log/"
    * @param [in] saveFileNum: Number of files to save; files exceeding both the number of files to save and the number of days to save will be deleted
    * @param [in] saveDays: Number of days to save; files exceeding both the number of files to save and the number of days to save will be deleted
    * @return Error code
    */
    int LoggerInit(FrLogType logType = FrLogType.DIRECT, FrLogLevel logLevel = FrLogLevel.INFO, string filePath = "", int saveFileNum = 10, int saveDays = 10);

Set log filtering level
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set log filtering level
    * @param [in] logLevel: Log filtering level, ERROR-error; WARNING-warning; INFO-information; DEBUG-debug
    * @return Error code
    */
    int SetLoggerLevel(FrLogLevel logLevel);

Get robot software version
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get robot software version information
    * @param [out] robotModel Robot model
    * @param [out] webVersion Web version
    * @param [out] controllerVersion Controller version
    * @return Error code 
    */ 
    int GetSoftwareVersion(ref string robotModel, ref string webVersion, ref string controllerVersion);

Get robot hardware version
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get robot hardware version information
    * @param [out] ctrlBoxBoardVersion Control box board hardware version
    * @param [out] driver1Version Driver 1 hardware version
    * @param [out] driver1Version Driver 2 hardware version
    * @param [out] driver1Version Driver 3 hardware version
    * @param [out] driver1Version Driver 4 hardware version
    * @param [out] driver1Version Driver 5 hardware version
    * @param [out] driver1Version Driver 6 hardware version
    * @param [out] endBoardVersion End board hardware version
    * @return Error code 
    */ 
    int GetHardwareVersion(ref string ctrlBoxBoardVersion, ref string driver1Version, ref string driver2Version, ref string driver3Version, ref string driver4Version, ref string driver5Version, ref string driver6Version, ref string endBoardVersion);

Get robot firmware version
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get robot firmware version information
    * @param [out] ctrlBoxBoardVersion Control box board firmware version
    * @param [out] driver1Version Driver 1 firmware version
    * @param [out] driver1Version Driver 2 firmware version
    * @param [out] driver1Version Driver 3 firmware version
    * @param [out] driver1Version Driver 4 firmware version
    * @param [out] driver1Version Driver 5 firmware version
    * @param [out] driver1Version Driver 6 firmware version
    * @param [out] endBoardVersion End board firmware version
    * @return Error code 
    */ 
    int GetFirmwareVersion(ref string ctrlBoxBoardVersion, ref string driver1Version, ref string driver2Version, ref string driver3Version, ref string driver4Version, ref string driver5Version, ref string driver6Version, ref string endBoardVersion);

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnGetVersions_Click(object sender, EventArgs e)
    {
        string[] ver = new string[20];
        int rtn = 0;
        rtn = robot.GetSoftwareVersion(ref ver[0], ref ver[1], ref ver[2]);
        rtn = robot.GetHardwareVersion(ref ver[3], ref ver[4], ref ver[5], ref ver[6], ref ver[7], ref ver[8], ref ver[9], ref ver[10]);
        rtn = robot.GetFirmwareVersion(ref ver[11], ref ver[12], ref ver[13], ref ver[14], ref ver[15], ref ver[16], ref ver[17], ref ver[18]);
        Console.WriteLine($"robotmodel  is: {ver[0]}");
        Console.WriteLine($"webVersion  is: {ver[1]}");
        Console.WriteLine($"controllerVersion  is: {ver[2]}");
        Console.WriteLine($"Hard ctrlBox Version  is: {ver[3]}");
        Console.WriteLine($"Hard driver1 Version  is: {ver[4]}");
        Console.WriteLine($"Hard driver2 Version is: {ver[5]}");
        Console.WriteLine($"Hard driver3 Version is: {ver[6]}");
        Console.WriteLine($"Hard driver4 Version is: {ver[7]}");
        Console.WriteLine($"Hard driver5 Version is: {ver[8]}");
        Console.WriteLine($"Hard driver6 Version is: {ver[9]}");
        Console.WriteLine($"Hard end Version is: {ver[10]}");
        Console.WriteLine($"Firm ctrlBox Version is: {ver[11]}");
        Console.WriteLine($"Firm driver1 Version is: {ver[12]}");
        Console.WriteLine($"Firm driver2 Version is: {ver[13]}");
        Console.WriteLine($"Firm driver3 Version is: {ver[14]}");
        Console.WriteLine($"Firm driver4 Version is: {ver[15]}");
        Console.WriteLine($"Firm driver5 Version is: {ver[16]}");
        Console.WriteLine($"Firm driver6 Version is: {ver[17]}");
        Console.WriteLine($"Firm end Version is: {ver[18]}");
    }

Switch to Manual High-Speed Mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Switch to manual high-speed mode
    * @param [in] state 0-exit manual high-speed; 1-enter manual high-speed
    * @return Error code
    */
    public int HiSpeedManualSwitch(int state)
