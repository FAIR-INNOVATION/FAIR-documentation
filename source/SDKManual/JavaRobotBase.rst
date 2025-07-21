Robot basics
==========================

.. toctree:: 
    :maxdepth: 5

Instantiate robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Robot interface class constructor
    */
    Robot robot = new Robot(); 

Establish communication with controller
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Establish communication with robot controller
    * @param [in] ip Controller ip address, default is 192.168.58.2
    * @return Error code
    */
    int RPC(String ip);

Close communication with robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Close communication with robot
    * @return Error code 
    */ 
    int CloseRPC(); 

Query sdk version
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Query sdk version 
    * @return Version number 
    */  
    String GetSDKVersion();

Get controller ip
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get controller ip
    * @param [out] ip Controller ip
    * @return Error code
    */
    int GetControllerIP(String[] ip);

Control robot to enter or exit drag teaching mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Control robot to enter or exit drag teaching mode
    * @param [in] state 0-exit drag teaching mode, 1-enter drag teaching mode
    * @return Error code
    */
    int DragTeachSwitch(int state);

Query whether robot is in drag teaching mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Query whether robot is in drag teaching mode
    * @param [in] state 0-not in drag teaching mode, 1-in drag teaching mode
    * @return Error code
    */
    int IsInDragTeach(List<Number> state);

Control robot enable or disable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Control robot enable or disable, robot is enabled by default after power on
    * @param [in] state 0-disable, 1-enable
    * @return Error code
    */
    int RobotEnable(int state); 

Control robot manual/auto mode switch
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Control robot manual/auto mode switch
    * @param [in] mode 0-auto mode, 1-manual mode
    * @return Error code
    */
    int Mode(int mode);

Shut down robot os
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.4-3.8.1

.. code-block:: Java
    :linenos:

    /**
    * @brief Shut down robot os
    * @return Error code
    */
    int ShutDownRobotOS();

Set robot communication reconnect parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set robot communication reconnect parameters
    * @param [in] enable Whether to enable, true:enable, false:disable
    * @param [in] times Reconnect times
    * @param [in] period Reconnect interval
    * @return Error code
    */
    int SetReconnectParam(boolean enable, int times, int period);

Initialize log parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Initialize log parameters
    * @param [in] logType Output mode, DIRECT-direct output; BUFFER-buffer output; ASYNC-async output
    * @param [in] logLevel Log filter level, ERROR-error; WARNING-warning; INFO-information; DEBUG-debug
    * @param [in] filePath File save path, e.g. "D://Log/"
    * @param [in] saveFileNum Save file count, files exceeding both save file count and save file days will be deleted
    * @param [in] saveDays Save file days, files exceeding both save file count and save file days will be deleted
    * @return Error code
    */
    int LoggerInit(FrLogType logType, FrLogLevel logLevel, String filePath, int saveFileNum, int saveDays)

Set log filter level
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set log filter level
    * @param [in] logLevel Log filter level, ERROR-error; WARNING-warning; INFO-information; DEBUG-debug
    * @return Error code
    */
    int SetLoggerLevel(FrLogLevel logLevel)

Robot basic control code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("Rpc connection success");
        }
        else
        {
            System.out.println("Rpc connection fail");
            return ;
        }
        String[] ip={""};
        String version = "";
        version=robot.GetSDKVersion();
        System.out.println("SDK version : " + version);
        int rtn = robot.GetControllerIP(ip);
        System.out.println("Controller ip : " +  ip[0] + "  " + rtn);
        robot.Mode(1);//1-manual mode  0-auto mode
        robot.Sleep(1000);
        robot.DragTeachSwitch(1);//Enter drag mode
        robot.Sleep(1000);
        ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
        System.out.println("Drag state : " + pkg.robot_state);
        robot.Sleep(1000);
        robot.DragTeachSwitch(0);//Exit drag mode
        robot.Sleep(1000);
        pkg = robot.GetRobotRealTimeState();
        System.out.println("Drag state : " + pkg.robot_state);
        
        if (pkg.robot_state ==4){
           System.out.println("Drag mode");
        }else {
           System.out.println("Non-drag mode");
        }
    }

Get robot software version
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get robot software version
    * @param [out] robotModel Robot model
    * @param [out] webVersion Web version
    * @param [out] controllerVersion Controller version
    * @return Error code 
    */
    int GetSoftwareVersion(String robotModel, String webVersion, String controllerVersion);

Get robot hardware version
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get robot hardware version
    * @param [out] ctrlBoxBoardVersion Control box board hardware version
    * @param [out] driver1Version Driver 1 hardware version
    * @param [out] driver2Version Driver 2 hardware version
    * @param [out] driver3Version Driver 3 hardware version
    * @param [out] driver4Version Driver 4 hardware version
    * @param [out] driver5Version Driver 5 hardware version
    * @param [out] driver6Version Driver 6 hardware version
    * @param [out] endBoardVersion End board hardware version
    * @return Error code 
    */
    int GetHardwareVersion(String ctrlBoxBoardVersion, String driver1Version, String driver2Version, String driver3Version,
                                          String driver4Version, String driver5Version, String driver6Version, String endBoardVersion);

Get robot firmware version
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get robot firmware version
    * @param [out] ctrlBoxBoardVersion Control box board firmware version
    * @param [out] driver1Version Driver 1 firmware version
    * @param [out] driver2Version Driver 2 firmware version
    * @param [out] driver3Version Driver 3 firmware version
    * @param [out] driver4Version Driver 4 firmware version
    * @param [out] driver5Version Driver 5 firmware version
    * @param [out] driver6Version Driver 6 firmware version
    * @param [out] endBoardVersion End board firmware version
    * @return Error code 
    */
    int GetFirmwareVersion(String ctrlBoxBoardVersion, String driver1Version, String driver2Version, String driver3Version,
                                          String driver4Version, String driver5Version, String driver6Version, String endBoardVersion);

Get robot software/firmware version code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set reconnect times and interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("Rpc connection success");
        }
        else
        {
            System.out.println("Rpc connection fail");
            return ;
        }
        String ctrlBoxBoardVersion = "";
        String driver1Version = "";
        String driver2Version = "";
        String driver3Version = "";
        String driver4Version = "";
        String driver5Version = "";
        String driver6Version = "";
        String endBoardVersion = "";
        robot.GetHardwareVersion(ctrlBoxBoardVersion ,driver1Version,  driver2Version,  driver3Version,
                 driver4Version,  driver5Version,  driver6Version,  endBoardVersion);

        robot.GetFirmwareVersion(ctrlBoxBoardVersion, driver1Version, driver2Version, driver3Version,
                driver4Version, driver5Version, driver6Version, endBoardVersion);
    }