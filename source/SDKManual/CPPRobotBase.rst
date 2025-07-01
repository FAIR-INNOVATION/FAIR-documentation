Basics
=================

.. toctree:: 
    :maxdepth: 5

Instantiate Robot
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Robot interface class constructor
    */
    FRRobot();

Establish Communication with Controller
++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Establish communication with robot controller
    * @param  [in] ip  Controller IP address, default is 192.168.58.2
    * @return Error code
    */
    errno_t  RPC(const char *ip);

Close Communication with Controller
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Close communication with robot controller
     * @return Error code
     */
    errno_t  CloseRPC();

Query SDK Version
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Query SDK version
    * @param  [out] version   SDK version
    * @return  Error code
    */  
    errno_t  GetSDKVersion(char *version);

Get Controller IP
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get controller IP
    * @param  [out] ip  Controller IP
    * @return  Error code
    */
    errno_t  GetControllerIP(char *ip);

Control Robot to Enter/Exit Drag Teaching Mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Control robot to enter/exit drag teaching mode
    * @param  [in] state 0-Exit drag teaching mode, 1-Enter drag teaching mode
    * @return  Error code
    */
    errno_t  DragTeachSwitch(uint8_t state);

Check if Robot is in Drag Mode
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Check if robot is in drag teaching mode
    * @param  [out] state 0-Not in drag teaching mode, 1-In drag teaching mode
    * @return  Error code
    */
    errno_t  IsInDragTeach(uint8_t *state);

Enable/Disable Robot
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Enable/disable robot (enabled by default after power on)
    * @param  [in] state  0-Disable, 1-Enable
    * @return  Error code
    */
    errno_t  RobotEnable(uint8_t state);

Switch Between Manual/Auto Mode
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Switch between manual/auto mode
    * @param [in] mode 0-Auto mode, 1-Manual mode
    * @return Error code
    */
    errno_t  Mode(int mode);

Shut Down Robot OS
+++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.1-3.8.1

.. code-block:: c++
    :linenos:

    /**
    * @brief Shut down robot operating system
    * @return Error code
    */
    errno_t ShutDownRobotOS();

Set Reconnection Parameters
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set communication reconnection parameters
    * @param [in] enable Enable reconnection when network fails true-enable false-disable
    * @param [in] reconnectTime Reconnection time (ms)
    * @param [in] period Reconnection interval (ms)
    * @return Error code
    */
    errno_t SetReConnectParam(bool enable, int reconnectTime = 30000, int period = 50);

Shut down the robot operating system
+++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Shut down the robot operating system
    * @return Error code
    */
    int ShutDownRobotOS();

Initialize Log Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Initialize log parameters;
    * @param output_model：Output mode, 0-Direct output; 1-Buffered output; 2-Asynchronous output;
    * @param file_path: File save path+name (max 256 chars), must be in xxx.log format, e.g. /home/fr/linux/fairino.log;
    * @param file_num：Number of rolling files, 1~20. Single file size limit 50MB;
    * @return errno_t Error code;
    */
    errno_t LoggerInit(int output_model = 0, std::string file_path = "", int file_num = 5);

Set Log Filter Level
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set log filter level;
    * @param lvl: Filter level, smaller value means less logs, default is 1. 1-error, 2-warning, 3-info, 4-debug;
    */
    void SetLoggerLevel(int lvl = 1);

Basic Robot Control Example
++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    private void btnStandard_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true, 100, 1000);
        robot.RPC("192.168.58.2")
        string ip = "";
        string version = "";
        byte state = 0
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
        Console.WriteLine($"drag state : {state}");
        Thread.Sleep(3000);
        robot.RobotEnable(0);
        Thread.Sleep(3000);
        robot.RobotEnable(1);

        robot.Mode(0);
        Thread.Sleep(1000);
        robot.Mode(1);
        Thread.Sleep(2000);

    }

Get Robot Software Version Example
++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Get robot software version
    * @param[out]    robotModel Robot model
    * @param[out]    webversion Web version
    * @param[out]    controllerVersion Controller version
    * @return Error code
    */
    errno_t GetSoftwareVersion(char robotModel[64], char webVersion[64], char controllerVersion[64]);

Get Robot Hardware Version
+++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get robot hardware version
    * @param[out] ctrlBoxBoardversion Control box board hardware version
    * @param[out] driver1version Driver 1 hardware version
    * @param[out] driver2version Driver 2 hardware version
    * @param[out] driver3version Driver 3 hardware version
    * @param[out] driver4version Driver 4 hardware version
    * @param[out] driver5version Driver 5 hardware version
    * @param[out] driver6version Driver 6 hardware version
    * @param[out] endBoardversion End board hardware version
    */
    errno_t GetHardwareVersion(char ctrlBoxBoardversion[128], char driver1version[128], char driver2version[128], char driver3version[128], char driver4version[128], char driver5version[128], char driver6version[128], char endBoardversion[128]);

Get Robot Firmware Version
+++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get robot firmware version
    * @param[out] ctrlBoxBoardversion Control box board firmware version
    * @param[out] driver1version Driver 1 firmware version
    * @param[out] driver2version Driver 2 firmware version
    * @param[out] driver3version Driver 3 firmware version
    * @param[out] driver4version Driver 4 firmware version
    * @param[out] driver5version Driver 5 firmware version
    * @param[out] driver6version Driver 6 firmware version
    * @param[out] endBoardversion End board firmware version
    */
    errno_t GetFirmwareVersion(char ctrlBoxBoardversion[128], char driver1version[128], char driver2version[128], char driver3version[128], char driver4version[128], char driver5version[128], char driver6version[128], char endBoardversion[128]);

Get Robot Software/Firmware Version Example
++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestGetVersions(void)
     {
         ROBOT_STATE_PKG pkg = {};
         FRRobot robot;
         robot.LoggerInit();
         robot.SetLoggerLevel(1);
         int rtn = robot.RPC("192.168.58.2");
         if (rtn != 0)
         {
             return -1;
         }
         robot.SetReConnectParam(true, 30000, 500);
         char robotModel[64] = { 0 };
         char webversion[64] = { 0 };
         char controllerVersion[64] = { 0 };
         char ctrlBoxBoardversion[128] = { 0 };
         char driver1version[128] = { 0 };
         char driver2version[128] = { 0 };
         char driver3version[128] = { 0 };
         char driver4version[128] = { 0 };
         char driver5version[128] = { 0 };
         char driver6version[128] = { 0 };
         char endBoardversion[128] = { 0 };
         rtn = robot.GetSoftwareVersion(robotModel, webversion, controllerVersion);
         printf("Getsoftwareversion rtn is: %d\n", rtn);
         printf("robotmodel is: %s, webversion is: %s, controllerVersion is: %s \n\n", robotModel, webversion, controllerVersion);
         rtn = robot.GetHardwareVersion(ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);
         printf("GetHardwareversion rtn is: %d\n", rtn);
         printf("GetHardwareversion get hardware versoin is: %s, %s, %s, %s, %s, %s, %s, %s\n\n", ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);
         rtn = robot.GetFirmwareVersion(ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);
         printf("GetFirmwareversion rtn is: %d\n", rtn);
         printf("GetHardwareversion get hardware versoin is: %s, %s, %s, %s, %s, %s, %s, %s\n\n", ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);
         robot.CloseRPC();
         return 0;
     }
