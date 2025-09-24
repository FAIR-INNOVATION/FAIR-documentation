Other Interfaces
=================

.. toctree:: 
    :maxdepth: 5

Get SSH Public Key
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Get SSH public key
    * @param [out] keygen Public key
    * @return Error code
    */
    errno_t GetSSHKeygen(char keygen[1024]);

Send SCP Command
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Send SCP command
    * @param [in] mode 0-Upload (host->controller), 1-Download (controller->host)
    * @param [in] sshname Host username
    * @param [in] sship Host IP address
    * @param [in] usr_file_url Host file path
    * @param [in] robot_file_url Robot controller file path
    * @return Error code
    */
    errno_t SetSSHScpCmd(int mode, char sshname[32], char sship[32], char usr_file_url[128], char robot_file_url[128]);

Calculate MD5 Value of Specified File
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Calculate MD5 value of specified file
    * @param [in] file_path File path including filename, default Traj folder path: "/fruser/traj/", e.g. "/fruser/traj/trajHelix_aima_1.txt"
    * @param [out] md5 File MD5 value
    * @return Error code
    */
    errno_t ComputeFileMD5(char file_path[256], char md5[256]);

Robot SSH/MD5 Command Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestSSHMd5(void)
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
      char file_path[256] = "/fruser/airlab.lua";
      char md5[256] = { 0 };
      uint8_t emerg_state = 0;
      uint8_t si0_state = 0;
      uint8_t si1_state = 0;
      int sdk_com_state = 0;
      char ssh_keygen[1024] = { 0 };
      int retval = robot.GetSSHKeygen(ssh_keygen);
      printf("GetSSHKeygen retval is: %d\n", retval);
      printf("ssh key is: %s \n", ssh_keygen);
      char ssh_name[32] = "fr";
      char ssh_ip[32] = "192.168.58.45";
      char ssh_route[128] = "/home/fr";
      char ssh_robot_url[128] = "/root/robot/dhpara.config";
      retval = robot.SetSSHScpCmd(1, ssh_name, ssh_ip, ssh_route, ssh_robot_url);
      printf("SetSSHScpCmd retval is: %d\n", retval);
      printf("robot url is: %s\n", ssh_robot_url);
      robot.ComputeFileMD5(file_path, md5);
      printf("md5 is: %s \n", md5);
      robot.CloseRPC();
      return 0;
    }

Set Robot Port 20004 Feedback Cycle
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set robot port 20004 feedback cycle
    * @param [in] period Robot port 20004 feedback cycle (ms)
    * @return Error code
    */
    errno_t SetRobotRealtimeStateSamplePeriod(int period);

Get Robot Port 20004 Feedback Cycle
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get robot port 20004 feedback cycle
    * @param [out] period Robot port 20004 feedback cycle (ms)
    * @return Error code
    */
    errno_t GetRobotRealtimeStateSamplePeriod(int& period);

Robot Port 20004 State Feedback Cycle Configuration Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestRealtimePeriod(void)
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
      robot.SetRobotRealtimeStateSamplePeriod(10);
      int getPeriod = 0;
      robot.GetRobotRealtimeStateSamplePeriod(getPeriod);
      cout << "period is " << getPeriod << endl;
      robot.Sleep(1000);
      robot.CloseRPC();
      return 0;
    }

Robot Software Upgrade
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Robot software upgrade
    * @param [in] filePath Full path of software upgrade package
    * @param [in] block Whether to block until upgrade completes (true: block; false: non-block)
    * @return Error code
    */
    errno_t SoftwareUpgrade(std::string filePath, bool block);

Get Robot Software Upgrade Status
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get robot software upgrade status
    * @param [out] state Robot software upgrade status (0-Idle or uploading upgrade package; 1~100: Upgrade completion percentage; -1: Upgrade failed; -2: Verification failed; -3: Version verification failed; -4: Decompression failed; -5: User configuration upgrade failed; -6: Peripheral configuration upgrade failed; -7: Extended axis configuration upgrade failed; -8: Robot configuration upgrade failed; -9: DH parameter configuration upgrade failed)
    * @return Error code
    */
    errno_t GetSoftwareUpgradeState(int &state);

Robot Software Upgrade Code Example
+++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestUpgrade(void)
    {
      ROBOT_STATE_PKG pkg = {};
      FRRobot robot;
      robot.LoggerInit();
      robot.SetLoggerLevel(3);
      int rtn = robot.RPC("192.168.58.2");
      if (rtn != 0)
      {
        return -1;
      }
      robot.SetReConnectParam(true, 30000, 500);
      robot.SoftwareUpgrade("D://zUP/QNX/software.tar.gz", false);
      while (true)
      {
        int curState = -1;
        robot.GetSoftwareUpgradeState(curState);
        printf("upgrade state is %d\n", curState);
        robot.Sleep(300);
      }
      robot.CloseRPC();
      return 0;
    }

Download Point Table Database
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Download point table database
    * @param [in] pointTableName Point table name to download    pointTable1.db
    * @param [in] saveFilePath Storage path for downloaded point table   C://test/
    * @return Error code
    */
    errno_t PointTableDownLoad(const std::string &pointTableName, const std::string &saveFilePath);

Upload Point Table Database
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Upload point table database
    * @param [in] pointTableFilePath Full path name of point table to upload   C://test/pointTable1.db
    * @return Error code
    */
    errno_t PointTableUpLoad(const std::string &pointTableFilePath);

Update Lua File for Point Table
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Update Lua file for point table
    * @param [in] pointTableName Point table name to switch to   "pointTable1.db", when empty (""), means updating Lua program to initial program without applied point table
    * @param [in] luaFileName Lua file name to update   "testPointTable.lua"
    * @param [out] errorStr Error message for switching point table
    * @return Error code
    */
    errno_t PointTableUpdateLua(const std::string &pointTableName, const std::string &luaFileName);

Robot Point Table Operation Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestPointTable(void)
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
      string save_path = "D://zDOWN/";
      string point_table_name = "point_table_FR5.db";
      rtn = robot.PointTableDownLoad(point_table_name, save_path);
      cout << "download : " << point_table_name << " fail: " << rtn << endl;
      string upload_path = "D://zUP/point_table_FR5.db";
      rtn = robot.PointTableUpLoad(upload_path);
      cout << "retval is: " << rtn << endl;
      string point_tablename = "point_table_FR5.db";
      string lua_name = "airlab.lua";
      rtn = robot.PointTableUpdateLua(point_tablename, lua_name);
      cout << "retval is: " << rtn << endl;
      robot.CloseRPC();
      return 0;
    }

Controller Log Download
+++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.1-3.8.1

.. code-block:: c++
    :linenos:

    /**
    * @brief Controller log download
    * @param [in] savePath Save file path "D://zDown/"
    * @return Error code
    */
    errno_t RbLogDownload(std::string savePath);

All Data Source Download
+++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.1-3.8.1

.. code-block:: c++
    :linenos:

    /**
    * @brief All data source download
    * @param [in] savePath Save file path "D://zDown/"
    * @return Error code
    */
    errno_t AllDataSourceDownload(std::string savePath);

Data Backup Package Download
+++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.1-3.8.1

.. code-block:: c++
    :linenos:

    /**
    * @brief Data backup package download
    * @param [in] savePath Save file path "D://zDown/"
    * @return Error code
    */
    errno_t DataPackageDownload(std::string savePath);

Download Controller Data Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestDownLoadRobotData(void)
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
      rtn = robot.RbLogDownload("D://zDOWN/");
      cout << "RbLogDownload rtn is " << rtn << endl;
      rtn = robot.AllDataSourceDownload("D://zDOWN/");
      cout << "AllDataSourceDownload rtn is " << rtn << endl;
      rtn = robot.DataPackageDownload("D://zDOWN/");
      cout << "DataPackageDownload rtn is " << rtn << endl;
      robot.CloseRPC();
      return 0;
    }

Set Joint Firmware Upgrade
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Set joint firmware upgrade
    * @param [in] type Upgrade file type: 1-Firmware upgrade (requires boot mode); 2-Slave config file upgrade (requires robot disable)
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code
    */
    errno_t SetJointFirmwareUpgrade(int type, std::string path);
  
Set Controller Firmware Upgrade
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Set controller firmware upgrade
    * @param [in] type Upgrade file type: 1-Firmware upgrade (requires boot mode); 2-Slave config file upgrade (requires robot disable)
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code
    */
    errno_t SetCtrlFirmwareUpgrade(int type, std::string path);
  
Set End-Effector Firmware Upgrade
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Set end-effector firmware upgrade
    * @param [in] type Upgrade file type: 1-Firmware upgrade (requires boot mode); 2-Slave config file upgrade (requires robot disable)
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code
    */
    errno_t SetEndFirmwareUpgrade(int type, std::string path);

Joint Full Parameter Configuration Upgrade
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Joint full parameter configuration upgrade (requires robot disable)
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code
    */
    errno_t JointAllParamUpgrade(std::string path);
    
Robot Slave Firmware Upgrade Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestFirmWareUpgrade()
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
    robot.RobotEnable(0);
    robot.Sleep(200);
    rtn = robot.JointAllParamUpgrade("D://zUP/upgrade/jointallparameters.db");
    printf("robot JointAllParamUpgrade rtn is %d\n", rtn);
    rtn = robot.SetCtrlFirmwareUpgrade(2, "D://zUP/upgrade/FAIR_Cobot_Cbd_Asix_V2.0.bin");
    printf("robot SetCtrlFirmwareUpgrade config param rtn is %d\n", rtn);
    rtn = robot.SetEndFirmwareUpgrade(2, "D://zUP/upgrade/FAIR_Cobot_Axle_Asix_V2.4.bin");
    printf("robot SetEndFirmwareUpgrade config param rtn is %d\n", rtn);
    robot.SetSysServoBootMode();
    rtn = robot.SetCtrlFirmwareUpgrade(1, "D://zUP/upgrade/FR_CTRL_PRIMCU_FV201212_MAIN_U4_T01_20250428(MT).bin");
    printf("robot SetCtrlFirmwareUpgrade rtn is %d\n", rtn);
    rtn = robot.SetEndFirmwareUpgrade(1, "D://zUP/upgrade/FR_END_FV201009_MAIN_U1_T01_20250428.bin");
    printf("robot SetEndFirmwareUpgrade rtn is %d\n", rtn);
    rtn = robot.SetJointFirmwareUpgrade(1, "D://zUP/upgrade/FR_SERVO_FV504214_MAIN_U7_T07_20250519.bin");
    printf("robot SetJointFirmwareUpgrade rtn is %d\n", rtn);
    robot.CloseRPC();
    return 0;
    }

Robot Operating System Upgrade (LA Control Box)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Robot Operating System Upgrade (LA Control Box)
    * @param [in] filePath Full path of the operating system upgrade package
    * @return Error code
    */
    errno_t KernelUpgrade(std::string filePath);
        
Get Robot Operating System Upgrade Result (LA Control Box)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get Robot Operating System Upgrade Result (LA Control Box)
    * @param [out] result Upgrade result: 0: Success; -1: Failure
    * @return Error code
    */
    errno_t GetKernelUpgradeResult(int& result);