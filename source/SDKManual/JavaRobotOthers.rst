Other interfaces
================

.. toctree:: 
    :maxdepth: 5

Get SSH public key
+++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get SSH public key
    * @param [out] keygen Public key
    * @return Error code
    */
    int GetSSHKeygen(String[] keygen)

Issue SCP command
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.6-3.8.3

.. code-block:: Java
    :linenos:

    /** 
    * @brief Issue SCP command
    * @param [in] mode 0-upload (host->controller), 1-download (controller->host)
    * @param [in] sshname Host username
    * @param [in] sship Host IP address
    * @param [in] usr_file_url Host file path
    * @param [in] robot_file_url Robot controller file path
    * @return Error code
    */
    int SetSSHScpCmd(int mode, String sshname, String sship, String usr_file_url, String robot_file_url)

Calculate MD5 value of specified file
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Calculate MD5 value of specified file
    * @param [in] file_path File path including filename, default Traj folder path:"/fruser/traj/", e.g. "/fruser/traj/trajHelix_aima_1.txt"
    * @param [out] md5 File MD5 value
    * @return Error code
    */
    int ComputeFileMD5(String file_path, String[] md5)

Robot SSH and MD5 command code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestSSHMd5(Robot robot)
    {
        String file_path= "/fruser/airlab.lua";
        String[] md5 =new String[]{""};

        String[] ssh_keygen=new String[]{""};
        int retval = robot.GetSSHKeygen(ssh_keygen);
        System.out.println(ssh_keygen[0]);

        String ssh_name = "fr";
        String ssh_ip = "192.168.58.45";
        String ssh_route = "/home/fr";
        String ssh_robot_url = "/root/robot/dhpara.config";
        retval = robot.SetSSHScpCmd(1, ssh_name, ssh_ip, ssh_route, ssh_robot_url);
        System.out.println("SetSSHScpCmd retval is:"+ retval);
        System.out.println("robot url is:"+ ssh_robot_url);

        robot.ComputeFileMD5(file_path, md5);
        System.out.println("md5 is:+"+ md5[0]);
        return 0;
    }

Set robot port 20004 feedback period
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set robot port 20004 feedback period
    * @param [in] period Robot port 20004 feedback period(ms)
    * @return  Error code
    */
    public int SetRobotRealtimeStateSamplePeriod(int period)

Get robot port 20004 feedback period
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get robot port 20004 feedback period
    * @return  List[0]:Error code; List[1]:Robot port 20004 feedback period(ms)
    */
    public List<Integer> GetRobotRealtimeStateSamplePeriod()

Robot port 20004 status feedback period configuration example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestRealtimePeriod(Robot robot)
    {
        robot.SetRobotRealtimeStateSamplePeriod(10);
        List<Integer> getPeriod = new ArrayList<>();
        getPeriod=robot.GetRobotRealtimeStateSamplePeriod();
        robot.Sleep(1000);

        return 0;
    }

Robot software upgrade
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Robot software upgrade
     * @param [in] filePath Full path of software upgrade package
     * @param [in] block Whether to block until upgrade completes true:block; false:non-block
     * @return  Error code
     */
    public int SoftwareUpgrade(String filePath, boolean block)

Get robot software upgrade status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get robot software upgrade status
    * @return  List[0]:Error code; List[1]:Robot software upgrade status 0-idle or uploading upgrade package; 1~100: upgrade completion percentage; -1:upgrade failed; -2:verification failed; -3:version verification failed; -4:decompression failed; -5:user configuration upgrade failed; -6:peripheral configuration upgrade failed; -7:extended axis configuration upgrade failed; -8:robot configuration upgrade failed; -9:DH parameter configuration upgrade failed
    */
    public List<Integer> GetSoftwareUpgradeState()

Robot software upgrade code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestUpgrade(Robot robot)
    {
        robot.SoftwareUpgrade("D://zUP/QNX382/software.tar.gz", false);
        while (true)
        {
            List<Integer> inter=new ArrayList<>();
            inter=robot.GetSoftwareUpgradeState();
            System.out.println("upgrade state is:"+ inter.get(1));
            robot.Sleep(300);
        }
    }

Download point table database
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Download point table database 
    * @param [in] pointTableName Point table name to download    pointTable1.db
    * @param [in] saveFilePath Storage path for downloaded point table   C://test/
    * @return Error code 
    */
    int PointTableDownLoad(String pointTableName, String saveFilePath);

Upload point table database
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Upload point table database 
    * @param [in] pointTableFilePath Full path name of point table to upload   C://test/pointTable1.db
    * @return Error code 
    */
    int PointTableUpLoad(String pointTableFilePath);

Point table update lua file
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Point table update lua file
    * @param [in] pointTableName Point table name to switch to   "pointTable1.db", when empty "", means updating lua program to initial program without applied point table
    * @param [in] luaFileName Lua file name to update   "testPointTable.lua"
    * @param [out] errorStr Point table switching error message
    * @return Error code 
    */
    int PointTableUpdateLua(String pointTableName, String luaFileName, String errorStr);

Robot point table operation code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestPointTable(Robot robot)
    {
        String save_path = "D://zDOWN/";
        String point_table_name = "point_table_FR5.db";
        int rtn = robot.PointTableDownLoad(point_table_name, save_path);

        String upload_path = "D://zUP/point_table_FR5.db";
        rtn = robot.PointTableUpLoad(upload_path);

        String point_tablename = "point_table_FR5.db";
        String lua_name = "airlab.lua";
        String err="";
        rtn = robot.PointTableUpdateLua(point_tablename, lua_name,err);

        robot.CloseRPC();
        return 0;
    }

Controller log download
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.4-3.8.1

.. code-block:: Java
    :linenos:

    /** 
    * @brief Controller log download
    * @param [in] savePath Save file path"D://zDown/"
    * @return Error code
    */
    int RbLogDownload(String savePath);

All data source download
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.4-3.8.1

.. code-block:: Java
    :linenos:

    /** 
    * @brief All data source download
    * @param [in] savePath Save file path"D://zDown/"
    * @return Error code
    */
    int AllDataSourceDownload(String savePath);

Data backup package download
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.4-3.8.1

.. code-block:: Java
    :linenos:

    /** 
    * @brief Data backup package download
    * @param [in] savePath Save file path"D://zDown/"
    * @return Error code
    */
    int DataPackageDownload(String savePath);

Download controller data code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestDownLoadRobotData(Robot robot)
    {
        int rtn = robot.RbLogDownload("D://zDOWN/");

        rtn = robot.AllDataSourceDownload("D://zDOWN/");

        rtn = robot.DataPackageDownload("D://zDOWN/");
        return 0;
    }

Set Encoder Upgrade
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.7-3.8.4

.. code-block:: Java
    :linenos:

    /**
    * @brief Set encoder upgrade
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code
    */
    int SetEncoderUpgrade(String path)

Set Joint Firmware Upgrade
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.7-3.8.4

.. code-block:: Java
    :linenos:

    /**
    * @brief Set joint firmware upgrade
    * @param [in] type Upgrade file type: 1-Firmware upgrade; 2-Slave configuration file upgrade
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code
    */
    public int SetJointFirmwareUpgrade(int type, String path)

Set Controller Firmware Upgrade
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.7-3.8.4

.. code-block:: Java
    :linenos:

    /**
    * @brief Set controller firmware upgrade
    * @param [in] type Upgrade file type: 1-Firmware upgrade; 2-Slave configuration file upgrade
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code
    */
    public int SetCtrlFirmwareUpgrade(int type, String path)

Set End-Effector Firmware Upgrade
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.7-3.8.4

.. code-block:: Java
    :linenos:

    /**
    * @brief Set end-effector firmware upgrade
    * @param [in] type Upgrade file type: 1-Firmware upgrade; 2-Slave configuration file upgrade
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code
    */
    public int SetEndFirmwareUpgrade(int type, String path)

Joint Complete Parameter Configuration Upgrade
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.7-3.8.4

.. code-block:: Java
    :linenos:

    /**
    * @brief Joint complete parameter configuration upgrade
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code
    */
    public int JointAllParamUpgrade(String path)

Robot Slave Firmware Upgrade Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void TestFirmWareUpgrade(Robot robot)
    {
        robot.RobotEnable(0);
        robot.Sleep(200);
        int rtn = robot.JointAllParamUpgrade("D://zUP/standardQX/jointallparametersFR56.0.db");
        System.out.println("robot JointAllParamUpgrade rtn is:"+ rtn);

        rtn = robot.SetCtrlFirmwareUpgrade(2, "D://zUP/upgrade/FAIR_Cobot_Cbd_Asix_V2.0.bin");
        System.out.println("robot SetCtrlFirmwareUpgrade config param rtn is:"+ rtn);

        rtn = robot.SetEndFirmwareUpgrade(2, "D://zUP/upgrade/FAIR_Cobot_Axle_Asix_V2.4.bin");
        System.out.println("robot SetEndFirmwareUpgrade config param rtn is:"+ rtn);

        robot.SetSysServoBootMode();
        rtn = robot.SetCtrlFirmwareUpgrade(1, "D://zUP/standardQX/FR_CTRL_PRIMCU_FV201010_MAIN_U4_T01_20240529.bin");
        System.out.println("robot SetCtrlFirmwareUpgrade rtn is:"+ rtn);

        rtn = robot.SetEndFirmwareUpgrade(1, "D://zUP/standardQX/FR_END_FV201010_MAIN_U01_T01_20250522.bin");
        System.out.println("robot SetEndFirmwareUpgrade rtn is:"+ rtn);

        rtn = robot.SetJointFirmwareUpgrade(1, "D://zUP/standardQX/FR_SERVO_FV502211_MAIN_U7_T07_20250217.bin");
        System.out.println("robot SetJointFirmwareUpgrade rtn is:"+ rtn);

        robot.CloseRPC();
    }

Robot Operating System Upgrade (LA Control Box)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: Java
    :linenos:

    /**
     * @brief Robot Operating System Upgrade (LA Control Box)
     * @param [in] filePath Full path of the operating system upgrade package
     * @return Error code
     */
    public int KernelUpgrade(String filePath)

Get Robot Operating System Upgrade Result (LA Control Box)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: Java
    :linenos:

    /**
     * @brief Get Robot Operating System Upgrade Result (LA Control Box)
     * @param [out] result Upgrade result: 0: Success; -1: Failure
     * @return Error code
     */
    public int GetKernelUpgradeResult(int[] result)

Robot MCU Log Generation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Robot MCU log generation
    * @return Error code
    */
    public int RobotMCULogCollect()

Set Robot to Stop Running When Port Communication is Disconnected
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set robot to stop running when port communication is disconnected
    * @param portID Port number 0-8080; 1-8083; 2-20002; 3-20004
    * @param enable 0-disable; 1-enable
    * @param confirmTime Communication disconnection confirmation duration (ms)[0-5000]
    * @return Error code
    */
    public int SetRobotStopOnComDisc(int portID, bool enable, int confirmTime)
    
Get Robot Stop on Communication Disconnection Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get robot stop on communication disconnection parameters
    * @param portID Port number 0-8080; 1-8083; 2-20002; 3-20004
    * @param enable Result array, index 0: 0-disable; 1-enable
    * @param confirmTime Result array, index 0: Communication disconnection confirmation duration (ms)[0-5000]
    * @return Error code
    */
    public int GetRobotStopOnComDisc(int portID, int[] enable, int[] confirmTime)

Robot Stop on Communication Disconnection Parameter Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    void TestRobotStopOnComDisc(Robot robot)
    {
        int[] enable = {0};
        int[] confirmTime = {0};
        int rtn = 0;
        rtn = robot.SetRobotStopOnComDisc(0, true, 330);
        rtn = robot.SetRobotStopOnComDisc(1, true, 550);
        rtn = robot.SetRobotStopOnComDisc(2, true, 110);
        rtn = robot.SetRobotStopOnComDisc(3, true, 220);
        System.out.printf("SetRobotStopOnComDisc %d\n", rtn);

        robot.GetRobotStopOnComDisc(0, enable, confirmTime);
        System.out.printf("GetRobotStopOnComDisc 8080 rtn %d; enable is %d; confirm time is %d\n", rtn, enable[0], confirmTime[0]);
        robot.GetRobotStopOnComDisc(1, enable, confirmTime);
        System.out.printf("GetRobotStopOnComDisc 8083 rtn %d; enable is %d; confirm time is %d\n", rtn, enable[0], confirmTime[0]);
        robot.GetRobotStopOnComDisc(2, enable, confirmTime);
        System.out.printf("GetRobotStopOnComDisc 20002 rtn %d; enable is %d; confirm time is %d\n", rtn, enable[0], confirmTime[0]);
        robot.GetRobotStopOnComDisc(3, enable, confirmTime);
        System.out.printf("GetRobotStopOnComDisc 20004 rtn %d; enable is %d; confirm time is %d\n", rtn, enable[0], confirmTime[0]);

        return;
    }

Send UDP Instruction Frame
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Send UDP instruction frame
    * @param frame Instruction frame
    * @return Error code
    */
    public int SendUDPFrame(String frame)
    
SDK Code Example for UDP Communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void TestRobotUDP (Robot robot) {
        robot.udpCmdClient.SetUDPCmdRpyCallback((srcType, count, cmdID, dataLen, content) -> {
            System.out.println("\n[Received robot UDP response]");
            System.out.println("srcType: " + srcType);
            System.out.println("count: " + count);
            System.out.println("cmdID: " + cmdID);
            System.out.println("dataLen: " + dataLen);
            System.out.println("content: " + content);
            return 0;
        });
        // Send frame
        String frameToSend = "/f/bIII52III236III7IIIMode(1)III/b/f";
        robot.SendUDPFrame(frameToSend);
        robot.Sleep(2000);
        frameToSend = "/f/bIII52III236III7IIIMode(0)III/b/f";
        robot.SendUDPFrame(frameToSend);
        robot.Sleep(2000);
        frameToSend = "/f/bIII41III201III153IIIMoveJ(53.857,-89.441,119.453,-22.664,61.059,3.369,-54.249,-491.930,375.396,96.474,-6.896,-7.783,0,0,100,100,100,0.000,0.000,0.000,0.000,-1,0,0,0,0,0,0,0)III/b/f";
        robot.SendUDPFrame(frameToSend);
        robot.Sleep(2000);
        frameToSend = "/f/bIII42III203III163IIIMoveL(81.736,-85.284,114.974,-23.261,88.746,6.799,125.744,-506.570,375.396,96.474,-6.896,-7.783,0,0,100,100,100,-1,0,0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0,100,0)III/b/f";
        robot.SendUDPFrame(frameToSend);
        robot.Sleep(2000);
        frameToSend = "/f/bIII47III400III15IIIGetMCVersion(1)III/b/f/f/bIII48III424III21IIIGetSlaveFirmVersion()III/b/f";
        robot.SendUDPFrame(frameToSend);
        robot.Sleep(2000);
    }
        
Set User-Defined Robot End-Effector LED Color
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set user-defined robot end-effector LED color
    * @param r End red LED control; 0-off; 1-on
    * @param g End green LED control; 0-off; 1-on
    * @param b End blue LED control; 0-off; 1-on
    * @return Error code
    */
    public int SetUserLEDColor(bool r, bool g, bool b)
            
SDK Code Example for Setting User-Defined Robot End-Effector LED Color
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public void testled(robot)
    {
        robot.SetUserLEDColor(true, true, true);
        robot.Sleep(1000);
        robot.SetUserLEDColor(false, false, false);
        robot.Sleep(1000);
        robot.SetUserLEDColor(true, false, false);
        robot.Sleep(1000);
        robot.SetUserLEDColor(false, true, false);
        robot.Sleep(1000);
        robot.SetUserLEDColor(false, false, true);
    }