Other interfaces
=================

.. toctree:: 
    :maxdepth: 5

Get SSH public key
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get SSH public key 
    * @param [out] keygen Public key
    * @return Error code 
    */
    int GetSSHKeygen(ref string keygen);

Send SCP command
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.4  Web-3.8.3
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Send SCP command
    * @param [in] mode 0-Upload (host computer -> controller), 1-Download (controller -> host computer)
    * @param [in] sshname Host computer username
    * @param [in] sship Host computer IP address
    * @param [in] usr_file_url Host computer file path
    * @param [in] robot_file_url Robot controller file path
    * @return Error code
    */
    int SetSSHScpCmd(int mode, string sshname, string sship, string usr_file_url, string robot_file_url);

Calculate the MD5 value of a file in a specified path
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Calculate the MD5 value of a file in the specified path 
    * @param [in] file_path File path including file name, default Traj folder path is "/fruser/traj/", such as "/fruser/traj/trajHelix_aima_1.txt"
    * @param [out] md5 The MD5 value of the file
    * @return Error code 
    */
    int ComputeFileMD5(string file_path, ref string md5);

Robot SSH and MD5 command code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.4  Web-3.8.3
    
.. code-block:: c#
    :linenos:

    private void button46_Click(object sender, EventArgs e)
    {
        string file_path = "/fruser/airlab.lua";
        string md5 = "";
        byte emerg_state = 0;
        byte si0_state = 0;
        byte si1_state = 0;
        int sdk_com_state = 0;

        string ssh_keygen = "";
        int retval = robot.GetSSHKeygen(ref ssh_keygen);
        Console.WriteLine("GetSSHKeygen retval is: {0}", retval);
        Console.WriteLine("ssh key is: {0}", ssh_keygen);

        string ssh_name = "fr";
        string ssh_ip = "192.168.58.45";
        string ssh_route = "/home/fr";
        string ssh_robot_url = "/root/robot/dhpara.config";
        retval = robot.SetSSHScpCmd(1, ssh_name, ssh_ip, ssh_route, ssh_robot_url);
        Console.WriteLine("SetSSHScpCmd retval is: {0}", retval);
        Console.WriteLine("robot url is: {0}", ssh_robot_url);

        robot.ComputeFileMD5(file_path, ref md5);
        Console.WriteLine("md5 is: {0}", md5);
    }

Set robot 20004 port feedback cycle
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the robot's 20004 port feedback period
    * @param [in] period Robot's 20004 port feedback period (ms)
    * @return Error code
    */
    int SetRobotRealtimeStateSamplePeriod(int period);

Get the robot's 20004 port feedback period
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the feedback period for robot port 20004
    * @param [out] period Feedback period for robot port 20004 (ms)
    * @return Error code
    */
    int GetRobotRealtimeStateSamplePeriod((ref int period);   

Robot 20004 port status feedback period configuration code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button47_Click(object sender, EventArgs e)
    {
        robot.SetRobotRealtimeStateSamplePeriod(10);
        int getPeriod = 0;
        robot.GetRobotRealtimeStateSamplePeriod(ref getPeriod);
        Console.WriteLine("period is {0}", getPeriod);
        Thread.Sleep(1000);
    }

Robot Software Upgrade
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Robot software upgrade
    * @param [in] filePath Full path of the software upgrade package
    * @param [in] block Whether to block until the upgrade is complete true: block; false: non-blocking
    * @return Error code
    */
    int SoftwareUpgrade(string filePath, bool block);

Get robot software upgrade status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get robot software upgrade status
    * @param [out] state Robot software package upgrade status  0-idle or uploading upgrade package; 1~100: upgrade completion percentage; -1: upgrade software failed; -2: verification failed; -3: Version verification failed; -4: Unzipping failed; -5: User configuration upgrade failed; -6: Peripheral configuration upgrade failed; -7: Extended axis configuration upgrade failed; -8: Robot configuration upgrade failed; -9: DH parameter configuration upgrade failed
    * @return Error code
    */
    int GetSoftwareUpgradeState(ref int state);

Robot software upgrade code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button48_Click(object sender, EventArgs e)
    {
        robot.SoftwareUpgrade("D://zUP/QNX382/software.tar.gz", false);
        while (true)
        {
            int curState = -1;
            robot.GetSoftwareUpgradeState(ref curState);
            Console.WriteLine("upgrade state is {0}", curState);
            Thread.Sleep(300);
        }
    }

Download point table
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Download the point table from the robot controller to the local computer 
    * @param [in] pointTableName The name of the point table in the controller: pointTable1.db
    * @param [in] saveFilePath The path where the point table is downloaded to the computer: C://test/
    * @return Error code 
    */
    int PointTableDownLoad(string pointTableName, string saveFilePath);

Upload point table
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Upload the point table from the local computer to the robot controller 
    * @param [in] pointTableFilePath The absolute path of the point table on the local computer C://test/pointTable1.db
    * @return Error code 
    */
    int PointTableUpLoad(string pointTableFilePath);

Point Table Update Lua Program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Use the given point table to update the points in the Lua program
    * @param [in] pointTableName The name of the point table in the controller: "pointTable1.db". When the point table is empty (i.e., ""), it indicates that the Lua program will be updated to the initial program without applying the point table.
    * @param [in] luaFileName The name of the Lua file to be updated: "test.lua"
    * @param [out] errorStr Error message for updating the point table in Lua  
    * @return Error code 
    */
    int PointTableUpdateLua(string pointTableName, string luaFileName, ref string errorStr);

Switch point tables and apply
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Switch point table and apply
    * @param [in] pointTableName Name of the point table to switch to   "pointTable1.db"
    * @param [out] errorStr Error message for switching point tables   
    * @return Error code 
    */
    int PointTableSwitch(string pointTableName, ref string errorStr);

Robot point table operation code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnUpload_Click(object sender, EventArgs e)
    {
        string save_path = "D://zDOWN/";
        string point_table_name = "test_point_A.db";
        int rtn = robot.PointTableDownLoad(point_table_name, save_path);
        Console.WriteLine("download : {0} fail: {1}", point_table_name, rtn);

        string upload_path = "D://zUP/test_point_A.db";
        rtn = robot.PointTableUpLoad(upload_path);
        Console.WriteLine("retval is: {0}", rtn);

        string point_tablename = "test_point_A.db";
        string lua_name = "Text1.lua";

        string errorStr = "";
        rtn = robot.PointTableUpdateLua(point_tablename, lua_name, ref errorStr);
        Console.WriteLine("retval is: {0}", rtn);
    }

Controller log download
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Controller log download
    * @param [in] savePath Save file path "D://zDown/"
    * @return Error code
    */
    int RbLogDownload(string savePath);

All Data Source Download
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief All Data Source Download
    * @param [in] savePath Save file path "D://zDown/"
    * @return Error code
    */
    int AllDataSourceDownload(string savePath);

Data backup package download
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Data backup package download
    * @param [in] savePath Save file path "D://zDown/"
    * @return Error code
    */
    int DataPackageDownload(string savePath);

Download controller data code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button50_Click(object sender, EventArgs e)
    {
        int rtn = robot.RbLogDownload("D://zDOWN/");
        Console.WriteLine("RbLogDownload rtn is {0}", rtn);

        rtn = robot.AllDataSourceDownload("D://zDOWN/");
        Console.WriteLine("AllDataSourceDownload rtn is {0}", rtn);

        rtn = robot.DataPackageDownload("D://zDOWN/");
        Console.WriteLine("DataPackageDownload rtn is {0}", rtn);
    }

Robot Operating System Upgrade (LA Control Box)
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
     * @brief Robot Operating System Upgrade (LA Control Box)
     * @param [in] filePath Full path of the operating system upgrade package
     * @return Error code
     */
    public int KernelUpgrade(string filePath)

Get Robot Operating System Upgrade Result (LA Control Box)
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
     * @brief Get Robot Operating System Upgrade Result (LA Control Box)
     * @param [out] result Upgrade result: 0: Success; -1: Failure
     * @return Error code
     */
    public int GetKernelUpgradeResult(ref int[] result)

Set encoder upgrade
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.5  Web-3.8.4
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set encoder upgrade
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code 
    */
    int SetEncoderUpgrade(string path);

Set joint firmware upgrade
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.5  Web-3.8.4
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set joint firmware upgrade
    * @param [in] type Upgrade file type; 1 - Upgrade firmware; 2 - Upgrade slave configuration file
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code 
    */
    int SetJointFirmwareUpgrade(int type, string path);

Set firmware upgrade for control box
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.5  Web-3.8.4
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set firmware upgrade for control box
    * @param [in] type Type of upgrade file; 1 - Upgrade firmware; 2 - Upgrade slave station configuration file
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code 
    */
    int SetCtrlFirmwareUpgrade(int type, string path);

Set end firmware upgrade
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.5  Web-3.8.4
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set end firmware upgrade
    * @param [in] type Upgrade file type; 1 - Upgrade firmware; 2 - Upgrade slave configuration file
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code 
    */
    int SetEndFirmwareUpgrade(int type, string path);

Upgrade of the joint full parameter configuration file
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.5  Web-3.8.4
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Upgrade of the joint full parameter configuration file
    * @param [in] path Full local upgrade package path (D://zUP/XXXXX.bin)
    * @return Error code 
    */
    int JointAllParamUpgrade(string path);

Example of upgrading code for robot from firmware
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.5  Web-3.8.4
    
.. code-block:: c#
    :linenos:

    private void button83_Click(object sender, EventArgs e)
    {
        robot.RobotEnable(0);
        Thread.Sleep(200);
        int rtn = robot.JointAllParamUpgrade("D://zUP/upgrade/jointallparameters.db");
        Console.WriteLine($"robot JointAllParamUpgrade rtn is{rtn}");
        rtn = robot.SetCtrlFirmwareUpgrade(2, "D://zUP/upgrade/FAIR_Cobot_Cbd_Asix_V2.0.bin");
        Console.WriteLine($"robot SetCtrlFirmwareUpgrade rtn is{rtn}");
        rtn = robot.SetEndFirmwareUpgrade(2, "D://zUP/upgrade/FAIR_Cobot_Axle_Asix_V2.4.bin");
        Console.WriteLine($"robot SetEndFirmwareUpgrade rtn is {rtn}");
        robot.SetSysServoBootMode();
        rtn = robot.SetCtrlFirmwareUpgrade(1, "D://zUP/upgrade/FR_CTRL_PRIMCU_FV201212_MAIN_U4_T01_20250428(MT).bin");
        Console.WriteLine($"robot SetCtrlFirmwareUpgrade rtn is{rtn}");
        rtn = robot.SetEndFirmwareUpgrade(1, "D://zUP/upgrade/FR_END_FV201009_MAIN_U1_T01_20250428.bin");
        Console.WriteLine($"robot SetEndFirmwareUpgrade rtn is {rtn}");
        rtn = robot.SetJointFirmwareUpgrade(1, "D://zUP/upgrade/FR_SERVO_FV504214_MAIN_U7_T07_20250519.bin");
        Console.WriteLine($"robot SetJointFirmwareUpgrade rtn is{rtn}");
    }

Robot MCU log generation
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Robot MCU log generation
    * @return Error code
    */
    public int RobotMCULogCollect();