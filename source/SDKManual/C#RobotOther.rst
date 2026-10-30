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
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

Set Robot to Stop Running When Port Communication is Disconnected
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:
    
    /**
    * @brief Set robot to stop running when port communication is disconnected
    * @param [in] portID Port number 0-8080; 1-8083; 2-20002; 3-20004
    * @param [in] enable 0-disable; 1-enable
    * @param [in] confirmTime Communication disconnection confirmation duration (ms)[0-5000]
    * @return  Error code
    */
    public int SetRobotStopOnComDisc(int portID, bool enable, int confirmTime)

Get Robot Stop on Communication Disconnection Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:
    
    /**
    * @brief Get robot stop on communication disconnection parameters
    * @param [in] portID Port number 0-8080; 1-8083; 2-20002; 3-20004
    * @param [out] enable 0-disable; 1-enable
    * @param [out] confirmTime Communication disconnection confirmation duration (ms)[0-5000]
    * @return  Error code
    */
    public int GetRobotStopOnComDisc(int portID, ref bool enable, ref int confirmTime)
    
Robot Stop on Communication Disconnection Parameter Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:
    
    void TestRobotStopOnComDisc()
    {
        int rtn = 0;

        // Set parameters for four ports
        rtn = robot.SetRobotStopOnComDisc(0, true, 330);
        rtn = robot.SetRobotStopOnComDisc(1, true, 550);
        rtn = robot.SetRobotStopOnComDisc(2, true, 110);
        rtn = robot.SetRobotStopOnComDisc(3, true, 220);
        Console.WriteLine($"SetRobotStopOnComDisc {rtn}");

        bool enable = false;
        int confirmTime = 0;

        // Get and print settings for each port
        robot.GetRobotStopOnComDisc(0, ref enable, ref confirmTime);
        Console.WriteLine($"GetRobotStopOnComDisc 8080 rtn {rtn}; enable is {(enable ? 1 : 0)}; confirm time is {confirmTime}");

        robot.GetRobotStopOnComDisc(1, ref enable, ref confirmTime);
        Console.WriteLine($"GetRobotStopOnComDisc 8083 rtn {rtn}; enable is {(enable ? 1 : 0)}; confirm time is {confirmTime}");

        robot.GetRobotStopOnComDisc(2, ref enable, ref confirmTime);
        Console.WriteLine($"GetRobotStopOnComDisc 20002 rtn {rtn}; enable is {(enable ? 1 : 0)}; confirm time is {confirmTime}");

        robot.GetRobotStopOnComDisc(3, ref enable, ref confirmTime);
        Console.WriteLine($"GetRobotStopOnComDisc 20004 rtn {rtn}; enable is {(enable ? 1 : 0)}; confirm time is {confirmTime}");

    }

Send UDP Instruction Frame
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Send UDP instruction frame
    * @param [in] Instruction frame
    * @return Error code
    */
    public int SendUDPFrame(string frame)
        
UDP Communication-Based SDK Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    void TestRobotUDP()
    {
        robot.OnUdpFrameReceived += (comType, frameCount, frameCmdID, contentLen, content) =>
        {
            Console.WriteLine($"[UDP Response] comType={comType}, count={frameCount}, cmdID={frameCmdID}, content={content}");
        };


        //Send frame
        string frameToSend = "/f/bIII52III236III7IIIMode(1)III/b/f";
        robot.SendUDPFrame(frameToSend);
        Thread.Sleep(2000);
        frameToSend = "/f/bIII52III236III7IIIMode(0)III/b/f";
        robot.SendUDPFrame(frameToSend);
        Thread.Sleep(2000);
        frameToSend = "/f/bIII41III201III153IIIMoveJ(53.857,-89.441,119.453,-22.664,61.059,3.369,-54.249,-491.930,375.396,96.474,-6.896,-7.783,0,0,100,100,100,0.000,0.000,0.000,0.000,-1,0,0,0,0,0,0,0)III/b/f";
        robot.SendUDPFrame(frameToSend);
        Thread.Sleep(2000);
        frameToSend = "/f/bIII42III203III163IIIMoveL(81.736,-85.284,114.974,-23.261,88.746,6.799,125.744,-506.570,375.396,96.474,-6.896,-7.783,0,0,100,100,100,-1,0,0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0,100,0)III/b/f";
        robot.SendUDPFrame(frameToSend);
        Thread.Sleep(2000);
        frameToSend = "/f/bIII47III400III15IIIGetMCVersion(1)III/b/f/f/bIII48III424III21IIIGetSlaveFirmVersion()III/b/f";
        robot.SendUDPFrame(frameToSend);
        Thread.Sleep(2000);

    }
        
Set User-Defined Robot End-Effector LED Color
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Set user-defined robot end-effector LED color
    * @param [in] r End red LED control; 0-off; 1-on
    * @param [in] g End green LED control; 0-off; 1-on
    * @param [in] b End blue LED control; 0-off; 1-on
    * @return Error code
    */
    public int SetUserLEDColor(bool r, bool g, bool b)
            
SDK Code Example for Setting User-Defined Robot End-Effector LED Color
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    public void testled()
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

Send Custom Command Frame via TCP 8080
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Send a custom command frame via TCP 8080 (automatically through the encrypted channel in mTLS mode)
    * @param frame Complete command frame, e.g. "/f/bIII52III236III7IIIMode(0)III/b/f"
    * @return Error code
    */
    public int SendTCPFrame(string frame)

Code Example for Sending a Custom Command Frame via TCP 8080    
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    public void TestServoJTcp()
    {
        Func<double[], string> fmtArr = a =>
        {
            string[] s = new string[a.Length];
            for (int i = 0; i < a.Length; i++)
                s[i] = a[i].ToString("F3");
            return string.Join(",", s);
        };

        int tcpRcvCnt = 0;
        robot.OnTcpFrameReceived += frame =>
        {
            tcpRcvCnt++;
            Console.WriteLine($"[TCP rpy #{tcpRcvCnt}] {DateTime.Now:HH:mm:ss.fff} {frame}");
        };

        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();

        float vel = 0.0f;
        float acc = 0.0f;
        float cmdT = 0.008f;
        float filterT = 0.0f;
        float gain = 0.0f;
        byte flag = 0;
        int count = 500;
        float dt = 0.1f;
        int cmdID = 0;
        int frameCount = 0;   

        while (true)
        {
            JointPos j = new JointPos(0, -90, 90, 0, 0, 0);
            ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
            DescPose offset_pos = new DescPose(0, -90, 90, 0, 0, 0);
            robot.MoveJ(j, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
            int ret = robot.GetActualJointPosDegree(flag, ref j);
            if (ret == 0)
            {
                count = 500;
                cmdID += 1;

                {
                    string s = "ServoMoveStart()";
                    FRAME f = new FRAME
                    {
                        count = frameCount++,
                        cmdID = 689,
                        content = s,
                        contentLen = s.Length,
                        head = "/f/b",
                        tail = "/b/f"
                    };
                    robot.SendTCPFrame(FrameHandle.PackFrame(f));
                }

                while (count > 0)
                {
                    {
                        string jointStr = fmtArr(j.jPos);
                        string axisStr = fmtArr(epos.ePos);
                        string s = $"ServoJ({jointStr},{axisStr},{acc:F3},{vel:F3},{cmdT:F3},{filterT:F3},{gain:F3},{cmdID})";
                        FRAME f = new FRAME
                        {
                            count = frameCount++,
                            cmdID = 376,
                            content = s,
                            contentLen = s.Length,
                            head = "/f/b",
                            tail = "/b/f"
                        };
                        robot.SendTCPFrame(FrameHandle.PackFrame(f));
                    }

                    j.jPos[0] += dt;
                    j.jPos[3] += dt;
                    j.jPos[4] += dt;
                    j.jPos[5] += dt;
                    epos.ePos[0] += dt;
                    count -= 1;
                    Thread.Sleep(1);
                    robot.GetRobotRealTimeState(ref pkg);
                    Console.WriteLine($"Servoj command count: {pkg.servoJCmdNum}");
                    Console.WriteLine($"Servoj Count {pkg.servoJCmdNum}; last pos is {pkg.lastServoTarget[0]} {pkg.lastServoTarget[1]} {pkg.lastServoTarget[2]} {pkg.lastServoTarget[3]} {pkg.lastServoTarget[4]} {pkg.lastServoTarget[5]}");
                }

                {
                    string s = "ServoMoveEnd()";
                    FRAME f = new FRAME
                    {
                        count = frameCount++,
                        cmdID = 690,
                        content = s,
                        contentLen = s.Length,
                        head = "/f/b",
                        tail = "/b/f"
                    };
                    robot.SendTCPFrame(FrameHandle.PackFrame(f));
                }

                Thread.Sleep(1000);
            }
        }
    }    

Data Backup Package Upload Upgrade  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Data backup package upload upgrade
    * @param [in] filePath Upload data backup package file path; D://zUP/fr_user_data.tar.gz
    * @return Error code
    */
    public int DataPackageUpgrade(string filePath)
    
Get Data Backup Package Upgrade Progress Percentage  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Get data backup package upgrade progress percentage
    * @param [out] progress Upgrade progress percentage [0-100], -1: upgrade failed
    * @return Error code
    */
    public int GetDataPackageUpgradeProgress(ref int progress)
    
Get Joint All Parameters Upgrade Progress Percentage  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Get joint all parameters upgrade progress percentage
    * @param [out] progress Upgrade progress percentage [0-100], -1: upgrade failed
    * @return Error code
    */
    public int GetJointAllParamUpgradeProgress(ref int progress)
        
Code Example for Data Backup Package Upload Upgrade and Querying Upgrade Progress  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    public void TestDataPackageUpgrade()
    {
        int rtn = robot.DataPackageUpgrade("D://zUP/upgrade/FR5/fr_user_data.tar.gz");
        Console.WriteLine($"robot DataPackageUpgrade rtn is {rtn}");

        int upgradeProgress = 0;
        while (upgradeProgress < 100)
        {
            int ret = robot.GetDataPackageUpgradeProgress(ref upgradeProgress);
            Console.WriteLine($"robot data package upgrade progress : {upgradeProgress}, rtn : {ret}");
            if (ret != 0 || upgradeProgress == -1)
            {
                break;
            }
            Thread.Sleep(200);
        }
    }
            
Code Example for Joint All Parameters Upgrade and Querying Upgrade Progress  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:
    
    public void TestJointAllParamUpgrade()
    {
        robot.RobotEnable(0);
        Thread.Sleep(200);
        int rtn = robot.JointAllParamUpgrade("D://zUP/upgrade/FR5/jointallparameters.db");
        Console.WriteLine($"robot JointAllParamUpgrade rtn is {rtn}");

        int upgradeProgress = 0;
        while (upgradeProgress < 100)
        {
            int ret = robot.GetJointAllParamUpgradeProgress(ref upgradeProgress);
            Console.WriteLine($"robot joint all param upgrade progress : {upgradeProgress}, rtn : {ret}");
            if (ret != 0 || upgradeProgress == -1)
            {
                break;
            }
            Thread.Sleep(200);
        }
    }    