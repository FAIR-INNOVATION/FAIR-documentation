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