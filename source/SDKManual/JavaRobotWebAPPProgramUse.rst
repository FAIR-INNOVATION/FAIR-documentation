Robot WebAPP program use
=====================================

.. toctree:: 
    :maxdepth: 5

Setting the default job program to load automatically on boot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the default job program to load automatically at boot time
    * @param [in] flag 0-power-on does not automatically load the default program, 1-power-on automatically loads the default program
    * @param [in] program_name Name and path of the job program, e.g. "/fruser/movej.lua", where "/fruser/" is a fixed path.
    * @return error code
    */
    int LoadDefaultProgConfig(int flag, String program_name); 

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.LoadDefaultProgConfig(1,"/fruser/1010Test.lua");
    }

Load the specified job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief loads the specified operational program
    * @param [in] program_name Name and path of the job program, e.g. "/fruser/movej.lua", where "/fruser/" is a fixed path.
    * @return error code
    */
    int ProgramLoad(String program_name). 

Get the name of the loaded job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get the name of the loaded job program
    * @param [out] program_name program_name[0]: name and path of the program, e.g. "/fruser/movej.lua", where "/fruser/" is a fixed path.
    * @return error code
    */
    int GetLoadedProgram(String[] program_name). 

Get the line number of the current robot job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get the line number of the current robot job program execution.
    * @param [out] List[0]:error code; List[1]:int line line number
    * @return error code
    */  
    List<Integer> GetCurrentLine();

Run the currently loaded job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Run the currently loaded job program
    * @return error code
    */
    int ProgramRun();

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.Mode(0);
        robot.ProgramLoad("/fruser/1010Test.lua");
        String[] loadedNameStr = new String[1];
        robot.GetLoadedProgram(loadedNameStr);
        System.out.println("Loaded lua Name is " + loadedNameStr[0]);
        robot.ProgramRun();
        while(true)
        {
            List<Integer> results = robot.GetCurrentLine();
            ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
            System.out.println("current line is " + results.get(1) + " Robot Running State: " + pkg.robot_state);
            robot.Sleep(500);
        }
    }

pause-for-pause campaign
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Suspend the currently running operational program
    * @return error code
    */ 
    int PauseMotion().

Resumption of movement
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Resumption of currently suspended operating procedures
    * @return error code
    */ 
    int ResumeMotion(). 

Terminate the currently running job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief terminate the currently running job program
    * @return error code
    */ 
    int StopMotion().   

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection successful");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.Mode(0);
        robot.ProgramLoad("/fruser/1010Test.lua");
        String[] loadedNameStr = new String[1];
        robot.GetLoadedProgram(loadedNameStr);
        System.out.println("Loaded lua Name is " + loadedNameStr[0]);
        robot.ProgramRun();

        for(int i = 0; i < 10; i++)
        {
            robot.PauseMotion();//pause motion
            robot.Sleep(1000);
            robot.ResumeMotion();//resume motion
            robot.Sleep(1000);
        }
        robot.StopMotion();//Stop
    }

Download Lua Programs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Downloading the operating procedures
    * @param [in] fileName The name of the lua file to be downloaded, "test.lua" or "test.tar.gz".
    * @param [in] savePath Save the local path of the file "D:///Down/".
    * @return error code 
    */
    int LuaDownLoad(String fileName, String savePath); int LuaDownLoad(String fileName, String savePath)

Uploading a Lua program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Uploading of operational procedures
    * @param [in] filePath Path name of local lua file "... /test.lua" or "... /test.tar.gz"
    * @param [out] errStr error message
    * @return error code 
    */
    int LuaUpload(String filePath, String errStr);

Deleting Lua Programs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Deletion of operational procedures
    * @param [in] fileName The name of the job program to be deleted, "test.lua".
    * @return error code 
    */
    int LuaDelete(String fileName).

Get the names of all current job programs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    *  @brief gets the names of all current operating procedures
    * @param [out] luaNames list of job program names
    * @return error code 
    */
    int GetLuaList(List<String> luaNames).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.LuaDownLoad("1010TestLUA.lua", "D://LUA/");
        List<String> names = new ArrayList<String>();
        robot.GetLuaList(names);
        System.out.println("lua Num " + names.size() + " " + names.get(0));
        String errStr = "";
        robot.LuaUpload("D://LUA/1010TestLUA.lua", errStr);
        System.out.println("robot upload 1010TestLUA lua result " + errStr);
        robot.LuaDelete("1010TestLUA.lua");
    }
