Robot WebAPP application use
===================================
.. toctree:: 
    :maxdepth: 5

Setting the default job program to load automatically on boot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the default job program to load automatically on boot up
    * @param [in] flag 0-boot-up does not automatically load the default program, 1-boot-up automatically loads the default program
    * @param [in] program_name Name and path of the job programme, e.g. "/fruser/movej.lua", where "/fruser/" is a fixed path.
    * @return error code
    */
    int LoadDefaultProgConfig(byte flag, string program_name); 

Load the specified job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief loads a specified programme of work
    * @param [in] program_name Name and path of the job programme, e.g. "/fruser/movej.lua", where "/fruser/" is a fixed path.
    * @return error code
    */
    int ProgramLoad(string program_name). 

Get the name of the loaded job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief Get the name of the loaded job programme
    * @param [out] program_name Name and path of the job programme, e.g. "/fruser/movej.lua", where "/fruser/" is a fixed path.
    * @return error code
    */
    int GetLoadedProgram(ref string program_name). 

Get the line number of the current robot job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the line number of the current robot job program execution.
    * @param [out] line line number
    * @return error code
    */  
    int GetCurrentLine(ref int line).

Run the currently loaded job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief Run the currently loaded job program
    * @return error code
    */
    int ProgramRun();

Suspend the currently running job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief Suspend the currently running operational programme
    * @return error code
    */ 
    int ProgramPause();

Resumption of the currently suspended operating procedure
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief Resumption of currently suspended operating procedures
    * @return error code
    */ 
    int ProgramResume(); 

Terminate the currently running job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief terminate the currently running job programme
    * @return error code
    */ 
    int ProgramStop();   

Getting the robot's job program execution status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief Obtaining robot programme execution status
    * @param [out] state 1-program stopped or no program running, 2-program running, 3-program suspended
    * @return error code
    */
    int GetProgramState(ref byte state).

code example
++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnWebApp_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        string program_name = "/fruser/testWebApp.lua";
        string loaded_name = "";
        byte state = 0;
        int line = 0;

        robot.Mode(0);
        robot.ProgramLoad(program_name);
        robot.ProgramRun();
        Thread.Sleep(2000);
        robot.ProgramPause();
        robot.GetProgramState(ref state);
        Console.WriteLine($"program state : {state}");
        robot.GetCurrentLine(ref line);
        Console.WriteLine($"current line : {line}");
        robot.GetLoadedProgram(ref loaded_name);
        Console.WriteLine($"program name : {loaded_name}");
        Thread.Sleep(1000);
        robot.ProgramResume();
        Thread.Sleep(1000);
        robot.ProgramStop();
    }

Downloading of operating procedures
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Downloading the work programme
    * @param [in] fileName The "test.lua" or "test.tar.gz" file to download.
    * @param [in] savePath Save the local path of the job programme "D:///Down/".
    * @return error code 
    */
    public int LuaDownLoad(string fileName, string savePath);

Uploading of operational procedures
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Uploading of operational procedures
    * @param [in] filePath Local job program path name "... /test.lua" or "... /test.tar.gz"
    * @param [out] errStr error message
    * @return error code 
    */
    public int LuaUpload(string filePath, ref string errStr);

Deletion of operational procedures
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Deletion of operating procedures
    * @param [in] fileName The name of the job programme to be deleted, "test.lua".
    * @return error code 
    */
    public int LuaDelete(string fileName).

Get the names of all current job programs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:

    /** 
    * :: @brief gets the names of all the current operating procedures
    * @param [out] luaNames list of job program names
    * @return error code 
    */
    public int GetLuaList(ref List<string> luaNames) ;


code example
++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:

    private void btnUploadLua_Click(object sender, EventArgs e)
    {
        string errstr = "";
        robot.LuaUpload("D://Upload/test.lua", ref errstr);
        Console.WriteLine(errstr);
        robot.LuaDownLoad("test.lua", "D://zDOWN/");
        robot.LuaDelete("test.lua");
        List<string> lualist = new List<string>();
        robot.GetLuaList(ref lualist);
        int n = lualist.Count;
        for (int i = 0; i < n; i++)
        {
            Console.WriteLine(lualist[i]);
        }
    }