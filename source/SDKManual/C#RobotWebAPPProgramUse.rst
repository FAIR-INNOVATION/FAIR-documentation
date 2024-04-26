WebAPP program use
================================

.. toctree:: 
    :maxdepth: 5

Set the default job program to be automatically loaded upon startup
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the default job program to be automatically loaded upon startup
    * @param  [in] flag  0- boot does not automatically load the default program, 1- boot automatically load the default program
    * @param  [in] program_name Job program name and path, for example, /fruser/movej.lua, where /fruser/ is a fixed path
    * @return  Error code
    */
    int LoadDefaultProgConfig(byte flag, string program_name); 

Load the specified job program
+++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Load the specified job program
    * @param  [in] program_name Job program name and path, for example, /fruser/movej.lua, where /fruser/ is a fixed path
    * @return  Error code
    */
    int ProgramLoad(string program_name);

Get the loaded job program name
+++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the loaded job program name
    * @param  [out] program_name Job program name and path, for example, /fruser/movej.lua, where /fruser/ is a fixed path
    * @return  Error code
    */
    int GetLoadedProgram(ref string program_name); 

Get the line number of the current robot job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the line number of the current robot job program
    * @param  [out] line  line number
    * @return  Error code
    */   
    int GetCurrentLine(ref int line);

Run the currently loaded job program
+++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Run the currently loaded job program
    * @return  Error code
    */
    int ProgramRun();

Pause the current running job program
++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Pause the current running job program
    * @return  Error code
    */ 
    int ProgramPause();

Resume the currently suspended job program
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Resume the currently suspended job program
    * @return  Error code
    */ 
    int ProgramResume();  

Terminates the currently running job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Terminates the currently running job program
    * @return  Error code
    */ 
    int ProgramStop();   

Get the robot job program execution state
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the robot job program execution state
    * @param  [out]  state 1- program stop or no program running, 2- program running, 3- program pause
    * @return  Error code
    */
    int GetProgramState(ref byte state);

Code example
++++++++++++++
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

Download job program
+++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.5

.. code-block:: c#
    :linenos:

    /** 
    * @brief Download job program
    * @param [in] fileName Job program to download "test.lua" or "test.tar.gz"
    * @param [in] savePath Save the local path of the job program“D://Down/”
    * @return Error Code 
    */
    public int LuaDownLoad(string fileName, string savePath);

Upload job program
+++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.5

.. code-block:: c#
    :linenos:

    /** 
    * @brief Upload job program
    * @param [in] filePath Local job program path name ".../test.lua"或".../test.tar.gz"
    * @param [out] errStr Error message
    * @return  
    */
    public int LuaUpload(string filePath, ref string errStr);

Delete job program
+++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.5

.. code-block:: c#
    :linenos:

    /** 
    * @brief Delete job program
    * @param [in] fileName The name of the job program to delete "test.lua"
    * @return Error Code
    */
    public int LuaDelete(string fileName);

Gets all current job program names
+++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.5

.. code-block:: c#
    :linenos:

    /** 
    * @brief Gets all current job program names
    * @param [out] luaNames List of job program names
    * @return Error Code
    */
    public int GetLuaList(ref List<string> luaNames) ;

Code example
++++++++++++++

.. versionadded:: C# SDK-v1.0.5

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