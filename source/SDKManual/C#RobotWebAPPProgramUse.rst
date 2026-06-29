The use of robot WebAPP programs
=======================================

.. toctree:: 
    :maxdepth: 5

Set the default job program to be automatically loaded upon startup
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief set the default job program to be automatically loaded upon startup
    * @param [in] flag 0- Default programs are not automatically loaded upon startup, 1- Default programs are automatically loaded upon startup
    * @param [in] program_name job program name and path, such as "movej.lua"
    * @return error code
    * /
    int LoadDefaultProgConfig(byte flag, string program_name);

Load the specified job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Load the specified job program
    * @param [in] program_name job program name and path, such as "movej.lua"
    * @return error code
    * /
    int ProgramLoad(string program_name);

Get the name of the loaded job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Get the name of the loaded job program
    * @param [out] program_name job program name and path, such as "movej.lua"
    * @return error code
    * /
    int GetLoadedProgram(ref string program_name);

Obtain the execution line number of the current robot operation program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief get the line number of the current robot job program being executed
    * @param [out] line number
    * @return error code
    * /
    int GetCurrentLine(ref int line);

Run the currently loaded job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief run the currently loaded job program
    * @return error code
    * /
    int ProgramRun();

Pause the currently running job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Suspend the currently running job program
    * @return error code
    * /
    int ProgramPause();

Resume the currently suspended job procedure
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Resume the currently suspended operation procedure
    * @return error code
    * /
    int ProgramResume();

Terminate the currently running job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Terminate the currently running job program
    * @return error code
    * /
    int ProgramStop();

Obtain the execution status of the robot's operation program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Get the execution status of the robot operation program
    * @param [out] state 1- The program stops or no program runs, 2- the program is running, 3- the program is paused
    * @return error code
    * /
    int GetProgramState(ref byte state);

Example of robot LUA program operation code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnWebApp_Click(object sender, EventArgs e)
    {
        string program_name = "Text1.lua";
        string loaded_name = "";
        byte state=0;
        int line=0;

        robot.Mode(0);
        robot.LoadDefaultProgConfig(0, program_name);
        robot.ProgramLoad(program_name);
        robot.ProgramRun();
        Thread.Sleep(1000);
        robot.ProgramPause();
        robot.GetProgramState(ref state);
        Console.WriteLine("program state:{0}\n", state);
        robot.GetCurrentLine(ref line);
        Console.WriteLine("current line:{0}\n", line);
        robot.GetLoadedProgram(ref loaded_name);
        Console.WriteLine("program name:{0}\n", loaded_name);
        Thread.Sleep(1000);
        robot.ProgramResume();
        Thread.Sleep(1000);
        robot.ProgramStop();
        Thread.Sleep(1000);
    }

Download the Lua file
+++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:

    / * *
    * @brief Download Lua files
    * @param [in] fileName The job program to be downloaded is "test.lua" or "test.tar.gz"
    * @param [in] savePath save the local path of the job program "D://Down/"
    * @return error code
    * /
    int LuaDownLoad(string fileName, string savePath);

Upload the Lua file
+++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:

    / * *
    * @brief Upload the Lua file
    * @param [in] filePath local job path name "... /test.lua" or "... /test.tar.gz"
    * @param [out] errStr error message
    * @return error code
    * /
    int LuaUpload(string filePath, ref string errStr);

Delete the Lua file
+++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:

    / * *
    * @brief Delete the Lua file
    * @param [in] fileName Name of the job program to be deleted "test.lua"
    * @return error code
    * /
    int LuaDelete(string fileName);

Get the names of all current lua files
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:

    / * *
    * @brief Get the names of all current lua files
    * @param [out] luaNames list of job program names
    * @return error code
    * /
    int GetLuaList(ref List<string> luaNames) ;

Code example for uploading and downloading robot LUA files
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.5

.. code-block:: c#
    :linenos:


    private void btnUploadLua_Click(object sender, EventArgs e)
    {
        int rtn;
        List<string> luaNames = new List<string>();
        rtn = robot.GetLuaList(ref luaNames);
        Console.WriteLine("res is: {0}", rtn);
        Console.WriteLine("size is: {0}", luaNames.Count);
        foreach (var name in luaNames)
        {
        Console.WriteLine(name);
        }
        rtn = robot.LuaDownLoad("TT.lua", "D://zDOWN/");
        Console.WriteLine("LuaDownLoad rtn is {0}", rtn);
        string errStr = "";
        Thread.Sleep(2000);

        rtn = robot.LuaUpload("D://zUP/airlab.lua", ref errStr);
        Console.WriteLine("LuaUpload rtn is {0}", errStr);
        Thread.Sleep(2000);
        rtn = robot.LuaDelete("TT.lua");
        Console.WriteLine("LuaDelete rtn is {0}", rtn);
    }