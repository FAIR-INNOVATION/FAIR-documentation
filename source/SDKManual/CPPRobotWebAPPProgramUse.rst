WebAPP program use
================================

.. toctree:: 
    :maxdepth: 5

Set Default Program to Load Automatically on Startup
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set default program to load automatically on startup
    * @param  [in] flag  0-Do not load default program on startup, 1-Load default program on startup
    * @param  [in] program_name Program name and path, e.g. "/fruser/movej.lua", where "/fruser/" is the fixed path
    * @return  Error code
    */
    errno_t  LoadDefaultProgConfig(uint8_t flag, char program_name[64]);

Load Specified Program
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Load specified program
    * @param  [in] program_name Program name and path, e.g. "/fruser/movej.lua", where "/fruser/" is the fixed path
    * @return  Error code
    */
    errno_t  ProgramLoad(char program_name[64]);

Get Loaded Program Name
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get loaded program name
    * @param  [out] program_name Program name and path, e.g. "/fruser/movej.lua", where "/fruser/" is the fixed path
    * @return  Error code
    */
    errno_t  GetLoadedProgram(char program_name[64]);  

Get Current Program Execution Line Number
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get current program execution line number
    * @param  [out] line  Line number
    * @return  Error code
    */   
    errno_t  GetCurrentLine(int *line);

Run Currently Loaded Program
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Run currently loaded program
    * @return  Error code
    */
    errno_t  ProgramRun();

Pause Currently Running Program
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Pause currently running program
    * @return  Error code
    */ 
    errno_t  ProgramPause();

Resume Currently Paused Program
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Resume currently paused program
    * @return  Error code
    */ 
    errno_t  ProgramResume();  

Stop Currently Running Program
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Stop currently running program
    * @return  Error code
    */ 
    errno_t  ProgramStop();    

Get Program Execution State
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get program execution state
    * @param  [out]  state 1-Program stopped or no program running, 2-Program running, 3-Program paused
    * @return  Error code
    */
    errno_t  GetProgramState(uint8_t *state);

Robot LUA Program Operation Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestLuaOp(void)
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
      char program_name[64] = "/fruser/test.lua";
      char loaded_name[64] = "";
      uint8_t state;
      int line;
      robot.Mode(0);
      robot.LoadDefaultProgConfig(0, program_name);
      robot.ProgramLoad(program_name);
      robot.ProgramRun();
      robot.Sleep(1000);
      robot.ProgramPause();
      robot.GetProgramState(&state);
      printf("program state:%u\n", state);
      robot.GetCurrentLine(&line);
      printf("current line:%d\n", line);
      robot.GetLoadedProgram(loaded_name);
      printf("program name:%s\n", loaded_name);
      robot.Sleep(1000);
      robot.ProgramResume();
      robot.Sleep(1000);
      robot.ProgramStop();
      robot.Sleep(1000);
      robot.CloseRPC();
      return 0;
    }

Download Lua File
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Download Lua file
    * @param [in] fileName Lua file name to download, e.g. "test.lua"
    * @param [in] savePath Local path to save file, e.g. "D://Down/"
    * @return Error code
    */
    errno_t LuaDownLoad(std::string fileName, std::string savePath);

Delete Lua File
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Delete Lua file
    * @param [in] fileName Lua file name to delete, e.g. "test.lua"
    * @return Error code
    */
    errno_t LuaDelete(std::string fileName);

Get All Current Lua File Names
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get all current Lua file names
    * @param [out] luaNames List of Lua file names
    * @return Error code
    */
    errno_t GetLuaList(std::list<std::string>* luaNames);

Upload Lua File
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Upload Lua file
    * @param [in] filePath Local Lua file path name
    * @return Error code
    */
    errno_t LuaUpload(std::string filePath);

Robot LUA File Upload/Download Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestLUAUpDownLoad(void)
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
      list<std::string> luaNames;
      rtn = robot.GetLuaList(&luaNames);
      std::cout << "res is: " << rtn << std::endl;
      std::cout << "size is: " << luaNames.size() << std::endl;
      for (auto it = luaNames.begin(); it != luaNames.end(); it++)
      {
        std::cout << it->c_str() << std::endl;
      }
      rtn = robot.LuaDownLoad("test.lua", "D://zDOWN/");
      printf("LuaDownLoad rtn is %d\n", rtn);
      rtn = robot.LuaUpload("D://zUP/airlab.lua");
      printf("LuaUpload rtn is %d\n", rtn);
      rtn = robot.LuaDelete("test.lua");
      printf("LuaDelete rtn is %d\n", rtn);
      robot.CloseRPC();
      return 0;
    }