Robot WebAPP program usage  
==============================

.. toctree::  
    :maxdepth: 5  

Set default program to load automatically on startup  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Set default program to load automatically on startup  
    * @param  [in] flag  0-Do not load default program automatically, 1-Load default program automatically  
    * @param  [in] program_name Program name and path, e.g., "/fruser/movej.lua" ("/fruser/" is the fixed path for QX, and "/usr/local/etc/controller/lua/" is the fixed path for LA)  
    * @return  Error code  
    */  
    int LoadDefaultProgConfig(int flag, String program_name);  

Load specified program  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Load specified program  
    * @param  [in] program_name Program name and path, e.g., "/fruser/movej.lua" ("/fruser/" is the fixed path for QX, and "/usr/local/etc/controller/lua/" is the fixed path for LA)  
    * @return  Error code  
    */  
    int ProgramLoad(String program_name);  

Get loaded program name  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get loaded program name  
    * @param  [out] program_name program_name[0]: Program name and path, e.g., "/fruser/movej.lua" ("/fruser/" is the fixed path for QX, and "/usr/local/etc/controller/lua/" is the fixed path for LA)  
    * @return  Error code  
    */  
    int GetLoadedProgram(String[] program_name);  

Get current program execution line number  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get current program execution line number  
    * @param  [out] List[0]: Error code; List[1]: int line - Line number  
    * @return  Error code  
    */  
    List<Integer> GetCurrentLine();  

Run currently loaded program  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Run currently loaded program  
    * @return  Error code  
    */  
    int ProgramRun();  

Pause current running program  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Pause current running program  
    * @return  Error code  
    */  
    int PauseMotion();  

Resume paused program  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Resume paused program  
    * @return  Error code  
    */  
    int ResumeMotion();  

Stop current running program  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Stop current running program  
    * @return  Error code  
    */  
    int StopMotion();  

Get robot program execution state  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get robot program execution state  
    * @param   [out] state 1-Program stopped/no program running, 2-Program running, 3-Program paused  
    * @return  Error code  
    */  
    public int GetProgramState(int[] state)  

Robot LUA program operation code example  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestLuaOp(Robot robot)  
    {  
        String program_name = "/fruser/Text1.lua";  
        String[] loaded_name = new String[]{""};  
        int[] state=new int[]{0};  
        List<Integer> line=new ArrayList<>();  

        robot.Mode(0);  
        robot.LoadDefaultProgConfig(0, program_name);  
        robot.ProgramLoad(program_name);  
        robot.ProgramRun();  
        robot.Sleep(1000);  
        robot.ProgramPause();  
        robot.GetProgramState(state);  
        System.out.println("program state:"+ state[0]);  
        line=robot.GetCurrentLine();  
        System.out.println("current line:"+ line);  
        robot.GetLoadedProgram(loaded_name);  
        System.out.println("program name:"+ loaded_name[0]);  
        robot.Sleep(1000);  
        robot.ProgramResume();  
        robot.Sleep(1000);  
        robot.ProgramStop();  
        robot.Sleep(1000);  

        robot.CloseRPC();  
        return 0;  
    }  

Download Lua program  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Download program  
    * @param [in] fileName Lua file name to download ("test.lua" or "test.tar.gz")  
    * @param [in] savePath Local save path ("D://Down/")  
    * @return Error code  
    */  
    int LuaDownLoad(String fileName, String savePath);  

Delete Lua program  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Delete program  
    * @param [in] fileName Program name to delete ("test.lua")  
    * @return Error code  
    */  
    int LuaDelete(String fileName);  

Get all current Lua file names  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get all current Lua file names  
    * @param [out] luaNames Program name list  
    * @return Error code  
    */  
    int GetLuaList(List<String> luaNames);  

Upload Lua program  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Upload program  
    * @param [in] filePath Local Lua file path (".../test.lua" or ".../test.tar.gz")  
    * @param [out] errStr Error message  
    * @return Error code  
    */  
    int LuaUpload(String filePath, String errStr);  

Robot LUA file upload/download code example  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestLUAUpDownLoad(Robot robot)  
    {  
        List<String> luaNames=new ArrayList<>();  
        int rtn = robot.GetLuaList(luaNames);  
        System.out.println("res is: "+rtn);  
        System.out.println("size is: "+luaNames.size());  
        for (int it =1; it < luaNames.size(); it++)  
        {  
            System.out.println(luaNames.get(it));  
        }  

        rtn = robot.LuaDownLoad("test.lua", "D://zDOWN/");  
        System.out.println("LuaDownLoad rtn is:"+rtn);  

        rtn = robot.LuaUpload("D://zUP/XG.lua","");  
        System.out.println("LuaUpload rtn is:"+ rtn);  

        rtn = robot.LuaDelete("XG.lua");  
        System.out.println("LuaDelete rtn is:"+ rtn);  

        return 0;  
    }  