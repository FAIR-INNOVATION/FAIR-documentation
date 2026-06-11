WebAPP program use
======================

.. toctree::
    :maxdepth: 5

Setting the default job program to load automatically on boot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoadDefaultProgConfig(flag,program_name)``"
    "Description", "Sets the default job program to be automatically loaded on boot"
    "Required parameters","
    - ``flag``: 1-automatically load the default program on power-up, 0-don't automatically load the default program
    - ``program_name``: the name of the job program and its path, e.g. “movej.lua”"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Load the specified job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramLoad(program_name)``"
    "Description", "Load the specified job program."
    "Mandatory parameters", "- ``program_name``: name of the job program and path, e.g. “movej.lua”"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Get the name of the loaded job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetLoadedProgram()``"
    "Description", "Get the name of the loaded job program"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``program_name``: the name of the loaded operating program."

Get the line number of the current robot job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetCurrentLine()``"
    "Description", "Get the execution line number of the current robot job program"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``line_num``: the line number of the current robot job program execution"

Run the currently loaded job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramRun()``"
    "Description", "Run the currently loaded job program"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Suspend the currently running job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramPause()``"
    "Description", "Suspend the currently running job program."
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Resuming a currently suspended program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramResume()``"
    "Description", "Resume the currently suspended job program"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Terminate the currently running job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramStop()``"
    "Description", "Terminate the currently running job program."
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Obtaining robot job program execution status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetProgramState()``"
    "Description", "Get robot job program execution status"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``state``: state of execution of the robot's operating program, 1 - program stopped or no program running, 2 - program running, 3 - program suspended"

Robot LUA program operation code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    program_name = "test0610.lua"
    loaded_name = ""
    state = 0
    line = 0
    robot.Mode(0)
    robot.LoadDefaultProgConfig(0, program_name)
    robot.ProgramLoad(program_name)
    robot.ProgramRun()
    time.sleep(1)
    robot.ProgramPause()
    error,state = robot.GetProgramState()
    print(f"program state:{state}")
    error,line = robot.GetCurrentLine()
    print(f"current line:{line}")
    error,loaded_name = robot.GetLoadedProgram()
    print(f"program name:{loaded_name}")
    time.sleep(1)
    robot.ProgramResume()
    time.sleep(1)
    robot.ProgramStop()
    time.sleep(1)
    robot.CloseRPC()

Download Lua files
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``LuaDownLoad(fileName, savePath)``"
    "Description", "Download Lua File"
    "Mandatory parameter", "- ``fileName``: the name of the lua file to be downloaded, e.g. “test.lua”"
    - ``savePath``: the local path to save the file, e.g. “D://Down/”"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Deleting Lua files
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LuaDelete(fileName)``"
    "Description", "Deleting Lua files."
    "Required Parameters", "- ``fileName``: the name of the lua file to be deleted, “test.lua”"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Get the names of all current lua files
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetLuaList()``"
    "Description", "Get the names of all current lua files"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``lua_num``: number of lua files
    - ``luaNames``: list of lua file names"

Uploading Lua files
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LuaUpload(filePath)``"
    "Description", "Uploading a Lua file"
    "Mandatory parameter", "- ``filePath``: full path name of the uploaded file e.g. D://test/test.lua"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - errorStr(lua file exists error returned)"

Robot LUA file upload and download code examples
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    rtn,lua_num,luaNames = robot.GetLuaList()
    print(f"res is:{rtn}")
    print(f"size is:{lua_num}")
    for name in luaNames:
        print(name)
    rtn = robot.LuaDownLoad("test0610.lua", "D://zDOWN/")
    print(f"LuaDownLoad rtn is:{rtn}")
    rtn = robot.LuaUpload("D://zDOWN/test0610.lua")
    print(f"LuaUpload rtn is:{rtn}")
    rtn = robot.LuaDelete("test0610.lua")
    print(f"LuaDelete rtn is:{rtn}")
    robot.CloseRPC()