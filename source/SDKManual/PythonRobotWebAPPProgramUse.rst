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
    "Required parameters","- ``flag``: 1-automatically load the default program on power-up, 0-don't automatically load the default program
    - ``program_name``: the name of the job program and its path, e.g. “/fruser/movej.lua”, where “/fruser/” is the fixed path."
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Load the specified job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramLoad(program_name)``"
    "Description", "Load the specified job program."
    "Mandatory parameters", "- ``program_name``: name of the job program and path, e.g. “/fruser/movej.lua”, where “/fruser/” is a fixed path"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

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

code example
----------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    def print_program_state():
        pstate = robot.GetProgramState() #query program running state,1-program stopped or no program running,2-program running,3-program suspended
        linenum = robot.GetCurrentLine() #Query the line number of the current job program execution
        name = robot.GetLoadedProgram() #query the name of the loaded program
        print("the robot program state is:",pstate[1])
        print("the robot program line number is:",linenum[1])
        print("the robot program name is:",name[1])
        time.sleep(1)
    Interface for #robot webapp programs
    robot.Mode(0) #Robot cuts to autorun mode
    print_program_state()
    ret = robot.ProgramLoad('/fruser/test0923.lua') #Load the robot program to be executed, the testPTP.lua program needs to be written first on the webapp
    print("Error code loading robot program to be executed", ret)
    ret = robot.ProgramRun() #Execute the robot program
    print("Execute robot program error code", ret)
    time.sleep(2)
    print_program_state()
    ret = robot.ProgramPause() #pause the executing robot program
    print("Error code to pause the executing robot program", ret)
    time.sleep(2)
    print_program_state()
    ret = robot.ProgramResume() # Resume the suspended robot program.
    print("Resume suspended robot program error code", ret)
    time.sleep(2)
    print_program_state()
    ret = robot.ProgramStop() #stop the executing robot program
    print("Stop the executing robot program", ret)
    time.sleep(2)
    print_program_state()
    flag = 1 #0-boot-up does not automatically load the default program, 1-boot-up automatically loads the default program
    ret = robot.LoadDefaultProgConfig(flag,'/fruser/testPTP.lua') #Set the default program to load automatically on power-up
    print("Setting the default program to load automatically on boot", ret)

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

code example
----------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.LuaDownLoad("test", "D://Desktop/test_download/")

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

code example
----------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.LuaUpload("D://test/test.lua")

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

code example
----------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetSoftwareVersion()
    robot.LuaDelete("test2")

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

code example
----------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret,num,name = robot.GetLuaList()
    print(num)
    print(name)
    
Documentation of teaching points
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SavePoint(name, update_allprogramfile=0)``"
    "Description", "Documentation of demonstration points"
    "Mandatory parameters", "- ``name``: name of the demonstration site"
    "Default Parameters", "- ``update_allprogramfile``: whether to overwrite, 0-no overwrite, 1-overwrite, default 0"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
----------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.SavePoint("test1")