WebAPP program use
==============================

.. toctree:: 
    :maxdepth: 5

Set up and automatically load the default operating program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoadDefaultProgConfig(flag,program_name)``"
    "Description", "Set up and automatically load the default operating program"
    "Required parameter", "- ``flag``:1-automatically load the default program upon startup, 0-do not automatically load the default program
    - ``program_name``:The name and path of the homework program, such as “/fraser/movej.lua”, where “/fraser/” is a fixed path"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Load the specified job program
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramLoad(program_name)``"
    "Description", "Load the specified job program"
    "Required parameter", "- ``program_name``:The name and path of the homework program, such as “/fraser/movej.lua”, where “/fraser/” is a fixed path"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Obtain the execution line number of the current robot job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetCurrentLine()``"
    "Description", "Obtain the execution line number of the current robot job program"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success):line_num "

Run the currently loaded job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramRun()``"
    "Description", "Run the currently loaded job program"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Pause the currently running job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramPause()``"
    "Description", "Pause the currently running job program"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Resume the currently paused job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramResume()``"
    "Description", "Resume the currently paused job program"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Terminate the currently running job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramStop()``"
    "Description", "Terminate the currently running job program"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Obtain the execution status of robot job programs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetProgramState()``"
    "Description", "Obtain the execution status of robot job programs"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode
    - Return(if success):state: 1-program stopped or no program running, 2-program running, 3-program paused "

Obtain the name of the loaded job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetLoadedProgram()``"
    "Description", "Obtain the name of the loaded job program"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): program_name "

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    def print_program_state():
        pstate = robot.GetProgramState()    # Obtain the execution status of robot job programs
        linenum = robot.GetCurrentLine()    # Obtain the execution line number of the current robot job program
        name = robot.GetLoadedProgram()     # Obtain the name of the loaded job program
        print("the robot program state is:",pstate[1])
        print("the robot program line number is:",linenum[1])
        print("the robot program name is:",name[1])
        time.sleep(1)
    robot.Mode(0)   
    print_program_state()
    ret = robot.ProgramLoad('/fruser/test0923.lua')   # Load the specified job program
    print("Load the specified job program ", ret)
    ret = robot.ProgramRun()     # Run the currently loaded job program
    print("Run the currently loaded job program ", ret)
    time.sleep(2)
    print_program_state()
    ret = robot.ProgramPause()   # Pause the currently running job program
    print("Pause the currently running job program ", ret)
    time.sleep(2)
    print_program_state()
    ret = robot.ProgramResume()  # Resume the currently paused job program
    print("Resume the currently paused job program ", ret)
    time.sleep(2)
    print_program_state()
    ret = robot.ProgramStop()    # Terminate the currently running job program
    print("Terminate the currently running job program ", ret)
    time.sleep(2)
    print_program_state()

Download the Lua file
+++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``LuaDownLoad(fileName, savePath)``"
    "Description", "Download the Lua file"
    "Required parameter", "- ``fileName``: The name of the lua file to be downloaded, for example, “test.lua”
    - ``savePath``:The local path for saving the file, for example, “D://Down/”"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    robot.LuaDownLoad("test", "D://Desktop/test_download/")

Upload the Lua file
+++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LuaUpload(filePath)``"
    "Description", "Upload the Lua file"
    "Required parameter", "- ``filePath``:The full path name of the uploaded file, for example, D://test/test.lua"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - errorStr(The lua file has an error return)"

Code example
------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    robot.LuaUpload("D://test/test.lua")

Delete the Lua file
+++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LuaDelete(fileName)``"
    "Description", "Delete the Lua file"
    "Required parameter", "- ``fileName``: The name of the lua file to be deleted, for example, “test.lua”"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetSoftwareVersion()
    robot.LuaDelete("test2")

Obtain Name of all current lua files
+++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetLuaList()``"
    "Description", "Obtain Name of all current lua files"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret,num,name  = robot.GetLuaList()
    print(num)
    print(name)