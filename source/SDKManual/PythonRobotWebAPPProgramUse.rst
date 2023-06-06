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
    "Parameter", "- ``flag``：1-automatically load the default program upon startup, 0-do not automatically load the default program
    - ``program_name``：The name and path of the homework program, such as “/fraser/movej.lua”, where “/fraser/” is a fixed path"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.LoadDefaultProgConfig(1, "/fruser/splineptp.lua")  # Set the default job program to automatically load upon start up

Load the specified job program
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramLoad(program_name)``"
    "Description", "Load the specified job program"
    "Parameter", "- ``program_name``：The name and path of the homework program, such as “/fraser/movej.lua”, where “/fraser/” is a fixed path"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    #The robot webapp program uses the interface
    robot.Mode(0)   #The robot goes into automatic operation mode
    robot.ProgramLoad('/fruser/testPTP.lua')   #To load the robot program to execute, the testPTP.lua program needs to be written on webapp first

Obtain the execution line number of the current robot job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetCurrentLine()``"
    "Description", "Obtain the execution line number of the current robot job program"
    "Parameter", "Nothing"
    "Return value", "- Success：[0,line_num]
    - Failed：[errcode]"

Run the currently loaded job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramRun()``"
    "Description", "Run the currently loaded job program"
    "Parameter", "Nothing"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Pause the currently running job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramPause()``"
    "Description", "Pause the currently running job program"
    "Parameter", "Nothing"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Resume the currently paused job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramResume()``"
    "Description", "Resume the currently paused job program"
    "Parameter", "Nothing"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Terminate the currently running job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramStop()``"
    "Description", "Terminate the currently running job program"
    "Parameter", "Nothing"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Obtain the execution status of robot job programs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetProgramState()``"
    "Description", "Obtain the execution status of robot job programs"
    "Parameter", "Nothing"
    "Return value", "- Success：[0,state],state:1-program stopped or no program running, 2-program running, 3-program paused
    - Failed：[errcode]"

Obtain the name of the loaded job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetLoadedProgram()``"
    "Description", "Obtain the name of the loaded job program"
    "Parameter", "Nothing"
    "Return value", "- Success：[0,program_name]
    - Failed：[errcode]"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6-8

    import frrpc
    import time
    import _thread
    def print_program_state(name,rb):
        while(1):
            pstate = robot.GetProgramState()    #Query program running status,1-program stopped or Nothing program running, 2-program running, 3-program suspended
            linenum = robot.GetCurrentLine()    #Query the line number of the current job program
            name = robot.GetLoadedProgram()     #Queries the name of the loaded job program
            print("the robot program state is:",pstate[1])
            print("the robot program line number is:",linenum[1])
            print("the robot program name is:",name[1])
            time.sleep(1)
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    #The robot webapp program uses the interface
    robot.Mode(0)   #The robot entered automatic operation mode
    robot.ProgramLoad('/fruser/testPTP.lua')   #To load the robot program to execute, the testPTP.lua program needs to be written on webapp first
    robot.ProgramRun()     #Executive robot program
    _thread.start_new_thread(print_program_state,("print_state",robot))
    time.sleep(5)         #10s rest
    robot.ProgramPause()   #Pause the robot program in progress
    time.sleep(5)
    robot.ProgramResume()  #Resume the suspended robot program
    time.sleep(5)
    robot.ProgramStop()    #Stop the robot program in progress
    time.sleep(2)