IO
============

.. toctree::
    :maxdepth: 5

Setting the control box digital output
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetDO(id, status, smooth=0, block=0)``"
    "Description", "Setting the control box digital outputs"
    "Mandatory parameters", "- ``id``: io number, range [0~15];
    - ``status``: 0 - off, 1 - on;"
    "Default Parameters", "- ``smooth``: 0-not smooth, 1-smooth Default 0;
    - ``block``:0-blocking, 1-non-blocking Default 0"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Test control box DO
    for i in range(0,16):
        error = robot.SetDO(i,1) #open the control box DO
    time.sleep(1)
    for i in range(0,16):
        robot.SetDO(i,0) #Close the control box DO

Setting Tool Digital Outputs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetToolDO (id, status, smooth=0, block=0)``"
    "Description", "Setting the digital output of the tool"
    "Mandatory parameters", "- ``id``: io number, range [0~1];
    - ``status``: 0 - off, 1 - on;"
    "Default Parameters", "- ``smooth``: 0-not smooth, 1-smooth;
    - ``block``: 0-blocking, 1-non-blocking."
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Test tool DO
    error_tooldo = 0
    for i in range(0,2):
        error = robot.SetToolDO(i,1) #Turn on tool DOs
    robot.WaitMs(1000)
    for i in range(0,2):
        error = robot.SetToolDO(i,0) #Turn off tool DOs


Setting the control box analog output
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAO(id,value,block=0)``"
    "Description", "Setting the control box analog output"
    "Mandatory parameters", "- ``id``: io number, range [0~1];
    - ``value``: percentage of current or voltage value in the range [0 to 100%] corresponding to current value [0 to 20 mA] or voltage [0 to 10 V];"
    "Default parameters", "- ``block``:[0]-blocking, [1]-non-blocking Default 0"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Test control box AO
    error = robot.SetAO(0,100.0)
    print("Setting AO0 error code:", error)
    error = robot.SetAO(1,100.0)
    print("Setting AO1 error code:", error)

Setting Tool Analog Outputs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolAO(id,value,block=0)``"
    "Description", "Setup Tool Analog Output"
    "Mandatory parameters", "- ``id``: io number, range [0];
    - ``value``: percentage of current or voltage value in the range [0 to 100%] corresponding to current value [0 to 20 mA] or voltage [0 to 10 V];"
    "Default parameters", "- ``block``:[0]-blocking, [1]-non-blocking Default 0"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Test end AO
    error = robot.SetToolAO(0,100.0)
    print("Setting ToolAO0 error code:", error)
    Robot.WaitMs(1000)
    error = robot.SetToolAO(0,0.0)
    print("Setting ToolAO0 error code:", error)

Getting control box digital inputs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetDI(id, block=0)``"
    "Description", "Get control box digital inputs"
    "Mandatory parameters", "- ``id``: io number, range [0~15];"
    "Default Parameters", "- ``block``: 0-blocking, 1-non-blocking Default 0"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``di``: 0-low level, 1-high level"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.GetDI(0,0)
    print("Getting DI0",error)

Get Tool Digital Inputs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetToolDI(id, block=0)``"
    "Description", "Get Tool Digital Inputs"
    "Mandatory parameters", "- ``id``: io number, range [0~1];"
    "Default Parameters", "- ``block``: 0-blocking, 1-non-blocking Default 0"
    "Return Value", "Error Code Success-0 Failure- errcode
    - ``di``: 0 - low level, 1 - high level"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    tool_di = robot.GetToolDI(1,0)
    print("Get ToolDI",tool_di)

Waiting for control box digital inputs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``WaitDI(id,status,maxtime,opt)``"
    "Description", "Waiting for control box digital input"
    "Mandatory parameters", "- ``id``: io number, range [0~15];
    - ``status``: 0-off, 1-on;
    - ``maxtime``: maximum waiting time in [ms];
    - ``opt``: post timeout policy, 0-program stops and prompts for timeout, 1-ignore timeout prompts program to continue execution, 2-always wait"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    max_waittime = 2000
    # Waiting for the control box DI
    error = robot.WaitDI(0,1,max_waittime,0)
    print("WaitDI error code",error)

Waiting for control box with multiple digital inputs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``WaitMultiDI(mode,id,status,maxtime,opt)``"
    "Description", "Waiting for control box with multiple digital inputs"
    "Mandatory parameters", "- ``mode``: [0]-multiple with, [1]-multiple or;
    - ``id``: io number, bit0~bit7 correspond to DI0~DI7, bit8~bit15 correspond to CI0~CI7;
    - ``status``: bit0~bit7 corresponds to DI0~DI7 status, bit8~bit15 corresponds to the status of CI0~CI7 status bits [0]-off, [1]-on;
    - ``maxtime``: maximum waiting time in [ms];
    - ``opt``: post-timeout policy, 0-program stops and prompts for timeout, 1-ignores timeout prompting program to continue execution, 2-always waits."
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    max_waittime = 2000
    #Waiting for control box multiplexed DI
    error = robot.WaitMultiDI(1,3,1,max_waittime,0)
    print("WaitMultiDI error code",error)

Waiting for tool digital inputs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``WaitToolDI(id,status,maxtime,opt)``"
    "Description", "Waiting for end digital input"
    "Mandatory parameters", "- ``id``: io number, range [0~1];
    - ``status``: 0-off, 1-on;
    - ``maxtime``: maximum waiting time in [ms];
    - ``opt``: post timeout policy, 0-program stops and prompts for timeout, 1-ignore timeout prompts program to continue execution, 2-always wait"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    max_waittime = 2000
    #Waiting for the tool DI
    error = robot.WaitToolDI(1,1,max_waittime,0)
    print("WaitToolDI error code",error)

Getting Control Box Analog Inputs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAI(id, block = 0)``"
    "Description", "Get control box analog inputs"
    "Mandatory parameters", "- ``id``: io number, range [0~1];"
    "Default Parameters","- ``block``:0-blocking, 1-non-blocking Default 0 "
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``value``: Percentage of input current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]."

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.GetAI(0)
    print("Getting AI0",error)

Get Tool Analog Inputs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetToolAI (id, block = 0)``"
    "Description", "Get end analog input"
    "Mandatory parameters", "- ``id``: io number, range [0];"
    "Default Parameters", "- ``block``: 0-blocking, 1-non-blocking Default 0"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``value``: Percentage of input current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]."

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.GetToolAI(0)
    print("Getting ToolAI0",error)

Waiting for control box analog inputs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``WaitAI(id,sign,value,maxtime,opt)``"
    "Description", "Waiting for control box analog input"
    "Mandatory parameters", "- ``id``: io number, range [0~1];
    - ``sign``: 0 - greater than, 1 - less than
    - ``value``: percentage of input current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V];
    - ``maxtime``: maximum waiting time in [ms];
    - ``opt``: post timeout policy, 0-program stops and prompts for timeout, 1-ignore timeout prompts program to continue execution, 2-always wait"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    max_waittime = 2000
    # Waiting for the control box AI
    error = robot.WaitAI(0,0,50,max_waittime,1) #Ignore the timeout prompts for the program to continue execution
    print("WaitAI error code",error)

Waiting for tool analog inputs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``WaitToolAI(id,sign,value,maxtime,opt)``"
    "Description", "Waiting for end analog input"
    "Mandatory parameters", "- ``id``: io number, range [0];
    - ``sign``: 0 - greater than, 1 - less than
    - ``value``: percentage of input current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V];
    - ``maxtime``: maximum waiting time in [ms];
    - ``opt``: post timeout policy, 0-program stops and prompts for timeout, 1-ignore timeout prompts program to continue execution, 2-always wait"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    max_waittime = 2000
    # Waiting for the tool AI
    error = robot.WaitToolAI(0,0,50,max_waittime,0)
    print("WaitToolAI error code",error)

Setting whether the output is reset after the control box DO stop/pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetCtlBoxDO(resetFlag)``"
    "Description", "Sets whether or not the output is reset after a control box DO stop/pause"
    "Mandatory parameters", "- ``resetFlag``: 0-no reset; 1-reset"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')
    time.sleep(5)
    error = robot.SetDO(1,1)
    print("SetDO 1 return:",error)

    error = robot.SetDO(3,1)
    print("SetDO 3 return:",error)

    error = robot.SetToolDO(1,1)
    print("SetToolDO return:",error)

    error = robot.SetAO(0,25)
    print("SetAO 0 return:",error)

    error = robot.SetAO(1,87)
    print("SetAO 1 return:",error)

    error = robot.SetToolAO(0,54)
    print("SetToolAO return:",error)

    error = robot.SetOutputResetCtlBoxDO(1)
    print("SetOutputResetCtlBoxDO return:",error)

    error = robot.SetOutputResetCtlBoxAO(1)
    print("SetOutputResetCtlBoxAO return:",error)

    error = robot.SetOutputResetAxleDO(1)
    print("SetOutputResetCtlBoxDO return:",error)

    error = robot.SetOutputResetAxleAO(1)
    print("SetOutputResetCtlBoxAO return:",error)

    error = robot.ProgramRun()
    print("ProgramRun return:",error)
    time.sleep(3)
    error = robot.ProgramStop()
    print("ProgramPause return:",error)

    time.sleep(5)

    error = robot.SetDO(1,1)
    print("SetDO 1 return:",error)

    error = robot.SetDO(3,1)
    print("SetDO 3 return:",error)

    error = robot.SetToolDO(1,1)
    print("SetToolDO return:",error)

    error = robot.SetAO(0,25)
    print("SetAO 0 return:",error)

    error = robot.SetAO(1,87)
    print("SetAO 1 return:",error)

    error = robot.SetToolAO(0,54)
    print("SetToolAO return:",error)
    error = robot.SetOutputResetCtlBoxDO(0)
    print("SetOutputResetCtlBoxDO return:",error)

    error = robot.SetOutputResetCtlBoxAO(0)
    print("SetOutputResetCtlBoxAO return:",error)

    error = robot.SetOutputResetAxleDO(0)
    print("SetOutputResetCtlBoxDO return:",error)

    error = robot.SetOutputResetAxleAO(0)
    print("SetOutputResetCtlBoxAO return:",error)

    error = robot.ProgramRun()
    print("ProgramRun return:",error)
    time.sleep(3)
    error = robot.ProgramStop()
    print("ProgramPause return:",error)

Setting whether the output is reset after the control box AO stop/pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetCtlBoxDO(resetFlag)``"
    "Description", "Sets whether the outputs are reset after a control box AO stop/pause"
    "Mandatory parameter", "- ``resetFlag``: 0-no reset; 1-reset"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Sets whether the output is reset after the end tool DO stops/pause.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetAxleDO(resetFlag)``"
    "Description", "Sets whether the output is reset after the end tool DO stops/pauses"
    "Mandatory parameter", "- ``resetFlag``: 0 - no reset; 1 - reset"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Set whether the output is reset after the end tool AO stops/pauses
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetAxleAO(resetFlag)``"
    "Description", "Sets whether the output is reset after the end tool AO stops/pauses"
    "Mandatory parameter", "- ``resetFlag``: 0-no reset; 1-reset"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Sets whether the outputs are reset after an extended DO stop/pause.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetOutputResetExtDO (resetFlag)``"
    "Description", "Sets whether the output is reset after an extended DO stop/pause"
    "Mandatory parameter", "- ``resetFlag``: 0-no reset; 1-reset"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    error = robot.SetAuxDO(1,True,False,False)
    print("SetAuxDO 1 return:",error)

    error = robot.SetAuxDO(3,True,False,False)
    print("SetAuxDO 3 return:",error)

    error = robot.SetAuxAO(0,10,False)
    print("SetAuxAO 0 return:",error)

    error = robot.SetAuxAO(1,87,False)
    print("SetAuxAO 1 return:",error)

    error = robot.SetOutputResetExtDO(1)
    print("SetOutputResetExtDO return:",error)

    error = robot.SetOutputResetExtAO(1)
    print("SetOutputResetExtAO return:",error)

    error = robot.ProgramRun()
    print("ProgramRun return:",error)
    time.sleep(3)
    error = robot.ProgramStop()
    print("ProgramPause return:",error)

    time.sleep(3)
    error = robot.SetAuxDO(1,True,False,False)
    print("SetAuxDO 1 return:",error)

    error = robot.SetAuxDO(3,True,False,False)
    print("SetAuxDO 3 return:",error)

    error = robot.SetAuxAO(0,10,False)
    print("SetAuxAO 0 return:",error)

    error = robot.SetAuxAO(1,87,False)
    print("SetAuxAO 1 return:",error)

    error = robot.SetOutputResetExtDO(0)
    print("SetOutputResetExtDO return:",error)

    error = robot.SetOutputResetExtAO(0)
    print("SetOutputResetExtAO return:",error)

    error = robot.ProgramRun()
    print("ProgramRun return:",error)
    time.sleep(3)
    error = robot.ProgramStop()
    print("ProgramPause return:",error)

Sets whether the output is reset after the expansion AO stops/pause.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetExtAO (resetFlag)``"
    "Description", "Sets whether the output is reset after an extended AO stop/pause"
    "Mandatory parameter", "- ``resetFlag``: 0-no reset; 1-reset"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"