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

Set digital, analog output code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    status = 1
    smooth = 0  
    block = 0 
    for i in range(16):
        robot.SetDO(i, status, smooth, block)
        time.sleep(0.3) 
    status = 0 
    for i in range(16):
        robot.SetDO(i, status, smooth, block)
        time.sleep(0.3)
    status = 1
    for i in range(2):
        robot.SetToolDO(i, status, smooth, block)
        time.sleep(1) 
    status = 0 
    for i in range(2):
        robot.SetToolDO(i, status, smooth, block)
        time.sleep(1)
    for i in range(100):
        robot.SetAO(0, i, block)
        time.sleep(0.03)
    for i in range(100):
        robot.SetToolAO(0, i, block)
        time.sleep(0.03)
    robot.CloseRPC()

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

Obtain the status of the button for recording the end point of the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAxlePointRecordBtnState()``"
    "Description", "Obtain the status of the button for recording the end point of the robot"
    "Mandatory parameters", "NULL"
    "Default Parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``buttonstatus``: Button status: 0- Press, 1- release"

Obtain the DO output status at the end of the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetToolDO()``"
    "Description", "Obtain the DO output status at the end of the robot"
    "Mandatory parameters", "NULL"
    "Default Parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``do_state``: DO output status: do0 to do1 correspond to bit1 to bit2, starting from bit0"

Obtain the DO output status of the robot controller
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetDO()``"
    "Description", "Obtain the DO output status of the robot controller"
    "Mandatory parameters", "NULL"
    "Default Parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``do_state_h``: DO output status: co0 to co7 correspond to bit0 to bit7. do_state_l DO output status: do0 to do7 correspond to bit0 to bit7"

Get the robot DI, DO status code examples
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    block = 0 
    error,di = robot.GetDI(0, block)
    print(f"di0: {di}")
    error,tool_di = robot.GetToolDI(1, block)
    print(f"tool_di1: {tool_di}")
    error,ai = robot.GetAI(0, block)
    print(f"ai0: {ai:.2f}") 
    error,tool_ai = robot.GetToolAI(0, block)
    print(f"tool_ai0: {tool_ai:.2f}")
    error,button_state = robot.GetAxlePointRecordBtnState()
    print(f"_button_state is: {button_state}")
    error,tool_do_state = robot.GetToolDO()
    print(f"tool DO state: {tool_do_state}")
    error,[do_state_h, do_state_l] = robot.GetDO()
    print(f"DO state hight  : {do_state_h}")
    print(f"DO state low : {do_state_l}")
    robot.CloseRPC()

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

Waiting control box digital, analog input signal code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    status = 1
    smooth = 0
    block = 0
    for i in range(16):
        robot.SetDO(i, status, smooth, block)
        time.sleep(0.3)
    status = 0
    for i in range(16):
        robot.SetDO(i, status, smooth, block)
        time.sleep(0.3)
    status = 1
    for i in range(2):
        robot.SetToolDO(i, status, smooth, block)
        time.sleep(1)
    status = 0
    for i in range(2):
        robot.SetToolDO(i, status, smooth, block)
        time.sleep(1)
    for i in range(100):
        robot.SetAO(0, i, block)
        time.sleep(0.03)
    for i in range(100):
        robot.SetToolAO(0, i, block)
        time.sleep(0.03)
    block = 0
    error,di = robot.GetDI(0, block)
    print(f"di0: {di}")
    error,tool_di = robot.GetToolDI(1, block)
    print(f"tool_di1: {tool_di}")
    error,ai = robot.GetAI(0, block)
    print(f"ai0: {ai:.2f}")
    error,tool_ai = robot.GetToolAI(0, block)
    print(f"tool_ai0: {tool_ai:.2f}")
    error,button_state = robot.GetAxlePointRecordBtnState()
    print(f"_button_state is: {button_state}")
    error,tool_do_state = robot.GetToolDO()
    print(f"tool DO state: {tool_do_state}")
    error,[do_state_h, do_state_l] = robot.GetDO()
    print(f"DO state hight  : {do_state_h}")
    print(f"DO state low : {do_state_l}")
    rtn = robot.WaitDI(0, 1, 1000, 1)
    print(f"WaitDI over; rtn is: {rtn}")
    rtn = robot.WaitMultiDI(1, 3, 3, 1000, 1)
    print(f"WaitDI over; rtn is: {rtn}")
    rtn = robot.WaitToolDI(1, 1, 1000, 1)
    print(f"WaitDI over; rtn is: {rtn}")
    rtn = robot.WaitAI(0, 0, 50, 1000, 1)
    print(f"WaitDI over; rtn is: {rtn}")
    rtn = robot.WaitToolAI(0, 0, 50, 1000, 1)
    print(f"WaitDI over; rtn is: {rtn}")
    robot.CloseRPC()

Set Whether Control Box DO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetCtlBoxDO(resetFlag,reloadFlag)``"
    "Description", "Set whether control box DO output resets after stop/pause"
    "Required Parameters", "
    - ``resetFlag``：0-Do not reset; 1-Reset
    - ``reloadFlag``：Whether to reload after pause resume, 0-Do not load; 1-Load"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Set Whether Control Box AO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetCtlBoxAO(resetFlag,reloadFlag)``"
    "Description", "Set whether control box AO output resets after stop/pause"
    "Required Parameters", "
    - ``resetFlag``：0-Do not reset; 1-Reset
    - ``reloadFlag``：Whether to reload after pause resume, 0-Do not load; 1-Load"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Set Whether End Tool DO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetAxleDO(resetFlag,reloadFlag)``"
    "Description", "Set whether end tool DO output resets after stop/pause"
    "Required Parameters", "
    - ``resetFlag``：0-Do not reset; 1-Reset
    - ``reloadFlag``：Whether to reload after pause resume, 0-Do not load; 1-Load"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Set Whether End Tool AO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetAxleAO(resetFlag,reloadFlag)``"
    "Description", "Set whether end tool AO output resets after stop/pause"
    "Required Parameters", "
    - ``resetFlag``：0-Do not reset; 1-Reset
    - ``reloadFlag``：Whether to reload after pause resume, 0-Do not load; 1-Load"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Set Whether Extended DO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetExtDO (resetFlag,reloadFlag)``"
    "Description", "Set whether extended DO output resets after stop/pause"
    "Required Parameters", "
    - ``resetFlag``：0-Do not reset; 1-Reset
    - ``reloadFlag``：Whether to reload after pause resume, 0-Do not load; 1-Load"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Set Whether Extended AO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetExtAO (resetFlag,reloadFlag)``"
    "Description", "Set whether extended AO output resets after stop/pause"
    "Required Parameters", "
    - ``resetFlag``：0-Do not reset; 1-Reset
    - ``reloadFlag``：Whether to reload after pause resume, 0-Do not load; 1-Load"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Set Whether SmartTool Output Resets After Stop/Pause
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetSmartToolDO(resetFlag,reloadFlag)``"
    "Description", "Set whether SmartTool output resets after stop/pause"
    "Required Parameters", "
    - ``resetFlag``：0-Do not reset; 1-Reset
    - ``reloadFlag``：Whether to reload after pause resume, 0-Do not load; 1-Load"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode "

Code Example for Setting Output Reset After Lua Program Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    robot = Robot.RPC('192.168.58.2')
    for i in range(16):
        robot.SetDO(i, 1, 0, 0)
        time.sleep(0.2)
    resetFlag = 0
    resumeReloadFlag = 0
    rtn = robot.SetOutputResetCtlBoxDO(resetFlag, resumeReloadFlag)
    robot.SetOutputResetCtlBoxAO(resetFlag, resumeReloadFlag)
    robot.SetOutputResetAxleDO(resetFlag, resumeReloadFlag)
    robot.SetOutputResetAxleAO(resetFlag, resumeReloadFlag)
    robot.SetOutputResetExtDO(resetFlag, resumeReloadFlag)
    robot.SetOutputResetExtAO(resetFlag, resumeReloadFlag)
    robot.SetOutputResetSmartToolDO(resetFlag, resumeReloadFlag)
    robot.ProgramLoad("/fruser/test.lua")
    robot.ProgramRun()
    time.sleep(2)
    robot.PauseMotion()
    time.sleep(2)
    robot.ResumeMotion()
    time.sleep(2)
    robot.CloseRPC()
    return 0