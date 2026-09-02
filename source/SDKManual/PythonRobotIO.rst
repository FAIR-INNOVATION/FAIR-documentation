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
    robot.ProgramLoad("test.lua")
    robot.ProgramRun()
    time.sleep(2)
    robot.PauseMotion()
    time.sleep(2)
    robot.ResumeMotion()
    time.sleep(2)
    robot.CloseRPC()
    return 0

Set Configurable CI Port Functions
+++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetDIConfig(config)``"
    "Description", "Set configurable CI port functions"
    "Required Parameters", "
    - ``config``: CI0-CI7 function code array, 0-None;1-Arc start success;2-Welder ready;3-Conveyor detection;4-Pause;5-Resume;6-Start;7-Stop;
      8-Pause/Resume;9-Start/Stop;10-Pedal drag;11-Move to home position;12-Manual/Auto switch;
      13-Wire search success;14-Motion interrupt;15-Start main program;16-Start rewind;17-Start confirmation;
      18-Photoelectric detection signal X;19-Photoelectric detection signal Y;20-External emergency stop input signal 1;21-External emergency stop input signal 2;
      22-Level 1 reduction mode;23-Level 2 reduction mode;24-Level 3 reduction mode (Stop);25-Resume welding;26-Terminate welding;
      27-Assist drag enable;28-Assist drag disable;29-Assist drag enable/disable;30-Clear all errors;
      31-Manual/Auto switch (high/low level);32-Enable;33-Disable;34-Enable/Disable (rising/falling edge);35-Fixed-point tracking start/end
      36-Enter safety speed motion;37-Current loop drag lock;38-Force sensor assisted lock"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Get Configurable CI Port Functions of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetDIConfig()``"
    "Description", "Get configurable CI port functions of the control box"
    "Required Parameters", "
    - ``config``: CI0-CI7 function code array, 0-None;1-Arc start success;2-Welder ready;3-Conveyor detection;4-Pause;5-Resume;6-Start;7-Stop;
      8-Pause/Resume;9-Start/Stop;10-Pedal drag;11-Move to home position;12-Manual/Auto switch;
      13-Wire search success;14-Motion interrupt;15-Start main program;16-Start rewind;17-Start confirmation;
      18-Photoelectric detection signal X;19-Photoelectric detection signal Y;20-External emergency stop input signal 1;21-External emergency stop input signal 2;
      22-Level 1 reduction mode;23-Level 2 reduction mode;24-Level 3 reduction mode (Stop);25-Resume welding;26-Terminate welding;
      27-Assist drag enable;28-Assist drag disable;29-Assist drag enable/disable;30-Clear all errors;
      31-Manual/Auto switch (high/low level);32-Enable;33-Disable;34-Enable/Disable (rising/falling edge);35-Fixed-point tracking start/end;
      36-Enter safety speed motion;37-Current loop drag lock;38-Force sensor assisted lock;
      201-External emergency stop input signal 1-dual channel; 202-External emergency stop input signal 2-dual channel; 203-Level 1 reduced mode-dual channel;
      204-Level 2 reduced mode-dual channel; 205-Level 3 reduced mode-dual channel; 206-Normal stop-dual channel; 207-Safety wall 1-dual channel; 208-Safety wall 2-dual channel;
      209-Safety wall 3-dual channel; 210-Safety wall 4-dual channel; 211-Safety wall 5-dual channel; 212-Safety wall 6-dual channel; 213-Safety wall 7-dual channel;
      214-Safety wall 8-dual channel; 215-Safety stop reset-dual channel;"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Set Configurable CO Port Functions
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetDOConfig(config)``"
    "Description", "Set configurable CO port functions"
    "Required Parameters", "
    - ``config``: CO0-CO7 function code array, 0-None;1-Robot error;2-Robot in motion;3-Spray start/stop;4-Spray gun cleaning;5-Gas supply signal;6-Arc start signal;7-Jog wire feed;
      8-Reverse wire feed;9-JOB input port 1;10-JOB input port 2;11-JOB input port 3;12-Conveyor start/stop control;13-Robot paused;14-Reached home position;
      15-Reached interference area;16-Wire search start/stop control;17-Robot start completed;18-Program start/stop;19-Auto/Manual mode;20-Emergency stop output signal 1-Safety;
      21-Emergency stop output signal 2-Safety;22-Lua script program running/stopped;23-Safety status output-Safety;24-Protective stop status output-Safety;
      25-Robot in motion-Safety;26-Robot reduced mode-Safety;27-Robot non-reduced mode-Safety;28-Robot non-stopped;29-Robot error-Instruction point error;
      30-Robot error-Driver error;31-Robot error-Soft limit exceeded error;32-Robot error-Collision error;33-Robot error-Active slave count error;
      34-Robot error-Slave error;35-Robot error-IO error;36-Robot error-Gripper error;37-Robot error-File error;38-Robot error-Singular pose error;
      39-Robot error-Driver communication error;40-Robot error-Parameter error;41-Robot error-External axis soft limit exceeded error;42-Robot warning-Warning;
      43-Robot warning-Safety door warning;44-Robot warning-Motion warning;45-Robot warning-Interference area warning;46-Robot warning-Safety wall warning;
      47-Enable status;48-Auto lift during disconnection;49-Cube 1 interference warning;50-Cube 2 interference warning;51-Cube 3 interference warning;52-Cube 4 interference warning;"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Get Configurable CO Port Functions
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetDOConfig()``"
    "Description", "Get configurable CO port functions"
    "Required Parameters", "
    - ``config``: CO0-CO7 function code array, 0-None;1-Robot error;2-Robot in motion;3-Spray start/stop;4-Spray gun cleaning;5-Gas supply signal;6-Arc start signal;7-Jog wire feed;
      8-Reverse wire feed;9-JOB input port 1;10-JOB input port 2;11-JOB input port 3;12-Conveyor start/stop control;13-Robot paused;14-Reached home position;
      15-Reached interference area;16-Wire search start/stop control;17-Robot start completed;18-Program start/stop;19-Auto/Manual mode;20-Emergency stop output signal 1-Safety;
      21-Emergency stop output signal 2-Safety;22-Lua script program running/stopped;23-Safety status output-Safety;24-Protective stop status output-Safety;
      25-Robot in motion-Safety;26-Robot reduced mode-Safety;27-Robot non-reduced mode-Safety;28-Robot non-stopped;29-Robot error-Instruction point error;
      30-Robot error-Driver error;31-Robot error-Soft limit exceeded error;32-Robot error-Collision error;33-Robot error-Active slave count error;
      34-Robot error-Slave error;35-Robot error-IO error;36-Robot error-Gripper error;37-Robot error-File error;38-Robot error-Singular pose error;
      39-Robot error-Driver communication error;40-Robot error-Parameter error;41-Robot error-External axis soft limit exceeded error;42-Robot warning-Warning;
      43-Robot warning-Safety door warning;44-Robot warning-Motion warning;45-Robot warning-Interference area warning;46-Robot warning-Safety wall warning;
      47-Enable status;48-Auto lift during disconnection;49-Cube 1 interference warning;50-Cube 2 interference warning;51-Cube 3 interference warning;52-Cube 4 interference warning;
      201-Emergency stop output signal 1-dual channel; 202-Emergency stop output signal 2-dual channel; 203-Safety status output-dual channel; 204-Protective stop status output-dual channel; 205-Robot in motion-dual channel;
	  206-Robot reduced mode-dual channel; 207-Robot non-reduced mode-dual channel;"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Set Configurable End-CI Port Functions of the End-Effector
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolDIConfig(config)``"
    "Description", "Set configurable End-CI port functions of the end-effector"
    "Required Parameters", "
    - ``config``: End CI0-CI1 function code array, 0-None;1-Drag teaching tool switch;2-Point recording signal;3-Manual/Auto switch (pulse signal);4-TPD recording start/stop;5-Pause motion;
      6-Resume motion;7-Start;8-Stop;9-Pause/Resume;10-Start/Stop;11-Force sensor assist drag enable;12-Force sensor assist drag disable;
      13-Force sensor assist drag enable/disable;14-Laser detection signal X;15-Laser detection signal Y;16-PTP motion to home position;17-Motion interrupt, stop current motion based on signal;
      18-Start main program;19-Start rewind;20-Start confirmation;21-Resume welding;22-Terminate welding;23-Clear error;24-Manual/Auto switch (high/low level);
      25-Enable;26-Disable;27-Enable/Disable;28-Laser servo tracking start/stop signal;"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Get Configurable End-CI Port Functions of the End-Effector
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetToolDIConfig()``"
    "Description", "Get configurable End-CI port functions of the end-effector"
    "Required Parameters", "
    - ``config``: End CI0-CI1 function code array, 0-None;1-Drag teaching tool switch;2-Point recording signal;3-Manual/Auto switch (pulse signal);4-TPD recording start/stop;5-Pause motion;
      6-Resume motion;7-Start;8-Stop;9-Pause/Resume;10-Start/Stop;11-Force sensor assist drag enable;12-Force sensor assist drag disable;
      13-Force sensor assist drag enable/disable;14-Laser detection signal X;15-Laser detection signal Y;16-PTP motion to home position;17-Motion interrupt, stop current motion based on signal;
      18-Start main program;19-Start rewind;20-Start confirmation;21-Resume welding;22-Terminate welding;23-Clear error;24-Manual/Auto switch (high/low level);
      25-Enable;26-Disable;27-Enable/Disable;28-Laser servo tracking start/stop signal;"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Set Configurable CI Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetDIConfigLevel(config)``"
    "Description", "Set configurable CI active state of the control box"
    "Required Parameters", "
    - ``config``: CI0-CI7 port active state array; 0-active high; 1-active low"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Get Configurable CI Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetDIConfigLevel()``"
    "Description", "Get configurable CI active state of the control box"
    "Required Parameters", "
    - ``config``: CI0-CI7 port active state array; 0-active high; 1-active low"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Set Configurable CO Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetDOConfigLevel(config)``"
    "Description", "Set configurable CO active state of the control box"
    "Required Parameters", "
    - ``config``: CO0-CO7 port active state array; 0-active high; 1-active low"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Get Configurable CO Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetDOConfigLevel()``"
    "Description", "Get configurable CO active state of the control box"
    "Required Parameters", "
    - ``config``: CO0-CO7 port active state array; 0-active high; 1-active low"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Set Configurable CI Active State of the End-Effector
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolDIConfigLevel(config)``"
    "Description", "Set configurable CI active state of the end-effector"
    "Required Parameters", "
    - ``config``: CI0-CI7 port active state array; 0-active high; 1-active low"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Get Configurable CI Active State of the End-Effector
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetToolDIConfigLevel()``"
    "Description", "Get configurable CI active state of the end-effector"
    "Required Parameters", "
    - ``config``: CI0-CI7 port active state array; 0-active high; 1-active low"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Set Standard DI Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetStandardDILevel(config)``"
    "Description", "Set standard DI active state of the control box"
    "Required Parameters", "
    - ``config``: DI0-DI7 port active state array; 0-active high; 1-active low"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Get Standard DI Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetStandardDILevel()``"
    "Description", "Get standard DI active state of the control box"
    "Required Parameters", "
    - ``config``: DI0-DI7 port active state array; 0-active high; 1-active low"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Set Standard DO Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetStandardDOLevel(config)``"
    "Description", "Set standard DO active state of the control box"
    "Required Parameters", "
    - ``config``: DO0-DO7 port active state array; 0-active high; 1-active low"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Get Standard DO Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetStandardDOLevel()``"
    "Description", "Get standard DO active state of the control box"
    "Required Parameters", "
    - ``config``: DO0-DO7 port active state array; 0-active high; 1-active low"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
IO Configuration Related SDK Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from time import sleep
    import time
    from fairino import Robot

    # Establish connection with robot controller
    robot = Robot.RPC('192.168.58.2')


    def TestIOConfig(self):
        # Set and get DI configuration
        setDIConfig = [1, 2, 3, 4, 5, 6, 7, 8]
        getDIConfig = [0] * 8
        rtn = robot.SetDIConfig(setDIConfig)
        print(f"SetDIConfig rtn is {rtn}")
        rtn, getDIConfig = robot.GetDIConfig()
        print(f"GetDIConfig rtn is {rtn}, value is {getDIConfig[0]} {getDIConfig[1]} {getDIConfig[2]} {getDIConfig[3]} {getDIConfig[4]} {getDIConfig[5]} {getDIConfig[6]} {getDIConfig[7]}")

        # Set and get DO configuration
        setDOConfig = [9, 10, 11, 12, 13, 14, 15, 16]
        getDOConfig = [0] * 8
        rtn = robot.SetDOConfig(setDOConfig)
        print(f"SetDOConfig rtn is {rtn}")
        rtn, getDOConfig = robot.GetDOConfig()
        print(f"GetDOConfig rtn is {rtn}, value is {getDOConfig[0]} {getDOConfig[1]} {getDOConfig[2]} {getDOConfig[3]} {getDOConfig[4]} {getDOConfig[5]} {getDOConfig[6]} {getDOConfig[7]}")

        # Set and get tool DI configuration
        setToolDIConfig = [17, 18]
        getToolDIConfig = [0] * 2
        rtn = robot.SetToolDIConfig(setToolDIConfig)
        print(f"SetToolDIConfig rtn is {rtn}")
        rtn, getToolDIConfig = robot.GetToolDIConfig()
        print(f"GetToolDIConfig rtn is {rtn}, value is {getToolDIConfig[0]} {getToolDIConfig[1]}")

        # Set and get DI level configuration (0: active low, 1: active high)
        setDIConfigLevel = [1, 1, 1, 1, 0, 0, 0, 0]
        getDIConfigLevel = [0] * 8
        rtn = robot.SetDIConfigLevel(setDIConfigLevel)
        print(f"SetDIConfigLevel rtn is {rtn}")
        rtn, getDIConfigLevel = robot.GetDIConfigLevel()
        print(f"GetDIConfigLevel rtn is {rtn}, value is {getDIConfigLevel[0]} {getDIConfigLevel[1]} {getDIConfigLevel[2]} {getDIConfigLevel[3]} {getDIConfigLevel[4]} {getDIConfigLevel[5]} {getDIConfigLevel[6]} {getDIConfigLevel[7]}")

        # Set and get DO level configuration (0: active low, 1: active high)
        setDOConfigLevel = [0, 0, 0, 0, 1, 1, 1, 1]
        getDOConfigLevel = [0] * 8
        rtn = robot.SetDOConfigLevel(setDOConfigLevel)
        print(f"SetDOConfigLevel rtn is {rtn}")
        rtn, getDOConfigLevel = robot.GetDOConfigLevel()
        print(f"GetDOConfigLevel rtn is {rtn}, value is {getDOConfigLevel[0]} {getDOConfigLevel[1]} {getDOConfigLevel[2]} {getDOConfigLevel[3]} {getDOConfigLevel[4]} {getDOConfigLevel[5]} {getDOConfigLevel[6]} {getDOConfigLevel[7]}")

        # Set and get tool DI level configuration
        setToolDIConfigLevel = [1, 0]
        getToolDIConfigLevel = [0] * 2
        rtn = robot.SetToolDIConfigLevel(setToolDIConfigLevel)
        print(f"SetToolDIConfigLevel rtn is {rtn}")
        rtn, getToolDIConfigLevel = robot.GetToolDIConfigLevel()
        print(f"GetToolDIConfigLevel rtn is {rtn}, value is {getToolDIConfigLevel[0]} {getToolDIConfigLevel[1]}")

        # Set and get standard DI level configuration
        setStandardDILevel = [1, 1, 1, 1, 0, 0, 0, 0]
        getStandardDILevel = [0] * 8
        rtn = robot.SetStandardDILevel(setStandardDILevel)
        print(f"SetStandardDILevel rtn is {rtn}")
        rtn, getStandardDILevel = robot.GetStandardDILevel()
        print(f"GetStandardDILevel rtn is {rtn}, value is {getStandardDILevel[0]} {getStandardDILevel[1]} {getStandardDILevel[2]} {getStandardDILevel[3]} {getStandardDILevel[4]} {getStandardDILevel[5]} {getStandardDILevel[6]} {getStandardDILevel[7]}")

        # Set and get standard DO level configuration
        setStandardDOLevel = [0, 0, 0, 0, 1, 1, 1, 1]
        getStandardDOLevel = [0] * 8
        rtn = robot.SetStandardDOLevel(setStandardDOLevel)
        print(f"SetStandardDOLevel rtn is {rtn}")
        rtn, getStandardDOLevel = robot.GetStandardDOLevel()
        print(f"GetStandsrdDOLevel rtn is {rtn}, value is {getStandardDOLevel[0]} {getStandardDOLevel[1]} {getStandardDOLevel[2]} {getStandardDOLevel[3]} {getStandardDOLevel[4]} {getStandardDOLevel[5]} {getStandardDOLevel[6]} {getStandardDOLevel[7]}")

        # Wait 2 seconds
        time.sleep(2)

        # Close connection
        robot.CloseRPC()
        time.sleep(1)

    # Call test function
    TestIOConfig(robot)