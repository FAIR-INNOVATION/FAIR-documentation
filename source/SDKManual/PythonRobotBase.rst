Robotics Basics
=======================================

.. toctree::
    :maxdepth: 5

Instantiated Robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``RPC(ip)``"
    "Description", "Instantiating a robot object"
    "Required parameter", "- ``ip``:The IP address of the robot, with a default factory IP of “192.168.58.2”"
    "Optional parameter", "NULL"
    "Return value", "- Success: Returns a robot object
    - Failed: The created object will be destroyed"
     
code example
------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

Close the RPC connection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``CloseRPC()``"
    "Description", "Close RPC connection"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "NULL"
     
code example
------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.CloseRPC()

Query SDK version number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetSDKVersion()``"
    "Description", "Query SDK version number"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Fail-errcode
    - ``sdk``: SDK version number, controller version number"

code example
---------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret,version = robot.GetSDKVersion() #query the SDK version number
    if ret == 0.
        print("SDK version number is ", version )
    else.
        print("Query failed with error code ",ret)

Get controller IP
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetControllerIP()``"
    "Description", "Query Controller IP"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``ip``: controller IP"

code example
--------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret,ip = robot.GetControllerIP() #query controller IP
    if ret == 0.
        print("Controller IP is ", ip)
    else.
        print("Query failed with error code ",ret)

Control of robot hand-automatic mode switching
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``Mode(state)``"
    "description", "control robot hand auto mode switching"
    "Mandatory parameters", "- ``state``: 0 - automatic mode, 1 - manual mode"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
--------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    #Robot hand-automatic mode switching
    ret = robot.Mode(0) #Robot cuts to autorun mode
    print("Robot cut into autorun mode", ret)
    time.sleep(1)
    ret = robot.Mode(1) #Robot cut to manual mode
    print("Robot cut to manual mode", ret)

Robot Drag Mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Controlling the robot into and out of drag-and-drop instructor mode
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``DragTeachSwitch(state)``"
    "Description", "Controls the robot into and out of drag-and-drop demonstration mode."
    "Mandatory parameter", "- ``state``: 1-entry into drag-indicator mode, 0-exit from drag-indicator mode"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Queries whether the robot is in drag mode
----------------------------------------------------------------------------------------------------------------------------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``IsInDragTeach()``"
    "Description", "Queries whether the robot is in drag-and-drop demonstration mode."
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``state``: 0 - non-drag instructional mode, 1 - drag instructional mode"

code example
--------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    #Robot hand-automatic mode switching
    ret = robot.Mode(0) #Robot cuts to autorun mode
    print("Robot cut into autorun mode", ret)
    time.sleep(1)
    ret = robot.Mode(1) #Robot cuts to manual mode
    print("Robot cut to manual mode", ret)

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Robot enters or exits drag-and-drop instructional mode
    ret = robot.Mode(1) #Robot cut to manual mode
    print("Robot cut to manual mode", ret)
    time.sleep(1)
    ret = robot.DragTeachSwitch(1) #Robot cuts to drag teach mode, must be in manual mode to cut to drag teach mode
    print("Robot cut to drag-and-drop instructional mode", ret)
    time.sleep(1)
    ret,state = robot.IsInDragTeach() #query if in drag teach mode, 1-drag teach mode, 0-non-drag teach mode
    if ret == 0.
        print("Current drag mode state:", state)
    else.
        print("Query failed with error code:",ret)
    time.sleep(3)
    ret = robot.DragTeachSwitch(0) # robot cuts to non-drag teach mode, must be in manual mode to cut to non-drag teach mode
    print("Robot cut to non-drag instructional mode", ret)
    time.sleep(1)
    ret,state = robot.IsInDragTeach() #query if in drag teach mode, 1-drag teach mode, 0-non-drag teach mode
    if ret == 0.
        print("Current drag mode state:", state)
    else.
        print("Query failed with error code:",ret)

Control robot up-enable or down-enable
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``RobotEnable(state)``"
    "Description", "Control robot up-enable or down-enable"
    "Mandatory parameters", "- ``state``: 1 - up enable, 0 - down enable"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
--------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Robot up-enable or down-enable
    ret = robot.RobotEnable(0) # robot down enable
    print("Enabling under robot", ret)
    time.sleep(3)
    ret = robot.RobotEnable(1) #RobotEnable, robot is automatically enabled by default after power up
    print("Enabling on robot", ret)

Joint Torque Power Detection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetPowerLimit(status, power)``"
    "Description", "Joint Torque Power Detection"
    "Mandatory parameters", "- ``state``: 0-off, 1-on
    - ``power``: set maximum power (W)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Setting the Robot 20004 Port Feedback Cycle
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotRealtimeStateSamplePeriod(period)``"
    "Description", "Setting the robot 20004 port feedback cycle"
    "Mandatory parameter", "- ``period``: robot 20004 port feedback period (ms)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Get robot 20004 port feedback cycle
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotRealtimeStateSamplePeriod()``"
    "Description", "Get robot 20004 port feedback cycle"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``period``: robot 20004 port feedback period (ms)"

Robot software upgrade
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SoftwareUpgrade(filePath, block)``"
    "Description", "Robot Software Upgrade"
    "Required parameters","- ``filePath``: full path of the software upgrade package
    - ``block``: whether to block until the upgrade is complete true:blocking; false:non-blocking"
    "Default parameters", "NULL"
    "Return Value", "- errcode Success-0 Failure- errcode "

Get robot software upgrade status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSoftwareUpgradeState()``"
    "Description", "Get robot software upgrade status"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``state``: robot package upgrade status, 0: idle in progress or uploading upgrade package in progress, 1~100: percentage of upgrade completed, -1: upgrade software failure, -2: checksum failure, -3: version checksum failure, -4: unpacking failure, -5: user configuration upgrade failure, -6: peripheral configuration upgrade failure, -7: extended axis configuration upgrade failure, -8: robot configuration upgrade failure, -9: DH parameter configuration upgrade failure"

    