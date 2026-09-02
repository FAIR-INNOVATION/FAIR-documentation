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

Controlling the robot into and out of drag-and-drop instructor mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``DragTeachSwitch(state)``"
    "Description", "Controls the robot into and out of drag-and-drop demonstration mode."
    "Mandatory parameter", "- ``state``: 1-entry into drag-indicator mode, 0-exit from drag-indicator mode"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Queries whether the robot is in drag mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``IsInDragTeach()``"
    "Description", "Queries whether the robot is in drag-and-drop demonstration mode."
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``state``: 0 - non-drag instructional mode, 1 - drag instructional mode"

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

Shut down the robot operating system
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ShutDownRobotOS()``"
    "Description", "Shut down the robot operating system"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Initialize the log parameters
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoggerInit(output_model=1, file_path="", file_num=5)``"
    "Description", "Initialize the log parameters"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``output_model``：Output mode, 0- Direct output; 1- Buffered output; 2- Asynchronous output, default 1
    - ``file_path``：File path + name, the name must be XXX. The log form, such as/home/fr. / Linux/fairino log. The default path where the program is executed, with the default name fairino_year +month+data.log(e.g., fairino_2024_03_13.log);
    - ``file_num``：The number of files for rolling storage ranges from 1 to 20, with a default value of 5. The upper limit for a single file is 50M."
    "Return Value", "Error Code Success-0 Failure- errcode "

Setting the log filter level
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLoggerLevel(lvl=1)``"
    "Description", "Set log filter level"
    "Mandatory parameters", "NULL"
    "Default Parameters","- ``lvl``: filter level value, the smaller the value the less output logs, 1-error, 2-warnning, 3-inform, 4-debug, default value is 1"
    "Return Value", "Error Code Success-0 Failure- errcode"

Robot base control code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    time.sleep(1)
    error,version = robot.GetSDKVersion()
    print(f"SDK version: {version}")
    error,ip = robot.GetControllerIP()
    print(f"controller ip: {ip}")

    robot.Mode(1)
    time.sleep(1)
    robot.DragTeachSwitch(state=1)
    time.sleep(1)
    error,state = robot.IsInDragTeach()
    print(f"drag state: {state}")
    time.sleep(3)
    robot.DragTeachSwitch(state=0)
    time.sleep(1)
    error,state = robot.IsInDragTeach()
    print(f"drag state: {state}")
    time.sleep(3)
    robot.RobotEnable(0)
    time.sleep(3)
    robot.RobotEnable(1)
    robot.Mode(0)
    time.sleep(1)
    robot.Mode(1)
    time.sleep(1)
    rtn = robot.HiSpeedManualSwitch(1)
    print(f"change high speed mode : {rtn}")
    time.sleep(1)
    rtn = robot.HiSpeedManualSwitch(0)
    print(f"change low speed mode : {rtn}")
    time.sleep(3)
    robot.ShutDownRobotOS()
  
    robot.CloseRPC()

Get the software version of the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSoftwareVersion()``"
    "Description", "Get the software version of the robot"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``robotModel``: robot model
    - ``webVersion``: web version
    - ``controllerVersion``: controller version"

Getting robot hardware version information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSlaveHardVersion()``"
    "Description", "Get robot hardware version information"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``ctrlBoxBoardVersion``: control box version
    - ``driver1Version``
    - ``driver2Version``
    - ``driver3Version``
    - ``driver4Version``
    - ``driver5Version``
    - ``driver6Version``
    - ``endBoardVersion``"

Getting robot firmware version information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSlaveFirmVersion()``"
    "Description", "Get information about the robot's firmware version."
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``ctrlBoxBoardVersion``: control box version
    - ``driver1Version``
    - ``driver2Version``
    - ``driver3Version``
    - ``driver4Version``
    - ``driver5Version``
    - ``driver6Version``
    - ``endBoardVersion``"

Get the robot software firmware version code sample
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    rtn,robotModel, webversion, controllerVersion = robot.GetSoftwareVersion()
    print(f"Getsoftwareversion rtn is: {rtn}")
    print(f"robotmodel is: {robotModel}, webversion is: {webversion}, controllerVersion is: {controllerVersion}\n")
    rtn,ctrlBoxBoardversion, driver1version, driver2version,driver3version, driver4version, driver5version,driver6version, endBoardversion = robot.GetHardwareversion()
    print(f"GetHardwareversion rtn is: {rtn}")
    print(f"GetHardwareversion get hardware version is: {ctrlBoxBoardversion}, {driver1version}, {driver2version}, "
          f"{driver3version}, {driver4version}, {driver5version}, {driver6version}, {endBoardversion}\n")
    rtn,ctrlBoxBoardversion, driver1version, driver2version,driver3version, driver4version, driver5version,driver6version, endBoardversion = robot.GetFirmwareVersion()
    print(f"GetFirmwareversion rtn is: {rtn}")
    print(f"GetFirmwareversion get firmware version is: {ctrlBoxBoardversion}, {driver1version}, {driver2version}, "
          f"{driver3version}, {driver4version}, {driver5version}, {driver6version}, {endBoardversion}\n")
    robot.CloseRPC()
