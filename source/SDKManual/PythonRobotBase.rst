Basic
===========================

.. toctree:: 
    :maxdepth: 5

Instantiating robots
++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``RPC(ip)``"
    "Description", "Instantiating a robot object"
    "Required parameter", "- ``ip``:The IP address of the robot, with a default factory IP of “192.168.58.2”"
    "Optional parameter", "Nothing"
    "Return value", "- Success: Returns a robot object
    - Failed: The created object will be destroyed"
     
Code example
---------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')

Query SDK version number
++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSDKVersion()``"
    "Description", "Query SDK version number"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0  Failed -errcode
    - Return:（if success） [SDK_version, Controller_version]"

Code example
--------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret,version  = robot.GetSDKVersion()    # Query SDK version numbe
    if ret ==0:
        print("SDK version is", version )
    else:
        print("The query failed with the error code is",ret)

Obtain Controller IP
++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetControllerIP()``"
    "Description", "Obtain Controller IP"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0  Failed -errcode
    - Return:（if success） IP"

Code example
--------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret,ip = robot.GetControllerIP()    # Obtain Controller IP
    if ret ==0:
        print("controller ip is ", ip)
    else:
        print("the errcode is ",ret)

Control robot manual/automatic mode switch
+++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``Mode(state)``"
    "Description", "Control robot manual/automatic mode switch"
    "Required parameter", "- ``state``:1-Manual mode,0-Automatic mode"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
--------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.Mode(0)   # The robot goes into automatic operation mode
    print("The robot goes into automatic operation mode ", ret)
    time.sleep(1)
    ret = robot.Mode(1)   # The robot goes into manual mode
    print("The robot goes into manual mode ", ret)

Robot drag mode
+++++++++++++++++

Control the robot to enter or exit the drag teaching mode
------------------------------------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``DragTeachSwitch(state)``"
    "Description", "Control the robot to enter or exit the drag teaching mode"
    "Required parameter", "- ``state``:1-Enter drag teaching mode,0-Exit drag teaching mode"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Check if the robot is in drag mode
-------------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``IsInDragTeach()``"
    "Description", "Check if the robot is in drag mode"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
-------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2') 
    robot.Mode(1) # The robot goes into manual mode
    print("The robot goes into manual mode ", ret)
    time.sleep(1)
    ret = robot.DragTeachSwitch(1)  # When the robot enters the drag teaching mode, it can only enter the drag teaching mode in manual mode
    print("the robot enters the drag teaching mode ", ret)
    time.sleep(1)
    ret,state = robot.IsInDragTeach()    # Check whether the user is in drag mode, 1-Drag mode, 0-No drag mode
    if ret == 0:
        print("drag state is：", state)
    else:
        print("the errcode is：",ret)
    time.sleep(3)
    ret = robot.DragTeachSwitch(0)  # When the robot enters the non-drag teaching mode, it can only enter the non-drag teaching mode in manual mode
    print("the robot enters the non-drag teaching mode ", ret)
    time.sleep(1)
    ret,state = robot.IsInDragTeach()    # Check whether the user is in drag mode, 1-Drag mode, 0-No drag mode
    if ret == 0:
        print("drag state is：", state)
    else:
        print("the errcode is：",ret)

Control the robot to enable or lower enable
+++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``RobotEnable(state)``"
    "Description", "Control the robot to enable or lower enable"
    "Required parameter", "- ``state``:1-Upper enable,0-Lower enable"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.RobotEnable(0)   # Unable the robot
    print("Unable the robot", ret)
    time.sleep(3)
    ret = robot.RobotEnable(1)   # This function is enabled on the robot. After the robot is powered on, it is automatically enabled by default
    print("Enabled on the robot ", ret)