Fundamentals of Robotics
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
    "Parameter", "- ``ip``：The IP address of the robot, with a default factory IP of “192.168.58.2”"
    "Return value", "- Success: Returns a robot object
    - Failed: The created object will be destroyed"
     
Code example
---------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 3

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')

Query SDK version number
++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSDKVersion()``"
    "Description", "Query SDK version number"
    "Parameter", "Nothing"
    "Return value", "- Success：[0,version]
    - Failed：[errcode,]"

Code example
--------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetSDKVersion()    # Query SDK version number
    if ret[0] == 0:  
        # 0-No fault, return format：[errcode,data],errcode-Fault code，data-Data
        print("SDK version is:",ret[1])
    else:
        print("the errcode is: ", ret[0])

Obtain Controller IP
++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetControllerIP()``"
    "Description", "Obtain Controller IP"
    "Parameter", "Nothing"
    "Return value", "- Success：[0,IP]
    - Failed：[errcode,]"

Code example
--------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetControllerIP()    #Obtain Controller IP
    if ret[0] == 0:
        print("controller ip is:",ret[1])
    else:
        print("the errcode is: ", ret[0])


Control robot hand automatic mode switching
+++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``Mode(state)``"
    "Description", "Control robot hand automatic mode switching"
    "Parameter", "- ``state``：1-Manual mode，0-Automatic mode"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
--------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 5, 7

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.Mode(0)   #The robot goes into automatic operation mode
    time.sleep(1)  
    robot.Mode(1)   #The robot goes into manual mode


Robot drag mode
+++++++++++++++++

Control the robot to enter or exit the drag teaching mode
------------------------------------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``DragTeachSwitch(state)``"
    "Description", "Control the robot to enter or exit the drag teaching mode"
    "Parameter", "- ``state``：1-Enter drag teaching mode，0-Exit drag teaching mode"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Check if the robot is in drag mode
-------------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``IsInDragTeach()``"
    "Description", "Check if the robot is in drag mode"
    "Parameter", "Nothing"
    "Return value", "- Success：[0,state]，state:0-Non drag teaching mode，1-Drag teaching mode
    - Failed：[errcode]"

Code example
^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:
    :emphasize-lines: 7, 9, 15, 17

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.Mode(1) #The robot goes into manual mode
    time.sleep(1)
    robot.DragTeachSwitch(1)  #When the robot enters the drag teaching mode, it can only enter the drag teaching mode in manual mode
    time.sleep(1)
    ret = robot.IsInDragTeach()    #Check whether the user is in drag mode, 1-Drag mode, 0-No drag mode
    if ret[0] == 0:
        print("drag state is:",ret[1])
    else:
        print("the errcode is: ", ret[0])
    time.sleep(3)
    robot.DragTeachSwitch(0)  #When the robot enters the non-drag teaching mode, it can only enter the non-drag teaching mode in manual mode
    time.sleep(1)
    ret = robot.IsInDragTeach()    #Check whether the user is in drag mode, 1-Drag mode, 0-No drag mode)
    if ret[0] == 0:
        print("drag state is:",ret[1])
    else:
        print("the errcode is: ", ret[0])


Control the robot to enable up or down
+++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``RobotEnable(state)``"
    "Description", "Control the robot to enable up or down"
    "Parameter", "- ``state``：1-Upper enable，0-Lower enable"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
---------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 5, 7

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.RobotEnable(0)   #Enable the robot
    time.sleep(3)
    robot.RobotEnable(1)   #This function is enabled on the robot. After the robot is powered on, it is automatically enabled by default