Common Robot Settings
====================================================================

.. toctree:. 
    :maxdepth: 5

Setting the global speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetSpeed(vel)``"
    "Description", "Set global speed"
    "Mandatory parameter", "- ``vel``: percentage of speed, range [0~100]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.SetSpeed(20)
    print("Setting global speed error code:",error)

Setting system variable values
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetSysVarValue(id,value)``"
    "Description", "Setting System Variables"
    "Mandatory parameters", "- ``id``: variable number, in the range [1~20].
    - ``value``: variable value"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    for i in range(1,21):
        error = robot.SetSysVarValue(i,10)
    robot.WaitMs(1000)
    for i in range(1,21):
        sys_var = robot.GetSysVarValue(i)
        print("System variable number:",i, "value",sys_var)

Setting Tool Reference Points - Six-Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolPoint(point_num)``"
    "Description", "Setting Tool Reference Points - Six Point Method"
    "Mandatory parameters", "- ``point_num``: point number, range [1~6]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
    for i in range(1,7):
        robot.DragTeachSwitch(1)# cut to drag teach mode
        time.sleep(5)
        error = robot.SetToolPoint(i) # actually should control the robot to move to the required position before sending the command
        print("Six-point method to set tool coordinate system, record points",i, "Error code",error)
        robot.DragTeachSwitch(0)
        time.sleep(1)
    error = robot.ComputeTool()
    print("Six-point method setup tool coordinate system error code",error)

Calculation tool coordinate system - six-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeTool()``"
    "Description", "Calculate the tool coordinate system - six-point method (after setting the six tool reference points)"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``tcp_pose=[x,y,z,rx,ry,rz]``: tool coordinate system"

Setting Tool Reference Points - Four Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTcp4RefPoint(point_num)``"
    "Description", "Setting Tool Reference Points - Four Point Method"
    "Mandatory parameter", "``point_num``: point number, range [1~4]"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``tcp_pose=[x,y,z,rx,ry,rz]``: tool coordinate system"

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
    for i in range(1,5):
        robot.DragTeachSwitch(1)# cut to drag teach mode
        time.sleep(5)
        error = robot.SetTcp4RefPoint(i) # should control the robot to move to the required position before sending the command
        print("Four-point method to set up tool coordinate system, record points",i, "Error code",error)
        robot.DragTeachSwitch(0)
        time.sleep(1)
    error,t_coord= robot.ComputeTcp4()
    print("Error code for four-point method of setting tool coordinate system",error, "Tool TCP",t_coord)

Calculation Tool Coordinate System - Four Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``ComputeTcp4()``"
    "Description", "Calculate tool coordinate system - four-point method (after setting the four tool reference points)"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``tcp_pose=[x,y,z,rx,ry,rz]``: tool coordinate system"

Setting the tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolCoord(id,t_coord,type,install,toolID,loadNum)``"
    "Description", "Setting the tool coordinate system"
    "Mandatory parameters", "- ``id``: coordinate system number, range [1~15];
    - ``t_coord``: Position of the tool center point relative to the center of the end flange in [mm][°];
    - ``type``: 0 - tool coordinate system, 1 - sensor coordinate system;
    - ``install``: installation position, 0 - robot end, 1 - robot exterior
    - ``toolID``: tool ID
    - ``loadNum``: load number"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
    error = robot.SetToolCoord(10,t_coord,0,0,0,0)
    print("Setting tool coordinate system error code",error)

Setting the tool coordinate system list
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolList(id,t_coord ,type, install, loadNum)``"
    "Description", "Set up a list of tool coordinate systems"
    "Mandatory parameters", "- ``id``: coordinate system number, range [1~15];
    - ``t_coord``: [x,y,z,rx,ry,rz] Tool center point relative to end flange center position in [mm][°];
    - ``type``: 0 - tool coordinate system, 1 - sensor coordinate system;
    - ``install``: installation position, 0 - robot end, 1 - robot exterior
    - ``loadNum``: load number"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
    error = robot.SetToolList(10,t_coord,0,0,0)
    print("Setting tool coordinate system list error code",error)

Setting External Tool Reference Points-Three-Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExTCPPoint(point_num)``"
    "Description", "Setting the external tool reference point - three-point method"
    "Mandatory parameters", "- ``point_num``: point number, range [1~3]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
    etool = [21.0,22.0,23.0,24.0,25.0,26.0]
    for i in range(1,4):
        error = robot.SetExTCPPoint(i) # should control the robot to move to the required position before sending the command
        print("Three-point method to set up external tool coordinate system, record points",i, "Error code",error)
        time.sleep(1)
    error,etcp = robot.ComputeExTCF()
    print("Three-point method to set external tool coordinate system error code",error, "external tool TCP",etcp)
    error = robot.SetExToolCoord(10,etcp,etool)
    print("Setting external tool coordinate system error code",error)
    error = robot.SetExToolList(10,etcp,etool)
    print("Setting external tool coordinate system list error code",error)

Calculation of the external tool coordinate system - three-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeExTCF (point_num)``"
    "Description", "Calculate external tool coordinate system - three-point method (after setting three reference points)"
    "Mandatory parameter", "``point_num``: point number, range [1~3]"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``etcp=[x,y,z,rx,ry,rz]``: external tool coordinate system"

Setting the external tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExToolCoord(id,etcp,etool)``"
    "Description", "Setting the external tool coordinate system"
    "Mandatory parameters", "- ``id``: coordinate system number, range [0~14];
    - ``etcp``: External tool coordinate system in [mm][°];
    - ``etool``: end-tool coordinate system in [mm] [°];"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
    etool = [21.0,22.0,23.0,24.0,25.0,26.0]
    error = robot.SetExToolCoord(10,etcp,etool)
    print("Setting external tool coordinate system error code",error)

Setting up a list of external tool coordinate systems
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExToolList(id,etcp ,etool)``"
    "Description", "Set the list of external tool coordinate systems"
    "Mandatory parameters", "- ``id``: coordinate system number, range [0~14];
    - ``etcp``: External tool coordinate system in [mm][°];
    - ``etool``: end-tool coordinate system in [mm] [°];"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
    etool = [21.0,22.0,23.0,24.0,25.0,26.0]
    error = robot.SetExToolList(10,etcp,etool)
    print("Setting external tool coordinate system list error code",error)

Setting the workpiece reference point - three-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWObjCoordPoint(point_num)``"
    "Description", "Setting the workpiece reference point - 3-point method"
    "Mandatory parameter", "``point_num``: point number, range [1~3]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
    robot.SetToolList(0,[0,0,0,0,0,0,0,0],0,0)#Set the reference point should be set before the tool and the workpiece number of the coordinate system should be switched to 0
    robot.SetWObjList(0,[0,0,0,0,0,0,0])
    for i in range(1,4):
        error = robot.SetWObjCoordPoint(i) # actually should control the robot to move to the required position before sending the command
        print("Three-point method to set the workpiece coordinate system, record points",i, "Error code",error)
        time.sleep(1)
    error, w_coord = robot.ComputeWObjCoord(0,0)
    print("Error code for calculating workpiece coordinate system by three-point method", error, "Workpiece coordinate system", w_coord)

Calculation of the workpiece coordinate system - three-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeWObjCoord(method, refFrame)``"
    "Description", "Calculate the workpiece coordinate system - three-point method (three reference points are set and then calculated);"
    "Mandatory parameters","- ``method``: calculation method 0: origin-x-axis-z-axis, 1: origin-x-axis-xy-plane
    - ``refFrame``: reference coordinate system"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``wobj_pose=[x,y,z,rx,ry,rz]``: workpiece coordinate system"


Setting the workpiece coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetWObjCoord(id, coord, refFrame)``"
    "Description", "Setting the workpiece coordinate system"
    "Mandatory parameters", "- ``id``: coordinate system number, range [0~14];
    - ``COORD``: Position of the workpiece in the coordinate system relative to the center of the end flange in [mm][°].
    - ``refFrame``: reference coordinate system"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
    error = robot.SetWObjCoord(id=11,coord=w_coord,refFrame=0)
    print("Error code for setting workpiece coordinate system",error)

Setting the list of workpiece coordinate systems
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetWObjList(id, coord, refFrame)``"
    "Description", "Set the list of workpiece coordinate systems"
    "Mandatory parameters", "- ``id``: coordinate system number, range [0~14];
    - ``COORD``: Position of the workpiece in the coordinate system relative to the center of the end flange in [mm][°].
    - ``refFrame``: reference coordinate system"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
    error = robot.SetWObjList(id=11,coord=w_coord,refFrame=0)
    print("Error code for setting workpiece coordinate system list",error)

Setting the end load weight
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Python SDK-v2.0.8-3.7.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLoadWeight(loadNum, weight)``"
    "Description", "Set the end load weight, incorrect load weight setting may cause the robot to go out of control in drag mode"
    "Mandatory parameters", "- ``loadNum``:load number
    - ``weight``: unit [kg]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Code example
------------------------------------------------------------------------
.. versionchanged:: Python SDK-v2.0.8-3.7.8

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.SetLoadWeight(0,0)#!!!! Load weight should be set to match actual (incorrect load weight setting may cause robot to lose control in drag mode)

Setting the robot mounting method - fixed mounting
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotInstallPos(method)``"
    "description", "set robot mounting method - fixed mounting, wrong mounting method setting can cause robot to lose control in drag mode"
    "Mandatory parameters", "- ``method``: 0-flat loading, 1-side loading, 2-hanging loading"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.SetRobotInstallPos(0) #!!!! The mounting method settings should be the same as the actual 0 - front mount, 1 - side mount, 2 - reverse mount (incorrect mounting method settings can cause the robot to go out of control in drag mode)
    print("Setting robot installation method error code",error)

Setting the robot mounting angle - free mounting
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotInstallAngle(yangle,zangle)``"
    "Description", "Setting the robot mounting angle - free mounting, wrong mounting angle setting can cause the robot to go out of control in drag mode"
    "Mandatory parameters", "- ``yangle``: angle of inclination;
    - ``zangle``: angle of rotation"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.SetRobotInstallAngle(0.0,0.0) #!!!! The mounting angle should be set to match the actual (incorrectly set mounting angle can lead to loss of robot control in drag mode)
    print("Setting robot installation angle error code",error)

Setting the end load center of mass coordinates
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLoadCoord(x,y,z)``"
    "Description", "Set end-load center of mass coordinates, incorrect load center of mass setting may cause robot to go out of control in drag mode"
    "Mandatory parameter", "- ``x``: center of mass coordinates in [mm].
    - ``y``: coordinates of the center of mass in [mm].
    - ``z``: coordinates of the center of mass in [mm]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.SetLoadCoord(3.0,4.0,5.0) #!!! Load center of mass should be set to match the actual (incorrect load center of mass settings can lead to loss of robot control in drag mode)
    print("Setting load center of mass error code",error)

Waiting for a specified time
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitMs(t_ms)``"
    "Description", "Waiting for specified time"
    "Mandatory parameters", "- ``t_ms``: unit [ms]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.WaitMs(1000)
    print("Waiting for specified time error code",error)

Setting robot acceleration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOaccScale(acc)``"
    "Description", "Setting the robot acceleration"
    "Mandatory parameter", "- ``acc``: percentage of robot acceleration"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.SetOaccScale (20)

Setting the machine's specified attitude speed on
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AngularSpeedStart(ratio)``"
    "Description", "Specifies that attitude speed is on."
    "Mandatory parameters", "- ``ratio``: percentage of attitude velocity [0-300]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Specify Attitude Velocity Off
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AngularSpeedEnd()``"
    "Description", "Specify Attitude Velocity Off"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Tool coordinate system transition begins
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Python SDK-v2.0.8-3.7.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ToolTrsfStart(toolNum)``"
    "Description", "Tool coordinate system transition begins"
    "Mandatory parameters", "- ``toolNum``:Tool coordinate system number[0-14]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Tool coordinate system conversion is complete
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Python SDK-v2.0.8-3.7.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ToolTrsfEnd()``"
    "Description", "Tool coordinate system conversion is complete"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "
    
Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    startjointPos = [52.850, -84.327, 102.163, -112.843, -84.131, 0.063]
    startdescPose = [-226.699, -501.969, 264.638, -174.973, 5.852, 143.301]
    endjointPos = [52.850, -77.596, 111.785, -129.196, -84.131, 0.062]
    enddescPose = [-226.702, -501.973, 155.833, -174.973, 5.852, 143.301]

    robot.ToolTrsfStart(1)
    rtn = robot.MoveJ(startjointPos, 0, 0, startdescPose)
    print("rtn is ", rtn)
    rtn = robot.MoveJ(endjointPos, 0, 0, enddescPose)
    print("rtn is ", rtn)
    robot.ToolTrsfEnd()

Calculate the tool coordinate system based on the point information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Python SDK-v2.0.8-3.7.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeToolCoordWithPoints(method, pos)``"
    "Description", "Calculate the tool coordinate system based on the point information"
    "Mandatory parameters", "- ``method``:Calculation method; 0-four point method; One - six point method
    - ``pos``:The array length of the joint position group is 4 in the four-point method and 6 in the six-point method"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode  
    - ``tcp_offset=[x,y,z,rx,ry,rz]``:Tool coordinate system calculated from point information, unit [mm][°]"

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    p1Desc = [-394.073, -276.405, 399.451, -133.692, 7.657, -139.047]
    p1Joint = [15.234, -88.178, 96.583, -68.314, -52.303, -122.926]

    p2Desc = [-187.141, -444.908, 432.425, 148.662, 15.483, -90.637]
    p2Joint = [61.796, -91.959, 101.693, -102.417, -124.511, -122.767]

    p3Desc = [-368.695, -485.023, 426.640, -162.588, 31.433, -97.036]
    p3Joint = [43.896, -64.590, 60.087, -50.269, -94.663, -122.652]

    p4Desc = [-291.069, -376.976, 467.560, -179.272, -2.326, -107.757]
    p4Joint = [39.559, -94.731, 96.307, -93.141, -88.131, -122.673]

    p5Desc = [-284.140, -488.041, 478.579, 179.785, -1.396, -98.030]
    p5Joint = [49.283, -82.423, 81.993, -90.861, -89.427, -122.678]

    p6Desc = [-296.307, -385.991, 484.492, -178.637, -0.057, -107.059]
    p6Joint = [40.141, -92.742, 91.410, -87.978, -88.824, -122.808]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    posJ = [p1Joint, p2Joint, p3Joint, p4Joint, p5Joint, p6Joint]
    rtn, coordRtn = robot.ComputeToolCoordWithPoints(1, posJ)
    print("ComputeToolCoordWithPoints ", rtn, "coord is ", coordRtn[0], coordRtn[1], coordRtn[2], coordRtn[3], coordRtn[4], coordRtn[5])

    robot.MoveJ(p1Joint, 0, 0, p1Desc)
    robot.SetToolPoint(1)
    robot.MoveJ(p2Joint, 0, 0, p2Desc)
    robot.SetToolPoint(2)
    robot.MoveJ(p3Joint, 0, 0, p3Desc)
    robot.SetToolPoint(3)
    robot.MoveJ(p4Joint, 0, 0, p4Desc)
    robot.SetToolPoint(4)
    robot.MoveJ(p5Joint, 0, 0, p5Desc)
    robot.SetToolPoint(5)
    robot.MoveJ(p6Joint, 0, 0, p6Desc)
    robot.SetToolPoint(6)
    rtn, coordRtn = robot.ComputeTool()
    print("ComputeTool ", rtn, "coord is ", coordRtn[0], coordRtn[1], coordRtn[2], coordRtn[3], coordRtn[4], coordRtn[5])

Calculate the workpiece coordinate system based on the point information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Python SDK-v2.0.8-3.7.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeWObjCoordWithPoints(method, pos, refFrame)``"
    "Description", "Calculate the workpiece coordinate system based on the point information"
    "Mandatory parameters", "- ``method``:Calculation method; 0: origin - X-axis - Z-axis 1: origin - X-axis -xy plane
    - ``pos``:Three TCP location groups
    - ``refFrame``: reference coordinate system"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode 
    - ``wobj_offset=[x,y,z,rx,ry,rz]``:Workpiece coordinate system calculated from point information, unit [mm][°]"

Code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    p1Desc = [-275.046, -293.122, 28.747, 174.533, -1.301, -112.101]
    p1Joint = [35.207, -95.350, 133.703, -132.403, -93.897, -122.768]

    p2Desc = [-280.339, -396.053, 29.762, 174.621, -3.448, -102.901]
    p2Joint = [44.304, -85.020, 123.889, -134.679, -92.658, -122.768]

    p3Desc = [-270.597, -290.603, 83.034, 179.314, 0.808, -114.171]
    p3Joint = [32.975, -99.175, 125.966, -116.484, -91.014, -122.857]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    posTCP = [p1Desc, p2Desc, p3Desc]
    rtn, coordRtn = robot.ComputeWObjCoordWithPoints(1, posTCP, 0)
    print("ComputeWObjCoordWithPoints ", rtn, "coord is ", coordRtn[0], coordRtn[1], coordRtn[2], coordRtn[3], coordRtn[4], coordRtn[5])

    robot.MoveJ(p1Joint, 1, 0, p1Desc)
    robot.SetWObjCoordPoint(1)
    robot.MoveJ(p2Joint, 1, 0, p2Desc)
    robot.SetWObjCoordPoint(2)
    robot.MoveJ(p3Joint, 1, 0, p3Desc)
    robot.SetWObjCoordPoint(3)
    rtn, coordRtn = robot.ComputeWObjCoord(1, 0)
    print("ComputeTool ", rtn, "coord is ", coordRtn[0], coordRtn[1], coordRtn[2], coordRtn[3], coordRtn[4], coordRtn[5])