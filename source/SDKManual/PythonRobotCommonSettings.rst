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

code example
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

code example
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

code example
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

code example
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

code example
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

code example
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

code example
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

code example
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

code example
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

code example
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

code example
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

code example
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
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLoadWeight(weight)``"
    "Description", "Set the end load weight, incorrect load weight setting may cause the robot to go out of control in drag mode"
    "Mandatory parameters", "- ``weight``: unit [kg]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.SetLoadWeight(0)#!!!! Load weight should be set to match actual (incorrect load weight setting may cause robot to lose control in drag mode)

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

code example
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

code example
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

code example
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

code example
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

code example
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