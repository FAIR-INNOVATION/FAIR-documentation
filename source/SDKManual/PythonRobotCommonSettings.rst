Common settings
===============================

.. toctree:: 
    :maxdepth: 5

Set global speed
+++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetSpeed(vel)``"
    "Description", "Set global speed"
    "Required parameter", "- ``vel``:Speed percentage, range[0~100]"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object 
    robot = Robot.RPC('192.168.58.2')
    error = robot.SetSpeed(20) # Set the global speed. Manual mode and automatic mode are set independently
    print("Set global speed:",error)

Setting System Variable Values
+++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetSysVarValue(id,value)``"
    "Description", "Setting System Variable Values"
    "Required parameter", "- ``id``:Variable number, range[1~20];
    - ``value``:Variable value"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5,8

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    for i in range(1,21):
        error = robot.SetSysVarValue(i,10)
    robot.WaitMs(1000)
    for i in range(1,21):
        sys_var = robot.GetSysVarValue(i)
        print("Variable number:",i," Variable value ",sys_var)

Set tool reference point - six point method
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolPoint(point_num)``"
    "Description", "Set tool reference point - six point method"
    "Required parameter", "``point_num``:Point number, range[1~6];"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Calculation tool coordinate system - six point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeTool()``"
    "Description", "Calculation tool coordinate system - six point method(Calculate after setting six tool reference points)"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode
    - Return(if success): tcp_pose: tool coordinate system [x,y,z,rx,ry,rz]"

Set tool reference point - four point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTcp4RefPoint(point_num)``"
    "Description", "Set tool reference point - four point method"
    "Required parameter", "``point_num``:Point number, range[1~4];"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
--------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5,8

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object successfully.
    robot = Robot.RPC('192.168.58.2')
    t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
    for i in range(1,5):
        robot.DragTeachSwitch(1)#Switch to drag teaching mode
        time.sleep(5)
        error = robot.SetTcp4RefPoint(i) #The robot should be controlled to move to the appropriate position as required before sending instructions.
        print("Four-point method sets tool coordinate system and records points",i,"error code",error)
        robot.DragTeachSwitch(0)
        time.sleep(1)
    error,t_coord= robot.ComputeTcp4()
    print("Four-point method setting tool coordinate system error code",error,"工具TCP",t_coord)

Calculation tool coordinate system - four point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeTcp4()``"
    "Description", "Calculation tool coordinate system - four point method(Calculate after setting six tool reference points)"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode
    - Return(if success): tcp_pose: tool coordinate system [x,y,z,rx,ry,rz]"

Set Tool Coordinate System
+++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolCoord(id,t_coord,type,install)``"
    "Description", "Set Tool Coordinate System"
    "Required parameter", "- ``id``:Coordinate system number, range[0~14];
    - ``t_coord``:Position of tool center point relative to end flange center, unit[mm][°];
    - ``type``:0-Tool coordinate system,1-Sensor coordinate system;
    - ``install``:Installation position,0-Robot end,1-Robot external"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
    for i in range(1,7):
        robot.DragTeachSwitch(1) 
        time.sleep(5)
        error = robot.SetToolPoint(i) # In fact, the robot should be controlled to move to the appropriate position according to the requirements before sending the command
        print("Set tool reference point - six point method",i,"errcode",error)
        robot.DragTeachSwitch(0)
        time.sleep(1)
    error = robot.ComputeTool()
    print("Calculation tool coordinate system - six point method ",error)
    for i in range(1,5):
        robot.DragTeachSwitch(1) 
        time.sleep(5)
        error = robot.SetTcp4RefPoint(i) # In fact, the robot should be controlled to move to the appropriate position according to the requirements before sending the command
        print("Set tool reference point - four point method",i,"errcode",error)
        robot.DragTeachSwitch(0)
        time.sleep(1)
    error,t_coord= robot.ComputeTcp4()
    print("Calculation tool coordinate system - six point method ",error,"tool TCP",t_coord) 
    error = robot.SetToolCoord(10,t_coord,0,0)
    print("Set Tool Coordinate System",error)

Set Tool Coordinate Series Table
++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolList(id,t_coord ,type,install)``"
    "Description", "Set Tool Coordinate Series Table"
    "Required parameter", "- ``id``:Coordinate system number, range[0~14];
    - ``t_coord``:Position of tool center point relative to end flange center, unit[mm][°];
    - ``type``:0-Tool coordinate system,1-Sensor coordinate system;
    - ``install``:Installation position,0-Robot end,1-Robot external"
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
    t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
    error = robot.SetToolList(10,t_coord,0,0)
    print("Set Tool Coordinate Series Table",error)

Set external tool reference point - three point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExTCPPoint(point_num)``"
    "Description", "Set external tool reference point - three point method"
    "Required parameter", "- ``point_num``:Point number, range[1~3];"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5,8

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object successfully.
    robot = Robot.RPC('192.168.58.2')
    etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
    etool = [21.0,22.0,23.0,24.0,25.0,26.0]
    for i in range(1,4):
        error = robot.SetExTCPPoint(i) #The robot should be controlled to move to the appropriate position as required before sending instructions.
        print("Three-point method sets the external tool coordinate system and records points",i,"error code",error)
        time.sleep(1)
    error,etcp = robot.ComputeExTCF()
    print("Error code for setting external tool coordinate system using three-point method",error,"External tool TCP",etcp)
    error = robot.SetExToolCoord(10,etcp,etool)
    print("Set external tool coordinate system error code",error)
    error = robot.SetExToolList(10,etcp,etool)
    print("Set external tool coordinate series table error code",error)

4.5.2Calculation external tool coordinate system - three point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeExTCF()``"
    "Description", "Calculation external tool coordinate system - three point method(Calculate after setting three tool reference points)"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode
    - Return(if success): etcp: External tool coordinate system [x,y,z,rx,ry,rz]"

Set the external tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExToolCoord(id,etcp ,etool)``"
    "Description", "Set the external tool coordinate system"
    "Required parameter", "- ``id``:Coordinate system number, range[0~14];
    - ``etcp``:External tool coordinate system, unit[mm][°];
    - ``etool``:End tool coordinate system, unit[mm][°];"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
    etool = [21.0,22.0,23.0,24.0,25.0,26.0]
    for i in range(1,4):
        error = robot.SetExTCPPoint(i) # In fact, the robot should be controlled to move to the appropriate position according to the requirements before sending the command
        print("Set external tool reference point - three point method，point:",i,"errcode",error)
        time.sleep(1)
    error,etcp = robot.ComputeExTCF()
    print("Calculation external tool coordinate system - three point method, errcode ",error," etcp: ",etcp) 
    etool = [21.0,22.0,23.0,24.0,25.0,26.0]
    error = robot.SetExToolCoord(10,etcp,etool)
    print("Set the external tool coordinate system ",error)

Set external tool coordinate series table
++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExToolList(id,etcp ,etool)``"
    "Description", "Set external tool coordinate series table"
    "Required parameter", "- ``id``:Coordinate system number, range[0~14];
    - ``etcp``:External tool coordinate system, unit[mm][°];
    - ``etool``:End tool coordinate system, unit[mm][°];"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
    etool = [21.0,22.0,23.0,24.0,25.0,26.0]
    error = robot.SetExToolList(10,etcp,etool)
    print("Set external tool coordinate series table",error)

Set the workpiece reference point - three point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWObjCoordPoint(point_num)``"
    "Description", "Set the workpiece reference point - three point method"
    "Required parameter", "- ``point_num``:Point number, range[1~3];"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Calculation of workpiece coordinate system - three point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWObjCoord(id, w_coord)``"
    "Description", "Calculation of workpiece coordinate system - three point method(Calculate after setting three workpiece reference points)"
    "Required parameter", "- ``Calculation mode``:0: origin - X-axis - Z-axis, 1: origin - X-axis -xy plane"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode
    - Return(if success): wobj_pose: workpiece coordinate system, [x,y,z,rx,ry,rz]"

Set the workpiece coordinate system
++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWObjCoord(id,w_coord)``"
    "Description", "Set the workpiece coordinate system"
    "Required parameter", "- ``id``:Coordinate system number, range[0~14];
    - ``w_coord``:Relative pose of coordinate system, unit[mm][°];"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
    for i in range(1,4):
        error = robot.SetWObjCoordPoint(i) # In fact, the robot should be controlled to move to the appropriate position according to the requirements before sending the command
        print("Set the workpiece reference point - three point method，point",i,"errcode",error)
        time.sleep(1)
    error, w_coord = robot.ComputeWObjCoord(0)
    print("Calculation of workpiece coordinate system - three point method,errcode: ",error," workpiece coordinate syste:", w_coord)
    error = robot.SetWObjCoord(11,w_coord)
    print("Set the workpiece coordinate system ",error)

Set the workpiece coordinate series table
++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWObjList(id,w_coord)``"
    "Description", "Set the workpiece coordinate series table"
    "Required parameter", "- ``id``:Coordinate system number, range[0~14];
    - ``w_coord``:Relative pose of coordinate system, unit[mm][°];"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
    error = robot.SetWObjList(11,w_coord)
    print("Set the workpiece coordinate series table ",error)

Set end load weight
+++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLoadWeight(weight)``"
    "Description", "Set end load weight (An incorrect load weight setting may cause the robot to lose control in drag mode)"
    "Required parameter", "- ``weight``:unit[kg]"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.SetLoadWeight(0)#！The load weight setting should match the actual load weight setting (wrong load weight setting may cause the robot to lose control in drag mode) 

Set the robot installation method - fixed installation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotInstallPos(method)``"
    "Description", "Set the robot installation method - fixed installation (The wrong setting of the installation mode will cause the robot to lose control in drag mode)"
    "Required parameter", "- ``method``:0-Flat installation, 1-Side installation, 2-Hanging installation"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.SetRobotInstallPos(0) #！！！The setting of the installation mode should be consistent with the actual setting (the wrong setting of the installation mode will cause the robot to lose control in drag mode)
    print("Set the robot installation method ",error)

Set robot installation angle - free installation
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotInstallAngle(yangle,zangle)``"
    "Description", "Set robot installation angle - free installation"
    "Required parameter", "- ``yangle``:Angle of roll
    - ``zangle``:Rotation angle"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.SetRobotInstallAngle(0.0,0.0) #！！！The installation Angle setting should be consistent with the actual setting (the wrong installation Angle setting will cause the robot to lose control in drag mode)
    print("Set robot installation angle - free installation ",error)

Set the centroid coordinates of the end load
++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLoadCoord(x,y,z)``"
    "Description", "Set the end load centroid coordinates(An incorrect load centroid setting may cause the robot to lose control in drag mode)"
    "Required parameter", "- ``x``: Barycentric coordinate,unit[mm]
    - ``y``: Barycentric coordinate,unit[mm]
    - ``z``: Barycentric coordinate,unit[mm]"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.SetLoadCoord(3.0,4.0,5.0) #！The load centroid setting should match the actual setting (wrong load centroid setting may cause the robot to lose control in drag mode)
    print("Set the end load centroid coordinates ",error)

Waiting for specified time
++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitMs(t_ms)``"
    "Description", "waiting for specified time"
    "Required parameter", "- ``t_ms``:unit[ms]"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.WaitMs(1000)
    print("Waiting for specified time ",error)
