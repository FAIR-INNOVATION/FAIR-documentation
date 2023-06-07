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
    "Parameter", "- ``vel``:Speed percentage, range[0~100]"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.SetSpeed(20)   # Set the global speed. Manual mode and automatic mode are set independently


Setting System Variable Values
+++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetSysVarValue(id,value)``"
    "Description", "Setting System Variable Values"
    "Parameter", "- ``id``:Variable number, range[1~20];
    - ``value``:Variable value"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5,8

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    for i in range(1,21):
        robot.SetSysVarValue(i,i+0.5)    #  Setting System Variable Values
    robot.WaitMs(1000)
    for i in range(1,21):
        sys_var = robot.GetSysVarValue(i)  #  Example Query the values of system variables
        print(sys_var)

Set Tool Coordinate System
+++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolCoord(id,t_coord,type,install)``"
    "Description", "Set Tool Coordinate System"
    "Parameter", "- ``id``:Coordinate system number, range[0~14];
    - ``t_coord``:Position of tool center point relative to end flange center, unit[mm][°];
    - ``type``:0-Tool coordinate system,1-Sensor coordinate system;
    - ``install``:Installation position,0-Robot end,1-Robot external"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
    robot.SetToolCoord(10,t_coord,0,0)  #  Set tool coordinate system

Set Tool Coordinate Series Table
++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolList(id,t_coord ,type,install)``"
    "Description", "Set Tool Coordinate Series Table"
    "Parameter", "- ``id``:Coordinate system number, range[0~14];
    - ``t_coord``:Position of tool center point relative to end flange center, unit[mm][°];
    - ``type``:0-Tool coordinate system,1-Sensor coordinate system;
    - ``install``:Installation position,0-Robot end,1-Robot external"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
    robot.SetToolList(10,t_coord,0,0)  #  Set tool coordinate system

Set the external tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExToolCoord(id,etcp ,etool)``"
    "Description", "Set the external tool coordinate system"
    "Parameter", "- ``id``:Coordinate system number, range[0~14];
    - ``etcp``:External tool coordinate system, unit[mm][°];
    - ``etool``:End tool coordinate system, unit[mm][°];"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
    etool = [21.0,22.0,23.0,24.0,25.0,26.0]
    robot.SetExToolCoord(10,etcp,etool)

Set external tool coordinate series table
++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExToolList(id,etcp ,etool)``"
    "Description", "Set external tool coordinate series table"
    "Parameter", "- ``id``:Coordinate system number, range[0~14];
    - ``etcp``:External tool coordinate system, unit[mm][°];
    - ``etool``:End tool coordinate system, unit[mm][°];"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
    etool = [21.0,22.0,23.0,24.0,25.0,26.0]
    robot.SetExToolList(10,etcp,etool)

Set the workpiece coordinate system
++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWObjCoord(id,w_coord)``"
    "Description", "Set the workpiece coordinate system"
    "Parameter", "- ``id``:Coordinate system number, range[0~14];
    - ``w_coord``:Relative pose of coordinate system, unit[mm][°];"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
    robot.SetWObjCoord(11,w_coord)

Set the workpiece coordinate series table
++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWObjList(id,w_coord)``"
    "Description", "Set the workpiece coordinate series table"
    "Parameter", "- ``id``:Coordinate system number, range[0~14];
    - ``w_coord``:Relative pose of coordinate system, unit[mm][°];"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
    robot.SetWObjList(11,w_coord)

Set end load weight
+++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLoadWeight(weight)``"
    "Description", "Set end load weight"
    "Parameter", "- ``weight``:unit[kg]"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.SetLoadWeight(3.0)   # Set load weight

Set the robot installation method - fixed installation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotInstallPos(method)``"
    "Description", "Set the robot installation method - fixed installation"
    "Parameter", "- ``method``:0-Flat, 1-Side, 2-Hanging"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.SetRobotInstallPos(0)    #   Set the robot installation mode

Set robot installation angle - free installation
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotInstallAngle(yangle,zangle)``"
    "Description", "Set robot installation angle - free installation"
    "Parameter", "- ``yangle``:Angle of roll
    - ``zangle``:Rotation angle"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.SetRobotInstallAngle(0.0,0.0)    #   Set the robot installation Angle

Set the centroid coordinates of the end load
++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLoadCoord(x,y,z)``"
    "Description", "Set the centroid coordinates of the end load"
    "Parameter", "- ``x``, ``y``, ``z``: Barycentric coordinate,unit[mm]"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.SetLoadCoord(3.0,4.0,5.0)    #   Set the load centroid coordinates

Waiting for specified time
++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitMs(t_ms)``"
    "Description", "waiting for specified time"
    "Parameter", "- ``t_ms``:unit[ms]"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.WaitMs(1000)    #  Wait 1000ms
