Conveyor
======================

.. toctree:: 
    :maxdepth: 5

Conveyor start or stop
++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorStartEnd(status)``"
    "Description", "Obtain force sensor configuration"
    "Description", "Conveyor start or stop"
    "Required parameter", "- ``status``: 1- Start, 0- stop"
    "Optional parameter", "Nothing"
    "Return value",	"Errcode: Success -0  Failed -errcod"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    status = 1
    robot.ConveyorStartEnd(status)

Convey record the IO detection points
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorPointIORecord()``"
    "Description", "Convey record the IO detection points"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcod"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.ConveyorPointIORecord()
    print("Convey record the IO detection points ",ret)

Convey record A point
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype",	"``ConveyorPointARecord()``"
    "Description",	"Convey record A point"
    "Required parameter",	"Nothing"
    "Optional parameter",	"Nothing"
    "Return value",	"Errcode: Success -0  Failed -errcod"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.ConveyorPointARecord()
    print("Convey record A point ",ret)

Convey record reference point
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype",	"``ConveyorRefPointRecord()``"
    "Description",	"Convey record reference point"
    "Required parameter",	"Nothing"
    "Optional parameter",	"Nothing"
    "Return value",	 "Errcode: Success -0  Failed -errcod"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot. ConveyorRefPointRecord()
    print("Convey record reference point ",ret)

Convey record B point
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype",	"``ConveyorPointBRecord()``"
    "Description",	"Convey record B point"
    "Required parameter",	"Nothing"
    "Optional parameter",	"Nothing"
    "Return value",	"Errcode: Success -0  Failed -errcod"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.ConveyorPointBRecord()
    print("Convey record B point ",ret)

Set Conveyor Param
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype",	"``ConveyorSetParam(param)``"
    "Description",	"Set Conveyor Param"
    "Required parameter",	"- ``param`` = [encChannel,resolution,lead,wpAxis,vision,speedRadio];
    - ``encChannel``: rang[1-2];
    - ``resolution``: The number of pulses per revolution of the encoder;
    - ``lead``: mechanical transmission ratio;
    - ``wpAxis Workpiece number``: Select the Workpiece number for the tracking motion function, and set the tracking grab and TPD tracking to 0;
    - ``vision``: 0- not match 1- match;
    - ``speedRadio``: For the belt tracking grab speed range of (1-100) tracking motion, TPD tracking set to 1"
    "Optional parameter",	"Nothing"
    "Return value",	"Errcode: Success -0  Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    param=[1,10000,200,0,0,20]
    ret = robot.ConveyorSetParam(param)
    print("Set Conveyor Param",ret)

Conveyor catch point compensation
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype",	"``ConveyorCatchPointComp(cmp)``"
    "Description",	"Conveyor catch point compensation"
    "Required parameter",	"- ``cmp``: [x,y,z]"
    "Optional parameter",	"Nothing"
    "Return value",	"Errcode: Success -0  Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    cmp = [0.00, 0.00, 0.00]
    ret1 = robot.ConveyorCatchPointComp(cmp)
    print("Conveyor catch point compensation ",ret1)

Conveyor IO detection
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype",	"``ConveyorIODetect(max_t)``"
    "Description",	"Conveyor IO detection"
    "Required parameter",	"- ``max_t``: Maximum detection time, [ms]"
    "Optional parameter",	"Nothing"
    "Return value",	"Errcode: Success -0  Failed -errcod"

Conveyor get track data
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype",	"``ConveyorGetTrackData(mode)``"
    "Description",	"Conveyor get track data"
    "Required parameter",	"- ``mode``:1- Tracking Grab 2- Tracking motion 3-TPD tracking"
    "Optional parameter",	"Nothing"
    "Return value",	"Errcode: Success -0  Failed -errcod"

Conveyor track start
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype",	"``ConveyorTrackStart(status)``"
    "Description",	"Conveyor track start"
    "Required parameter",	"- ``status``: 1- Start, 0- stop"
    "Optional parameter",	"Nothing"
    "Return value",	"Errcode: Success -0  Failed -errcode"

Conveyor track end
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype",	"``ConveyorTrackEnd()``"
    "Description",	"Conveyor track end"
    "Required parameter",	"Nothing"
    "Optional parameter",	"Nothing"
    "Return value",	"Errcode: Success -0  Failed -errcode"

Conveyor track moveL
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype",	"``ConveyorTrackMoveL(name,tool,wobj,vel=20,acc=100,ovl=100,blendR=-1.0)``"
    "Description",	"Conveyor track moveL"
    "Required parameter",	"- ``name`` :cvrCatchPoint or cvrRaisePoint;
    - ``tool`` : Tool number;
    - ``wobj``: Workpiece number;"
    "Optional parameter",	"- ``vel``:Speed percentage,[0~100],default to 20;
    - ``acc``:Acceleration percentage,[0~100],temporarily closed,default to 100;
    - ``ovl``:Speed scaling factor,[0~100] ,default to 100;
    - ``blendR``:[-1.0]-motion in place (blocked), [0-1000]-smooth radius(non blocked), unit[mm] ,default to -1.0"
    "Return value",	"Errcode: Success -0  Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    # Conveyor track
    while(1):
        robot.MoveL([-333.597, 60.354, 404.341, -179.143, -0.778, 91.275],0,0)
        error =robot.ConveyorIODetect(1000)
        print("Conveyor IO detection ",error)
        error =robot.ConveyorGetTrackData(1)
        print("Conveyor get track data ",error)
        error =robot.ConveyorTrackStart(1)
        print("Conveyor track start ",error)
        error =robot.ConveyorTrackMoveL("cvrCatchPoint",0,0,vel = 60.0)
        print("Conveyor track moveL ",error)
        error =robot.MoveGripper(1,55,20,20,30000,0)
        print("Gripper control",error)
        error =robot.ConveyorTrackMoveL("cvrRaisePoint",0,0,vel = 60.0)
        print("Conveyor track moveL ",error)
        error = robot.ConveyorTrackEnd()
        print("Conveyor track end ",error)
        error = robot.MoveL([-333.625, -229.039, 404.340, -179.141, -0.778, 91.276], 0, 0,vel =30)
        error = robot.MoveL([-333.564, 332.204, 342.217, -179.145, -0.780, 91.268], 0, 0,vel =30)
        error = robot.MoveGripper(1,100,10,21,30000,0)
        print("Gripper control ",error)