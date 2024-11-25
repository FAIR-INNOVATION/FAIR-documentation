Conveyor
======================

.. toctree::
    :maxdepth: 5

Drive belt start and stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorStartEnd(status)``"
    "Description", "Drive belt start, stop"
    "Mandatory parameters", "- ``status``: status of the drive belt, 1-start, 0-stop"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Conveyor motion, 1-motion, 0-stop
    status = 1
    robot.ConveyorStartEnd(status)
    # Point records
    ret = robot.ConveyorPointIORecord()
    print("Record IO detection point",ret)
    ret = robot.ConveyorPointARecord()
    print("Record point A", ret)
    ret = robot.ConveyorRefPointRecord()
    print("Record reference point",ret)
    ret = robot.ConveyorPointBRecord()
    print("Record point B", ret)

Record IO detection points
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorPointIORecord()``"
    "Description", "Record IO detection points"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Record point A
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorPointARecord()``"
    "Description", "Record point A."
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Recording reference points
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorRefPointRecord()``"
    "Description", "Record reference point"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller. Successful connection returns the robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.ConveyorRefPointRecord()
    print("Convey record reference point ",ret)

Record point B
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorPointBRecord()``"
    "Description", "Record point B."
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller. Successful connection returns the robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.ConveyorPointBRecord()
    print("Convey record B point ",ret)

Drive Belt Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorSetParam(param)``"
    "Description", "Configuration of drive belt parameters"
    "Mandatory parameters", "- `` param= [encChannel,resolution,lead,wpAxis,vision,speedRadio]``: 
    - ``encChannel``: encoder channels 1-2
    - ``resolution``: encoder resolution Number of pulses per encoder revolution
    - ``lead``: Mechanical transmission ratio Distance traveled by the conveyor belt in one revolution of the encoder
    - ``wpAxis``: Workpiece coordinate system number Select the coordinate system number of the workpiece for the tracking motion function, and set the tracking gripping and TPD tracking to 0.
    - ``vision``: whether or not to match vision 0 - no 1 - match, 
    - ``speedRadio``: speed ratio For conveyor tracking gripping speed range (1-100) Tracking motion, TPD tracking set to 1"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller. Successful connection returns the robot object
    robot = Robot.RPC('192.168.58.2')
    param=[1,10000,200,0,0,20]
    ret = robot.ConveyorSetParam(param)
    print("Set Conveyor Param",ret)

Belt Grip Point Compensation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorCatchPointComp(cmp)``"
    "Description", "Drive belt grip point compensation"
    "Mandatory parameters", "- ``cmp``: Compensate for position [x,y,z]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Conveyorized workpiece IO inspection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorIODetect(max_t)``"
    "Description", "Conveyorized workpiece IO detection"
    "Mandatory parameter", "- ``max_t``: Maximum detection time in ms"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Conveyor belt tracking grabs
    while(1).
        robot.MoveL([-333.597, 60.354, 404.341, -179.143, -0.778, 91.275],0,0)
        error =robot.ConveyorIODetect(1000)
        print("Conveyor workpiece IO detection error code",error)
        error =robot.ConveyorGetTrackData(1)
        print("Get object current position error code",error)
        error =robot.ConveyorTrackStart(1)
        print("Drive belt tracking started error code",error)
        error =robot.ConveyorTrackMoveL("cvrCatchPoint",0,0,vel = 60.0)
        print("Linear motion error code",error)
        error =robot.MoveGripper(1,55,20,20,30000,0)
        print("Jaw control error code",error)
        error =robot.ConveyorTrackMoveL("cvrRaisePoint",0,0,vel = 60.0)
        print("Linear motion error code",error)
        error = robot.ConveyorTrackEnd()
        print("End of drive belt tracking error code error code",error)
        error = robot.MoveL([-333.625, -229.039, 404.340, -179.141, -0.778, 91.276], 0, 0,vel =30)
        error = robot.MoveL([-333.564, 332.204, 342.217, -179.145, -0.780, 91.268], 0, 0,vel =30)
        error = robot.MoveGripper(1,100,10,21,30000,0)
        print("Jaw control error code",error)

Get the current position of the object
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorGetTrackData(mode)``"
    "Description", "Get the current position of the object."
    "Mandatory Parameters", "- ``mode``: 1-Tracking Capture 2-Tracking Motion 3-TPD Tracking"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Drive belt tracking started
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorTrackStart(status)``"
    "Description", "Drive belt tracking started"
    "Mandatory parameters", "- ``status``: status, 1-start, 0-stop"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Belt tracking stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorTrackEnd()``"
    "Description", "Drive belt tracking stop"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

linear motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype","``ConveyorTrackMoveL(name,tool,wobj,vel=20,acc=100,ovl=100,blendR=-1.0)``"
    "Description", "Linear motion"
    "Mandatory parameters", "- ``name``: cvrCatchPoint or cvrRaisePoint
    - ``tool``: tool number
    - ``wobj``: workpiece number"
    "Default Parameters","- ``vel``: speed default 20
    - ``acc``: acceleration default 100
    - ``ovl``: velocity scaling factor default 100
    - ``blendR``: [-1.0]-motion in place (blocking), [0~1000]-smoothing radius (non-blocking) in [mm] default -1.0"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Parameter Configuration
    param=[1,10000,200,0,0,20]
    ret = robot.ConveyorSetParam(param)
    print("Conveyor parameter configuration error code",ret)
    time.sleep(1)
    # Grab point compensation
    comp = [0.00, 0.00, 0.00]
    ret1 = robot.ConveyorCatchPointComp(comp)
    print("Error code for compensation of drive belt gripping point", ret1)