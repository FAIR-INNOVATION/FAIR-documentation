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

Code example
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

Code example
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

Code example
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller. Successful connection returns the robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.ConveyorPointBRecord()
    print("Convey record B point ",ret)

Conveyor Parameter Configuration
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorSetParam(param, followType, startDis, endDis)``"
    "Description", "Conveyor parameter configuration"
    "Required Parameters", "- ``param``: = [encChannel,resolution,lead,wpAxis,vision,speedRadio] 
                    - ``encChannel``: Encoder channel 1-2
                    - ``resolution``: Encoder resolution (pulses per revolution)
                    - ``lead``: Mechanical transmission ratio (distance moved per encoder revolution)
                    - ``wpAxis``: Workpiece coordinate system number (0 for tracking capture/TPD tracking)
                    - ``vision``: Vision configuration 0-No 1-Yes
                    - ``speedRadio``: Speed ratio (1-100 for tracking capture, 1 for motion tracking/TPD tracking)
    - ``followType``: Tracking motion type, 0-Motion tracking; 1-Inspection tracking"
    "Default Parameters", "- ``startDis``: For inspection capture - tracking start distance (-1: auto calculate), default 0 (mm)
    - ``endDis``: For inspection capture - tracking end distance, default 100 (mm)"
    "Return Value", "Error code (0-success, errcode-failure)"

Code example
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller. Successful connection returns the robot object
    robot = Robot.RPC('192.168.58.2')
    param=[1,10000,200,0,0,20]
    ret = robot.ConveyorSetParam(param,0,0,0)
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

Code example
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

Code example
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Parameter Configuration
    param=[1,10000,200,0,0,20]
    ret = robot.ConveyorSetParam(param,0,0,0)
    print("Conveyor parameter configuration error code",ret)
    time.sleep(1)
    # Grab point compensation
    comp = [0.00, 0.00, 0.00]
    ret1 = robot.ConveyorCatchPointComp(comp)
    print("Error code for compensation of drive belt gripping point", ret1)

Conveyor Communication Input Detection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorComDetect(timeout)``"
    "Description", "Conveyor communication input detection"
    "Required Parameters", "- ``timeout``: Wait timeout (ms)"
    "Default Parameters", "None"
    "Return Value", "Error code (0-success, errcode-failure)"

Conveyor Communication Input Detection Trigger
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorComDetectTrigger()``"
    "Description", "Conveyor communication input detection trigger"
    "Required Parameters", "None"
    "Default Parameters", "None"
    "Return Value", "Error code (0-success, errcode-failure)"

Code Example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    import threading
    # Connect to robot controller
    robot = Robot.RPC('192.168.58.2')

    def Trigger(robot):
        i = int(input("Please input a number to trigger: "))

        rtn = robot.ConveyorComDetectTrigger()
        print(f"ConveyorComDetectTrigger retval is: {rtn}")

    def ConveyorTest(robot):
        retval = 0

        # Uncomment if needed
        # param = [1, 10000, 200, 0, 0, 20]
        # retval = robot.ConveyorSetParam(param, 0, 0, 0)
        # print(f"ConveyorSetParam retval is: {retval}")

        index = 1
        max_time = 30000
        block = 0
        retval = 0

        # Define poses and joint positions
        startdescPose = [139.176, 4.717, 9.088, -179.999, -0.004, -179.990]
        startjointPos = [-34.129, -88.062, 97.839, -99.780, -90.003, -34.140]

        homePose = [139.177, 4.717, 69.084, -180.000, -0.004, -179.989]
        homejointPos = [-34.129, -88.618, 84.039, -85.423, -90.003, -34.140]

        exaxisPos = [0, 0, 0, 0]
        offdese = [0, 0, 0, 0, 0, 0]

        # Move to home position
        retval = robot.MoveL(desc_pos=homePose, tool=1, user=1)
        print(f"MoveL to safety retval is: {retval}")

        # Start trigger thread
        textT = threading.Thread(target=Trigger, args=(robot,))
        textT.daemon = True
        textT.start()

        # Conveyor operations
        retval = robot.ConveyorComDetect(10000)
        print(f"ConveyorComDetect retval is: {retval}")

        retval = robot.ConveyorGetTrackData(2)
        print(f"ConveyorGetTrackData retval is: {retval}")

        retval = robot.ConveyorTrackStart(2)
        print(f"ConveyorTrackStart retval is: {retval}")

        # Movement commands
        robot.MoveL(desc_pos=startdescPose, tool=1, user=1)
        robot.MoveL(desc_pos=startdescPose, tool=1, user=1)

        # End conveyor tracking
        retval = robot.ConveyorTrackEnd()
        print(f"ConveyorTrackEnd retval is: {retval}")

        # Return to home position
        robot.MoveL(desc_pos=homePose, tool=1, user=1)

    ConveyorTest(robot)
