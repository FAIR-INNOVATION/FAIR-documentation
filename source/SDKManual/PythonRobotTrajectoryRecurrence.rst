Trajectory recurrence
====================================================================

.. toctree::
    :maxdepth: 5

Setting Track Recording Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetTPDParam(name, period_ms, type=1,di_choose=0, do_choose=0)``"
    "Description", "Setting parameters for track logging"
    "Mandatory parameters", "- ``name``: track name;
    - ``period_ms``: sampling period, fixed value, 2ms or 4ms or 8ms;"
    "Default parameters", "- ``type``: data type, 1-joint position;
    - ``di_choose``: DI choose, bit0~bit7 corresponds to control box DI0~DI7, bit8~bit9 corresponds to end DI0~DI1, 0-no choose, 1-choose Default 0.
    - ``do_choose``: DO choose, bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0-no choose, 1-choose Default 0"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
----------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    type = 1 # data type, 1-joint position
    name = 'tpd2023' # track name
    period = 4 # sampling period, 2ms or 4ms or 8ms
    di = 0 # di input configuration
    do = 0 # do output configuration
    ret = robot.SetTPDParam(name, period, di_choose=di) #configure TPD parameters
    print("Configuration TPD parameter error code", ret)
    robot.Mode(1) # robot cut to manual mode
    time.sleep(1)  
    robot.DragTeachSwitch(1) # robot cuts to drag teach mode
    ret = robot.GetActualTCPPose()
    print("Get current tool position", ret)
    time.sleep(1)
    ret = robot.SetTPDStart(name, period, do_choose=do) # start logging the demonstration trajectory
    print("Starting to record the demonstration track error code", ret)
    time.sleep(15)
    ret = robot.SetWebTPDStop() # stop logging the demonstration trajectory
    print("Stopped recording of the demonstration track error code", ret)
    robot.DragTeachSwitch(0) # robot cuts to non-drag teach mode
    # robot.SetTPDDelete('tpd2023') # Delete TPD tracks

Start Track Recording
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTPDStart(name, period_ms, type=1,di_choose=0, do_choose=0)``"
    "Description", "Start Track Record"
    "Mandatory parameters", "- ``name``: track name;
    - ``period_ms``: sampling period, fixed value, 2ms or 4ms or 8ms;"
    "Default parameters", "- ``type``: number datatype, 1-joint position default 1;
    - ``di_choose``: DI choose, bit0~bit7 corresponds to control box DI0~DI7, bit8~bit9 corresponds to end DI0~DI1, 0-no choose, 1-choose Default 0.
    - ``do_choose``: DO choose, bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0-no choose, 1-choose Default 0"
    "Return Value", "Error Code Success-0 Failure- errcode"

Stop Track Recording
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetWebTPDStop()``"
    "Description", "Stop Track Record"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Deleting track records
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTPDDelete(name)``"
    "Description", "Delete Track Record"
    "Mandatory parameters", "- ``name``: track name"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Trajectory preloading
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoadTPD(name)``"
    "Description", "Trajectory Preloading"
    "Mandatory parameters", "- ``name``: track name"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
----------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # P1=[-321.821, 125.694, 282.556, 174.106, -15.599, 152.669]
    name = 'tpd2023' #track name
    blend = 1 # whether to smooth, 1-smooth, 0-not smooth
    ovl = 100.0 #speed scaling
    ret = robot.LoadTPD(name) # track preloading
    print("track preload error code",ret)
    ret,P1 = robot.GetTPDStartPose(name) #Get trajectory start pose
    print ("Get trajectory start position error code",ret, "start position",P1)
    ret = robot.MoveL(P1,0,0) #move to start point
    print("Movement to start point error code",ret)
    time.sleep(10)
    ret = robot.MoveTPD(name, blend, ovl) # trajectory replication
    print("Trajectory reproduction error code",ret)

Get the starting position of the trajectory
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTPDStartPose(name)``"
    "Description", "Get trajectory start position"
    "Mandatory parameters", "- ``name``: track name"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``desc_pose=[x,y,z,rx,ry,rz]``: trajectory start position"

Trajectory Reproduction
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveTPD(name,blend,ovl)``"
    "Description", "Trajectory Reproduction"
    "Mandatory parameters", "- ``name``: track name
    - ``blend``: smooth or not, 0 - not smooth, 1 - smooth
    - ``ovl``: velocity scaling factor, range [0 to 100]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Trajectory preprocessing
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``LoadTrajectoryJ(name,ovl,opt=1)``"
    "description", "trajectory preprocessing"
    "Mandatory parameters", "- ``name``: track name, e.g., /fruser/traj/trajHelix_aima_1.txt.
    - ``ovl``: percentage of speed scaling, range [0~100];"
    "Default parameter", "- ``opt``: 1-control point, default 1"
    "Return Value", "Error Code Success-0 Failure- errcode"

Trajectory Reproduction
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveTrajectoryJ()``"
    "Description", "Trajectory Reproduction"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Getting the starting position of the trajectory
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTrajectoryStartPose(name)``"
    "Description", "Get trajectory starting position"
    "Mandatory parameters", "``name``: track name"
    "Default parameters", "NULL"      
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``desc_pose=[x,y,z,rx,ry,rz]``: trajectory start position"

Get track point number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetTrajectoryPointNum()``"
    "Description", "Get track point number"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``pnum``: track point number"

Setting the speed of the trajectory in operation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJSpeed(ovl)``"
    "Description", "Sets the speed of the trajectory as it runs."
    "Mandatory parameter", "``ovl``: speed scaling percentage, range [0~100]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the force and torque during trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJForceTorque(ft)``"
    "Description", "Setting the force and torque in the trajectory run"
    "Mandatory parameters", "``ft=[fx,fy,fz,tx,ty,tz]``: units N and Nm"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the force along the x-direction in the trajectory run
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJForceFx(fx)``"
    "Description", "Set the force along the x-direction in the trajectory run"
    "Mandatory parameter", "``ft``: force in x direction, in N"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the force along the y-direction in the trajectory run
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJForceFx(fy)``"
    "Description", "Set the force along the y-direction in the trajectory run"
    "Mandatory parameter", "``fy``: force along y direction in N"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the force along the z-direction in a trajectory run
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJForceFx(fz)``"
    "Description", "Set the force along the z-direction in the trajectory run"
    "Mandatory parameter", "``fz``: force along the z-direction, in N"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the torque around the x-axis in a trajectory run
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJTorqueTx(tx)``"
    "Description", "Set the torque around the x-axis for the trajectory run"
    "Mandatory parameter", "``tx``: torque around x-axis in Nm"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the torque around the y-axis in trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetTrajectoryJTorqueTx(ty)``"
    "Description", "Set the torque around the y-axis for the trajectory run"
    "Mandatory parameter", "``ty``: torque around y-axis in Nm"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the torque around the z-axis in trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJTorqueTx(tz)``"
    "Description", "Sets the torque around the z-axis for the trajectory run"
    "Mandatory parameter", "``tz``: torque around z-axis in Nm"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
----------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    name = "/fruser/traj/trajHelix_aima_1.txt" #track name
    blend = 1 # whether to smooth, 1-smooth, 0-not smooth
    ovl = 50.0 #speed scaling
    ft = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
    ret = robot.LoadTrajectoryJ(name,ovl) #trajectory preloading
    print("track preload error code",ret)
    ret,P1 = robot.GetTrajectoryStartPose(name) #Get Trajectory Start Pose
    print ("Get trajectory start position error code",ret, "start position",P1)
    ret = robot.MoveL(P1,1,0) #move to start point
    print("Movement to start point error code",ret)
    ret = robot.GetTrajectoryPointNum() #get the trajectory point number
    print("Get track point number error code",ret)
    time.sleep(10)
    ret = robot.MoveTrajectoryJ() # trajectory replication
    print("Trajectory reproduction error code",ret)
    time.sleep(10)
    ret = robot.SetTrajectoryJSpeed(ovl) #Set the speed in the trajectory run
    print("Setting the speed error code for the trajectory run",ret)
    time.sleep(1)
    ret = robot.SetTrajectoryJForceTorque(ft) #Set the force and torque in the trajectory run
    print("Setting the force and torque error codes for the trajectory run",ret)
    time.sleep(1)
    ret = robot.SetTrajectoryJForceFx(0) #Set the force along the x-direction in the trajectory run
    print("Setting the force along x direction error code in the trajectory run",ret)
    time.sleep(1)
    ret = robot.SetTrajectoryJForceFy(0) #Set the force along the y-direction in the trajectory run
    print("Setting the force error code along the y-direction in the trajectory run",ret)
    time.sleep(1)
    ret = robot.SetTrajectoryJForceFz(0) #Set the force along the z-direction in the trajectory run
    print("Setting the force error code along the z-direction in the trajectory run",ret)
    time.sleep(1)
    ret = robot.SetTrajectoryJTorqueTx(0) #Set the torque around the x-axis for the trajectory run
    print("Setting the torque around x-axis error code for the trajectory run",ret)
    time.sleep(1)
    ret = robot.SetTrajectoryJTorqueTy(0) #Set the torque around the y-axis for the trajectory run
    print("Setting the torque around y-axis error code for the trajectory run",ret)
    time.sleep(1)
    ret = robot.SetTrajectoryJTorqueTz(0) #Set the torque around the z-axis for the trajectory run
    print("Setting the torque around z-axis error code for the trajectory run",ret)
    time.sleep(1)
