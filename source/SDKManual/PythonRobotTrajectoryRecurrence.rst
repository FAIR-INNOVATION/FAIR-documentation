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

code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    type = 1
    name = "tpd2025"
    period_ms = 4
    di_choose = 0
    do_choose = 0
    robot.SetTPDParam(name, period_ms)
    robot.Mode(1)
    time.sleep(1)
    robot.DragTeachSwitch(1)
    robot.SetTPDStart(name, period_ms)
    print("SetTPDStart")
    time.sleep(10)
    robot.SetWebTPDStop()
    robot.DragTeachSwitch(0)
    robot.CloseRPC()

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

Example of robot TPD trajectory recording code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    type = 1
    name = "tpd2025"
    period_ms = 4
    di_choose = 0
    do_choose = 0
    ovl = 100.0
    blend = 0
    rtn = robot.LoadTPD(name)
    print(f"LoadTPD rtn is: {rtn}")
    error,start_pose = robot.GetTPDStartPose(name)
    print(f"start pose, xyz is: {start_pose[0]},{start_pose[1]},{start_pose[2]}. "
          f"rpy is: {start_pose[3]},{start_pose[4]},{start_pose[5]}")
    robot.MoveCart(start_pose, 0, 0, 100, 100)
    time.sleep(1)
    rtn = robot.MoveTPD(name, blend, ovl)
    print(f"MoveTPD rtn is: {rtn}")
    time.sleep(5)
    robot.SetTPDDelete(name)
    robot.CloseRPC()

Trajectory preprocessing
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``LoadTrajectoryJ(name,ovl,opt=1)``"
    "description", "trajectory preprocessing"
    "Mandatory parameters", "
    - ``name``: Trajectory name, e.g. trajHelix_aima_1.txt, while also supporting full path filenames, e.g. /fruser/traj/trajHelix_aima_1.txt.
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

Set Speed During Trajectory Execution
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJSpeed(ovl, mode)``"
    "Description", "Set the speed during trajectory execution"
    "Required Parameters", "
    - ``ovl``: Speed scaling percentage, range [0~100]
    - ``mode``: 0-speed reduction mode; 1-direct switching"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

Code Example for Setting Speed During Trajectory Execution
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from time import sleep
    import time
    from fairino import Robot

    # Establish connection with robot controller
    robot = Robot.RPC('192.168.58.2')


    def TestSetTrajectoryJSpeed(self):
        # Upload trajectory file
        rtn = robot.TrajectoryJUpLoad("C://Users/lenovo/Desktop/trajHelix_aima_1.txt")
        print(f"Upload TrajectoryJ A {rtn}")

        traj_file_name = "trajHelix_aima_1.txt"
        # Load trajectory file, parameters: file name, speed percentage, loop (1: loop)
        rtn = robot.LoadTrajectoryJ(name=traj_file_name, ovl=100, opt=1)
        print(f"LoadTrajectoryJ {traj_file_name}, rtn is: {rtn}")

        # Get trajectory start pose
        rtn, traj_start_pose = robot.GetTrajectoryStartPose(name=traj_file_name)
        print(f"GetTrajectoryStartPose is: {rtn}")
        print(
            f"desc_pos:{traj_start_pose[0]},{traj_start_pose[1]},{traj_start_pose[2]},{traj_start_pose[3]},{traj_start_pose[4]},{traj_start_pose[5]}")

        time.sleep(1)

        # Set base speed and move to trajectory start point
        robot.SetSpeed(50)
        robot.MoveCart(desc_pos=traj_start_pose, tool=0, user=0, vel=100, acc=100, ovl=100, blendT=-1, config=-1)

        # Get trajectory point count
        rtn, traj_num = robot.GetTrajectoryPointNum()
        print(f"GetTrajectoryStartPose rtn is: {rtn}, traj num is: {traj_num}")

        # Start trajectory motion
        rtn = robot.MoveTrajectoryJ()
        print(f"MoveTrajectoryJ rtn is: {rtn}")

        time.sleep(1)

        # Get robot real-time status
        trajspeedMode = 0
        while True:
            rtn, pkg = robot.GetRobotRealTimeState()
            if pkg.motion_done != 0:
                break

            # Set trajectory speed to 10%
            rtn = robot.SetTrajectoryJSpeed(ovl=10.0, mode=trajspeedMode)
            print(f"SetTrajectoryJSpeed is: {rtn}")

            time.sleep(1)

            # Set trajectory speed to 80%
            rtn = robot.SetTrajectoryJSpeed(ovl=80.0, mode=trajspeedMode)
            print(f"SetTrajectoryJSpeed is: {rtn}")

            time.sleep(1)

        # Close connection
        robot.CloseRPC()
        time.sleep(1)


    # Call test function
    TestSetTrajectoryJSpeed(robot)

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

Upload trace J file
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``TrajectoryJUpLoad(filePath)``"
    "Description", "Upload trace J file"
    "Mandatory parameter", "- ``filePath``:Full path name of the uploaded trajectory file，C://test/testJ.txt"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Delete the track J file
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``TrajectoryJDelete(filePath)``"
    "Description", "Delete the track J file"
    "Mandatory parameter", "- ``filePath``:Removes the full pathname of the trace file,C://test/testJ.txt"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Robot trajectory J file reproduction code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    rtn = robot.TrajectoryJUpLoad("D://zUP/traj.txt")
    print(f"Upload TrajectoryJ A {rtn}")
    traj_file_name = "traj.txt"
    rtn = robot.LoadTrajectoryJ(traj_file_name, 100, 1)
    print(f"LoadTrajectoryJ {traj_file_name}, rtn is: {rtn}")
    rtn,traj_start_pose = robot.GetTrajectoryStartPose(traj_file_name)
    print(f"GetTrajectoryStartPose is: {rtn}")
    print(f"desc_pos:{traj_start_pose[0]},{traj_start_pose[1]},{traj_start_pose[2]},"
          f"{traj_start_pose[3]},{traj_start_pose[4]},{traj_start_pose[5]}")
    time.sleep(1)
    robot.SetSpeed(50)
    robot.MoveCart(traj_start_pose, 0, 0, 50, 100, 100)
    rtn,traj_num = robot.GetTrajectoryPointNum()
    print(f"GetTrajectoryStartPose rtn is: {rtn}, traj num is: {traj_num}")
    rtn = robot.SetTrajectoryJSpeed(50.0)
    print(f"SetTrajectoryJSpeed is: {rtn}")
    traj_force = [0.0,0.0,0.0,0.0,0.0,0.0]
    traj_force[0] = 10  # fx = 10
    rtn = robot.SetTrajectoryJForceTorque(traj_force)
    print(f"SetTrajectoryJForceTorque rtn is: {rtn}")
    rtn = robot.SetTrajectoryJForceFx(10.0)
    print(f"SetTrajectoryJForceFx rtn is: {rtn}")
    rtn = robot.SetTrajectoryJForceFy(0.0)
    print(f"SetTrajectoryJForceFy rtn is: {rtn}")
    rtn = robot.SetTrajectoryJForceFz(0.0)
    print(f"SetTrajectoryJForceFz rtn is: {rtn}")
    rtn = robot.SetTrajectoryJTorqueTx(10.0)
    print(f"SetTrajectoryJTorqueTx rtn is: {rtn}")
    rtn = robot.SetTrajectoryJTorqueTy(10.0)
    print(f"SetTrajectoryJTorqueTy rtn is: {rtn}")
    rtn = robot.SetTrajectoryJTorqueTz(10.0)
    print(f"SetTrajectoryJTorqueTz rtn is: {rtn}")
    rtn = robot.MoveTrajectoryJ()
    print(f"MoveTrajectoryJ rtn is: {rtn}")
    robot.CloseRPC()

Trajectory preprocessing(Trajectory foresight)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoadTrajectoryLA(name, mode, errorLim, type, precision, vamx, amax, jmax, flag)``"
    "Description", "Trajectory preprocessing(Trajectory foresight)"
    "Mandatory parameter", "- ``name``:track file name
    - ``mode``: sampling mode, 0- No sampling; 1- equal data interval sampling; 2- Equal error limit sampling
    - ``errorLim``: error limitation, effective using straight line fitting
    - ``type``: smooth mode, 0-Bessel smooth
    - ``precision``: smoothing accuracy, effective when using Bessel smoothing
    - ``vamx``: set maximum speed, mm/s
    - ``amax``: set maximum acceleration, mm/s2
    - ``jmax``: maximum acceleration, mm/s3
    - ``flag``: Uniform speed forward opening switch 0- No opening; 1- Start"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Trajectory reproduction(Trajectory foresight)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.0

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveTrajectoryLA()``"
    "Description", "Trajectory reproduction(Trajectory foresight)"
    "Mandatory parameter", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example for trajectory reproduction
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller and a robot object is returned if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    rtn = robot.TrajectoryJUpLoad("D://zUP/traj.txt")
    print(f"Upload TrajectoryJ A {rtn}")
    traj_file_name = "traj.txt"
    rtn = robot.LoadTrajectoryLA(traj_file_name, 1, 2, 0, 2, 50, 200, 1000, 0)
    print(f"LoadTrajectoryLA {traj_file_name}, rtn is: {rtn}")
    rtn, traj_start_pose = robot.GetTrajectoryStartPose(traj_file_name)
    print(f"GetTrajectoryStartPose is: {rtn}")
    print(f"desc_pos: {traj_start_pose[0]},{traj_start_pose[1]},{traj_start_pose[2]},{traj_start_pose[3]},{traj_start_pose[4]},{traj_start_pose[5]}")
    time.sleep(1)
    robot.SetSpeed(50)
    robot.MoveCart(traj_start_pose, 0, 0, 100, 100, 100)
    rtn = robot.MoveTrajectoryLA()
    print(f"MoveTrajectoryLA rtn is: {rtn}")
    robot.CloseRPC()

Move to TPD Trajectory Recording Start Point
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveToTPDStart(name, moveType, ovl)``"
    "Description", "Move to TPD trajectory recording start point"
    "Required Parameters", "
    - ``name``: Trajectory file name
    - ``moveType``: Motion type; 0-PTP; 1-LIN
    - ``ovl``: Speed scaling percentage, range [0~100]
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

SDK Code Example for Moving to TPD Trajectory Recording Start Point
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from time import sleep
    from fairino import Robot
    from ctypes import sizeof
    # Establish connection with robot controller, returns a robot object upon successful connection
    robot = Robot.RPC('192.168.58.2')
    import time

    def TestTPD(self):
        type = 1
        name = "tpd2025"
        period_ms = 4
        di_choose = 0
        do_choose = 0

        robot.SetTPDParam(type=type, name=name, period_ms=period_ms, di_choose=di_choose, do_choose=do_choose)

        robot.Mode(1)
        time.sleep(1)
        robot.DragTeachSwitch(1)
        robot.SetTPDStart(type=type, name=name, period_ms=period_ms, di_choose=di_choose, do_choose=do_choose)
        time.sleep(3)
        robot.SetWebTPDStop()
        robot.DragTeachSwitch(0)

        time.sleep(1)
        ovl = 100.0
        blend = 0
        start_pose = [0.0] * 6
        rtn = robot.LoadTPD(name)
        print(f"LoadTPD rtn is: {rtn}")

        rtn, start_pose = robot.GetTPDStartPose(name)
        print(f"start pose, xyz is: {start_pose[0]},{start_pose[1]},{start_pose[2]}. rpy is: {start_pose[3]},{start_pose[4]},{start_pose[5]}")
        # robot.MoveCart(desc_pos=start_pose, tool=0, user=0, vel=100, acc=100, ovl=ovl, blendT=-1, config=-1)
        #time.sleep(1)

        rtn = robot.MoveToTPDStart(name, 0, 100)
        print(f"MoveToTPDStart rtn is: {rtn}")

        rtn = robot.MoveTPD(name, blend, ovl)
        print(f"MoveTPD rtn is: {rtn}")
        time.sleep(5)

        robot.SetTPDDelete(name)

        robot.CloseRPC()
        return 0

    TestTPD(robot)