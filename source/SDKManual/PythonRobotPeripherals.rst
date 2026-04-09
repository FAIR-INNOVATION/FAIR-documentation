Peripherals
====================================

.. toctree::
    :maxdepth: 5

Configuration of jaws
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetGripperConfig(company,device,softversion=0,bus=0)``"
    "Description", "Configuration jaws"
    "Required Parameters","- ``company``: gripper claw manufacturer, 1-Robotiq, 2-Huiling, 3-Tianji, 4-Dahuan, 5-Knowledge;
    - ``device``: device number, Robotiq (0-2F-85 series), Huiling (0-NK series, 1-Z-EFG-100), Tianji (0-TEG-110), Dahuan (0-PGI-140), Zhixing (0-CTPM2F20)"
    "Default parameters", "- ``softversion``: software version number, not used for now, default is 0;
    - ``bus``: device mount end bus location, not used yet, default is 0;"
    "Return Value", "Error Code Success-0 Failure- errcode "

Get Jaw Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperConfig()``"
    "Description", "Get Jaw Configuration"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``[number,company,device,softversion]``: number, jaw number; company, jaw manufacturer, 1-Robotiq, 2-Huiling, 3-Tianji, 4-Dahuan, 5-Zhixing ;device, device number, Robotiq (0-2F-85 series), Huiling (0 -NK series,1-Z-EFG-100), Tianji(0-TEG-110), Dahuan(0-PGI-140), Zhixing(0-CTPM2F20) ;softvesion, software version number, not in use for the time being, default is 0."

Activate jaws
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ActGripper(index,action)``"
    "Description", "Activate the jaws."
    "Mandatory parameter", "- ``index``: jaw number;
    - ``action``: 0-reset, 1-activate"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Control jaws
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveGripper(index,pos,vel,force,maxtime,block,type,rotNum,rotVel,rotTorque)``"
    "Description", "Control jaws"
    "Mandatory parameter", "- ``index``: jaw number;
    - ``pos``: percentage of position, range [0~100];
    - ``vel``: percentage of speed, in the range [0 to 100]; ``vel``: percentage of speed, in the range [0 to 100].
    - ``force``: percentage of torque, range [0 to 100];
    - ``maxtime``: maximum wait time, range [0~30000], unit [ms];
    - ``block``: 0-blocking, 1-non-blocking;
    - ``type``: type of jaws, 0-parallel jaws; 1-rotary jaws;
    - ``rotNum``:rotNum The number of rotations;
    - ``rotVel``: Percentage of rotational velocity [0-100];
    - ``rotTorque``: percentage of rotational torque [0-100]."
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Getting the jaw movement status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperMotionDone()``"
    "Description", "Get the state of the jaw motion"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``[fault,status]``: status of gripper movement, fault:0-no error, 1-with error; status:0-movement not completed, 1-movement completed"

Obtain the activated status of the gripper
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperActivateStatus()``"
    "Description", "Obtain the activated status of the gripper"
    "Mandatory parameter", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``fault``：0-correct，1-error
    - ``gripper_active``：bit0 to bit15 correspond to the claw numbers 0 to 15. bit=0 indicates not activated, and bit=1 indicates activated"

Obtain the position of the gripper
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperCurPosition()``"
    "Description", "Obtain the position of the gripper"
    "Mandatory parameter", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``fault``：0-correct，1-error
    - ``position``：Position percentage, ranging from 0 to 100%"

Obtain the gripper speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperCurSpeed()``"
    "Description", "Obtain the gripper speed"
    "Mandatory parameter", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``fault``：0-correct，1-error
    - ``speed``：Speed percentage, range 0 to 100%"

Obtain the gripper current
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperCurCurrent()``"
    "Description", "Obtain the gripper current"
    "Mandatory parameter", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``fault``：0-correct，1-error
    - ``current``：Current percentage, range 0 to 100%"

Obtain the gripper voltage
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperVoltage()``"
    "Description", "Obtain the gripper voltage"
    "Mandatory parameter", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``fault``：0-correct，1-error
    - ``voltage``：Voltage, unit: 0.1V"

Obtain the temperature of the gripper
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperTemp()``"
    "Description", "Obtain the temperature of the gripper"
    "Mandatory parameter", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``fault``：0-correct，1-error
    - ``temp``：Temperature, unit: 0.1V"

Calculate pre-capture point-visual
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputePrePick(desc_pos, zlength, zangle)``"
    "Description", "Calculate pre-capture point-visual"
    "Mandatory parameter", "- ``desc_pos``: clip grab point Cartesian position.
    - ``zlength``: z-axis offset.
    - ``zangle``: rotational offset around the z-axis"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``pre_pos``: pre-grip point Cartesian position"

Calculate retreat point-visual
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputePostPick(desc_pos, zlength, zangle)``"
    "Description", "Computing Retreat Point-Vision"
    "Mandatory parameters", "- ``desc_pos``: grab point Cartesian position;
    - ``zlength``: z-axis offset.
    - ``zangle``: rotational offset around the z-axis"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``post_pos``: retreat point Cartesian poses"

Robot claw operation code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    company = 4
    device = 0
    softversion = 0
    bus = 2
    index = 2
    act = 0
    max_time = 30000
    block = 0
    status = 0
    fault = 0
    active_status = 0
    current_pos = 0
    current = 0
    voltage = 0
    temp = 0
    speed = 0
    robot.SetGripperConfig(company, device, softversion, bus)
    time.sleep(1)
    error,[company, device, softversion, bus] = robot.GetGripperConfig()
    print(f"gripper config:{company},{device},{softversion},{bus}")
    robot.ActGripper(index, act)
    time.sleep(1)
    act = 1
    robot.ActGripper(index, act)
    time.sleep(1)
    error = robot.MoveGripper(index, 90, 50, 50, max_time, block, 0, 0, 0, 0)
    print(f"MoveGripper retval is:{error}")
    time.sleep(1)
    error = robot.MoveGripper(index, 30, 50, 0, max_time, block, 0, 0, 0, 0)
    print(f"MoveGripper retval is:{error}")
    error, [fault, status] = robot.GetGripperMotionDone()
    print(f"motion status:{fault},{status}")
    error, [fault, active_status] = robot.GetGripperActivateStatus()
    print(f"gripper active fault is:{fault},status is:{active_status}")
    error, [fault, current_pos] = robot.GetGripperCurPosition()
    print(f"fault is:{fault},current position is:{current_pos}")
    error, [fault, current] = robot.GetGripperCurCurrent()
    print(f"fault is:{fault},current current is:{current}")
    error, [fault, voltage] = robot.GetGripperVoltage()
    print(f"fault is:{fault},current voltage is:{voltage}")
    error, [fault, temp] = robot.GetGripperTemp()
    print(f"fault is:{fault},current temperature is:{temp}")
    error, [fault, speed] = robot.GetGripperCurSpeed()
    print(f"fault is:{fault},current speed is:{speed}")
    retval = 0
    prepick_pose = [0.0]*6
    postpick_pose = [0.0]*6
    p1Desc = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    p2Desc = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    retval, prepick_pose = robot.ComputePrePick(p1Desc, 10, 0)
    print(f"ComputePrePick retval is:{retval}")
    print(f"xyz is:{prepick_pose[0]},{prepick_pose[1]},{prepick_pose[2]};rpy is:{prepick_pose[3]},{prepick_pose[4]},{prepick_pose[5]}")
    retval, postpick_pose = robot.ComputePostPick(p2Desc, -10, 0)
    print(f"ComputePostPick retval is:{retval}")
    print(f"xyz is:{postpick_pose[0]},{postpick_pose[1]},{postpick_pose[2]};rpy is:{postpick_pose[3]},{postpick_pose[4]},{postpick_pose[5]}")
    robot.CloseRPC()

Get the number of rotation turns of the rotary gripper
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperRotNum()``"
    "Description", "Get the number of rotation turns of the rotary gripper"
    "Mandatory parameter", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``fault``：0-correct，1-error
    - ``num``：Number of turns"

Gets the percentage of rotation speed of the rotating gripper
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperRotSpeed()``"
    "Description", "Gets the percentage of rotation speed of the rotating gripper"
    "Mandatory parameter", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``fault``：0-correct，1-error
    - ``speed``：Percent rotation speed"

Obtains the percentage of rotating torque of the rotating gripper
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperRotTorque()``"
    "Description", "Obtains the percentage of rotating torque of the rotating gripper"
    "Mandatory parameter", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``fault``：0-correct，1-error
    - ``torque``：Percent torque of rotation"

Get the rotary gripper status code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    fault = 0
    rotNum = 0.0
    rotSpeed = 0
    rotTorque = 0
    error,fault, rotNum = robot.GetGripperRotNum()
    error,fault, rotSpeed = robot.GetGripperRotSpeed()
    error,fault, rotTorque = robot.GetGripperRotTorque()
    print(f"gripper rot num:{rotNum},gripper rotSpeed:{rotSpeed},gripper rotTorque:{rotTorque}")
    robot.CloseRPC()

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

Drive Belt Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorSetParam(param, followType, startDis, endDis)``"
    "Description", "Configuration of drive belt parameters"
    "Mandatory parameters", "- ``param= [encChannel,resolution,lead,wpAxis,vision,speedRadio]``: 
                                - ``encChannel``: encoder channels 1-2
                                - ``resolution``: encoder resolution Number of pulses per encoder revolution
                                - ``lead``: Mechanical transmission ratio Distance traveled by the conveyor belt in one revolution of the encoder
                                - ``wpAxis``: Workpiece coordinate system number Select the coordinate system number of the workpiece for the tracking motion function, and set the tracking gripping and TPD tracking to 0.
                                - ``vision``: whether or not to match vision 0 - no 1 - match, 
                                - ``speedRadio``: speed ratio For conveyor tracking gripping speed range (1-100) Tracking motion, TPD tracking set to 1
    - ``followType``: Track movement type, 0- track movement; 1- Follow-up campaign"
    "Default parameters", "- ``startDis``: Tracking grasp needs to be set, tracking start distance, -1: automatic calculation (automatic tracking after the workpiece reaches the robot), the unit is mm, the default value is 0
    - ``endDis``: The tracking capture needs to be set. The tracking termination distance, in mm, is 100 by default"
    "Return Value", "Error Code Success-0 Failure- errcode"

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

Conveyor communication input detection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorComDetect(timeout)``"
    "Description", "Conveyor communication input detection"
    "Mandatory parameters", "- ``timeout``: Wait timeout duration ms"
    "Default Parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Conveyor communication input detection triggered
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ConveyorComDetectTrigger()``"
    "Description", "Conveyor communication input detection triggered"
    "Mandatory parameters", "NULL"
    "Default Parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Robot conveyor operation code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    import threading
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    retval = robot.ConveyorStartEnd(1)
    print(f"ConveyorStartEnd retval is:{retval}")
    retval = robot.ConveyorPointIORecord()
    print(f"ConveyorPointIORecord retval is:{retval}")
    retval = robot.ConveyorPointARecord()
    print(f"ConveyorPointARecord retval is:{retval}")
    retval = robot.ConveyorRefPointRecord()
    print(f"ConveyorRefPointRecord retval is:{retval}")
    retval = robot.ConveyorPointBRecord()
    print(f"ConveyorPointBRecord retval is:{retval}")
    retval = robot.ConveyorStartEnd(0)
    print(f"ConveyorStartEnd retval is:{retval}")
    param = [1.0, 10000.0, 200.0, 0.0, 0.0, 20.0]
    retval = robot.ConveyorSetParam(param,0)
    print(f"ConveyorSetParam retval is:{retval}")
    cmp = [0.0, 0.0, 0.0]
    retval = robot.ConveyorCatchPointComp(cmp)
    print(f"ConveyorCatchPointComp retval is:{retval}")
    index = 1
    max_time = 30000
    block = 0
    retval = 0
    p1Desc = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    p2Desc = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    retval = robot.MoveCart(p1Desc, 1, 0, 100.0)
    print(f"MoveCart retval is:{retval}")
    retval = robot.WaitMs(1)
    print(f"WaitMs retval is:{retval}")
    retval = robot.ConveyorIODetect(10000)
    print(f"ConveyorIODetect retval is:{retval}")
    retval = robot.ConveyorGetTrackData(1)
    print(f"ConveyorGetTrackData retval is:{retval}")
    retval = robot.ConveyorTrackStart(1)
    print(f"ConveyorTrackStart retval is:{retval}")
    retval = robot.ConveyorTrackMoveL("cvrCatchPoint", 1, 0, 100)
    print(f"TrackMoveL retval is:{retval}")
    retval = robot.MoveGripper(index, 51, 40, 30, max_time, block, 0, 0, 0, 0)
    print(f"MoveGripper retval is:{retval}")
    retval = robot.ConveyorTrackMoveL("cvrRaisePoint", 1, 0, 100)
    print(f"TrackMoveL retval is:{retval}")
    retval = robot.ConveyorTrackEnd()
    print(f"ConveyorTrackEnd retval is:{retval}")
    robot.MoveCart(p2Desc, 1, 0, 100.0, 100.0)
    retval = robot.MoveGripper(index, 100, 40, 10, max_time, block, 0, 0, 0, 0)
    print(f"MoveGripper retval is:{retval}")
    robot.CloseRPC()

End Sensor Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AxleSensorConfig(idCompany, idDevice, idSoftware, idBus)``"
    "Description", "End Sensor Configuration"
    "Mandatory parameters", "
    - ``idCompany``: manufacturer, 18-Junkong; 25-Huide
    - ``idDevice``: type, 0-JUNKONG/RYR6T.V1.0
    - ``idSoftware``: software version, 0-J1.0/HuiDe1.0 (not yet available)
    - ``idBus``: mount location, 1-end port 1; 2-end port 2... ..8-end port 8 (not open yet)
    "
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
 
Get End Sensor Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AxleSensorConfigGet()``"
    "Description", "Get end sensor configuration"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``idCompany``: manufacturer, 18-Junkong; 25-Huide
    - ``idDevice``: type, 0-JUNKONG/RYR6T.V1.0"
        
End sensor activation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AxleSensorActivate(actFlag)``"
    "Description", "End sensor activation"
    "Mandatory parameters", "``actFlag``: 0-reset; 1-activate"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``coord``: coordinate system values [x,y,z,rx,ry,rz]"

End Sensor Register Write
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AxleSensorRegWrite(devAddr, regHAddr, regLAddr, regNum, data1, data2, isNoBlock)``"
    "Description", "End Sensor Register Write"
    "Mandatory parameters", "- ``devAddr``: device address number 0-255
    - ``regHAddr``: register address high 8 bits
    - ``regLAddr``: register address lower 8 bits
    - ``regNum``: number of registers 0-255
    - ``data1``: write to register value 1
    - ``data2``: write register value 2
    - ``isNoBlock``: whether blocking 0 - blocking; 1 - non-blocking"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

End sensor code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    import threading
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.AxleSensorConfig(18, 0, 0, 1)
    error, company, type = robot.AxleSensorConfigGet()
    print(f"company is:{company},type is:{type}")
    rtn = robot.AxleSensorActivate(1)
    print(f"AxleSensorActivate rtn is:{rtn}")
    time.sleep(1)
    rtn = robot.AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0)
    print(f"AxleSensorRegWrite rtn is:{rtn}")
    robot.CloseRPC()

Obtaining Robot Peripheral Protocols
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetExDevProtocol()``"
    "Description", "Get robot peripheral protocol"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode. 
    - ``protocol``: Robot peripheral protocol number 4096-Extended Axis Control Card; 4097-ModbusSlave; 4098-ModbusMaster"

Setting up robot peripheral protocols
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExDevProtocol(protocol)``"
    "Description", "Setting the robot peripheral protocol"
    "Mandatory parameters", "- ``protocol``: robot peripheral protocol number 4096 - Extended Axis Control Card; 4097 - ModbusSlave; 4098 - ModbusMaster"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Example of setup robot peripheral protocol code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    protocol = 4096
    rtn = robot.SetExDevProtocol(protocol)
    print(f"SetExDevProtocol rtn:{rtn}")
    rtn, protocol = robot.GetExDevProtocol()
    print(f"GetExDevProtocol rtn:{rtn},protocol is:{protocol}")
    robot.CloseRPC()

Getting end communication parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAxleCommunicationParam()``"
    "Description", "Get end communication parameters"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``baudRate``: baud rate: support 1-9600, 2-14400, 3-19200, 4-38400, 5-56000, 6-67600, 7-115200, 8-128000
    - ``dataBit``: data bit: data bit support (8,9), currently commonly used 8
    - ``stopBit``: stop bit: 1-1, 2-0.5, 3-2, 4-1.5, currently 1 is commonly used.
    - ``verify``: check digit: 0-None, 1-Odd, 2-Even, currently 0.
    - ``timeout``: timeout time: 1~1000ms, this value needs to be combined with the peripheral with the setting of reasonable time parameters
    - ``timeoutTimes``: timeout times: 1~10, mainly for timeout retransmission, reduce occasional exceptions to improve user experience
    - ``period``: periodic instruction time interval:1~1000ms, mainly used for the time interval between each issuance of periodic instructions"

Setting the end communication parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAxleCommunicationParam(baudRate, dataBit, stopBit, verify, timeout, timeoutTimes, period)``"
    "description", "set end communication parameters"
    "Required Parameters","- ``baudRate``: baud rate: supports 1-9600, 2-14400, 3-19200, 4-38400, 5-56000, 6-67600, 7-115200, 8-128000
    - ``dataBit``: data bit: data bit support (8,9), currently commonly used 8
    - ``stopBit``: stop bit: 1-1, 2-0.5, 3-2, 4-1.5, currently 1 is commonly used.
    - ``verify``: check digit: 0-None, 1-Odd, 2-Even, currently 0.
    - ``timeout``: timeout time: 1~1000ms, this value needs to be combined with the peripheral with the setting of reasonable time parameters
    - ``timeoutTimes``: timeout times: 1~10, mainly for timeout retransmission, reduce occasional exceptions to improve user experience
    - ``period``: periodic instruction time interval:1~1000ms, mainly used for the time interval between each issuance of periodic instructions"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Setting the end file transfer type
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAxleFileType(type)``"
    "Description", "Set the end file transfer type"
    "Mandatory parameters", "- ``type``: 1-MCU upgrade file, 2-LUA file"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Setting Enable End LUA Execution
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAxleLuaEnable(enable)``"
    "Description", "Set Enable End LUA Enforcement"
    "Mandatory parameters", "- ``enable``: 0 - not enabled; 1 - enabled"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

End LUA file exception error recovery
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRecoverAxleLuaErr(enable)``"
    "Description", "End LUA File Exception Error Recovery"
    "Mandatory parameters", "- ``status``: 0 - no recovery; 1 - recovery"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Get end LUA execution enable status
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAxleLuaEnableStatus()``"
    "description", "Get end LUA execution enable state"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``enable``: 0-don't enable; 1-enable"

Setting the end LUA end device enable type
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetAxleLuaEnableDeviceType(forceSensorEnable, gripperEnable, IOEnable)``"
    "Description", "Set end LUA end device enable type"
    "Mandatory parameters","- ``forceSensorEnable``: force sensor enable status, 0 - not enabled; 1 - enabled
    - ``gripperEnable``: gripper enable status, 0 - not enabled; 1 - enabled
    - ``IOEnable``: IO device enable status, 0-not enabled; 1-enabled"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Get End LUA End Device Enablement Type
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAxleLuaEnableDeviceType()``"
    "Description", "Get End LUA End Device Enablement Type"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``forceSensorEnable``: force sensor enable status, 0 - not enabled; 1 - enabled
    - ``gripperEnable``: gripper enable status, 0 - not enabled; 1 - enabled
    - ``IOEnable``: IO device enable status, 0-not enabled; 1-enabled"

Get the currently configured end device
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAxleLuaEnableDevice()``"
    "Description", "Get the currently configured end device"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``forceSensorEnable[8]``: force sensor enable state, 0 - not enabled; 1 - enabled
    - ``gripperEnable[8]``: gripper enable status, 0 - not enabled; 1 - enabled
    - ``IOEnable[8]``: IO device enable status, 0-not enabled; 1-enabled"

Setting to enable the jaw movement control function
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAxleLuaGripperFunc(id, func)``"
    "Description", "Set to enable the jaw motion control function"
    "Mandatory parameters", "- ``id``: gripper device number
    - ``func``: 0-jaw enable; 1-jaw initialization; 2-position setting; 3-speed setting; 4-torque setting; 6-reading the jaw status; 7-reading the initialization status; 8-reading the fault code; 9-reading the position; 10-reading the speed; 11-reading the torque,12-15 reserved"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Getting to Enable Jaw Motion Control
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAxleLuaGripperFunc(id)``"
    "Description", "Get the Enable Jaw Motion Control function"
    "Mandatory parameter", "- ``id``: gripper device number"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``func``: 0-jaw enable; 1-jaw initialization; 2-position setting; 3-speed setting; 4-torque setting; 6-reading the jaw status; 7-reading the initialization status; 8-reading the fault code; 9-reading the position; 10-reading the speed; 11-reading the torque,12-15 reserved"

The Ethercat slave file is written by the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SlaveFileWrite(type,slaveID,fileName)``"
    "Description", "The Ethercat slave file is written by the robot"
    "Mandatory parameter", "- ``type``：Slave file type, 1- Upgrade slave file; 2- Upgrade the slave station configuration file
    - ``slaveID``：From the station number
    - ``fileName``：Upload the file name"
    "Default parameters", "NULL"
    "Return Value", "errorcode Success-0 Failure- errcode"


Upload the end Lua open protocol file
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AxleLuaUpload(filePath)``"
    "Description", "Upload the end Lua open protocol file"
    "Mandatory parameter", "- ``filePath``：Local lua file path name .../AXLE_LUA_End_DaHuan.lua"
    "Default parameters", "NULL"
    "Return Value", "errorcode Success-0 Failure- errcode"

The robot Ethercat enters boot mode from the station
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetSysServoBootMode(filePath)``"
    "Description", "The robot Ethercat enters boot mode from the station"
    "Mandatory parameter", "NULL"
    "Default parameters", "NULL"
    "Return Value", "errorcode Success-0 Failure- errcode"

Example of LUA file manipulation code at the end of the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan.lua")
    param = [7, 8, 1, 0, 5, 3, 1]  # 对应AxleComParam参数
    robot.SetAxleCommunicationParam(7, 8, 1, 0, 5, 3, 1)
    error,getParam0,getParam1,getParam2,getParam3,getParam4,getParam5,getParam6 = robot.GetAxleCommunicationParam()
    print(f"GetAxleCommunicationParam param is:{getParam0} {getParam1} {getParam2} {getParam3} {getParam4} {getParam5} {getParam6}")
    robot.SetAxleLuaEnable(1)
    error,luaEnableStatus = robot.GetAxleLuaEnableStatus()
    robot.SetAxleLuaEnableDeviceType(0, 1, 0)
    error,forceEnable, gripperEnable, ioEnable = robot.GetAxleLuaEnableDeviceType()
    print(f"GetAxleLuaEnableDeviceType param is:{forceEnable} {gripperEnable} {ioEnable}")
    func = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
    robot.SetAxleLuaGripperFunc(1, func)
    error,getFunc = robot.GetAxleLuaGripperFunc(1)
    error,getforceEnable, getgripperEnable, getioEnable = robot.GetAxleLuaEnableDevice()
    print("\ngetforceEnable status:", end=" ")
    for i in range(8):
        print(f"{getforceEnable[i]},", end="")
    print("\ngetgripperEnable status:", end=" ")
    for i in range(8):
        print(f"{getgripperEnable[i]},", end="")
    print("\ngetioEnable status:", end=" ")
    for i in range(8):
        print(f"{getioEnable[i]},", end="")
    print()
    robot.ActGripper(1, 0)
    time.sleep(2)
    robot.ActGripper(1, 1)
    time.sleep(2)
    robot.MoveGripper(1, 90, 10, 100, 50000, 0, 0, 0, 0, 0)
    while True:
        error,pkg = robot.GetRobotRealTimeState()
        print(f"gripper pos is:{pkg.gripper_position}")
        time.sleep(0.1)
    robot.CloseRPC()


Obtain the status of the SmartTool button
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSmarttoolBtnState()``"
    "Description", "Obtain the status of the SmartTool button"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode 
    - ``state``: SmartTool handle button status (bit0:0- Communication is normal; 1- Communication disconnection; bit1- Undo operation bit2- Clear the program; bit3-A key bit4-B key bit5-C key bit6-D key bit7-E key bit8-IO key bit9- Manual automatic Starting from bit10"

SmartTool button code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    while True:
        error,state = robot.GetSmarttoolBtnState()
        print(f"{state:016b}")
        time.sleep(0.1)
        
Set the load detection before drag is started
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTorqueDetectionSwitch(flag)``"
    "Description", "Set the load detection before drag is started"
    "Mandatory parameters", "- ``flag``：0- closed; 1- On"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Laser peripheral open and close function
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LaserTrackingLaserOnOff(OnOff, weldId)``"
    "Description", "Laser peripheral open and close function"
    "Mandatory parameters", "- ``OnOff``：0- closed; 1- On"
    "Default parameters", "- ``weldId``：The default weld ID is 0"
    "Return Value", "Error Code Success-0 Failure- errcode"

Laser tracking start-end function
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LaserTrackingTrackOnOff(OnOff, coordId)``"
    "Description", "Laser tracking start-end function"
    "Mandatory parameters", "- ``OnOff``：0- closed; 1- On
    - ``coordId``：Laser peripheral tool coordinate system No"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Laser positioning - Fixed direction
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LaserTrackingSearchStart_xyz(direction, vel, distance, timeout, posSensorNum)``"
    "Description", "Laser positioning - Fixed direction"
    "Mandatory parameters", "- ``direction``：0-x+ 1-x- 2-y+ 3-y- 4-z+ 5-z-
    - ``vel``：Unit of speed %
    - ``distance``：The maximum positioning distance unit is mm
    - ``timeout``：The unit of seek timeout time is ms
    - ``posSensorNum``：The coordinate number of the tool calibrated by laser"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Laser positioning - in any direction
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LaserTrackingSearchStart_point(directionPoint, vel, distance, timeout, posSensorNum)``"
    "Description", "Laser positioning - in any direction"
    "Mandatory parameters", "- ``directionPoint``：To the left of the xyz of the input point for positioning,[x,y,z]
    - ``vel``：Unit of speed %
    - ``distance``：The maximum positioning distance unit is mm
    - ``timeout``：The unit of seek timeout time is ms
    - ``posSensorNum``：The coordinate number of the tool calibrated by laser"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Laser IP configuration
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LaserTrackingSensorConfig(ip, port)``"
    "Description", "Laser IP configuration"
    "Mandatory parameters", "- ``ip``：The ip address of the laser peripheral
    - ``port``：The port number of the laser peripheral"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Configuration of sampling period for laser peripherals
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LaserTrackingSensorSamplePeriod(period)``"
    "Description", "Configuration of sampling period for laser peripherals"
    "Mandatory parameters", "- ``period``：The unit of sampling period for laser peripherals is ms"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Laser peripheral driver loading
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoadPosSensorDriver(type)``"
    "Description", "Laser peripheral driver loading"
    "Mandatory parameters", "- ``type``：Protocol type of the laser device driver: 101-Ruineng 102-Chuangxiang 103-Quanshi 104-Tongzhou 105-Aotai"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Laser peripheral driver unloading
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``UnLoadPosSensorDriver()``"
    "Description", "Laser peripheral driver unloading"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Laser weld seam trajectory recording
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LaserSensorRecord1(status, delayTime)``"
    "Description", "Laser weld seam trajectory recording"
    "Mandatory parameters", "- ``status``：0- Stop recording 1- Real-time tracking 2- Start recording
    - ``delayTime``：The delay time unit is ms"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Laser weld seam trajectory reproduction
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LaserSensorReplay(delayTime, speed)``"
    "Description", "Laser weld seam trajectory reproduction"
    "Mandatory parameters", "- ``delayTime``：The delay time unit is ms
    - ``speed``：Unit of speed %"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
    
Laser tracking reproduction
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveLTR()``"
    "Description", "Laser tracking reproduction"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Laser weld seam trajectory reproduction
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LaserSensorRecordandReplay(delayMode, delayTime, delayDisExAxisNum, delayDis, sensitivePara, int trackMode, int triggerMode, double runTime, speed)``"
    "Description", "Laser Weld Seam Trajectory Replay"
    "Required Parameters", "- ``delayMode``: Mode 0-Delay Time 1-Delay Distance
    - ``delayTime``: Delay time in milliseconds (ms)
    - ``delayDisExAxisNum``: Extended Axis Number
    - ``delayDis``: Delay distance in millimeters (mm)
    - ``sensitivePara``: Compensation Sensitivity Coefficient
    - ``trackMode``: Fixed-point Tracking Type. 0-Extended Axis Asynchronous Motion; 1-Robot
    - ``triggerMode``: Fixed-point Tracking Trigger Method. 0-Tracking Duration; 1-IO
    - ``runTime``: Robot Fixed-point Tracking Duration in seconds (s)
    - ``speed``: Speed in percentage (%)"
    "Default Parameters", "None"
    "Return Value", "Error Code. Success - 0, Failure - errcode"

Movement to the starting point of weld record
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveToLaserRecordStart(moveType, ovl)``"
    "Description", "Movement to the starting point of weld record"
    "Mandatory parameters", "- ``moveType``：0-PTP 1-LIN
    - ``ovl``：Speed unit %"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Movement to the end of the weld record
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveToLaserRecordEnd(moveType, ovl)``"
    "Description", "Movement to the end of the weld record"
    "Mandatory parameters", "- ``moveType``：0-PTP 1-LIN
    - ``ovl``：Speed unit %"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Move to the laser sensor to find the site
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveToLaserSeamPos(moveFlag, ovl, dataFlag, plateType, trackOffectType, offset)``"
    "Description", "Move to the laser sensor to find the site"
    "Mandatory parameters", "- ``moveFlag``：Motion type: 0-PTP; 1-LIN
    - ``ovl``：Speed scaling factor, 0-100
    - ``dataFlag``：Weld cache data selection: 0-execution planning data; 1- Perform logging data
    - ``plateType``：Plate type: 0-corrugated plate; 1- Corrugated board; 2- Fence board; 3- oil barrel; 4- Corrugated shell steel
    - ``trackOffectType``：Laser sensor offset type: 0-no offset; 1-base coordinate shift; 2- Tool coordinate offset; 3-Laser sensor raw data offset
    - ``offset``：Offset value"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Obtain the coordinate information of the laser sensor location
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetLaserSeamPos(trackOffectType, offset)``"
    "Description", "Obtain the coordinate information of the laser sensor location"
    "Mandatory parameters", "- ``trackOffectType``：Laser sensor offset type: 0-no offset; 1-base coordinate shift; 2- Tool coordinate offset; 3-Laser sensor raw data offset
    - ``offset``：Offset value"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``jPos``：Joint position[°]
    - ``descPos``：Cartesian position[mm]
    - ``tool``：Tool coordinate system
    - ``user``：Workpiece coordinate system
    - ``exaxis``：Extension axis position[mm]"

Example of laser peripheral sensor parameter configuration and debugging code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.LaserTrackingSensorConfig("192.168.58.20", 5020)
    robot.LaserTrackingSensorSamplePeriod(20)
    robot.LoadPosSensorDriver(101)
    robot.LaserTrackingLaserOnOff(0, 0)
    time.sleep(3)
    robot.LaserTrackingLaserOnOff(1, 0)
    robot.CloseRPC()

Code example of laser trajectory scanning and trajectory reproduction
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.OpenLuaUpload("D://zUP/CtrlDev_laser_ruiniu-0117.lua")
    time.sleep(2)
    robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua")
    robot.UnloadCtrlOpenLUA(0)
    robot.LoadCtrlOpenLUA(0)
    time.sleep(8)
    i = 0
    while i<10:
        startjointPos = [56.205, -117.951, 141.872, -118.149, -94.217, -122.176]
        startdescPose = [-97.552, -282.855, 26.675, 174.182, -1.338, -91.707]
        exaxisPos = [0.0] * 4
        offdese = [0.0] * 6
        robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)
        robot.LaserSensorRecord1(2, 10)
        endjointPos = [68.809, -87.100, 121.120, -127.233, -95.038, -109.555]
        enddescPose = [-103.555, -464.234, 13.076, 174.179, -1.344, -91.709]
        robot.MoveL(desc_pos=enddescPose,tool= 1,user= 0,vel= 50,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)
        robot.LaserSensorRecord1(0, 10)
        robot.MoveToLaserRecordStart(1, 30)
        robot.LaserSensorReplay(10, 100)
        robot.MoveLTR()
        robot.LaserSensorRecord1(0, 10)
        i = i+1
    robot.CloseRPC()

Code examples for laser locating and real-time tracking
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.OpenLuaUpload("D://zUP/CtrlDev_laser_ruiniu-0117.lua")
    time.sleep(2)
    robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua")
    robot.UnloadCtrlOpenLUA(0)
    robot.LoadCtrlOpenLUA(0)
    time.sleep(8)
    time.sleep(8)
    i = 0
    while i < 10:
        startjointPos = [56.205, -117.951, 141.872, -118.149, -94.217, -122.176]
        startdescPose = [-97.552, -282.855, 26.675, 174.182, -1.338, -91.707]
        exaxisPos = [0.0] * 4
        offdese = [0.0] * 6
        directionPoint = [0.0] * 3
        robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)
        robot.LaserTrackingSearchStart_xyz(3, 100, 300, 1000, 3)
        robot.LaserTrackingSearchStop()
        robot.MoveToLaserSeamPos(1, 30, 0, 0, 0, offdese)
        robot.LaserTrackingTrackOnOff(1, 3)
        endjointPos = [68.809, -87.100, 121.120, -127.233, -95.038, -109.555]
        enddescPose = [-103.555, -464.234, 13.076, 174.179, -1.344, -91.709]
        robot.MoveL(desc_pos=enddescPose,tool= 1,user= 0,vel= 20,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)
        robot.LaserTrackingTrackOnOff(0, 3)
        i = i + 1
        print(i)
    robot.CloseRPC()

Code example of the extended axis synchronized with the robot for laser tracking
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    startexaxisPos = [0.0, 0.0, 0.0, 0.0]
    seamexaxisPos = [-10.0, 0.0, 0.0, 0.0]
    endexaxisPos = [-30.0, 0.0, 0.0, 0.0]
    offdese = [0.0] * 6
    seamjointPos = [0.0] * 6
    seamdescPose = [0.0] * 6
    i=0
    while i < 10:
        startjointPos = [58.337, -119.628, 146.037, -116.358, -92.224, -117.654]
        startdescPose = [-53.375, -255.363, 0.919, 178.054, 1.077, -94.026]
        robot.ExtAxisSyncMoveJ(joint_pos=startjointPos, tool=1,user= 0,vel= 100,acc= 100, ovl=100,exaxis_pos= startexaxisPos,blendT= -1,offset_flag= 0,offset_pos= offdese)
        ret = robot.LaserTrackingSearchStart_xyz(3, 100, 300, 1000, 2)
        robot.LaserTrackingSearchStop()
        tool = 0
        user = 0
        rnte, seamjointPos, seamdescPose, tool, user, startexaxisPos = robot.GetLaserSeamPos(0, offdese)
        print(f"{seamjointPos[0]},{seamjointPos[1]},{seamjointPos[2]},{seamjointPos[3]},{seamjointPos[4]},{seamjointPos[5]},{seamdescPose[0]},{seamdescPose[1]},{seamdescPose[2]},{seamdescPose[3]},{seamdescPose[4]},{seamdescPose[5]}")
        if ret == 0:
            robot.ExtAxisSyncMoveJ(joint_pos=seamjointPos, tool=1,user= 0,vel= 100,acc= 100, ovl=100,exaxis_pos= seamexaxisPos,blendT= -1,offset_flag= 0,offset_pos= offdese)
            robot.LaserTrackingTrackOnOff(1, 2)
            endjointPos = [70.580, -90.918, 126.593, -125.154, -92.162, -105.403]
            enddescPose = [-53.375, -419.020, 0.920, 178.054, 1.076, -94.026]
            robot.ExtAxisSyncMoveL(desc_pos=enddescPose, tool=1,user= 0,vel= 20,acc= 100, ovl=100,blendR= -1,exaxis_pos= endexaxisPos,offset_pos= offdese)
            robot.LaserTrackingTrackOnOff(0, 2)
        i = i+1
        print(i)
    robot.CloseRPC()

Control Array Suction Cup
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetSuckerCtrl(slaveID, len, ctrlValue)``"
    "Description", "Control array suction cup"
    "Required Parameters", "- ``slaveID``: Slave station number
    - ``len``: Length
    - ``ctrlValue``: Control value 1-Suction at maximum vacuum 2-Suction at set vacuum 3-Stop suction"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

Get Array Suction Cup Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSuckerState(slaveID)``"
    "Description", "Get array suction cup status"
    "Required Parameters", "- ``slaveID``: Slave station number"
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure-errcode
    - ``state``: Suction status 0-Object released 1-Workpiece suction successful 2-No object suctioned 3-Object detached
    - ``pressValue``: Current vacuum pressure unit kPa
    - ``error``: Current error code of the suction cup"

Wait for Suction Cup Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitSuckerState(slaveID, state, ms)``"
    "Description", "Wait for suction cup status"
    "Required Parameters", "- ``slaveID``: Slave station number
    - ``state``: Suction status 0-Object released 1-Workpiece suction successful 2-No object suctioned 3-Object detached
    - ``ms``: Maximum waiting time"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

Array Suction Cup Control Command Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish connection with robot controller, returns a robot object upon successful connection
    robot = Robot.RPC('192.168.58.2')
    robot.OpenLuaUpload("C://Project/PeripheralSDK/CtrlDev_sucker.lua")
    time.sleep(2)
    robot.UnloadCtrlOpenLUA(1)
    robot.LoadCtrlOpenLUA(1)
    time.sleep(1)
    ctrl = bytearray(20)
    ctrl[0] = 1
    robot.SetSuckerCtrl(0, 1, ctrl)
    for i in range(100):
        rtn, state, press_value, error = robot.GetSuckerState(1)
        print(f"sucker1 state is {state}, pressValue is {press_value}, error num is {error}")
        rtn, state, press_value, error = robot.GetSuckerState(12)
        print(f"sucker12 state is {state}, pressValue is {press_value}, error num is {error}")
        time.sleep(0.1)
    ret = robot.WaitSuckerState(1, 1, 100)
    print(f"WaitSuckerState result is {ret}")
    ctrl[0] = 3
    robot.SetSuckerCtrl(1, 1, ctrl)
    robot.SetSuckerCtrl(12, 1, ctrl)
    robot.CloseRPC()

Upload Open Protocol Lua File
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``OpenLuaUpload(filePath)``"
    "Description", "Upload open protocol Lua file"
    "Required Parameters", "- ``filePath``: Local open protocol Lua file path name"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

Get Slave Station Board Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetFieldBusConfig()``"
    "Description", "Get slave station board parameters"
    "Required Parameters", "None"
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure-errcode
    - ``type``: 0-Ethercat, 1-CClink, 3-Ethercat, 4-EIP
    - ``version``: Protocol version
    - ``connState``: 0-Not connected 1-Connected"

Write Slave Station DO
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FieldBusSlaveWriteDO(DOIndex, writeNum, status)``"
    "Description", "Write slave station DO"
    "Required Parameters", "- ``DOIndex``: DO number
    - ``writeNum``: Number to write
    - ``status``: Value to write, maximum 8"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

Write Slave Station AO
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FieldBusSlaveWriteAO(AOIndex, writeNum, status)``"
    "Description", "Write slave station AO"
    "Required Parameters", "- ``AOIndex``: AO number
    - ``writeNum``: Number to write
    - ``status``: Value to write, maximum 8"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

Read Slave Station DI
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FieldBusSlaveReadDI(DIIndex, readNum)``"
    "Description", "Read slave station DI"
    "Required Parameters", "- ``DIIndex``: DI number
    - ``readNum``: Number to read"
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure-errcode
    - ``status[8]``: Read values, maximum 8"

Read Slave Station AI
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FieldBusSlaveReadAI(AIIndex, readNum)``"
    "Description", "Read slave station AI"
    "Required Parameters", "- ``AIIndex``: AI number
    - ``readNum``: Number to read"
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure-errcode
    - ``status[8]``: Read values, maximum 8"

Wait for Extended DI Input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FieldBusSlaveWaitDI(DIIndex, status, waitMs)``"
    "Description", "Wait for extended DI input"
    "Required Parameters", "- ``DIIndex``: DI number
    - ``status``: 0-Low level; 1-High level
    - ``waitMs``: Maximum waiting time (ms)"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

Wait for Extended AI Input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FieldBusSlaveWaitAI(AIIndex, waitType, value, waitMs)``"
    "Description", "Wait for extended AI input"
    "Required Parameters", "- ``AIIndex``: AI number
    - ``waitType``: 0-Greater than; 1-Less than
    - ``value``: AI value
    - ``waitMs``: Maximum waiting time (ms)"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

Slave Station Mode Related Interface Command Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish connection with robot controller, returns a robot object upon successful connection
    robot = Robot.RPC('192.168.58.2')
    robot.OpenLuaUpload("D://zUP/Peripheral/CtrlDev_field.lua")
    time.sleep(2)
    robot.SetCtrlOpenLUAName(3,"CtrlDev_field.lua")
    robot.UnloadCtrlOpenLUA(3)
    robot.LoadCtrlOpenLUA(3)
    time.sleep(8)
    rtn,type, version, conn_state = robot.GetFieldBusConfig()
    print(f"type is {type}, version is {version}, connState is {conn_state}")
    # Write digital outputs
    ctrl = [1, 0, 1]  # DO0=1, DO1=0, DO2=1
    robot.FieldBusSlaveWriteDO(0, 3, ctrl)
    # Write analog output
    ctrl_ao = [0x1000]  # AO2 = 0x1000
    robot.FieldBusSlaveWriteAO(2, 1, ctrl_ao)
    for i in range(100):
        rtn,di = robot.FieldBusSlaveReadDI(0, 4)
        print(f"DI0 is {di[0]}, DI1 is {di[1]}, DI2 is {di[2]}, DI3 is {di[3]}")
        rtn, ai = robot.FieldBusSlaveReadAI(0, 3)
        print(f"AI0 is {ai[0]}, AI1 is {ai[1]}, AI2 is {ai[2]}")
        time.sleep(0.01)
    ret = robot.FieldBusSlaveWaitDI(0, 1, 100)
    print(f"FieldBusSlaveWaitDI result is {ret}")
    ret = robot.FieldBusSlaveWaitAI(0, 0, 400.00, 100)
    print(f"FieldBusSlaveWaitAI result is {ret}")
    robot.CloseRPC()

End-Effector Transparent Transmission Function Enable/Disable SDK Interface
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAxleGenComEnable(mode)``"
    "Description", "Enable end-effector general transparent transmission function"
    "Required Parameters", "- ``mode``: Enable, 0-disable, 1-enable"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

End-Effector Transparent Transmission Function Non-Periodic Data Transmission and Reception SDK Interface
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SndRcvAxleGenComCmdData(len_snd, sndBuff, len_rcv)``"
    "Description", "End-effector sends non-periodic data and waits for response"
    "Required Parameters", "
    - ``len_snd``: Length of data to send;
    - ``sndBuff[]``: Data to send;
    - ``len_rcv``: Length of data to receive;
    - ``rcvBuff[]``: Response data;"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

Code Example for Non-Periodic Data Communication of DIO Health Care Moxibustion Head Based on End-Effector Transparent Transmission Function
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from time import sleep
    from fairino import Robot
    from ctypes import sizeof
    # Establish connection with robot controller, returns a robot object upon successful connection
    robot = Robot.RPC('192.168.58.2')

    import time


    def testAxleGenCom(self):

        led_on = [0xAB, 0xBA, 0x12, 0x01, 0x01, 0x79]
        led_off = [0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78]
        version = [0xAB, 0xBA, 0x11, 0x00, 0x76]
        state = [0xAB, 0xBA, 0x1B, 0x01, 0xAA, 0x2B]
        cycleState = [0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78]
        cnt = 1

        p1Joint = [88.708, -86.178, 140.989, -141.825, -89.162, -49.879]
        p1Desc = [188.007, -377.850, 260.207, 178.715, 2.823, -131.466]
        p2Joint = [112.131, -75.554, 126.989, -139.027, -88.044, -26.477]
        p2Desc = [368.003, -377.848, 260.211, 178.715, 2.823, -131.465]

        exaxisPos = [0, 0, 0, 0]
        offdese = [0, 0, 0, 0, 0, 0]

        # Enable end-effector transparent transmission function
        robot.SetAxleGenComEnable(1)
        robot.SetAxleLuaEnable(1)

        while cnt <= 10000:
            # Read version number
            ret,rcvdata = robot.SndRcvAxleGenComCmdData(len_snd=5, sndBuff=version, len_rcv=10)
            print(ret)
            print(rcvdata)
            print(f"hard version : {rcvdata[4]},hard code:{rcvdata[5]}, soft version:{rcvdata[6]} {rcvdata[7]}, soft code:{rcvdata[8]}")
            if ret != 0:
                break
            time.sleep(1)
            # Read moxibustion head presence status
            ret,rcvdata = robot.SndRcvAxleGenComCmdData(6, state, 6)
            print(f"state : {rcvdata[4]} ")
            time.sleep(1)
            # Turn on moxibustion head laser
            ret,rcvdata = robot.SndRcvAxleGenComCmdData(6, led_on, 6)
            print(f"led on rcv data is: {rcvdata[0]}, {rcvdata[1]}, {rcvdata[2]}, {rcvdata[3]}, {rcvdata[4]}, {rcvdata[5]}")
            robot.MoveJ(joint_pos=p1Joint, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=exaxisPos, blendT=-1,
                            offset_flag=0, offset_pos=offdese)
            time.sleep(4)
            # Turn off moxibustion head laser
            ret, rcvdata = robot.SndRcvAxleGenComCmdData(6, led_off, 6)
            print(f"led off rcv data is: {rcvdata[0]}, {rcvdata[1]}, {rcvdata[2]}, {rcvdata[3]}, {rcvdata[4]}, {rcvdata[5]}")
            robot.MoveJ(joint_pos=p2Joint, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=exaxisPos, blendT=-1,offset_flag=0, offset_pos=offdese)
            time.sleep(1)
            print(f"***********************complate No. {cnt} SDK test*****************************")
            cnt = cnt + 1

        robot.CloseRPC()
        return 0

    testAxleGenCom(robot)
    
Download Open Protocol Lua File
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``OpenLuaDownload(fileName, savePath)``"
    "Description", "Download open protocol Lua file"
    "Required Parameters", "
    - ``fileName``: Open protocol file name `CtrlDev_XXX.lua`;
    - ``savePath``: Path to save the open protocol file;
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Delete Specified Open Protocol Lua File
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``OpenLuaDelete(fileName)``"
    "Description", "Delete specified open protocol Lua file"
    "Required Parameters", "
    - ``fileName``: Name of the open protocol Lua file to delete `CtrlDev_XXX.lua`
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"
    
Delete All Open Protocol Lua Files
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AllOpenLuaDelete()``"
    "Description", "Delete all open protocol Lua files"
    "Required Parameters", "None"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

Open Protocol Lua File Operation SDK Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from time import sleep
    import time
    from fairino import Robot

    # Establish connection with robot controller
    robot = Robot.RPC('192.168.58.2')


    def TestCtrlOpenLuaOperate(self):
        # Upload Lua file to robot
        rtn = robot.OpenLuaUpload("D://zUP/openlua/CtrlDev_WELDING_A.lua")
        print(f"OpenLuaUpload rtn is {rtn}")
        
        rtn = robot.OpenLuaUpload("D://zUP/openlua/CtrlDev_SWDPOLISH.lua")
        print(f"OpenLuaUpload rtn is {rtn}")
        
        # Download Lua file from robot
        rtn = robot.OpenLuaDownload("CtrlDev_WELDING_A.lua", "D://zDOWN/")
        print(f"OpenLuaDownload rtn is {rtn}")
        
        rtn = robot.OpenLuaDownload("CtrlDev_SWDPOLISH.lua", "D://zDOWN/")
        print(f"OpenLuaDownload rtn is {rtn}")
        
        # Set control open protocol Lua name
        rtn = robot.SetCtrlOpenLUAName(0, "CtrlDev_WELDING_A.lua")
        print(f"SetCtrlOpenLUAName rtn is {rtn}")
        
        rtn = robot.SetCtrlOpenLUAName(1, "CtrlDev_SWDPOLISH.lua")
        print(f"SetCtrlOpenLUAName rtn is {rtn}")
        
        # Get control open protocol Lua name
        rtn, name = robot.GetCtrlOpenLUAName()
        print(f"ctrl open lua names : {name[0]}, {name[1]}, {name[2]}, {name[3]}")
        
        # Load control open protocol Lua
        rtn = robot.LoadCtrlOpenLUA(1)
        print(f"LoadCtrlOpenLUA rtn is {rtn}")
        time.sleep(2)
        
        # Unload control open protocol Lua
        rtn = robot.UnloadCtrlOpenLUA(1)
        print(f"UnloadCtrlOpenLUA rtn is {rtn}")
        
        # Delete specified Lua file
        rtn = robot.OpenLuaDelete("CtrlDev_WELDING_A.lua")
        print(f"OpenLuaDelete rtn is {rtn}")
        
        # Delete all Lua files
        rtn = robot.AllOpenLuaDelete()
        print(f"AllOpenLuaDelete rtn is {rtn}")
        
        # Close connection
        robot.CloseRPC()
        time.sleep(1)


    # Call test function
    TestCtrlOpenLuaOperate(robot)