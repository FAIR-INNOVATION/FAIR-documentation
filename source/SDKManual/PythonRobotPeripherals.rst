Peripherals
====================================

.. toctree::
    :maxdepth: 5

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

code example
--------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    desc_pos1=[-333.683,-228.968,404.329,-179.138,-0.781,91.261]
    desc_pos2=[-333.683,-100.8,404.329,-179.138,-0.781,91.261]
    zlength1 =10
    zlength2 =15
    zangle1 =10
    zangle2 =15
    # Test peripheral commands
    ret = robot.SetGripperConfig(4,0) #Configure the gripper jaws
    print("Configuration jaw error code", ret)
    time.sleep(1)
    config = robot.GetGripperConfig() #Get the gripper configuration
    print("Getting the jaw configuration",config)
    error = robot.ActGripper(1,0) #Activate gripper
    print("Activating jaws error code",error)
    time.sleep(1)
    error = robot.ActGripper(1,1)#Activate gripper
    print("Activating jaws error code",error)
    time.sleep(2)
    error = robot.MoveGripper(1,100,48,46,30000,0,0,0,0,0,0) #Control jaws
    print("Control jaw error code",error)
    time.sleep(3)
    error = robot.MoveGripper(1,0,50,0,30000,0,0,0,0,0,0) #Control jaws
    print("Control jaw error code",error)
    error = robot.GetGripperMotionDone() #Get Gripper Motion Done
    print("Error code for obtaining jaw movement status",error)
    error = robot.ComputePrePick(desc_pos1, zlength1, zangle1) #compute pre-pick point-vision
    print("Calculating pre-capture points",error)
    error = robot.ComputePrePick(desc_pos2, zlength2, zangle2) #calculate retreat point-vision
    print("Calculating retreat point",error)

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