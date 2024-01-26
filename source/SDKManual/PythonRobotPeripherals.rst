Peripheral
=========================

.. toctree:: 
    :maxdepth: 5

Obtain gripper configuration
+++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperConfig()``"
    "Description", "Obtain gripper configuration"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): [number,company,device,softversion]
    - number: Gripper number, range[1]
    - company:Claw manufacturers, 1-Robotiq, 2-Huiling, 3-Tianji, 4-Dahuan, 5-Zhixing
    - device:Equipment number: Robotiq(0-2F-85 series), Huiling(0-NK series, 1-Z-EFG-100), Tianji(0-TEG-110), Dahuan(0-PGI-140), Zhixing(0-CTPM2F20)
    - softversion:Software version number, temporarily not used, defaults to 0;"

Activate gripper
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ActGripper(index,action)``"
    "Description", "Activate gripper"
    "Required parameter", "- ``index``:Claw number;
    - ``action``: 0-reset, 1-activate"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Control gripper
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveGripper(index,pos,speed,force,maxtime,block)``"
    "Description", "Control gripper"
    "Required parameter", "- ``index``:Claw number;
    - ``pos``:Position percentage, range[0~100];
    - ``speed``:Speed percentage, range[0~100];
    - ``force``:Moment percentage, range[0~100];
    - ``maxtime``:Maximum waiting time, range[0~30000],unit[ms];
    - ``block``:0-blocking, 1-non blocking"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Obtain gripper movement status
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperMotionDone()``"
    "Description", "Obtain gripper movement status"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success):status:0-incomplete movement,1-exercise "

Configure gripper
+++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetGripperConfig(company, device, softversion = 0,bus = 0)``"
    "Description", "Configure gripper"
    "Required parameter", "- ``company``:Claw manufacturers, 1-Robotiq, 2-Huiling, 3-Tianji, 4-Dahuan, 5-Zhixing;
    - ``device``:Equipment number: Robotiq(0-2F-85 series), Huiling(0-NK series, 1-Z-EFG-100), Tianji(0-TEG-110), Dahuan(0-PGI-140), Zhixing(0-CTPM2F20)"
    "Optional parameter", "- ``softversion``::Software version number, temporarily not used, defaults to 0;
    - ``bus``:Device mounted terminal bus position, temporarily not used, defaults to 0"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    desc_pos1=[-333.683,-228.968,404.329,-179.138,-0.781,91.261]
    desc_pos2=[-333.683,-100.8,404.329,-179.138,-0.781,91.261]
    zlength1 =10
    zlength2 =15
    zangle1 =10
    zangle2 =15
    ret = robot.SetGripperConfig(4,0)  # Configure gripper
    print("Configure gripper ", ret)
    time.sleep(1)
    config = robot.GetGripperConfig()     # Obtain gripper configuration
    print("Obtain gripper configuration ",config)
    error = robot.ActGripper(1,0)  # Reset e gripper
    print("Reset gripper",error)
    time.sleep(1)
    error = robot.ActGripper(1,1)# Activate gripper
    print("Activate gripper ",error)
    time.sleep(2)
    error = robot.MoveGripper(1,100,48,46,30000,0) # Control gripper
    print("Control gripper ",error)
    time.sleep(3)
    error = robot.MoveGripper(1,0,50,0,30000,0) # Control gripper
    print("Control gripper ",error)
    error = robot.GetGripperMotionDone() # Obtain gripper movement status
    print("Obtain gripper movement status ",error)
    error = robot.ComputePrePick(desc_pos1, zlength1, zangle1) # Calculate pre- pick points - Vision
    print("Calculate pre- pick points - Vision",error)
    error = robot.ComputePrePick(desc_pos2, zlength2, zangle2) # Calculate post pick points - Vision
    print("Calculate post pick points - Vision ",error)

Calculate pre- pick points - Vision
+++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputePrePick(desc_pos, zlength, zangle)``"
    "Description", "Calculate pre- pick points - Vision"
    "Required parameter", "- ``desc``:Grab point Cartesian pose;
    - ``zlength``:Indicates the offset of the z axis;
    - ``zangle``:Rotation offset about the z axis"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcod
    - Return(if success): pre_pos: Pre- pick point’s Cartesian pose"

Calculate post pick points - Vision
+++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputePostPick(desc_pos, zlength, zangle)``"
    "Description", "Calculate post pick points - Vision"
    "Required parameter", "- ``desc``:Grab point Cartesian pose;
    - ``zlength``:Indicates the offset of the z axis;
    - ``zangle``:Rotation offset about the z axis"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcod
    - Return(if success): post_pos: Post pick point’s Cartesian pose"

