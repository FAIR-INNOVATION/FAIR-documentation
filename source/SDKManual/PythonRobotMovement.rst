Movement
============

.. toctree::
    :maxdepth: 5

jog point and click
++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``StartJOG(ref,nb,dir,max_dis,vel=20.0,acc=100.0)``"
    "description", "jog dot motion"
    "Mandatory parameters","- ``ref``: 0 - joint point movement, 2 - base coordinate system point movement, 4 - tool coordinate system point movement, 8 - workpiece coordinate system point movement;
    - ``nb``: 1-1 joint (x-axis), 2-2 joint (y-axis), 3-3 joint (z-axis), 4-4 joint (rx), 5-5 joint (ry), 6-6 joint (rz).
    - ``dir``: 0 - negative direction, 1 - positive direction.
    - ``max_dis``: maximum angle/distance of a single tap in ° or mm;"
    "Default Parameters", "- ``vel``: percentage of speed, [0 to 100] default 20;
    - ``acc``: acceleration percentage, [0~100] default 100;"
    "Return Value", "Error Code Success-0 Failure- errcode"

jog tap to decelerate and stop
++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``StopJOG(ref)``"
    "description", "jog nudging deceleration stop"
    "Mandatory parameters", "- ``ref``: 1 - joint point stop, 3 - base coordinate system point stop, 5 - tool coordinate system point stop, 9 - workpiece coordinate system point stop"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Immediate stop for jog taps
++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ImmStopJOG()``"
    "Description", "jog nudging stops immediately"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Robot point control code example
++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    for i in range(6):
        robot.StartJOG(0, i + 1, 0, 20.0, 20.0, 30.0)
        time.sleep(1)
        robot.ImmStopJOG()
        time.sleep(1)
    for i in range(6):
        robot.StartJOG(2, i + 1, 0, 20.0, 20.0, 30.0)
        time.sleep(1)
        robot.ImmStopJOG()
        time.sleep(1)
    for i in range(6):
        robot.StartJOG(4, i + 1, 0, 20.0, 20.0, 30.0)
        time.sleep(1)
        robot.StopJOG(5)
        time.sleep(1)
    for i in range(6):
        robot.StartJOG(8, i + 1, 0, 20.0, 20.0, 30.0)
        time.sleep(1)
        robot.StopJOG(9)
        time.sleep(1)
    robot.CloseRPC()

Joint space motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype","``MoveJ(joint_pos, tool, user, desc_pos = [0.0,0.0,0.0,0.0,0.0,0.0,0.0], vel = 20.0, acc = 0.0, ovl = 100.0, exaxis_pos = [0.0,0.0,0.0,0.0], blendT = -1.0, offset_flag = 0, offset_pos = [0.0,0.0,0.0,0.0,0.0,0.0])``"
    "description", "joint space motion"
    "Mandatory parameters", "- ``joint_pos``: target joint position in [°];
    - ``tool``: tool number, [0 to 14];
    - ``user``: artifact number, [0 to 14];"
    "Default parameters","- ``desc_pos``: target Cartesian position in [mm][°] Default initial value [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls positive kinematics to solve for the return value.
    - ``vel``: percentage of speed, [0~100] default 20.0.
    - ``acc``: percentage of acceleration, [0~100], not open yet;
    - ``ovl``: velocity scaling factor, [0~100] default 100.0.
    - ``exaxis_pos``: external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0].
    - ``blendT``:[-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in [ms] default -1.0;
    - ``offset_flag``:[0]-no offset, [1]-offset in workpiece/base coordinate system, [2]-offset in tool coordinate system Default 0;
    - ``offset_pos``: position offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0];"
    "Return Value", "Error Code Success-0 Failure- errcode"

Cartesian Space Linear Motion
++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveL(desc_pos, tool, user, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0, ovl=100.0,blendR=-1.0, blendMode = 0,exaxis_pos=[0.0, 0.0, 0.0, 0.0], search=0, offset_flag=0,offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],oacc = 100.0,config=-1,velAccParamMode=0,overSpeedStrategy=0,speedPercent=10)``"
    "Description", "Cartesian Space Linear Motion"
    "Required Parameters", "- ``desc_pos``: Target Cartesian pose, unit [mm][°]；
    - ``tool``: Tool number, [0~14]；
    - ``user``: Workpiece/User number, [0~14]；"
    "Default Parameters", "- ``joint_pos``: Target joint positions, unit [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0], default value is the return value from inverse kinematics；
    - ``vel``: Velocity percentage, [0~100] Default 20.0；
    - ``acc``: Acceleration percentage, [0~100], currently unavailable Default 0.0；
    - ``ovl``: Velocity scaling factor, [0~100] Default 100.0；
    - ``blendR``:[-1.0]-Block until motion completes (blocking), [0~1000]-Blend radius (non-blocking), unit [mm] Default -1.0；
    - ``blendMode``: Transition method; 0-Tangent blend； 1-Corner blend, default 0；
    - ``exaxis_pos``: External axis 1 position ~ External axis 4 position Default [0.0,0.0,0.0,0.0]；
    - ``search``:[0]-No wire search, [1]-Wire search；
    - ``offset_flag``:[0]-No offset, [1]-Offset in workpiece/base coordinate system, [2]-Offset in tool coordinate system Default 0；
    - ``offset_pos``: Pose offset value, unit [mm][°] Default [0.0,0.0,0.0,0.0,0.0,0.0]；
    - ``oacc``: Acceleration scaling factor [0-100] / Physical acceleration (mm/s²) Default 100；
    - ``config``: Inverse solution joint space configuration, [-1]-Solve with reference to current joint positions, [0~7]-Solve according to a specific joint space configuration, default -1
    - ``velAccParamMode``: Velocity/Acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) and acceleration (mm/s²) Default 0
    - ``overSpeedStrategy``: Overspeed handling strategy, 0-Strategy disabled; 1-Standard; 2-Stop with error on overspeed; 3-Adaptive speed reduction, default 0
    - ``speedPercent``: Allowable speed reduction threshold percentage [0-100], default 10%
    "
    "Return Value", "Error code 0-Success  Error- errcode"

Cartesian Space Circular Arc Motion
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveC(desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],vel_p=20.0, acc_p=100.0, exaxis_pos_p=[0.0, 0.0, 0.0, 0.0], offset_flag_p=0,offset_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],vel_t=20.0, acc_t=100.0, exaxis_pos_t=[0.0, 0.0, 0.0, 0.0], offset_flag_t=0,offset_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],ovl=100.0, blendR=-1.0,oacc=100.0,config=-1,velAccParamMode=0)``"
    "Description", "Cartesian Space Circular Arc Motion"
    "Required Parameters", "- ``desc_pos_p``: Path point Cartesian pose, unit [mm][°]；
    - ``tool_p``: Path point tool number, [0~14];
    - ``user_p``: Path point workpiece/user number, [0~14];
    - ``desc_pos_t``: Target point Cartesian pose, unit [mm][°];
    - ``tool_t``: Tool number, [0~14]；
    - ``user_t``: Workpiece/User number, [0~14]；"
    "Default Parameters", "- ``joint_pos_p``: Path point joint positions, unit [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0], default value is the return value from inverse kinematics;
    - ``joint_pos_t``: Target point joint positions, unit [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0], default value is the return value from inverse kinematics;
    - ``vel_p``: Path point velocity percentage, [0~100] Default 20.0;
    - ``acc_p``: Path point acceleration percentage, [0~100] Currently unavailable, default 0.0;
    - ``exaxis_pos_p``: Path point external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0];
    - ``offset_flag_p``: Whether the path point is offset [0]-No offset, [1]-Offset in workpiece/base coordinate system, [2]-Offset in tool coordinate system Default 0;
    - ``vel_t``: Target point velocity percentage, [0~100] Default 20.0;
    - ``acc_t``: Target point acceleration percentage, [0~100] Currently unavailable default 0.0;
    - ``exaxis_pos_t``: Target point external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0];
    - ``offset_flag_t``: Whether the target point is offset [0]-No offset, [1]-Offset in workpiece/base coordinate system, [2]-Offset in tool coordinate system Default 0;
    - ``offset_pos_t``: Target point pose offset value, unit [mm][°] Default [0.0,0.0,0.0,0.0,0.0,0.0];
    - ``ovl:``: Velocity scaling factor, [0~100] Default 100.0;
    - ``blendR``:[-1.0]-Block until motion completes (blocking), [0~1000]-Blend radius (non-blocking), unit [mm] Default -1.0;
    - ``oacc``: Acceleration scaling factor [0-100] / Physical acceleration (mm/s²) Default 100;
    - ``config``: Inverse solution joint space configuration, [-1]-Solve with reference to current joint positions, [0~7]-Solve according to a specific joint space configuration, default -1;
    - ``velAccParamMode``: Velocity/Acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) and acceleration (mm/s²) Default 0"
    "Return Value", "Error code 0-Success  Error- errcode"

Cartesian Space Full Circle Motion
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``Circle(desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],vel_p=20.0, acc_p=0.0, exaxis_pos_p=[0.0, 0.0, 0.0, 0.0], vel_t=20.0, acc_t=0.0,exaxis_pos_t=[0.0, 0.0, 0.0, 0.0],ovl=100.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], oacc=100.0, blendR=-1,config=-1,velAccParamMode=0)``"
    "Description", "Cartesian Space Full Circle Motion"
    "Required Parameters", "- ``desc_pos_p``: Path point Cartesian pose, unit [mm][°]；
    - ``tool_p``: Tool number, [0~14]；
    - ``user_p``: Workpiece/User number, [0~14]；
    - ``desc_pos_t``: Target point Cartesian pose, unit [mm][°]；
    - ``tool_t``: Tool number, [0~14]；
    - ``user_t``: Workpiece/User number, [0~14]；"
    "Default Parameters", "- ``joint_pos_p``: Path point joint positions, unit [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0], default value is the return value from inverse kinematics;
    - ``joint_pos_t``: Target point joint positions, unit [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0], default value is the return value from inverse kinematics;
    - ``vel_p``: Velocity percentage, [0~100] Default 20.0;
    - ``acc_p``: Path point acceleration percentage, [0~100] Currently unavailable default 0.0;
    - ``exaxis_pos_p``: Path point external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0];
    - ``vel_t``: Target point velocity percentage, [0~100] Default 20.0;
    - ``acc_t``: Target point acceleration percentage, [0~100] Currently unavailable default 0.0;
    - ``exaxis_pos_t``: Target point external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0]
    - ``ovl``: Velocity scaling factor, [0~100] Default 100.0;
    - ``offset_flag``: Whether to offset [0]-No offset, [1]-Offset in workpiece/base coordinate system, [2]-Offset in tool coordinate system Default 0;
    - ``offset_pos``: Pose offset value, unit [mm][°] Default [0.0,0.0,0.0,0.0,0.0,0.0]
    - ``oacc``: Acceleration scaling factor [0-100] / Physical acceleration (mm/s²), default: 100；
    - ``blendR``:-1：Blocking；0~1000：Blend radius, default: -1；
    - ``config``: Inverse solution joint space configuration, [-1]-Solve with reference to current joint positions, [0~7]-Solve according to a specific joint space configuration, default -1;
    - ``velAccParamMode``: Velocity/Acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) and acceleration (mm/s²) Default 0"
    "Return Value", "Error code 0-Success  Error- errcode"

Point-to-point motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveCart(desc_pos, tool, user, vel = 20.0, acc = 0.0, ovl = 100.0, blendT = -1.0, config = -1)``"
    "Description", "Point-to-point motion in Cartesian space"
    "Mandatory parameters", "- ``desc_pos``: target Cartesian position;
    - ``tool``: tool number, [0 to 14];
    - ``user``: artifact number, [0 to 14];"
    "Default parameters", "- ``vel``: velocity, range [0 to 100], default 20.0;
    - ``acc``: acceleration, range [0-100], not available, default 0.0; 
    - ``ovl``: velocity scaling factor, [0 to 100], default 100.0.
    - ``blendT``:[-1.0]-motion in place (blocking), [0~500]-smoothing time (non-blocking) in [ms] default -1.0;
    - ``config``: joint configuration, [-1] - solve with reference to current joint position, [0~7] - solve based on joint configuration default is -1"
    "Return Value", "Error Code Success-0 Failure- errcode"

Sample robot basic motion commands code
+++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    robot = Robot.RPC('192.168.58.2')
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    j3 = [-29.777, -84.536, 109.275, -114.075, -86.655, 74.257]
    j4 = [-31.154, -95.317, 94.276, -88.079, -89.740, 74.256]
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    desc_pos3 = [-487.434, 154.362, 308.576, 176.600, 0.268, -14.061]
    desc_pos4 = [-443.165, 147.881, 480.951, 179.511, -0.775, -15.409]
    offset_pos = [0.0] * 6
    epos = [0.0] * 4
    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    oacc = 100.0
    blendT = 0.0
    blendR = 0.0
    flag = 0
    search = 0
    blendMode = 0
    velAccMode = 0
    robot.SetSpeed(20)
    rtn = robot.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos, blendT=blendT, offset_flag=flag, offset_pos=offset_pos)
    print(f"movej errcode:{rtn}")
    rtn = robot.MoveL(desc_pos=desc_pos2, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, blendR=blendR, blendMode=blendMode, exaxis_pos=epos, search=search, offset_flag=flag, offset_pos=offset_pos,oacc=oacc, velAccParamMode=velAccMode)
    print(f"movel errcode:{rtn}")
    rtn = robot.MoveC(desc_pos_p=desc_pos3, tool_p=tool, user_p=user, vel_p=vel, acc_p=acc, exaxis_pos_p=epos, offset_flag_p=flag, offset_pos_p=offset_pos, desc_pos_t=desc_pos4, tool_t=tool, user_t=user, vel_t=vel,acc_t=acc, exaxis_pos_t=epos, offset_flag_t=flag, offset_pos_t=offset_pos, ovl=ovl, blendR=blendR, oacc=oacc, velAccParamMode=velAccMode)
    print(f"movec errcode:{rtn}")
    rtn = robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos, blendT=blendT, offset_flag=flag, offset_pos=offset_pos)
    print(f"movej errcode:{rtn}")
    rtn = robot.Circle(desc_pos_p=desc_pos3, tool_p=tool, user_p=user, vel_p=vel, acc_p=acc, exaxis_pos_p=epos, desc_pos_t=desc_pos1, tool_t=tool, user_t=user, vel_t=vel, acc_t=acc, exaxis_pos_t=epos, ovl=ovl,offset_flag=flag, offset_pos=offset_pos, oacc=oacc, blendR=-1, velAccParamMode=velAccMode)
    print(f"circle errcode:{rtn}")
    rtn = robot.MoveCart(desc_pos=desc_pos4, tool=tool, user=user, vel=vel, acc=acc,ovl=ovl, blendT=blendT, config=-1)
    print(f"MoveCart errcode:{rtn}")
    rtn = robot.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos, blendT=blendT, offset_flag=flag, offset_pos=offset_pos)
    print(f"movej errcode:{rtn}")
    rtn = robot.MoveL(desc_pos=desc_pos2, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, blendR=blendR, blendMode=blendMode, exaxis_pos=epos, search=search, offset_flag=flag, offset_pos=offset_pos, config=-1,velAccParamMode=velAccMode)
    print(f"movel errcode:{rtn}")
    rtn = robot.MoveC(desc_pos_p=desc_pos3, tool_p=tool, user_p=user, vel_p=vel, acc_p=acc, exaxis_pos_p=epos, offset_flag_p=flag, offset_pos_p=offset_pos, desc_pos_t=desc_pos4, tool_t=tool, user_t=user, vel_t=vel, acc_t=acc,exaxis_pos_t=epos, offset_flag_t=flag, offset_pos_t=offset_pos, ovl=ovl, blendR=blendR, config=-1, velAccParamMode=velAccMode)
    print(f"movec errcode:{rtn}")
    rtn = robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos, blendT=blendT, offset_flag=flag, offset_pos=offset_pos)
    print(f"movej errcode:{rtn}")
    rtn = robot.Circle(desc_pos_p=desc_pos3, tool_p=tool, user_p=user, vel_p=vel, acc_p=acc, exaxis_pos_p=epos, desc_pos_t=desc_pos1, tool_t=tool, user_t=user, vel_t=vel, acc_t=acc, exaxis_pos_t=epos, ovl=ovl, offset_flag=flag,offset_pos=offset_pos, oacc=oacc, blendR=-1, velAccParamMode=velAccMode)
    print(f"circle errcode:{rtn}")
    robot.CloseRPC()
    return 0

Spiral motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype","``NewSpiral(desc_pos, tool, user, param, joint_pos = [0.0,0.0,0.0,0.0,0.0,0.0,0.0], vel = 20.0, acc = 0.0, exaxis_pos = [0.0,0.0,0.0,0.0], ovl = 100.0, offset_flag = 0, offset_pos = [0.0,0.0,0.0,0.0,0.0,0.0], config = -1)``"
    "Description", "Spiral motion in Cartesian space"
    "Mandatory parameters", "- ``desc_pos``: target Cartesian position in [mm][°];
    - ``tool``: tool number, [0 to 14].
    - ``user``: artifact number, [0 to 14].
    - ``param=[circle_num, circle_angle, rad_init, rad_add, rotaxis_add, rot_direction, velAccMode]``: circle_num: number of spiral turns; circle_angle: spiral inclination; rad_init: initial radius of the spiral. rad_add: radius increment; rotaxis_add: rotational direction increment; rot_direction: rotational direction, 0-clockwise, 1-counterclockwise,velAccMode speed acceleration parameter mode: 0- constant angular velocity, 1- constant linear velocity;"
    "Default parameters","- ``joint_pos``: target joint position in [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls inverse kinematics to solve for the return value.
    - ``vel``: percentage of speed, [0~100] default 20.0.
    - ``acc``: percentage of acceleration, [0~100] default 100.0.
    - ``exaxis_pos``: external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0].
    - ``ovl``: velocity scaling factor, [0~100] default 100.0.
    - ``offset_flag``:[0]-no offset, [1]-offset in workpiece/base coordinate system, [2]-offset in tool coordinate system Default 0;
    - ``offset_pos``: position offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0]
    - ``config``: Reverse the joint space configuration, [-1]- calculate based on the current joint position, [0~7]- solve based on the specific joint space configuration, default -1"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j = [67.957, -81.482, 87.595, -95.691, -94.899, -9.727]
    desc_pos = [-123.142, -551.735, 430.549, 178.753, -4.757, 167.754]
    offset_pos1 = [50.0, 0.0, 0.0, -30.0, 0.0, 0.0]
    offset_pos2 = [50.0, 0.0, 0.0, -30.0, 0.0, 0.0]
    epos = [0.0] * 4
    sp = [2, 30.0, 50.0, 10.0, 10.0, 0, 1]  # [circle_num, circle_angle, rad_init, rad_add, rotaxis_add, rot_direction, velAccMode]
    tool = 0
    user = 0
    vel = 30.0
    acc = 60.0
    ovl = 100.0
    blendT = -1.0
    flag = 2
    robot.SetSpeed(20)
    rtn = robot.MoveJ(joint_pos=j, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos, blendT=blendT, offset_flag=flag, offset_pos=offset_pos1)
    print(f"movej errcode:{rtn}")
    rtn = robot.NewSpiral(desc_pos=desc_pos, tool=tool, user=user, vel=vel, acc=acc, exaxis_pos=epos, ovl=ovl, offset_flag=flag, offset_pos=offset_pos2, param=sp)
    print(f"newspiral errcode:{rtn}")
    robot.CloseRPC()
    return 0

Start of servo motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoMoveStart()``"
    "Description", "Servo motion start, used with ServoJ, ServoCart commands"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

End of servo motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoMoveEnd()``"
    "Description", "End of servo motion, used with ServoJ, ServoCart commands"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Joint space servo mode motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoJ(joint_pos, axisPos, acc = 0.0, vel = 0.0, cmdT = 0.008, filterT = 0.0, gain = 0.0, id=0)``"
    "Description", "Joint space servo mode motion"
    "Mandatory parameters", "- ``joint_pos``: target joint position in [°];
    - ``axisPos``: external axis position in mm;"
    "Default parameter", "- ``acc``: acceleration, range [0~100], not open yet, default 0.0.
    - ``vel``: velocity, range [0~100], not open, default 0.0.
    - ``cmdT``: command send cycle, unit s, recommended range [0.001~0.0016], default is 0.008.
    - ``filterT``: filter time in [s], not open, default is 0.0; ``filterT``: filter time in [s], not open, default is 0.0.
    - ``gain``: proportional amplifier for target position, not open yet, default 0.0;
    - ``id``: servoJ command ID, default 0."
    "Return Value", "Error Code Success-0 Failure- errcode"

Example of joint space servo mode motion code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j = [0.0] * 6
    epos = [0.0] * 4
    vel = 0.0
    acc = 0.0
    cmdT = 0.008
    filterT = 0.0
    gain = 0.0
    flag = 0
    count = 500
    dt = 0.1
    cmdID = 0
    ret, j = robot.GetActualJointPosDegree(flag)
    if ret == 0:
        cmdID += 1
        robot.ServoMoveStart()
        while count:
            robot.ServoJ(joint_pos=j,axisPos= epos,acc= acc,vel= vel, cmdT=cmdT, filterT=filterT, gain=gain, id=cmdID)
            j[4] += dt
            count -= 1
            time.sleep(cmdT)
            rtn,pkg = robot.GetRobotRealTimeState()
            print(f"Servoj Count {pkg.servoJCmdNum}; last pos is {pkg.lastServoTarget[0]},{pkg.lastServoTarget[1]},{pkg.lastServoTarget[2]},{pkg.lastServoTarget[3]},{pkg.lastServoTarget[4]},{pkg.lastServoTarget[5]}")

            if count < 50:
                robot.MotionQueueClear()
                print(f"After queue clear, Servoj Count {pkg.servoJCmdNum}; last pos is {pkg.lastServoTarget[0]},{pkg.lastServoTarget[1]},{pkg.lastServoTarget[2]},{pkg.lastServoTarget[3]},{pkg.lastServoTarget[4]},{pkg.lastServoTarget[5]}")
                break
        robot.ServoMoveEnd()
    else:
        print(f"GetActualJointPosDegree errcode:{ret}")
    robot.CloseRPC()

Joint torque control begins
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoJTStart()``"
    "Description", "Joint torque control begins"
    "Mandatory parameters", "NULL"
    "Default_parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Joint torque control
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoJT(torque, interval, checkFlag=0, jPowerLimit=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], jVelLimit=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])``"
    "Description", "Joint Torque Control"
    "Required Parameters", "- ``torque``: Torque for joints j1~j6, unit: Nm
                            - ``interval``: Command cycle, unit: s, range: [0.001~0.008]
                            - ``checkFlag``: Detection strategy 0-No restrictions; 1-Power limit; 2-Velocity limit; 3-Both power and velocity limits, default: 0
                            - ``jPowerLimit``: Default parameter jPowerLimit Maximum joint power limit (W), default: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                            - ``jVelLimit``: Maximum joint velocity (°/s), default: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Joint torque control is completed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoJTEnd()``"
    "Description", "Joint torque control is completed"
    "Mandatory parameters", "NULL"
    "Default_parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Sample code for joint torque control
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.DragTeachSwitch(1)
    # torques = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    error,torques = robot.GetJointTorques(1)
    robot.ServoJTStart()
    count = 100
    while count > 0:
        error = robot.ServoJT(torques, 0.001)
        count -= 1
        time.sleep(0.001)
    error = robot.ServoJTEnd()
    robot.DragTeachSwitch(0)
    robot.CloseRPC()

Joint Torque Control Code Example with Overspeed Protection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    robot = Robot.RPC('192.168.58.2')
    robot.ResetAllError()
    time.sleep(0.5)
    torques = [0.0] * 6
    rtn, torques = robot.GetJointTorques(1)
    robot.ServoJTStart()
    robot.DragTeachSwitch(1)
    checkFlag = 3
    jPowerLimit = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    jVelLimit = [181,80,80,80,80,80]
    count = 800000
    error = 0
    while count > 0:
        torques[2] = torques[2] + 0.01
        error = robot.ServoJT(torques, 0.008, checkFlag, jPowerLimit, jVelLimit)
        print(f"ServoJT rtn is {error}")
        count = count - 1
        time.sleep(0.001)
        rtn,pkg = robot.GetRobotRealTimeState()
        print(f"maincode {pkg.main_code},subcode {pkg.sub_code}")
    robot.DragTeachSwitch(0)
    error = robot.ServoJTEnd()

Cartesian Space Servo Mode Motion
++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoCart(mode, desc_pos, exaxis, pos_gain=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], acc=0.0, vel=0.0, cmdT=0.008,filterT=0.0, gain=0.0)``"
    "Description", "Cartesian space servo mode motion"
    "Required Parameters", "- ``mode``:[0]-Absolute motion (base coordinate system), [1]-Incremental motion (base coordinate system), [2]-Incremental motion (tool coordinate system);
    - ``exaxis``:Extended axis position;
    - ``desc_pos``:Target Cartesian position/Target Cartesian position increment;"
    "Default Parameters", "- ``pos_gain``:Pose increment proportionality coefficient, only effective in incremental motion, range [0~1], default [1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
    - ``acc``:Acceleration, range [0~100], temporarily not available, default 0.0;
    - ``vel``:Velocity, range [0~100], temporarily not available, default 0.0;
    - ``cmdT``:Command transmission period, unit s, recommended range [0.001~0.0016], default 0.008;
    - ``filterT``:Filter time, unit [s], temporarily not available, default 0.0;
    - ``gain``:Proportional amplifier for target position, temporarily not available, default 0.0;"
    "Return Value", "Error code Success-0 Failure- errcode"

Cartesian Space Servo Mode Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    robot = Robot.RPC('192.168.58.2')
    desc_pos_dt = [83.00800, 50.525000, 29.246, 179.629, -7.138, -166.975]
    exaxis = [100.0, 0.0, 0.0, 0.0]
    pos_gain = [0.0] * 6
    mode = 0
    vel = 0.0
    acc = 0.0
    cmdT = 0.001
    filterT = 0.0
    gain = 0.0
    flag = 0
    count = 5000
    robot.SetSpeed(20)
    while count:
        rtn = robot.ServoCart(mode, desc_pos_dt, exaxis, pos_gain, acc, vel, cmdT, filterT, gain)
        print(f"ServoCart rtn is {rtn}")
        count -= 1
        desc_pos_dt[0] += 0.01
        exaxis[0] += 0.01
    robot.CloseRPC()
    return 0

Start of spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SplineStart()``"
    "Description", "Sample movement begins"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Sample motion PTP
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SplinePTP(joint_pos, tool, user, desc_pos = [0.0,0.0,0.0,0.0,0.0,0.0,0.0], vel = 20.0, acc = 100.0, ovl = 100.0)``"
    "Description", "Sample Motion PTP"
    "Mandatory parameters", "- ``joint_pos``: target joint position in [°];
    - ``tool``: tool number, [0 to 14];
    - ``user``: artifact number, [0 to 14];"
    "Default parameters", "- ``desc_pos``: target Cartesian position in [mm][°] Default initial value [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls positive kinematics to solve for the return value.
    - ``vel``: velocity, range [0-100], default 20.0.
    - ``acc``: acceleration, range [0 to 100], default 100.0.
    - ``ovl``: velocity scaling factor, [0 to 100], default 100.0"
    "Return Value", "Error Code Success-0 Failure- errcode"

End of spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SplineEnd()``"
    "Description", "End of spline motion"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Spline motion code example
+++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    joint_points = [
        [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256],  # j1
        [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255],  # j2
        [-61.954, -84.409, 108.153, -116.316, -91.283, 74.260],  # j3
        [-89.575, -80.276, 102.713, -116.302, -91.284, 74.267]  # j4
    ]
    cart_points = [
        [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833],  # desc_pos1
        [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869],  # desc_pos2
        [-327.622, 402.230, 320.402, -178.067, 2.127, -46.207],  # desc_pos3
        [-104.066, 544.321, 327.023, -177.715, 3.371, -73.818]  # desc_pos4
    ]
    offset_pos = [0] * 6 
    epos = [0] * 4 
    tool = user = 0
    vel = acc = ovl = 100.0 
    blendT = -1.0  
    flag = 0 
    robot.SetSpeed(20)
    err1 = robot.MoveJ(joint_pos=joint_points[0],tool=tool, user=user,vel=vel)
    print(f"MoveJ 错误码: {err1}")
    robot.SplineStart()
    robot.SplinePTP(joint_pos=joint_points[0],tool=tool, user=user)
    robot.SplinePTP(joint_pos=joint_points[1],tool=tool, user=user)
    robot.SplinePTP(joint_pos=joint_points[2],tool=tool, user=user)
    robot.SplinePTP(joint_pos=joint_points[3],tool=tool, user=user)
    robot.SplineEnd()
    robot.CloseRPC()

New spline movement begins
+++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``NewSplineStart(type,averageTime=2000)``"
    "Description", "New Sample Movement Begins"
    "Mandatory parameters", "- ``type``: 0 - arc transition, 1 - given point path point"
    "Default Parameters", "- ``averageTime``: global average articulation time (ms) defaults to 2000"
    "Return Value", "Error Code Success-0 Failure- errcode"

New spline command point
+++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype","``NewSplinePoint(desc_pos,tool,user,lastFlag,joint_pos=[0.0,0.0,0.0,0.0,0.0,0.0], vel = 0.0, acc = 0.0, ovl = 100.0 ,blendR = 0.0 )``"
    "description", "new spline command point"
    "Mandatory parameters", "- ``desc_pos``: target Cartesian position in [mm][°].
    - ``tool``: tool number, [0 to 14];
    - ``user``: artifact number, [0 to 14];
    - ``lastFlag``: whether it is the last point, 0 - no, 1 - yes;"
    "Default parameters","- ``joint_pos``: target joint position in [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls inverse kinematics to solve for the return value.
    - ``vel``: velocity, range [0~100], not open yet, default is 0.0;;
    - ``acc``: acceleration, range [0 to 100], not open, default 0.0.
    - ``ovl``: velocity scaling factor, [0~100] default 100.0.
    - ``blendR``: [0~1000]-smoothing radius in [mm] default 0.0;"
    "Return Value", "Error Code Success-0 Failure- errcode"

End of new spline movement
+++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``NewSplineEnd()``"
    "Description", "End of New Sample Campaign"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Example of new spline motion code
+++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    j3 = [-61.954, -84.409, 108.153, -116.316, -91.283, 74.260]
    j4 = [-89.575, -80.276, 102.713, -116.302, -91.284, 74.267]
    j5 = [-95.228, -54.621, 73.691, -112.245, -91.280, 74.268]
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    desc_pos3 = [-327.622, 402.230, 320.402, -178.067, 2.127, -46.207]
    desc_pos4 = [-104.066, 544.321, 327.023, -177.715, 3.371, -73.818]
    desc_pos5 = [-33.421, 732.572, 275.103, -177.907, 2.709, -79.482]
    offset_pos = [0, 0, 0, 0, 0, 0]
    epos = [0, 0, 0, 0]
    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    blendT = -1.0
    flag = 0
    robot.SetSpeed(20)
    err1 = robot.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel)
    print(f"movej errcode:{err1}")
    robot.NewSplineStart(1, 2000)
    robot.NewSplinePoint(desc_pos=desc_pos1, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplinePoint(desc_pos=desc_pos2, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplinePoint(desc_pos=desc_pos3, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplinePoint(desc_pos=desc_pos4, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplinePoint(desc_pos=desc_pos5, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplineEnd()
    robot.CloseRPC()

Robot termination motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``StopMotion()``"
    "Description", "Terminate motion, use of terminate motion requires motion instruction to be non-blocking"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Robot pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``PauseMotion()``"
    "Description", "Pause motion. Using pause motion requires the motion instruction to be in a non-blocking state"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Robot resume motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``ResumeMotion()``"
    "Description", "Resume motion, using resume motion requires the motion instruction to be in a non-blocking state"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Motion pause, resume, and stop code examples
+++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j1 =[-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j5 =[-95.228, -54.621, 73.691, -112.245, -91.280, 74.268]
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos5 = [-33.421, 732.572, 275.103, -177.907, 2.709, -79.482]
    offset_pos = [0, 0, 0, 0, 0, 0]
    epos = [0, 0, 0, 0]
    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    blendT = -1.0
    flag = 0
    robot.SetSpeed(20)
    rtn = robot.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel)
    rtn = robot.MoveJ(joint_pos=j5, tool=tool, user=user, vel=vel, blendT=1)
    time.sleep(1)
    robot.PauseMotion()
    time.sleep(1)
    robot.ResumeMotion()
    time.sleep(1)
    robot.StopMotion()
    time.sleep(1)
    robot.CloseRPC()

Overall shift in points begins
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``PointsOffsetEnable(flag,offset_pos)``"
    "Description", "Point Overall Offset Begins"
    "Mandatory parameters", "- ``flag``: 0 - offset in base or work coordinate system, 2 - offset in tool coordinate system;
    - ``offset_pos``: offset in [mm][°]."
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Overall offset of points ends
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``PointsOffsetDisable()``"
    "Description", "End of overall point offset"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Point offset code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    offset_pos = [0, 0, 0, 0, 0, 0]
    offset_pos1 = [0, 0, 50, 0, 0, 0]
    epos = [0, 0, 0, 0]
    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    blendT = -1.0
    flag = 0
    robot.SetSpeed(20)
    robot.MoveJ(joint_pos=j1,tool=tool, user=user, vel=vel)
    robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel)
    time.sleep(1)
    robot.PointsOffsetEnable(flag=0, offset_pos=offset_pos1)
    robot.MoveJ(joint_pos=j1,tool=tool, user=user, vel=vel)
    robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel)
    robot.PointsOffsetDisable()
    robot.CloseRPC()

Control box motion AO start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype","``MoveAOStart(AONum,maxTCPSpeed=1000,maxAOPercent=100,zeroZoneCmp=20)``"
    "Description", "Control Box Motion AO Start"
    "Mandatory parameters", "- ``AONum``: control box AO number"
    "Default Parameters", "
    - ``maxTCPSpeed``: Maximum TCP speed value [1-5000mm/s], default 1000;
    - ``maxAOPercent``: percentage of AO corresponding to the maximum TCP speed value, default 100%;
    - ``zeroZoneCmp``: deadzone compensation value AO percentage, shaped, default 20%, range [0-100]."
    "Return Value", "Error Code Success-0 Failure- errcode"

End of control box movement AO
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``MoveAOStop()``"
    "Description", "End of control box motion AO"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

End Motion AO Start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype","``MoveToolAOStart(AONum,maxTCPSpeed=1000,maxAOPercent=100,zeroZoneCmp =20)``"
    "Description", "End Motion AO Start"
    "Mandatory parameters", "- ``AONum``: end AO number"
    "Default Parameters", "
    - ``maxTCPSpeed``: Maximum TCP speed value [1-5000mm/s], default 1000;
    - ``maxAOPercent``: percentage of AO corresponding to the maximum TCP speed value, default 100%;
    - ``zeroZoneCmp``: deadzone compensation value AO percentage, shaped, default 20%, range [0-100]."
    "Return Value", "Error Code Success-0 Failure- errcode"

End movement AO end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``MoveToolAOStop()``"
    "Description", "end movement AO end"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

AO flyshot code example
+++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    offset_pos = [0, 0, 0, 0, 0, 0]
    offset_pos1 = [0, 0, 50, 0, 0, 0]
    epos = [0, 0, 0, 0]
    tool = 0
    user = 0
    vel = 20.0
    acc = 20.0
    ovl = 100.0
    blendT = -1.0
    flag = 0
    robot.SetSpeed(20)
    robot.MoveAOStart(0, 100, 100, 20)
    robot.MoveJ(joint_pos=j1,tool=tool, user=user, vel=vel)
    robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel)
    robot.MoveAOStop()
    time.sleep(1)
    robot.MoveToolAOStart(0, 100, 100, 20)
    robot.MoveJ(joint_pos=j1,tool=tool, user=user, vel=vel)
    robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel)
    robot.MoveToolAOStop()
    robot.CloseRPC()

Start Ptp motion FIR filtering
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``PtpFIRPlanningStart(maxAcc, maxJek)``"
    "Description", "Start Ptp motion FIR filtering"
    "Mandatory parameters", "- ``maxAcc``:Maximum acceleration extremum(deg/s2)
    - ``maxJek``:Unify the extreme values of joint urgency (deg/s3)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Disable Ptp motion FIR filtering
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``PtpFIRPlanningEnd()``"
    "Description", "Disable Ptp motion FIR filtering"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

LIN, ARC motion FIR filtering is started
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``LinArcFIRPlanningStart(maxAccLin,maxAccDeg,maxJerkLin,maxJerkDeg)``"
    "Description", "LIN, ARC motion FIR filtering is started"
    "Mandatory parameter", "- ``maxAccLin``:Linear acceleration extremum(mm/s2)
    - ``maxAccDeg``:Angular acceleration extremum(deg/s2)
    - ``maxJerkLin``:Linear plus acceleration extremum(mm/s3)
    - ``maxJerkDeg``:Angle plus acceleration extremum(deg/s3)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Turn off LIN and ARC motion FIR filtering
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``LinArcFIRPlanningEnd()``"
    "Description", "Turn off LIN and ARC motion FIR filtering"
    "Mandatory parameter", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"  
    
FIR filtering code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    startjointPos = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    startjointPos = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    midjointPos = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    endjointPos = [-29.777, -84.536, 109.275, -114.075, -86.655, 74.257]
    startdescPose = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    middescPose = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    enddescPose = [-487.434, 154.362, 308.576, 176.600, 0.268, -14.061]
    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    rtn = robot.PtpFIRPlanningStart(1000.0, 1000.0)
    print(f"PtpFIRPlanningStart rtn is {rtn}")
    error = robot.MoveJ(joint_pos=startjointPos,tool= 0,user= 0,desc_pos=startdescPose,vel= 100,acc=100,ovl=100, blendT=-1.0, offset_flag=0)
    print(f"MoveJ rtn is {rtn}")
    error = robot.MoveJ(joint_pos=endjointPos,tool= 0,user= 0,desc_pos=enddescPose,vel= 100,acc=100,ovl=100, blendT=-1.0, offset_flag=0)
    print(f"MoveJ rtn is {rtn}")
    robot.PtpFIRPlanningEnd()
    print(f"PtpFIRPlanningEnd rtn is {rtn}")
    rtn = robot.LinArcFIRPlanningStart(1000, 1000, 1000, 1000)
    print(f"LinArcFIRPlanningStart rtn is {rtn}")
    error = robot.MoveL(desc_pos=startdescPose,tool= 0,user= 0, joint_pos=startjointPos,vel= 100,overSpeedStrategy=1,speedPercent=1)
    print(f"MoveL rtn is {rtn}")
    error = robot.MoveC(desc_pos_p=middescPose,tool_p= 0,user_p= 0, joint_pos_p=midjointPos,vel_p= 100,desc_pos_t=enddescPose,tool_t= 0,user_t= 0,joint_pos_t=endjointPos,vel_t= 100)
    print(f"MoveC rtn is {rtn}")
    robot.LinArcFIRPlanningEnd()
    print(f"LinArcFIRPlanningEnd rtn is {rtn}")
    robot.CloseRPC()

Acceleration smooth on
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AccSmoothStart(saveFlag_flag)``"
    "Description", "Acceleration smooth on"
    "Mandatory parameters", "- ``saveFlag_flag``: Power off and save"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Acceleration smooth closing
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AccSmoothEnd(saveFlag_flag)``"
    "Description", "Acceleration smooth closing"
    "Mandatory parameters", "- ``saveFlag_flag``: Power off and save"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Acceleration smoothing code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    startjointPos = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    endjointPos = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    startdescPose = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    enddescPose = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    rtn = robot.AccSmoothStart(0)
    print(f"AccSmoothStart rtn is {rtn}")
    robot.MoveJ(joint_pos=startjointPos,tool= 0,user= 0,vel= 100)
    robot.MoveJ(joint_pos=endjointPos,tool= 0,user= 0,vel= 100)
    rtn = robot.AccSmoothEnd(0)
    print(f"AccSmoothEnd rtn is {rtn}")

Setting the machine's specified attitude speed on
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AngularSpeedStart(ratio)``"
    "Description", "Specifies that attitude speed is on."
    "Mandatory parameters", "- ``ratio``: percentage of attitude velocity [0-300]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Specify Attitude Velocity Off
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AngularSpeedEnd()``"
    "Description", "Specify Attitude Velocity Off"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Robot specified pose velocity code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    startjointPos = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    endjointPos = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    startdescPose = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    enddescPose = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    rtn = robot.AngularSpeedStart(50)
    print(f"AngularSpeedStart rtn is {rtn}")
    robot.MoveJ(joint_pos=startjointPos, tool=0,user= 0,vel= 100)
    robot.MoveJ(joint_pos=endjointPos, tool=0,user= 0,vel= 100)
    rtn = robot.AngularSpeedEnd()
    print(f"AngularSpeedEnd rtn is {rtn}")
    robot.CloseRPC()

Odd-position protection on.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SingularAvoidStart(protectMode, minShoulderPos=100, minElbowPos=50, minWristPos=10)``"
    "Description", "Turn on odd-bit posture protection."
    "Mandatory parameters", "
    - ``protectMode``: singular position protection protection mode: 0 - articulated mode; 1 - Cartesian mode
    "
    "Default Parameters", "- ``minShoulderPos``: Shoulder singularity adjustment range (mm), default 100.0
    - ``minElbowPos``: elbow singularity adjustment range (mm), default 50.0
    - ``minWristPos``: range of wrist singularity adjustment (°), default 10.0"
    "Return Value", "- errcode Success-0 Failure- errcode"

Odd position protection off
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SingularAvoidEnd()``"
    "Description", "Turn off odd-position protection"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errcode Success-0 Failure- errcode"

Example of robot singular pose protection code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    startjointPos = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    endjointPos = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    startdescPose = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    enddescPose = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    rtn = robot.SingularAvoidStart(2, 10, 5, 5)
    print(f"SingularAvoidStart rtn is {rtn}")
    robot.MoveJ(joint_pos=startjointPos, tool=0,user= 0,vel= 100)
    robot.MoveJ(joint_pos=endjointPos, tool=0,user= 0,vel= 100)
    rtn = robot.SingularAvoidEnd()
    print(f"SingularAvoidEnd rtn is {rtn}")
    robot.CloseRPC()

Clear the motor command queue
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MotionQueueClear()``"
    "Description", "Clear the motor command queue"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errcode Success-0 Failure- errcode"

Clear Motion Command Queue
+++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveToIntersectLineStart(mainPoint, piecePoint, tool, wobj, vel, acc, ovl, oacc, moveType,mainExaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],pieceExaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],extAxisFlag=0,exaxisPos=[0.0,0.0,0.0,0.0],moveDirection=0,offset=[0.0,0.0,0.0,0.0,0.0,0.0])``"
    "Description", "Clear motion command queue"
    "Required Parameters", "
    - ``mainPoint``：Cartesian poses of 6 taught points on the main pipe
    - ``piecePoint``：Cartesian poses of 6 taught points on the branch pipe
    - ``tool``：Tool coordinate system number
    - ``wobj``：Workpiece coordinate system number
    - ``vel``：Velocity percentage
    - ``acc``：Acceleration percentage
    - ``ovl``：Velocity scaling factor
    - ``oacc``：Acceleration scaling factor
    - ``moveType``：Motion type; 0-PTP; 1-LIN
    - ``mainExaxisPos``：Extended axis positions for 6 taught points on the main pipe, default [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    - ``pieceExaxisPos``：Extended axis positions for 6 taught points on the branch pipe, default [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    - ``extAxisFlag``：Whether to enable extended axis; 0-Disable; 1-Enable
    - ``exaxisPos``：Start point extended axis position [0.0,0.0,0.0,0.0]
    - ``moveDirection``：Motion direction; 0-Clockwise; 1-Counterclockwise
    - ``offset``：Offset value
    "
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure- errcode"

Intersecting Line Motion
+++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveIntersectLine(mainPoint, piecePoint, tool, wobj, vel, acc, ovl, oacc, moveDirection,mainExaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],pieceExaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],extAxisFlag=0,exaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],offset=[0.0,0.0,0.0,0.0,0.0,0.0])``"
    "Description", "Intersecting line motion"
    "Required Parameters", "
    - ``mainPoint``：Cartesian poses of 6 taught points on the main pipe
    - ``piecePoint``：Cartesian poses of 6 taught points on the branch pipe
    - ``tool``：Tool coordinate system number
    - ``wobj``：Workpiece coordinate system number
    - ``vel``：Velocity percentage
    - ``acc``：Acceleration percentage
    - ``ovl``：Velocity scaling factor
    - ``oacc``：Acceleration scaling factor
    - ``moveDirection``：Motion direction; 0-Clockwise; 1-Counterclockwise
    - ``mainExaxisPos``：Extended axis positions for 6 taught points on the main pipe, default [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    - ``pieceExaxisPos``：Extended axis positions for 6 taught points on the branch pipe, default [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    - ``extAxisFlag``：Whether to enable extended axis; 0-Disable; 1-Enable
    - ``exaxisPos``：Start point extended axis position [0.0,0.0,0.0,0.0]
    - ``offset``：Offset value
    "
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure- errcode"

Robot Intersecting Line Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos: 

    from fairino import Robot
    import time
    robot = Robot.RPC('192.168.58.2')
    mainPoint = [[0.0] * 6 for _ in range(6)]
    piecePoint = [[0.0] * 6 for _ in range(6)]
    mainExaxisPos = [[0.0] * 4 for _ in range(6)]
    pieceExaxisPos = [[0.0] * 4 for _ in range(6)]
    extAxisFlag = 1
    exaxisPos = [[0.0] * 4 for _ in range(4)]
    offset = [0.0, 2.0, 30.0, -2.0, 0.0, 0.0]
    mainPoint[0] = [490.004, -383.194, 402.735, -9.332, -1.528, 69.594]
    mainPoint[1] = [444.950, -407.117, 389.011, -5.546, -2.196, 65.279]
    mainPoint[2] = [445.168, -463.605, 355.759, -1.544, -10.886, 57.104]
    mainPoint[3] = [507.529, -485.385, 343.013, -0.786, -4.834, 61.799]
    mainPoint[4] = [554.390, -442.647, 367.701, -4.761, -10.181, 64.925]
    mainPoint[5] = [532.552, -394.003, 396.467, -13.732, -13.592, 67.411]
    mainExaxisPos[0] = [-29.996, 0.000, 0.000, 0.000]
    mainExaxisPos[1] = [-29.996, 0.000, 0.000, 0.000]
    mainExaxisPos[2] = [-29.996, 0.000, 0.000, 0.000]
    mainExaxisPos[3] = [-29.996, 0.000, 0.000, 0.000]
    mainExaxisPos[4] = [-29.996, 0.000, 0.000, 0.000]
    mainExaxisPos[5] = [-29.996, 0.000, 0.000, 0.000]
    piecePoint[0] = [505.571, -192.408, 316.759, 38.098, 37.051, 139.447]
    piecePoint[1] = [533.837, -201.558, 332.340, 34.644, 42.339, 137.748]
    piecePoint[2] = [530.386, -225.085, 373.808, 35.431, 45.111, 137.560]
    piecePoint[3] = [485.646, -229.195, 383.778, 33.870, 45.173, 137.064]
    piecePoint[4] = [460.551, -212.161, 354.256, 28.856, 45.602, 135.930]
    piecePoint[5] = [474.217, -197.124, 324.611, 42.469, 41.133, 148.167]
    pieceExaxisPos[0] = [-29.996, -0.000, 0.000, 0.000]
    pieceExaxisPos[1] = [-29.996, -0.000, 0.000, 0.000]
    pieceExaxisPos[2] = [-29.996, -0.000, 0.000, 0.000]
    pieceExaxisPos[3] = [-29.996, -0.000, 0.000, 0.000]
    pieceExaxisPos[4] = [-29.996, -0.000, 0.000, 0.000]
    pieceExaxisPos[5] = [-29.996, -0.000, 0.000, 0.000]
    exaxisPos[0] = [-29.996, -0.000, 0.000, 0.000]
    exaxisPos[1] = [-44.994, 90.000, 0.000, 0.000]
    exaxisPos[2] = [-59.992, 0.002, 0.000, 0.000]
    exaxisPos[3] = [-44.994, -89.997, 0.000, 0.000]
    tool = 2
    wobj = 0
    vel = 100.0
    acc = 100.0
    ovl = 12.0
    oacc = 12.0
    moveType = 1
    moveDirection = 1
    rtn = robot.MoveToIntersectLineStart(mainPoint=mainPoint, mainExaxisPos=mainExaxisPos, piecePoint=piecePoint, pieceExaxisPos=pieceExaxisPos, extAxisFlag=extAxisFlag,exaxisPos=exaxisPos[0], tool=tool, wobj=wobj, vel=vel, acc=acc, ovl=ovl, oacc=oacc, moveType=moveType, moveDirection=moveDirection, offset=offset)
    print(f"MoveToIntersectLineStart rtn is {rtn}")
    rtn = robot.MoveIntersectLine(mainPoint=mainPoint, mainExaxisPos=mainExaxisPos, piecePoint=piecePoint, pieceExaxisPos=pieceExaxisPos, extAxisFlag=extAxisFlag, exaxisPos=exaxisPos, tool=tool,wobj=wobj, vel=vel, acc=acc, ovl=5.0, oacc=5.0, moveDirection=moveDirection, offset=offset)
    print(f"MoveIntersectLine rtn is {rtn}")
    robot.CloseRPC()

Stationary Air Motion
+++++++++++++++++++++++++++++++++
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveStationary()``"
    "Description", "Stationary Air Motion"
    "Required Parameters", "None"
    "Default Parameters", "None"
    "Return Value", "- Error code. Success - 0, Failure - errcode"
 
Stationary Air Motion Code Example
++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos: 

    from fairino import Robot
    import time
    robot = Robot.RPC('192.168.58.2')
    rtn = robot.LaserSensorRecordandReplay(0, 10, 1, 0, 0.1, 1, 0, 10, 100)
    print(f"LaserSensorRecordandReplay rtn is {rtn}")
    rtn = robot.MoveStationary()
    print(f"MoveStationary rtn is {rtn}")
    rtn = robot.LaserSensorRecord1(0, 10)
    print(f"LaserSensorRecordandReplay rtn is {rtn}")
    robot.CloseRPC()
    return 0

