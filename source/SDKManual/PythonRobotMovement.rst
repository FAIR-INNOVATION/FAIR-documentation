Movement
============

.. toctree::
    :maxdepth: 5

robot spotting
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

jog point and click
---------------------------------------------------------------

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
-----------------------------------------------------------------------------------------------------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``StopJOG(ref)``"
    "description", "jog nudging deceleration stop"
    "Mandatory parameters", "- ``ref``: 1 - joint point stop, 3 - base coordinate system point stop, 5 - tool coordinate system point stop, 9 - workpiece coordinate system point stop"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Immediate stop for jog taps
-----------------------------------------------------------------------------------------------------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ImmStopJOG()``"
    "Description", "jog nudging stops immediately"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Robot single-axis pointing
    robot.StartJOG(0,1,0,20.0,20.0,30.0) # single-joint motion, StartJOG is a non-blocking command, other motion commands (including StartJOG) will be discarded if received during motion.
    time.sleep(1)
    # Robot single-axis pointing deceleration stop
    ret = robot.StopJOG(1)
    print(ret)
    # Robot single-axis pointing stops immediately
    robot.ImmStopJOG()
    robot.StartJOG(0,2,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(0,3,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(0,4,1,20.0,vel=40)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(0,5,1,20.0,acc=50)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(0,6,1,20.0,20.0,30.0)
    time.sleep(1)
    robot.ImmStopJOG()
    # Base coordinates
    robot.StartJOG(2,1,0,20.0) #point motion in base coordinate system
    time.sleep(1) 
    # Robot single-axis pointing stops immediately
    robot.ImmStopJOG()
    robot.StartJOG(2,1,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,2,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,3,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,4,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,5,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,6,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    # Tool coordinates
    robot.StartJOG(4,1,0,20.0,20.0,100.0) #point motion in tool coordinate system
    time.sleep(1)
    # Robot single-axis pointing stops immediately
    robot.ImmStopJOG()
    robot.StartJOG(4,1,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,2,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,3,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,4,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,5,1,20.0,vel=10.0,acc=20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,6,1,20.0,acc=40.0)
    time.sleep(1)
    robot.ImmStopJOG()
    # Workpiece coordinates
    robot.StartJOG(8,1,0,20.0,20.0,100.0) #point motion in work coordinate system
    time.sleep(1)
    # Robot single-axis pointing stops immediately
    robot.ImmStopJOG()
    robot.StartJOG(8,1,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,2,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,3,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,4,1,20.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,5,1,20.0,vel=30.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,6,1,20.0,20.0,acc=90.0)
    time.sleep(1)
    robot.ImmStopJOG()

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

code example
-------------------------------------------------------------------------------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    joint_pos4 = [-83.24, -96.476, 93.688, -114.079, -62, -100]
    joint_pos5 = [-43.24, -70.476, 93.688, -114.079, -62, -80]
    joint_pos6 = [-83.24, -96.416, 43.188, -74.079, -80, -10]
    tool = 0 #Tool coordinate system number
    user = 0 #Workpiece coordinate system number
    ret = robot.MoveJ(joint_pos4, tool, user, vel=30) #joint space motion
    print("Joint space motion point 4: error code", ret)
    ret = robot.MoveJ(joint_pos5, tool, user)
    print("Joint space motion point 5: error code", ret)
    robot.MoveJ(joint_pos6, tool, user, offset_flag=1, offset_pos=[10,10,10,0,0,0])
    print("Joint space motion point 6: error code", ret)

Cartesian linear motion in space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype","``MoveL(desc_pos, tool, user, joint_pos = [0.0,0.0,0.0,0.0,0.0,0.0,0.0], vel = 20.0, acc = 0.0 , ovl = 100.0, blendR = -1.0, exaxis_pos = [0.0,0.0, 0.0,0.0], search = 0, offset_flag = 0, offset_pos = [0.0,0.0,0.0,0.0,0.0,0.0],overSpeedStrategy=0,speedPercent=10)``"
    "Description", "Cartesian linear motion in space"
    "Mandatory parameters", "- ``desc_pos``: target Cartesian position in [mm][°];
    - ``tool``: tool number, [0 to 14];
    - ``user``: artifact number, [0 to 14];"
    "Default parameters","- ``joint_pos``: target joint position in [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls inverse kinematics to solve for the return value.
    - ``vel``: percentage of speed, [0~100] default 20.0;
    - ``acc``: acceleration percentage, [0~100], not open Default 0.0;
    - ``ovl``: velocity scaling factor, [0~100] default 100.0;
    - ``blendR``:blendR:[-1.0]-movement in place (blocking), [0~1000]-smoothing radius (non-blocking) in [mm] default -1.0;
    - ``exaxis_pos``: external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0].
    - ``search``: [0] - no wire search, [1] - wire search;
    - ``offset_flag``:offset_flag:[0]-no offset, [1]-offset in workpiece/base coordinate system, [2]-offset in tool coordinate system Default 0;
    - ``offset_pos``: position offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0]
    - ``overSpeedStrategy``: over speed handling strategy, 0 - strategy off; 1 - standard; 2 - stop on error when over speeding; 3 - adaptive speed reduction, default 0
    - ``speedPercent``: Percentage of allowable speed reduction threshold [0-100], default 10%
    "
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    desc_pos1 = [36.794,-475.119, 65.379, -176.938, 2.535, -179.829]
    desc_pos2 = [136.794,-475.119, 65.379, -176.938, 2.535, -179.829]
    desc_pos3 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
    tool = 0 #Tool coordinate system number
    user = 0 #Workpiece coordinate system number
    ret = robot.MoveL(desc_pos1, tool, user) # Cartesian space linear motion
    print("Cartesian space linear motion point 1: error code", ret) 
    robot.MoveL(desc_pos2, tool, user, vel=20, acc=100)
    print("Cartesian space linear motion point 2: error code", ret) 
    robot.MoveL(desc_pos3, tool, user, offset_flag=1, offset_pos=[10,10,10,0,0,0])
    print("Cartesian space linear motion point 3: error code", ret)

Circular motion in Cartesian space
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype","``MoveC(desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p =[0.0,0.0,0.0,0.0,0.0,0.0], joint_pos_t =[0.0,0.0,0.0,0.0 ,0.0,0.0], vel_p = 20.0,acc_p=100.0, exaxis_pos_p = [0.0,0.0,0.0,0.0], offset_flag_p = 0, offset_pos_p = [0.0,0.0,0.0,0.0,0.0,0.0], vel_t= 20.0, acc_t= 100.0,exaxis_pos_t=[0.0,0.0,0.0,0.0], offset_flag_t = 0, offset_pos_t = [0.0,0.0,0.0,0.0,0.0,0.0,0.0], ovl = 100.0, blendR = -1.0)``"
    "Description", "Circular motion in Cartesian space"
    "Mandatory parameters", "- ``desc_pos_p``: path point Cartesian position in [mm][°];
    - ``tool_p``: pathpoint tool number, [0~14].
    - ``user_p``: pathpoint artifact number, [0~14].
    - ``desc_pos_t``: Cartesian position of the target point in [mm][°].
    - ``tool_t``: tool number, [0 to 14];
    - ``user_t``: artifact number, [0~14];"
    "Default parameters","- ``joint_pos_p``: path point joint position in [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls inverse kinematics to solve for the return value.
    - ``joint_pos_t``: target point joint position in [°] default initial value [0.0,0.0,0.0,0.0,0.0,0.0,0.0] default value calls inverse kinematics to solve for the return value.
    - ``vel_p``: path point velocity percentage, [0~100] default 20.0.
    - ``acc_p``: path point acceleration percentage, [0~100] not open yet, default 0.0; ``acc_p``: path point acceleration percentage, [0~100] not open yet, default 0.0.
    - ``exaxis_pos_p``: path point external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0];
    - ``offset_flag_p``: whether the path point is offset [0] - no offset, [1] - offset in the workpiece/base coordinate system, [2] - offset in the tool coordinate system Default 0;
    - ``vel_t``: percentage of velocity at target point, [0~100] default 20.0; ``vel_t``: percentage of velocity at target point, [0~100] default 20.0.
    - ``acc_t``: target point acceleration percentage, [0~100] not open yet default 0.0.
    - ``exaxis_pos_t``: target point external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0];
    - ``offset_flag_t``: whether the target point is offset or not [0]-no offset, [1]-offset in the workpiece/base coordinate system, [2]-offset in the tool coordinate system Default 0;
    - ``offset_pos_t``: target point attitude offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0].
    - ``ovl:``: velocity scaling factor, [0~100] default 100.0.
    - ``blendR``:[-1.0]-motion in place (blocking), [0~1000]-smoothing radius (non-blocking) in [mm] default -1.0;"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
-------------------------------------------------------------------------------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    desc_pos1 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
    desc_posc1 = [266.794,-455.119, 65.379, -176.938, 2.535, -179.829] #MoveC transition point
    desc_posc2 = [286.794,-475.119, 65.379, -176.938, 2.535, -179.829] #MoveC target points
    tool = 0#Tool coordinate system number
    user = 0 #Workpiece coordinate system number
    ret = robot.MoveL(desc_pos1, tool, user, vel=30, acc=100)
    print("Linear motion in Cartesian space: error code", ret) 
    ret = robot.MoveC(desc_posc1, tool, user, desc_posc2,tool, user) #Cartesian space circular motion
    print("Cartesian space circular motion: error code", ret)

Whole circle motion in Cartesian space
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype","``Circle(desc_pos_p,tool_p,user_p,desc_pos_t,tool_t,user_t,joint_pos_p=[0.0,0.0,0.0,0.0,0.0,0.0], joint_pos_t = [0.0,0.0,0.0, 0.0,0.0,0.0,0.0], vel_p = 20.0, acc_p=0.0, exaxis_pos_p= [0.0,0.0, 0.0,0.0], vel_t=20.0, acc_t = 0.0, exaxis_pos_t = [0.0,0.0,0.0,0.0], ovl=100.0, offset_flag=0, offset_pos= [0.0,0.0,0.0,0.0,0.0,0.0])``"
    "Description", "Cartesian Space Integral Circular Motion"
    "Mandatory parameters", "- ``desc_pos_p``: path point Cartesian position in [mm][°];
    - ``tool_p``: tool number, [0 to 14];
    - ``user_p``: artifact number, [0~14];
    - ``desc_pos_t``: Cartesian position of the target point in [mm][°];
    - ``tool_t``: tool number, [0 to 14];
    - ``user_t``: artifact number, [0~14];"
    "Default parameters","- ``joint_pos_p``: path point joint position in [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls inverse kinematics to solve for the return value.
    - ``joint_pos_t``: target point joint position in [°] default initial value [0.0,0.0,0.0,0.0,0.0,0.0,0.0] default value calls inverse kinematics to solve for the return value.
    - ``vel_p``: velocity percentage, [0~100] default 20.0.
    - ``acc_p``: path point acceleration percentage, [0~100] not open yet default 0.0.
    - ``exaxis_pos_p``: path point external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0];
    - ``vel_t``: percentage of velocity at target point, [0~100] default 20.0; ``vel_t``: percentage of velocity at target point, [0~100] default 20.0.
    - ``acc_t``: target point acceleration percentage, [0~100] not open yet default 0.0.
    - ``exaxis_pos_t``: point external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0]
    - ``ovl``: velocity scaling factor, [0~100] default 100.0.
    - ``offset_flag``: whether or not to offset [0] - no offset, [1] - offset in the workpiece/base coordinate system, [2] - offset in the tool coordinate system Default 0;
    - ``offset_pos``: position offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0]"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
-------------------------------------------------------------------------------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    desc_pos2 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
    desc_posc3 = [256.794,-435.119, 65.379, -176.938, 2.535, -179.829] #Circle path points
    desc_posc4 = [286.794,-475.119, 65.379, -176.938, 2.535, -179.829] #Circle target points
    tool = 0#Tool coordinate system number
    user = 0 #Workpiece coordinate system number
    robot.MoveL(desc_pos2, tool, user, vel=40, acc=100)
    print("Linear motion in Cartesian space: error code", ret) 
    ret = robot.Circle(desc_posc3, tool, user, desc_posc4, tool, user, vel_t=40, offset_flag=1, offset_pos=[5,10,15,0,0,1]) #Cartesian space circular motion
    print("Circular motion in Cartesian space: error code", ret) #Circular motion in Cartesian space

Spiral motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype","``NewSpiral(desc_pos, tool, user, param, joint_pos = [0.0,0.0,0.0,0.0,0.0,0.0,0.0], vel = 20.0, acc = 0.0, exaxis_pos = [0.0,0.0,0.0,0.0], ovl = 100.0, offset_flag = 0, offset_pos = [0.0,0.0,0.0,0.0,0.0,0.0])``"
    "Description", "Spiral motion in Cartesian space"
    "Mandatory parameters", "- ``desc_pos``: target Cartesian position in [mm][°];
    - ``tool``: tool number, [0 to 14].
    - ``user``: artifact number, [0 to 14].
    - ``param=[circle_num, circle_angle, rad_init, rad_add, rotaxis_add, rot_direction]``: circle_num: number of spiral turns; circle_angle: spiral inclination; rad_init: initial radius of the spiral. rad_add: radius increment; rotaxis_add: rotational direction increment; rot_direction: rotational direction, 0-clockwise, 1-counterclockwise;"
    "Default parameters","- ``joint_pos``: target joint position in [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls inverse kinematics to solve for the return value.
    - ``vel``: percentage of speed, [0~100] default 20.0.
    - ``acc``: percentage of acceleration, [0~100] default 100.0.
    - ``exaxis_pos``: external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0].
    - ``ovl``: velocity scaling factor, [0~100] default 100.0.
    - ``offset_flag``:[0]-no offset, [1]-offset in workpiece/base coordinate system, [2]-offset in tool coordinate system Default 0;
    - ``offset_pos``: position offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0]"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------------------------------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    desc_pos_spiral= [236.794,-475.119, -65.379, -176.938, 2.535, -179.829]#Spiral target point
    #spiral parameters [circle_num,circle_angle,rad_init,rad_add,rotaxis_add,rot_direction]
    # circle_num: the number of circles in the spiral, circle_angle: the inclination of the spiral, rad_init: the initial radius of the spiral, rad_add: the increment of the radius, # circle_num: the number of circles in the spiral, circle_angle: the inclination of the spiral, rad_init: the initial radius of the spiral, rad_add: the radius increment.
    # rotaxis_add: rotaxis direction increment, rot_direction: direction of rotation, 0 - clockwise, 1 - counterclockwise
    param = [5.0,10,30,10,5,0]
    tool = 0#Tool coordinate system number
    user = 0 #Workpiece coordinate system number
    ret = robot.NewSpiral(desc_pos_spiral, tool, user, param,vel=40 ) #Cartesian space spiral motion
    print("Spiral motion in Cartesian space: error code", ret)

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

    "Prototype", "``ServoJ(joint_pos, axisPos, acc = 0.0, vel = 0.0, cmdT = 0.008, filterT = 0.0, gain = 0.0)``"
    "Description", "Joint space servo mode motion"
    "Mandatory parameters", "- ``joint_pos``: target joint position in [°];
    - ``axisPos``: external axis position in mm;"
    "Default parameter", "- ``acc``: acceleration, range [0~100], not open yet, default 0.0.
    - ``vel``: velocity, range [0~100], not open, default 0.0.
    - ``cmdT``: command send cycle, unit s, recommended range [0.001~0.0016], default is 0.008.
    - ``filterT``: filter time in [s], not open, default is 0.0; ``filterT``: filter time in [s], not open, default is 0.0.
    - ``gain``: proportional amplifier for target position, not open yet, default 0.0;"
    "Return Value", "Error Code Success-0 Failure- errcode"

Servo-mode motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoCart(mode, desc_pos, pos_gain = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] , acc = 0.0, vel = 0.0, cmdT = 0.008, filterT = 0.0, gain = 0.0)``"
    "Description", "Servo mode motion in Cartesian space"
    "Mandatory parameters", "- ``mode``: [0]-absolute motion (base coordinate system), [1]-incremental motion (base coordinate system), [2]-incremental motion (tool coordinate system);
    - ``desc_pos``: target Cartesian position/target Cartesian position increment;"
    "default_parameters", "- ``pos_gain``: bit-pose incremental scale factor, valid only for incremental motion, range [0~1], default [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
    - ``acc``: acceleration, range [0 to 100], not open, default 0.0.
    - ``vel``: velocity, range [0~100], not open, default 0.0.
    - ``cmdT``: command send cycle, unit s, recommended range [0.001~0.0016], default is 0.008.
    - ``filterT``: filter time in [s], not open, default is 0.0; ``filterT``: filter time in [s], not open, default is 0.0.
    - ``gain``: proportional amplifier for target position, not open yet, default 0.0;"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------------------------------------------------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error,joint_pos = robot.GetActualJointPosDegree()
    print("Current joint position of the robot",joint_pos)
    joint_pos = [joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5]]
    error_joint = 0
    count =100
    error = robot.ServoMoveStart() #ServoMoveStart
    print("Servo motion start error code",error)
    while(count).
        error = robot.ServoJ(joint_pos=joint_pos,axisPos=[0,0,0,0,0,0]) #joint space servo mode motion
        if error!=0.
            error_joint =error
        joint_pos[0] = joint_pos[0] + 0.1 # 1-axis movement 0.1 degree each time, 100 movements
        count = count - 1
        time.sleep(0.008)
    print("Joint space servo mode motion error code",error_joint)
    error = robot.ServoMoveEnd() # servo move end
    print("Servo motion end error code",error) 
    mode = 2 #[0]-absolute motion (base coordinate system), [1]-incremental motion (base coordinate system), [2]-incremental motion (tool coordinate system)
    n_pos = [0.0,0.0,0.5,0.0,0.0,0.0] #Cartesian space bit-position increments
    error,desc_pos = robot.GetActualTCPPose()
    print("Current Cartesian position of the robot",desc_pos)
    count = 100
    error_cart =0
    error = robot.ServoMoveStart() #ServoMoveStart
    print("Servo motion start error code",error)
    while(count).
        error = robot.ServoCart(mode, n_pos, vel=40) #Cartesian space servo mode motion
        if error!=0.
            error_cart =error
        count = count - 1
        time.sleep(0.008)
    print("Cartesian space servo mode motion error code", error_cart)
    error = robot.ServoMoveEnd() # servo move end
    print("Servo motion end error code",error)

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

code example
-------------------------------------------------------------------------------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    desc_pos7 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
    desc_pos8 = [236.794,-575.119, 165.379, -176.938, 2.535, -179.829]
    desc_pos9 = [236.794,-475.119, 265.379, -176.938, 2.535, -179.829]
    tool = 0 #Tool coordinate system number
    user = 0 #Workpiece coordinate system number
    robot.MoveCart(desc_pos7, tool, user)
    print("Cartesian space point-to-point motion point 7: error code", ret) 
    robot.MoveCart(desc_pos8, tool, user, vel=30)
    print("Cartesian space point-to-point motion point 8: error code", ret) 
    robot.MoveCart(desc_pos9, tool, user,)
    print("Cartesian space point-to-point motion point 9: error code", ret)

Robot spline motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Start of spline motion
-----------------------------------------------------------------------------------------------------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SplineStart()``"
    "Description", "Sample movement begins"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Sample motion PTP
---------------------------------------------------------------------------------------------------------------------------
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
----------------------------------------------------------------------------------------------------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SplineEnd()``"
    "Description", "End of spline motion"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    tool = 0 #Tool coordinate system number
    user = 0 #Workpiece coordinate system number
    joint_pos1 = [116.489,-85.278,111.501,-112.486,-85.561,24.693]
    joint_pos2 = [86.489,-65.278,101.501,-112.486,-85.561,24.693]
    joint_pos3 = [116.489,-45.278,91.501,-82.486,-85.561,24.693]
    ret = robot.SplineStart() # Spline motion start
    print("Sample motion started: error code", ret)
    ret = robot.SplinePTP(joint_pos1, tool, user) #spline motion PTP
    print("Sample motion PTP motion point 1: error code", ret) 
    ret = robot.SplinePTP(joint_pos2, tool, user) #spline motion PTP
    print("Sample motion PTP motion point 2: error code", ret) 
    ret = robot.SplinePTP(joint_pos3, tool, user) #spline motion PTP
    print("Sample motion PTP motion point 3: error code", ret)
    ret = robot.SplineEnd() # end of spline motion
    print("End of spline motion: error code", ret)

Robotics new spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
New spline movement begins
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. versionchanged:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``NewSplineStart(type,averageTime=2000)``"
    "Description", "New Sample Movement Begins"
    "Mandatory parameters", "- ``type``: 0 - arc transition, 1 - given point path point"
    "Default Parameters", "- ``averageTime``: global average articulation time (ms) defaults to 2000"
    "Return Value", "Error Code Success-0 Failure- errcode"

End of new spline movement
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``NewSplineEnd()``"
    "Description", "End of New Sample Campaign"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

new spline command point
----------------------------------------------------------------------------------------------------------------------------
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


code example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    tool = 0 #Tool coordinate system number
    user = 0 #Workpiece coordinate system number
    lastFlag= 0 # whether it is the last point, 0 - no, 1 - yes
    desc_pos4 = [236.794,-375.119, 65.379, -176.938, 2.535, -179.829]
    desc_pos5 = [236.794,-275.119, 165.379, -176.938, 2.535, -179.829]
    desc_pos6 = [286.794,-375.119, 265.379, -176.938, 2.535, -179.829]
    ret = robot.NewSplineStart(1) # new spline motion start
    print("New spline motion started: error code", ret)
    ret = robot.NewSplinePoint(desc_pos4, tool, user, lastFlag)#New Spline Instruction point
    print("New Sample Instruction Point 4: Error Code", ret) 
    ret = robot.NewSplinePoint(desc_pos5, tool, user, lastFlag, vel=30)#NewSplineInstructionPoint
    print("New Sample Instruction Point 5: Error Code", ret) 
    lastFlag = 1
    ret = robot.NewSplinePoint(desc_pos6, tool, user, lastFlag, vel=30)#NewSplineInstructionPoint
    print("New Sample Instruction Point 6: Error Code", ret) 
    ret = robot.NewSplineEnd() # end of new spline motion
    print("End of new spline motion: error code", ret)

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

code example
-------------------------------------------------------------------------------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    desc_pos1 = [-187.519, 319.248, 397, -157.278, -31.188, 107.199]
    desc_pos2 = [-187.519, 310.248, 297, -157.278, -31.188, 107.199]
    joint_pos1 = [-83.24, -96.476, 93.688, -114.079, -62, -100]
    tool = 0 #Tool coordinate system number
    user = 0 #Workpiece coordinate system number
    ret = robot.MoveL(desc_pos1, tool, user, joint_pos=joint_pos1) # linear motion in Cartesian space
    print("Cartesian space linear motion point 1: error code", ret)
    ret = robot.StopMotion() #Stop motion
    print("Terminating motion: error code", ret) 
    robot.MoveL(desc_pos2, tool, user, vel=40, acc=100)
    print("Cartesian space linear motion point 2: error code", ret)

Robot points are shifted overall
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Overall shift in points begins
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
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
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``PointsOffsetDisable()``"
    "Description", "End of overall point offset"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    desc_pos3 = [-127.519, 256.248, 312, -147.278, -51.588, 107.199]
    desc_pos4 = [-140.519, 219.248, 300, -137.278, -11.188, 127.199]
    desc_pos5 = [-187.519, 319.248, 397, -157.278, -31.188, 107.199]
    desc_pos6 = [-207.519, 229.248, 347, -157.278, -31.188, 107.199]
    tool = 0 #Tool coordinate system number
    user = 0 #Workpiece coordinate system number
    flag = 1 #0 - offset in base/workpiece coordinate system, 2 - offset in tool coordinate system
    offset_pos = [10,20,30,0,0,0] #bit position offset
    ret = robot.PointsOffsetEnable(flag,offset_pos)
    print("Point overall offset started: error code", ret)
    robot.MoveL(desc_pos3, tool, user, offset_flag=1, offset_pos=[10,10,10,0,0,0])
    print("Cartesian space linear motion point 3: error code", ret) 
    robot.MoveL(desc_pos4, tool, user, vel=30, acc=100)
    print("Cartesian space linear motion point 4: error code", ret) 
    robot.MoveL(desc_pos5, tool, user)
    print("Cartesian space linear motion point 5: error code", ret) 
    ret = robot.PointsOffsetDisable()
    print("End of overall point offset: error code", ret)

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
    
code example
---------------------------------------------------------------------------------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Control box motion AO start
    error = robot.MoveAOStart(0,100,98,1)
    print("MoveAOStart",error)
    error,joint_pos = robot.GetActualJointPosDegree()
    print("GetActualJointPosDegree",error,joint_pos)
    joint_pos[0] = joint_pos[0]+10
    # Robot joint motion
    error = robot.MoveJ(joint_pos,1,1)
    print("MoveJ",error)
    time.sleep(3)
    # Control box motion AO stop
    error = robot.MoveAOStop()
    print("MoveAOStop",error)

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
        
code example
---------------------------------------------------------------------------------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # End movement AO starts
    error = robot.MoveToolAOStart(0,100,98,1)
    print("MoveToolAOStart",error)
    error,desc_pos = robot.GetActualTCPPose()
    print("GetActualTCPPose",error,desc_pos)
    desc_pos[2] = desc_pos[2]-50
    # Linear motion in Cartesian space
    error = robot.MoveL(desc_pos,1,1)
    print("MoveL",error)
    time.sleep(3)
    # End motion AO stops
    error = robot.MoveToolAOStop()
    print("MoveToolAOStop",error)
    
End movement AO end
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``MoveToolAOStop()``"
    "description", "end movement AO end"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"