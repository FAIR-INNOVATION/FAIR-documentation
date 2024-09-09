Movement
=================

.. toctree:: 
    :maxdepth: 5

Robot Jog
+++++++++++++

jog Jog
---------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``StartJOG(ref,nb,dir,vel,acc,max_dis)``"
    "Description", "jog Jog"
    "Required parameter", "- ``ref``:0-joint jogging, 2-base coordinate system jogging, 4-tool coordinate system jogging, 8-workpiece coordinate system jogging;
    - ``nb``:1-1joint(x-axis), 2-2joint(y-axis), 3-3join(z-axis), 4-4joint(rx), 5-5joint (ry), 6-6joint(rz);
    - ``dir``:0-negative direction, 1-positive direction;
    - ``vel``:Speed percentage,[0~100];
    - ``acc``:Acceleration percentage,[0~100];
    - ``max_dis``:Maximum angle/distance for a single jog,unit[° or mm]"
    "Optional parameter", "- ``vel``:Speed percentage,[0~100], default to is 20.0;
                           - ``acc``:Acceleration percentage,[0~100], default to is 20.0"
    "Return value", "Errcode: Success -0  Failed -errcode"

jog jog deceleration stops
----------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``StopJOG(ref)``"
    "Description", "jog jog deceleration stops"
    "Required parameter", "- ``ref``:1-joint jog stop, 3-base coordinate system jog stop, 5-tool coordinate system jog stop, 9-workpiece coordinate system jog stop"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

jog jog immediately stops
-----------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ImmStopJOG()``"
    "Description", "jog jog immediately stops"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"
     
Code example
^^^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')

    # Robot single axis point
    robot.StartJOG(0,1,0,20.0,20.0,30.0)     # Single joint motion, StartJOG is a non blocking command, and other motion commands (including StartJOG) received during motion will be discarded
    time.sleep(1)
    #Robot single axis jog deceleration stop
    # ret = robot.StopJOG(1)
    # print(ret)
    #Immediate stop of robot single axis jog
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
    # Base coordinate
    robot.StartJOG(2,1,0,20.0)  #Jogging in the base coordinate system
    time.sleep(1)
    #Robot single axis jog deceleration stop
    # robot.StopJOG(3)
    #Immediate stop of robot single axis jog
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

    # Tool coordinate
    robot.StartJOG(4,1,0,20.0,20.0,100.0)  #Point in the tool coordinate system
    time.sleep(1)
    #Robot single axis jog deceleration stop
    # robot.StopJOG(5)
    #Immediate stop of robot single axis jog
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
    # Job coordinate
    robot.StartJOG(8,1,0,20.0,20.0,100.0)  #Point in the workpiece coordinate system
    time.sleep(1)
    #Robot single axis jog deceleration stop
    # robot.StopJOG(9)
    # #Immediate stop of robot single axis jog
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
++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveJ(joint_pos, tool, user, desc_pos = [0.0,0.0,0.0,0.0,0.0,0.0], vel = 20.0, acc = 0.0, ovl = 100.0, exaxis_pos = [0.0,0.0,0.0,0.0], blendT = -1.0, offset_flag = 0, offset_pos = [0.0,0.0,0.0,0.0,0.0,0.0])``"
    "Description", "Joint space motion"
    "Required parameter", "- ``joint_pos``:Target joint position, unit[°];
    - ``tool``:Tool number,[0~14];
    - ``user``:Workpiece number,[0~14];"
    "Optional parameter", "- ``desc_pos``:Target Cartesian pose,unit[mm], Default initial value is [0.0,0.0,0.0,0.0,0.0, 0.0,0.0], default value calls forward kinematics to solve for return value;
    - ``vel``:Speed percentage,[0~100], default to is 20.0;
    - ``acc``:Acceleration percentage,[0~100], default to is 0.0;
    - ``ovl``:Speed scaling factor,[0~100], default to is 100.0;
    - ``exaxis_pos``:External axis 1 position to external axis 4 position, default to is[0.0,0.0,0.0,0.0];
    - ``blendT``:[-1.0]-Motion in place (blocked), [0-500]-Smoothing time (non blocked),unit[ms] , default to is -1.0;
    - ``offset_flag``:[0]-no offset, [1]-offset under workpiece/base coordinate system,[2]-offset under tool coordinate system, default to is 0;
    - ``offset_pos``:Pose offset,unit[mm][°] , default to is [0.0,0.0,0.0,0.0,0.0,0.0]"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    joint_pos4 = [-83.24, -96.476, 93.688, -114.079, -62, -100]
    joint_pos5 = [-43.24, -70.476, 93.688, -114.079, -62, -80]
    joint_pos6 = [-83.24, -96.416, 43.188, -74.079, -80, -10]
    tool = 0 # Tool number
    user = 0 # Workpiece number
    ret = robot.MoveJ(joint_pos4, tool, user, vel=30)   # Joint space motionPTP, the actual test is based on field data ,Tool number and Workpiece number
    print("Joint space motion, Point4:errcode", ret)
    ret = robot.MoveJ(joint_pos5, tool, user)
    print("Joint space motion, Point5:errcode ", ret)
    robot.MoveJ(joint_pos6, tool, user, offset_flag=1, offset_pos=[10,10,10,0,0,0])
    print("Joint space motion, Point6:errcode ", ret)

Linear motion in Cartesian space
++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveL(desc_pos, tool, user, joint_pos = [0.0,0.0,0.0,0.0,0.0,0.0], vel = 20.0, acc = 0.0 , ovl = 100.0, blendR = -1.0, exaxis_pos = [0.0,0.0,0.0,0.0], search = 0, offset_flag = 0, offset_pos = [0.0,0.0,0.0,0.0,0.0,0.0],overSpeedStrategy=0,speedPercent=10)``"
    "Description", "Linear motion in Cartesian space"
    "Required parameter", "- ``desc_pos``:Target Cartesian pose,unit[mm][°];
    - ``tool``:Tool number,[0~14];
    - ``user``:Workpiece number,[0~14];"
    "Optional parameter", "- ``joint_pos``:Target joint position, unit[°], Default initial value is [0.0,0.0,0.0,0.0,0.0, 0.0,0.0], default value calls inverse kinematics to solve for return value;
    - ``vel``:Speed percentage,[0~100], default to is 20.0;;
    - ``acc``:Acceleration percentage,[0~100], default to is 0.0;
    - ``ovl``:Speed scaling factor,[0~100], default to is 100.0;
    - ``blendR``:[-1.0]-motion in place (blocked), [0-1000]-smooth radius(non blocked),unit[mm] , default to is -1.0;
    - ``exaxis_pos``:External axis 1 position to external axis 4 position, default to is[0.0,0.0,0.0,0.0];
    - ``search``:[0]-non welding wire positioning, [1]-welding wire positioning ,default to is 0;
    - ``offset_flag``:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system, default to is 0;
    - ``offset_pos``:Pose offset,unit[mm][°] , default to is [0.0,0.0,0.0,0.0,0.0,0.0]
    - ``overSpeedStrategy``:Overspeed handling policy, 0-policy off; 1-standard; 2-error stop on overspeed; 3-adaptive speed reduction, default is 0
    - ``speedPercent``:Percentage of allowable speed reduction threshold [0-100], default 10%
    "
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
--------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    desc_pos1 = [36.794,-475.119, 65.379, -176.938, 2.535, -179.829]
    desc_pos2 = [136.794,-475.119, 65.379, -176.938, 2.535, -179.829]
    desc_pos3 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
    tool = 0 # Tool number
    user = 0 # Workpiece number
    ret = robot.MoveL(desc_pos1, tool, user)   # Rectilinear motion in Cartesian space
    print("Rectilinear motion in Cartesian space, Point1:errcode ", ret) 
    robot.MoveL(desc_pos2, tool, user, vel=20, acc=100)
    print("Rectilinear motion in Cartesian space, Point 2:errcode ", ret) 
    robot.MoveL(desc_pos3, tool, user, offset_flag=1, offset_pos=[10,10,10,0,0,0])
    print("Rectilinear motion in Cartesian space, Point 3:errcode ", ret)

Circular arc motion in Cartesian space
++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveC(desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p= [0.0,0.0,0.0, 0.0,0.0,0.0],joint_pos_t=[0.0,0.0,0.0,0.0,0.0,0.0], vel_p = 20.0, acc_p= 100.0,exaxis_pos_p=[0.0,0.0,0.0,0.0],offset_flag_p=0,offset_pos_p= [0.0,0.0,0.0,0.0,0.0,0.0], vel_t= 20.0, acc_t=100.0,exaxis_pos_t=[0.0,0.0,0.0,0.0], offset_flag_t = 0, offset_pos_t = [0.0,0.0,0.0, 0.0,0.0,0.0], ovl = 100.0, blendR = -1.0)``"
    "Description", "Circular arc motion in Cartesian space"
    "Required parameter", "- ``desc_pos_p``:Path point Cartesian pose,unit[mm][°];
    - ``tool_p``:Tool number,[0~14];
    - ``user_p``:Workpiece number,[0~14];
    - ``desc_pos_t``:Cartesian pose of the target point,unit[mm][°];
    - ``tool_t``:Tool number,[0~14];
    - ``user_t``:Workpiece number,[0~14];"
    "Optional parameter", "- ``joint_pos_p``:Path point joint position,unit[°], Default initial value is [0.0,0.0,0.0, 0.0,0.0, 0.0,0.0], default value calls inverse kinematics to solve for return value;
    - ``joint_pos_t``:Target point joint position,unit[°],Default initial value is [0.0,0.0,0.0, 0.0,0.0, 0.0,0.0], default value calls inverse kinematics to solve for return value;
    - ``vel_p``:Speed percentage,[0~100], default to is 20.0;
    - ``acc_p``:Acceleration percentage,[0~100], default to is 0.0;
    - ``exaxis_pos_p``:External axis 1 position to external axis 4 position, default to is[0.0,0.0,0.0,0.0];
    - ``offset_flag_p``:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system, default to is 0;
    - ``offset_pos_p``:Pose offset,unit[mm][°] , default to is [0.0,0.0,0.0,0.0,0.0,0.0];
    - ``vel_t``:Speed percentage,[0~100], default to is 20.0;
    - ``acc_t``:Acceleration percentage,[0~100], default to is 0.0;
    - ``exaxis_pos_t``:External axis 1 position to external axis 4 position, default to is[0.0,0.0,0.0,0.0];
    - ``offset_flag_t``:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system, default to is 0;
    - ``offset_pos_t``:Pose offset,unit[mm][°] , default to is [0.0,0.0,0.0,0.0,0.0,0.0];
    - ``ovl``:Speed scaling factor,[0~100], default to is 100.0;
    - ``blendR``:[-1.0]-motion in place (blocked), [0-1000]-smooth radius(non blocked),unit[mm] , default to is -1.0"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
-------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    desc_pos1 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
    desc_posc1 = [266.794,-455.119, 65.379, -176.938, 2.535, -179.829] #MoveC Path point
    desc_posc2 = [286.794,-475.119, 65.379, -176.938, 2.535, -179.829] #MoveC Target point
    tool = 0 # Tool number
    user = 0 # Workpiece number
    ret = robot.MoveL(desc_pos1, tool, user, vel=30, acc=100)
    print("Linear motion in Cartesian space: errorcode ", ret) 
    ret = robot.MoveC(desc_posc1, tool, user, desc_posc2,tool, user)  # Circular arc motion in Cartesian space
    print("Circular ar motion in Cartesian space:errorcode", ret)

Circular motion in Cartesian space
++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``Circle(desc_pos_p,tool_p,user_p,desc_pos_t,tool_t,user_t,joint_pos_p=[0.0,0.0,0.0,0.0,0.0, 0.0], joint_pos_t = [0.0,0.0,0.0,0.0,0.0,0.0], vel_p = 20.0, acc_p=0.0, exaxis_pos_p= [0.0,0.0, 0.0,0.0], vel_t=20.0, acc_t = 0.0, exaxis_pos_t =[0.0,0.0,0.0,0.0], ovl=100.0, offset_flag=0, offset_pos= [0.0,0.0,0.0,0.0,0.0,0.0])``"
    "Description", "Circular motion in Cartesian space"
    "Required parameter", "- ``desc_pos_p``:Path point Cartesian pose,unit[mm][°];
    - ``tool_p``:Tool number,[0~14];
    - ``user_p``:Workpiece number,[0~14];
    - ``desc_pos_t``:Cartesian pose of the target point,unit[mm][°];
    - ``tool_t``:Tool number,[0~14];
    - ``user_t``:Workpiece number,[0~14];"
    "Optional parameter", "- ``joint_pos_p``:Path point joint position,unit[°], Default initial value is [0.0,0.0,0.0,0.0,0.0, 0.0,0.0], default value calls inverse kinematics to solve for return value;
    - ``joint_pos_t``:arget point joint position,unit[°],Default initial value is [0.0,0.0,0.0,0.0,0.0, 0.0,0.0], default value calls inverse kinematics to solve for return value;
    - ``vel_p``:Speed percentage,[0~100], default to is 20.0;
    - ``acc_p``:Acceleration percentage,[0~100], default to is 0.0;
    - ``exaxis_pos_p``:External axis 1 position to external axis 4 position, default to is[0.0,0.0,0.0,0.0];
    - ``desc_pos_t``:Cartesian pose of the target point,unit[mm][°];
    - ``vel_t``:Speed percentage,[0~100], default to is 20.0;
    - ``acc_t``:Acceleration percentage,[0~100], default to is 0.0;
    - ``exaxis_pos_t``:External axis 1 position to external axis 4 position, default to is[0.0,0.0,0.0,0.0]
    - ``ovl``:Speed scaling factor,[0~100], default to is 100.0;
    - ``offset_flag``:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system, default to is 0;
    - ``offset_pos``:Pose offset,unit[mm][°] , default to is [0.0,0.0,0.0,0.0,0.0,0.0]"
    "Return value", "-Errcode: Success -0  Failed -errcode"

Code example
-------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    desc_pos2 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
    desc_posc3 = [256.794,-435.119, 65.379, -176.938, 2.535, -179.829]   #Circle Path point
    desc_posc4 = [286.794,-475.119, 65.379, -176.938, 2.535, -179.829]  #Circle Target point
    tool = 0 # Tool number
    user = 0 # Workpiece number
    robot.MoveL(desc_pos2, tool, user, vel=40, acc=100)
    print("Linear motion in Cartesian space: errorcode ", ret) 
    ret = robot.Circle(desc_posc3, tool, user, desc_posc4, tool, user, vel_t=40, offset_flag=1, offset_pos=[5,10,15,0,0,1])  # Circular motion in Cartesian space
    print("Circular motion in Cartesian space:errcode", ret)    #Circular motion in Cartesian space

Spiral motion in Cartesian space
++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``NewSpiral(desc_pos, tool, user, param, joint_pos = [0.0,0.0,0.0,0.0,0.0,0.0], vel = 20.0, acc = 0.0, exaxis_pos = [0.0,0.0,0.0,0.0], ovl = 100.0, offset_flag = 0, offset_pos = [0.0,0.0,0.0,0.0,0.0,0.0])``"
    "Description", "Spiral motion in Cartesian space"
    "Required parameter", "- ``desc_pos``:Target Cartesian pose,unit[mm][°];
    - ``tool``:Tool number,[0~14];
    - ``user``:Workpiece number,[0~14];
    - ``param``:[circle_num,circle_angle,rad_init,rad_add,rotaxis_add,rot_direction]
                circle_num: number of coils;
                circle_angle: helix angle;
                rad_init: initial radius of the helix;
                rad_add: radius increment;
                rotaxis_add: axis direction increment;
                rot_direction: rotation direction, 0-clockwise, 1-counterclockwise;"
    "Optional parameter", "- ``joint_pos``:Target joint position,,unit[°], Default initial value is [0.0,0.0,0.0,0.0,0.0, 0.0,0.0], default value calls inverse kinematics to solve for return value;
    - ``vel``:Speed percentage,[0~100], default to is 20.0;
    - ``acc``:Acceleration percentage,[0~100], default to is 0.0;
    - ``exaxis_pos``:External axis 1 position to external axis 4 position, default to is[0.0,0.0,0.0,0.0]
    - ``ovl``:Speed scaling factor,[0~100];
    - ``offset_flag``:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system, default to is 0;
    - ``offset_pos``:Pose offset,unit[mm][°] , default to is [0.0,0.0,0.0,0.0,0.0,0.0]"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    desc_pos_spiral= [236.794,-475.119, -65.379, -176.938, 2.535, -179.829]#Spiral target point
    # Spiral param [circle_num,circle_angle,rad_init,rad_add,rotaxis_add,rot_direction]
    param = [5.0,10,30,10,5,0]
    tool = 0 # Tool number
    user = 0 # Workpiece number
    ret = robot.NewSpiral(desc_pos_spiral, tool, user, param,vel=40 )  # Spiral motion in Cartesian space
    print("Spiral motion in Cartesian space:errcode", ret)

Servo motion start
++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoMoveStart()``"
    "Description", "Start of servo motion, used with ServoJ and ServoCart commands"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Servo motion end
++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoMoveEnd()``"
    "Description", "Servo motion end, used with ServoJ and ServoCart commands"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Joint space servo mode motion
++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoJ(joint_pos, acc = 0.0, vel = 0.0, cmdT = 0.008, filterT = 0.0, gain = 0.0)``"
    "Description", "Joint space servo mode motion"
    "Required parameter", "- ``joint_pos``:Target joint position, unit[°];"
    "Optional parameter", "- ``acc``:Acceleration, range[0~100],temporarily closed,default to 0.0;
    - ``vel``: Speed, range[0~100],temporarily closed,default to 0.0;
    - ``cmdT``:Instruction Cycle,unit[s],[0.001~0.016], default to 0.008;
    - ``filterT``:Filtering time,unit[s], default to 0.0;
    - ``gain``:Proportional amplifier for target position, default to 0.0"
    "Return value", "Errcode: Success -0  Failed -errcode"

Cartesian space servo mode motion
++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoCart(mode, desc_pos, pos_gain = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0] , acc = 0.0, vel = 0.0, cmdT = 0.008, filterT = 0.0, gain = 0.0)``"
    "Description", "Cartesian space servo mode motion"
    "Required parameter", "- ``mode``:[0]-absolute motion (base coordinate system), [1]-incremental motion (base coordinate system), [2]-incremental motion (tool coordinate system);
    - ``desc_pos``:Target Cartesian Position/Target Cartesian Position Increment;"
    "Optional parameter", "- ``pos_gain``:Pose increment ratio coefficient, only effective in incremental motion, range[0~1], default to[1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
    - ``acc``:Acceleration, range[0~100],temporarily closed,default to 0.0;
    - ``vel``: Speed, range[0~100],temporarily closed,default to 0.0;
    - ``cmdT``:Instruction Cycle,unit[s],[0.001~0.016], default to 0.008;
    - ``filterT``:Filtering time,unit[s], default to 0.0;
    - ``gain``:Proportional amplifier for target position, default to 0.0"
    "Return value", "Success -0  Failed -errcode"

Code example
--------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error,joint_pos = robot.GetActualJointPosDegree()
    print("Current joint position of robot :",joint_pos)
    joint_pos = [joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5]]
    error_joint = 0
    count =100
    error = robot.ServoMoveStart()  #Servo motion start
    print("The errcode of Servo motion start",error)
    while(count):
        error = robot.ServoJ(joint_pos)   # Joint space servo mode motion
        if error!=0:
            error_joint =error
        joint_pos[0] = joint_pos[0] + 0.1  # 1 axis movement 0.1 degrees each time, movement 100 times
        count = count - 1
        time.sleep(0.008)
    print("The errcode of joint space servo mode motion ",error_joint)
    error = robot.ServoMoveEnd()  #Servo motion end
    print("Servo motion end",error) 
    mode = 2  
    n_pos = [0.0,0.0,0.5,0.0,0.0,0.0]   # Target Cartesian Position Increment;
    error,desc_pos = robot.GetActualTCPPose()
    print("Current Cartesian position of robot ",desc_pos)
    count = 100
    error_cart =0
    error = robot.ServoMoveStart()  #Servo motion start
    print("The errcode of Servo motion start",error)
    while(count):
        error = robot.ServoCart(mode, n_pos, vel=40)   # Cartesian space servo mode motion
        if error!=0:
            error_cart =error
        count = count - 1
        time.sleep(0.008)
    print("The errcode of cartesian space servo mode motion", error_cart)
    error = robot.ServoMoveEnd()  #Servo motion end
    print("The errcode of servo motion end",error)

Point-to-point motion in Cartesian space
++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveCart(desc_pos, tool, user, vel = 20.0, acc = 0.0, ovl = 100.0, blendT = -1.0, config = -1)``"
    "Description", "Point-to-point motion in Cartesian space"
    "Required parameter", "- ``desc_pos``:Target Cartesian position;
    - ``tool``:Tool number,[0~14];
    - ``user``:Workpiece number,[0~14];"
    "Optional parameter", "- ``vel``: Speed, range[0~100],temporarily closed,default to 20.0;
    - ``acc``:Acceleration, range[0~100],temporarily closed,default to 0.0;
    - ``ovl``:Speed scaling factor,[0~100] ,default to 100.0;
    - ``blendT``:[-1.0]-Motion in place (blocked), [0-500]-Smoothing time (non blocked),unit[ms] ,default to -1.0;
    - ``config``:Joint configuration, [-1]-refer to the current joint position for solution, [0-7]-solve based on joint configuration,default to -1"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
-------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    desc_pos7 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
    desc_pos8 = [236.794,-575.119, 165.379, -176.938, 2.535, -179.829]
    desc_pos9 = [236.794,-475.119, 265.379, -176.938, 2.535, -179.829]
    tool = 0 #Tool number
    user = 0 #Workpiece number
    robot.MoveCart(desc_pos7, tool, user)
    print("Point-to-point motion in Cartesian space: ", ret) 
    robot.MoveCart(desc_pos8, tool, user, vel=30)
    print("Point-to-point motion in Cartesian space: ", ret) 
    robot.MoveCart(desc_pos9, tool, user,)
    print("Point-to-point motion in Cartesian space: ", ret)

Robot spline motion
+++++++++++++++++++++

Spline motion start
------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SplineStart()``"
    "Description", "Spline motion start"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Spline motion PTP
-------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SplinePTP(joint_pos, tool, user, desc_pos = [0.0,0.0,0.0,0.0,0.0,0.0],  vel = 20.0,  acc = 100.0, ovl = 100.0)``"
    "Description", "Spline motion PTP"
    "Required parameter", "- ``joint_pos``:Target joint position, unit[°];
    - ``tool``:Tool number,[0~14];
    - ``user``:Workpiece number,[0~14];"
    "Optional parameter", "- ``desc_pos``:arget Cartesian pose,unit[mm], Default initial value is [0.0,0.0,0.0,0.0,0.0, 0.0,0.0], default value calls forward kinematics to solve for return value;
    - ``vel``: Speed, range[0~100],temporarily closed,default to 20.0;
    - ``acc``:Acceleration, range[0~100],temporarily closed,default to 100.0;
    - ``ovl``:Speed scaling factor,[0~100], default to 100.0"
    "Return value", "Errcode: Success -0  Failed -errcode"

Spline motion end
-----------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SplineEnd()``"
    "Description", "Spline motion end"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    tool = 0 #Tool number
    user = 0 #Workpiece number
    joint_pos1 = [116.489,-85.278,111.501,-112.486,-85.561,24.693]
    joint_pos2 = [86.489,-65.278,101.501,-112.486,-85.561,24.693]
    joint_pos3 = [116.489,-45.278,91.501,-82.486,-85.561,24.693]
    ret = robot.SplineStart() #	Spline motion start
    print(" Spline motion start: errcode ", ret)
    ret = robot.SplinePTP(joint_pos1, tool, user)   # Spline motion PTP
    print("Spline motion PTP: errcode ", ret) 
    ret = robot.SplinePTP(joint_pos2, tool, user)   # Spline motion PTP
    print("Spline motion PTP: errcode ", ret) 
    ret = robot.SplinePTP(joint_pos3, tool, user)   # Spline motion PTP
    print("Spline motion PTP: errcode ", ret)
    ret = robot.SplineEnd() # Spline motion end
    print("Spline motion end", ret)

Robot New Spline Motion
++++++++++++++++++++++++++++++++

New spline motion start
-----------------------------------

.. versionchanged:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``NewSplineStart(type, averageTime)``"
    "Description", "New spline motion start"
    "Required parameter", "``type``:0-arc transition, 1-given point position path point"
    "Optional parameter", "``averageTime``: global average connection time (ms), default to 2000"
    "Return value", "Errcode: Success -0  Failed -errcode"


New spline motion end
----------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``NewSplineEnd()``"
    "Description", "New spline motion end"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"


New Spline Instruction Points
------------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``NewSplinePoint(desc_pos,tool,user,lastFlag,joint_pos=[0.0,0.0,0.0,0.0,0.0,0.0], vel = 0.0, acc = 0.0, ovl = 100.0 ,blendR = 0.0 )``"
    "Description", "New Spline Instruction Points"
    "Required parameter", "- ``desc_pos``:Target Cartesian pose,unit[mm][°];
    - ``tool``:Tool number,[0~14];
    - ``user``:Workpiece number,[0~14];
    - ``lastFlag``:Is it the last point, 0-No, 1-Yes"
    "Optional parameter", "- ``joint_pos``:Target joint position, unit[°], Default initial value is [0.0,0.0,0.0,0.0,0.0, 0.0,0.0], default value calls inverse kinematics to solve for return value;
    - ``vel``: Speed, range[0~100],temporarily closed,default to 20.0;
    - ``acc``:Acceleration, range[0~100],temporarily closed,default to 100;
    - ``ovl``:Speed scaling factor,[0~100], default to 100;
    - ``blendR``: blendR: [0-1000]-smooth radius,unit[mm] default to 0"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
^^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    tool = 0 #Tool number
    user = 0 #Workpiece number
    lastFlag= 0 # Is it the last point, 0-No, 1-Yes;
    desc_pos4 = [236.794,-375.119, 65.379, -176.938, 2.535, -179.829]
    desc_pos5 = [236.794,-275.119, 165.379, -176.938, 2.535, -179.829]
    desc_pos6 = [286.794,-375.119, 265.379, -176.938, 2.535, -179.829]
    ret = robot.NewSplineStart(1) # New spline motion start
    print("New spline motion start:errcode", ret)
    ret = robot.NewSplinePoint(desc_pos4, tool, user, lastFlag)# New Spline Instruction Points
    print("New Spline Instruction Points:errcode ", ret) 
    ret = robot.NewSplinePoint(desc_pos5, tool, user, lastFlag, vel=30)# New Spline Instruction Points
    print("New Spline Instruction Points:errcode ", ret) 
    lastFlag = 1
    ret = robot.NewSplinePoint(desc_pos6, tool, user, lastFlag, vel=30)# New Spline Instruction Points
    print("New Spline Instruction Points:errcode ", ret) 
    ret = robot.NewSplineEnd() # New spline motion end
    print("New spline motion end:errcode ", ret)

Robot terminates motion
+++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``StopMotion()``"
    "Description", "To terminate motion, use the termination motion instructions as non-blocking state"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
-------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    desc_pos1 = [-187.519, 319.248, 397, -157.278, -31.188, 107.199]
    desc_pos2 = [-187.519, 310.248, 297, -157.278, -31.188, 107.199]
    joint_pos1 = [-83.24, -96.476, 93.688, -114.079, -62, -100]
    tool = 0 #Tool number
    user = 0 #Workpiece number
    ret = robot.MoveL(desc_pos1, tool, user, joint_pos=joint_pos1)   # Linear motion in Cartesian space
    print("Linear motion in Cartesian space:errcode", ret)
    ret = robot.StopMotion()  # Stop Motion
    print("Stop Motion: errcode ", ret) 
    robot.MoveL(desc_pos2, tool, user, vel=40, acc=100)
    print("Linear motion in Cartesian space:errcode", ret)

Overall displacement of robot points
+++++++++++++++++++++++++++++++++++++++++
Starting point overall offset
----------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``PointsOffsetEnable(flag,offset_pos)``"
    "Description", "Starting point overall offset"
    "Required parameter", "- ``flag``:0-offset under base coordinate or workpiece coordinate system, 2-offset under tool coordinate system;
    - ``offset_pos``:Offset,unit[mm][°]."
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

The overall offset of the point ends
----------------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``PointsOffsetDisable()``"
    "Description", "The overall offset of the point ends"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
^^^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    desc_pos3 = [-127.519, 256.248, 312, -147.278, -51.588, 107.199]
    desc_pos4 = [-140.519, 219.248, 300, -137.278, -11.188, 127.199]
    desc_pos5 = [-187.519, 319.248, 397, -157.278, -31.188, 107.199]
    desc_pos6 = [-207.519, 229.248, 347, -157.278, -31.188, 107.199]
    tool = 0 #Tool number
    user = 0 #Workpiece number
    flag = 1 
    offset_pos = [10,20,30,0,0,0]  
    ret = robot.PointsOffsetEnable(flag,offset_pos)
    print("Starting point overall offset:errcode", ret)
    robot.MoveL(desc_pos3, tool, user, offset_flag=1, offset_pos=[10,10,10,0,0,0])
    print("Linear motion in Cartesian space:errcode ", ret) 
    robot.MoveL(desc_pos4, tool, user, vel=30, acc=100)
    print("Linear motion in Cartesian space:errcode ", ret) 
    robot.MoveL(desc_pos5, tool, user)
    print("Linear motion in Cartesian space:errcode ", ret) 
    ret = robot.PointsOffsetDisable()
    print("The overall offset of the point ends:errcode ", ret)
    
Control box movement AO starts
++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``MoveAOStart(AONum,maxTCPSpeed=1000,maxAOPercent=100,zeroZoneCmp=20)``"
    "Description", "Control box movement AO starts"
    "Required parameter", "- ``AONum``:Control box AO num"
    "Optional parameter", "
    - ``maxTCPSpeed``:Indicates the maximum TCP speed [1-5000mm/s]. The default value is 1000;
    - ``maxAOPercent``:Indicates the AO percentage corresponding to the maximum TCP speed. The default value is 100;
    - ``zeroZoneCmp``:Dead zone compensation value AO percentage, integer, default is 20, range [0-100]."
    "Return value", "Errcode: Success -0  Failed -errcode"
    
Code example
---------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    # Control box movement AO starts
    error = robot.MoveAOStart(0,100,98,1)
    print("MoveAOStart",error)
    error,joint_pos = robot.GetActualJointPosDegree()
    print("GetActualJointPosDegree",error,joint_pos)
    joint_pos[0] = joint_pos[0]+10
    # Robot joint motion
    error = robot.MoveJ(joint_pos,1,1)
    print("MoveJ",error)
    time.sleep(3)
    # Control box movement AO stops
    error = robot.MoveAOStop()
    print("MoveAOStop",error)

Control box movement AO stops
+++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``MoveAOStop()``"
    "Description", "Control box movement AO stops"
    "Required parameter", "NULL"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0  Failed -errcode"

Tool movement AO starts
+++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``MoveToolAOStart(AONum,maxTCPSpeed=1000,maxAOPercent=100,zeroZoneCmp =20)``"
    "Description", "Tool movement AO starts"
    "Required parameter", "- ``AONum``:末端AO编号"
    "Optional parameter", "
    - ``maxTCPSpeed``:Indicates the maximum TCP speed [1-5000mm/s]. The default value is 1000;
    - ``maxAOPercent``:Indicates the AO percentage corresponding to the maximum TCP speed. The default value is 100;
    - ``zeroZoneCmp``:Dead zone compensation value AO percentage, integer, default is 20, range [0-100]."
    "Return value", "Errcode: Success -0  Failed -errcode"
        
Code example
------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # onnection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #Tool movement AO stars
    error = robot.MoveToolAOStart(0,100,98,1)
    print("MoveToolAOStart",error)
    error,desc_pos = robot.GetActualTCPPose()
    print("GetActualTCPPose",error,desc_pos)
    desc_pos[2] = desc_pos[2]-50
    #Rectilinear motion in Cartesian space
    error = robot.MoveL(desc_pos,1,1)
    print("MoveL",error)
    time.sleep(3)
    #Tool movement AO stops
    error = robot.MoveToolAOStop()
    print("MoveToolAOStop",error)
    
Tool movement AO stops
--------------------------
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``MoveToolAOStop()``"
    "Description", "Tool movement AO stops"
    "Required parameter", "NULL"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0  Failed -errcode"
