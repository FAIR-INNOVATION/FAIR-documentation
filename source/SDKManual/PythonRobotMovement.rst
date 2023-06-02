Robot Movement
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
    "Parameter", "- ``ref``：0-joint jogging, 2-base coordinate system jogging, 4-tool coordinate system jogging, 8-workpiece coordinate system jogging；
    - ``nb``：1-1joint(x-axis), 2-2joint(y-axis), 3-3join(z-axis), 4-4joint(rx), 5-5joint (ry), 6-6joint(rz);
    - ``dir``：0-negative direction, 1-positive direction;
    - ``vel``：Speed percentage，[0~100];
    - ``acc``：Acceleration percentage，[0~100];
    - ``max_dis``：Maximum angle/distance for a single jog，unit[° or mm]"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

jog jog immediately stops
----------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``StopJOG(ref)``"
    "Description", "jog jog immediately stops"
    "Parameter", "- ``ref``：1-joint jog stop, 3-base coordinate system jog stop, 5-tool coordinate system jog stop, 9-workpiece coordinate system jog stop"
    "Return value", "- Success：[0]
    - Failed：[errcode]"


jog jog immediately stops
-----------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ImmStopJOG()``"
    "Description", "jog jog immediately stops"
    "Parameter", "Nothing"
    "Return value", "- Success：[0]
    - Failed：[errcode]"
     
Code example
^^^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:
    :emphasize-lines: 6,9,11

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    # Robot single axis point
    robot.StartJOG(0,1,0,20.0,20.0,30.0)    # Single joint motion, StartJOG is a non blocking command, and other motion commands (including StartJOG) received during motion will be discarded
    time.sleep(1)
    #Robot single axis jog deceleration stop
    # robot.StopJOG(1)
    #Immediate stop of robot single axis jog
    robot.ImmStopJOG()
    robot.StartJOG(0,2,1,20.0,20.0,30.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(0,3,1,20.0,20.0,30.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(0,4,1,20.0,20.0,30.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(0,5,1,20.0,20.0,30.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(0,6,1,20.0,20.0,30.0)
    time.sleep(1)
    robot.ImmStopJOG()
    # Base coordinate
    robot.StartJOG(2,1,0,20.0,20.0,100.0)  #Jogging in the base coordinate system
    time.sleep(1)
    #Robot single axis jog deceleration stop
    # robot.StopJOG(2)
    # #Immediate stop of robot single axis jog
    robot.ImmStopJOG()
    robot.StartJOG(2,1,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,2,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,3,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,4,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,5,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,6,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    # Tool coordinate
    robot.StartJOG(4,1,0,20.0,20.0,100.0)  #Point in the tool coordinate system
    time.sleep(1)
    #Robot single axis jog deceleration stop
    # robot.StopJOG(5)
    # #Immediate stop of robot single axis jog
    robot.ImmStopJOG()
    robot.StartJOG(4,1,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,2,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,3,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,4,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,5,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,6,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    # Job coordinate
    robot.StartJOG(8,1,0,20.0,20.0,100.0)  #Point in the workpiece coordinate system
    time.sleep(1)
    #Robot single axis jog deceleration stop
    # robot.StopJOG(9)
    # #Immediate stop of robot single axis jog
    robot.ImmStopJOG()
    robot.StartJOG(8,1,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,2,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,3,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,4,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,5,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,6,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()


Joint space motion
++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveJ(joint_pos,desc_pos,tool,user,vel,acc,ovl,exaxis_pos,blendT,offset_flag,offset_pos)``"
    "Description", "Joint space motion"
    "Parameter", "- ``joint_pos``:Target joint position, unit[°]；
    - ``desc_pos``:Target Cartesian pose，unit[mm][°]；
    - ``tool``:TOOL No，[0~14]；
    - ``user``:Workpiece number，[0~14]；
    - ``vel``:Speed percentage，[0~100]；
    - ``acc``:Acceleration percentage，[0~100]，temporarily closed；
    - ``ovl``:Speed scaling factor，[0~100]；
    - ``exaxis_pos``:External axis 1 position to external axis 4 position；
    - ``blendT``:[-1.0]-Motion in place (blocked), [0-500]-Smoothing time (non blocked)，unit[ms]；
    - ``offset_flag``:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system；
    - ``offset_pos``:Pose offset，unit[mm][°]"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
------------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 13,14

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    J1=[-168.847,-93.977,-93.118,-80.262,88.985,11.831]
    P1=[-558.082,27.343,208.135,-177.205,-0.450,89.288]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[1.000,1.000,1.000,1.000,1.000,1.000]
    J2=[168.968,-93.977,-93.118,-80.262,88.986,11.831]
    P2=[-506.436,236.053,208.133,-177.206,-0.450,67.102]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[1.000,1.000,1.000,1.000,1.000,1.000]
    robot.MoveJ(J1,P1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)    #Joint space motionPTP,TOOL No1，the actual test is based on field data and TOOL No
    robot.MoveJ(J2,P2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
    time.sleep(2)
    j1 = robot.GetInverseKin(0,P1,-1)       #In the case of Cartesian space coordinates only, the inverse kinematic interface can be used to solve the joint position
    print(j1)
    j1 = [j1[1],j1[2],j1[3],j1[4],j1[5],j1[6]]
    robot.MoveJ(j1,P1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1) 
    j2 = robot.GetInverseKin(0,P2,-1)
    print(j2)
    j2 = [j2[1],j2[2],j2[3],j2[4],j2[5],j2[6]]
    robot.MoveJ(j2,P2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
    time.sleep(2)
    p1 = robot.GetForwardKin(J1)       #The forward kinematic interface can be used to solve Cartesian space coordinates with only joint positions
    print(p1)
    p1 = [p1[1],p1[2],p1[3],p1[4],p1[5],p1[6]]
    robot.MoveJ(J1,p1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1) 
    p2 = robot.GetForwardKin(J2)
    print(p2)
    p2 = [p2[1],p2[2],p2[3],p2[4],p2[5],p2[6]]
    robot.MoveJ(J2,p2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)

Linear motion in Cartesian space
++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveL(joint_pos,desc_pos,tool,user,vel,acc,ovl,blendR,exaxis_pos,search,offset_flag,offset_pos)``"
    "Description", "Linear motion in Cartesian space"
    "Parameter", "- ``joint_pos``:Target joint position, unit[°]；
    - ``desc_pos``:Target Cartesian pose，unit[mm][°]；
    - ``tool``:TOOL No，[0~14]；
    - ``user``:Workpiece number，[0~14]；
    - ``vel``:Speed percentage，[0~100]；
    - ``acc``:Acceleration percentage，[0~100]，temporarily closed；
    - ``ovl``:Speed scaling factor，[0~100]；
    - ``blendR``:[-1.0]-motion in place (blocked), [0-1000]-smooth radius(non blocked)，unit[mm]；
    - ``exaxis_pos``:Position of external axis 1~position of external axis 4；
    - ``search``:[0]-non welding wire positioning, [1]-welding wire positioning；
    - ``offset_flag``:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system；
    - ``offset_pos``:Pose offset，unit[mm][°]"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
--------------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 16-18

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[10.000,10.000,10.000,0.000,0.000,0.000]
    J2=[123.709,-121.190,-82.838,-63.499,90.471,-47.174]
    P2=[-273.856,643.260,259.235,-177.972,-1.494,80.866]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[0.000,0.000,0.000,0.000,0.000,0.000]
    J3=[167.066,-95.700,-123.494,-42.493,90.466,-47.174]
    P3=[-423.044,229.703,241.080,-173.990,-5.772,123.971]
    eP3=[0.000,0.000,0.000,0.000]
    dP3=[0.000,0.000,0.000,0.000,0.000,0.000]
    robot.MoveL(J1,P1,0,0,100.0,180.0,100.0,-1.0,eP1,0,1 ,dP1)   #Rectilinear motion in Cartesian space
    robot.MoveL(J2,P2,0,0,100.0,180.0,100.0,-1.0,eP2,0,0,dP2)
    robot.MoveL(J3,P3,0,0,100.0,180.0,100.0,-1.0,eP3,0,0,dP3)

Circular arc motion in Cartesian space
++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveC(joint_pos_p,desc_pos_p,ptool,puser,pvel,pacc,exaxis_pos_p,poffset_flag,offset_pos_p,joint_pos_t,desc_pos_t,ttool,tuser,tvel,tacc,exaxis_pos_t ,toffset_flag,offset_pos_t,ovl,blendR)``"
    "Description", "Circular arc motion in Cartesian space"
    "Parameter", "- ``joint_pos_p``:Path point joint position，unit[°]；
    - ``desc_pos_p``:Path point Cartesian pose，unit[mm][°]；
    - ``ptool``:TOOL No，[0~14]；
    - ``puser``:Workpiece number，[0~14]；
    - ``pvel``:Speed percentage，[0~100]；
    - ``pacc``:Acceleration percentage，[0~100]，temporarily closed；
    - ``exaxis_pos_p``:Position of external axis 1~position of external axis 4；
    - ``poffset_flag``:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system；
    - ``offset_pos_p``:Offset，unit[mm][°]；
    - ``joint_pos_t``:Target point joint position，unit[°]；
    - ``desc_pos_t``:Cartesian pose of the target point，unit[mm][°]；
    - ``ttool``:TOOL No，[0~14]；
    - ``tuser``:Workpiece number，[0~14]；
    - ``tvel``:Speed percentage，[0~100]；
    - ``tacc``:Acceleration percentage，[0~100]，temporarily closed；
    - ``exaxis_pos_t``:Position of external axis 1~position of external axis 4；
    - ``toffset_flag``:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system；
    - ``offset_pos_t``:Offset，unit[mm][°]。
    - ``ovl``:Speed scaling factor，[0~100]；
    - ``blendR``:[-1.0]-motion in place (blocked), [0-1000]-smooth radius(non blocked)，unit[mm]"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
-------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 19,20

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    J1=[121.381,-97.108,-123.768,-45.824,89.877,-47.296]
    P1=[-127.772,459.534,221.274,-177.850,-2.507,78.627]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[10.000,10.000,10.000,10.000,10.000,10.000]
    J2=[138.884,-114.522,-103.933,-49.694,90.688,-47.291]
    P2=[-360.468,485.600,196.363,-178.239,-0.893,96.172]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[10.000,10.000,10.000,10.000,10.000,10.000]
    pa2=[0.0,0.0,100.0,180.0]
    J3=[159.164,-96.105,-128.653,-41.170,90.704,-47.290]
    P3=[-360.303,274.911,203.968,-176.720,-2.514,116.407]
    eP3=[0.000,0.000,0.000,0.000]
    dP3=[10.000,10.000,10.000,10.000,10.000,10.000]
    pa3=[0.0,0.0,100.0,180.0]
    dP=[10.000,10.000,10.000,10.000,10.000,10.000]
    robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)       #Joint space motionPTP
    robot.MoveC(J2,P2,pa2,eP2,0,dP2,J3,P3,pa3,eP3,0,dP3,100.0,-1.0)    #Circular motion in Cartesian space

Circular motion in Cartesian space
++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``Circle(joint_pos_p,desc_pos_p,ptool,puser,pvel,pacc,exaxis_pos_p,joint_pos_t,desc_pos_t,ttool,tuser,tvel,tacc,exaxis_pos_t,ovl,offset_flag,offset_pos)``"
    "Description", "Circular motion in Cartesian space"
    "Parameter", "- ``joint_pos_p``:Path point joint position，unit[°]；
    - ``desc_pos_p``:Path point Cartesian pose，unit[mm][°]；
    - ``ptool``:TOOL No，[0~14]；
    - ``puser``:Workpiece number，[0~14]；
    - ``pvel``:Speed percentage，[0~100]；
    - ``pacc``:Acceleration percentage，[0~100]，temporarily closed；
    - ``exaxis_pos_p``:Position of external axis 1~position of external axis 4；
    - ``joint_pos_t``:Target point joint position，unit[°]；
    - ``desc_pos_t``:Cartesian pose of the target point，unit[mm][°]；
    - ``ttool``:TOOL No，[0~14]；
    - ``tuser``:Workpiece number，[0~14]；
    - ``tvel``:Speed percentage，[0~100]；
    - ``tacc``:Acceleration percentage，[0~100]，temporarily closed；
    - ``exaxis_pos_t``:Position of external axis 1~position of external axis 4；
    - ``ovl``:Speed scaling factor，[0~100%]；
    - ``offset_flag``:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system；
    - ``offset_pos``:Offset，unit[mm][°]"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
-------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 19,20

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    J1=[121.381,-97.108,-123.768,-45.824,89.877,-47.296]
    P1=[-127.772,459.534,221.274,-177.850,-2.507,78.627]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[10.000,10.000,10.000,10.000,10.000,10.000]
    J2=[138.884,-114.522,-103.933,-49.694,90.688,-47.291]
    P2=[-360.468,485.600,196.363,-178.239,-0.893,96.172]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[10.000,10.000,10.000,10.000,10.000,10.000]
    pa2=[0.0,0.0,100.0,180.0]
    J3=[159.164,-96.105,-128.653,-41.170,90.704,-47.290]
    P3=[-360.303,274.911,203.968,-176.720,-2.514,116.407]
    eP3=[0.000,0.000,0.000,0.000]
    dP3=[10.000,10.000,10.000,10.000,10.000,10.000]
    pa3=[0.0,0.0,100.0,180.0]
    dP=[10.000,10.000,10.000,10.000,10.000,10.000]
    robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)    #Joint space motionPTP
    robot.Circle(J2,P2,pa2,eP2,J3,P3,pa3,eP3,100.0,0,dP)    #Circular motion in Cartesian space

Spiral motion in Cartesian space
++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``NewSpiral(joint_pos,desc_pos,tool,user,vel,acc,exaxis_pos,ovl,offset_flag,offset_pos,param)``"
    "Description", "Spiral motion in Cartesian space"
    "Parameter", "- ``joint_pos``:Target joint position, unit[°]；
    - ``desc_pos``:Target Cartesian pose，unit[mm][°]；
    - ``tool``:TOOL No，[0~14]；
    - ``user``:Workpiece number，[0~14]；
    - ``vel``:Speed percentage，[0~100]；
    - ``acc``:Acceleration percentage，[0~100]，temporarily closed；
    - ``exaxis_pos``:Position of external axis 1~position of external axis 4；
    - ``ovl``:Speed scaling factor，[0~100]；
    - ``offset_flag``:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system；
    - ``offset_pos``:Pose offset，unit[mm][°]
    - ``param``:[circle_num,circle_angle,rad_init,rad_add,rotaxis_add,rot_direction]，circle_num: number of coils, circle_angle: helix angle, rad_init: initial radius of the helix, rad_add: radius increment, rotaxis_add: axis direction increment, rot_direction: rotation direction, 0-clockwise, 1-counterclockwise"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
---------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 18

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    J1=[127.888,-101.535,-94.860,17.836,96.931,-61.325]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[50.0,0.0,0.0,-30.0,0.0,0.0]
    J2=[127.888,-101.535,-94.860,17.836,96.931,-61.325]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[50.0,0.0,0.0,-5.0,0.0,0.0]
    Pa = [5.0,5.0,50.0,10.0,10.0,0.0]
    P1 = robot.GetForwardKin(J1)       #The forward kinematic interface can be used to solve Cartesian space coordinates with only joint positions
    print(P1)
    P1 = [P1[1],P1[2],P1[3],P1[4],P1[5],P1[6]]
    robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,0.0,2,dP1)
    P2 = robot.GetForwardKin(J2)       #The forward kinematic interface can be used to solve Cartesian space coordinates with only joint positions
    print(P2)
    P2 = [P2[1],P2[2],P2[3],P2[4],P2[5],P2[6]]
    robot.NewSpiral(J2,P2,0,0,100.0,180.0,eP2,100.0,2,dP2,Pa)   #Helical motion

Joint space servo mode motion
++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoJ(joint_pos,acc,vel,cmdT,filterT,gain)``"
    "Description", "Joint space servo mode motion"
    "Parameter", "- ``joint_pos``:Target joint position, unit[°]；
    - ``acc``:Acceleration, range[0~100]，temporarily closed，default to0；
    - ``vel``: Speed, range[0~100]，temporarily closed，default to0；
    - ``cmdT``:Instruction Cycle，unit[s]，[0.001~0.016]；
    - ``filterT``:Filtering time，unit[s]，temporarily closed；
    - ``gain``:Proportional amplifier for target position，temporarily closed"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
--------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 15

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    joint_pos = robot.GetActualJointPosDegree(0)
    print(joint_pos)
    joint_pos = [joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5],joint_pos[6]]
    acc = 0.0
    vel = 0.0
    t = 0.008
    lookahead_time = 0.0
    P = 0.0
    count = 100
    while(count):
        robot.ServoJ(joint_pos, acc, vel, t, lookahead_time, P)
        joint_pos[0] = joint_pos[0] + 0.1
        count = count - 1
        time.sleep(0.008)

Cartesian space servo mode motion
++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ServoCart(mode,desc_pos,pos_gain,acc,vel,cmdT,filterT,gain)``"
    "Description", "Cartesian space servo mode motion"
    "Parameter", "- ``mode``:[0]-absolute motion (base coordinate system), [1]-incremental motion (base coordinate system), [2]-incremental motion (tool coordinate system)；
    - ``desc_pos``:Target Cartesian Position/Target Cartesian Position Increment；
    - ``pos_gain``:Pose increment ratio coefficient, only effective in incremental motion, range[0~1]；
    - ``acc``:Acceleration, range[0~100]，temporarily closed，default to0；
    - ``vel``: Speed, range[0~100]，temporarily closed，default to0；
    - ``cmdT``:Instruction Cycle，unit[s]，[0.001~0.016]；
    - ``filterT``:Filtering time，unit[s]，temporarily closed；
    - ``gain``:Proportional amplifier for target position，temporarily closed"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
--------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 15

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    mode = 2  #Tool coordinate system incremental motion
    n_pos = [0.0,0.0,0.5,0.0,0.0,0.0]   #Incremental pose in Cartesian space
    gain = [0.0,0.0,1.0,0.0,0.0,0.0]
    acc = 0.0
    vel = 0.0
    t = 0.008
    lookahead_time = 0.0
    P = 0.0
    count = 100
    while(count):
        robot.ServoCart(mode, n_pos, gain, acc, vel, t, lookahead_time, P)
        count = count - 1
        time.sleep(0.008)

Point-to-point motion in Cartesian space
++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveCart(desc_pos,tool,user,vel,acc,ovl,blendT,config)``"
    "Description", "Point-to-point motion in Cartesian space"
    "Parameter", "- ``desc_pos``:Target Cartesian position；
    - ``tool``:TOOL No，[0~14]；
    - ``user``:Workpiece number，[0~14]；
    - ``vel``: Speed, range[0~100]，temporarily closed，default to0；
    - ``acc``:Acceleration, range[0~100]，temporarily closed，default to0；
    - ``ovl``:Speed scaling factor，[0~100]；
    - ``blendT``:[-1.0]-Motion in place (blocked), [0-500]-Smoothing time (non blocked)，unit[ms]；
    - ``config``:Joint configuration, [-1]-refer to the current joint position for solution, [0-7]-solve based on joint configuration"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
-------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 8-10

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    P2=[-273.856,643.260,259.235,-177.972,-1.494,80.866]
    P3=[-423.044,229.703,241.080,-173.990,-5.772,123.971]
    robot.MoveCart(P1,0,0,100.0,100.0,100.0,-1.0,-1)       #Point-to-point motion in Cartesian space
    robot.MoveCart(P2,0,0,100.0,100.0,100.0,-1.0,-1)
    robot.MoveCart(P3,0,0,100.0,100.0,100.0,0.0,-1)
    time.sleep(1)
    robot.StopMotion()    #Stop moving

Robot spline motion
+++++++++++++++++++++

Spline motion start
------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SplineStart()``"
    "Description", "Spline motion start"
    "Parameter", "Nothing"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Spline motion PTP
-------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SplinePTP(joint_pos,desc_pos,tool,user,vel,acc,ovl)``"
    "Description", "Spline motion PTP"
    "Parameter", "- ``joint_pos``:Target joint position, unit[°]；
    - ``desc_pos``:Target Cartesian pose，unit[mm][°]；
    - ``tool``:TOOL No，[0~14]；
    - ``user``:Workpiece number，[0~14]；
    - ``vel``: Speed, range[0~100]，temporarily closed，default to0；
    - ``acc``:Acceleration, range[0~100]，temporarily closed，default to0；
    - ``ovl``:Speed scaling factor，[0~100]；"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

End of spline motion
-----------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SplineEnd()``"
    "Description", "End of spline motion"
    "Parameter", "Nothing"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:
    :emphasize-lines: 15-20

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    J1 = [114.578,-117.798,-97.745,-54.436,90.053,-45.216]
    P1 = [-140.418,619.351,198.369,-179.948,0.023,69.793]
    eP1 = [0.000,0.000,0.000,0.000]
    dP1 = [0.000,0.000,0.000,0.000,0.000,0.000]
    J2 = [115.401,-105.206,-117.959,-49.727,90.054,-45.222]
    P2 = [-95.586,504.143,186.880,178.001,2.091,70.585]
    J3 = [135.609,-103.249,-120.211,-49.715,90.058,-45.219]
    P3 = [-252.429,428.903,188.492,177.804,2.294,90.782]
    J4 = [154.766,-87.036,-135.672,-49.045,90.739,-45.223]
    P4 = [-277.255,272.958,205.452,179.289,1.765,109.966]
    robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
    robot.SplineStart()    #Spline motion start
    robot.SplinePTP(J1,P1,0,0,100.0,180.0,100.0)    #Spline motion PTP
    robot.SplinePTP(J2,P2,0,0,100.0,180.0,100.0)
    robot.SplinePTP(J3,P3,0,0,100.0,180.0,100.0)
    robot.SplinePTP(J4,P4,0,0,100.0,180.0,100.0)
    robot.SplineEnd()     #End of spline motion

Robot New Spline Motion
++++++++++++++++++++++++++++++++
New spline motion begins
-----------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``NewSplineStart(type)``"
    "Description", "New spline motion begins"
    "Parameter", "- ``type``:0-arc transition, 1-given point position path point"
    "Return value", "- Success：[0]
    - Failed：[errcode]"


End of new spline motion
----------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``NewSplineEnd()``"
    "Description", "End of new spline motion"
    "Parameter", "Nothing"
    "Return value", "- Success：[0]
    - Failed：[errcode]"


New Spline Instruction Points
------------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``NewSplinePoint(joint_pos,desc_pos,tool,user,vel,acc,ovl,blendR,lastFlag)``"
    "Description", "New Spline Instruction Points"
    "Parameter", "- ``joint_pos``:Target joint position, unit[°]；
    - ``desc_pos``:Target Cartesian pose，unit[mm][°]；
    - ``tool``:TOOL No，[0~14]；
    - ``user``:Workpiece number，[0~14]；
    - ``vel``: Speed, range[0~100]，temporarily closed，default to0；
    - ``acc``:Acceleration, range[0~100]，temporarily closed，default to0；
    - ``ovl``:Speed scaling factor，[0~100]；
    - ``blendR``: [0-1000]-smooth radius，unit[mm]；
    - ``lastFlag``:Is it the last point, 0-No, 1-Yes"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
^^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:
    :emphasize-lines: 15-20

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    J1 = [114.578,-117.798,-97.745,-54.436,90.053,-45.216]
    P1 = [-140.418,619.351,198.369,-179.948,0.023,69.793]
    eP1 = [0.000,0.000,0.000,0.000]
    dP1 = [0.000,0.000,0.000,0.000,0.000,0.000]
    J2 = [115.401,-105.206,-117.959,-49.727,90.054,-45.222]
    P2 = [-95.586,504.143,186.880,178.001,2.091,70.585]
    J3 = [135.609,-103.249,-120.211,-49.715,90.058,-45.219]
    P3 = [-252.429,428.903,188.492,177.804,2.294,90.782]
    J4 = [154.766,-87.036,-135.672,-49.045,90.739,-45.223]
    P4 = [-277.255,272.958,205.452,179.289,1.765,109.966]
    robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
    robot.NewSplineStart(1)    #The spline motion begins
    robot.NewSplinePoint(J1,P1,0,0,50.0,50.0,50.0,0.0,0)    #Spline control point
    robot.NewSplinePoint(J2,P2,0,0,50.0,50.0,50.0,0.0,0)
    robot.NewSplinePoint(J3,P3,0,0,50.0,50.0,50.0,0.0,0)
    robot.NewSplinePoint(J4,P4,0,0,50.0,50.0,50.0,0.0,1)
    robot.NewSplineEnd() 

Robot terminates motion
+++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``StopMotion()``"
    "Description", "Terminate the motion, and the motion command must be in a non blocking state when terminating the motion"
    "Parameter", "Nothing"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
-------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 12

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    P2=[-273.856,643.260,259.235,-177.972,-1.494,80.866]
    P3=[-423.044,229.703,241.080,-173.990,-5.772,123.971]
    robot.MoveCart(P1,0,0,100.0,100.0,100.0,-1.0,-1)       #Point to point motion in joint space
    robot.MoveCart(P2,0,0,100.0,100.0,100.0,-1.0,-1)
    robot.MoveCart(P3,0,0,100.0,100.0,100.0,0.0,-1)   #This motion instruction is in a non-blocking state
    time.sleep(1)
    robot.StopMotion()    #Stop motion

Overall displacement of robot points
+++++++++++++++++++++++++++++++++++++++++
Starting point overall offset
----------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``PointsOffsetEnable(flag,offset_pos)``"
    "Description", "Starting point overall offset"
    "Parameter", "- ``flag``:0-offset under base coordinate or workpiece coordinate system, 2-offset under tool coordinate system；
    - ``offset_pos``:Offset，unit[mm][°]。"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

The overall offset of the point ends
----------------------------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``PointsOffsetDisable()``"
    "Description", "The overall offset of the point ends"
    "Parameter", "Nothing"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
^^^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:
    :emphasize-lines: 19,22

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    #Overall shift of robot point position
    J1=[-168.847,-93.977,-93.118,-80.262,88.985,11.831]
    P1=[-558.082,27.343,208.135,-177.205,-0.450,89.288]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[10.000,10.000,10.000,0.000,0.000,0.000]
    J2=[168.968,-93.977,-93.118,-80.262,88.986,11.831]
    P2=[-506.436,236.053,208.133,-177.206,-0.450,67.102]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[0.000,0.000,0.000,0.000,0.000,0.000]
    robot.MoveJ(J1,P1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
    robot.MoveJ(J2,P2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
    time.sleep(2)
    flag = 0
    offset = [100.0,5.0,6.0,0.0,0.0,0.0]   #Pose offset
    robot.PointsOffsetEnable(flag, offset)   #Global offset start
    robot.MoveJ(J1,P1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
    robot.MoveJ(J2,P2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
    robot.PointsOffsetDisable()  #End of global shift
