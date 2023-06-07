Status query
=====================

.. toctree:: 
    :maxdepth: 5

Obtain robot installation angle
+++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotInstallAngle()``"
    "Description", "Obtain robot installation angle"
    "Parameter", "Nothing"
    "Return value", "- Success:[0,yangle,zangle],yangle-angle of roll,zangle-rotation angle
    - Failed:[errcode,]"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetRobotInstallAngle()  # Obtain robot installation angle
    print(ret)

Obtain system variable values
++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSysVarValue(id)``"
    "Description", "Obtain system variable values"
    "Parameter", "- ``id``:System variable number, range[1~20]"
    "Return value", "- Success:[0,var_value]
    - Failed:[errcode,]"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 8

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    for i in range(1,21):
        robot.SetSysVarValue(i,i+0.5)    #  Setting System Variable Values
    robot.WaitMs(1000)
    for i in range(1,21):
        sys_var = robot.GetSysVarValue(i)  #  Query system variable values
        print(sys_var)

Obtain the current joint position (angle)
++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualJointPosDegree(flag)``"
    "Description", "Obtain the current joint position (angle))"
    "Parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0,joint_pos],joint_pos=[j1,j2,j3,j4,j5,j6]
    - Failed:[errcode,]"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetActualJointPosDegree(0)  # Obtain the current joint position of the robot
    print(ret)

Obtain the current joint position(radian)
+++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualJointPosRadian(flag)``"
    "Description", "Obtain the current joint position(radian)"
    "Parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0,joint_pos],joint_pos=[j1,j2,j3,j4,j5,j6]
    - Failed:[errcode,]"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetActualJointPosRadian(0)  # Obtain the current joint position of the robot
    print(ret)

Obtain the current tool pose
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualTCPPose(flag)``"
    "Description", "Obtain the current tool pose"
    "Parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0,tcp_pose],tcp_pose=[x,y,z,rx,ry,rz]
    - Failed:[errcode,]"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetActualTCPPose(0)  # Obtain the current tool pose of the robot
    print(ret)

Obtain the current tool coordinate system number
++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualTCPNum(flag)``"
    "Description", "Obtain the current tool coordinate system number"
    "Parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0,tool_id]
    - Failed:[errcode,]"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetActualTCPNum(0)  # Obtain the current tool coordinate system number
    print(ret)

Obtain the current workpiece coordinate system number
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualWObjNum(flag)``"
    "Description", "Obtain the current workpiece coordinate system number"
    "Parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0,wobj_id]
    - Failed:[errcode,]"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetActualWObjNum(0)  # Obtain the current workpiece coordinate system number
    print(ret)

Obtain the current end flange pose
+++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualToolFlangePose(flag)``"
    "Description", "Obtain the current end flange pose"
    "Parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0,flange_pose],flange_pose=[x,y,z,rx,ry,rz]
    - Failed:[errcode,]"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetActualToolFlangePose(0)  # Obtain the current end flange pose
    print(ret)

Inverse kinematics solution
+++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetInverseKin(type,desc_pos,config)``"
    "Description", "Inverse kinematics, Cartesian pose to solve joint position"
    "Parameter", "- ``type``:0-absolute pose (base coordinate system), 1-relative pose (base coordinate system), 2-relative pose (tool coordinate system)
    - ``desc_pose``:[x,y,z,rx,ry,rz],tool posture,unit[mm][°]
    - ``config``:Joint configuration, [-1]-refer to the current joint position for solution, [0-7]-solve based on joint configuration"
    "Return value", "- Success:[0,joint_pos],joint_pos=[j1,j2,j3,j4,j5,j6]
    - Failed:[errcode,]"

Code example
--------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    ret = robot.GetInverseKin(0,P1,-1)
    print(ret)

Inverse kinematics solution - Specify reference location
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetInverseKinRef(type,desc_pos,joint_pos_ref)``"
    "Description", "Inverse kinematics solve inverse kinematics, tool pose solve joint position, and refer to specified joint position to solve"
    "Parameter", "- ``type``:0-absolute pose (base coordinate system), 1-relative pose (base coordinate system), 2-relative pose (tool coordinate system)
    - ``desc_pos``:[x,y,z,rx,ry,rz]tool posture,unit[mm][°]
    - ``joint_pos_ref``:[j1,j2,j3,j4,j5,j6], joint reference position,unit[°]"
    "Return value", "- Success:[0,joint_pos],joint_pos=[j1,j2,j3,j4,j5,j6]
    - Failed:[errcode,]"

Code example
--------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    ret = robot.GetInverseKinRef(0,P1,J1)
    print(ret)

Inverse kinematics solution - whether there is a solution
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetInverseKinHasSolution(type,desc_pos,joint_pos_ref)``"
    "Description", "Inverse kinematics, tool pose solution, whether joint position is solved"
    "Parameter", "- ``type``:0-Absolute pose (base coordinate system), 1-Relative pose (base coordinate system), 2-Relative pose (tool coordinate system)
    - ``desc_pos``:[x,y,z,rx,ry,rz]tool posture, unit[mm][°]
    - ``joint_pos_ref``:[j1,j2,j3,j4,j5,j6],joint reference position, unit[°]"
    "Return value", "- Success:[0,result],“True”-with solution,“False”-without solution
    - Failed:[errcode,]"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    ret = robot.GetInverseKinHasSolution(0,P1,J1)
    print(ret)

Forward kinematics solution
+++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetForwardKin(joint_pos)``"
    "Description", "Forward kinematics, joint position solving tool pose"
    "Parameter", "- ``joint_pos``:[j1,j2,j3,j4,j5,j6]:joint Position,unit[°]"
    "Return value", "- Success:[0,desc_pos],desc_pos=[x,y,z,rx,ry,rz]:tool posture,unit[mm][°]
    - Failed:[errcode,]"

Code example
-------------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    ret = robot.GetForwardKin(J1)
    print(ret)

Obtain the current joint torque
+++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetJointTorques(flag)``"
    "Description", "Obtain the current joint torque"
    "Parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0,torques],torques=[j1,j2,j3,j4,j5,j6]
    - Failed:[errcode,]"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetJointTorques(0)  # Obtain the current joint torque
    print(ret)

Obtain the weight of the current load
+++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetPayload(flag)``"
    "Description", "Obtain the weight of the current load"
    "Parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0,weight],unit[kg]
    - Failed:[errcode,]"

Code example
-----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetTargetPayload(0)  # Obtain the weight of the current load
    print(ret)

Obtain the centroid of the current load
++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetPayloadCog(flag)``"
    "Description", "Obtain the centroid of the current load"
    "Parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0,cog], cog=[x,y,z]:barycentric coordinate,unit[mm]
    - Failed:[errcode,]"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetTargetPayloadCog(0)  # Obtain the centroid of the current load
    print(ret)

Obtain the current tool coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTCPOffset(flag)``"
    "Description", "Obtain the current tool coordinate system"
    "Parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0,tcp_offset], tcp_offset=[x,y,z,rx,ry,rz]:相对位姿,unit[mm][°]
    - Failed:[errcode,]"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetTCPOffset(0)  # Obtain the current tool coordinate system
    print(ret)

Obtain the current workpiece coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetWObjOffset(flag)``"
    "Description", "Obtain the current workpiece coordinate system"
    "Parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0,wobj_offset], wobj _offset=[x,y,z,rx,ry,rz]:relative pose,unit[mm][°]
    - Failed:[errcode,]"

Code example
--------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetWObjOffset(0)  # Obtain the current workpiece coordinate system
    print(ret)

Obtain joint soft limit angle
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetJointSoftLimitDeg(flag)``"
    "Description", "Obtain joint soft limit angle"
    "Parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0, j1min,j1max,j2min,j2max,j3min,j3max,j4min,j4max,j5min,j5max,j6min,j6max] :axis 1 to axis 6 joint negative limit and positive limit,unit[mm]
    - Failed:[errcode,]"

Code example
--------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetJointSoftLimitDeg(0)  # btain joint soft limit angle
    print(ret)

Get system time
++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSystemClock()``"
    "Description", "Get system time"
    "Parameter", "Nothing"
    "Return value", "- Success:[0,t_ms]:unit[ms]
    - Failed:[errcode,]"

Code example
--------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetSystemClock()  # Obtain the system time of the controller
    print(ret)

Obtain the current joint configuration of the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotCurJointsConfig()``"
    "Description", "Obtain the current joint configuration of the robot"
    "Parameter", "Nothing"
    "Return value", "- Success:[0,config]:range[0~7]
    - Failed:[errcode,]"

Code example
--------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetRobotCurJointsConfig()  # Obtain the current joint configuration of the robot
    print(ret)

Get default speed
++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetDefaultTransVel()``"
    "Description", "Get default speed"
    "Parameter", "Nothing"
    "Return value", "- Success:[0,vel]:unit[mm/s]
    - Failed:[errcode,]"

Code example
--------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetDefaultTransVel()  # Gets the robot's current speed
    print(ret)

Check if the robot motion is complete
+++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotMotionDone()``"
    "Description", "Check if the robot motion is complete"
    "Parameter", "Nothing"
    "Return value", "- Success:[0,state],state:0-incomplete,1-complete
    - Failed:[errcode,]"

Code example
--------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetRobotMotionDone()    #Query the motion completion status of the robot
    if ret[0] == 0:
        print(ret[1])
    else:
        print("the errcode is: ", ret[0])
