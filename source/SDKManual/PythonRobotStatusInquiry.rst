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
    "Required parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode
    - Return(if success): [yangle,zangle],yangle-angle of roll,zangle-rotation angle"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetRobotInstallAngle()
    print("Obtain robot installation angle", ret)

Obtain system variable values
++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSysVarValue(id)``"
    "Description", "Obtain system variable values"
    "Required parameter", "- ``id``:System variable number, range[1~20]"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode
    - Return(if success): var_value"

Code example
------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    for i in range(1,21):
        error = robot.GetSysVarValue(i)
        print("Obtain system variable number:",i,"value:", error)

Obtain the current joint position (angle)
++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualJointPosDegree(flag = 1)``"
    "Description", "Obtain the current joint position(radian)"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): joint_pos=[j1,j2,j3,j4,j5,j6]"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualJointPosDegree()
    print("Obtain the current joint position (angle)", ret)

Obtain the current joint position(radian)
+++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualJointPosRadian(flag = 1)``"
    "Description", "Obtain the current joint position(radian)"
    "Required parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Success:[0,joint_pos],joint_pos=[j1,j2,j3,j4,j5,j6]
    - Failed:[errcode,]"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualJointPosRadian()
    print("Obtain the current joint position(radian)", ret)

Obtain joint Actual Speed -deg/s
++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualJointSpeedsDegree (flag = 1 )``"
    "Description", "Obtain joint Actual Speed -deg/s"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): joint_pos=[j1,j2,j3,j4,j5,j6]"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualJointSpeedsDegree()
    print("Obtain joint Actual Speed -deg/s ", ret)

Obtain Target TCP Composite Speed
++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetTCPCompositeSpeed (flag = 1)``"
    "Description", "Obtain Target TCP Composite Speed"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): [tcp_speed,ori_speed]"

Code example
--------------
.. code-block:: python
    :linenos:
    
    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetTargetTCPCompositeSpeed()
    print("Obtain Target TCP Composite Speed", ret)


Obtain Actual TCP Composite Speed
++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualTCPCompositeSpeed (flag = 1)``"
    "Description", "Obtain Actual TCP Composite Speed"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): [tcp_speed,ori_speed]"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualTCPCompositeSpeed()
    print("Obtain Actual TCP Composite Speed ", ret)

Obtain Target TCP Speed
++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetTCPSpeed (flag = 1)``"
    "Description", "Obtain Actual TCP Composite Speed"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): speed [x,y,z,rx,ry,rz]"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetTargetTCPSpeed()
    print("Obtain Target TCP Speed ", ret)

Obtain Actual TCP Speed
++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualTCPSpeed (flag = 1)``"
    "Description", "Obtain Actual TCP Speed"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): speed [x,y,z,rx,ry,rz]"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualTCPSpeed()
    print("Obtain Actual TCP Speed ", ret)

Obtain the current tool pose
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualTCPPose (flag = 1)``"
    "Description", "Obtain the current tool pose"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): tcp_pose=[x,y,z,rx,ry,rz]"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualTCPPose()
    print("Obtain the current tool pose", ret)

Obtain the current tool coordinate system number
++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualTCPNum (flag = 1)``"
    "Description", "Obtain the current tool coordinate system number"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): tool_id:Tool number"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualTCPNum()
    print("Obtain the current tool coordinate system number", ret)

Obtain the current workpiece coordinate system number
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualWObjNum (flag = 1)``"
    "Description", "Obtain the current workpiece coordinate system number"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): wobj_id:Workpiece number"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualWObjNum()
    print("Obtain the current workpiece coordinate system number", ret)

Obtain the current end flange pose
+++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualToolFlangePose(flag)``"
    "Description", "Obtain the current end flange pose"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): flange_pose=[x,y,z,rx,ry,rz]"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualToolFlangePose()
    print("Obtain the current end flange pose ", ret)

Inverse kinematics solution
+++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetInverseKin(type,desc_pos,config = -1)``"
    "Description", "Inverse kinematics, Cartesian pose to solve joint position"
    "Required parameter", "- ``type``:0-absolute pose (base coordinate system), 1-relative pose (base coordinate system), 2-relative pose (tool coordinate system)
    - ``desc_pose``:[x,y,z,rx,ry,rz],tool posture,unit[mm][°]"
    "Optional parameter", "- ``config``:Joint configuration, [-1]-refer to the current joint position for solution, [0-7]-solve based on joint configuration, default to -1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): joint_pos=[j1,j2,j3,j4,j5,j6]"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    ret = robot.GetInverseKin(0,P1,config=-1)
    print("Inverse kinematics, Cartesian pose to solve joint position ", ret)

Inverse kinematics solution - Specify reference location
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetInverseKinRef(type,desc_pos,joint_pos_ref)``"
    "Description", "Inverse kinematics solve inverse kinematics, tool pose solve joint position, and refer to specified joint position to solve"
    "Required parameter", "- ``type``:0-absolute pose (base coordinate system), 1-relative pose (base coordinate system), 2-relative pose (tool coordinate system)
    - ``desc_pos``:[x,y,z,rx,ry,rz]tool posture,unit[mm][°]
    - ``joint_pos_ref``:[j1,j2,j3,j4,j5,j6], joint reference position,unit[°]"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): joint_pos=[j1,j2,j3,j4,j5,j6]"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    ret = robot.GetInverseKinRef(0,P1,J1)
    print("Inverse kinematics solve inverse kinematics, tool pose solve joint position, and refer to specified joint position to solve", ret)

Inverse kinematics solution - whether there is a solution
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetInverseKinHasSolution(type,desc_pos,joint_pos_ref)``"
    "Description", "Inverse kinematics, tool pose solution, whether joint position is solved"
    "Required parameter", "- ``type``:0-Absolute pose (base coordinate system), 1-Relative pose (base coordinate system), 2-Relative pose (tool coordinate system)
    - ``desc_pos``:[x,y,z,rx,ry,rz]tool posture, unit[mm][°]
    - ``joint_pos_ref``:[j1,j2,j3,j4,j5,j6],joint reference position, unit[°]"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): result= “True”-with solution,“False”-without solution"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    ret = robot.GetInverseKinHasSolution(0,P1,J1)
    print("Inverse kinematics, tool pose solution, whether joint position is solved", ret)

Forward kinematics solution
+++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetForwardKin(joint_pos)``"
    "Description", "Forward kinematics, joint position solving tool pose"
    "Required parameter", "- ``joint_pos``:[j1,j2,j3,j4,j5,j6]:joint Position,unit[°]"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): desc_pos=[x,y,z,rx,ry,rz]:tool posture,unit[mm][°]"

Code example
-------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    ret = robot.GetForwardKin(J1)
    print("Forward kinematics, joint position solving tool pose", ret)

Obtain the current joint torque
+++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetJointTorques (flag = 1)``"
    "Description", "Obtain the current joint torque"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success):  errcode =0, joint_torque =[x,y,z,rx,ry,rz]"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot 
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetJointTorques()
    print("Obtain the current joint torque ", ret)

Obtain the weight of the current load
+++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetPayload (flag = 1)``"
    "Description", "Obtain the weight of the current load"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): errcode =0, weight unit[kg]"

Code example
-----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetTargetPayload(0)
    print("Obtain the weight of the current load ", ret)

Obtain the centroid of the current load
++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetPayloadCog (flag = 1)``"
    "Description", "Obtain the centroid of the current load"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): weight,unit[kg]"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetTargetPayloadCog(0)
    print("Obtain the centroid of the current load", ret)

Obtain the current tool coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTCPOffset (flag = 1)``"
    "Description", "Obtain the current tool coordinate system"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): tcp_offset=[x,y,z,rx,ry,rz]: Relative pose,unit[mm][°]"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetTCPOffset()
    print("Obtain the current tool coordinate system", ret)

Obtain the current workpiece coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetWObjOffset(flag = 1)``"
    "Description", "Obtain the current workpiece coordinate system"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): wobj _offset=[x,y,z,rx,ry,rz]:relative pose,unit[mm][°]"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetWObjOffset()
    print("Obtain the current workpiece coordinate system", ret)

Obtain joint soft limit angle
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetJointSoftLimitDeg(flag)``"
    "Description", "Obtain joint soft limit angle"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``flag``:0-blocking, 1-non blocking, default to 1"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): [j1min,j1max,j2min,j2max,j3min,j3max,j4min,j4max,j5min,j5max, j6min, j6max] :axis 1 to axis 6 joint negative limit and positive limit,unit[mm]"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetJointSoftLimitDeg()
    print("Obtain joint soft limit angle", ret)

Get system time
++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSystemClock()``"
    "Description", "Get system time"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success):  t_ms unit[ms] "

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetSystemClock()
    print("Get system time", ret)

Obtain the current joint configuration of the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotCurJointsConfig()``"
    "Description", "Obtain the current joint configuration of the robot"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): config range[0~7]"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetRobotCurJointsConfig()
    print("Obtain the current joint configuration of the robot ", ret)

Get default speed
++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetDefaultTransVel()``"
    "Description", "Obtain default speed"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): vel:unit[mm/s]"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetDefaultTransVel()
    print("Obtain default speed ", ret)

Check if the robot motion is complete
+++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotMotionDone()``"
    "Description", "Check if the robot motion is complete"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): state 0-incomplete,1-complete"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetRobotMotionDone()
    print("Check if the robot motion is complete ", ret)

Obtain the robot error code
+++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotErrorCode()``"
    "Description", "Obtain the robot error code"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): [maincode,subcode]"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetRobotErrorCode()
    print("Obtain the robot error code ", ret)

Obtain the robot teaching point data
+++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotTeachingPoint(name)``"
    "Description", "Obtain the robot teaching point data"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): [x,y,z,rx,ry,rz,j1,j2,j3,j4,j5,j6,tool, wobj,speed,acc,e1,e2,e3,e4]"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetRobotTeachingPoint("11")
    print("Obtain the robot teaching point data", ret)


Obtain SSH Keygen
+++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSSHKeygen()``"
    "Description", "Obtain SSH Keygen"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): keygen"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetSSHKeygen() # Obtain SSH Keygen
    print("Obtain SSH Keygen", ret)

6.31Calculates the MD5 value of the file in the specified path
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeFileMD5(file_path)``"
    "Description", "Calculates the MD5 value of the file in the specified path"
    "Required parameter", "``file_path``:The traj folder path is /fruser/traj/, such as /fruser/traj/trajHelix_aima_1.txt."
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): MD5"

Code example
--------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.ComputeFileMD5("/fruser/201.lua")   #Calculate the MD5 value of the file in the specified path
    print("Calculates the MD5 value of the file in the specified path ", ret)

Obtain robot version information
+++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSoftwareVersion()``"
    "Description", "Obtain robot version information"
    "Required parameter", "Null"
    "Optional parameter", "Nothing"
    "Return value", "- Success -0 , Failed -errcode
    - Return:（if success）robotModel, webVersion, controllerVersion"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    robot = Robot.RPC('192.168.58.2')

    ret = robot.GetSoftwareVersion()
    print("GetSoftwareVersion():", ret)

Obtain robot hardware version information
+++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSlaveHardVersion()``"
    "Description", "Obtain robot hardware version information"
    "Required parameter", "Null"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return:（if success）ctrlBoxBoardVersion, driver1Version, driver2Version , driver3Version, driver4Version, driver5Version, driver6Version, endBoardVersion"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetSlaveHardVersion()
    print("GetSlaveHardVersion():", ret)

Obtain robot firmware version information
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSlaveFirmVersion()``"
    "Description", "Obtain robot firmware version information"
    "Required parameter", "Null"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return:（if success）ctrlBoxBoardVersion, driver1Version, driver2Version , driver3Version, driver4Version, driver5Version, driver6Version, endBoardVersion"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetSlaveFirmVersion()
    print("GetSlaveFirmVersion():", ret)

Obtain DH compensation parameters
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetDHCompensation()``"
    "Description", "Obtain DH compensation parameters"
    "Required parameter", "Null"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return:(if success) dhCompensation Robot DH parameter compensation value (mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]"

Code example
------------
.. code-block:: python
    :linenos:

    import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.GetDHCompensation()
    print(error)
