Status query
===============

.. toctree::
    :maxdepth: 5

Getting the robot mounting angle
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotInstallAngle()``"
    "Description", "Get robot mounting angle"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``[yangle,zangle]``: yangle - angle of inclination, zangle - angle of rotation."

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetRobotInstallAngle()
    print("Get robot mounting angle", ret)

Getting system variable values
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSysVarValue(id)``"
    "Description", "Get system variable values"
    "Mandatory parameters", "- ``id``: system variable number, range [1~20]"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``var_value``: system variable value"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    for i in range(1,21):
        error = robot.GetSysVarValue(i)
        print("System variable number:", i, "value", error)

Get the current joint position (angle).
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetActualJointPosDegree(flag=1)``"
    "Description", "Get the current position (angle) of the joint."
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``: 0-blocking, 1-non-blocking, default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``joint_pos=[j1,j2,j3,j4,j5,j6]``: current joint position (angle)"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualJointPosDegree()
    print("Get current joint position (angle)", ret)

Get the current joint position in radians.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualJointPosRadian(flag=1)``"
    "Description", "Get the current position (in radians) of the joint."
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``joint_pos=[j1,j2,j3,j4,j5,j6]``: current joint position (in radians)"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualJointPosRadian()
    print("Get current joint position in radians", ret)

Get joint feedback speed -deg/s
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetActualJointSpeedsDegree (flag=1)``"
    "Description", "Get joint feedback speed -deg/s"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``speed=[j1,j2,j3,j4,j5,j6]``: joint feedback speed -deg/s"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualJointSpeedsDegree()
    print("Getting joint feedback speed -deg/s", ret)

Get TCP command synthesis speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetTCPCompositeSpeed(flag=1)``"
    "Description", "Get TCP command synthesis speed"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``[tcp_speed,ori_speed]``: tcp_speed - linear closing speed ori_speed - attitude closing speed"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetTargetTCPCompositeSpeed()
    print("Getting TCP command synthesis speed", ret)

Getting TCP Feedback Hopping Speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualTCPCompositeSpeed(flag=1)``"
    "Description", "Get TCP feedback closing speed"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``[tcp_speed,ori_speed]``: tcp_speed - linear closing speed ori_speed - attitude closing speed"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualTCPCompositeSpeed()
    print("Getting TCP Feedback Hopping Speed", ret)

Get TCP command speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetTCPSpeed(flag=1)``"
    "Description", "Get TCP command speed"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``speed=[x,y,z,rx,ry,rz]``: TCP command speed, mm/s"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetTargetTCPSpeed()
    print("Getting TCP command speed", ret)

Getting TCP feedback speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualTCPSpeed(flag=1)``"
    "Description", "Get TCP feedback on speed"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``speed=[x,y,z,rx,ry,rz]``: TCP feedback speed"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualTCPSpeed()
    print("Getting TCP feedback speed", ret)

Get current tool position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualTCPPose(flag=1)``"
    "Description", "Get current tool position"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``tcp_pose=[x,y,z,rx,ry,rz]``: current tool pose"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualTCPPose()
    print("Get current tool position", ret)

Get the current tool coordinate system number
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetActualTCPNum(flag=1)``"
    "Description", "Get the current tool coordinate system number"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``tool_id``: tool coordinate system number"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualTCPNum()
    print("Get current tool coordinate system number", ret)

Get the current workpiece coordinate system number
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetActualWObjNum(flag=1)``"
    "Description", "Get the current workpiece coordinate system number"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``wobj_id``: the workpiece coordinate system number"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualWObjNum()
    print("Get current workpiece coordinate system number", ret)

Get the current end flange position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualToolFlangePose(flag=1)``"
    "Description", "Get current end flange position"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``flange_pose=[x,y,z,rx,ry,rz]``: current end flange pose"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetActualToolFlangePose()
    print("Get current end flange position", ret)

Inverse kinematics solution
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetInverseKin(type,desc_pos,config=-1)``"
    "Description", "Inverse kinematics, Cartesian position solving for joint positions "
    "Mandatory parameters", "- ``type``: 0-absolute position (base coordinate system), 1-relative position (base coordinate system), 2-relative position (tool coordinate system)
    - ``desc_pose``:[x,y,z,rx,ry,rz], tool position in [mm][°]"
    "Default parameters", "- ``config``: joint configuration, [-1] - solved with reference to current joint position, [0~7] - solved based on joint configuration Default -1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``joint_pos=[j1,j2,j3,j4,j5,j6]``: inverse kinematics solution, Cartesian positional solution for joint positions"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    p1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    ret = robot.GetInverseKin(0,P1,config=-1)
    print("Inverse kinematics, Cartesian position solving for joint position", ret)

Inverse Kinematics Solution - Specifying Reference Positions
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetInverseKinRef(type,desc_pos,joint_pos_ref)``"
    "Description", "Inverse kinematics, tool position solving for joint positions, solving with reference to specified joint positions"
    "Mandatory parameters", "- ``type``: 0-absolute position (base coordinate system), 1-relative position (base coordinate system), 2-relative position (tool coordinate system)
    - ``desc_pos``: [x,y,z,rx,ry,rz] tool position in [mm][°]
    - ``joint_pos_ref``: [j1,j2,j3,j4,j5,j6], joint reference position in [°]"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``joint_pos=[j1,j2,j3,j4,j5,j6]``: inverse kinematics solution, tool position solving for joint position"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    p1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    ret = robot.GetInverseKinRef(0,P1,J1)
    print("Inverse kinematics, tool position solving for joint position, solving with reference to specified joint position", ret)

Inverse kinematics solving-whether there is a solution
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetInverseKinHasSolution(type,desc_pos,joint_pos_ref)``"
    "Description", "Inverse kinematics, tool position solving for joint position Is there a solution"
    "Mandatory parameters", "- ``type``: 0-absolute position (base coordinate system), 1-relative position (base coordinate system), 2-relative position (tool coordinate system)
    - ``desc_pos``: [x,y,z,rx,ry,rz] tool position in [mm][°]
    - ``joint_pos_ref``: [j1,j2,j3,j4,j5,j6], joint reference position in [°]"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``result``: ``True``-with solution, ``False``-without solution"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    p1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    ret = robot.GetInverseKinHasSolution(0,P1,J1)
    print("Inverse kinematics, tool position solving for joint position with or without a solution", ret)

Positive kinematics solving
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetForwardKin(joint_pos)``"
    "Description", "Positive Kinematics, Joint Position Solving Tool Posture"
    "Mandatory parameters", "- ``joint_pos``:[j1,j2,j3,j4,j5,j6]: joint position in [°]"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``desc_pos=[x,y,z,rx,ry,rz]``: positive kinematics solution, joint position solver tool position"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    ret = robot.GetForwardKin(J1)
    print("Positive kinematics, joint position solving tool positon", ret)

Get current joint torque
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetJointTorques(flag=1)``"
    "Description", "Get the current joint torque"
    "Mandatory parameters", "NULL"
    "Default parameters", "``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``torques=[j1,j2,j3,j4,j5,j6]``: joint torques."

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot 
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetJointTorques()
    print("Get current joint torque", ret)

Get the weight of the current load
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetPayload(flag=1)``"
    "Description", "Get the quality of the current load"
    "Mandatory parameters", "NULL"
    "Default parameters", "``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``weight``: current load weight in [kg]"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetTargetPayload(0)
    print("Getting the quality of the current load", ret)

Get the center of mass of the current load
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetPayloadCog(flag=1)``"
    "Description", "Get the center of mass of the current load"
    "Mandatory parameters", "NULL"
    "Default parameters", "``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``cog=[x,y,z]``: coordinates of the current center of mass in [mm]"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetTargetPayloadCog(0)
    print("Get the center of mass of the current load", ret)

Get the current tool coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTCPOffset(flag=1)``"
    "Description", "Get current tool coordinate system"
    "Mandatory parameters", "NULL"
    "Default parameters", "``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``tcp_offset=[x,y,z,rx,ry,rz]``: Relative position of the current tool coordinate system in [mm][°]"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetTCPOffset()
    print("Get current tool coordinate system", ret)

Get the current workpiece coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetWObjOffset(flag=1)``"
    "Description", "Get current workpiece coordinate system"
    "Mandatory parameters", "NULL"
    "Default parameters", "``flag``: 0-blocking, 1-non-blocking, default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``wobj_offset=[x,y,z,rx,ry,rz]``: Relative position of the current workpiece coordinate system in [mm][°]"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetWObjOffset()
    print("Get current workpiece coordinate system", ret)

Obtaining the soft limiting angle of a joint
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetJointSoftLimitDeg(flag=1)``"
    "Description", "Acquisition of joint soft limiting angle"
    "Mandatory parameters", "NULL"
    "Default parameters", "``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``[j1min,j1max,j2min,j2max,j3min,j3max, j4min,j4max,j5min, j5max, j6min,j6max]``: Axis 1 to Axis 6, joints with negative and positive limits, in [mm]"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetJointSoftLimitDeg()
    print("Get joint soft limit angle", ret)

Get system time
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSystemClock()``"
    "Description", "Get system time"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``t_ms``: system time in [ms]"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetSystemClock()
    print("Getting system time", ret)

Get the current joint configuration of the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotCurJointsConfig()``"
    "Description", "Get the current joint configuration of the robot"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``config``: Robot's current joint configuration, range [0~7]"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetRobotCurJointsConfig()
    print("Get the current joint configuration of the robot", ret)

Getting the default speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetDefaultTransVel()``"
    "Description", "Get Default Speed"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``vel``: default speed in [mm/s]"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetDefaultTransVel()
    print("Getting default speed", ret)

Queries whether robot motion is complete
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotMotionDone()``"
    "Description", "Query if robot movement is complete"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``state``: state of robot motion, 0 - unfinished, 1 - finished"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetRobotMotionDone()
    print("Querying if robot movement is complete", ret)

Query Robot Error Code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotErrorCode()``"
    "Description", "Query Robot Error Code"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``[maincode subcode]``: robot error code, maincode - main error code, subcode - suberror code"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetRobotErrorCode()
    print("Query robot error code", ret)

Query Robot Teaching Management Points Data
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotTeachingPoint(name)``"
    "Description", "Query Robot Demonstration and Management point data"
    "Mandatory parameters", "``name``: name of point"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``[x,y,z,rx,ry,rz,j1,j2,j3,j4,j5,j6,TOOL,WOBJ,SPEED,ACC,E1,E2,E3,E4]``: point data"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetRobotTeachingPoint("11")
    print("Query robot demonstration management point data error code", ret)

Get SSH public key
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSSHKeygen()``"
    "description", "Get SSH public key"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``keygen``: public key"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetSSHKeygen() #Get SSH
    print("Getting SSH", ret)

Calculate the MD5 value of a file in a specified path
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeFileMD5(file_path)``"
    "Description", "Calculates the MD5 value of a file in the specified path."
    "Mandatory parameter","- ``file_path``: file path including filename, default Traj folder path is :/fruser/traj/, e.g. /fruser/traj/trajHelix_aima_1.txt"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``md5``: the MD5 value of the file."

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.ComputeFileMD5("/fruser/201.lua") #Calculate the MD5 value of the file under the specified path
    print("Calculating MD5 values for files in the specified path", ret)

Getting robot version information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSoftwareVersion()``"
    "Description", "Get robot version information"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``robotModel``: robot model
    - ``webVersion``: web version
    - ``controllerVersion``: controller version"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    ret = robot.GetSoftwareVersion()
    print("GetSoftwareVersion():", ret)

Getting robot hardware version information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSlaveHardVersion()``"
    "Description", "Get robot hardware version information"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``ctrlBoxBoardVersion``: control box version
    - ``driver1Version``
    - ``driver2Version``
    - ``driver3Version``
    - ``driver4Version``
    - ``driver5Version``
    - ``driver6Version``
    - ``endBoardVersion``"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetSlaveHardVersion()
    print("GetSlaveHardVersion():", ret)

Getting robot firmware version information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSlaveFirmVersion()``"
    "Description", "Get information about the robot's firmware version."
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``ctrlBoxBoardVersion``: control box version
    - ``driver1Version``
    - ``driver2Version``
    - ``driver3Version``
    - ``driver4Version``
    - ``driver5Version``
    - ``driver6Version``
    - ``endBoardVersion``"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetSlaveFirmVersion()
    print("GetSlaveFirmVersion():", ret)

Get DH compensation parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetDHCompensation()``"
    "Description", "Get DH compensation parameters"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``dhCompensation=[cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]``: Robot DH Parameter Compensation Values (mm)"

code example
---------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.GetDHCompensation()
    print(error)

Getting the current torque of a joint actuator
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetJointDriverTorque()``"
    "Description", "Get the current torque of the joint actuator"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``data=[j1,j2,j3,j4,j5,j6]``: current torque of joint drive"

Get the current temperature of the joint drive
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetJointDriverTemperature()``"
    "Description", "Get the current temperature of the articulated drive"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``data=[t1,t2,t3,t4,t5,t6]``: current temperature of the joint drive"