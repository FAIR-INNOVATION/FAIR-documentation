Status query
===============

.. toctree::
    :maxdepth: 5

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

Get joint feedback speed -deg/s
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualJointSpeedsDegree(flag=1)``"
    "Description", "Get joint feedback speed -deg/s"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``speed=[j1,j2,j3,j4,j5,j6]``: joint feedback speed -deg/s"

Obtain joint feedback acceleration-deg/s^2
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetActualJointAccDegree(flag=1)``"
    "Description", "Obtain joint feedback acceleration-deg/s^2"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``flag``： 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``acc=[j1,j2,j3,j4,j5,j6]``：Joint feedback acceleration-deg/s^2"

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

Query the cache length of the robot motion queue
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetMotionQueueLength()``"
    "Description", "Query the cache length of the robot motion queue"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``len``：Cache length"

Obtain the emergency stop status of the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotEmergencyStopState()``"
    "Description", "Obtain the emergency stop status of the robot"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``state``：Emergency stop status: 0- non-emergency stop, 1- emergency stop"

Obtain the communication status between the SDK and the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSDKComState()``"
    "Description", "Obtain the communication status between the SDK and the robot"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``state``：Communication status: 0- Normal communication, 1- Abnormal communication"

Obtain the safety stop signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSafetyStopState()``"
    "Description", "Obtain the safety stop signal"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``[si0_state,si1_state]``：si0_state safety stop signal SI0, 0- invalid, 1- valid si1_state safety stop signal SI1, 0- invalid, 1- valid"

Obtain the current temperature of the joint drive(℃)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetJointDriverTemperature()``"
    "Description", "Obtain the current temperature of the joint drive(℃)"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``data=[t1,t2,t3,t4,t5,t6]``：The current temperatures of each joint"

Obtain the current torque of the joint drive(Nm)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetJointDriverTorque()``"
    "Description", "Obtain the current torque of the joint drive(Nm)"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``data=[j1,j2,j3,j4,j5,j6]``：Joint torque [fx,fy,fz,tx,ty,tz]"

Obtain the status of the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotRealTimeState()``"
    "Description", "Obtain the status of the robot"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``robot_state_pkg``：Robot state structure"

Robot status query code example
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error,[yangle, zangle] = robot.GetRobotInstallAngle()
    print(f"yangle:{yangle},zangle:{zangle}")
    error,j_deg = robot.GetActualJointPosDegree(0)
    print(f"joint pos deg:{j_deg[0]},{j_deg[1]},{j_deg[2]},{j_deg[3]},{j_deg[4]},{j_deg[5]}")
    error,jointSpeed = robot.GetActualJointSpeedsDegree(0)
    print(f"joint speeds deg:{jointSpeed[0]},{jointSpeed[1]},{jointSpeed[2]},{jointSpeed[3]},{jointSpeed[4]},{jointSpeed[5]}")
    error,jointAcc = robot.GetActualJointAccDegree(0)
    print(f"joint acc deg:{jointAcc[0]},{jointAcc[1]},{jointAcc[2]},{jointAcc[3]},{jointAcc[4]},{jointAcc[5]}")
    error,[tcp_speed, ori_speed] = robot.GetTargetTCPCompositeSpeed(0)
    print(f"GetTargetTCPCompositeSpeed tcp {tcp_speed}; ori {ori_speed}")
    error,[tcp_speed, ori_speed] = robot.GetActualTCPCompositeSpeed(0)
    print(f"GetActualTCPCompositeSpeed tcp {tcp_speed}; ori {ori_speed}")
    error,targetSpeed = robot.GetTargetTCPSpeed(0)
    print(f"GetTargetTCPSpeed {targetSpeed[0]},{targetSpeed[1]},{targetSpeed[2]},{targetSpeed[3]},{targetSpeed[4]},{targetSpeed[5]}")
    error,actualSpeed = robot.GetActualTCPSpeed(0)
    print(f"GetActualTCPSpeed {actualSpeed[0]},{actualSpeed[1]},{actualSpeed[2]},{actualSpeed[3]},{actualSpeed[4]},{actualSpeed[5]}")
    error,tcp = robot.GetActualTCPPose(0)
    print(f"tcp pose:{tcp[0]},{tcp[1]},{tcp[2]},{tcp[3]},{tcp[4]},{tcp[5]}")
    error,flange = robot.GetActualToolFlangePose(0)
    print(f"flange pose:{flange[0]},{flange[1]},{flange[2]},{flange[3]},{flange[4]},{flange[5]}")
    error,id = robot.GetActualTCPNum(0)
    print(f"tcp num:{id}")
    error,id = robot.GetActualWObjNum(0)
    print(f"wobj num:{id}")
    error,jtorque = robot.GetJointTorques(0)
    print(f"torques:{jtorque[0]},{jtorque[1]},{jtorque[2]},{jtorque[3]},{jtorque[4]},{jtorque[5]}")
    error,t_ms = robot.GetSystemClock()
    print(f"system clock:{t_ms}")
    error,config = robot.GetRobotCurJointsConfig()
    print(f"joint config:{config}")
    error,motionDone = robot.GetRobotMotionDone()
    print(f"GetRobotMotionDone:{motionDone}")
    error,len = robot.GetMotionQueueLength()
    print(f"GetMotionQueueLength:{len}")
    error,emergState = robot.GetRobotEmergencyStopState()
    print(f"GetRobotEmergencyStopState:{emergState}")
    error,comstate = robot.GetSDKComState()
    print(f"GetSDKComState:{comstate}")
    error,[si0_state, si1_state] = robot.GetSafetyStopState()
    print(f"GetSafetyStopState:{si0_state} {si1_state}")
    error,temp = robot.GetJointDriverTemperature()
    print(f"Temperature:{temp[0]},{temp[1]},{temp[2]},{temp[3]},{temp[4]},{temp[5]}")
    error,torque = robot.GetJointDriverTorque()
    print(f"torque:{torque[0]},{torque[1]},{torque[2]},{torque[3]},{torque[4]},{torque[5]}")
    error,pkg = robot.GetRobotRealTimeState()
    robot.CloseRPC()

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

Inverse Kinematics Solution, Cartesian Space Includes Extended Axis Position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetInverseKinExaxis(type, desc_pos, exaxis, tool, workPiece)``"
    "Description", "Inverse kinematics solution, Cartesian space includes extended axis position"
    "Required Parameters", "- ``type``: 0-Absolute pose (base coordinate system), 1-Incremental pose (base coordinate system), 2-Incremental pose (tool coordinate system)
    - ``desc_pos``: Cartesian pose
    - ``exaxis``: Extended axis position
    - ``tool``: Tool number
    - ``workPiece``: Workpiece number"
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure- errcode
    - ``joint_pos``: Joint position"

Inverse Kinematics Solution Including Extended Axis Position Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    robot = Robot.RPC('192.168.58.2')
    desc = [99.957, -0.002, 29.994, -176.569, -6.757, -167.462]
    exaxis = [100.0, 0.0, 0.0, 0.0]
    jointPos = [0.0] * 6
    offsetPos = [0.0] * 6
    rtn,pkg = robot.GetRobotRealTimeState()
    toolnum = pkg.tool
    workPcsNum = pkg.user
    rtn, jointPos = robot.GetInverseKinExaxis(0, desc, exaxis, toolnum, workPcsNum)
    print(f"GetInverseKinExaxis joint is {jointPos[0]},{jointPos[1]},{jointPos[2]},{jointPos[3]},{jointPos[4]},{jointPos[5]}")
    robot.ExtAxisMove(exaxis, 100, -1)
    robot.MoveJ(joint_pos=jointPos, desc_pos=desc, tool=toolnum, user=workPcsNum, vel=100.0, acc=100.0, ovl=100.0, exaxis_pos=exaxis, blendT=-1, offset_flag=0, offset_pos=offsetPos)
    robot.CloseRPC()
    return 0

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

Example code for robot forward and inverse kinematics calculation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    error, inverseRtn = robot.GetInverseKin(0, desc_pos=desc_pos1, config=-1)
    print(f"dcs1 GetInverseKin rtn is {inverseRtn[0]}, {inverseRtn[1]}, {inverseRtn[2]}, "
          f"{inverseRtn[3]}, {inverseRtn[4]}, {inverseRtn[5]}")
    error, inverseRtn = robot.GetInverseKinRef(0, desc_pos=desc_pos1, joint_pos_ref=j1)
    print(f"dcs1 GetInverseKinRef rtn is {inverseRtn[0]}, {inverseRtn[1]}, {inverseRtn[2]}, "
          f"{inverseRtn[3]}, {inverseRtn[4]}, {inverseRtn[5]}")
    error, hasResult = robot.GetInverseKinHasSolution(0, desc_pos=desc_pos1, joint_pos_ref=j1)
    print(f"dcs1 GetInverseKinRef result {hasResult}")
    error, forwordResult = robot.GetForwardKin(j1)
    print(f"jpos1 forwordResult rtn is {forwordResult[0]}, {forwordResult[1]}, {forwordResult[2]}, "
          f"{forwordResult[3]}, {forwordResult[4]}, {forwordResult[5]}")
    robot.CloseRPC()

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

Obtain the SN code of the control box
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotSN()``"
    "Description", "Obtain the SN code of the control box"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``SNCode``: SN of the control box"

Query robot teaching management point data code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    name = "P1"
    rtn, data = robot.GetRobotTeachingPoint(name)
    print(f"{rtn} name is: {name}")
    for i in range(20):
        print(f"data is: {data[i]}")
    rtn,que_len = robot.GetMotionQueueLength()
    print(f"GetMotionQueueLength rtn is: {rtn}, queue length is: {que_len}")
    retval,dh = robot.GetDHCompensation()
    print(f"retval is: {retval}")
    print(f"dh is: {dh[0]} {dh[1]} {dh[2]} {dh[3]} {dh[4]} {dh[5]}")
    error,sn = robot.GetRobotSN()
    print(f"robot SN is {sn[0]}")
    robot.CloseRPC()

Get the tool coordinate system according to the number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetToolCoordWithID(id)``"
    "Description", "Get the tool coordinate system according to the number"
    "Mandatory parameters", "- ``id``: Tool coordinate system number"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``coord``: Coordinate system value"

The workpiece coordinate system is obtained according to the No
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetWObjCoordWithID(id)``"
    "Description", "The workpiece coordinate system is obtained according to the No"
    "Mandatory parameters", "- ``id``: Workpiece coordinate system number"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``coord``: Coordinate system value"

The external tool coordinate system is obtained according to the number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetExToolCoordWithID(id)``"
    "Description", The external tool coordinate system is obtained according to the number
    "Mandatory parameters", "- ``id``：External tool coordinate system No"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``coord``: Coordinate system value"

The extended axis coordinate system is obtained according to the No
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetExAxisCoordWithID(id)``"
    "Description", "The extended axis coordinate system is obtained according to the No"
    "Mandatory parameters", "- ``id``：Extended axis coordinate system No"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``coord``: Coordinate system value"

Get the load mass and centroid according to the number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetPayloadWithID(id)``"
    "Description", "Get the load mass and centroid according to the number"
    "Mandatory parameters", "- ``id``：Extended axis coordinate system No"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``weight``: Quality of load
    - ``cog``: Centroid of the load"

Gets the current tool coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetCurToolCoord()``"
    "Description", "Gets the current tool coordinate system"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``coord``: Coordinate system value"

Gets the current workpiece coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetCurWObjCoord()``"
    "Description", "Gets the current workpiece coordinate system"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``coord``: Coordinate system value"

Gets the current external tool coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetCurExToolCoord()``"
    "Description", "Gets the current external tool coordinate system"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``coord``: Coordinate system value"

Gets the current extended axis coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetCurExAxisCoord()``"
    "描述", "Gets the current extended axis coordinate system"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``coord``: Coordinate system value"

Get robot coordinate system and load code sample
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    id = 1
    toolCoord = [0.0] * 6
    extoolCoord = [0.0] * 6
    wobjCoord = [0.0] * 6
    exAxisCoord = [0.0] * 6
    for i in range(100):
        print(f"当前ID为:{id}")
        coordSet0 = [0.0] * 6
        coordSet = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        etcp = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        etool = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        cog = [1.0, 2.0, 3.0]
        if i % 2 == 0:
            robot.SetToolCoord(id, coordSet, 0, 0, 1, 0)
            time.sleep(0.1)
            robot.SetWObjCoord(id, coordSet, 0)
            time.sleep(0.1)
            robot.ExtAxisActiveECoordSys(id, 1, coordSet, 1)
            time.sleep(0.1)
            rtn = robot.SetExToolCoord(id, etcp, etool)
            time.sleep(0.1)
            rtn = robot.SetLoadWeight(id, 1.5)
            time.sleep(0.1)
            rtn = robot.SetLoadCoord(cog[0],cog[1],cog[2],id)
            time.sleep(0.1)
        else:
            robot.SetToolCoord(id, coordSet0, 0, 0, 1, 0)
            time.sleep(0.1)
            robot.SetWObjCoord(id, coordSet0, 0)
            time.sleep(0.1)
            robot.ExtAxisActiveECoordSys(id, 1, coordSet0, 1)
            time.sleep(0.1)
            rtn = robot.SetExToolCoord(id, coordSet0, coordSet0)
            time.sleep(0.1)
            rtn = robot.SetLoadWeight(id, 0)
            time.sleep(0.1)
            rtn = robot.SetLoadCoord(coordSet0[0],coordSet0[1],coordSet0[2] , id)
            time.sleep(0.1)
        rtn, toolCoord = robot.GetCurToolCoord()
        print(f"GetToolCoord {toolCoord[0]},{toolCoord[1]},{toolCoord[2]},{toolCoord[3]},{toolCoord[4]},{toolCoord[5]}")
        rtn, wobjCoord = robot.GetCurWObjCoord()
        print(f"GetWObjCoord {wobjCoord[0]},{wobjCoord[1]},{wobjCoord[2]},{wobjCoord[3]},{wobjCoord[4]},{wobjCoord[5]}")
        rtn, extoolCoord = robot.GetCurExToolCoord()
        print(f"GetExToolCoord {extoolCoord[0]},{extoolCoord[1]},{extoolCoord[2]},{extoolCoord[3]},{extoolCoord[4]},{extoolCoord[5]}")
        rtn, exAxisCoord = robot.GetCurExAxisCoord()
        print(f"GetExAxisCoord {exAxisCoord[0]},{exAxisCoord[1]},{exAxisCoord[2]},{exAxisCoord[3]},{exAxisCoord[4]},{exAxisCoord[5]}")
        weight = 0.0
        getCog = [0.0] * 3
        rtn, weight = robot.GetTargetPayload(0)
        rtn, getCog = robot.GetTargetPayloadCog(0)
        print(f"GetTargetPayload {weight},{getCog[0]},{getCog[1]},{getCog[2]}")
        
        rtn, toolCoord = robot.GetToolCoordWithID(id)
        print(f"GetToolCoordWithID {id},{toolCoord[0]},{toolCoord[1]},{toolCoord[2]},{toolCoord[3]},{toolCoord[4]},{toolCoord[5]}")
        rtn, wobjCoord = robot.GetWObjCoordWithID(id)
        print(f"GetWObjCoordWithID {id},{wobjCoord[0]},{wobjCoord[1]},{wobjCoord[2]},{wobjCoord[3]},{wobjCoord[4]},{wobjCoord[5]}")
        rtn, extoolCoord = robot.GetExToolCoordWithID(id)
        print(f"GetExToolCoordWithID {id},{extoolCoord[0]},{extoolCoord[1]},{extoolCoord[2]},{extoolCoord[3]},{extoolCoord[4]},{extoolCoord[5]}")
        rtn, exAxisCoord = robot.GetExAxisCoordWithID(id)
        print(f"GetExAxisCoordWithID {id},{exAxisCoord[0]},{exAxisCoord[1]},{exAxisCoord[2]},{exAxisCoord[3]},{exAxisCoord[4]},{exAxisCoord[5]}")
        weight = 0.0
        getCog = [0.0] * 3
        rtn, weight, getCog = robot.GetTargetPayloadWithID(id)
        print(f"GetTargetPayloadWithID {id},{weight},{getCog[0]},{getCog[1]},{getCog[2]}")
        time.sleep(0.5)
        print(f"times {i}")