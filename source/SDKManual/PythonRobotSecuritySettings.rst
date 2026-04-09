Security Settings
===================================================

.. toctree::
    :maxdepth: 5

Setting the collision level
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAnticollision (mode,level,config)``"
    "Description", "Setting the collision level"
    "Mandatory parameters", "- ``mode``: 0 - level, 1 - percentage;
    - ``level=[j1,j2,j3,j4,j5,j6]``: collision threshold;
    - ``config``: 0 - do not update configuration file, 1 - update configuration file"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the post-collision strategy
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetCollisionStrategy(strategy,safeTime,safeDistance,safeVel,safetyMargin)``"
    "Description", "Set post-crash strategy"
    "Mandatory parameters", "- ``strategy``: 0 - report error and pause, 1 - keep running, 2 - error stop, 3 - heavy moment mode, 4 - shock response mode, 5 - impact rebound mode"
    "Default parameters", "- ``safeTime``: safe stop time [1000-2000] ms, default: 1000
    - ``safeDistance``: safe stopping distance [1-150] mm, default: 100
    - ``safeVel``: safe stopping speed[50-250]mm/s, default: 250
    - ``safetyMargin[6]``: safety margin [1-10], default: [10,10,10,10,10,10,10]"
    "Return Value", "Error Code Success-0 Failure- errcode"

The Custom collision detection threshold function starts to set the collision detection thresholds of the joint end and TCP end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.0

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``CustomCollisionDetectionStart(flag, jointDetectionThreshould, tcpDetectionThreshould, block)``"
    "Description", "The Custom collision detection threshold function starts to set the collision detection thresholds of the joint end and TCP end"
    "Mandatory parameters", "- ``flag``: 1- Only joint detection is turned on; 2- Only TCP detection is enabled. 3- Joint and TCP detection are enabled simultaneously
    - ``jointDetectionThreshould``: Joint collision detection threshold j1-j6
    - ``tcpDetectionThreshould``: indicates TCP collision detection threshold, xyzabc
    - ``block``: 0- non-blocking; 1- block"
    "Default parameters", "NULL"
    "Return Value", "- errcode Success-0 Failure- errcode"

The custom collision detection threshold function is disabled
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.0

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``CustomCollisionDetectionEnd()``"
    "Description", "The custom collision detection threshold function is disabled"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errcode Success-0 Failure- errcode"

Robot collision level setting code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    mode = 0
    config = 1
    level1 = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    level2 = [50.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    rtn = robot.SetAnticollision(mode, level1, config)
    print(f"SetAnticollision mode 0 rtn is {rtn}")
    mode = 1
    rtn = robot.SetAnticollision(mode, level2, config)
    print(f"SetAnticollision mode 1 rtn is {rtn}")
    p1Joint = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    p2Joint = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    p1Desc = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    p2Desc = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot.MoveL(desc_pos=p2Desc, tool=0, user=0, vel=100, blendR=2)
    robot.ResetAllError()
    safety = [5, 5, 5, 5, 5, 5]
    rtn = robot.SetCollisionStrategy(3, 1000, 150, 250, safety)
    print(f"SetCollisionStrategy rtn is {rtn}")
    jointDetectionThreshould = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    tcpDetectionThreshould = [60, 60, 60, 60, 60, 60]
    rtn = robot.CustomCollisionDetectionStart(3, jointDetectionThreshould, tcpDetectionThreshould, 0)
    print(f"CustomCollisionDetectionStart rtn is {rtn}")
    robot.MoveL(desc_pos=p1Desc, tool=0, user=0, vel=100)
    robot.MoveL(desc_pos=p2Desc, tool=0, user=0, vel=100)
    rtn = robot.CustomCollisionDetectionEnd()
    print(f"CustomCollisionDetectionEnd rtn is {rtn}")
    robot.CloseRPC()

Setting the positive limit
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLimitPositive(p_limit)``"
    "Description", "Setting positive limits"
    "Mandatory parameters", "- ``p_limit=[j1,j2,j3,j4,j5,j6]``: six joint positions"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the negative limit
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLimitNegative(n_limit)``"
    "Description", "Setting negative limits"
    "Mandatory parameters", "- ``n_limit=[j1,j2,j3,j4,j5,j6]``: six joint positions"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

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

Robot limit setting code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    plimit = [170.0, 80.0, 150.0, 80.0, 170.0, 160.0]
    robot.SetLimitPositive(plimit)
    nlimit = [-170.0, -260.0, -150.0, -260.0, -170.0, -160.0]
    robot.SetLimitNegative(nlimit)
    error,neg_deg = robot.GetJointSoftLimitDeg(0)
    print(f"pos limit deg: {neg_deg[1]}, {neg_deg[3]}, {neg_deg[5]}, {neg_deg[7]}, {neg_deg[9]}, {neg_deg[11]}")
    print(f"neg limit deg: {neg_deg[0]}, {neg_deg[2]}, {neg_deg[4]}, {neg_deg[6]}, {neg_deg[8]}, {neg_deg[10]}")
    robot.CloseRPC()

Setting up a robot collision detection method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetCollisionDetectionMethod(method, thresholdMode)``"
    "Description", "Sets the robot collision detection method."
    "Mandatory parameters", "
    - ``method``: collision detection method: 0 - current mode; 1 - dual encoder; 2 - current and dual encoder on at the same time  
    - ``thresholdMode``: Collision level threshold method 0-Collision level fixed threshold mode 1- Customize collision detection thresholds
    "
    "Default parameters", "NULL"
    "Return Value", "- errcode Success-0 Failure- errcode"

Set static undercollision detection to start off
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetStaticCollisionOnOff(status)``"
    "Description", "Set static undercollision detection to start off"
    "Mandatory parameters", "
    - ``status``: 0 - off; 1 - on
    "
    "Default parameters", "NULL"
    "Return Value", "- errcode Success-0 Failure- errcode"

Set up the robot collision detection method code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    rtn = robot.SetCollisionDetectionMethod(0,0)
    rtn = robot.SetStaticCollisionOnOff(1)
    print(f"SetStaticCollisionOnOff On rtn is {rtn}")
    time.sleep(5)
    rtn = robot.SetStaticCollisionOnOff(0)
    print(f"SetStaticCollisionOnOff Off rtn is {rtn}")
    robot.CloseRPC()

Joint torque and power detection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetPowerLimit(status, power)``"
    "Description", "Joint torque and power detection"
    "Mandatory parameters", "
    - ``status``: 0 - off; 1 - on
    - ``power``：Set the maximum power (W)
    "
    "Default parameters", "NULL"
    "Return Value", "- errcode Success-0 Failure- errcode"
    
Sample code for joint torque power detection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.DragTeachSwitch(1)
    robot.SetPowerLimit(1, 200)
    error,torques = robot.GetJointTorques(1)
    count = 100
    robot.ServoJTStart()
    while count > 0:
        error = robot.ServoJT(torques, 0.001)
        count -= 1
        time.sleep(0.001)  # 1ms delay
    error = robot.ServoJTEnd()
    robot.DragTeachSwitch(0)
    robot.CloseRPC()

Set Safety Speed Parameters
+++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetVelReducePara(enable, maxTCPVel, strategy)``"
    "Description", "Set safety speed parameters"
    "Required Parameters", "
    - ``enable``: 0-off; 1-enabled in manual mode; 2-enabled in all modes (automatic speed limiting not supported)
    - ``maxTCPVel``: Maximum TCP speed limit; [0-1000] mm/s
    - ``strategy``: Strategy after overspeed; 0-stop with alarm; 1-automatic speed limiting; 2-stop with alarm and disable
    "
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure-errcode"

SDK Code Example for Setting Safety Speed Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from time import sleep
    import time
    from fairino import Robot

    # Establish connection with robot controller
    robot = Robot.RPC('192.168.58.2')

    def TestSetVelReducePara(self):
        # Initialize joint position, external axis and offset
        j1 = [0, -90, 90, 0, 0, 0]
        j2 = [90, -90, 90, 0, 0, 0]
        epos = [0, 0, 0, 0]
        offset_pos = [0, 0, 0, 0, 0, 0]

        # Set base speed
        robot.SetSpeed(80)

        # Test parameter error case (mode=2 invalid?)
        rtn = robot.SetVelReducePara(2, 30, 1)
        print(f"SetVelReducePara param error rtn is {rtn}")

        # Disable speed reduction function (mode=0, action=1 indicates disable)
        rtn = robot.SetVelReducePara(0, 30, 1)
        print(f"SetVelReducePara disable reduce vel rtn is {rtn}")
        robot.MoveJ(joint_pos=j1, tool=0, user=0, vel=100, acc=100, ovl=100,
                    exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
        robot.MoveJ(joint_pos=j2, tool=0, user=0, vel=100, acc=100, ovl=100,
                    exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)

        # Enable speed reduction function (mode=1, action=1)
        rtn = robot.SetVelReducePara(1, 30, 1)
        print(f"SetVelReducePara reduce vel rtn is {rtn}")
        robot.MoveJ(joint_pos=j1, tool=0, user=0, vel=100, acc=100, ovl=100,
                    exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
        robot.MoveJ(joint_pos=j2, tool=0, user=0, vel=100, acc=100, ovl=100,
                    exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)

        # Test action=2 (may indicate emergency stop or disable robot)
        rtn = robot.SetVelReducePara(2, 30, 2)
        print(f"SetVelReducePara disable robot rtn is {rtn}")
        robot.MoveJ(joint_pos=j1, tool=0, user=0, vel=100, acc=100, ovl=100,
                    exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
        robot.MoveJ(joint_pos=j2, tool=0, user=0, vel=100, acc=100, ovl=100,
                    exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)

        # Wait, reset errors and re-enable robot
        time.sleep(2)
        robot.ResetAllError()
        robot.RobotEnable(1)
        time.sleep(1)

        # Test action=0 (may indicate only report error, no action)
        rtn = robot.SetVelReducePara(2, 30, 0)
        print(f"SetVelReducePara report error rtn is {rtn}")
        robot.MoveJ(joint_pos=j1, tool=0, user=0, vel=100, acc=100, ovl=100,
                    exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
        robot.MoveJ(joint_pos=j2, tool=0, user=0, vel=100, acc=100, ovl=100,
                    exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)

        # Close connection
        robot.CloseRPC()

    TestSetVelReducePara(robot)