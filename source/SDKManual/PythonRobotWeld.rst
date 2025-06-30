Weld
======================

.. toctree::
    :maxdepth: 5

Welding Start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ARCStart(ioType, arcNum, timeout)``"
    "Description", "Welding Start"
    "Required Parameters", "- ``ioType``: io type 0 - controller IO; 1 - extended IO
    - ``arcNum``: Welder profile number
    - ``timeout``: timeout for starting an arc"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    weldIOType =0
    arcNum = 0
    weldTimeout=5000
    # start an arc
    ret = robot.ARCStart(weldIOType,arcNum,weldTimeout)
    print("ARCStart error code", ret)
    time.sleep(3)

End of welding
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``ARCEnd(ioType, arcNum, timeout)``"
    "Description", "End of welding"
    "Mandatory parameters", "- ``ioType``: type 0 - controller IO; 1 - extended IO
    - ``arcNum``: Welder profile number
    - ``timeout``: timeout for starting an arc"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    weldIOType =0
    arcNum = 0
    weldTimeout=5000
    # Close the arc
    ret = robot.ARCEnd(weldIOType,arcNum,weldTimeout)
    print("ARCEnd error code", ret)
    time.sleep(3)

Setting of welding current and output analog correspondences
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingSetCurrentRelation(currentMin, currentMax, outputVoltageMin, outputVoltageMax)``"
    "Description", "Setting the welding current to correspond to the output analog"
    "Mandatory parameters", "- ``currentMin``: Welding current - analog output linear relationship left point current value (A)
    - ``currentMax``: Welding current - analog output linear relationship right point current value (A)
    - ``outputVoltageMin``: analog output voltage value (V) at the left point of the welding current-analog output linear relationship.
    - ``outputVoltageMax``: analog output voltage value (V) at the point on the right side of the weld current-analog output linear relationship.
    - ``AOIndex``: analog output port for welding current"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    weldIOType =0

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    weldIOType =0
    arcNum = 0
    weldTimeout=5000

    # Set weld current linearly to analog
    ret = robot.WeldingSetCurrentRelation(0,400,0,10,0)
    print("WeldingSetCurrentRelation", ret)
    time.sleep(1)
    # Get a linear relationship between welding current and analog
    ret = robot.WeldingGetCurrentRelation()
    print("WeldingGetCurrentRelation", ret)
    time.sleep(1)

    #Set weld voltage vs. analog linear relationship
    ret = robot.WeldingSetVoltageRelation(0,400,0,10,0)
    print("WeldingSetVoltageRelation", ret)
    time.sleep(1)
    #Get a linear relationship between weld voltage and analog
    ret = robot.WeldingGetVoltageRelation()
    print("WeldingGetVoltageRelation", ret)
    time.sleep(1)

Setting the welding voltage and output analog correspondence
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingSetVoltageRelation(weldVoltageMin, weldVoltageMax, outputVoltageMin, outputVoltageMax)``"
    "Description", "Setting the weld voltage to correspond to the output analog"
    "Mandatory parameters", "- ``weldVoltageMin``: Welding voltage - analog output linear relationship left point welding voltage value (A)
    - ``weldVoltageMax``: Welding voltage - analog output linear relationship right point welding voltage value (A)
    - ``outputVoltageMin``: analog output voltage value (V) at the left point of the welding voltage-analog output linear relationship.
    - ``outputVoltageMax``: analog output voltage value (V) at the point on the right side of the weld voltage-analog output linear relationship.
    - ``AOIndex``: welding voltage analog output port"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Acquiring the correspondence between welding current and output analog quantity
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingGetCurrentRelation()``"
    "Description", "Get weld current to output analog correspondence"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``currentMin``: welding current - analog output linear relationship left point current value (A)
    - ``currentMax``: welding current - analog output linear relationship right point current value (A)
    - ``outputVoltageMin``: analog output voltage value at the left point of the welding current-analog output linear relationship (V)
    - ``outputVoltageMax``: analog output voltage value (V) at the point on the right side of the weld current-analog output linear relationship.
    - ``AOIndex``: welding voltage analog output port"

Getting welding voltage and output analog correspondence
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingGetVoltageRelation()``"
    "Description", "Get weld voltage to output analog correspondence"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``weldVoltageMin``: Welding voltage - analog output linear relationship left point welding voltage value (V)
    - ``weldVoltageMax``: Welding voltage - analog output linear relationship of the right point welding voltage value (V)
    - ``outputVoltageMin``: Analog output voltage value at the left point of the welding voltage-analog output linear relationship (V)
    - ``outputVoltageMax``: Analog output voltage value (V) at the right point of the weld current-analog output linear relationship
    - ``AOIndex``: welding voltage analog output port"

Setting the welding current
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingSetCurrent(ioType, current, AOIndex, blend)``"
    "Description", "Setting the welding current"
    "Mandatory parameters", "- ``ioType``: type 0 - controller IO; 1 - extended IO
    - ``current``: Welding current value (A)
    - ``AOIndex``: Analog output port (0-1) of the welding current control box
    - ``blend``: smooth or not 0 - not smooth, 1 - smooth"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the welding voltage
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingSetVoltage(ioType, voltage, AOIndex, blend)``"
    "Description", "Set weld voltage"
    "Mandatory parameters", "- ``ioType``: type 0 - controller IO; 1 - extended IO
    - ``voltage``: Welding voltage value (V)
    - ``AOIndex``: Analog output port (0-1) of the welding current control box
    - ``blend``: smooth or not 0 - not smooth, 1 - smooth"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting Oscillation Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Python SDK-v2.0.8-3.7.8
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeaveSetPara(weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange, weaveLeftRange, weaveRightRange. additionalStayTime, weaveLeftStayTime, weaveRightStayTime, weaveCircleRadio, weaveStationary, weaveYawAngle=0)``"
    "Description", "Setting swing parameters"
    "Mandatory parameters", "- ``weaveNum``: Pendulum welding parameter configuration number
    - ``weaveType``: type of oscillation 0-planar triangular oscillation; 1-vertical L-shaped triangular oscillation; 2-clockwise circular oscillation; 3-counterclockwise circular oscillation; 4-planar sinusoidal oscillation; 5-vertical L-shaped sinusoidal oscillation; 6-vertical triangular oscillation; 7-vertical sinusoidal oscillation.
    - ``weaveFrequency``: swing frequency (Hz)
    - ``weaveIncStayTime``: wait mode 0-cycle without wait time; 1-cycle with wait time mandatory parameter
    - ``weaveRange``: swing range (mm)
    - ``weaveLeftRange``:  vertical triangle swing left chord length(mm)
    - ``weaveRightRange``:  vertical triangle swing right chord length(mm)
    - ``additionalStayTime``:  vertical triangle swing vertical triangle point residence time(mm)
    - ``weaveLeftStayTime``: wave left stay time (ms)
    - ``weaveRightStayTime``: wave right stay time (ms)
    - ``weaveCircleRadio``: Circle swing-back ratio (0-100%)
    - ``weaveStationary``: swing position wait, 0 - position continues to move during wait time; 1 - position is stationary during wait time"
    "Default parameters", "- ``weaveYawAngle``: azimuth angle of the swing direction (rotation around the swing Z-axis) in °, default 0
    - ``weaveRotAngle``: Swing direction azimuth (rotation around the X-axis of the swing), unit °, default 0"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    weaveNum =0
    weaveType = 0
    weaveFraquency = 1
    weavelncStayTime = 0
    weaveRange = 10
    weaveLeftStayTime = 10
    weaveRightStayTime = 10
    weaveCircleRadio = 0
    weaveStationary =1
    # Setting the swing parameters
    ret = robot.WeaveSetPara(weaveNum,weaveType,weaveFraquency,weavelncStayTime,weaveRange,weaveLeftStayTime,weaveRightStayTime, weaveCircleRadio,weaveStationary)
    print("WeaveSetPara ", ret)
    time.sleep(1)

    # Swing to start
    ret = robot.WeaveStart(0)
    print("WeaveStart ", ret)
    time.sleep(1)
    ret,pose =robot.GetActualTCPPose(1);
    print(ret,pose)
    pose[2]=pose[2]+50
    ret = robot.MoveL(pose,tool,user)
    print("MoveL ", ret)
    time.sleep(1)
    # Instantly set swing parameters
    ret = robot.WeaveOnlineSetPara (weaveNum,weaveType,weaveFraquency,weavelncStayTime,weaveRange,weaveLeftStayTime,weaveRightStayTime, weaveCircleRadio,weaveStationary)
    print("WeaveOnlineSetPara ", ret)
    time.sleep(1)
    #Swing ends
    ret = robot.WeaveEnd(0)
    print("WeaveEnd ", ret)
    time.sleep(1)

Instant setup of swing parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeaveOnlineSetPara (weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange, weaveLeftStayTime, weaveRightStayTime. weaveCircleRadio, weaveStationary)``"
    "Description", "Set swing parameters on the fly"
    "Mandatory parameters", "- ``weaveNum``: Pendulum welding parameter configuration number
    - ``weaveType``: type of oscillation 0-planar triangular oscillation; 1-vertical L-shaped triangular oscillation; 2-clockwise circular oscillation; 3-counterclockwise circular oscillation; 4-planar sinusoidal oscillation; 5-vertical L-shaped sinusoidal oscillation; 6-vertical triangular oscillation; 7-vertical sinusoidal oscillation.
    - ``weaveFrequency``: swing frequency (Hz)
    - ``weaveIncStayTime``: wait mode 0-cycle without wait time; 1-cycle with wait time mandatory parameter
    - ``weaveRange``: swing range (mm)
    - ``weaveLeftStayTime``: wave left stay time (ms)
    - ``weaveRightStayTime``: wave right stay time (ms)
    - ``weaveCircleRadio``: Circle swing-back ratio (0-100%)
    - ``weaveStationary``: swing position wait, 0 - position continues to move during wait time; 1 - position is stationary during wait time"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

swing start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeaveStart(weaveNum)``"
    "Description", "Swing Start"
    "Mandatory parameters", "- ``weaveNum``: type 0 - controller IO; 1 - extended IO"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

end of swing (math.)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeaveEnd(weaveNum)``"
    "description", "end of swing"
    "Mandatory parameter", "- ``weaveNum``: Pendulum welding parameter configuration number"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Positive wire feed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetForwardWireFeed(ioType, wireFeed)``"
    "Description", "Positive Wire Feed"
    "Mandatory parameters", "- ``ioType``: 0 - controller IO; 1 - extended IO
    - ``wireFeed``: wire feed control 0 - stop wire feed; 1 - wire feed."
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    weldIOType =0
    # Positive wire feed
    ret = robot.SetForwardWireFeed(weldIOType,1)
    print("SetForwardWireFeed error code", ret)
    time.sleep(1)
    ret = robot.SetForwardWireFeed(weldIOType,0)
    print("SetForwardWireFeed error code", ret)
    time.sleep(1)

    # Reverse wire feed
    ret = robot.SetReverseWireFeed(weldIOType,1)
    print("SetReverseWireFeed error code", ret)
    time.sleep(1)
    # Stop reverse wire feed
    ret = robot.SetReverseWireFeed(weldIOType,0)
    print("SetReverseWireFeed error code", ret)
    time.sleep(1)

    # Send gas
    ret = robot.SetAspirated(weldIOType,1)
    print("SetAspirated error code", ret)
    time.sleep(1)
    #Stopped air delivery
    ret = robot.SetAspirated(weldIOType,0)
    print("SetAspirated error code", ret)
    time.sleep(1)

Reverse wire feed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetReverseWireFeed(ioType, wireFeed)``"
    "Description", "Reverse Wire Feed"
    "Mandatory parameters", "- ``ioType``: 0 - controller IO; 1 - extended IO
    - ``wireFeed``: wire feed control 0 - stop wire feed; 1 - wire feed."
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

aspiration (phonetics, explosion of breath on consonants distinguishing Chinese p, t from b, d)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAspirated(ioType, airControl)``"
    "Description", "Air delivery"
    "Mandatory parameters", "- ``ioType``: 0 - controller IO; 1 - extended IO
    - ``airControl``: air delivery control 0 - stop air delivery; 1 - air delivery"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Segment welding for position and attitude acquisition
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSegmentWeldPoint(startPos, endPos, startDistance)``"
    "Description", "Segment Welding Acquisition Position and Attitude"
    "Mandatory parameters", "- ``startPos=[x,y,z,rx,ry,rz]``: coordinates of the start point
    - ``endPos=[x,y,z,rx,ry,rz]``: coordinates of end point
    - ``startDistance``: the length from the weld point to the start point"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``weldPointDesc=[x,y,z,rx,ry,rz]``: Cartesian coordinate information for the weld points 
    - ``weldPointJoint=[j1,j2,j3,j4,j5,j6]``: information on the joint coordinates of the weld point
    - ``tool``: tool number
    - ``user``: workpiece number"

Segmented welding startup
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype","``SegmentWeldStart(startDesePos, endDesePos, startJPos, endJPos, weldLength, noWeldLength, weldIOType, arcNum, weldTimeout, isWeave. weaveNum,tool,user,vel=20.0, acc=0.0, ovl=100.0, blendR=-1.0,exaxis_pos=[0.0, 0.0, 0.0, 0.0], search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])``"
    "Description", "Segmented Welding Initiation"
    "Mandatory parameters", "- ``startDesePos``: Initial Cartesian position in [mm][°].
    - ``endDesePos``: target Cartesian position in [mm][°].
    - ``startJPos``: initial joint position in [°]. 
    - ``endJPos``: target joint position in [°]  
    - ``weldLength``: weld length in [mm] 
    - ``noWeldLength``: non-weld length in [mm] 
    - ``weldIOType``: weld IO type (0-control box IO; 1-expansion IO) arcNum welder profile number 
    - ``timeout``: timeout for arc extinguishing 
    - ``isWeave``: welded False -- not welded 
    - ``weaveNum``: Pendulum welding parameter configuration number 
    - ``tool``: tool number, [0 to 14]
    - ``user``: artifact number, [0 to 14]"
    "Default parameters", "- ``vel``: percentage of speed, [0~100] default 20.0
    - ``acc``: acceleration [0~100] Not open Default 0.0
    - ``ovl``: velocity scaling factor, [0~100] default 100.0
    - ``blendR``: [-1.0] - motion in place (blocking), [0~1000] - smoothing radius (non-blocking), unit [mm] default -1.0
    - ``exaxis_pos``: external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0]
    - ``search``: [0] - no wire seek, [1] - wire seek
    - ``offset_flag``: [0] - no offset, [1] - offset in workpiece/base coordinate system, [2] - offset in tool coordinate system Default 0
    - ``offset_pos``: position offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0]"
    "Return Value", "- errcode Success-0 Failure- errcode"

Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    weldIOType =0
    arcNum = 0
    weldTimeout=5000
    weaveNum =0
    tool =1
    user =0
    weaveType = 0
    weaveFraquency = 1
    weavelncStayTime = 0
    weaveRange = 10
    weaveLeftStayTime = 10
    weaveRightStayTime = 10
    weaveCircleRadio = 0
    weaveStationary =1
    start_desc=[0,0,0,0,0,0,0]
    end_desc=[0,0,0,0,0,0,0]
    start_joint=[0,0,0,0,0,0,0]
    end_joint=[0,0,0,0,0,0,0]
    ret,start_desc =robot.GetActualTCPPose(1);
    print("start_desc",start_desc)
    ret,end_desc =robot.GetActualTCPPose(1);
    end_desc[1]=end_desc[1]+200
    print("start_desc",start_desc)
    print("end_desc",end_desc)
    ret,start_joint=robot.GetInverseKin(0,start_desc)
    ret,end_joint=robot.GetInverseKin(0,end_desc)
    print("start_joint",start_joint)
    print("end_joint",end_joint)
    weldLength =40
    noweldLength =40
    # Segment welding

    ret = robot.SegmentWeldStart(start_desc,end_desc,start_joint,end_joint,weldLength,noweldLength,weldIOType,arcNum,weldTimeout,True, weaveNum,tool,user)
    print("SegmentWeldStart", ret)

Segmented welding termination
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SegmentWeldEnd(ioType, arcNum, timeout)``"
    "Description", "Segmented weld termination"
    "Required Parameters", "- ``ioType``: io type 0 - controller IO; 1 - extended IO
    - ``arcNum``: welder profile number
    - ``timeout``: timeout for arc extinguishing"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Welding wire position finding start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WireSearchStart(refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag)``"
    "Description", "Welding wire seek start"
    "Mandatory parameters", "- ``refPos``: 1 - reference point 2 - contact point
    - ``searchVel``: search velocity %
    - ``searchDis``: search distance mm
    - ``autoBackFlag``: autoBackFlag, 0 - not auto; - auto
    - ``autoBackVel``: Automatic Back Velocity %
    - ``autoBackDis``: automatic back distance mm
    - ``offectFlag``: 1 - seek with offset; 2 - teach point seek"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    refPos = 1 # 1-datum 2-contact point
    searchVel = 10 #searchVelocity %
    searchDis = 100 #search distance mm
    autoBackFlag = 0 #autoBackFlag, 0-not auto; 1-auto
    autoBackVel = 10 #autoBackVel %
    autoBackDis = 100 #autoBackDistance mm
    offectFlag = 0 #1-with offset seek; 2-teach point seek
    descStart = [203.061, 56.768, 62.719, -177.249, 1.456, -83.597]
    jointStart = [-127.012, -112.931, -94.078, -62.014, 87.186, 91.326]
    descEnd = [122.471, 55.718, 62.209, -177.207, 1.375, -76.310]
    jointEnd = [-119.728, -113.017, -94.027, -62.061, 87.199, 91.326]

    robot.MoveL(descStart,1,1,joint_pos= jointStart,vel=100)
    robot.MoveL(descEnd,1,1,joint_pos= jointEnd,vel=100)

    descREF0A = [147.139, -21.436, 60.717, -179.633, -3.051, -83.170]
    jointREF0A = [-121.731, -106.193, -102.561, -64.734, 89.972, 96.171]

    descREF0B = [139.247, 43.721, 65.361, -179.634, -3.043, -83.170]
    jointREF0B = [-122.364, -113.991, -90.860, -68.630, 89.933, 95.540]

    descREF1A = [289.747, 77.395, 58.390, -179.074, -2.901, -89.790]
    jointREF1A = [-135.719, -119.588, -83.454, -70.245, 88.921, 88.819]

    descREF1B = [259.310, 79.998, 64.774, -179.073, -2.900, -89.790]
    jointREF1B = [-133.133, -119.029, -83.326, -70.976, 89.069, 91.401]

    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print("WireSearchStart return:",error)

    robot.MoveL(descREF0A,1,1, joint_pos = jointREF0A, vel=100)
    print("MoveL(descREF0A return:",error)
    robot.MoveL(descREF0B,1,1, joint_pos = jointREF0B, vel=10,search=1)
    print("MoveL(descREF0B return:",error)

    error =robot.WireSearchWait("REF0")
    print("WireSearchWait return:",error)

    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    print("WireSearchEnd return:",error)

    error = robot.WireSearchStart(1,10,100,0,10,100,0)
    print("WireSearchStart return:",error)

    robot.MoveL(descREF1A,1,1, joint_pos = jointREF1A, vel=100)
    robot.MoveL(descREF1B,1,1, joint_pos = jointREF1B, vel=10,search=1)

    error =robot.WireSearchWait("REF1")
    print("WireSearchWait return:",error)

    error = robot.WireSearchEnd(1,10,100,0,10,100,0)
    print("WireSearchEnd return:",error)

    error = robot.WireSearchStart(1,10,100,0,10,100,0)
    print("WireSearchStart return:",error)

    robot.MoveL(descREF0A,1,1, joint_pos = jointREF0A, vel=100)
    robot.MoveL(descREF0B,1,1, joint_pos = jointREF0B, vel=10,search=1)

    error =robot.WireSearchWait("RES0")
    print("WireSearchWait return:",error)

    error = robot.WireSearchEnd(1,10,100,0,10,100,0)
    print("WireSearchEnd return:",error)

    error = robot.WireSearchStart(1,10,100,0,10,100,0)
    print("WireSearchStart return:",error)

    robot.MoveL(descREF1A,1,1, joint_pos = jointREF1A, vel=100)
    robot.MoveL(descREF1B,1,1, joint_pos = jointREF1B, vel=10,search=1)

    error =robot.WireSearchWait("RES1")
    print("WireSearchWait return:",error)

    error = robot.WireSearchEnd(1,10,100,0,10,100,0)
    print("WireSearchEnd return:",error)

    varNameRef = ["REF0", "REF1", "#", "#", "#", "#"]
    varNameRes = ["RES0", "RES1", "#", "#", "#", "#"]
    error = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes)
    print("GetWireSearchOffect return:",error)
    if error[0]==0.
        ref = error[1]
        offdesc =error[2]

        error = robot.PointsOffsetEnable(ref,offdesc)
        print("PointsOffsetEnable return:",error)

        error = robot.MoveL(descStart, 1, 1, joint_pos=jointStart, vel=100)
        print("MoveL return:",error)
        robot.MoveL(descEnd, 1, 1, joint_pos=jointEnd, vel=10)
        print("MoveL return:",error)
        error = robot.PointsOffsetDisable()
        print("PointsOffsetDisable return:",error)

End of wire position finding
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WireSearchEnd(refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag)``"
    "Description", "Welding wire seek end"
    "Mandatory parameters", "- ``refPos``: 1 - reference point 2 - contact point
    - ``searchVel``: search velocity %
    - ``searchDis``: search distance mm
    - ``autoBackFlag``: autoBackFlag, 0 - not auto; - auto
    - ``autoBackVel``: Automatic Back Velocity %
    - ``autoBackDis``: automatic back distance mm
    - ``offectFlag``: 1 - seek with offset; 2 - teach point seek"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Calculate the wire finding offset
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetWireSearchOffset(seamType, method,varNameRef,varNameRes)``"
    "Description", "Calculate the welding wire seek offset"
    "Mandatory parameters", "- ``seamType``: weld type
    - ``method``: method of calculation
    - ``varNameRef``: datums 1-6, “#” denotes a pointless variable
    - ``varNameRes``: Contacts 1-6, “#” for dotless variables"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``offsetFlag``: 0 - the offset is superimposed directly on the instruction point; 1 - the offset requires a coordinate transformation of the instruction point
    - ``offset``: offset position [x, y, z, a, b, c]"

Waiting for wire seek to complete
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WireSearchWait(varname)``"
    "Description", "Waiting for wire seek to complete."
    "Mandatory parameters", "- ``varName``: contact name “RES0” ~ “RES99”"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 

Wire seek contact points written to database
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetPointToDatabase(varName,pos)``"
    "Description", "Welding wire seeker contact points written to database"
    "Mandatory parameters", "- ``varName``: contact name “RES0” ~ “RES99”
    - ``pos``: contact point data [x, y, x, a, b, c]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 

Arc tracking control
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ArcWeldTraceControl(flag,delaytime, isLeftRight, klr, tStartLr, stepMaxLr, sumMaxLr, isUpLow, kud, tStartUd, stepMaxUd, sumMaxUd. axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent, offsetType, offsetParameter)``"
    "Description", "Arc tracking control"
    "Mandatory parameters", "- ``flag``: switch, 0-off; 1-on
    - ``delayTime``: lag time in ms
    - ``isLeftRight``: left-right deviation compensation 0 - off, 1 - on
    - ``klr``: left/right adjustment factor (sensitivity)
    - ``tStartLr``: left and right start compensation time cyc
    - ``stepMaxLr``: Maximum compensation in mm for each left and right step.
    - ``sumMaxLr``: total maximum compensation left and right mm
    - ``isUpLow``: up and down deviation compensation 0 - off, 1 - on
    - ``kud``: upward and downward adjustment factor (sensitivity)
    - ``tStartUd``: up and down start compensation time cyc
    - ``stepMaxUd``: Maximum compensation per step up and down mm
    - ``sumMaxUd``: total maximum compensation up and down
    - ``axisSelect``: upper and lower coordinate system selection, 0 - swing; 1 - tool; 2 - base
    - ``referenceType``: upper and lower reference current setting mode, 0 - feedback; 1 - constant
    - ``referSampleStartUd``: upper and lower reference current sampling start count (feedback), cyc
    - ``referSampleCountUd``: upper and lower reference current sample cycle count (feedback), cyc
    - ``referenceCurrent``: upper and lower reference currents mA
    - ``offsetType``: bias tracking type, 0- no bias; 1- Sampling; 2- percent
    - ``offsetParameter``: bias parameter; Sampling (offset sampling start time, default sampling cycle); Percentage (offset percentage (-100 ~ 100))"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 

Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    flag = 1 # switch, 0-off; 1-on
    delaytime=0 # lag time in ms
    isLeftRight=0 #Left/Right Deviation Compensation 0-Off, 1-On
    klr = 0.06 # left/right adjustment factor (sensitivity)
    tStartLr = 5 # around start compensation time cyc
    stepMaxLr =5 #Maximum compensation in mm at a time, left or right
    sumMaxLr = 300 # Total maximum compensation left and right mm
    isUpLow = 1 #Up and down offset compensation
    kud =-0.06 #Up and down adjustment factor (sensitivity)
    tStartUd = 5 # up and down start compensation time cyc
    stepMaxUd = 5 # Maximum compensation per step up and down in mm
    sumMaxUd = 300 # total maximum compensation up and down
    axisSelect = 1 # up/down coordinate system selection, 0-swing; 1-tool; 2-base
    referenceType = 0 # upper and lower reference current setting mode, 0 - feedback; 1 - constant
    referSampleStartUd = 4 # upper and lower reference current sample start count (feedback), cyc
    referSampleCountUd = 1 # Upper and lower reference current sample cycle count (feedback), cyc
    referenceCurrent = 10 # upper and lower reference current mA

    startdescPose = [-583.168, 325.637, 1.176, 75.262, 0.978, -3.571]
    startjointPos = [-49.049, -77.203, 136.826, -189.074, -79.407, -11.811]
    enddescPose = [-559.439, 420.491, 32.252, 77.745, 1.460, -10.130]
    endjointPos = [-54.986, -77.639, 131.865, -185.707, -80.916, -12.218]

    error = robot.WeldingSetCurrent(1, 230, 0)
    print("WeldingSetCurrent return:",error)
    robot.WeldingSetVoltage(1, 24, 0)

    print("WeldingSetVoltage return:",error)
    robot.ArcWeldTraceExtAIChannelConfig(0)
    print("ArcWeldTraceExtAIChannelConfig return:",error)

    robot.MoveJ(startjointPos,13,0,desc_pos=startdescPose,vel =5)
    print("MoveJ return:",error)

    error = robot.ArcWeldTraceControl(flag,delaytime, isLeftRight, klr, tStartLr, stepMaxLr, sumMaxLr, isUpLow, kud, tStartUd, stepMaxUd,
                                sumMaxUd, axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent,0,0)
    print("WireSearchStart return:",error)

    robot.ARCStart(1, 0, 10000)
    print("ARCStart return:",error)

    robot.MoveL(enddescPose,13,0,joint_pos=endjointPos,vel =5)
    print("MoveJ return:",error)

    robot.ARCEnd(1, 0, 10000)
    print("ARCEnd return:",error)

    flag = 0
    error = robot.ArcWeldTraceControl(flag,delaytime, isLeftRight, klr, tStartLr, stepMaxLr, sumMaxLr, isUpLow, kud, tStartUd, stepMaxUd,
                                sumMaxUd, axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent,0,0)
    print("WireSearchStart return:",error)

Arc tracking AI passband selection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ArcWeldTraceExtAIChannelConfig(channel)``"
    "Description", "Arc Tracking AI Passband Selection"
    "Mandatory parameters", "- ``channel``: arc tracking AI passband selection, [0-3]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 

Simulated swing start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeaveStartSim(weaveNum)``"
    "Description", "Simulation of swing start"
    "Mandatory parameters", "- ``weaveNum``: swing parameter number"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 
    
Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    desc1 = [238.209, -403.633, 251.291, 177.222, -1.433, 133.675]
    joint1= [-48.728, -86.235, -95.288, -90.025, 92.715, 87.595]
    desc2 = [238.207, -596.305, 251.294, 177.223, -1.432, 133.675]
    joint2= [-60.240, -110.743, -66.784, -94.531, 92.351, 76.078 ]


    error = robot.MoveL(desc1,1,0,joint_pos=joint1)
    print("MoveL return:",error)

    error = robot.WeaveStartSim(0)
    print("WeaveStartSim return:",error)

    error = robot.MoveL(desc2,1,0,joint_pos=joint2)
    print("MoveL return:",error)

    error = robot.WeaveEndSim(0)
    print("WeaveEndSim return:",error)

End of simulation swing
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeaveEndSim(weaveNum)``"
    "description", "end of simulation swing"
    "Mandatory parameters", "- ``weaveNum``: swing parameter number"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 

Start trajectory detection warning (no movement)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeaveInspectStart(weaveNum)``"
    "Description", "Start Trajectory Detection Warning (No Movement)"
    "Mandatory parameters", "- ``weaveNum``: swing parameter number"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 
        
Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    desc1 = [238.209, -403.633, 251.291, 177.222, -1.433, 133.675]
    joint1= [-48.728, -86.235, -95.288, -90.025, 92.715, 87.595]
    desc2 = [238.207, -596.305, 251.294, 177.223, -1.432, 133.675]
    joint2= [-60.240, -110.743, -66.784, -94.531, 92.351, 76.078 ]

    error = robot.MoveL(desc1,1,0,joint_pos=joint1)
    print("MoveL return:",error)

    error = robot.WeaveInspectStart(0)
    print("WeaveInspectStart return:",error)

    error = robot.MoveL(desc2,1,0,joint_pos=joint2)
    print("MoveL return:",error)

    error = robot.WeaveInspectEnd(0)
    print("WeaveInspectEnd return:",error)
    
End trajectory detection warning (no movement)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeaveInspectEnd(weaveNum)``"
    "Description", "End Trajectory Detection Warning (No Movement)"
    "Mandatory parameters", "- ``weaveNum``: swing parameter number"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 
    
Setting Welding Process Curve Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingSetProcessParam(id, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime)``"
    "Description", "Set weld process profile parameters"
    "Mandatory parameters", "
    - ``id``: Welding process number (1-99)
    - ``startCurrent``: Arc starting current (A)
    - ``startVoltage``: startVoltage Arc-starting voltage (V)
    - ``startTime``: startTime Arc start time (ms)
    - ``weldCurrent``: weldCurrent Welding current (A)
    - ``weldVoltage``: weldVoltage Welding voltage (V)
    - ``endCurrent``: endCurrent Arc recovery current (A)
    - ``endVoltage``: endVoltage Arc charging voltage (V)
    - ``endTime``: endTime closing time (ms)
    "
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 
            
Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    id = 1 # Welding process number (1-99)
    startCurrent = 177 # Arc start current (A)
    startVoltage = 27 # Arc Start Voltage (V)
    startTime = 1000 # Arc start time (ms)
    weldCurrent = 178 #Weld Current (A)
    weldVoltage = 28 # Weld Voltage (V)
    endCurrent = 176 # Arc closing current (A)
    endVoltage = 26 # Arc charging voltage (V)
    endTime = 1000 #Arc closing time (ms)

    error = robot.WeldingSetProcessParam(id, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage,
                                            endCurrent, endVoltage, endTime)

    print("WeldingSetProcessParam return:",error)

    error = robot.WeldingGetProcessParam(1)
    print("WeldingGetProcessParam return:",error)
        
Obtaining Welding Process Curve Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingGetProcessParam(id)``"
    "Description", "Get welding process curve parameters"
    "Mandatory parameters", "
    - ``id``: Welding process number (1-99)
    "
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``startCurrent``: arc starting current (A)
    - ``startVoltage``: Arc starting voltage (V)
    - ``startTime``: start time (ms)
    - ``weldCurrent``: welding current (A)
    - ``weldVoltage``: welding voltage (V)
    - ``endCurrent``: arc closing current (A)
    - ``endVoltage``: arc closing voltage (V)
    - ``endTime``: arc closing time (ms)
    " 
    
Extended IO-Configuration Welder Gas Detection Signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAirControlExtDoNum(DONum)``"
    "Description", "Extended IO-Configuration Welder Gas Detection Signal"
    "Mandatory parameters", "
    - ``DONum``: gas detection signal extension DO number
    "
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 
            
Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    #Expanded IO-Configuration Welder Gas Detection Signal
    error = robot.SetAirControlExtDoNum(10)
    print("SetAirControlExtDoNum 10 return:",error)

    # Expanded IO-Configure welder arc start signals
    error = robot.SetArcStartExtDoNum(11)
    print("SetArcStartExtDoNum 11 return:",error)

    #Expansion IO-Configuration of the welder's reverse wire feed signal
    error = robot.SetWireReverseFeedExtDoNum(12)
    print("SetWireReverseFeedExtDoNum 12 return:",error)

    #Expansion IO-Configuration of the welder's forward wire feed signal
    error = robot.SetWireForwardFeedExtDoNum(13)
    print("SetWireForwardFeedExtDoNum 13 return:",error)

    #Expanded IO-Configure Welder Arc Start Success Signal
    error = robot.SetArcDoneExtDiNum(10)
    print("SetArcDoneExtDiNum 10 return:",error)

    # Extended IO-Configuration Welder Ready Signal
    error = robot.SetWeldReadyExtDiNum(11)
    print("SetWeldReadyExtDiNum 11 return:",error)

    # Expanded IO-Configuration Weld Interrupt Recovery Signals
    error = robot.SetExtDIWeldBreakOffRecover(12,13)
    print("SetExtDIWeldBreakOffRecover 12 13 return:",error)
        
Extended IO-Configuration of welder arc start signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetArcStartExtDoNum(DONum)``"
    "Description", "Extended IO-Configuration Welder Arc Start Signal"
    "Mandatory parameters", "
    - ``DONum``: gas detection signal extension DO number
    "
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 
        
Extended IO-Configuration of the welder's reverse wire feed signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWireReverseFeedExtDoNum(DONum)``"
    "Description", "Extended IO-Configuration Welder Reverse Wire Feed Signal"
    "Mandatory parameters", "
    - ``DONum``: gas detection signal extension DO number
    "
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 
        
Extended IO-Configuration of the welder's forward wire feed signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWireForwardFeedExtDoNum(DONum)``"
    "Description", "Extended IO-Configure welder positive wire feed signal"
    "Mandatory parameters", "
    - ``DONum``: gas detection signal extension DO number
    "
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 
        
Extended IO-Configuration of the welder's arc start success signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetArcDoneExtDiNum(DINum)``"
    "Description", "Extended IO-Configuration Welder Arc Start Success Signal"
    "Mandatory parameters", "
    - ``DINum``: Welder Ready Signal Expansion DI Number
    "
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 
        
Extended IO-Configuration Welder Ready Signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetArcDoneExtDiNum(DINum)``"
    "Description", "Extended IO-Configuration Welder Ready Signal"
    "Mandatory parameters", "
    - ``DINum``: Welder Ready Signal Expansion DI Number
    "
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 
        
Extended IO-Configuration Weld Interrupt Recovery Signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExtDIWeldBreakOffRecover(reWeldDINum, abortWeldDINum)``"
    "description", "Extended IO-Configuration Weld Interrupt Recovery Signal"
    "Mandatory parameters", "
    - ``reWeldDINum``: Weld signal extension DI number for resumption of welding after a weld interruption
    - ``abortWeldDINum``: exit weld signal extension DI number after weld interruption
    "
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 

Setting Up the Weld Wire Seek Expansion IO Port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWireSearchExtDIONum(searchDoneDINum, searchStartDONum)``"
    "Description", "Setting up the Weld Wire Seeker Expansion IO Port"
    "Mandatory parameter", "- ``searchDoneDINum``: weld wire seek success DO port (0-127)
    - ``searchStartDONum``: wire seek start/stop control DO port (0-127)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 

Welder control mode switching
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

Set welder control mode to expand DO port
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWeldMachineCtrlModeExtDoNum(DONum)``"
    "Description", "Set welder control mode to extend DO port"
    "Mandatory parameters", "- ``DONum``: welder control mode DO port (0-127)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 

Setting the welder control mode
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWeldMachineCtrlMode(mode)``"
    "Description", "Set welder control mode"
    "Mandatory parameters", "- ``mode``: welder control mode; 0 - unitary"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 

Code example
----------------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10)
    print("ExtDevSetUDPComParam return ", error)
    error = robot.ExtDevLoadUDPDriver()
    print("ExtDevLoadUDPDriver return ", error)

    robot.SetWeldMachineCtrlModeExtDoNum(DONum=17)
    robot.SetWeldMachineCtrlMode(mode=0)
    robot.SetWeldMachineCtrlModeExtDoNum(DONum=18)
    robot.SetWeldMachineCtrlMode(mode=0)
    robot.SetWeldMachineCtrlModeExtDoNum(DONum=19)
    robot.SetWeldMachineCtrlMode(mode=0)

    error = robot.SetWeldMachineCtrlModeExtDoNum(DONum=17)
    print("SetWeldMachineCtrlModeExtDoNum return ", error)
    for i in range(0,5):
        error = robot.SetWeldMachineCtrlMode(mode=0)
        print("SetWeldMachineCtrlMode return ", error)
        time.sleep(0.5)
        error = robot.SetWeldMachineCtrlMode(mode=1)
        print("SetWeldMachineCtrlMode return ", error)
        time.sleep(0.5)

    error = robot.SetWeldMachineCtrlModeExtDoNum(DONum=18)
    print("SetWeldMachineCtrlModeExtDoNum return ", error)
    for i in range(0,5):
        error = robot.SetWeldMachineCtrlMode(mode=0)
        print("SetWeldMachineCtrlMode return ", error)
        time.sleep(0.5)
        error = robot.SetWeldMachineCtrlMode(mode=1)
        print("SetWeldMachineCtrlMode return ", error)
        time.sleep(0.5)

    error = robot.SetWeldMachineCtrlModeExtDoNum(DONum=19)
    print("SetWeldMachineCtrlModeExtDoNum return ", error)
    for i in range(0,5):
        error = robot.SetWeldMachineCtrlMode(mode=0)
        print("SetWeldMachineCtrlMode return ", error)
        time.sleep(0.5)
        error = robot.SetWeldMachineCtrlMode(mode=1)
        print("SetWeldMachineCtrlMode return ", error)
        time.sleep(0.5)

Arc tracking + multi-layer multi-channel compensation on
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``ArcWeldTraceReplayStart()``"
    "Description", "Arc Tracking + Multi-Layer Multi-Channel Compensation On"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Arc Tracking + Multi-Layer Multi-Channel Compensation Off
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ArcWeldTraceReplayEnd()``"
    "Description", "Arc Tracking + Multi-Layer Multi-Channel Compensation Shutdown"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Offset Coordinate Change - Multi-layer Multi-pass Welding
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Python SDK-v2.0.8-3.7.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MultilayerOffsetTrsfToBase(pointo, pointX, pointZ, dx, dy, db)``"
    "Description", "Offset Coordinate Change - Multi-Layer Multi-Pass Welding"
    "Mandatory parameters", "- ``pointo``: datum Cartesian orientation
    - ``pointX``: Cartesian position of point X in the offset direction from the reference point X.
    - ``pointZ``: Cartesian position of point Z in the direction of offset from datum Z.
    - ``dx``: x-direction offset (mm)
    - ``dz``: z-direction offset (mm)
    - ``dry``: offset around the y-axis (°)"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode 
    - ``offset``: the offset of the result of the calculation"

Set the detection parameters of robot welding arc unexpected interruption
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Python SDK-v2.0.8-3.7.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingSetCheckArcInterruptionParam(checkEnable, arcInterruptTimeLength)``"
    "Description", "Set the detection parameters of robot welding arc unexpected interruption"
    "Mandatory parameters", "- ``checkEnable``: Whether to enable detection. 0: Indicates that the function is disabled. 1- Enable
    - ``arcInterruptTimeLength``:Arc break confirmation time (ms)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

The detection parameters of unexpected interruption of robot welding arc were obtained
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.8-3.7.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingGetCheckArcInterruptionParam()``"
    "Description", "The detection parameters of unexpected interruption of robot welding arc were obtained"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode 
    - ``checkEnable``: Whether to enable detection. 0: Indicates that the function is disabled. 1- Enable
    - ``arcInterruptTimeLength``:Arc break confirmation time (ms)"

Set the robot welding interrupt recovery parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.8-3.7.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingSetReWeldAfterBreakOffParam(enable, length, velocity, moveType)``"
    "Description", "Set the robot welding interrupt recovery parameters"
    "Mandatory parameters", "- ``enable``:Whether to enable recovery from welding interruption
    - ``length``: weld overlap distance (mm)
    - ``velocity``: Percentage of robot return to rearcing point (0-100)
    - ``moveType``: The way the robot moves to the rearcing point; 0-LIN; 1-PTP"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Obtain the robot welding interrupt recovery parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.8-3.7.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingGetReWeldAfterBreakOffParam()``"
    "Description", "Obtain the robot welding interrupt recovery parameters"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode 
    - ``enable``: Whether to enable recovery from welding interruption
    - ``length``: weld overlap distance (mm)
    - ``velocity``: Percentage of robot return to rearcing point (0-100)
    - ``moveType``: The way the robot moves to the rearcing point; 0-LIN; 1-PTP"

Set the robot to resume welding after welding interruption
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.8-3.7.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingStartReWeldAfterBreakOff()``"
    "Description", "Set the robot to resume welding after welding interruption"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Set the robot to exit welding after welding interruption
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.8-3.7.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingAbortWeldAfterBreakOff()``"
    "Description", "Set the robot to exit welding after welding interruption"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    rtn = -1
    rtn = robot.WeldingSetCheckArcInterruptionParam(1, 200)
    print("WeldingSetCheckArcInterruptionParam return", rtn)
    rtn = robot.WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0)
    print("WeldingSetReWeldAfterBreakOffParam return", rtn)
    enable = 0
    length = 0
    velocity = 0
    moveType = 0
    checkEnable = 0
    arcInterruptTimeLength = 0
    rtn, checkEnable, arcInterruptTimeLength = robot.WeldingGetCheckArcInterruptionParam()
    print("WeldingGetCheckArcInterruptionParam checkEnable:", checkEnable)
    print("WeldingGetCheckArcInterruptionParam arcInterruptTimeLength:", arcInterruptTimeLength)
    rtn, enable, length, velocity, moveType = robot.WeldingGetReWeldAfterBreakOffParam()
    print("*****")
    print("WeldingGetReWeldAfterBreakOffParam enable:", enable)
    print("WeldingGetReWeldAfterBreakOffParam length:", length)
    print("WeldingGetReWeldAfterBreakOffParam velocity:", velocity)
    print("WeldingGetReWeldAfterBreakOffParam moveType:", moveType)

    robot.ProgramLoad("/fruser/test.lua")
    robot.ProgramRun()

    time.sleep(5)

    while True:
        print("welding breakoff state is ", robot.robot_state_pkg.weldingBreakOffState.breakOffState)
        if robot.robot_state_pkg.weldingBreakOffState.breakOffState == 1:
            print("welding breakoff !")
            time.sleep(2)
            rtn = robot.WeldingStartReWeldAfterBreakOff()
            print("WeldingStartReWeldAfterBreakOff return", rtn)
            break
        time.sleep(0.1)

Wobble start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeaveChangeStart(weaveNum)``"
    "Description", "Wobble start"
    "Mandatory parameters", "- ``weaveChangeFlag``: Swing number 1- Variable swing parameters; 2- Variable swing parameters + welding speed
    - ``weaveNum``: Swing number
    - ``velStart``: Welding start speed, (cm/min)
    - ``velEnd``: Welding end speed, (cm/min)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Wobble end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.9-3.8.0

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeaveChangeEnd()``"
    "Description", "Wobble end"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
----------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    p1Joint = [74.620, -80.903, 94.608, -109.882, -90.436, -13.432]
    p1Desc = [-72.912, -587.664, 31.849, 43.283, -6.731, 15.068]
    p2Joint = [66.431, -92.875, 116.362, -120.516, -88.627, -24.731]
    p2Desc = [-104.915, -483.712, -25.231, 42.228, -6.572, 18.433]
    p3Joint = [56.457, -84.796, 104.618, -114.497, -92.422, -25.430]
    p3Desc = [-240.651, -483.840, -7.161, 46.577, -5.286, 8.318]
    
    robot.WeldingSetVoltage(1, 19, 0, 0)
    robot.WeldingSetCurrent(1, 190, 0, 0)
    robot.MoveJ(joint_pos=p1Joint, tool=1, user=1, vel=100.0, acc=100.0, ovl=100.0)
    robot.MoveL(desc_pos=p2Desc, tool=1, user=1, vel=100.0, acc=100.0, ovl=50.0)
    robot.ARCStart(1, 0, 10000)
    robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0)
    robot.WeaveStart(0)
    robot.WeaveChangeStart(1,1,0,0)
    robot.MoveL(desc_pos=p3Desc, tool=1, user=1, vel=100.0, acc=100.0, ovl=1.0)
    robot.WeaveChangeEnd()
    robot.WeaveEnd(0)
    robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0)
    robot.ARCEnd(1, 0, 10000)

Selection of AI channels for current feedback in arc tracking welding machines
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ArcWeldTraceAIChannelCurrent(channel)``"
    "Description", "Selection of AI channels for current feedback in arc tracking welding machines"
    "Mandatory parameters", "- ``channel``: Passage 0- Expand AI0; 1- Expand AI1; 2- Expand AI2; 3- Expand AI3; 4- Control Box AI0 5- Control Box AI1"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Selection of AI channel for voltage feedback of arc tracking welding machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ArcWeldTraceAIChannelVoltage(channel)``"
    "Description", "Selection of AI channel for voltage feedback of arc tracking welding machine"
    "Mandatory parameters", "- ``channel``: Passage 0- Expand AI0; 1- Expand AI1; 2- Expand AI2; 3- Expand AI3; 4- Control Box AI0 5- Control Box AI1"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Current feedback conversion parameters of arc tracking welding machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ArcWeldTraceCurrentPara(AILow, AIHigh, currentLow, currentHigh)``"
    "Description", "Current feedback conversion parameters of arc tracking welding machine"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``AILow``: Lower limit of AI channel, default value 0V, range [0-10V]
    - ``AIHigh``: AI channel upper limit, default value 10V, range [0-10V]
    - ``currentLow``: The lower limit of the AI channel corresponds to the current value of the welding machine. The default value is 0V, and the range is [0-200V]
    - ``currentHigh``: The upper limit of the AI channel corresponds to the current value of the welding machine. The default value is 100V, and the range is [0-200V]"
    "Return Value", "Error Code Success-0 Failure- errcode "

Voltage feedback conversion parameters of arc tracking welding machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ArcWeldTraceVoltagePara(AILow, AIHigh, voltageLow, voltageHigh)``"
    "Description", "Voltage feedback conversion parameters of arc tracking welding machine"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``AILow``: Lower limit of AI channel, default value 0V, range [0-10V]
    - ``AIHigh``: AI channel upper limit, default value 10V, range [0-10V]
    - ``voltageLow``: The lower limit of the AI channel corresponds to the welding machine voltage value. The default value is 0V, and the range is [0-200V]
    - ``voltageHigh``: The upper limit of the AI channel corresponds to the voltage value of the welding machine. The default value is 100V, and the range is [0-200V]"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    safetydescPose = [-504.043,275.181,40.908,-28.002,-42.025,-14.044]
    safetyjointPos = [-39.078,-76.732,87.227,-99.47,-94.301,18.714]
    startdescPose = [-473.86,257.879,-20.849,-37.317,-42.021,2.543]
    startjointPos = [-43.487,-76.526,95.568,-104.445,-89.356,3.72]
    enddescPose = [-499.844,141.225,7.72,-34.856,-40.17,13.13]
    endjointPos = [-31.305,-82.998,99.401,-104.426,-89.35,3.696]
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    robot.MoveJ(joint_pos=safetyjointPos, tool=1, user=0, vel=20, acc=100)
    robot.WeldingSetCurrentRelation(0, 495, 1, 10, 0)
    robot.WeldingSetVoltageRelation(10, 45, 1, 10, 1)
    robot.WeldingSetVoltage(0, 25, 1, 0)
    robot.WeldingSetCurrent(0, 260, 0, 0)
    rtn = robot.ArcWeldTraceAIChannelCurrent(4)
    print("ArcWeldTraceAIChannelCurrent rtn is", rtn)
    rtn = robot.ArcWeldTraceAIChannelVoltage(5)
    print("ArcWeldTraceAIChannelVoltage rtn is", rtn)
    rtn = robot.ArcWeldTraceCurrentPara(0, 5, 0, 500)
    print("ArcWeldTraceCurrentPara rtn is", rtn)
    rtn = robot.ArcWeldTraceVoltagePara(1.018, 10, 0, 50)
    print("ArcWeldTraceVoltagePara rtn is", rtn)
    robot.MoveJ(joint_pos=startjointPos, tool=1, user=0, vel=20, acc=100)
    robot.ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0)
    robot.ARCStart(0, 0, 10000)
    robot.WeaveStart(0)
    robot.MoveL(desc_pos=enddescPose, tool=1, user=0, vel=100, ovl= 2, acc=100)
    robot.ARCEnd(0, 0, 10000)
    robot.WeaveEnd(0)
    robot.ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0)
    robot.MoveJ(joint_pos=safetyjointPos, tool=1, user=0, vel=20, acc=100)

Set the welding voltage to start gradually
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingSetVoltageGradualChangeStart(IOType, voltageStart, voltageEnd, AOIndex, blend)``"
    "Description", "Set the welding voltage to start gradually"
    "Mandatory parameters", "- ``IOType``: Control type 0- Control Box IO 1- Digital Communication Protocol (UDP) 2- Digital Communication Protocol (ModbusTCP)
    - ``voltageStart``: Initial welding voltage (V)
    - ``voltageEnd``: Termination welding voltage (V)
    - ``AOIndex``: The AO port number of the control box (0-1)
    - ``blend``: Whether smooth 0- Not smooth; 1- Smooth"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Set the welding voltage gradient to end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingSetVoltageGradualChangeEnd()``"
    "Description", "Set the welding voltage gradient to end"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Set the welding current to start gradually
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingSetCurrentGradualChangeStart(IOType, currentStart, currentEnd, AOIndex, blend)``"
    "Description", "Set the welding current to start gradually"
    "Mandatory parameters", "- ``IOType``: Control type 0- Control Box IO 1- Digital Communication Protocol (UDP) 2- Digital Communication Protocol (ModbusTCP)
    - ``currentStart``: Initial welding current (A)
    - ``currentEnd``: Termination welding current (A)
    - ``AOIndex``: The AO port number of the control box (0-1)
    - ``blend``: Whether smooth 0- Not smooth; 1- Smooth"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Set the welding current to gradually end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingSetCurrentGradualChangeEnd()``"
    "Description", "Set the welding current to gradually end"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    startdescPose = [-484.707, 276.996, -14.013, -37.657, -40.508, -1.548]
    startjointPos = [-45.421, -75.673, 93.627, -104.302, -87.938, 6.005]
    enddescPose = [-508.767, 137.109, -13.966, -37.639, -40.508, -1.559]
    endjointPos = [-32.768, -80.947, 100.254, -106.201, -87.201, 18.648]
    safedescPose = [-484.709, 294.436, 13.621, -37.660, -40.508, -1.545]
    safejointPos = [-46.604, -75.410, 89.109, -100.003, -88.012, 4.823]
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    robot.WeldingSetCurrentRelation(0, 495, 1, 10, 0)
    robot.WeldingSetVoltageRelation(10, 45, 1, 10, 1)
    robot.WeldingSetVoltage(0, 25, 1, 0)
    robot.WeldingSetCurrent(0, 260, 0, 0)
    robot.MoveJ(joint_pos=safejointPos, tool=1, user=0, vel=5, acc=100)
    rtn = robot.WeldingSetCurrentGradualChangeStart(0, 260, 220, 0, 0)
    print("WeldingSetCurrentGradualChangeStart rtn is", rtn)
    rtn = robot.WeldingSetVoltageGradualChangeStart(0, 25, 22, 1, 0)
    print("WeldingSetVoltageGradualChangeStart rtn is", rtn)
    rtn = robot.ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0)
    print("ArcWeldTraceControl rtn is", rtn)
    robot.MoveJ(joint_pos=startjointPos, tool=1, user=0, vel=5, acc=100)
    robot.ARCStart(0, 0, 10000)
    robot.WeaveStart(0)
    robot.WeaveChangeStart(2, 1, 24, 36)
    robot.MoveL(desc_pos=enddescPose, tool=1, user=0, vel=100, ovl=2, acc=100)
    robot.ARCEnd(0, 0, 10000)
    robot.WeaveChangeEnd()
    robot.WeaveEnd(0)
    robot.ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0)
    robot.WeldingSetCurrentGradualChangeEnd()
    robot.WeldingSetVoltageGradualChangeEnd()