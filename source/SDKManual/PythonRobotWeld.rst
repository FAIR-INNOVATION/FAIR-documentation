Weld
======================

.. toctree:: 
    :maxdepth: 5

Welding Start
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "ARCStart(ioType, arcNum, timeout)"
    "Description", "Welding Start"
    "Required parameter", "- ``ioType``: 0-control box IO； 1-extend IO
    - ``arcNum``: Welder configuration file number
    - ``timeout``: Arcing timeout time"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed- errcode"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')

    weldIOType =0
    arcNum =0
    weldTimeout=5000
    # Welding Start
    ret = robot.ARCStart(weldIOType,arcNum,weldTimeout)
    print("ARCStart ", ret)
    time.sleep(3)

Welding end
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "ARCEnd(ioType, arcNum, timeout)"
    "Description", "Welding end"
    "Required parameter", "- ``ioType``: 0-control box IO； 1-extend IO
    - ``arcNum``: Welder configuration file number
    - ``timeout``: Arcing timeout time"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed- errcode"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')

    weldIOType =0
    arcNum =0
    weldTimeout=5000
    #Welding end
    ret = robot.ARCEnd(weldIOType,arcNum,weldTimeout)
    print("ARCEnd ", ret)
    time.sleep(3)

Set the relationship between welding current and output analog
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingSetCurrentRelation(currentMin, currentMax, outputVoltageMin, outputVoltageMax)"
    "Description", "Set the relationship between welding current and output analog"
    "Required parameter", "- ``currentMin``: Welding current-analog output linear relationship left point current value(A)
    - ``currentMax``:  Welding current-analog output linear relationship right point current value(A)
    - ``outputVoltageMin``: Welding current-analog output linear relationship left point analog output voltage value(V)
    - ``outputVoltageMax``:Welding current-analog output linear relationship right point analog output voltage value(V)
    - ``AOIndex``:Welding current analog output port
    "
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed- errcode"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establishes a connection with the robot controller and returns a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    weldIOType =0
    arcNum =0
    weldTimeout=5000

    #Set the welding current linearly to the analog
    ret = robot.WeldingSetCurrentRelation(0,400,0,10,0)
    print("WeldingSetCurrentRelation", ret)
    time.sleep(1)
    #Get a linear relationship between welding current and analog
    ret = robot.WeldingGetCurrentRelation()
    print("WeldingGetCurrentRelation", ret)
    time.sleep(1)

    #Set the welding voltage linearly to the analog
    ret = robot.WeldingSetVoltageRelation(0,400,0,10,0)
    print("WeldingSetVoltageRelation", ret)
    time.sleep(1)
    #Get a linear relationship between weld voltage and analog
    ret = robot.WeldingGetVoltageRelation()
    print("WeldingGetVoltageRelation", ret)
    time.sleep(1)

Set the relationship between welding voltage and output analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingSetVoltageRelation(weldVoltageMin, weldVoltageMax, outputVoltageMin, outputVoltageMax)"
    "Description", "Set the relationship between welding voltage and output analog"
    "Required parameter", "- ``weldVoltageMin``: Welding voltage - analog output linear relationship left spot welding voltage value(V)
    - ``weldVoltageMax``: Welding voltage - analog output linear relationship right spot welding voltage value(V)
    - ``outputVoltageMin``: Welding voltage-analog output linear relationship left point analog output voltage value(V)
    - ``outputVoltageMax``:Welding voltage-analog output linear relationship right point analog output voltage value(V)
    - ``AOIndex``:Welding current analog output port"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed- errcode"

Obtain the corresponding relationship between welding current and output analog
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingGetCurrentRelation()"
    "Description", "Obtain the corresponding relationship between welding current and output analog"
    "Required parameter", "Null"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed- errcode
    - Return:(if success)currentMin,currentMax,outputVoltageMin,outputVoltageMax
    - ``currentMin``:Welding current-analog output linear relationship left point current value(A)
    - ``currentMax``:Welding current-analog output linear relationship right point current value(A)
    - ``outputVoltageMin``:Welding current-analog output linear relationship left point analog output voltage value(V)
    - ``outputVoltageMax``:Welding current-analog output linear relationship right point analog output voltage value(V)
    - ``AOIndex``:Welding current analog output port"

Obtain the corresponding relationship between welding voltage and output analog
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingGetVoltageRelation()"
    "Description", "Obtain  the corresponding relationship between welding voltage and output analog"
    "Required parameter", "Null"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed- errcode
    - Return:（if success）weldVoltageMin,weldVoltageMax,outputVoltageMin, outputVoltageMax, weldVoltageMin 
    - ``weldVoltageMin``:Welding voltage - analog output linear relationship left spot welding voltage value(V)
    - ``weldVoltageMax``:Welding voltage - analog output linear relationship right spot welding voltage value(V)
    - ``outputVoltageMin``:Welding voltage-analog output linear relationship left point analog output voltage value(V)
    - ``outputVoltageMax``:Welding voltage-analog output linear relationship right point analog output voltage value(V)
    - ``AOIndex``:Welding current analog output port"

Set welding current
++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingSetCurrent(ioType,current, AOIndex)"
    "Description", "Set welding current"
    "Required parameter", "- ``ioType``: 0-control box IO； 1-extend IO
    - ``current``: current welding current(A)
    - ``AOIndex``: Welding current control box analog output port(0-1)"
    "Optional parameter", "- ``blend``: Smooth or not 0 - not smooth, 1 - smooth, default 0"
    "Return value", "- Errcode: Success -0 , Failed- errcode"

Set welding voltage
++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingSetVoltage(ioType,voltage, AOIndex)"
    "Description", "Set welding voltage"
    "Required parameter", "- ``ioType``: 0-control box IO； 1-extend IO 
    - ``voltage``: welding voltage(V)
    - ``AOIndex``: Welding voltage control box analog output port(0-1)"
    "Optional parameter", "- ``blend``: Smooth or not 0 - not smooth, 1 - smooth, default 0"
    "Return value", "- Errcode: Success -0 , Failed- errcode"

Set weave parameters
++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeaveSetPara(weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange, weaveLeftRange, weaveRightRange, additionalStayTime, weaveLeftStayTime, weaveRightStayTime, weaveCircleRadio, weaveStationary, weaveYawAngle=0)"
    "Description", "Set weave parameters"
    "Required parameter", "- ``weaveNum``: parameters number
    - ``weaveType``: weave type:0- plane triangular weave ; 1- vertical L-shaped triangular weave; 2- clockwise circular weave; 3-counterclockwise circular weave; 4-plane sine weave; 5-vertical L-shaped sine weave; 6- vertical triangular weave; 7- Vertical sine weave
    - ``weaveFrequency``: weave frequency(Hz)
    - ``weaveIncStayTime``: Wait mode 0- period does not contain wait time; 1- Period contains the wait time
    - ``weaveRange``: weave amplitude(mm)
    - ``weaveLeftStayTime``: weave left residence time(ms)
    - ``weaveRightStayTime``: weave right residence time(ms)
    - ``weaveCircleRadio``: Circular wiggle-pullback ratio(0-100%)
    - ``weaveStationary``: weave position wait, 0- position continue to move within the waiting time; 1- The position is stationary during the waiting time
    - ``weaveLeftRange``:weave amplitude(mm)
    - ``weaveRightRange``:weave amplitude(mm)
    - ``additionalStayTime``:weave right residence time(ms)
    "
    "Optional parameter", "- ``weaveYawAngle``:Swing direction azimuth (rotation around swing Z-axis), in °, default 0"
    "Return value", "- Errcode: Success -0 , Failed- errcode"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    weaveNum =0
    weaveType = 0
    weaveFraquency = 1
    weavelncStayTime = 0
    weaveRange = 10
    weaveLeftStayTime = 10
    weaveRightStayTime = 10
    weaveCircleRadio =0
    weaveStationary =1
    # Set weave parameters
    ret = robot.WeaveSetPara(weaveNum,weaveType,weaveFraquency,weavelncStayTime,weaveRange,weaveLeftStayTime,weaveRightStayTime,weaveCircleRadio,weaveStationary)
    print("WeaveSetPara ", ret)
    time.sleep(1)

    # Weave start
    ret = robot.WeaveStart(0)
    print("WeaveStart ", ret)
    time.sleep(1)
    ret,pose =robot.GetActualTCPPose(1);
    print(ret,pose)
    pose[2]=pose[2]+50
    ret = robot.MoveL(pose,tool,user)
    print("MoveL ", ret)
    time.sleep(1)
    #Set weave parameters in real time
    ret = robot.WeaveOnlineSetPara (weaveNum,weaveType,weaveFraquency,weavelncStayTime,weaveRange,weaveLeftStayTime,weaveRightStayTime,weaveCircleRadio,weaveStationary)
    print("WeaveOnlineSetPara ", ret)
    time.sleep(1)

    # Weave end
    ret = robot.WeaveEnd(0)
    print("WeaveEnd ", ret)
    time.sleep(1)

Set weave parameters in real time
++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeaveOnlineSetPara(weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange, weaveLeftStayTime, weaveRightStayTime, weaveCircleRadio, weaveStationary)"
    "Description", "Set weave parameters in real time"
    "Required parameter", "- ``weaveNum``: parameters number
    - ``weaveType``: weave type:0- plane triangular weave ; 1- vertical L-shaped triangular weave; 2- clockwise circular weave; 3-counterclockwise circular weave; 4-plane sine weave; 5-vertical L-shaped sine weave; 6- vertical triangular weave; 7- Vertical sine weave
    - ``weaveFrequency``: weave frequency(Hz)
    - ``weaveIncStayTime``: Wait mode 0- period does not contain wait time; 1- Period contains the wait time
    - ``weaveRange``: weave amplitude(mm)
    - ``weaveLeftStayTime``: weave left residence time(ms)
    - ``weaveRightStayTime``:  weave right residence time(ms)
    - ``weaveCircleRadio``: Circular wiggle-pullback ratio(0-100%)
    - ``weaveStationary``: weave position wait, 0- position continue to move within the waiting time; 1- The position is stationary during the waiting time"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed- errcode"

Weave start
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeaveStart(weaveNum)"
    "Description", "Weave start"
    "Required parameter", "- ``weaveNum``: Weave welding parameter configuration number"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed- errcode"

Weave end
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeaveEnd(weaveNum)"
    "Description", "Weave end"
    "Required parameter", "- ``weaveNum``: Weave welding parameter configuration number"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed- errcode"

Forward wire feed
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetForwardWireFeed(ioType, wireFeed)"
    "Description", "Forward wire feed"
    "Required parameter", "- ``ioType``: 0-control box IO； 1-extend IO
    - ``wireFeed``: wire control: 0-stop wire feed ；1-wire feed"
    "Return value", "- Errcode: Success -0 , Failed- errcode"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')

    weldIOType =0
    # Forward wire feed
    ret = robot.SetForwardWireFeed(weldIOType,1)
    print("SetForwardWireFeed ", ret)
    time.sleep(1)
    ret = robot.SetForwardWireFeed(weldIOType,0)
    print("SetForwardWireFeed ", ret)
    time.sleep(1)

    # Reverse wire feed
    ret = robot.SetReverseWireFeed(weldIOType,1)
    print("SetReverseWireFeed ", ret)
    time.sleep(1)
    ret = robot.SetReverseWireFeed(weldIOType,0)
    print("SetReverseWireFeed ", ret)
    time.sleep(1)

    # Aspirated
    ret = robot.SetAspirated(weldIOType,1)
    print("SetAspirated ", ret)
    time.sleep(1) ret = robot.SetAspirated(weldIOType,0)
    print("SetAspirated ", ret)
    time.sleep(1)

Reverse wire feed
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetReverseWireFeed(ioType, wireFeed)"
    "Description", "Reverse wire feed"
    "Required parameter", "- ``ioType``: 0-control box IO； 1-extend IO
    - ``wireFeed``: wire control  0-stop wire feed ；1-wire feed"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed- errcode"

Aspirated
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetAspirated(ioType, airControl)"
    "Description", "Aspirated"
    "Required parameter", "- ``ioType``: 0-control box IO； 1-extend IO
    - ``airControl``: 0-stop aspirated；1-aspirated"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed- errcode"

Segment weld start
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SegmentWeldStart(startDesePos,  endDesePos, startJPos, endJPos, weldLength, noWeldLength, weldIOType, arcNum, weldTimeout, isWeave,weaveNum,tool,user,vel=20.0, acc=0.0, ovl=100.0, blendR=-1.0,exaxis_pos=[0.0, 0.0, 0.0, 0.0],  search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])"
    "Description", "Segment weld start"
    "Required parameter", "- ``startDesePos``: Starting point Cartesian position
    - ``endDesePos``: Ending point Cartesian position
    - ``startJPos``: Starting point joint position
    - ``endJPos``:Ending point joint position 
    - ``weldLength``:Weld length(mm)
    - ``noWeldLength``:Length of unwelded section(mm)  
    - ``weldIOType``:0-control box IO； 1-extend IO
    - ``timeout``:Arcing timeout time
    - ``isWeave``:True-Weave False- Not weave
    - ``weaveNum``:Weave welding parameter configuration number Required parameter
    - ``tool``:Tool coordinate number, range[0~14]
    - ``user``:Workpiece coordinate number, range,[0~14]"
    "Optional parameter", "- ``vel``:Percentage of speed, range [0~100],[0~100] , Default 20.0
    - ``acc``:Acceleration percentage, range [0~100] ,not open for now , Default 0.0
    - ``ovl``:Velocity scaling factor, range [0~100] , Default 100.0 [-1.0]- movement in place (blocking), [0~1000.0] Smoothing radius (non-blocking), unit: mm, Default -1.0
    - ``blendR``:[-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm, Default -1.0
    - ``exaxis_pos``:Position of expansion shaft, unit: m , Default [0.0,0.0,0.0,0.0]
    - ``search``:0-no wire seeking, 1-wire seeking Default 0
    - ``offset_flag``:0-no offset, 1-offset in base/job coordinate system, 2- offset in tool coordinate system Default 0
    - ``offset_pos``:The pose offset Default [0.0,0.0,0.0,0.0,0.0,0.0]"
    "Return value", "- Errcode: Success -0 , Failed -errcode"

Code example
-------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')

    weldIOType =0
    arcNum =0
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
    weaveCircleRadio =0
    weaveStationary =1
    start_desc=[0,0,0,0,0,0]
    end_desc=[0,0,0,0,0,0]
    start_joint=[0,0,0,0,0,0]
    end_joint=[0,0,0,0,0,0]
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
    #Segment weld start

    ret = robot.SegmentWeldStart(start_desc,end_desc,start_joint,end_joint,weldLength,noweldLength,weldIOType,arcNum,weldTimeout,True,weaveNum,tool,user)
    print("SegmentWeldStart", ret)

Welding wire positioning start
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WireSearchStart(refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag) "
    "Description", "Welding wire positioning start"
    "Required parameter", "- ``refPos``： 1-datum point 2-reference point
    - ``searchVel``： positioning speed %
    - ``searchDis``： positioning distance mm
    - ``autoBackFlag``： automatic return flag，0-not automatic；1-automatic 
    - ``autoBackVel``： automatic return speed %
    - ``autoBackDis``： automatic return distance mm
    - ``offectFlag``： 1-With offset location；2-Teaching point location"
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode"

Code example
---------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establishes a connection with the robot controller and returns a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    refPos = 1 #  1-datum point 2-reference point
    searchVel = 10 # positioning speed %
    searchDis = 100 #positioning distance mm
    autoBackFlag = 0 # automatic return flag，0-not automatic；1-automatic 
    autoBackVel = 10 #automatic return speed %
    autoBackDis = 100 #automatic return distance mm
    offectFlag = 0  #1-With offset location；2-Teaching point location

    descStart =[203.061, 56.768, 62.719, -177.249, 1.456, -83.597]
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
    jointREF1A =[-135.719, -119.588, -83.454, -70.245, 88.921, 88.819]

    descREF1B = [259.310, 79.998, 64.774, -179.073, -2.900, -89.790]
    jointREF1B =[-133.133, -119.029, -83.326, -70.976, 89.069, 91.401]

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
    if error[0]==0:
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

Welding wire positioning end
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WireSearchEnd(refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag)"
    "Description", "Welding wire positioning end"
    "Required parameter", "- ``refPos``： 1-datum point 2-reference point
    - ``searchVel``： positioning speed %
    - ``searchDis``： positioning distance mm
    - ``autoBackFlag``： automatic return flag，0-not automatic；1-automatic
    - ``autoBackVel``： automatic return speed %
    - ``autoBackDis``： automatic return distance mm
    - ``offectFlag``： 1-With offset location；2-Teaching point location"
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode"

Calculate the welding wire positioning offset
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "GetWireSearchOffset(seamType, method,varNameRef,varNameRes)"
    "Description", "Calculate the welding wire positioning offset"
    "Required parameter", "- ``seamType``： Weld Type
    - ``method``： Calculation method
    - ``varNameRef``： datum point 1-6，“#”Represents a pointless variable
    - ``varNameRes``： Touch points1-6，“#”Represents a pointless variable"
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode
    - ``offsetFlag``： 0-The offset is added directly to the command point；1-The offset requires coordinate transformation of the instruction point
    - ``offset``： Offset pose[x, y, z, a, b, c]"

Waiting for welding wire positioning to complete
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WireSearchWait(varname)"
    "Description", "Waiting for welding wire positioning to complete"
    "Required parameter", "- ``varName``： touch points name “RES0” ~ “RES99”"
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 

Write welding wire positioning touch points to database
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetPointToDatabase(varName,pos)"
    "Description", "Write welding wire positioning touch points to database"
    "Required parameter", "- ``varName``： touch points names “RES0” ~ “RES99”
    - ``pos``：touch points data[x, y, x, a, b, c]"
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 

Arc tracking control
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "ArcWeldTraceControl(flag,delaytime, isLeftRight, klr, tStartLr, stepMaxLr, sumMaxLr, isUpLow, kud, tStartUd, stepMaxUd, sumMaxUd, axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent)"
    "Description", "Arc tracking control"
    "Required parameter", "- ``flag``： 0-close；1-open
    - ``delayTime``：delay time ms
    - ``isLeftRight``：Left and right deviation compensation 0-close，1-open
    - ``klr``：Left and right adjustment coefficient(sensitivity)
    - ``tStartLr``：Left and right start compensation time cyc
    - ``stepMaxLr``：Left and right maximum compensation amount for each time mm
    - ``sumMaxLr``：Maximum compensation amount for left and right mm
    - ``isUpLow``：Upper and lower deviation compensation 0-close，1-open
    - ``kud``：Upper and lower adjustment coefficient(sensitivity)
    - ``tStartUd``：Upper and lower start compensation time cyc
    - ``stepMaxUd``：Maximum compensation per up and down mm
    - ``sumMaxUd``：Maximum compensation for upper and lower totals
    - ``axisSelect``：Upper and lower coordinate system selection，0-swing；1-tool；2-base
    - ``referenceType``：Upper and lower reference current Set method，0-feedback；1-constant
    - ``referSampleStartUd``：Upper and lower reference current sampling starts counting (feedback)，cyc，cyc
    - ``referSampleCountUd``：Upper and lower reference current sampling cycle count (feedback)，cyc
    - ``referenceCurrent``：Upper and lower reference current mA"
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establishes a connection with the robot controller and returns a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    flag = 1 #switch, 0-close;1-open
    delaytime=0 #delay time，ms
    isLeftRight=0 #Left and right deviation compensation 0-close，1-open
    klr = 0.06 #Left and right adjustment coefficient(sensitivity))
    tStartLr = 5 #Left and right start compensation time cyc
    stepMaxLr =5 #Left and right maximum compensation amount for each time mm
    sumMaxLr = 300 #Maximum compensation amount for left and right mm
    isUpLow = 1 #Upper and lower deviation compensation 
    kud =-0.06 #Upper and lower adjustment coefficient(sensitivity)
    tStartUd = 5 #Upper and lower start compensation time cyc
    stepMaxUd = 5 #Maximum compensation per up and down mm
    sumMaxUd = 300 #Maximum compensation for upper and lower totals
    axisSelect = 1 #Upper and lower coordinate system selection, 0-swing; 1-tool; 2-base
    referenceType = 0 #Upper and lower reference current Set method, 0-feedback: 1-constant
    referSampleStartUd = 4 # Upper and lower reference current sampling starts counting (feedback)，cyc
    referSampleCountUd = 1 # Upper and lower reference current sampling cycle count (feedback)，cyc
    referenceCurrent = 10 # Upper and lower reference current mA

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
                                sumMaxUd, axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent)
    print("WireSearchStart return:",error)

    robot.ARCStart(1, 0, 10000)
    print("ARCStart return:",error)

    robot.MoveL(enddescPose,13,0,joint_pos=endjointPos,vel =5)
    print("MoveJ return:",error)

    robot.ARCEnd(1, 0, 10000)
    print("ARCEnd return:",error)

    flag = 0
    error = robot.ArcWeldTraceControl(flag,delaytime, isLeftRight, klr, tStartLr, stepMaxLr, sumMaxLr, isUpLow, kud, tStartUd, stepMaxUd,
                                sumMaxUd, axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent)
    print("WireSearchStart return:",error)

Arc Tracking AI Passband Selection
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "ArcWeldTraceExtAIChannelConfig(channel)"
    "Description", "Arc Tracking AI Passband Selection"
    "Required parameter", "- ``channel``：Arc Tracking AI Passband Selection,[0-3]"
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 

Simulated swing start
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeaveStartSim(weaveNum)"
    "Description", "Simulated swing start"
    "Required parameter", "- ``weaveNum``: swing parameter number"
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 
    
Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establishes a connection with the robot controller and returns a robot object if the connection is successful

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

Simulated swing end
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeaveEndSim(weaveNum)"
    "Description", "Simulated swing end"
    "Required parameter", "- ``weaveNum``: swing parameter number"
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 

Start trajectory detection warning (no motion)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeaveInspectStart(weaveNum)"
    "Description", "Start trajectory detection warning (no motion)"
    "Required parameter", "- ``weaveNum``: swing parameter number"
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 
        
Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establishes a connection with the robot controller and returns a robot object if the connection is successful

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
    
End track detection warning (no motion)
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeaveInspectEnd(weaveNum)"
    "Description", "End track detection warning (no motion)"
    "Required parameter", "- ``weaveNum``: swing parameter number"
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 
    
Set welding process curve parameters
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingSetProcessParam(id, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime)"
    "Description", "Set welding process curve parameters"
    "Required parameter", "
    - ``id``： Welding process number(1-99)
    - ``startCurrent``： Arc starting current(A)
    - ``startVoltage``：Arc starting voltage(V)
    - ``startTime``： Arc starting time(ms)
    - ``weldCurrent``：Welding current(A)
    - ``weldVoltage``：Welding voltage(V)
    - ``endCurrent``：Arc ending current(A)
    - ``endVoltage``：Arc ending voltage(V)
    - ``endTime``：Arc ending time(ms)
    "
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 
            
Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establishes a connection with the robot controller and returns a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    id = 1 #Welding process number(1-99)
    startCurrent = 177 #Arc starting current(A)
    startVoltage = 27 #Arc starting voltage(V)
    startTime = 1000 #Arc starting time(ms)
    weldCurrent = 178 #Welding current(A)
    weldVoltage = 28 #Welding voltage(V)
    endCurrent = 176 #Arc ending current(A)
    endVoltage = 26 # Arc ending voltage(V)
    endTime = 1000 #Arc ending time(ms)

    error = robot.WeldingSetProcessParam(id, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage,
                                            endCurrent, endVoltage, endTime)

    print("WeldingSetProcessParam return:",error)

    error = robot.WeldingGetProcessParam(1)
    print("WeldingGetProcessParam return:",error)
        
Get welding process curve parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingGetProcessParam(id)"
    "Description", "Get welding process curve parameters"
    "Required parameter", "
    - ``id``： Welding process number(1-99)
    "
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode
    - ``Return（control state） startCurrent``：Arc starting current(A)
    - ``startVoltage``：startVoltage Arc starting voltage(V)
    - ``startTime``：startTime Arc starting time(ms)
    - ``weldCurrent``：weldCurrent Welding current(A)
    - ``weldVoltage``：weldVoltage Welding voltage(V)
    - ``endCurrent``：endCurrent Arc ending current(A)
    - ``endVoltage``：endVoltage Arc ending voltage(V) 
    - ``endTime``：endTime Arc ending time(ms)
    " 
    
Extended IO-Configure welding machine gas detection signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetAirControlExtDoNum(DONum)"
    "Description", "Extended IO-Configure welding machine gas detection signal"
    "Required parameter", "
    - ``DONum``：Gas detection signal extended DO number
    "
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 
            
Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establishes a connection with the robot controller and returns a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    #Extended IO-Configure welding machine gas detection signal
    error = robot.SetAirControlExtDoNum(10)
    print("SetAirControlExtDoNum 10 return:",error)

    #Extended IO-Configure welding machine arc starting signal
    error = robot.SetArcStartExtDoNum(11)
    print("SetArcStartExtDoNum 11 return:",error)

    #Extended IO-Configure welding machine reverse wire feeding signal
    error = robot.SetWireReverseFeedExtDoNum(12)
    print("SetWireReverseFeedExtDoNum 12 return:",error)

    #Extended IO-Configure welding machine forward wire feeding signal
    error = robot.SetWireForwardFeedExtDoNum(13)
    print("SetWireForwardFeedExtDoNum 13 return:",error)

    #Extended IO-Configure welding machine arc starting success signal
    error = robot.SetArcDoneExtDiNum(10)
    print("SetArcDoneExtDiNum 10 return:",error)

    #Extended IO-Configure welding machine ready signal
    error = robot.SetWeldReadyExtDiNum(11)
    print("SetWeldReadyExtDiNum 11 return:",error)

    #Extended IO-Configure welding interrupt recovery signal
    error = robot.SetExtDIWeldBreakOffRecover(12,13)
    print("SetExtDIWeldBreakOffRecover 12  13 return:",error)
        
Extended IO-Configure welding machine arc starting signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetArcStartExtDoNum(DONum)"
    "Description", "Extended IO-Configure welding machine arc starting signal"
    "Required parameter", "
    - ``DONum``：Welding machine arc starting signal extended DO number
    "
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 
        
Extended IO-Configure welding machine reverse wire feeding signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetWireReverseFeedExtDoNum(DONum)"
    "Description", "Extended IO-Configure welding machine reverse wire feeding signal"
    "Required parameter", "
    - ``DONum``：Reverse wire feeding signal extended DO number
    "
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 
        
Extended IO-Configure welding machine forward wire feeding signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetWireForwardFeedExtDoNum(DONum)"
    "Description", "Extended IO-Configure welding machine forward wire feeding signal"
    "Required parameter", "
    - ``DONum``：Forward wire feeding signal extended DO number
    "
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 
        
Extended IO-Configure welding machine arc starting success signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetArcDoneExtDiNum(DINum)"
    "Description", "Extended IO-Configure welding machine arc starting success signal"
    "Required parameter", "
    - ``DINum``：Arc starting success signal extended DI number
    "
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 
        
Extended IO-Configure welding machine ready signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetArcDoneExtDiNum(DINum)"
    "Description", "Extended IO-Configure welding machine ready signal"
    "Required parameter", "
    - ``DINum``：Arc starting success signal extended DI number
    "
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 
        
Extended IO-Configure welding interrupt recovery signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetExtDIWeldBreakOffRecover(reWeldDINum, abortWeldDINum)"
    "Description", "Extended IO-Configure welding interrupt recovery signal"
    "Required parameter", "
    - ``reWeldDINum``：Weld signal extension DI number for resuming welding after a weld interruption
    - ``abortWeldDINum``：Exit welding signal extension DI number after welding interruption
    "
    "Optional parameter", "NULL"
    "Return value", "- Errcode: Success -0 , Failed- errcode" 