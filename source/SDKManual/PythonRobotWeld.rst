Weld
======================

.. toctree:: 
    :maxdepth: 5

Welding Start
++++++++++++++++++++++++++++++++++
.. versionadded:: python sdk-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "ARCStart(ioType, arcNum, timeout)"
    "Description", "Welding Start"
    "Parameter", "- ``Required parameter ioType``： 0-control box IO； 1-extend IO
    - ``Required parameter arcNum``： Welder configuration file number
    - ``Required parameter timeout``： Arcing timeout time"
    "Return value", "Errcode: Success -0  Failed- errcode"

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
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "ARCEnd(ioType, arcNum, timeout)"
    "Description", "Welding end"
    "Parameter", "- ``Required parameter ioType``： 0-control box IO； 1-extend IO
    - ``Required parameter arcNum``： Welder configuration file number
    - ``Required parameter timeout``： Arcing timeout time"
    "Return value", "Errcode: Success -0  Failed- errcode"

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
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingSetCurrentRelation(currentMin, currentMax, outputVoltageMin, outputVoltageMax)"
    "Description", "Set the relationship between welding current and output analog"
    "Parameter", "- ``Required parameter currentMin``： Welding current-analog output linear relationship left point current value(A)
    - ``Required parameter currentMax``：  Welding current-analog output linear relationship right point current value(A)
    - ``Required parameter outputVoltageMin``： Welding current-analog output linear relationship left point analog output voltage value(V)
    - ``Required parameter outputVoltageMax``：Welding current-analog output linear relationship right point analog output voltage value(V)"
    "Return value", "Errcode: Success -0  Failed- errcode"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    weldIOType =0

    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    # Set the relationship between welding current and output analog
    ret = robot.WeldingSetCurrentRelation(0,400,0,10)
    print("WeldingSetCurrentRelation ", ret)
    time.sleep(1)
    # Obtain the corresponding relationship between welding current and output analog
    ret = robot.WeldingGetCurrentRelation()
    print("WeldingGetCurrentRelation ", ret)
    time.sleep(1)

    #Set the relationship between welding voltage and output analog
    ret = robot.WeldingSetVoltageRelation(0,400,0,10)
    print("WeldingSetVoltageRelation ", ret)
    time.sleep(1)
    # The corresponding relationship between welding voltage and output analog is obtained
    ret = robot.WeldingGetVoltageRelation()
    print("WeldingGetVoltageRelation ", ret)
    time.sleep(1)

    # Set welding current
    ret = robot.WeldingSetCurrent(weldIOType,100,0)
    print("WeldingSetCurrent ", ret)
    time.sleep(1)

    # Set welding voltage
    ret = robot.WeldingSetVoltage(weldIOType,19,1)
    print("WeldingSetVoltage ", ret)
    time.sleep(1)

Set the relationship between welding voltage and output analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingSetVoltageRelation(weldVoltageMin, weldVoltageMax, outputVoltageMin, outputVoltageMax)"
    "Description", "Set the relationship between welding voltage and output analog"
    "Parameter", "- ``Required parameter weldVoltageMin``： Welding voltage - analog output linear relationship left spot welding voltage value(V)
    - ``Required parameter weldVoltageMax``： Welding voltage - analog output linear relationship right spot welding voltage value(V)
    - ``Required parameter outputVoltageMin``： Welding voltage-analog output linear relationship left point analog output voltage value(V)
    - ``Required parameter outputVoltageMax``：Welding voltage-analog output linear relationship right point analog output voltage value(V)"
    "Return value", "Errcode: Success -0  Failed- errcode"

Obtain the corresponding relationship between welding current and output analog
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingGetCurrentRelation()"
    "Description", "Obtain the corresponding relationship between welding current and output analog"
    "Parameter", "Null"
    "Return value", "- Errcode: Success -0  Failed- errcode
    - Return:(if success)currentMin,currentMax,outputVoltageMin,outputVoltageMax
    - currentMin Welding current-analog output linear relationship left point current value(A)
    - currentMax Welding current-analog output linear relationship right point current value(A)
    - outputVoltageMin Welding current-analog output linear relationship left point analog output voltage value(V)
    - outputVoltageMax Welding current-analog output linear relationship right point analog output voltage value(V)"

Obtain the corresponding relationship between welding voltage and output analog
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingGetVoltageRelation()"
    "Description", "Obtain  the corresponding relationship between welding voltage and output analog"
    "Parameter", "Null"
    "Return value", "- Errcode: Success -0  Failed- errcode
    - Return:（if success）weldVoltageMin,weldVoltageMax,outputVoltageMin, outputVoltageMax, weldVoltageMin 
    - weldVoltageMin  Welding voltage - analog output linear relationship left spot welding voltage value(V)
    - weldVoltageMax  Welding voltage - analog output linear relationship right spot welding voltage value(V)
    - outputVoltageMin  Welding voltage-analog output linear relationship left point analog output voltage value(V)
    - outputVoltageMax  Welding voltage-analog output linear relationship right point analog output voltage value(V)"

Set welding current
++++++++++++++++++++++++++++++++++
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingSetCurrent(ioType,current, AOIndex)"
    "Description", "Set welding current"
    "Parameter", "- ``Required parameter ioType``： 0-control box IO； 1-extend IO
    - ``Required parameter current``： current welding current(A)
    - ``Required parameter AOIndex``： Welding current control box analog output port(0-1)"
    "Return value", "- Errcode: Success -0  Failed- errcode"

Set welding voltage
++++++++++++++++++++++++++++++++++
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeldingSetVoltage(ioType,voltage, AOIndex)"
    "Description", "Set welding voltage"
    "Parameter", "- ``Required parameter ioType``： 0-control box IO； 1-extend IO 
    - ``Required parameter voltage``： welding voltage(V)
    - ``Required parameter AOIndex``： Welding voltage control box analog output port(0-1)"
    "Return value", "- Errcode: Success -0  Failed- errcode"

Set weave parameters
++++++++++++++++++++++++++++++++++
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeaveSetPara(weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange, weaveLeftStayTime, weaveRightStayTime, weaveCircleRadio, weaveStationary)"
    "Description", "Set weave parameters"
    "Parameter", "- ``Required parameter weaveNum``： parameters number
    - ``Required parameter weaveType``： weave type：0- plane triangular weave ; 1- vertical L-shaped triangular weave; 2- clockwise circular weave; 3-counterclockwise circular weave; 4-plane sine weave; 5-vertical L-shaped sine weave; 6- vertical triangular weave; 7- Vertical sine weave
    - ``Required parameter weaveFrequency``： weave frequency(Hz)
    - ``Required parameter weaveIncStayTime``： Wait mode 0- period does not contain wait time; 1- Period contains the wait time
    - ``Required parameter weaveRange``： weave amplitude(mm)
    - ``Required parameter weaveLeftStayTime``： weave left residence time(ms)
    - ``Required parameter weaveRightStayTime``： weave right residence time(ms)
    - ``Required parameter weaveCircleRadio``： Circular wiggle-pullback ratio(0-100%)
    - ``Required parameter weaveStationary``： weave position wait, 0- position continue to move within the waiting time; 1- The position is stationary during the waiting time"
    "Return value", "- Errcode: Success -0  Failed- errcode"

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
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeaveOnlineSetPara(weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange, weaveLeftStayTime, weaveRightStayTime, weaveCircleRadio, weaveStationary)"
    "Description", "Set weave parameters in real time"
    "Parameter", "- ``Required parameter weaveNum``： parameters number
    - ``Required parameter weaveType``： weave type：0- plane triangular weave ; 1- vertical L-shaped triangular weave; 2- clockwise circular weave; 3-counterclockwise circular weave; 4-plane sine weave; 5-vertical L-shaped sine weave; 6- vertical triangular weave; 7- Vertical sine weave
    - ``Required parameter weaveFrequency``： weave frequency(Hz)
    - ``Required parameter weaveIncStayTime``： Wait mode 0- period does not contain wait time; 1- Period contains the wait time
    - ``Required parameter weaveRange``： weave amplitude(mm)
    - ``Required parameter weaveLeftStayTime``： weave left residence time(ms)
    - ``Required parameter weaveRightStayTime``：  weave right residence time(ms)
    - ``Required parameter weaveCircleRadio``： Circular wiggle-pullback ratio(0-100%)
    - ``Required parameter weaveStationary``： weave position wait, 0- position continue to move within the waiting time; 1- The position is stationary during the waiting time"
    "Return value", "- Errcode: Success -0  Failed- errcode"

Weave start
++++++++++++++++++++++++++++++++++
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeaveStart(weaveNum)"
    "Description", "Weave start"
    "Parameter", "- ``Required parameter weaveNum``： Weave welding parameter configuration number"
    "Return value", "- Errcode: Success -0  Failed- errcode"

Weave end
++++++++++++++++++++++++++++++++++
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "WeaveEnd(weaveNum)"
    "Description", "Weave end"
    "Parameter", "- ``Required parameter weaveNum``： Weave welding parameter configuration number"
    "Return value", "- Errcode: Success -0  Failed- errcode"

Forward wire feed
++++++++++++++++++++++++++++++++++
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetForwardWireFeed(ioType, wireFeed)"
    "Description", "Forward wire feed"
    "Parameter", "- ``Required parameter ioType``： 0-control box IO； 1-extend IO
    - ``Required parameter wireFeed``： wire control: 0-stop wire feed ；1-wire feed"
    "Return value", "- Errcode: Success -0  Failed- errcode"

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
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetReverseWireFeed(ioType, wireFeed)"
    "Description", "Reverse wire feed"
    "Parameter", "- ``Required parameter ioType``： 0-control box IO； 1-extend IO
    - ``Required parameter wireFeed``： wire control  0-stop wire feed ；1-wire feed"
    "Return value", "- Errcode: Success -0  Failed- errcode"

Aspirated
++++++++++++++++++++++++++++++++++
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SetAspirated(ioType, airControl)"
    "Description", "Aspirated"
    "Parameter", "- ``Required parameter ioType``： 0-control box IO； 1-extend IO
    - ``Required parameter airControl``： 0-stop aspirated；1-aspirated"
    "Return value", "- Errcode: Success -0  Failed- errcode"

Segment weld start
++++++++++++++++++++++++++++++++++
.. versionadded:: python sdk-v2.0.1
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "SegmentWeldStart(startDesePos,  endDesePos, startJPos, endJPos, weldLength, noWeldLength, weldIOType, arcNum, weldTimeout, isWeave,weaveNum,tool,user,vel=20.0, acc=0.0, ovl=100.0, blendR=-1.0,exaxis_pos=[0.0, 0.0, 0.0, 0.0],  search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])"
    "Description", "Segment weld start"
    "Parameter", "- ``Required parameter startDesePos``： Starting point Cartesian position
    - ``Required parameter endDesePos``： Ending point Cartesian position
    - ``Required parameter startJPos``： Starting point joint position
    - ``Required parameter endJPos``：Ending point joint position 
    - ``Required parameter weldLength``：Weld length(mm)
    - ``Required parameter noWeldLength``：Length of unwelded section(mm)  
    - ``Required parameter weldIOType``：0-control box IO； 1-extend IO
    - ``Required parameter timeout``：Arcing timeout time
    - ``Required parameter isWeave``：True-Weave False- Not weave
    - ``Required parameter weaveNum``：Weave welding parameter configuration number Required parameter
    - ``Required parameter tool``：Tool coordinate number, range[0~14]
    - ``Required parameter user``：Workpiece coordinate number, range,[0~14]
    - ``Optional parameter vel``：Percentage of speed, range [0~100],[0~100] , Default 20.0
    - ``Optional parameter acc``：Acceleration percentage, range [0~100] ,not open for now , Default 0.0
    - ``Optional parameter ovl``：Velocity scaling factor, range [0~100] , Default 100.0 [-1.0]- movement in place (blocking), [0~1000.0] Smoothing radius (non-blocking), unit: mm, Default -1.0
    - ``Optional parameter blendR``：[-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm, Default -1.0
    - ``Optional parameter exaxis_pos``：Position of expansion shaft, unit: m , Default [0.0,0.0,0.0,0.0]
    - ``Optional parameter search``：0-no wire seeking, 1-wire seeking Default 0
    - ``Optional parameter offset_flag``：0-no offset, 1-offset in base/job coordinate system, 2- offset in tool coordinate system Default 0
    - ``Optional parameter offset_pos``：The pose offset Default [0.0,0.0,0.0,0.0,0.0,0.0]"
    "Return value", "- Errcode: Success -0  Failed -errcode"

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