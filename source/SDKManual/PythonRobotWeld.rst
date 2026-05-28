Weld
======================

.. toctree::
    :maxdepth: 5
    
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
.. versionchanged:: python SDK-v2.0.5
    
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

Example code for setting welding parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.WeldingSetProcessParam(1, 177, 27, 1000, 178, 28, 176, 26, 1000)
    robot.WeldingSetProcessParam(2, 188, 28, 555, 199, 29, 133, 23, 333)
    start_current = 0
    start_voltage = 0
    start_time = 0
    weld_current = 0
    weld_voltage = 0
    end_current = 0
    end_voltage = 0
    end_time = 0
    error, start_current, start_voltage, start_time, weld_current, weld_voltage, end_current,end_voltage, end_time = robot.WeldingGetProcessParam(1)
    print(f"the Num 1 process param is {start_current} {start_voltage} {start_time} {weld_current} {weld_voltage} {end_current} {end_voltage} {end_time}")
    error, start_current, start_voltage, start_time, weld_current, weld_voltage, end_current,end_voltage, end_time = robot.WeldingGetProcessParam(2)
    print(f"the Num 2 process param is {start_current} {start_voltage} {start_time} {weld_current} {weld_voltage} {end_current} {end_voltage} {end_time}")
    rtn = robot.WeldingSetCurrentRelation(0, 400, 0, 10, 0)
    print(f"WeldingSetCurrentRelation rtn is: {rtn}")
    rtn = robot.WeldingSetVoltageRelation(0, 40, 0, 10, 1)
    print(f"WeldingSetVoltageRelation rtn is: {rtn}")
    current_min = 0
    current_max = 0
    vol_min = 0
    vol_max = 0
    output_vmin = 0
    output_vmax = 0
    cur_index = 0
    vol_index = 0
    rtn,current_min, current_max, output_vmin, output_vmax, cur_index = robot.WeldingGetCurrentRelation()
    print(f"WeldingGetCurrentRelation rtn is: {rtn}")
    print(f"current min {current_min} current max {current_max} output vol min {output_vmin} output vol max {output_vmax}")
    rtn,vol_min, vol_max, output_vmin, output_vmax, vol_index = robot.WeldingGetVoltageRelation()
    print(f"WeldingGetVoltageRelation rtn is: {rtn}")
    print(f"vol min {vol_min} vol max {vol_max} output vol min {output_vmin} output vol max {output_vmax}")
    rtn = robot.WeldingSetCurrent(1, 100, 0, 0)
    print(f"WeldingSetCurrent rtn is: {rtn}")
    time.sleep(3)
    rtn = robot.WeldingSetVoltage(1, 10, 0, 0)
    print(f"WeldingSetVoltage rtn is: {rtn}")
    rtn = robot.WeaveSetPara(0, 0, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0,0.0, 60.000000)
    print(f"rtn is: {rtn}")
    robot.WeaveOnlineSetPara(0, 0, 1, 0, 20, 0, 0, 0, 0)
    rtn = robot.WeldingSetCheckArcInterruptionParam(1, 200)
    print(f"WeldingSetCheckArcInterruptionParam {rtn}")
    rtn = robot.WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0)
    print(f"WeldingSetReWeldAfterBreakOffParam {rtn}")
    enable = 0
    length = 0
    velocity = 0
    move_type = 0
    check_enable = 0
    arc_interrupt_time_length = 0
    rtn,check_enable, arc_interrupt_time_length = robot.WeldingGetCheckArcInterruptionParam()
    print(f"WeldingGetCheckArcInterruptionParam checkEnable {check_enable} arcInterruptTimeLength {arc_interrupt_time_length}")
    rtn,enable, length, velocity, move_type = robot.WeldingGetReWeldAfterBreakOffParam()
    print(f"WeldingGetReWeldAfterBreakOffParam enable = {enable}, length = {length}, velocity = {velocity}, moveType = {move_type}")
    robot.SetWeldMachineCtrlModeExtDoNum(17)
    for i in range(5):
        robot.SetWeldMachineCtrlMode(0)
        time.sleep(1)
        robot.SetWeldMachineCtrlMode(1)
        time.sleep(1)
    robot.CloseRPC()

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

The detection parameters of unexpected interruption of robot welding arc were obtained
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.8

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

Set the detection parameters of robot welding arc unexpected interruption
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingSetCheckArcInterruptionParam(checkEnable, arcInterruptTimeLength)``"
    "Description", "Set the detection parameters of robot welding arc unexpected interruption"
    "Mandatory parameters", "- ``checkEnable``: Whether to enable detection. 0: Indicates that the function is disabled. 1- Enable
    - ``arcInterruptTimeLength``:Arc break confirmation time (ms)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Obtain the robot welding interrupt recovery parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.8

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

Set the robot welding interrupt recovery parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.8

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

Set welder control mode to expand DO port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWeldMachineCtrlModeExtDoNum(DONum)``"
    "Description", "Set welder control mode to extend DO port"
    "Mandatory parameters", "- ``DONum``: welder control mode DO port (0-127)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode" 

Setting the welder control mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWeldMachineCtrlMode(mode, ioType)``"
    "Description", "Set welding machine control mode"
    "Required Parameters", "
    - ``ioType``: Control type; 0-Control box IO; 1-Digital communication protocol (UDP); 2-Digital communication protocol (ModbusTCP)
    - ``mode``: Welding machine control mode; 0-One-knob mode"
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure-errcode"

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

aspiration
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

Set the robot to resume welding after welding interruption
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.8

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
.. versionadded:: python SDK-v2.0.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeldingAbortWeldAfterBreakOff()``"
    "Description", "Set the robot to exit welding after welding interruption"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Sample code for robot welding control
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.SetForwardWireFeed(0, 1)
    time.sleep(1)
    robot.SetForwardWireFeed(0, 0)
    robot.SetReverseWireFeed(0, 1)
    time.sleep(1)
    robot.SetReverseWireFeed(0, 0)
    robot.SetAspirated(0, 1)
    time.sleep(1)
    robot.SetAspirated(0, 0)
    robot.WeldingSetCurrent(1, 230, 0, 0)
    robot.WeldingSetVoltage(1, 24, 0, 1)
    p1Desc = [228.879, -503.594, 453.984, -175.580, 8.293, 171.267]
    p1Joint = [102.700, -85.333, 90.518, -102.365, -83.932, 22.134]
    p2Desc = [-333.302, -435.580, 449.866, -174.997, 2.017, 109.815]
    p2Joint = [41.862, -85.333, 90.526, -100.587, -90.014, 22.135]
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    robot.MoveJ(joint_pos=p1Joint, tool=13, user=0)
    robot.ARCStart(1, 0, 10000)
    robot.WeaveStart(0)
    robot.MoveL(desc_pos=p2Desc, tool=13, user=0)
    robot.ARCEnd(1, 0, 10000)
    robot.WeaveEnd(0)
    robot.WeldingStartReWeldAfterBreakOff()
    robot.WeldingAbortWeldAfterBreakOff()
    robot.CloseRPC()

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

Sample robot segment welding code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.WeldingSetCurrent(1, 230, 0, 0)
    robot.WeldingSetVoltage(1, 24, 0, 1)
    p1Desc = [228.879, -503.594, 453.984, -175.580, 8.293, 171.267]
    p1Joint = [102.700, -85.333, 90.518, -102.365, -83.932, 22.134]
    p2Desc = [-333.302, -435.580, 449.866, -174.997, 2.017, 109.815]
    p2Joint = [41.862, -85.333, 90.526, -100.587, -90.014, 22.135]
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    rtn = robot.SegmentWeldStart(p1Desc, p2Desc, p1Joint, p2Joint, 20, 20, 0, 0, 5000, 0, 0, 0, 0)
    print(f"SegmentWeldStart rtn is {rtn}")
    robot.CloseRPC()

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

Robot swing gradient welding code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    p1Desc = [228.879, -503.594, 453.984, -175.580, 8.293, 171.267]
    p1Joint = [102.700, -85.333, 90.518, -102.365, -83.932, 22.134]
    p2Desc = [-333.302, -435.580, 449.866, -174.997, 2.017, 109.815]
    p2Joint = [41.862, -85.333, 90.526, -100.587, -90.014, 22.135]
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    robot.MoveJ(joint_pos= p1Joint,tool= 13,user= 0)
    robot.WeaveStartSim(0)
    robot.MoveL(desc_pos= p2Desc,tool= 13,user= 0)
    robot.WeaveEndSim(0)
    robot.MoveJ(joint_pos= p1Joint,tool= 13,user= 0)
    robot.WeaveInspectStart(0)
    robot.MoveL(desc_pos= p2Desc,tool= 13,user= 0,)
    robot.WeaveInspectEnd(0)
    robot.WeldingSetVoltage(1, 19, 0, 0)
    robot.WeldingSetCurrent(1, 190, 0, 0)
    robot.MoveL(desc_pos= p1Desc,tool= 1,user= 1,vel= 100,acc= 100,ovl= 50)
    robot.ARCStart(1, 0, 10000)
    robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0)
    robot.WeaveStart(0)
    robot.WeaveChangeStart(1, 0, 50, 30)
    robot.MoveL(desc_pos= p2Desc,tool= 1,user= 1,vel= 100)
    robot.WeaveChangeEnd()
    robot.WeaveEnd(0)
    robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0)
    robot.ARCEnd(1, 0, 10000)
    robot.CloseRPC()

Wobble end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.9-3.7.9

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WeaveChangeEnd()``"
    "Description", "Wobble end"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

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

Example code for setting up extended IO solder signals
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    rtn = robot.SetArcStartExtDoNum(10)
    print(f"SetArcStartExtDoNum rtn is {rtn}")
    rtn = robot.SetAirControlExtDoNum(20)
    print(f"SetAirControlExtDoNum rtn is {rtn}")
    rtn = robot.SetWireForwardFeedExtDoNum(30)
    print(f"SetWireForwardFeedExtDoNum rtn is {rtn}")
    rtn = robot.SetWireReverseFeedExtDoNum(40)
    rtn = robot.SetWeldReadyExtDiNum(50)
    print(f"SetWeldReadyExtDiNum rtn is {rtn}")
    rtn = robot.SetArcDoneExtDiNum(60)
    print(f"SetArcDoneExtDiNum rtn is {rtn}")
    rtn = robot.SetExtDIWeldBreakOffRecover(70, 80)
    print(f"SetExtDIWeldBreakOffRecover rtn is {rtn}")
    rtn = robot.SetWireSearchExtDIONum(0, 1)
    print(f"SetWireSearchExtDIONum rtn is {rtn}")
    robot.CloseRPC()

Arc tracking control
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.9-3.7.9

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
.. versionadded:: python SDK-v2.0.5

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

Example code for multi-layer multi-pass welding arc tracking
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    mulitilineorigin1_joint = [-24.090, -63.501, 84.288, -111.940, -93.426, 57.669]
    mulitilineorigin1_desc = [-677.559, 190.951, -1.205, 1.144, -41.482, -82.577]
    mulitilineX1_desc = [-677.556, 211.949, -1.206]
    mulitilineZ1_desc = [-677.564, 190.956, 19.817]
    mulitilinesafe_joint = [-25.734, -63.778, 81.502, -108.975, -93.392, 56.021]
    mulitilinesafe_desc = [-677.561, 211.950, 19.812, 1.144, -41.482, -82.577]
    mulitilineorigin2_joint = [-29.743, -75.623, 101.241, -116.354, -94.928, 55.735]
    mulitilineorigin2_desc = [-563.961, 215.359, -0.681, 2.845, -40.476, -87.443]
    mulitilineX2_desc = [-563.965, 220.355, -0.680]
    mulitilineZ2_desc = [-563.968, 215.362, 4.331]
    epos = [0, 0, 0, 0]
    offset = [0, 0, 0, 0, 0, 0]
    time.sleep(0.01)
    error = robot.MoveJ(joint_pos= mulitilinesafe_joint,tool= 13,user= 0,vel= 10)
    print(f"MoveJ return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin1_desc,tool= 13,user= 0,vel= 10,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.MoveJ(joint_pos= mulitilinesafe_joint,tool= 13,user= 0,vel= 10)
    print(f"MoveJ return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin2_desc,tool= 13,user= 0,vel= 10,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.MoveJ(joint_pos= mulitilinesafe_joint,tool= 13,user= 0,vel= 10)
    print(f"MoveJ return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin1_desc,tool= 13,user= 0,vel= 10,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.ARCStart(1, 0, 3000)
    print(f"ARCStart return: {error}")
    error = robot.WeaveStart(0)
    print(f"WeaveStart return: {error}")
    error = robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10,0,0)
    print(f"ArcWeldTraceControl return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin2_desc,tool= 13,user= 0,vel= 1,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10,0,0)
    print(f"ArcWeldTraceControl return: {error}")
    error = robot.WeaveEnd(0)
    print(f"WeaveEnd return: {error}")
    error = robot.ARCEnd(1, 0, 10000)
    print(f"ARCEnd return: {error}")
    error = robot.MoveJ(joint_pos= mulitilinesafe_joint,tool= 13,user= 0,vel= 10)
    print(f"MoveJ return: {error}")
    error,offset = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc[:3], mulitilineX1_desc, mulitilineZ1_desc, 10.0, 0.0, 0.0)
    print(f"MultilayerOffsetTrsfToBase return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin1_desc,tool= 13,user= 0,vel= 10,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.ARCStart(1, 0, 3000)
    print(f"ARCStart return: {error}")
    error, offset = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc[:3], mulitilineX2_desc, mulitilineZ2_desc, 10, 0, 0)
    print(f"MultilayerOffsetTrsfToBase return: {error}")
    error = robot.ArcWeldTraceReplayStart()
    print(f"ArcWeldTraceReplayStart return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin2_desc,tool= 13,user= 0,vel= 2,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.ArcWeldTraceReplayEnd()
    print(f"ArcWeldTraceReplayEnd return: {error}")
    error = robot.ARCEnd(1, 0, 10000)
    print(f"ARCEnd return: {error}")
    error = robot.MoveJ(joint_pos= mulitilinesafe_joint,tool= 13,user= 0,vel= 10)
    print(f"MoveJ return: {error}")
    error, offset = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc[:3], mulitilineX1_desc, mulitilineZ1_desc, 0, 10, 0)
    print(f"MultilayerOffsetTrsfToBase return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin1_desc,tool= 13,user= 0,vel= 10,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.ARCStart(1, 0, 3000)
    print(f"ARCStart return: {error}")
    error, offset = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc[:3], mulitilineX2_desc, mulitilineZ2_desc, 0, 10, 0)
    error = robot.ArcWeldTraceReplayStart()
    print(f"ArcWeldTraceReplayStart return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin2_desc,tool= 13,user= 0,vel= 2,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.ArcWeldTraceReplayEnd()
    print(f"ArcWeldTraceReplayEnd return: {error}")
    error = robot.ARCEnd(1, 0, 3000)
    print(f"ARCEnd return: {error}")
    error = robot.MoveJ(joint_pos= mulitilinesafe_joint,tool= 13,user= 0,vel= 10)
    print(f"MoveJ return: {error}")
    robot.CloseRPC()

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

Example arc tracking code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    toolCoord = [0, 0, 200, 0, 0, 0]
    robot.SetToolCoord(1, toolCoord, 0, 0, 1, 0)
    wobjCoord = [0, 0, 0, 0, 0, 0]
    robot.SetWObjCoord(1, wobjCoord, 0)
    robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10)
    robot.ExtDevLoadUDPDriver()
    robot.SetWireSearchExtDIONum(0, 0)
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    descStart = [216.543, 445.175, 93.465, 179.683, 1.757, -112.641]
    jointStart = [-128.345, -86.660, 114.679, -119.625, -89.219, 74.303]
    descEnd = [111.143, 523.384, 87.659, 179.703, 1.835, -97.750]
    jointEnd = [-113.454, -81.060, 109.328, -119.954, -89.218, 74.302]
    error = robot.MoveL(desc_pos=descStart,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos=descEnd,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    descREF0A = [142.135, 367.604, 86.523, 179.728, 1.922, -111.089]
    jointREF0A = [-126.794, -100.834, 128.922, -119.864, -89.218, 74.302]
    descREF0B = [254.633, 463.125, 72.604, 179.845, 2.341, -114.704]
    jointREF0B = [-130.413, -81.093, 112.044, -123.163, -89.217, 74.303]
    descREF1A = [92.556, 485.259, 47.476, -179.932, 3.130, -97.512]
    jointREF1A = [-113.231, -83.815, 119.877, -129.092, -89.217, 74.303]
    descREF1B = [203.103, 583.836, 63.909, 179.991, 2.854, -103.372]
    jointREF1B = [-119.088, -69.676, 98.692, -121.761, -89.219, 74.303]
    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchStart return: {error}")
    error = robot.MoveL(desc_pos=descREF0A,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos=descREF0B,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.WireSearchWait("REF0")
    print(f"WireSearchWait return: {error}")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchEnd return: {error}")
    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchStart return: {error}")
    error = robot.MoveL(desc_pos= descREF1A,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos= descREF1B,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.WireSearchWait("REF1")
    print(f"WireSearchWait return: {error}")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchStart return: {error}")
    error = robot.MoveL(desc_pos= descREF0A,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos= descREF0B,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.WireSearchWait("RES0")
    print(f"WireSearchWait return: {error}")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchEnd return: {error}")
    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchStart return: {error}")
    error = robot.MoveL(desc_pos= descREF1A,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos= descREF1B,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.WireSearchWait("RES1")
    print(f"WireSearchWait return: {error}")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchEnd return: {error}")
    varNameRef = ["REF0", "REF1", "#", "#", "#", "#"]
    varNameRes = ["RES0", "RES1", "#", "#", "#", "#"]
    offectFlag = 0
    offectPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    error, offectFlag, offectPos = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes)
    print(f"GetWireSearchOffset return: {error}")
    error = robot.PointsOffsetEnable(0, offectPos)
    print(f"PointsOffsetEnable return: {error}")
    error = robot.MoveL(desc_pos= descStart,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos= descEnd,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.PointsOffsetDisable()
    robot.CloseRPC()

Welding wire position finding start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WireSearchStart(refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag)``"
    "Description", "Welding wire seek start"
    "Mandatory parameters", "- ``refPos``: 1 - reference point 0 - contact point
    - ``searchVel``: search velocity %
    - ``searchDis``: search distance mm
    - ``autoBackFlag``: autoBackFlag, 0 - not auto; - auto
    - ``autoBackVel``: Automatic Back Velocity %
    - ``autoBackDis``: automatic back distance mm
    - ``offectFlag``: 1 - seek with offset; 0 - teach point seek"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

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

Example of robot wire locating code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    toolCoord = [0, 0, 200, 0, 0, 0]
    robot.SetToolCoord(1, toolCoord, 0, 0, 1, 0)
    wobjCoord = [0, 0, 0, 0, 0, 0]
    robot.SetWObjCoord(1, wobjCoord, 0)
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    descStart = [216.543, 445.175, 93.465, 179.683, 1.757, -112.641]
    jointStart = [-128.345, -86.660, 114.679, -119.625, -89.219, 74.303]
    descEnd = [111.143, 523.384, 87.659, 179.703, 1.835, -97.750]
    jointEnd = [-113.454, -81.060, 109.328, -119.954, -89.218, 74.302]
    error = robot.MoveL(desc_pos=descStart,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos=descEnd,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    descREF0A = [142.135, 367.604, 86.523, 179.728, 1.922, -111.089]
    jointREF0A = [-126.794, -100.834, 128.922, -119.864, -89.218, 74.302]
    descREF0B = [254.633, 463.125, 72.604, 179.845, 2.341, -114.704]
    jointREF0B = [-130.413, -81.093, 112.044, -123.163, -89.217, 74.303]
    descREF1A = [92.556, 485.259, 47.476, -179.932, 3.130, -97.512]
    jointREF1A = [-113.231, -83.815, 119.877, -129.092, -89.217, 74.303]
    descREF1B = [203.103, 583.836, 63.909, 179.991, 2.854, -103.372]
    jointREF1B = [-119.088, -69.676, 98.692, -121.761, -89.219, 74.303]
    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchStart return: {error}")
    error = robot.MoveL(desc_pos=descREF0A,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos=descREF0B,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.WireSearchWait("REF0")
    print(f"WireSearchWait return: {error}")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchEnd return: {error}")
    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchStart return: {error}")
    error = robot.MoveL(desc_pos= descREF1A,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos= descREF1B,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.WireSearchWait("REF1")
    print(f"WireSearchWait return: {error}")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchStart return: {error}")
    error = robot.MoveL(desc_pos= descREF0A,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos= descREF0B,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.WireSearchWait("RES0")
    print(f"WireSearchWait return: {error}")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchEnd return: {error}")
    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchStart return: {error}")
    error = robot.MoveL(desc_pos= descREF1A,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos= descREF1B,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.WireSearchWait("RES1")
    print(f"WireSearchWait return: {error}")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchEnd return: {error}")
    varNameRef = ["REF0", "REF1", "#", "#", "#", "#"]
    varNameRes = ["RES0", "RES1", "#", "#", "#", "#"]
    offectFlag = 0
    offectPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    error, offectFlag, offectPos = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes)
    print(f"GetWireSearchOffset return: {error}")
    error = robot.PointsOffsetEnable(0, offectPos)
    print(f"PointsOffsetEnable return: {error}")
    error = robot.MoveL(desc_pos= descStart,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos= descEnd,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.PointsOffsetDisable()
    robot.CloseRPC()

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

Robot welding current voltage gradient code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

Set custom swing parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``CustomWeaveSetPara(id, pointNum, point, stayTime, frequency, incStayType, stationary)``"
    "Description", "Set custom swing parameters"
    "Mandatory parameters", "- ``id``：Custom swing number: 0-2
    - ``pointNum``：The number of swing points is 0-10
    - ``point``：Move endpoint data x,y,z
    - ``stayTime``：The swing residence time is ms
    - ``frequency``：Swing frequency Hz
    - ``incStayType``：Waiting mode: 0-cycle does not contain waiting time; The 1-cycle contains the waiting time
    - ``stationary``：Swing position waiting: 0- continue to move within the waiting time; 1 -The position is stationary during the waiting time"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Gets custom swing parameters
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``CustomWeaveGetPara(id)``"
    "Description", "Gets custom swing parameters"
    "Mandatory parameters", "- ``id``：Custom swing number: 0-2"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode 
    - ``pointNum``：The number of swing points is 0-10
    - ``point``：Move endpoint data x,y,z
    - ``stayTime``：The swing residence time is ms
    - ``frequency``：Swing frequency Hz
    - ``incStayType``：Waiting mode: 0-cycle does not contain waiting time; The 1-cycle contains the waiting time
    - ``stationary``：Swing position waiting: 0- continue to move within the waiting time; 1 -The position is stationary during the waiting time"

Custom swing parameter code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    point = [0.0] * 30
    point[0] = -3.0
    point[1] = -3.0
    point[2] = 0.0
    point[3] = -6.0
    point[4] = 0.0
    point[5] = 0.0
    point[6] = -3.0
    point[7] = 3.0
    point[8] = 0.0
    point[9] = 0.0
    point[10] = 0.0
    point[11] = 0.0
    stayTime = [0.0] * 10
    rtn = robot.CustomWeaveSetPara(2, 4, point, stayTime, 1.000, 0, 0)
    print(f"CustomWeaveSetPara rtn is {rtn}")
    time.sleep(1)
    pointNum = 0
    frequency = 0.0
    incStayType = 0
    stationary = 0
    rtn, pointNum, point, stayTime, frequency, incStayType, stationary = robot.CustomWeaveGetPara(2)
    print(f"pointNum is {pointNum}")
    for i in range(pointNum):
        print(f"point {i}, point x y z {point[i * 3 + 0]},{point[i * 3 + 1]},{point[i * 3 + 2]}")
    print(f"fre is {frequency}, stay is {incStayType},{stationary}")
    robot.WeaveSetPara(0, 9, 1.000000, 1, 5.000000, 6.000000, 5.000000, 50, 100, 100, 0, 1, 0.000000, 0.000000)
    desc_p1 = [-288.650, 367.807, 288.404, 0.000, -0.001, 0.001]
    desc_p2 = [-431.714, 367.815, 288.415, 0.001, 0.001, 0.000]
    desc_p3 = [-348.666, 427.798, 288.404, -0.000, -0.000, 0.001]
    j1 = [140.656, -84.560, -91.707, -93.734, 90.000, 50.655]
    j2 = [149.873, -98.298, -77.599, -94.103, 90.000, 59.873]
    j3 = [139.773, -96.173, -80.014, -93.814, 90.000, 49.772]
    epos = [0.0] * 4
    offset_pos = [0.0] * 6
    robot.MoveJ(joint_pos=j1, tool=3, user=0, vel=100)
    robot.WeaveStart(0)
    robot.Circle(desc_pos_p=desc_p3, tool_p=3, user_p=0, vel_p=50, desc_pos_t=desc_p2, tool_t=3, user_t=0, vel_t=50, oacc=10)
    robot.WeaveEnd(0)
    robot.MoveJ(joint_pos=j1, tool=3, user=0, vel=100)
    robot.WeaveStart(0)
    robot.MoveC(desc_pos_p=desc_p3, tool_p=3, user_p=0, vel_p=50, desc_pos_t=desc_p2, tool_t=3, user_t=0, vel_t=50)
    robot.WeaveEnd(0)
    robot.MoveJ(joint_pos=j1, tool=3, user=0, vel=100)
    robot.WeaveStart(0)
    robot.MoveL(desc_pos=desc_p2, tool=3, user=0, vel=100, ovl=10, speedPercent=100)
    robot.WeaveEnd(0)
    robot.CloseRPC()

Laser Welding Machine Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLaserWeldingParam(self, num, scanSpeed, scanWidth, peakPower, dutyCycle, Freq, io_type=1)``"
    "Description", "Write configuration parameters of one of the 10 process groups of the laser welding machine and configure them to the welder"
    "Required Parameters", "
    - ``io_type``: Communication type 0-IO 1-UDP
    - ``Num``: Group number to set (1~10)
    - ``scanSpeed``: Scanning speed
    - ``scanWidth``: Scanning width
    - ``peakPower``: Peak power
    - ``dutyCycle``: Duty cycle
    - ``Freq``: Frequency
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Start/Stop Laser Welding
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLaserWeldingStartEnd(self, status, io_type=1, max_waittime=10000)``"
    "Description", "Start/stop laser welding machine"
    "Required Parameters", "
    - ``io_type``: Communication type 0-IO 1-UDP
    - ``status``: Control word 0-laser off 1-laser on
    - ``max_waittime``: Maximum wait time
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Enable/Disable Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLaserWeldingEnable(self, status, io_type=1)``"
    "Description", "Enable/disable laser welding machine"
    "Required Parameters", "
    - ``io_type``: Communication type 0-IO 1-UDP
    - ``status``: 0-disable 1-enable
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Laser Welding Machine Fault Reset
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ResetLaserWeldingErr(self,status, io_type=1)``"
    "Description", "Laser welding machine fault reset"
    "Required Parameters", "
    - ``io_type``: Communication type 0-IO 1-UDP
    - ``status``: Control word 0-invalid 1-fault reset
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Get Laser Welding Machine Running Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetLaserWeldingRunningState(self, io_type=1)``"
    "Description", "Get laser welding machine running status"
    "Required Parameters", "
    - ``io_type``: Communication type 0-IO 1-UDP
    - ``status``: Control word 0-stopped 1-running
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Get Laser Welding Machine Fault Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetLaserWeldingErrState(self, io_type=1)``"
    "Description", "Get laser welding machine fault status"
    "Required Parameters", "
    - ``io_type``: Communication type 0-IO 1-UDP
    - ``status``: 0-no fault 1-fault present
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Get Laser Welding Machine Configuration Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetLaserWeldingParamTarget(self, num)``"
    "Description", "Get configuration parameters of one of the 10 process groups of the laser welding machine"
    "Required Parameters", "
    - ``Num``: Group number to set (1~10)
    - ``scanSpeed``: Scanning speed
    - ``scanWidth``: Scanning width
    - ``peakPower``: Peak power
    - ``dutyCycle``: Duty cycle
    - ``Freq``: Frequency
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Get Currently Active Configuration Parameters of Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetLaserWeldingParamActual(self, io_type=1)``"
    "Description", "Get currently active configuration parameters of the laser welding machine"
    "Required Parameters", "
    - ``io_type``: Communication type 0-IO 1-UDP
    - ``scanSpeed``: Scanning speed
    - ``scanWidth``: Scanning width
    - ``peakPower``: Peak power
    - ``dutyCycle``: Duty cycle
    - ``Freq``: Frequency
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Configure Extended IO Enable DO Port for Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLaserWeldingEnableExtDoNum(self, ctrlModeDONum)``"
    "Description", "Configure extended IO enable DO port for laser welding machine"
    "Required Parameters", "
    - ``ctrlModeDONum``: Extended DO port number for laser welding machine enable
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Configure Extended IO Start DO Port for Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLaserWeldingStartExtDoNum(self, ctrlModeDONum)``"
    "Description", "Configure extended IO start DO port for laser welding machine"
    "Required Parameters", "
    - ``ctrlModeDONum``: Extended DO port number for laser welding machine start/stop
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Configure Extended IO Fault Reset DO Port for Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLaserWeldingErrResetExtDoNum(self, ctrlModeDONum)``"
    "Description", "Configure extended IO fault reset DO port for laser welding machine"
    "Required Parameters", "
    - ``ctrlModeDONum``: Extended DO port number for laser welding machine fault reset
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Configure Extended IO Running Status (Laser On Status) DI Port for Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLaserWeldingRunningStateExtDiNum(self, diNum)``"
    "Description", "Configure extended IO running status (laser on status) DI port for laser welding machine"
    "Required Parameters", "
    - ``diNum``: Extended DI port number for laser welding machine running status (laser on status)
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Configure Extended IO Fault Status DI Port for Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLaserWeldingErrStateExtDiNum(self, diNum)``"
    "Description", "Configure extended IO fault status DI port for laser welding machine"
    "Required Parameters", "
    - ``diNum``: Extended DI port number for laser welding machine fault status
    "
    "Default Parameters", "None"
    "Return Value", "Error code Success-0 Failure- errcode"

Laser Welding Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos: 

    from time import sleep
    from fairino import Robot
    import time

    # Establish connection with the robot controller, return a robot object upon successful connection
    robot = Robot.RPC('192.168.58.2')

    def testLsaerWeld():
        robot.ExtDevLoadUDPDriver()
        time.sleep(1)

        robot.SetLaserWeldingParam(num = 3, scanSpeed = 2000, scanWidth = 3, peakPower = 1500, dutyCycle = 100, Freq = 1000, io_type=1)
        robot.SetLaserWeldingStartExtDoNum(ctrlModeDONum=1)

        robot.Mode(0)
        time.sleep(1)

        desc_pos1 = [-303.721, -206.960, 297.105, 152.209, 19.857, 109.166]
        desc_pos2 = [-301.575, -254.888, 284.786, 155.919, 26.946, 111.629]
        desc_safe = [-344.386, -280.830, 435.073, 173.835, 15.333, 124.931]
        jointPos1 = [9.827, -99.740, 120.088, -78.900, -77.241, -17.904]
        jointPos2 = [15.251, -96.456, 120.138, -84.664, -68.542, -17.843]
        jointSafe = [19.142, -98.078, 101.493, -83.078, -77.070, -17.794]

        error = robot.MoveL(desc_pos=desc_pos1,joint_pos=jointPos1, tool=1, user=0, vel=100, ovl= 2, acc=100)
        print("MoveL return:", error)
        robot.SetLaserWeldingStartEnd(1, io_type=1, max_waittime=10000)

        error = robot.MoveL(desc_pos=desc_pos2, joint_pos=jointPos2, tool=1, user=0, vel=100, ovl= 2, acc=100)
        print("MoveL return:", error)
        robot.SetLaserWeldingStartEnd(0, io_type=1, max_waittime=10000)

        error = robot.MoveL(desc_pos=desc_safe, joint_pos=jointSafe, tool=1, user=0, vel=100, ovl= 2, acc=100)
        print("MoveL return:", error)

        robot.Mode(1)
        time.sleep(1)

        # Close connection
        robot.CloseRPC()
        time.sleep(1)

    # Call test function
    testLsaerWeld()