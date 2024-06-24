Extended Axis
=========================

.. toctree:: 
    :maxdepth: 5

Set 485 extended axis parameters
+++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoSetParam(servoId,servoCompany,servoModel,servoSoftVersion, servoResolution,axisMechTransRatio)``"
    "Description", "Set 485 extended axis parameters"
    "Required parameter", "- ``servoId``: servo drive ID, range [1-15], corresponding slave ID;
    - ``servoCompany``: Servo drive manufacturer, 1-Dynatec;
    - ``servoModel``: servo drive model, 1-FD100-750C;
    - ``servoSoftVersion``: servo driver software version, 1-V1.0;
    - ``servoResolution``: encoder resolution;
    - ``axisMechTransRatio``: mechanical transmission ratio;"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos:

    from time import sleep
    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    ret = robot = Robot.RPC('192.168.58.2')

    ret =robot.AuxServoSetParam(1,1,1,1,131072,15.45)# Set 485 extended axis parameters
    print("AuxServoSetParam",ret)
    sleep(1)

    ret =robot.AuxServoGetParam(1)# Get 485 extended axis configuration parameters
    print("AuxServoGetParam",ret)
    sleep(1)

    ret =robot.AuxServoGetStatus(1)# Get 485 extended axis servo status
    print("AuxServoGetStatus",ret)
    sleep(1)

    ret =robot.AuxServoClearError(1)# Clear 485 extended axis error message
    print("AuxServoClearError",ret)
    sleep(1)
 
Get 485 extended axis configuration parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoGetParam(servoId)``"
    "Description", "Get 485 extended axis configuration parameters"
    "Required parameter", "- ``servoId``: servo drive ID, range [1-15], corresponding slave ID"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0  Failed -errcode
    - ``servoCompany(return if success)``: Servo drive manufacturer, 1-Dynatec 
    - ``servoModel(return if success)``: servo drive model, 1-FD100-750C 
    - ``servoSoftVersion(return if success)``: servo driver software version, 1-V1.0 
    - ``servoResolution(return if success)``: encoder resolution 
    - ``axisMechTransRatio(return if success)``: mechanical transmission ratio"

Code example
------------

Refer to the code example for "Set 485 extended axis parameters".

Set 485 extended axis enable/disable
+++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoEnable(servoId,status)``"
    "Description", "Set 485 extended axis enable/disable"
    "Required parameter", "- ``servoId``: servo drive ID, range [1-15], corresponding slave ID;
    - ``status``: enable status, 0-disabled, 1-enabled"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------------
.. code-block:: python
    :linenos: 

    from time import sleep
    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    ret = robot = Robot.RPC('192.168.58.2')
    ret =robot.AuxServoEnable(1,0)#You must disable befor modify the control mode 
    print("AuxServoEnable(0)",ret)
    sleep(3)

    ret =robot.AuxServoSetControlMode(1,0)#Set the control mode to pos mode
    print("AuxServoSetControlMode",ret)
    sleep(3)

    ret =robot.AuxServoEnable(1,1)#You must enable after set control mode
    print("AuxServoEnable(1)",ret)
    sleep(3)

    ret =robot.AuxServoHoming(1,1,10,10)#Servo homing
    print("AuxServoHoming",ret)
    sleep(5)

    ret =robot.AuxServoGetStatus(1)#Get Servo status
    print("AuxServoGetStatus",ret)
    sleep(1)
    i=1
    while(i<5):
        ret =robot.AuxServoSetTargetPos(1,300*i,30)#pos mode motion,speed 30
        print("AuxServoSetTargetPos",ret)
        sleep(11)
        ret =robot.AuxServoGetStatus(1)# Get Servo status
        print("AuxServoGetStatus",ret)
        sleep(1)
        i=i+1

Set 485 extended axis control mode
++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoSetControlMode(servoId,mode)``"
    "Description", "Set 485 extended axis control mode"
    "Required parameter", "- ``servoId``: servo drive ID, range [1-15], corresponding slave ID;
    - ``mode``: control mode, 0-position mode, 1-speed mode"
    "Optional parameter", "Nothing" 
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------

Refer to the code example for "Set 485 extended axis enable/disable".

Set the 485 extended axis target position (position mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoSetTargetPos(servoId,pos,speed)``"
    "Description", "Set the 485 extended axis target position (position mode)"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``servoId``: servo drive ID, range [1-15], corresponding slave ID;
    - ``pos``: target position, mm or °;
    - ``speed``: target speed, mm/s or °/s."
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------

Refer to the code example for "Set 485 extended axis enable/disable".

Set the 485 extended axis target speed (speed mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoSetTargetSpeed(servoId,speed)``"
    "Description", "Set the 485 extended axis target speed (speed mode)"
    "Required parameter", "- ``servoId``: servo drive ID, range [1-15], corresponding slave ID;
    - ``speed``: target speed, mm/s or °/s"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos: 

    from time import sleep
    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    ret = robot = Robot.RPC('192.168.58.2')
    ret =robot.AuxServoEnable(1,0)#You must disable befor modify the control mode
    print("AuxServoEnable(0)",ret)
    sleep(3)

    ret = robot.AuxServoSetControlMode(1, 1)  #Set the control mode to speed mode
    print("AuxServoSetControlMode",ret)
    sleep(3)

    ret =robot.AuxServoEnable(1,1) #You must enable after set control mode
    print("AuxServoEnable(1)",ret)
    sleep(3)

    ret =robot.AuxServoHoming(1,1,10,10) #Servo homing
    print("AuxServoHoming",ret)
    sleep(5)

    ret =robot.AuxServoGetStatus(1)# Get 485 extended axis servo status
    print("AuxServoGetStatus",ret)
    sleep(1)

    ret = robot.AuxServoSetTargetSpeed(1, 30)  # speed mode motion,speed 30
    print("AuxServoSetTargetSpeed", ret)
    sleep(10)

    ret = robot.AuxServoGetStatus(1)  # Get 485 extended axis servo status
    print("AuxServoGetStatus", ret)
    sleep(1)

    ret = robot.AuxServoSetTargetSpeed(1, 60)  # speed mode motion,speed 60
    print("AuxServoSetTargetSpeed", ret)
    sleep(10)
    ret = robot.AuxServoGetStatus(1)  # Get 485 extended axis servo status
    print("AuxServoGetStatus", ret)
    sleep(1)

    ret = robot.AuxServoSetTargetSpeed(1, 0)  # before end speed mode motion, you should sete speed to 0
    print("AuxServoSetTargetSpeed", ret)
    sleep(3)
    ret = robot.AuxServoGetStatus(1)  # Get 485 extended axis servo status
    print("AuxServoGetStatus", ret)
    sleep(1)

Set 485 extended axis target torque (torque mode) -- Not open yet
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoSetTargetTorque(servoId,torque)``"
    "Description", "Set 485 extended axis target torque (torque mode)-- Not open yet"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``servoId``: servo drive ID, range [1-15], corresponding slave ID;
    - ``torque``: target torque, Nm"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Set 485 extended axis homing
+++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoHoming(servoId,mode,searchVel,latchVel)``"
    "Description", "Set 485 extended axis homing"
    "Required parameter", "- ``servoId``: servo drive ID, range [1-15], corresponding slave ID
    - ``mode``: homing mode, 0-current position homing; 1-negative limit homing; 2-postive limit homing
    - ``searchVel``: homing speed, mm/s or °/s
    - ``latchVel``: hoop speed, mm/s or °/s"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
--------------

Refer to the code example for "Set 485 extended axis enable/disable".

Clear 485 extended axis error message
+++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoClearError(servoId)``"
    "Description", "Clear 485 extended axis error message"
    "Required parameter", "- ``servoId``: servo drive ID, range [1-15], corresponding slave ID"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
--------------

Refer to the code example for "Set 485 extended axis parameters".

Get 485 extended axis servo status
+++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoGetStatus(servoId)``"
    "Description", "Get 485 extended axis servo status"
    "Required parameter", "- ``servoId``: servo drive ID, range [1-15], corresponding slave ID"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode;
    - ``servoErrCode(return if success)``:  servo drive fault code;
    - ``servoState(return if success)``:  servo drive status [decimal number converted to binary, bit0: 0-disable,1-enable;bit1:0-not running,1-running; bit2: 0-attch postive limit, 1-not touch negative limit; bit3:0-attach negative limit,1-not attach negative limit; bit4:0-not locate,1-loacated; bit5:0-not zero,1-have zero];
    - ``servoPos(return if success)``:  servo current position mm or °;
    - ``servoSpeed(return if success)``:  Servo current speed mm/s or °/s;
    - ``servoTorque(return if success)``:  Servo current torque Nm"

Code example
--------------

Refer to the code example for "Set 485 extended axis enable/disable".

Set the 485 extended axis data axis number in status feedback -- Not open yet
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServosetStatusID(servoId)``"
    "Description", "Set the 485 extended axis data axis number in status feedback-- Not open yet"
    "Required parameter", "- ``servoId``: servo drive ID, range [1-15], corresponding slave ID"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Configure UDP extension axis communication parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtDevSetUDPComParam(ip,port,period,lossPkgTime,lossPkgNum,disconnectTime,reconnectEnable,reconnectPeriod,reconnectNum)``"
    "Description", "Configure UDP extension axis communication parameters"
    "Required parameter", "
    - ``ip``:PLC IP address;
    - ``port``:port number;
    - ``period``: communication period (ms, not open);
    - ``lossPkgTime``:packet loss detection time (ms);
    - ``lossPkgNum``: number of packet loss times;
    - ``disconnectTime``: duration of communication disconnection confirmation;
    - ``reconnectEnable``:automatic reconnection when communication is disconnected Enable 0- Disable 1- Enable;
    - ``reconnectPeriod``:reconnection period interval (ms);
    - ``reconnectNum``:number of reconnections"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #Configure UDP extension axis communication parameters
    error = robot.ExtDevSetUDPComParam('192.168.58.88',2021,2,50,5,50,1,2,5)
    print("ExtDevSetUDPComParam return:",error)
    #Get the UDP extension axis communication parameter configuration
    error = robot.ExtDevGetUDPComParam()
    print("ExtDevGetUDPComParam return:",error)
    
Get the UDP extension axis communication parameter configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtDevGetUDPComParam()``"
    "Description", "Get the UDP extension axis communication parameter configuration"
    "Required parameter", "NULL"
    "Optional parameter", "NULL"
    "Return value", "
    - Errcode: Success -0 , Failed -errcode;
    - ``Return value(return if success)ip``:indicates the IP address of the PLC;
    - ``Return value(return if success)port``:indicates the port number;
    - ``Return value(return if success)period``:Communication period (ms);
    - ``Return value(return if success)lossPkgTime``:indicates the packet loss detection time (ms);
    - ``Return value(return if success)lossPkgNum``: indicates the number of lost packets;
    - ``Return value(return if success)disconnectTime``:indicates the duration of communication disconnection confirmation;
    - ``Return value(return if success)reconnectEnable``: automatic reconnection when communication is disconnected. 0- Disable 1- Enable;
    - ``Return value(return if success)reconnectPeriod``:Reconnection period interval (ms);
    - ``Return value(return if success)reconnectNum``:indicates the number of reconnections"
 
Load the UDP communication connection
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtDevLoadUDPDriver()``"
    "Description", "Load the UDP communication connection"
    "Required parameter", "NULL"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #Load the UDP communication connection
    error = robot.ExtDevLoadUDPDriver()
    print("ExtDevLoadUDPDriver return:",error)
    #Unmount the UDP communication connection
    error = robot.ExtDevUnloadUDPDriver()
    print("ExtDevUnloadUDPDriver return:",error)
     
Unmount the UDP communication connection
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtDevUnloadUDPDriver()``"
    "Description", "Unmount the UDP communication connection"
    "Required parameter", "NULL"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
     
The UDP extension axis communication is disconnected and the connection is restored
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtDevUDPClientComReset()``"
    "Description", "The UDP extension axis communication is disconnected and the connection is restored"
    "Required parameter", "NULL"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
    
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #The UDP extension axis communication is disconnected and the connection is restored
    error = robot.ExtDevUDPClientComReset()
    print("ExtDevUDPClientComReset return:",error)
    #The UDP extension axis communication is disconnected abnormally
    error = robot.ExtDevUDPClientComClose()
    print("ExtDevUDPClientComClose return:",error)
         
The UDP extension axis communication is disconnected abnormally
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtDevUDPClientComClose()``"
    "Description", "The UDP extension axis communication is disconnected abnormally"
    "Required parameter", "NULL"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
         
Set the position of the expansion robot relative to the expansion axis
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotPosToAxis(installType)``"
    "Description", "Set the position of the expansion robot relative to the expansion axis"
    "Required parameter", "- ``installType``:0-The robot is installed on the external axis, 1- the robot is installed outside the external axis;"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
        
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #Set the position of the expansion robot relative to the expansion axis
    error = robot.SetRobotPosToAxis(1)
    print("SetRobotPosToAxis return:",error)
    #Set the extended shaft system DH parameters
    error = robot.SetAxisDHParaConfig(4,128.5,206.4,0,0,0,0,0,0,)
    print("SetAxisDHParaConfig return:",error)
    #Configure UDP extension axis parameters
    error = robot.ExtAxisParamConfig(1,1,0,1000,-1000,1000,1000,1.905,262144, 200,1,1,0)
    print("ExtAxisParamConfig return:",error)

Set the extended shaft system DH parameters
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAxisDHParaConfig(axisConfig,axisDHd1,axisDHd2,axisDHd3,axisDHd4,axisDHa1, axisDHa2,axisDHa3,axisDHa4)``"
    "Description", "Set the extended shaft system DH parameters"
    "Required parameter", "
    - ``axisConfig``:External axis configuration, 0-single DOF linear slide, 1-2 DOF L-type positioner, 2-3 DOF, 3-4 DOF, 4-single DOF positioner;
    - ``axisDHd1``:External axisDH parameter d1 mm;
    - ``axisDHd2``:External axisDH parameter d2 mm;
    - ``axisDHd3``:External axisDH parameter d3 mm;
    - ``axisDHd4``:External axisDH parameter d4 mm;
    - ``axisDHa1``:External axisDH parameter a1 mm;
    - ``axisDHa2``:External axisDH parameter a2 mm;
    - ``axisDHa3``:External axisDH parameter a3 mm;
    - ``axisDHa4``:External axisDH parameter a4 mm;"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Configure UDP extension axis parameters
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAxisDHParaConfig(axisConfig,axisDHd1,axisDHd2,axisDHd3,axisDHd4,axisDHa1, axisDHa2,axisDHa3,axisDHa4)``"
    "Description", "Configure UDP extension axis parameters"
    "Required parameter", "
    - ``axisId``:Axis number[1-4];
    - ``axisType``:Extended axis type 0- translation; 1- rotation;
    - ``axisDirection``:The expansion axis direction is 0-forward; 1-reverse;
    - ``axisMax``:The maximum position of the extension axis is mm;
    - ``axisMin``:Minimum position of the extension axis mm;
    - ``axisVel``:Speed mm/s;
    - ``axisAcc``:Acceleration mm/s2;
    - ``axisLead``:Lead mm;
    - ``encResolution``:Encoder resolution;
    - ``axisOffect``:The start point of the weld extension axis offset;
    - ``axisCompany``:Driver manufacturer 1-Hechuan; 2- Huichuan; 3- Panasonic;
    - ``axisModel``:Driver models 1-Hechuan SV-XD3EA040L-E, 2-Hechuan SV-X2EA150A-A, 1-Huichuan SV620PT5R4I, 1-Matsushita MADLN15SG, 2-Matsushita MSDLN25SG, 3-Matsushita MCDLN35SG;
    - ``axisEncType``:Encoder type 0-increments; 1- absolute value;"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
         
Set the reference point of the extended axis coordinate system - four-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisSetRefPoint(pointNum)``"
    "Description", "Set the reference point of the extended axis coordinate system - four-point method"
    "Required parameter", "- ``pointNum``:Point number[1-4];"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
            
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #Set the reference point of the extended axis coordinate system - four-point method
    error = robot.ExtAxisSetRefPoint(1)
    print("ExtAxisComputeECoordSys(1) return:",error)
             
Calculation of extended axis coordinate system - four-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisComputeECoordSys()``"
    "Description", "Calculation of extended axis coordinate system - four-point method"
    "Required parameter", "NULL"
    "Optional parameter", "NULL"
    "Return value", "
    - Errcode: Success -0 , Failed -errcode;
    - ``Return value(return if success)coord``:Coordinate system value[x,y,z,rx,ry,rz];"
                  
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #Calculation of extended axis coordinate system - four-point method
    error,coord = robot.ExtAxisComputeECoordSys()
    print("ExtAxisComputeECoordSys() return:",error,coord)
    #Apply the extended axis coordinate system
    error = robot.ExtAxisActiveECoordSys(1,1,coord,1)
    print("ExtAxisActiveECoordSys() return:",error)
         
Apply the extended axis coordinate system
++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisActiveECoordSys(axisCoordNum,toolNum,coord,calibFlag)``"
    "Description", "Apply the extended axis coordinate system"
    "Required parameter", "
    - ``axisCoordNum``:coordinate system number;
    - ``toolNum``:tool number;
    - ``coord``:coordinate system value[x,y,z,rx,ry,rz];
    - ``calibFlag``:indicates the calibflag 0- No, 1- yes;"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
             
Set the pose of the calibration reference point in the end coordinate system of the positioner
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRefPointInExAxisEnd(pos)``"
    "Description", "Set the pose of the calibration reference point in the end coordinate system of the positioner"
    "Required parameter", "- ``pos``:Position value[x,y,z,rx,ry,rz];"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
                      
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #Set the pose of the calibration reference point in the end coordinate system of the positioner
    error = robot.SetRefPointInExAxisEnd(desc_pos)
    print("SetRefPointInExAxisEnd(1) return:",error)
                 
Positioner coordinate system reference point setting - four-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``PositionorSetRefPoint(pointNum)``"
    "Description", "Positioner coordinate system reference point setting - four-point method"
    "Required parameter", "- ``pointNum``:Point number[1-4];"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
                          
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #Positioner coordinate system reference point setting - four-point method
    error = robot.SetRefPointInExAxisEnd(desc_pos)
    print("SetRefPointInExAxisEnd(1) return:",error)
                     
Coordinate system calculation of positioner - four-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``PositionorComputeECoordSys()``"
    "Description", "Coordinate system calculation of positioner - four-point method"
    "Required parameter", "NULL"
    "Optional parameter", "NULL"
    "Return value", "
    - Errcode: Success -0 , Failed -errcode;
    - ``Return value(return if success)coord``:Coordinate system value[x,y,z,rx,ry,rz];"
                            
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #Coordinate system calculation of positioner - four-point method
    error,coord = robot.PositionorComputeECoordSys()
    print("PositionorComputeECoordSys() return:",error,coord)
                     
The UDP extension axis is enabled
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisServoOn()``"
    "Description", "The UDP extension axis is enabled"
    "Required parameter", "
    - ``axisID``:Axis number[1-4];
    - ``status``:0- Disables the function. 1- Enable;"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
                                
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #The UDP extension axis is disabled
    error = robot.ExtAxisServoOn(1,0)
    print("ExtAxisServoOn return:",error)
    #The UDP extension axis is enabled
    error = robot.ExtAxisServoOn(1,1)
    print("ExtAxisServoOn return:",error)
    #The UDP extension axis is homed
    error = robot.ExtAxisSetHoming(1,0,40,40)
    print("ExtAxisSetHoming return:",error)
    time.sleep(1)
    #The UDP extension axis starts Jog
    error = robot.ExtAxisStartJog(1,1,20,20,20)
    print("ExtAxisStartJog return:",error)
    time.sleep(1)
    #The UDP extension axis stops Jog
    error = robot.ExtAxisStopJog(1)
    print("ExtAxisStopJog return:",error)

The UDP extension axis is homed
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisSetHoming()``"
    "Description", "The UDP extension axis is homed"
    "Required parameter", "
    - ``axisID``:Axis number[1-4];
    - ``mode``:Zero return mode 0 Current position returns to zero, 1 negative limit returns to zero, 2- positive limit returns to zero searchVel Zero search speed (mm/s);
    - ``latchVel``:zero collar speed (mm/s);"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"

The UDP extension axis starts Jog
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisStartJog( axisID, direction, vel, acc, maxDistance)``"
    "Description", "The UDP extension axis starts Jog"
    "Required parameter", "
    - ``axisID``:Axis number[1-4];
    - ``direction``:Rotation direction 0- reverse; 1-forward;
    - ``vel``:Speed (mm/s);
    - ``acc``:Acceleration (mm/s);
    - ``maxDistance``:indicates the maximum point moving distance;"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"

The UDP extension axis stops Jog
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisStopJog(axisID)``"
    "Description", "The UDP extension axis stops Jog"
    "Required parameter", "- ``axisID``:Axis number[1-4];"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
    
Set extended DO
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAuxDO(DONum,bOpen,smooth,block)``"
    "Description", "Set extended DO"
    "Required parameter", "
    - ``DONum``: DO number;
    - ``bOpen``:switch True- on,False- off;
    - ``smooth``:Indicates whether it is smooth. True - Yes, False - no;
    - ``block``:Whether to block True - yes, False - no;"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
                                    
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #Set extended DO
    error = robot.SetAuxDO(1,True,False,True)
    print("GetAuxAI",error)
    #Set extended AO
    error = robot.SetAuxAO(1,60,True)
    print("SetAuxAO",error)
    #Set the extended DI input filtering time
    error = robot.SetAuxDIFilterTime(10,False)
    print("SetAuxDIFilterTime",error)
    #Set the extended AI input filtering time
    error = robot.SetAuxAIFilterTime(10,True)
    print("SetAuxAIFilterTime",error)
    #Wait for the extended DI input
    error = robot.WaitAuxDI(0,False,100,False)
    print("WaitAuxDI",error)
    #Wait for the extended AI input
    error = robot.WaitAuxAI(0,0,100,500,False)
    print("WaitAuxAI",error)
    #Gets the extended AI value
    error = robot.GetAuxAI(0,False)
    print("GetAuxAI",error)
    #Gets the extended DI value
    error = robot.GetAuxDI(0,True)
    print("GetAuxDI",error)
        
Set extended AO
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAuxAO(self,AONum,value,block)``"
    "Description", "Set extended AO"
    "Required parameter", "
    - ``AONum``: indicates the AO number;
    - ``value``:analog quantity value [0-4095];
    - ``block``:Whether to block True - yes, False - no;"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
        
Set the extended DI input filtering time
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAuxDIFilterTime(filterTime)``"
    "Description", "Set the extended DI input filtering time"
    "Required parameter", "- ``filterTime``: Filter time (ms);"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
        
Set the extended AI input filtering time
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAuxAIFilterTime(AINum,filterTime)``"
    "Description", "Set the extended AI input filtering time"
    "Required parameter", "
    - ``AINum``: AI number;
    - ``filterTime``: Filter time (ms);"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
        
Wait for the extended DI input
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitAuxDI(DINum,bOpen,time,errorAlarm)``"
    "Description", "Wait for the extended DI input"
    "Required parameter", "
    - ``DINum``: indicates the DI number;
    - ``bOpen``:switch True- on,False- off;
    - ``time``:Maximum waiting time (ms);
    - ``errorAlarm``:Whether to continue a motion. True- Yes,False- no"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
        
Wait for the extended AI input
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitAuxAI(,AINum,sign,value,time,errorAlarm)``"
    "Description", "Wait for the extended AI input"
    "Required parameter", "
    - ``AINum``: indicates the AI number;
    - ``sign``: 0- greater than; 1- less than;
    - ``value``:indicates the AI value;
    - ``time``:Maximum waiting time (ms);
    - ``errorAlarm``:Whether to continue a motion. True- Yes,False- no"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode"
        
Gets the extended DI value
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAuxDI(DINum,isNoBlock)``"
    "Description", "Gets the extended DI value"
    "Required parameter", "
    - ``DINum``: indicates the DI number;
    - ``isNoBlock``:Whether to block True- block false- not block;"
    "Optional parameter", "NULL"
    "Return value", "
    - Errcode: Success -0 , Failed -errcode;
    - ``Return value(return if success) isOpen``:0-Open,1-Close;"
          
Gets the extended AI value
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAuxAI(AINum,isNoBlock)``"
    "Description", "Gets the extended AI value"
    "Required parameter", "
    - ``AINum``: indicates the AI number;
    - ``isNoBlock``: Whether to block True- block false- not block"
    "Optional parameter", "NULL"
    "Return value", "
    - Errcode: Success -0 , Failed -errcode;
    - ``Return value(return if success)value``: indicates the input value;"
          
UDP extension axis movement
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisMove(pos,ovl)``"
    "Description", "UDP extension axis movement"
    "Required parameter", "
    - ``pos``:target position Axis 1 position ~ axis 4 position, pos=[exaxis[0], exaxis[1],exaxis[2], exaxis[3]];
    - ``ovl``:Speed percentage"
    "Optional parameter", "NULL"
    "Return value", "Errcode: Success -0 , Failed -errcode;"
                                        
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error,joint_pos = robot.GetActualJointPosDegree()
    print("GetActualJointPosDegree",error,joint_pos)
    e_pos =[-10,0,0,0]
    joint_pos[0] = joint_pos[0]+30
    #UDP extension axis movement
    error = robot.ExtAxisMove(e_pos,30)
    print("ExtAxisMove",error)
    print("joint_pos",joint_pos)
    error = robot.MoveJ(joint_pos,0,0,exaxis_pos=e_pos)
    print("MoveJ",error)
              
The UDP expansion axis moves synchronously with the robot joint movement
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisSyncMoveJ(joint_pos,desc_pos,tool,user,exaxis_pos, vel=20.0, acc=0.0, ovl= 100.0,  blendT=-1.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])``"
    "Description", "The UDP expansion axis moves synchronously with the robot joint movement"
    "Required parameter", "
    - ``joint_pos``:Position of the target joint, unit[°];
    - ``desc_pos``:Target Cartesian pose, unit[mm][°]
    - ``tool``:Tool number,[0~14]
    - ``user``: Job number,[0~14]
    - ``exaxis_pos``:indicates the position from external axis 1 to external axis 4"
    "Optional parameter", "
    - ``vel``:Speed percentage, [0 to 100] 20.0 by default;
    - ``acc``:Acceleration percentage, [0~100] Disabled, default 0.0;
    - ``ovl``:Speed scaling factor, [0~100] Default 100.0;
    - ``blendT``: [1.0] - movement in place (block), [0 ~ 500.0] - smooth time (non-blocking), unit [ms] 1.0 by default;
    - ``offset_flag``:[0]- No offset, [1]- Offset in the job/base coordinate system, [2]- Offset in the tool coordinate system defaults to 0;
    - ``offset_pos``:position offset, the unit (mm) [°] default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];"
    "Return value", "Errcode: Success -0 , Failed -errcode;"
                                        
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robotfrom fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #1.Calibrate and apply the robot tool coordinate system. You can use the four-point method or the six-point method
    # to calibrate and apply the tool coordinate system. The interfaces involved in the calibration of the
    # tool coordinate system are as follows:
    point_num=1
    id=1
    coord=[100,200,300,0,0,0,]
    type=0
    install=0
    #1.Set tool coordinate system
    # robot.SetToolPoint(point_num)  #Set tool reference point - six point method
    # robot.ComputeTool() #Computational tool coordinate system
    # robot.SetTcp4RefPoint()   #Set tool reference point - four point method
    # robot.ComputeTcp4()   #Calculating tool coordinate system - four-point method
    # robot.SetToolCoord(id, coord,type,install)  #Set the application tool coordinate system
    # robot.SetToolList(id, coord,type,install)   #Sets the list of application tool coordinate systems
    #2.Set UDP communication parameters and load UDP communication
    robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
    robot.ExtDevLoadUDPDriver();
    #3.Set the extension shaft parameters, including the extension shaft type, extension shaft drive parameters, and extension shaft DH parameters
    robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0)# Single axis positioner and DH parameters
    robot.SetRobotPosToAxis(1);  #Expansion shaft mounting position
    robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0)# Servo drive parameters, this example is a single-axis positioner, so only one drive parameter needs to be set. If you choose an extension shaft type with multiple axes, you need to set the drive parameters for each axis
    #4.Set the selected axis to enable and homing
    robot.ExtAxisServoOn(1, 0);
    robot.ExtAxisSetHoming(1, 0, 20, 3);
    #5.Carry out calibration and application of extended axis coordinate system
    pos =[0,0,0,0,0,0] #Enter your marker coordinates
    robot.SetRefPointInExAxisEnd(pos)
    robot.PositionorSetRefPoint(1)#You need to calibrate the extension axis through four points in different locations, so you need to call this interface four times to complete the calibration
    error,coord = robot.PositionorComputeECoordSys()#Calculate the calibration results of the extension axis
    robot.ExtAxisActiveECoordSys(1, 1, coord, 1); #The calibration results are applied to the extended axis coordinate system
    method=1
    #6.To calibrate the workpiece coordinate system on the extension axis, you need the following interfaces
    # robot.SetWObjCoordPoint( point_num)
    # error,coord=robot.ComputeWObjCoord( method)
    # robot.SetWObjCoord(id,coord)
    # robot.SetWObjList(id, coord)
    #7.Record the start point of your synchronous joint movement
    startdescPose = [0,0,0,0,0,0]# Enter your coordinates
    startjointPos = [0,0,0,0,0,0]# Enter your coordinates
    startexaxisPos = [0,0,0,0,]# Enter your coordinates
    #8.Record the coordinates of the end point of your synchronous joint movement
    enddescPose = [0,0,0,0,0,0]# Enter your coordinates
    endjointPos = [0,0,0,0,0,0]# Enter your coordinates
    endexaxisPos = [0,0,0,0,]# Enter your coordinates
    #9.Write synchronous motion program
    #Move to the starting point, assuming that the tool coordinate system and the workpiece coordinate system are both 1
    robot.ExtAxisMove(startexaxisPos, 20);
    robot.MoveJ(startjointPos,  1, 1, desc_pos=startdescPose,exaxis_pos=startexaxisPos);
    #Start synchronized motion
    robot.ExtAxisSyncMoveJ(endjointPos, enddescPose, 1, 1, endexaxisPos);
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    robot.Mode(0)
    time.sleep(1)
    e_pos =[-20,0,0,0]
    joint_pos0 = [114.089,-85.740, 119.106,-129.884,-91.655, 79.642]
    desc_pos0= [-87.920,-178.539,-64.513,-175.471,7.664,139.650]
    # The UDP expansion axis moves synchronously with the robot joint movement
    error = robot.ExtAxisSyncMoveJ(joint_pos0,desc_pos0,1,1,e_pos)
    print("ExtAxisSyncMoveJ",error)
                  
The UDP extension axis moves synchronously with the robot’s linear motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisSyncMoveL(self, joint_pos,desc_pos, tool, user,exaxis_pos, vel=20.0, acc=0.0, ovl=100.0, blendR=-1.0, search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])``"
    "Description", " The UDP extension axis moves synchronously with the robot’s linear motion"
    "Required parameter", "
    - ``joint_pos``:  Position of the target joint, unit[°];
    - ``desc_pos``:Target Cartesian pose, unit[mm][°];
    - ``tool``:Tool number,[0~14];
    - ``user``:Job numbe,[0~14];
    - ``exaxis_pos``:indicates the position from external axis 1 to external axis 4;"
    "Optional parameter", "
    - ``vel``: Speed percentage, [0 to 100] 20.0 by default;
    - ``acc``:Acceleration percentage, 0 to 100. The default value is 0.0;
    - ``ovl``:Speed scaling factor, [0~100] Default 100.0;
    - ``blendR``:[1.0] - movement in place (block), [0 ~ 1000] - smooth radius (non-blocking), unit (mm) 1.0 by default;
    - ``search``:[0]- No wire search, [1]- wire search;
    - ``offset_flag``:[0]- No offset, [1]- Offset in the job/base coordinate system, [2]- Offset in the tool coordinate system defaults to 0;
    - ``offset_pos``:position offset, the unit (mm) [°] default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];"
    "Return value", "Errcode: Success -0 , Failed -errcode;"
                                            
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    robot.Mode(0)
    time.sleep(1)
    e_pos =[-20,0,0,0]
    joint_pos0 = [114.089,-85.740, 119.106,-129.884,-91.655, 79.642]
    desc_pos0= [-87.920,-178.539,-64.513,-175.471,7.664,139.650]
    #The UDP extension axis moves synchronously with the robot’s linear motion
    error = robot.ExtAxisSyncMoveL(joint_pos0,desc_pos0,1,1,e_pos)
    print("ExtAxisSyncMoveL",error)
                      
The UDP extension axis moves synchronously with the robot arc motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisSyncMoveC(joint_pos_p, desc_pos_p, tool_p, user_p,exaxis_pos_p, joint_pos_t, desc_pos_t, tool_t, user_t,exaxis_pos_t,vel_p=20.0, acc_p=100.0, offset_flag_p=0, offset_pos_p =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel_t=20.0, acc_t=100.0, offset_flag_t=0, offset_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], ovl=100.0, blendR=-1.0)``"
    "Description", " The UDP extension axis moves synchronously with the robot arc motion"
    "Required parameter", "
    - ``joint_pos_p``: joint position of a waypoint, unit [°];
    - ``desc_pos_p``:waypoint Cartesian pose, unit [mm][°];
    - ``tool_p``:path point tool number, [0~14];
    - ``user_p``:waypoint job number, [0~14];
    - ``exaxis_pos_p``:Pathpoint external axis 1 position to external axis 4 position default [0.0,0.0,0.0,0.0];
    - ``joint_pos_t``: joint position of the target point, unit [°];
    - ``desc_pos_t``:cartesian position of the target point, unit [mm][°];
    - ``tool_t``:tool number, [0~14];
    - ``user_t``:job number, [0~14];
    - ``exaxis_pos_t``:target point external axis 1 position to external axis 4 position default [0.0,0.0,0.0,0.0] ;"
    "Optional parameter", "
    - ``vel_p``: percentage of waypoint speed. The value ranges from 0 to 100. The default value is 20.0;
    - ``acc_p``: percentage of waypoint acceleration (0 to 100) Disabled. The default value is 0.0;   
    - ``offset_flag_p``: Whether the waypoint is offset [0]- not offset, [1]- offset in the job/base coordinate system, [2]- offset in the tool coordinate system by default 0;
    - ``offset_pos_p``:path to the position offset of point, the unit (mm) [°] default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    - ``vel_t``:percentage of target speed. The value ranges from 0 to 100. The default value is 20.0;
    - ``acc_t``:percentage of target acceleration. The value ranges from 0 to 100. The default value is 0.0;
    - ``offset_flag_t``:Whether the target point is offset [0]- No offset, [1]- offset in the job/base coordinate system, [2]- offset in the tool coordinate system by default 0;
    - ``offset_pos_t``:  the target position offset of point, the unit (mm) [°] default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    - ``ovl``:Speed scaling factor, [0~100] Default 100.0;
    - ``blendR``:[1.0] - movement in place (block), [0 ~ 1000] - smooth radius (non-blocking), unit (mm) 1.0 by default;"
    "Return value", "Errcode: Success -0 , Failed -errcode;"
                                                
Code example
------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    #  A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    robot.Mode(0)
    time.sleep(1)
    desc_pos_mid =[-131.2748107910156, -60.21242523193359, -22.55266761779785, 175.9907989501953, 5.92541742324829, 145.5211791992187]
    desc_pos_end =[-91.3530502319336, -174.5040588378906, -64.93866729736328, 177.1370544433593, 15.96347618103027, 136.1746368408203]
    joint_pos_mid = [120.9549040841584, -109.8869943146658, 134.1448068146658, -126.2150709699876, -88.6738087871287, 79.6419593131188]
    joint_pos_end =[110.1896078279703, -89.01601659189356, 125.5532806698638, -139.7967831451114, -82.93198387221534, 79.6452225788985]
    # #The UDP extension axis moves synchronously with the robot arc motion
    time.sleep(3)
    error = robot.ExtAxisSyncMoveC(joint_pos_mid,desc_pos_mid,1,1,[-10,0,0,0],joint_pos_end,desc_pos_end,1,1,[-20,0,0,0])
    print("ExtAxisSyncMoveC",error)