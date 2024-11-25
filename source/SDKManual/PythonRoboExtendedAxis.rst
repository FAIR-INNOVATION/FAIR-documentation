Extended Axis
==========================

.. toctree::
    :maxdepth: 5

Setting the 485 Extended Axis Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoSetParam(servoId,servoCompany,servoModel,servoSoftVersion, servoResolution,axisMechTransRatio)``"
    "Description", "Setting the 485 extended axis parameters"
    "Mandatory parameters","- ``servoId``: servo drive ID, range [1-15], corresponds to slave ID;
    - ``servoCompany``: servo drive manufacturer, 1 - Dynatec;
    - ``servoModel``: Servo drive model, 1-FD100-750C;
    - ``servoSoftVersion``: servo drive software version, 1-V1.0;
    - ``servoResolution``: encoder resolution;
    - ``axisMechTransRatio``: mechanical transmission ratio;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
-------------

.. code-block:: python
    :linenos:

    from time import sleep
    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    ret = robot = Robot.RPC('192.168.58.2')

    ret = robot.AuxServoSetParam(1,1,1,1,1,131072,15.45)#Set 485 extended axis parameter
    print("AuxServoSetParam",ret)
    sleep(1)

    ret = robot.AuxServoGetParam(1)#Get 485 extended axis configuration parameter
    print("AuxServoGetParam",ret)
    sleep(1)

    ret = robot.AuxServoGetStatus(1)#query status
    print("AuxServoGetStatus",ret)
    sleep(1)

    ret = robot.AuxServoClearError(1)#clear error
    print("AuxServoClearError",ret)
    sleep(1)

Getting 485 Expansion Axis Configuration Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoGetParam(servoId)``"
    "Description", "Get 485 extended axis configuration parameters"
    "Mandatory parameters", "- ``servoId``: servo drive ID, range [1-15], corresponding to slave ID;"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode.
    - ``servoCompany``: servo drive manufacturer, 1 - Dynatec;
    - ``servoModel``: servo drive model, 1-FD100-750C;
    - ``servoSoftVersion``: servo drive software version, 1-V1.0;
    - ``servoResolution``: encoder resolution;
    - ``axisMechTransRatio``: mechanical transmission ratio;"

code example
--------------------------------------------
Refer to the code example for setting the 485 extended axis parameters

Setting the 485 expansion axis enable/disable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoEnable(servoId,status)``"
    "Description", "Setting the 485 extension axis to enable/de-enable"
    "Mandatory parameters","- ``servoId``: servo drive ID, range [1-15], corresponds to slave ID;
    - ``status``: enabling status, 0-de-enabling, 1-enabling;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------------

.. code-block:: python
    :linenos:

    from time import sleep
    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    ret = robot = Robot.RPC('192.168.58.2')
    ret =robot.AuxServoEnable(1,0)#Enable before modifying control mode.
    print("AuxServoEnable(0)",ret)
    sleep(3)

    ret =robot.AuxServoSetControlMode(1,0)#set to position mode
    print("AuxServoSetControlMode",ret)
    sleep(3)

    ret =robot.AuxServoEnable(1,1)#Enable after modifying control mode.
    print("AuxServoEnable(1)",ret)
    sleep(3)

    ret = robot.AuxServoHoming(1,1,10,10)# return to zero
    print("AuxServoHoming",ret)
    sleep(5)

    ret = robot.AuxServoGetStatus(1)#query status
    print("AuxServoGetStatus",ret)
    sleep(1)
    i=1
    while(i<5):
        ret =robot.AuxServoSetTargetPos(1,300*i,30)#position mode motion, speed 30
        print("AuxServoSetTargetPos",ret)
        sleep(11)
        ret = robot.AuxServoGetStatus(1)#query status
        print("AuxServoGetStatus",ret)
        sleep(1)
        i=i+1

Setting the 485 Extended Axis Control Mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoSetControlMode(servoId,mode)``"
    "Description", "Setting the 485 Extended Axis Control Mode"
    "Mandatory parameters","- ``servoId``: servo drive ID, range [1-15], corresponds to slave ID;
    - ``mode``: control mode, 0-position mode, 1-velocity mode;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------
Refer to the code example for setting 485 extension axis enable/disable

Setting the 485 extended axis target position (position mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoSetTargetPos(servoId,pos,speed)``"
    "Description", "Setting the 485 extended axis target position (position mode)"
    "Mandatory parameters","- ``servoId``: servo drive ID, range [1-15], corresponds to slave ID;
    - ``pos``: target position, mm or °;
    - ``speed``: target speed, mm/s or °/s;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
---------------------------------
Refer to the code example for setting 485 extension axis enable/disable

Setting the 485 extended axis target speed (velocity mode)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoSetTargetSpeed(servoId,speed)``"
    "Description", "Setting the 485 extended axis target speed (velocity mode)"
    "Mandatory parameters","- ``servoId``: servo drive ID, range [1-15], corresponds to slave ID;
    - ``speed``: target speed, mm/s or °/s;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
----------------------------------------------------

.. code-block:: python
    :linenos:

    from time import sleep
    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    ret = robot = Robot.RPC('192.168.58.2')
    ret =robot.AuxServoEnable(1,0)#Enable before modifying control mode.
    print("AuxServoEnable(0)",ret)
    sleep(3)

    ret = robot.AuxServoSetControlMode(1, 1) # set to speed mode
    print("AuxServoSetControlMode",ret)
    sleep(3)

    ret =robot.AuxServoEnable(1,1)#Enable after modifying control mode.
    print("AuxServoEnable(1)",ret)
    sleep(3)

    ret = robot.AuxServoHoming(1,1,10,10)# return to zero
    print("AuxServoHoming",ret)
    sleep(5)

    ret = robot.AuxServoGetStatus(1)#query status
    print("AuxServoGetStatus",ret)
    sleep(1)

    ret = robot.AuxServoSetTargetSpeed(1, 30) # speed mode motion, speed 30
    print("AuxServoSetTargetSpeed", ret)
    sleep(10)

    ret = robot.AuxServoGetStatus(1) # query status
    print("AuxServoGetStatus", ret)
    sleep(1)

    ret = robot.AuxServoSetTargetSpeed(1, 60) # Speed mode motion, speed 60
    print("AuxServoSetTargetSpeed", ret)
    sleep(10)
    ret = robot.AuxServoGetStatus(1) # query status
    print("AuxServoGetStatus", ret)
    sleep(1)

    ret = robot.AuxServoSetTargetSpeed(1, 0) # speed should be set to 0 before ending speed mode movement
    print("AuxServoSetTargetSpeed", ret)
    sleep(3)
    ret = robot.AuxServoGetStatus(1) # query status
    print("AuxServoGetStatus", ret)
    sleep(1)

Setting the 485 extended axis target torque (torque mode)-not yet available
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``AuxServoSetTargetTorque(servoId,torque)``"
    "Description", "Setting the 485 extended axis target torque (torque mode)"
    "Mandatory parameters","- ``servoId``: servo drive ID, range [1-15], corresponds to slave ID;
    - ``torque``: target moment, Nm;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the 485 extended axis back to zero
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoHoming(servoId,mode,searchVel,latchVel)``"
    "Description", "Setting the 485 extension axis back to zero"
    "Mandatory parameters","- ``servoId``: servo drive ID, range [1-15], corresponds to slave ID;
    - ``mode``: return to zero mode, 1 - return to zero at current position; 2 - return to zero at negative limit; 3 - return to zero at positive limit.
    - ``searchVel``: return-to-zero velocity, mm/s or °/s.
    - ``latchVel``: hoop speed, mm/s or °/s;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
    
code example
---------------------------------
Refer to the code example for setting 485 extension axis enable/disable

Clearing 485 Expansion Axis Error Messages
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoClearError(servoId)``"
    "Description", "Clearing 485 Extended Axis Error Message"
    "Mandatory parameters", "- ``servoId``: servo drive ID, range [1-15], corresponding to slave ID;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
    
code example
---------------------------------
Refer to the code example for setting the 485 extended axis parameters

Get 485 extended axis servo status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``AuxServoGetStatus(servoId)``"
    "Description", "Get 485 extended axis servo status"
    "Mandatory parameters", "- ``servoId``: servo drive ID, range [1-15], corresponding to slave ID;"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode.
    - ``servoErrCode``: servo drive error code
    - ``servoState``: servo drive state bit0:0-not enabled; 1-enabled; bit1:0-not in motion; 1-being in motion; bit2 0-positive limit not triggered; 1-positive limit triggered; bit3 0-negative limit not triggered; 1-negative limit triggered; bit4 0-positioning not completed; 1-positioning complete; bit5: 0-not zero return; 1 -zero return complete;
    - ``servoPos``: servo current position mm or °;
    - ``servoSpeed``: servo current speed mm/s or °/s;
    - ``servoTorque``: servo current torque Nm;"

code example
---------------------------------
Refer to the code example for setting 485 extension axis enable/disable

Setting the 485 extended axis data axis number in the status feedback - not open yet
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServosetStatusID(servoId)``"
    "Description", "Sets the 485 extended axis data axis number in status feedback"
    "Mandatory parameters", "- ``servoId``: servo drive ID, range [1-15], corresponding to slave ID;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the 485 Extended Axis Motion Acceleration and Deceleration Speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoSetAcc(acc, dec)``"
    "Description", "Sets the 485 extended axis motion acceleration and deceleration speeds."
    "Mandatory parameters","- ``acc``: 485 extended axis motion acceleration
    - ``dec``: 485 extended axis motion deceleration"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the 485 extended axis emergency stop acceleration and deceleration speeds
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoSetEmergencyStopAcc(acc, dec)``"
    "Description", "Setting the 485 extended axis emergency stop acceleration and deceleration speed"
    "Mandatory parameters","- ``acc``: 485 extended axis emergency stop acceleration
    - ``dec``: 485 extended axis emergency stop deceleration"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Get 485 extended axis emergency stop acceleration and deceleration speeds
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoGetEmergencyStopAcc()``"
    "Description", "Get 485 extended axis emergency stop acceleration and deceleration speed"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``acc``: 485 extended axis emergency stop acceleration
    - ``dec``: 485 extended axis emergency stop deceleration"

Get 485 Extended Axis Motion Acceleration and Deceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AuxServoGetAcc()``"
    "Description", "Get 485 extended axis motion plus or minus velocity"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``acc``: 485 extended axis motion acceleration
    - ``dec``: 485 extended axis motion deceleration"

Parameter configuration for UDP extended axis communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``ExtDevSetUDPComParam(ip, port, period, lossPkgTime, lossPkgNum, disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum)``"
    "Description", "UDP Extended Axis Communication Parameter Configuration"
    "Mandatory parameters", "
    - ``ip``: PLC IP address;
    - ``port``: port number;
    - ``period``: communication period (ms, not open yet);
    - ``lossPkgTime``: packet loss detection time (ms);
    - ``lossPkgNum``: number of packets lost;
    - ``disconnectTime``: the length of the communication disconnect confirmation;
    - ``reconnectEnable``: communication disconnection auto reconnect enable 0-not enable 1-enable;
    - ``reconnectPeriod``: reconnect period interval (ms);
    - ``reconnectNum``: number of reconnections"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # UDP extended axis communication parameter configuration
    error = robot.ExtDevSetUDPComParam('192.168.58.88',2021,2,50,5,50,1,2,5)
    print("ExtDevSetUDPComParam return:",error)
    # UDP extended axis communication parameter configuration
    error = robot.ExtDevGetUDPComParam()
    print("ExtDevGetUDPComParam return:",error)
    
Get UDP extended axis communication parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtDevGetUDPComParam()``"
    "Description", "Get UDP extended axis communication parameters"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "
    - Error Code Success-0 Failure- errcode;
    - ``ip``: PLC IP address;
    - ``port``: port number;
    - ``period``: communication period (ms, not open yet);
    - ``lossPkgTime``: packet loss detection time (ms);
    - ``lossPkgNum``: number of packets lost;
    - ``disconnectTime``: the length of the communication disconnect confirmation;
    - ``reconnectEnable``: communication disconnection auto reconnect enable 0-not enable 1-enable;
    - ``reconnectPeriod``: reconnect period interval (ms);
    - ``reconnectNum``: number of reconnections"
 
Load UDP communication
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtDevLoadUDPDriver()``"
    "Description", "Load UDP communication"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    #Load UDP communications
    error = robot.ExtDevLoadUDPDriver()
    print("ExtDevLoadUDPDriver return:",error)
    # Offload UDP communications
    error = robot.ExtDevUnloadUDPDriver()
    print("ExtDevUnloadUDPDriver return:",error)
     
Offloading UDP communication
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtDevUnloadUDPDriver()``"
    "Description", "Offloading UDP communication"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
     
UDP Extended Axis Communication Recovery after Abnormal Disconnection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``ExtDevUDPClientComReset()``"
    "Description", "UDP Extended Axis Communication Abnormally Disconnected After Restoring Connection"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
    
code example
------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    #UDP Extended Axis communication restores connection after abnormal disconnection
    error = robot.ExtDevUDPClientComReset()
    print("ExtDevUDPClientComReset return:",error)
    # UDP Extended Axis communication shut down after abnormal disconnections
    error = robot.ExtDevUDPClientComClose()
    print("ExtDevUDPClientComClose return:",error)
         
UDP extension axis communication is closed after abnormal disconnection.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtDevUDPClientComClose()``"
    "Description", "UDP Extended Axis Communication Abnormal Disconnect Closes Communication"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
         
Setting the extended robot position relative to the extended axis
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotPosToAxis(installType)``"
    "Description", "Set the position of the extended robot relative to the extended axis"
    "Mandatory parameters", "- ``installType``: 0 - robot mounted on external axis, 1 - robot mounted outside external axis;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
        
code example
------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Setting the extended robot position relative to the extended axis
    error = robot.SetRobotPosToAxis(1)
    print("SetRobotPosToAxis return:",error)
    #Setup extended axis system DH parameter configuration
    error = robot.SetAxisDHParaConfig(4,128.5,206.4,0,0,0,0,0,0,)
    print("SetAxisDHParaConfig return:",error)
    #UDP Extended Axis Parameter Configuration
    error = robot.ExtAxisParamConfig(1,1,0,1000,-1000,1000,1000,1000,1.905,262144, 200,1,1,0)
    print("ExtAxisParamConfig return:",error)

Setting the extended axis system DH parameter configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAxisDHParaConfig(axisConfig,axisDHd1,axisDHd2,axisDHd3,axisDHd4,axisDHa1,axisDHa2,axisDHa3,axisDHa4)``"
    "Description", "Sets the extended axis system DH parameter configuration"
    "Mandatory parameters", "
    - ``axisConfig``: external axis configuration, 0 - single degree of freedom linear slide, 1 - two degree of freedom L-type indexer, 2 - three degree of freedom, 3 - four degree of freedom, 4 - single degree of freedom indexer;
    - ``axisDHd1``: external axis DH parameter d1 mm;
    - ``axisDHd2``: external axis DH parameter d2 mm;
    - ``axisDHd3``: external axis DH parameter d3 mm;
    - ``axisDHd4``: external axis DH parameter d4 mm;
    - ``axisDHa1``: external axis DH parameter a1 mm;
    - ``axisDHa2``: external axis DH parameter a2 mm;
    - ``axisDHa3``: external axis DH parameter a3 mm;
    - ``axisDHa4``: external axis DH parameter a4 mm;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

UDP Extended Axis Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisParamConfig(axisId, axisType, axisDirection, axisMax, axisMin, axisVel, axisAcc,axisLead, encResolution, axisOffect. axisCompany, axisModel, axisEncType)``"
    "Description", "UDP Extended Axis Parameter Configuration"
    "Mandatory parameters", "
    - ``axisId``: axis number [1-4];
    - ``axisType``: extended axis type 0 - translation; 1 - rotation;
    - ``axisDirection``: extended axis direction 0 - forward; 1 - reverse;
    - ``axisMax``: Maximum position of the extended axis in mm;
    - ``axisMin``: Extended axis minimum position in mm;
    - ``axisVel``: speed mm/s;
    - ``axisAcc``: acceleration mm/s2;
    - ``axisLead``: lead in mm;
    - ``encResolution``: encoder resolution;
    - ``axisOffect``: Extended axis offset from the start of the weld;
    - ``axisCompany``: drive manufacturers 1-Hochuan; 2-HuiChuan; 3-Panasonic;
    - ``axisModel``: Drive Model 1-Hochuan-SV-XD3EA040L-E, 2-Hochuan-SV-X2EA150A-A, 1-HuiChuan-SV620PT5R4I, 1-Panasonic-MADLN15SG, 2-Panasonic-MSDLN25SG, 3-Panasonic-MCDLN35SG;
    - ``axisEncType``: encoder type 0-incremental; 1-absolute;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
         
Setting the reference point of the extended axis coordinate system - four-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisSetRefPoint(pointNum)``"
    "Description", "Setting the reference point of the extended axis coordinate system - four-point method"
    "Mandatory parameters", "- ``pointNum``: point number [1-4];"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
            
code example
------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Setting the reference point of the extended axis coordinate system - four-point method
    error = robot.ExtAxisSetRefPoint(1)
    print("ExtAxisComputeECoordSys(1) return:",error)
             
Calculating the Extended Axis Coordinate System - Four Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisComputeECoordSys()``"
    "Description", "Calculating Extended Axis Coordinate Systems - Four Point Method"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode.
    - ``coord``: extended axis coordinate system values [x,y,z,rx,ry,rz];"
                  
code example
------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Calculate the extended axis coordinate system - four point method
    error,coord = robot.ExtAxisComputeECoordSys()
    print("ExtAxisComputeECoordSys() return:",error,coord)
    # Apply extended axis coordinate system
    error = robot.ExtAxisActiveECoordSys(1,1,coord,1)
    print("ExtAxisActiveECoordSys() return:",error)
         
Applying the Extended Axis Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisActiveECoordSys(applyAxisId,axisCoordNum,coord,calibFlag)``"
    "Description", "Apply extended axis coordinate system"
    "Mandatory parameters", "
    - ``applyAxisId``:Extended Axis Numbering bit0-bit3 corresponds to extended axis numbering 1-4, e.g., if you apply extended axes 1 and 3, it would be 0b 0000 0101, or 5;
    - ``axisCoordNum``: extended axis coordinate system number;
    - ``coord``: coordinate system value [x,y,z,rx,ry,rz];
    - ``calibFlag``: calibration flag 0 - no, 1 - yes;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
             
Setting of the calibration reference point in the position in the coordinate system of the end of the translator
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetRefPointInExAxisEnd(pos)``"
    "Description", "Set the calibration reference point to be positioned in the coordinate system of the end of the variator"
    "Mandatory parameters", "- ``pos``: bit position values [x,y,z,rx,ry,rz];"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
                      
code example
------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    #Set the calibration reference point in the position in the coordinate system of the end of the translator
    error = robot.SetRefPointInExAxisEnd(desc_pos)
    print("SetRefPointInExAxisEnd(1) return:",error)
                 
Reference Point Setting for the Shifter Coordinate System - Four-Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``PositionorSetRefPoint(pointNum)``"
    "Description", "Reference Point Setting for the Variable Position Machine Coordinate System - Four Point Method"
    "Mandatory parameters", "- ``pointNum``: point number [1-4];"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
                          
code example
------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Shifter coordinate system reference point setting - four-point method
    error = robot.SetRefPointInExAxisEnd(desc_pos)
    print("SetRefPointInExAxisEnd(1) return:",error)
                     
Shifter Coordinate System Calculation - Four Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``PositionorComputeECoordSys()``"
    "Description", "Translator Coordinate System Calculation - Four Point Method"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode.
    - ``coord``: the value of the coordinate system of the translocator [x,y,z,rx,ry,rz];"
                            
code example
------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Shifter Coordinate System Calculation - Four Point Method
    error,coord = robot.PositionorComputeECoordSys()
    print("PositionorComputeECoordSys() return:",error,coord)
        
End Sensor Register Write
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AxleSensorRegWrite(devAddr, regHAddr, regLAddr, regNum, data1, data2, isNoBlock)``"
    "Description", "End Sensor Register Write"
    "Mandatory parameters", "
    - ``devAddr``: device address number 0-255
    - ``regHAddr``: register address high 8 bits
    - ``regLAddr``: register address lower 8 bits
    - ``regNum``: number of registers 0-255
    - ``data1``: write to register value 1
    - ``data2``: write register value 2
    - ``isNoBlock``: 0 - blocking; 1 - non-blocking
    "
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode;"
                          
UDP Extended Axis Enable
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisServoOn(axisID, status)``"
    "description", "UDP Extended Axis Enable"
    "Mandatory parameters", "- ``axisID``: axis number [1-4];
    - ``status``: 0-de-enable; 1-enable;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
                                
code example
------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    #UDP extension axis de-enabling
    error = robot.ExtAxisServoOn(1,0)
    print("ExtAxisServoOn return:",error)
    #UDP Extended Axis Enable
    error = robot.ExtAxisServoOn(1,1)
    print("ExtAxisServoOn return:",error)
    #UDP extended axis back to zero
    error = robot.ExtAxisSetHoming(1,0,40,40)
    print("ExtAxisSetHoming return:",error)
    time.sleep(1)
    #UDP extended axis pointing start
    error = robot.ExtAxisStartJog(1,1,20,20,20)
    print("ExtAxisStartJog return:",error)
    time.sleep(1)
    #UDP Extended Axis Tap Stop
    error = robot.ExtAxisStopJog(1)
    print("ExtAxisStopJog return:",error)

UDP Extended Axis Zero Return
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisSetHoming(axisID, mode, searchVel, latchVel)``"
    "Description", "UDP extended axis back to zero"
    "Mandatory parameters", "
    - ``axisID``: axis number [1-4];
    - ``mode``: return to zero mode 0 current position return to zero, 1 negative limit return to zero, 2 - positive limit return to zero;
    - ``searchVel``: search velocity (mm/s);
    - ``latchVel``: zeroing hoop speed (mm/s);"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

UDP Extended Axis Tap Start
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisStartJog( axisID, direction, vel, acc, maxDistance)``"
    "Description", "UDP Extended Axis Tap Start"
    "Mandatory parameters", "
    - ``axisID``: axis number [1-4];
    - ``direction``: direction of rotation 0 - reverse; 1 - forward;
    - ``vel``: velocity (mm/s);
    - ``acc``: acceleration (mm/s);
    - ``maxDistance``: maximum pointing distance;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

UDP Extended Axis Tap Stop
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisStopJog(axisID)``"
    "Description", "UDP Extended Axis Tap Stop"
    "Mandatory parameters", "- ``axisID``: axis number [1-4];"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
    
Setting the Extended DO
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAuxDO(DONum,bOpen,smooth,block)``"
    "Description", "Setting the Extended DO"
    "Mandatory parameters", "
    - ``DONum``: DO number;
    - ``bOpen``: switch True-Open, False-Off;
    - ``smooth``: smooth or not True - yes, False - no;
    - ``block``: whether to block True - Yes, False - No;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
                                    
code example
------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    #Set up extended DO
    error = robot.SetAuxDO(1,True,False,True)
    print("GetAuxAI",error)
    #Setup Extended AO
    error = robot.SetAuxAO(1,60,True)
    print("SetAuxAO",error)
    # Set the extended DI input filter time
    error = robot.SetAuxDIFilterTime(10,False)
    print("SetAuxDIFilterTime",error)
    # Set the extended AI input filter time
    error = robot.SetAuxAIFilterTime(10,True)
    print("SetAuxAIFilterTime",error)
    #Waiting for extended DI input
    error = robot.WaitAuxDI(0,False,100,False)
    print("WaitAuxDI",error)
    #Waiting for extended AI input
    error = robot.WaitAuxAI(0,0,100,500,False)
    print("WaitAuxAI",error)
    # Get the extended AI value
    error = robot.GetAuxAI(0,False)
    print("GetAuxAI",error)
    # Get extended DI values
    error = robot.GetAuxDI(0,True)
    print("GetAuxDI",error)
        
Setting up Extended AO
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAuxAO(AONum,value,block)``"
    "Description", "Setting the Extended AO"
    "Mandatory parameters", "
    - ``AONum``: AO number;
    - ``value``: analog value [0-4095];
    - ``block``: whether to block True - Yes, False - No;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
        
Setting the Extended DI Input Filter Time
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAuxDIFilterTime(filterTime)``"
    "Description", "Setting the Extended DI Input Filter Time"
    "Mandatory parameters", "- ``filterTime``: filter time (ms);"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
        
Setting the Extended AI Input Filter Time
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAuxAIFilterTime(AINum,filterTime)``"
    "Description", "Set the extended AI input filter time"
    "Mandatory parameters", "
    - ``AINum``: AI number;
    - ``filterTime``: filter time (ms);"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
        
Waiting for extended DI input
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitAuxDI(DINum,bOpen,time,errorAlarm)``"
    "Description", "Waiting for extended DI input"
    "Mandatory parameters", "
    - ``DINum``: DI number;
    - ``bOpen``: switch True-Open, False-Off;
    - ``time``: maximum waiting time (ms);
    - ``errorAlarm``: whether to continue the campaign True-Yes,False-No"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
        
Waiting for extended AI input
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitAuxAI(,AINum,sign,value,time,errorAlarm)``"
    "Description", "Waiting for extended AI input"
    "Mandatory parameters", "
    - ``AINum``: AI number;
    - ``sign``: 0 - greater than; 1 - less than;
    - ``value``: AI value;
    - ``time``: maximum waiting time (ms);
    - ``errorAlarm``: whether to continue the campaign True-Yes,False-No"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
        
Get Extended DI Value
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAuxDI(DINum,isNoBlock)``"
    "Description", "Get Extended DI Value"
    "Mandatory parameters", "
    - ``DINum``: DI number;
    - ``isNoBlock``: whether to block True-blocking false-non-blocking;"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode;
    - ``isOpen``: 0 - off; 1 - on;"
          
Get Extended AI Value
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAuxAI(AINum,isNoBlock)``"
    "Description", "Get Extended AI Value"
    "Mandatory parameters", "
    - ``AINum``: AI number;
    - ``isNoBlock``: whether to block True-blocking False-non-blocking"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode;
    - ``value``: input value;"
          
UDP Extended Axis Motion
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisMove(pos,ovl)``"
    "Description", "UDP Extended Axis Motion"
    "Mandatory parameters", "- ``pos=[exaxis[0],exaxis[1],exaxis[2],exaxis[3]]``: target position Axis 1 position to Axis 4 position;
    - ``ovl``: percentage of speed"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
                                        
code example
------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error,joint_pos = robot.GetActualJointPosDegree()
    print("GetActualJointPosDegree",error,joint_pos)
    e_pos = [-10,0,0,0]
    joint_pos[0] = joint_pos[0]+30
    #UDP Extended Axis Asynchronous Motion
    error = robot.ExtAxisMove(e_pos,30)
    print("ExtAxisMove",error)
    print("joint_pos",joint_pos)
    error = robot.MoveJ(joint_pos,0,0,exaxis_pos=e_pos)
    print("MoveJ",error)
              
UDP extension axes synchronized with robot joint motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype","``ExtAxisSyncMoveJ(joint_pos,desc_pos,tool,user,exaxis_pos, vel=20.0, acc=0.0, ovl= 100.0, blendT=-1.0, offset_flag=0, offset_pos=[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])``"
    "Description", "UDP extension axis synchronized motion with robot joint motion"
    "Mandatory parameters", "
    - ``joint_pos``: target joint position in [°];
    - ``desc_pos``: Cartesian position of the target in [mm][°];
    - ``tool``: tool number, [0 to 14]
    - ``user``: artifact number, [0~14]
    - ``exaxis_pos``: external axis 1 position ~ external axis 4 positions"
    "Default Parameters", "
    - ``vel``: percentage of speed, [0~100] default 20.0;
    - ``acc``: percentage of acceleration, [0~100] not open yet, default 0.0;
    - ``ovl``: velocity scaling factor, [0~100] default 100.0 ;
    - ``blendT``: [-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in [ms] default -1.0;
    - ``offset_flag``: [0] - no offset, [1] - offset in workpiece/base coordinate system, [2] - offset in tool coordinate system Default 0;
    - ``offset_pos``: position offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0] ;"
    "Return Value", "Error Code Success-0 Failure- errcode;"
                                        
code example
------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    #1. Calibrate and apply the robot's tool coordinate system. You can calibrate and apply the tool coordinate system using the four-point or six-point method. The interfaces involved in tool coordinate system calibration are as follows:
    point_num=1
    id=1
    coord=[100,200,300,0,0,0,]
    type=0
    install=0
    #1. Setting up the tool coordinate system
    # robot.SetToolPoint(point_num) # set the tool reference point - six point method
    # robot.ComputeTool() # compute the tool coordinate system
    # robot.SetTcp4RefPoint() # set the tool reference point - four point method
    # robot.ComputeTcp4() # compute tool coordinate system - four point method
    # robot.SetToolCoord(id, coord,type,install) # set the application tool coordinate system
    # robot.SetToolList(id, coord,type,install) # set the list of application tool coordinate systems
    #2. Set UDP communication parameters and load UDP communication
    robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
    robot.ExtDevLoadUDPDriver();
    #3. Set the extended axis parameters, including extended axis type, extended axis driver parameters, and extended axis DH parameters.
    robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0, 0)#Single Axis Shifter and DH parameters
    robot.SetRobotPosToAxis(1); # Extended axis mounting position
    robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0)#Servo drive parameter, this example is a single-axis variant of the positioner, so you need to set only one drive parameter, if you choose to include multiple axes of the type of the extended axis, you need to set the drive parameters of each axis
    #4. Set the selected axis to enable, return to zero
    robot.ExtAxisServoOn(1, 0);
    robot.ExtAxisSetHoming(1, 0, 20, 3);
    #5. Extended axis coordinate system calibration and application (Note: the calibration interfaces of the indexer and the linear slide are different, and the following is the calibration interface of the indexer).
    pos =[0,0,0,0,0,0,0] #Input your calibration point coordinates
    robot.SetRefPointInExAxisEnd(pos)
    robot.PositionorSetRefPoint(1)#You need to calibrate the extended axis with four differently positioned points, so you need to call this interface four times to complete the calibration
    error,coord = robot.PositionorComputeECoordSys()# Calculate extended axis calibration results
    robot.ExtAxisActiveECoordSys(1, 1, coord, 1); # Apply calibration results to the extended axis coordinate system
    method=1
    #6. To calibrate the workpiece coordinate system in the extended axes, you need the following interfaces
    # robot.SetWObjCoordPoint( point_num)
    # error,coord=robot.ComputeWObjCoord( method)
    # robot.SetWObjCoord(id,coord)
    # robot.SetWObjList(id, coord)
    #7. Record your synchronized joint motion starting point
    startdescPose = [0,0,0,0,0,0,0]#Input your coordinates
    startjointPos = [0,0,0,0,0,0,0]#Input your coordinates
    startexaxisPos = [0,0,0,0,0,]#Enter your coordinates
    #8. Record your synchronized joint motion endpoint coordinates
    enddescPose = [0,0,0,0,0,0,0]#Input your coordinates
    endjointPos = [0,0,0,0,0,0,0]#Input your coordinates
    endexaxisPos = [0,0,0,0,0,]#Enter your coordinates
    #9. Write synchronized motion programs
    # Motion to the starting point, assuming the applied tool coordinate system, workpiece coordinate system are 1
    robot.ExtAxisMove(startexaxisPos, 20);
    robot.MoveJ(startjointPos, 1, 1, desc_pos=startdescPose,exaxis_pos=startexaxisPos);
    # Start synchronized movement
    robot.ExtAxisSyncMoveJ(endjointPos, enddescPose, 1, 1, endexaxisPos);
                  
UDP extension axes synchronized with robot linear motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype","``ExtAxisSyncMoveL(self, joint_pos,desc_pos, tool, user, exaxis_pos, vel=20.0, acc=0.0, ovl=100.0, blendR=-1.0, search=0, offset_flag= 0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])``"
    "Description", "UDP extension axis synchronized motion with robot linear motion"
    "Mandatory parameters", "
    - ``joint_pos``: target joint position in [°];
    - ``desc_pos``: target Cartesian position in [mm][°];
    - ``tool``: tool number, [0 to 14];
    - ``user``: artifact number, [0 to 14];
    - ``exaxis_pos``: external axis 1 position ~ external axis 4 positions;"
    "Default Parameters", "
    - ``vel``: percentage of speed, [0~100] default 20.0;
    - ``acc``: percentage of acceleration, [0~100] not open yet, default 0.0;
    - ``ovl``: velocity scaling factor, [0~100] default 100.0;
    - ``blendR``: [-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in [ms] default -1.0;
    - ``search``: [0] - no wire search, [1] - wire search;
    - ``offset_flag``: [0] - no offset, [1] - offset in workpiece/base coordinate system, [2] - offset in tool coordinate system Default 0;
    - ``offset_pos``: position offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0] ;"
    "Return Value", "Error Code Success-0 Failure- errcode;"
                                            
code example
------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.Mode(0)
    time.sleep(1)
    e_pos = [-20,0,0,0]
    joint_pos0 = [114.089,-85.740, 119.106,-129.884,-91.655, 79.642]
    desc_pos0= [-87.920,-178.539,-64.513,-175.471,7.664,139.650]
    Synchronized motion of #UDP extended axes with robot linear motion
    error = robot.ExtAxisSyncMoveL(joint_pos0,desc_pos0,1,1,e_pos)
    print("ExtAxisSyncMoveL",error)
                      
UDP extension axes synchronized with robot circular motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype","``ExtAxisSyncMoveC(joint_pos_p, desc_pos_p, tool_p, user_p,exaxis_pos_p, joint_pos_t, desc_pos_t, tool_t, user_t,exaxis_pos_t,vel_p =20.0, acc_p=100.0, offset_flag_p=0, offset_pos_p =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel_t=20.0, acc_t=100.0, offset_flag_t=0, offset_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], ovl=100.0, blendR=-1.0)``"
    "Description", " UDP extension axis synchronized with robot circular motion"
    "Mandatory parameters", "
    - ``joint_pos_p``: path point joint position in [°];
    - ``desc_pos_p``: path point Cartesian position in [mm][°];
    - ``tool_p``: pathpoint tool number, [0~14];
    - ``user_p``: pathpoint artifact number, [0~14];
    - ``exaxis_pos_p``: path point external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0];
    - ``joint_pos_t``: target point joint position in [°];
    - ``desc_pos_t``: Cartesian position of the target point in [mm][°];
    - ``tool_t``: tool number, [0~14];
    - ``user_t``: artifact number, [0~14];
    - ``exaxis_pos_t``: target point external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0];"
    "Default Parameters", "
    - ``vel_p``: path point velocity percentage, [0~100] default 20.0;
    - ``acc_p``: path point acceleration percentage, [0~100] not open yet, default 0.0;   
    - ``offset_flag_p``: whether the path point is offset [0]-no offset, [1]-offset in workpiece/base coordinate system, [2]-offset in tool coordinate system Default 0;
    - ``offset_pos_p``: path point position offset in [mm][°] Default [0.0,0.0,0.0,0.0,0.0,0.0];
    - ``vel_t``: Target point velocity percentage, [0~100] default 20.0;
    - ``acc_t``: target point acceleration percentage, [0~100] Not open yet Default 0.0;
    - ``offset_flag_t``: whether the target point is offset or not [0]-no offset, [1]-offset in workpiece/base coordinate system, [2]-offset in tool coordinate system Default 0;
    - ``offset_pos_t``: target point attitude offset in [mm][°] Default [0.0,0.0,0.0,0.0,0.0,0.0];
    - ``ovl``: velocity scaling factor, [0~100] default 100.0;
    - ``blendR``: [-1.0] - motion in place (blocking), [0~1000] - smoothing radius (non-blocking) in [mm] default -1.0;"
    "Return Value", "Error Code Success-0 Failure- errcode;"
                                                
code example
------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.Mode(0)
    time.sleep(1)
    desc_pos_mid = [-131.2748107910156, -60.21242523193359, -22.55266761779785, 175.9907989501953, 5.92541742324829, 145.5211791992187]
    desc_pos_end = [-91.3530502319336, -174.5040588378906, -64.93866729736328, 177.1370544433593, 15.96347618103027, 136.1746368408203]
    joint_pos_mid = [120.9549040841584, -109.8869943146658, 134.1448068146658, -126.2150709699876, -88.6738087871287, 79.6419593131188]
    joint_pos_end = [110.1896078279703, -89.01601659189356, 125.5532806698638, -139.7967831451114, -82.93198387221534, 79.6452225788985]
    # #UDP extension axis synchronized with robot circular motion
    time.sleep(3)
    error = robot.ExtAxisSyncMoveC(joint_pos_mid,desc_pos_mid,1,1,[-10,0,0,0],joint_pos_end,desc_pos_end,1,1,[-20,0,0,0])
    print("ExtAxisSyncMoveC",error)

Removable unit control
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

Removable Device Enable
---------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``TractorEnable(enable)``"
    "description", "removable device enable"
    "Mandatory parameters", "- ``enable``: enable state, 0-de-enable, 1-enable"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Zeroing of removable units
---------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``TractorHoming()``"
    "Description", "Removable unit back to zero"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Movable unit linear motion
---------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``TractorMoveL(distance, vel)``"
    "Description", "Movable device linear motion"
    "Mandatory parameters", "- ``distance``: distance of linear movement (mm)
    - ``vel``: percentage of linear motion speed (0-100)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Movable unit circular motion
---------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``TractorMoveC(radio, angle, vel)``"
    "Description", "Movable device circular motion"
    "Mandatory parameters", "- ``radio``: radius of circular motion (mm)
    - ``angle``: angle of circular motion (°)
    - ``vel``: Percentage of speed of circular motion (0-100)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Stopping motion of movable devices
------------------------------------------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramStop()``"
    "Description", "Movable unit stops moving"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10)
    robot.ExtDevLoadUDPDriver()
    robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0)
    robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0)
    robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    robot.TractorEnable(False)
    time.sleep(2)
    robot.TractorEnable(True)
    time.sleep(2)
    robot.TractorHoming()
    time.sleep(2)
    robot.TractorMoveL(100, 20)
    time.sleep(5)
    robot.TractorMoveL(-100, 20)
    time.sleep(5)
    robot.TractorMoveC(300, 90, 20)
    time.sleep(4)
    error = robot.TractorStop()
    print("TractorStop return ", error)