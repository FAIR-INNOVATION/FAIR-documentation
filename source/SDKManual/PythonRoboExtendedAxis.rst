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

Setting the 485 extended axis data axis number in the status feedback
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

Extended axis control code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 15.45)
    print(f"AuxServoSetParam is: {retval}")
    servoCompany = 0
    servoModel = 0
    servoSoftVersion = 0
    servoResolution = 0
    axisMechTransRatio = 0.0
    retval, servoCompany, servoModel, servoSoftVersion, servoResolution, axisMechTransRatio = robot.AuxServoGetParam(1)
    print(f"servoCompany {servoCompany}\n"
          f"servoModel {servoModel}\n"
          f"servoSoftVersion {servoSoftVersion}\n"
          f"servoResolution {servoResolution}\n"
          f"axisMechTransRatio {axisMechTransRatio}\n")
    retval = robot.AuxServoSetParam(1, 10, 11, 12, 13, 14)
    print(f"AuxServoSetParam is: {retval}")
    retval, servoCompany, servoModel, servoSoftVersion, servoResolution, axisMechTransRatio = robot.AuxServoGetParam(1)
    print(f"servoCompany {servoCompany}\n"
          f"servoModel {servoModel}\n"
          f"servoSoftVersion {servoSoftVersion}\n"
          f"servoResolution {servoResolution}\n"
          f"axisMechTransRatio {axisMechTransRatio}\n")
    retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 36)
    print(f"AuxServoSetParam is: {retval}")
    time.sleep(3)
    robot.AuxServoSetAcc(3000, 3000)
    robot.AuxServoSetEmergencyStopAcc(5000, 5000)
    time.sleep(1)
    emagacc = 0.0
    emagdec = 0.0
    acc = 0.0
    dec = 0.0
    error,emagacc, emagdec = robot.AuxServoGetEmergencyStopAcc()
    print(f"emergency acc is {emagacc}  dec is {emagdec}")
    error,acc, dec = robot.AuxServoGetAcc()
    print(f"acc is {acc}  dec is {dec}")
    robot.AuxServoSetControlMode(1, 0)
    time.sleep(2)
    retval = robot.AuxServoEnable(1, 0)
    print(f"AuxServoEnable disenable {retval}")
    time.sleep(1)
    servoErrCode = 0
    servoState = 0
    servoPos = 0.0
    servoSpeed = 0.0
    servoTorque = 0.0
    retval, servoErrCode, servoState, servoPos, servoSpeed, servoTorque = robot.AuxServoGetStatus(1)
    print(f"AuxServoGetStatus servoState {servoState}")
    time.sleep(1)
    retval = robot.AuxServoEnable(1, 1)
    print(f"AuxServoEnable enable {retval}")
    time.sleep(1)
    retval, servoErrCode, servoState, servoPos, servoSpeed, servoTorque = robot.AuxServoGetStatus(1)
    print(f"AuxServoGetStatus servoState {servoState}")
    time.sleep(1)
    retval = robot.AuxServoHoming(1, 1, 5, 1,100)
    print(f"AuxServoHoming {retval}")
    time.sleep(3)
    retval = robot.AuxServoSetTargetPos(1, 200, 30,100)
    print(f"AuxServoSetTargetPos {retval}")
    time.sleep(1)
    retval, servoErrCode, servoState, servoPos, servoSpeed, servoTorque = robot.AuxServoGetStatus(1)
    print(f"AuxServoGetStatus servoSpeed {servoSpeed}")
    time.sleep(8)
    robot.AuxServoSetControlMode(1, 1)
    time.sleep(2)
    robot.AuxServoEnable(1, 0)
    time.sleep(1)
    robot.AuxServoEnable(1, 1)
    time.sleep(1)
    robot.AuxServoSetTargetSpeed(1, 100, 80)
    time.sleep(5)
    robot.AuxServoSetTargetSpeed(1, 0, 80)
    robot.CloseRPC()

Parameter configuration for UDP extended axis communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``ExtDevSetUDPComParam(ip, port, period, lossPkgTime, lossPkgNum, disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum, selfConnect)``"
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
    - ``reconnectNum``: number of reconnections
    - ``selfConnect``: Whether the connection is automatically established upon power-off and restart; 0- No connection established; 1- Establish a connection"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

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
    - ``reconnectNum``: number of reconnections
    - ``reconnectNum``: Number of reconnection attempts
    - ``selfConnect``: Whether to automatically reconnect after control box restart; 0-no reconnection; 1-reconnect"
 
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

Example of UDP extension axis configuration and tapping code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1)
    print(f"ExtDevSetUDPComParam rtn is {rtn}")
    ip = ""
    port = 0
    period = 0
    lossPkgTime = 0
    lossPkgNum = 0
    disconnectTime = 0
    reconnectEnable = 0
    reconnectPeriod = 0
    reconnectNum = 0
    rtn,[ip, port, period, lossPkgTime, lossPkgNum,disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum] = robot.ExtDevGetUDPComParam()
    param_str = (f"\nip {ip}\nport {port}\nperiod {period}\nlossPkgTime {lossPkgTime}"
                 f"\nlossPkgNum {lossPkgNum}\ndisConntime {disconnectTime}"
                 f"\nreconnecable {reconnectEnable}\nreconnperiod {reconnectPeriod}"
                 f"\nreconnnun {reconnectNum}")
    print(f"ExtDevGetUDPComParam rtn is {rtn}{param_str}")
    robot.ExtDevLoadUDPDriver()
    rtn = robot.ExtAxisServoOn(1, 1)
    print(f"ExtAxisServoOn axis id 1 rtn is {rtn}")
    rtn = robot.ExtAxisServoOn(2, 1)
    print(f"ExtAxisServoOn axis id 2 rtn is {rtn}")
    time.sleep(2)
    robot.ExtAxisSetHoming(1, 0, 10, 2)
    time.sleep(2)
    rtn = robot.ExtAxisSetHoming(2, 0, 10, 2)
    print(f"ExtAxisSetHoming rtn is {rtn}")
    time.sleep(4)
    rtn = robot.SetRobotPosToAxis(1)
    print(f"SetRobotPosToAxis rtn is {rtn}")
    rtn = robot.SetAxisDHParaConfig(10, 20, 0, 0, 0, 0, 0, 0, 0)
    print(f"SetAxisDHParaConfig rtn is {rtn}")
    rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0)
    print(f"ExtAxisParamConfig axis 1 rtn is {rtn}")
    rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0)
    print(f"ExtAxisParamConfig axis 2 rtn is {rtn}")
    time.sleep(3)
    robot.ExtAxisStartJog(1, 0, 10, 10, 30)
    time.sleep(1)
    robot.ExtAxisStopJog(1)
    time.sleep(3)
    robot.ExtAxisServoOn(1, 0)
    time.sleep(3)
    robot.ExtAxisStartJog(2, 0, 10, 10, 30)
    time.sleep(1)
    robot.ExtAxisStopJog(2)
    time.sleep(3)
    robot.ExtAxisServoOn(2, 0)
    robot.ExtDevUnloadUDPDriver()
    robot.CloseRPC()

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

Obtain the extended axis coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisGetCoord()``"
    "Description", "Obtain the extended axis coordinate system"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``coord``：Extended axis coordinate system"

Extended axis coordinate system calibration code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1)
    print(f"ExtDevSetUDPComParam rtn is {rtn}")
    rtn,udp_params = robot.ExtDevGetUDPComParam()
    ip, port, period, lossPkgTime, lossPkgNum, disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum = udp_params
    patam = (
        f"\nip {ip}\nport {port}\nperiod {period}\nlossPkgTime {lossPkgTime}\n"
        f"lossPkgNum {lossPkgNum}\ndisConntime {disconnectTime}\nreconnecable {reconnectEnable}\n"
        f"reconnperiod {reconnectPeriod}\nreconnnun {reconnectNum}"
    )
    print(f"ExtDevGetUDPComParam rtn is {rtn}{patam}")
    robot.ExtDevLoadUDPDriver()
    rtn = robot.ExtAxisServoOn(1, 1)
    print(f"ExtAxisServoOn axis id 1 rtn is {rtn}")
    rtn = robot.ExtAxisServoOn(2, 1)
    print(f"ExtAxisServoOn axis id 2 rtn is {rtn}")
    time.sleep(2)
    robot.ExtAxisSetHoming(1, 0, 10, 2)
    time.sleep(2)
    rtn = robot.ExtAxisSetHoming(2, 0, 10, 2)
    print(f"ExtAxisSetHoming rtn is {rtn}")
    time.sleep(4)
    rtn = robot.SetRobotPosToAxis(1)
    print(f"SetRobotPosToAxis rtn is {rtn}")
    rtn = robot.SetAxisDHParaConfig(1, 128.5, 206.4, 0, 0, 0, 0, 0, 0)
    print(f"SetAxisDHParaConfig rtn is {rtn}")
    rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0)
    print(f"ExtAxisParamConfig axis 1 rtn is {rtn}")
    rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0)
    print(f"ExtAxisParamConfig axis 2 rtn is {rtn}")
    toolCoord = [0, 0, 210, 0, 0, 0]
    robot.SetToolCoord(1, toolCoord, 0, 0, 1, 0)
    jSafe = [115.193, -96.149, 92.489, -87.068, -89.15, -83.488]
    j1 = [117.559, -92.624, 100.329, -96.909, -94.057, -83.488]
    j2 = [112.239, -90.096, 99.282, -95.909, -89.824, -83.488]
    j3 = [110.839, -83.473, 93.166, -89.22, -90.499, -83.487]
    j4 = [107.935, -83.572, 95.424, -92.873, -87.933, -83.488]
    descSafe = [0.0,0.0,0.0,0.0,0.0,0.0]
    desc1 = [0.0,0.0,0.0,0.0,0.0,0.0]
    desc2 = [0.0,0.0,0.0,0.0,0.0,0.0]
    desc3 = [0.0,0.0,0.0,0.0,0.0,0.0]
    desc4 = [0.0,0.0,0.0,0.0,0.0,0.0]
    exaxisPos = [0.0,0.0,0.0,0.0]
    offdese = [0.0,0.0,0.0,0.0,0.0,0.0]
    error, descSafe = robot.GetForwardKin(jSafe)
    robot.MoveJ(joint_pos=jSafe,tool= 1,user= 0,vel= 100)
    time.sleep(2)
    error, desc1 = robot.GetForwardKin(j1)
    robot.MoveJ(joint_pos=j1,tool= 1,user= 0,vel= 100)
    time.sleep(2)
    actualTCPPos = [0.0,0.0,0.0,0.0,0.0,0.0]
    error, actualTCPPos = robot.GetActualTCPPose(0)
    robot.SetRefPointInExAxisEnd(actualTCPPos)
    rtn = robot.PositionorSetRefPoint(1)
    print(f"PositionorSetRefPoint 1 rtn is {rtn}")
    time.sleep(2)
    robot.MoveJ(joint_pos=jSafe,tool= 1,user= 0,vel= 100)
    robot.ExtAxisStartJog(1, 0, 50, 50, 10)
    time.sleep(1)
    robot.ExtAxisStartJog(2, 0, 50, 50, 10)
    time.sleep(1)
    error, desc2 = robot.GetForwardKin(j2)
    rtn = robot.MoveJ(joint_pos=j2,tool= 1,user= 0,vel= 100)
    rtn = robot.PositionorSetRefPoint(2)
    print(f"PositionorSetRefPoint 2 rtn is {rtn}")
    time.sleep(2)
    robot.MoveJ(joint_pos=jSafe,tool= 1,user= 0,vel= 100)
    robot.ExtAxisStartJog(1, 0, 50, 50, 10)
    time.sleep(1)
    robot.ExtAxisStartJog(2, 0, 50, 50, 10)
    time.sleep(1)
    error, desc3 = robot.GetForwardKin(j3)
    robot.MoveJ(joint_pos=j3,tool= 1,user= 0,vel= 100)
    rtn = robot.PositionorSetRefPoint(3)
    print(f"PositionorSetRefPoint 3 rtn is {rtn}")
    time.sleep(2)
    robot.MoveJ(joint_pos=jSafe,tool= 1,user= 0,vel= 100)
    robot.ExtAxisStartJog(1, 0, 50, 50, 10)
    time.sleep(1)
    robot.ExtAxisStartJog(2, 0, 50, 50, 10)
    time.sleep(1)
    error, desc4 = robot.GetForwardKin(j4)
    robot.MoveJ(joint_pos=j4,tool= 1,user= 0,vel= 100)
    rtn = robot.PositionorSetRefPoint(4)
    print(f"PositionorSetRefPoint 4 rtn is {rtn}")
    time.sleep(2)
    axisCoord = [0.0,0.0,0.0,0.0,0.0,0.0]
    error,axisCoord = robot.PositionorComputeECoordSys()
    robot.MoveJ(joint_pos=jSafe,tool= 1,user= 0,vel= 100)
    print(f"PositionorComputeECoordSys rtn is {axisCoord[0]} {axisCoord[1]} {axisCoord[2]} {axisCoord[3]} {axisCoord[4]} {axisCoord[5]}")
    rtn = robot.ExtAxisActiveECoordSys(3, 1, axisCoord, 1)
    print(f"ExtAxisActiveECoordSys rtn is {rtn}")
    robot.CloseRPC()
          
UDP Extended Axis Motion
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ExtAxisMove(pos,ovl,blend=-1)``"
    "Description", "UDP Extended Axis Motion"
    "Mandatory parameters", "- ``pos=[exaxis[0],exaxis[1],exaxis[2],exaxis[3]]``: target position Axis 1 position to Axis 4 position;
    - ``ovl``: percentage of speed"
    "Default parameters", "- ``blend``：Smoothing parameter (mm or ms), -1, waiting for the motion to complete, Default -1"
    "Return Value", "Error Code Success-0 Failure- errcode"
                                        
UDP Extended axis motion code example
++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    axisPos = [20,0,0,0]
    robot.ExtAxisMove(axisPos, 50, -1)
    robot.CloseRPC()
    return 0

UDP extension axes synchronized with robot joint motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype","``ExtAxisSyncMoveJ(joint_pos,tool,user,exaxis_pos, desc_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0, ovl= 100.0, blendT=-1.0, offset_flag=0, offset_pos=[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])``"
    "Description", "UDP extension axis synchronized motion with robot joint motion"
    "Mandatory parameters", "
    - ``joint_pos``: target joint position in [°];
    - ``desc_pos``: Cartesian position of the target in [mm][°];
    - ``tool``: tool number, [0 to 14]
    - ``user``: artifact number, [0~14]
    - ``exaxis_pos``: external axis 1 position ~ external axis 4 positions"
    "Default Parameters", "
    - ``desc_pos``: target Cartesian position in [mm][°] Default initial value [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls positive kinematics to solve for the return value.
    - ``vel``: percentage of speed, [0~100] default 20.0;
    - ``acc``: percentage of acceleration, [0~100] not open yet, default 0.0;
    - ``ovl``: velocity scaling factor, [0~100] default 100.0 ;
    - ``blendT``: [-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in [ms] default -1.0;
    - ``offset_flag``: [0] - no offset, [1] - offset in workpiece/base coordinate system, [2] - offset in tool coordinate system Default 0;
    - ``offset_pos``: position offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0] ;"
    "Return Value", "Error Code Success-0 Failure- errcode;"
                                        
UDP extension axes synchronized with robot joint motion code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Set the UDP communication parameters and load
    robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10)
    robot.ExtDevLoadUDPDriver()
    # Set the parameters of the extended axis
    robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0)
    robot.SetRobotPosToAxis(1)
    robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0)
    # Expand the axis to enable and return to zero
    robot.ExtAxisServoOn(1, 0)
    robot.ExtAxisSetHoming(1, 0, 20, 3)
    # Extended axis coordinate system calibration
    pos = []  # Please fill in the specific coordinates
    robot.SetRefPointInExAxisEnd(pos)
    robot.PositionorSetRefPoint(1)  # This operation should be repeated 4 times (using 4 dots)
    error,coord = robot.PositionorComputeECoordSys()
    robot.ExtAxisActiveECoordSys(1, 1, coord, 1)
    # Synchronize the starting and ending points of the movement
    startdescPose = []  # Please fill in the specific coordinates
    startjointPos = []  # Please fill in the specific coordinates
    startexaxisPos = []  # Please fill in the specific coordinates
    enddescPose = []  # Please fill in the specific coordinates
    endjointPos = []  # Please fill in the specific coordinates
    endexaxisPos = []  # Please fill in the specific coordinates
    # Move to the starting point
    robot.ExtAxisMove(startexaxisPos, 20, -1)
    offdese = [0, 0, 0, 0, 0, 0]
    robot.MoveJ(joint_pos=startjointPos,tool= 1,user= 1,vel= 100,acc= 100,ovl= 100,exaxis_pos= startexaxisPos,blendT= 0,offset_flag= 0,offset_pos= offdese)
    robot.ExtAxisSyncMoveJ(endjointPos, enddescPose, 1, 1, endexaxisPos, 100, 100, 100, -1, 0, offdese)
    robot.CloseRPC()
                  
UDP extension axes synchronized with robot linear motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype","``ExtAxisSyncMoveL(desc_pos, tool, user, exaxis_pos, joint_pos = [0.0,0.0,0.0,0.0,0.0,0.0,0.0], vel=20.0, acc=0.0, ovl=100.0, blendR=-1.0, search=0, offset_flag= 0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],config=-1)``"
    "Description", "UDP extension axis synchronized motion with robot linear motion"
    "Mandatory parameters", "
    - ``desc_pos``: target Cartesian position in [mm][°];
    - ``tool``: tool number, [0 to 14];
    - ``user``: artifact number, [0 to 14];
    - ``exaxis_pos``: external axis 1 position ~ external axis 4 positions;"
    "Default Parameters", "
    - ``joint_pos``: target joint position in [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls inverse kinematics to solve for the return value.
    - ``vel``: percentage of speed, [0~100] default 20.0;
    - ``acc``: percentage of acceleration, [0~100] not open yet, default 0.0;
    - ``ovl``: velocity scaling factor, [0~100] default 100.0;
    - ``blendR``: [-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in [ms] default -1.0;
    - ``search``: [0] - no wire search, [1] - wire search;
    - ``offset_flag``: [0] - no offset, [1] - offset in workpiece/base coordinate system, [2] - offset in tool coordinate system Default 0;
    - ``offset_pos``: position offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0] ;
    - ``config``: Reverse the joint space configuration, [-1]- calculate based on the current joint position, [0~7]- solve based on the specific joint space configuration, default -1"
    "Return Value", "Error Code Success-0 Failure- errcode;"

UDP extension axes synchronized with robot linear motion code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Set the UDP communication parameters and load
    robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10)
    robot.ExtDevLoadUDPDriver()
    # Set the parameters of the extended axis
    robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0)
    robot.SetRobotPosToAxis(1)
    robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0)
    # Expand the axis to enable and return to zero
    robot.ExtAxisServoOn(1, 0)
    robot.ExtAxisSetHoming(1, 0, 20, 3)
    # Extended axis coordinate system calibration
    pos = []  # Please fill in the coordinates of the marking points
    robot.SetRefPointInExAxisEnd(pos)
    robot.PositionorSetRefPoint(1)  # It needs to be called four times for calibration
    error,coord = robot.PositionorComputeECoordSys()
    robot.ExtAxisActiveECoordSys(1, 1, coord, 1)
    # Synchronize the starting and ending points of the movement
    startdescPose = []  # Please fill in the coordinates of the marking points
    startjointPos = []  # Please fill in the coordinates of the marking points
    startexaxisPos = []  # Please fill in the coordinates of the marking points
    enddescPose = []  # Please fill in the coordinates of the marking points
    endjointPos = []  # Please fill in the coordinates of the marking points
    endexaxisPos = []  # Please fill in the coordinates of the marking points
    # Move to the starting point
    robot.ExtAxisMove(startexaxisPos, 20, -1)
    offdese = [0, 0, 0, 0, 0, 0]
    robot.MoveJ(joint_pos=startjointPos, tool= 1,user= 1,vel= 100,acc= 100,ovl= 100,exaxis_pos= startexaxisPos,blendT= 0)
    # Perform synchronous linear motion
    robot.ExtAxisSyncMoveL(endjointPos, enddescPose, 1, 1, endexaxisPos, 100, 100, 100, 0, 0, offdese)
    robot.CloseRPC()
                      
UDP extension axes synchronized with robot circular motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype","``ExtAxisSyncMoveC(desc_pos_p, tool_p, user_p,exaxis_pos_p, desc_pos_t, tool_t, user_t,exaxis_pos_t,joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],vel_p =20.0, acc_p=100.0, offset_flag_p=0, offset_pos_p =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel_t=20.0, acc_t=100.0, offset_flag_t=0, offset_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], ovl=100.0, blendR=-1.0, config=-1)``"
    "Description", " UDP extension axis synchronized with robot circular motion"
    "Mandatory parameters", "
    - ``desc_pos_p``: path point Cartesian position in [mm][°];
    - ``tool_p``: pathpoint tool number, [0~14];
    - ``user_p``: pathpoint artifact number, [0~14];
    - ``exaxis_pos_p``: path point external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0];
    - ``desc_pos_t``: Cartesian position of the target point in [mm][°];
    - ``tool_t``: tool number, [0~14];
    - ``user_t``: artifact number, [0~14];
    - ``exaxis_pos_t``: target point external axis 1 position ~ external axis 4 position default [0.0,0.0,0.0,0.0];"
    "Default Parameters", "
    - ``joint_pos_p``: target joint position in [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls inverse kinematics to solve for the return value.
    - ``joint_pos_t``: target joint position in [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls inverse kinematics to solve for the return value.
    - ``vel_p``: path point velocity percentage, [0~100] default 20.0;
    - ``acc_p``: path point acceleration percentage, [0~100] not open yet, default 0.0;   
    - ``offset_flag_p``: whether the path point is offset [0]-no offset, [1]-offset in workpiece/base coordinate system, [2]-offset in tool coordinate system Default 0;
    - ``offset_pos_p``: path point position offset in [mm][°] Default [0.0,0.0,0.0,0.0,0.0,0.0];
    - ``vel_t``: Target point velocity percentage, [0~100] default 20.0;
    - ``acc_t``: target point acceleration percentage, [0~100] Not open yet Default 0.0;
    - ``offset_flag_t``: whether the target point is offset or not [0]-no offset, [1]-offset in workpiece/base coordinate system, [2]-offset in tool coordinate system Default 0;
    - ``offset_pos_t``: target point attitude offset in [mm][°] Default [0.0,0.0,0.0,0.0,0.0,0.0];
    - ``ovl``: velocity scaling factor, [0~100] default 100.0;
    - ``blendR``: [-1.0] - motion in place (blocking), [0~1000] - smoothing radius (non-blocking) in [mm] default -1.0;
    - ``config``: Reverse the joint space configuration, [-1]- calculate based on the current joint position, [0~7]- solve based on the specific joint space configuration, default -1"
    "Return Value", "Error Code Success-0 Failure- errcode;"
                                                
UDP extension axes synchronized with robot circular motion code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Set the UDP communication parameters and load
    robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10)
    robot.ExtDevLoadUDPDriver()
    # Set the parameters of the extended axis
    robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0)
    robot.SetRobotPosToAxis(1)
    robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0)
    # Expand the axis to enable and return to zero
    robot.ExtAxisServoOn(1, 0)
    robot.ExtAxisSetHoming(1, 0, 20, 3)
    # Extended axis coordinate system calibration
    pos = []  # Enter the coordinates of the reference point
    robot.SetRefPointInExAxisEnd(pos)
    robot.PositionorSetRefPoint(1)  # Call four times to complete the calibration
    coord = []
    error,coord = robot.PositionorComputeECoordSys()
    robot.ExtAxisActiveECoordSys(1, 1, coord, 1)
    # Synchronous arc starting point, middle point and end point
    startdescPose = []# Input coordinates
    startjointPos = []# Input coordinates
    startexaxisPos =[]  # Enter the coordinates of the extended axis
    middescPose = []# Input the midpoint
    midjointPos = []
    midexaxisPos =[]
    enddescPose = []
    endjointPos = []
    endexaxisPos =[]
    # Move to the starting point
    robot.ExtAxisMove(startexaxisPos, 20, -1)
    offdese = [0, 0, 0, 0, 0, 0]
    robot.MoveJ(joint_pos=startjointPos,tool= 1,user= 1,vel= 100,acc= 100,ovl= 100,exaxis_pos= startexaxisPos,blendT= 0,offset_flag= 0,offset_pos= offdese)
    # Start synchronous circular arc movement
    robot.ExtAxisSyncMoveC(midjointPos,middescPose,1,1,midexaxisPos,
                           endjointPos,enddescPose,1,1,endexaxisPos,
                           100,100,0,offdese,
                           100,100,0,offdese,
                           100,0)
    robot.CloseRPC()

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

Extended IO code examples
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    for i in range(128):
        robot.SetAuxDO(i, True, False, True)
        time.sleep(0.1)
    for i in range(128):
        robot.SetAuxDO(i, False, False, True)
        time.sleep(0.1)
    for i in range(409):
        value1 = i * 10
        value2 = 4095 - i * 10
        robot.SetAuxAO(0, value1, True)
        robot.SetAuxAO(1, value2, True)
        robot.SetAuxAO(2, value1, True)
        robot.SetAuxAO(3, value2, True)
        time.sleep(0.01)
    robot.SetAuxDIFilterTime(10)
    robot.SetAuxAIFilterTime(0, 10)
    for i in range(20):
        curValue = False
        error, curValue = robot.GetAuxDI(i, False)  # 注意：如库内部需引用方式，这里需修改
        print(f"DI{i}   {curValue}")
    curValue = -1
    for i in range(4):
        error, curValue = robot.GetAuxAI(i, True)  # 同样注意引用传参问题
        print(f"AI{i}   {curValue}")
    robot.WaitAuxDI(1, False, 1000, False)
    robot.WaitAuxAI(1, 1, 132, 1000, False)
    robot.CloseRPC()

Removable Device Enable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``TractorEnable(enable)``"
    "description", "removable device enable"
    "Mandatory parameters", "- ``enable``: enable state, 0-de-enable, 1-enable"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Zeroing of removable units
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``TractorHoming()``"
    "Description", "Removable unit back to zero"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Movable unit linear motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

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
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

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
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ProgramStop()``"
    "Description", "Movable unit stops moving"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Portable device code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10, 1)
    robot.ExtDevLoadUDPDriver()
    rtn = robot.ExtAxisServoOn(1, 1)
    rtn = robot.ExtAxisServoOn(2, 1)
    time.sleep(2)
    robot.ExtAxisSetHoming(1, 0, 10, 2)
    time.sleep(2)
    rtn = robot.ExtAxisSetHoming(2, 0, 10, 2)
    time.sleep(4)
    robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0)
    robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0)
    robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0)
    robot.TractorEnable(False)
    time.sleep(2)
    robot.TractorEnable(True)
    time.sleep(2)
    robot.TractorHoming()
    time.sleep(2)
    robot.TractorMoveL(100, 2)
    time.sleep(5)
    robot.TractorStop()
    robot.TractorMoveL(-100, 20)
    time.sleep(5)
    robot.TractorMoveC(300, 90, 20)
    time.sleep(10)
    robot.TractorMoveC(300, -90, 20)
    time.sleep(1)
    robot.CloseRPC()

Laser sensor recording points
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LaserRecordPoint(coordID)``"
    "Description", "Laser sensor recording points"
    "Mandatory par", "- ``coordID``：Laser sensor coordinate system"
    "Default param", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``joint``：The laser sensor identifies the position of the point joint
    - ``desc``：The laser sensor identifies the Cartesian position of the point
    - ``exaxis``：The laser sensor identifies the extended axis position of the point"

Sample code for laser sensor recording points
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    direction_point = [0, 0, 0]
    rtn = robot.LaserTrackingSearchStart(2, direction_point, 10, 100, 10000, 2)
    print(f"LaserTrackingSearchStart rtn is {rtn}")
    robot.LaserTrackingSearchStop()
    coord_id = 2
    rtn, joint, desc, exaxis = robot.LaserRecordPoint(coord_id)
    print(f"rtn is {rtn}")
    print(f"desc_pos:{desc[0]},{desc[1]},{desc[2]},"
          f"{desc[3]},{desc[4]},{desc[5]}")
    print(f"joint_pos:{joint[0]},{joint[1]},{joint[2]},{joint[3]},{joint[4]},{joint[5]}")
    print(f"exaxis pos is {exaxis[0]} {exaxis[1]} {exaxis[2]} {exaxis[3]}")
    off = [0] * 6
    robot.MoveJ(joint,tool=1,user=0,vel=100,acc=100,ovl=50,exaxis_pos=exaxis,blendT=-1,offset_flag=0,offset_pos=off)
    robot.CloseRPC()

Set the synchronous movement strategy of the extended axis and the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExAxisRobotPlan(strategy)``"
    "Description", "Set the synchronous movement strategy of the extended axis and the robot"
    "Mandatory par", "- ``strategy``：Strategy; 0- Mainly robots; 1- The extended axis is synchronized with the robot"
    "Default param", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Code example for setting the synchronous motion strategy of the extended axis and the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.5

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    joint_pos1 = [-22.016, -49.217, 124.714, -161.100, -85.108, -0.333]
    joint_pos2 = [-21.083, -46.613, 110.079, -147.796, -80.757, -0.330]
    joint_pos3 = [-25.572, -60.090, 135.397, -163.889, -82.489, -0.345]
    desc_pos1 = [2.637, -0.001, 30.673, 178.786, -4.134, 68.326]
    desc_pos2 = [213.812, -1.440, 47.311, 177.410, 0.166, 68.946]
    desc_pos3 = [444.342, -12.723, 82.470, -177.701, -1.325, 65.151]
    epos1 = [0.001, 0.000, 0.000, 0.000]
    epos2 = [299.977, 0.000, 0.000, 0.000]
    epos3 = [399.969, 0.000, 0.000, 0.000]
    offset_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    rtn = robot.SetExAxisRobotPlan(0)
    print(f"SetExAxisRobotPlan rtn is {rtn}")
    time.sleep(1)
    rtn = robot.ExtAxisSyncMoveL(desc_pos=desc_pos1,tool=1,user=0,vel=100,acc=100,ovl=100,blendR=-1,exaxis_pos=epos1,offset_flag=0,offset_pos=offset_pos)
    print(f"ExtAxisSyncMoveL 1 rtn is {rtn}")
    rtn = robot.ExtAxisSyncMoveL(desc_pos=desc_pos2,tool=1,user=0,vel=100,acc=100,ovl=100,blendR=-1,exaxis_pos=epos2,offset_flag=0,offset_pos=offset_pos)
    print(f"ExtAxisSyncMoveL 2 rtn is {rtn}")
    rtn = robot.ExtAxisSyncMoveL(desc_pos=desc_pos3,tool=1,user=0,vel=100,acc=100,ovl=100,blendR=-1,exaxis_pos=epos3,offset_flag=0,offset_pos=offset_pos)
    print(f"ExtAxisSyncMoveL 3 rtn is {rtn}")
    time.sleep(8)
    robot.CloseRPC()
