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

    ret = robot.AuxServoSetTargetSpeed(1, 30)  # speed mode motion，speed 30
    print("AuxServoSetTargetSpeed", ret)
    sleep(10)

    ret = robot.AuxServoGetStatus(1)  # Get 485 extended axis servo status
    print("AuxServoGetStatus", ret)
    sleep(1)

    ret = robot.AuxServoSetTargetSpeed(1, 60)  # speed mode motion，speed 60
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
