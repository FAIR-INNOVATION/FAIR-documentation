Extended axis
=================

.. toctree:: 
    :maxdepth: 5

Set 485 extended axis parameters
++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Set 485 extended axis parameters
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID 
    * @param [in] servoCompany Servo driver manufacturer, 1-Dynatech
    * @param [in] servoModel Servo driver model, 1-FD100-750C
    * @param [in] servoSoftVersion Servo driver software version, 1-V1.0
    * @param [in] servoResolution Encoder resolution
    * @param [in] axisMechTransRatio Mechanical transmission ratio
    * @return Error code
    */
    int AuxServoSetParam(int servoId, int servoCompany, int servoModel, int servoSoftVersion, int servoResolution, double axisMechTransRatio);

Get 485 extended axis configuration parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Get 485 extended axis configuration parameters
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @param [out] servoCompany Servo driver manufacturer, 1-Dynatec
    * @param [out] servoModel Servo drive model, 1-FD100-750C
    * @param [out] servoSoftVersion Servo drive software version, 1-V1.0
    * @param [out] servoResolution Encoder resolution
    * @param [out] axisMechTransRatio Mechanical transmission ratio
    * @return Error code 
    */
    int AuxServoGetParam(int servoId, ref int servoCompany, ref int servoModel, ref int servoSoftVersion, ref int servoResolution, ref double axisMechTransRatio);

Set 485 extended axis enable/disable
+++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:
     
    * @brief Set 485 extension axis enable/disable
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @param [in] status Enable status, 0-disable, 1-enable
    * @return Error code 
    */
    int AuxServoEnable(int servoId, int status);

Set 485 extension axis control mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Set 485 extension axis control mode
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @param [in] mode Control mode, 0-position mode, 1-speed mode
    * @return Error code   
    */
    int AuxServoSetControlMode(int servoId, int mode);

Set the target position of the 485 extended axis (position mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Set the target position of the 485 extended axis (position mode)
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID 
    * @param [in] pos Target position, mm or °
    * @param [in] speed Target speed, mm/s or °/s
    * @return Error code 
    */
    int AuxServoSetTargetPos(int servoId, double pos, double speed);

Set the target speed of the 485 extended axis (speed mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Set the target speed of the 485 extended axis (speed mode)
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to the slave ID
    * @param [in] speed Target speed, mm/s or °/s
    * @return Error code 
    */
    int AuxServoSetTargetSpeed(int servoId, double speed);

Set the target torque of the 485 extended axis (torque mode) -- Not available yet
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:
    
    /** 
    * @brief Set the target torque of the 485 extended axis (torque mode) -- not yet available
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to the slave ID
    * @param [in] torque Target torque, Nm
    * @return Error code 
    */   
    int AuxServoSetTargetTorque(int servoId, double torque); 

Set 485 extended axis zero
++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Set 485 extended axis zero
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID 
    * @param [in] mode Home mode, 1-current position home; 2-negative limit home; 3-positive limit home
    * @param [in] searchVel Home speed, mm/s or °/s
    * @param [in] latchVel Latching speed, mm/s or °/s
    * @return Error code
    */
    int AuxServoHoming(int servoId, int mode, double searchVel, double latchVel);
        
Clear 485 extended axis error information
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:
    
    /** 
    * @brief Clear 485 extended axis error information
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @return Error code
    */
    int AuxServoClearError(int servoId);

Get 485 extended axis servo status
++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Get the status of the 485 extended axis servo
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to the slave ID 
    * @param [out] servoErrCode Servo driver error code
    * @param [out] servoState Servo driver status  bit0: 0-Disabled; 1-Enabled;  bit1: 0-Not moving; 1-Moving;  bit2 0-Positive limit not triggered; 1-Positive limit triggered; bit3 0-Negative limit not triggered; 1-Negative limit triggered; bit4 0-Positioning not completed;1 - Positioning complete;  bit5: 0 - Not zeroed; 1 - Zeroing complete
    * @param [out] servoPos Servo current position mm or °
    * @param [out] servoSpeed Servo current speed mm/s or °/s
    * @param [out] servoTorque Servo current torque Nm
    * @return Error code 
    */
    int AuxServoGetStatus(int servoId, ref int servoErrCode, ref int servoState, ref double servoPos, ref double servoSpeed, ref double servoTorque);

Set the 485 extended axis data axis number in the status feedback
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Set the data axis number of the 485 extended axis in the status feedback
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to the slave ID
    * @return Error code
    */
    int AuxServosetStatusID(int servoId);

Set the acceleration and deceleration of the 485 extended axis
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: C#
    :linenos:

    /**
    * @brief Set the acceleration and deceleration of the 485 extended axis
    * @param [in] acc 485 extended axis acceleration
    * @param [in] dec 485 extended axis deceleration
    * @return  Error code
    */
    int AuxServoSetAcc(double acc, double dec);

Set 485 extended axis emergency stop acceleration and deceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: C#
    :linenos:

    /**
    * @brief Set 485 extended axis emergency stop acceleration and deceleration
    * @param [in] acc 485 extended axis emergency stop acceleration
    * @param [in] dec 485 extended axis emergency stop deceleration
    * @return  Error code
    */
    int AuxServoSetEmergencyStopAcc(double acc, double dec);

Get 485 extended axis movement acceleration and deceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: C#
    :linenos:

    /**
    * @brief Get 485 extended axis motion acceleration and deceleration 
    * @param [out] acc 485 extended axis motion acceleration
    * @param [out] dec 485 extended axis motion deceleration
    * @return  Error code
    */
    int AuxServoGetAcc(ref double acc, ref double dec);

Get 485 extended axis emergency stop acceleration and deceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: C#
    :linenos:

    /**
    * @brief Get 485 extended axis emergency stop acceleration and deceleration
    * @param [out] acc 485 extended axis emergency stop acceleration
    * @param [out] dec 485 extended axis emergency stop deceleration
    * @return  Error code
    */
    int AuxServoGetEmergencyStopAcc(ref double acc, ref double dec);

Extended axis control code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2

.. code-block:: c#
    :linenos:
    
    private void button64_Click(object sender, EventArgs e)
    {
        int retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 15.45);
        Console.WriteLine($"AuxServoSetParam is: {retval}");

        int servoCompany = 0;
        int servoModel = 0;
        int servoSoftVersion = 0;
        int servoResolution = 0;
        double axisMechTransRatio = 0;
        retval = robot.AuxServoGetParam(1, ref servoCompany, ref servoModel, ref servoSoftVersion, ref servoResolution, ref axisMechTransRatio);
        Console.WriteLine($"servoCompany {servoCompany}\n" +
            $"servoModel {servoModel}\n" +
            $"servoSoftVersion {servoSoftVersion}\n" +
            $"servoResolution {servoResolution}\n" +
            $"axisMechTransRatio {axisMechTransRatio}\n");

        retval = robot.AuxServoSetParam(1, 10, 11, 12, 13, 14);
        Console.WriteLine($"AuxServoSetParam is: {retval}");

        retval = robot.AuxServoGetParam(1, ref servoCompany, ref servoModel, ref servoSoftVersion, ref servoResolution, ref axisMechTransRatio);
        Console.WriteLine($"servoCompany {servoCompany}\n" +
            $"servoModel {servoModel}\n" +
            $"servoSoftVersion {servoSoftVersion}\n" +
            $"servoResolution {servoResolution}\n" +
            $"axisMechTransRatio {axisMechTransRatio}\n");

        retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 36);
        Console.WriteLine($"AuxServoSetParam is: {retval}");
        Thread.Sleep(3000);

        robot.AuxServoSetAcc(3000, 3000);
        robot.AuxServoSetEmergencyStopAcc(5000, 5000);
        Thread.Sleep(1000);
        double emagacc = 0, acc = 0;
        double emagdec = 0, dec = 0;
        robot.AuxServoGetEmergencyStopAcc(ref emagacc, ref emagdec);
        Console.WriteLine($"emergency acc is {emagacc}  dec is {emagdec}");
        robot.AuxServoGetAcc(ref acc, ref dec);
        Console.WriteLine($"acc is {acc}  dec is {dec}");

        robot.AuxServoSetControlMode(1, 0);
        Thread.Sleep(2000);

        retval = robot.AuxServoEnable(1, 0);
        Console.WriteLine($"AuxServoEnable disenable {retval}");
        Thread.Sleep(1000);
        int servoerrcode = 0;
        int servoErrCode = 0;
        int servoState = 0;
        double servoPos = 0;
        double servoSpeed = 0;
        double servoTorque = 0;
        retval = robot.AuxServoGetStatus(1, ref servoErrCode, ref servoState, ref servoPos, ref servoSpeed, ref servoTorque);
        Console.WriteLine($"AuxServoGetStatus servoState {servoState}");
        Thread.Sleep(1000);

        retval = robot.AuxServoEnable(1, 1);
        Console.WriteLine($"AuxServoEnable enable {retval}");
        Thread.Sleep(1000);
        retval = robot.AuxServoGetStatus(1, ref servoErrCode, ref servoState, ref servoPos, ref servoSpeed, ref servoTorque);
        Console.WriteLine($"AuxServoGetStatus servoState {servoState}");
        Thread.Sleep(1000);

        retval = robot.AuxServoHoming(1, 1, 5, 1);
        Console.WriteLine($"AuxServoHoming {retval}");
        Thread.Sleep(3000);

        retval = robot.AuxServoSetTargetPos(1, 200, 30);
        Console.WriteLine($"AuxServoSetTargetPos {retval}");
        Thread.Sleep(1000);
        retval = robot.AuxServoGetStatus(1, ref servoErrCode, ref servoState, ref servoPos, ref servoSpeed, ref servoTorque);
        Console.WriteLine($"AuxServoGetStatus servoSpeed {servoSpeed}");
        Thread.Sleep(8000);

        robot.AuxServoSetControlMode(1, 1);
        Thread.Sleep(2000);

        robot.AuxServoEnable(1, 0);
        Thread.Sleep(1000);
        robot.AuxServoEnable(1, 1);
        Thread.Sleep(1000);
        robot.AuxServoSetTargetSpeed(1, 100, 80);

        Thread.Sleep(5000);
        robot.AuxServoSetTargetSpeed(1, 0, 80);
    }

UDP extended axis communication parameter configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief UDP extended axis communication parameter configuration
    * @param [in] ip PLC IP address
    * @param [in] port  Port number
    * @param [in] period    Communication cycle (ms, default is 2, do not modify this parameter)
    * @param [in] lossPkgTime   Packet loss detection time (ms)
    * @param [in] lossPkgNum    Number of packet losses
    * @param [in] disconnectTime    Communication disconnection confirmation time
    * @param [in] reconnectEnable   Enable automatic reconnection after communication disconnect 0-Disabled 1-Enabled
    * @param [in] reconnectPeriod   Reconnection period (ms)
    * @param [in] reconnectNum  Reconnection count
    * @param [in] selfConnect Whether to automatically establish a connection after power failure; 0-Do not establish a connection; 1-Establish a connection
    * @return Error code
    */
    int ExtDevSetUDPComParam(std::string ip, int port, int period, int lossPkgTime, int lossPkgNum, int disconnectTime, int reconnectEnable, int reconnectPeriod, int reconnectNum, int selfConnect);

Get UDP Extension Axis Communication Parameter Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Get UDP extension axis communication parameters
    * @param [out] ip PLC IP address
    * @param [out] port	Port number
    * @param [out] period	Communication period (ms, default is 2, do not modify this parameter)
    * @param [out] lossPkgTime	Packet loss detection time (ms)
    * @param [out] lossPkgNum	Number of packet losses
    * @param [out] disconnectTime	Communication disconnection confirmation duration
    * @param [out] reconnectEnable	Automatic reconnection enable after communication disconnection 0-disable 1-enable
    * @param [out] reconnectPeriod	Reconnection interval (ms)
    * @param [out] reconnectNum	Number of reconnection attempts
    * @param [out] selfConnect	Whether to automatically reconnect after control box restart; 0-no reconnection; 1-reconnect
    * @return Error code
    */
    public int ExtDevGetUDPComParam(ref string ip, ref int port, ref int period, ref int lossPkgTime, ref int lossPkgNum, ref int disconnectTime, ref int reconnectEnable, ref int reconnectPeriod, ref int reconnectNum, ref int selfConnect)

Load UDP communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Load UDP communication
    * @return Error code
    */
    int ExtDevLoadUDPDriver();

Unload UDP communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Unload UDP communication
    * @return Error code
    */
    int ExtDevUnloadUDPDriver();

Restore connection after abnormal disconnection of UDP extended axis communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:
    
    /**
    * @brief Restore connection after abnormal disconnection of UDP extended axis communication
    * @return Error code
    */
    int ExtDevUDPClientComReset();

Close communication after abnormal disconnection of UDP extended axis communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Close communication after abnormal disconnection of UDP extended axis communication
    * @return Error code
    */
    int ExtDevUDPClientComClose();

UDP extended axis parameter configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief UDP extended axis parameter configuration
    * @param [in] axisID Axis number
    * @param [in] axisType Extended axis type 0-translation; 1-rotation
    * @param [in] axisDirection Extended axis direction 0-forward; 1-backward
    * @param [in] axisMax Maximum position of the extended axis mm
    * @param [in] axisMin Minimum position of the extended axis mm
    * @param [in] axisVel Speed mm/s
    * @param [in] axisAcc Acceleration mm/s2
    * @param [in] axisLead Lead mm
    * @param [in] encResolution Encoder resolution
    * @param [in] axisOffect Weld start point extension axis offset
    * @param [in] axisCompany Driver manufacturer 1-Hechuan; 2-Huichuan; 3-Panasonic
    * @param [in] axisModel Driver model 1-Hechuan-SV-XD3EA040L-E, 2-Hechuan-SV-X2EA150A-A, 1-Huichuan-SV620PT5R4I,1-Panasonic-MADLN15SG, 2-Panasonic-MSDLN25SG, 3-Panasonic-MCDLN35SG
    * @param [in] axisEncType Encoder type  0-Incremental; 1-Absolute
    * @return Error code
    */
    int ExtAxisParamConfig(int axisID, int axisType, int axisDirection, double axisMax, double axisMin, double axisVel, double axisAcc, double axisLead, long encResolution, double axisOffect, int axisCompany, int axisModel, int axisEncType);

Set the installation position of the extended axis
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set the installation position of the extended axis
    * @param [in] installType 0-robot installed on external axis, 1-robot installed outside external axis
    * @return Error code
    */
    int SetRobotPosToAxis(int installType);

Set extended axis system DH parameter configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set the DH parameter configuration for the extended axis system
    * @param [in]  axisConfig External axis configuration, 0-single-degree-of-freedom linear slide rail, 1-two-degree-of-freedom L-type indexer, 2-three-degree-of-freedom, 3-four-degree-of-freedom, 4-single-degree-of-freedom indexer
    * @param [in]  axisDHd1 External axis DH parameter d1 mm
    * @param [in]  axisDHd2 External axis DH parameter d2 mm
    * @param [in]  axisDHd3 External axis DH parameter d3 mm
    * @param [in]  axisDHd4 External axis DH parameter d4 mm
    * @param [in]  axisDHa1 External axis DH parameter 11 mm
    * @param [in]  axisDHa2 External axis DH parameter a2 mm
    * @param [in]  axisDHa3 External axis DH parameter a3 mm
    * @param [in]  axisDHa4 External axis DH parameter a4 mm
    * @return Error code
    */
    int SetAxisDHParaConfig(int axisConfig, double axisDHd1, double axisDHd2, double axisDHd3, double axisDHd4, double axisDHa1, double axisDHa2, double axisDHa3, double axisDHa4);

UDP extended axis enable
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief UDP extended axis enable
    * @param [in] axisID Axis number [1-4]
    * @param [in] status 0-disable; 1-enable
    * @return Error code
    */
    int ExtAxisServoOn(int axisID, int status);

UDP extended axis reset
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:
    
    /**
    * @brief UDP extended axis zero return
    * @param [in] axisID axis number [1-4]
    * @param [in] mode zero return mode 0-return to current position, 1-return to negative limit, 2-return to positive limit
    * @param [in] searchVel zero search speed (mm/s)
    * @param [in] latchVel Latching speed (mm/s)
    * @return Error code
    */
    int ExtAxisSetHoming(int axisID, int mode, double searchVel, double latchVel);

UDP extended axis jog start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief UDP extended axis jog start
    * @param [in] axisID Axis number [1-4]
    * @param [in] direction Rotation direction 0-reverse; 1-forward
    * @param [in] vel Speed (mm/s)
    * @param [in] acc Acceleration (mm/s2)
    * @param [in] maxDistance Maximum jog distance
    * @return Error code
    */
    int ExtAxisStartJog(int axisID, int direction, double vel, double acc, double maxDistance);
    
UDP extended axis jog stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief UDP extended axis jog stop
    * @param [in] axisID axis ID[1-4]
    * @return Error code
    */
    int ExtAxisStopJog(int axisID);

UDP extended axis configuration and jog code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: C#
    :linenos:

    private void btnJog_Click(object sender, EventArgs e)
    {
        int rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5,1);
        Console.WriteLine("ExtDevSetUDPComParam rtn is " + rtn);
        string ip = ""; int port = 0; int period = 0; int lossPkgTime = 0; int lossPkgNum = 0; int disconnectTime = 0; int reconnectEnable = 0; int reconnectPeriod = 0; int reconnectNum = 0;
        rtn = robot.ExtDevGetUDPComParam(ref ip, ref port, ref period, ref lossPkgTime, ref lossPkgNum, ref disconnectTime, ref reconnectEnable, ref reconnectPeriod, ref reconnectNum);
        string param = "\nip " + ip + "\nport " + port.ToString() + "\nperiod  " + period.ToString() + "\nlossPkgTime " + lossPkgTime.ToString() + "\nlossPkgNum  " + lossPkgNum.ToString() + "\ndisConntime  " + disconnectTime.ToString() + "\nreconnecable  " + reconnectEnable.ToString() + "\nreconnperiod  " + reconnectPeriod.ToString() + "\nreconnnun  " + reconnectNum.ToString();
        Console.WriteLine("ExtDevGetUDPComParam rtn is " + rtn + param);

        robot.ExtDevLoadUDPDriver();

        rtn = robot.ExtAxisServoOn(1, 1);
        Console.WriteLine("ExtAxisServoOn axis id 1 rtn is " + rtn);
        rtn = robot.ExtAxisServoOn(2, 1);
        Console.WriteLine("ExtAxisServoOn axis id 2 rtn is " + rtn);
        Thread.Sleep(2000);

        rtn = robot.ExtAxisSetHoming(1, 0, 10, 2);
        Console.WriteLine("ExtAxisSetHoming 1 rtnn is  " + rtn);
        Thread.Sleep(2000);
        rtn = robot.ExtAxisSetHoming(2, 0, 10, 2);
        Console.WriteLine("ExtAxisSetHoming 2 rtnn is  " + rtn);

        Thread.Sleep(4000);

        rtn = robot.SetRobotPosToAxis(1);
        Console.WriteLine("SetRobotPosToAxis rtn is " + rtn);
        rtn = robot.SetAxisDHParaConfig(10, 20, 0, 0, 0, 0, 0, 0, 0);
        Console.WriteLine("SetAxisDHParaConfig rtn is " + rtn);
        rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905f, 262144, 200, 1, 0, 0);
        Console.WriteLine("ExtAxisParamConfig axis 1 rtn is " + rtn);
        rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444f, 262144, 200, 1, 0, 0);
        Console.WriteLine("ExtAxisParamConfig axis 2 rtn is " + rtn);

        Thread.Sleep(3000);
        robot.ExtAxisStartJog(1, 0, 10, 10, 30);
        Thread.Sleep(1000);
        robot.ExtAxisStopJog(1);
        Thread.Sleep(3000);
        robot.ExtAxisServoOn(1, 0);

        Thread.Sleep(3000);
        robot.ExtAxisStartJog(2, 0, 10, 10, 30);
        Thread.Sleep(1000);
        robot.ExtAxisStopJog(2);
        Thread.Sleep(3000);
        robot.ExtAxisServoOn(2, 0);
        Thread.Sleep(3000);
        robot.ExtDevUnloadUDPDriver();
    }

Set the reference point of the extended axis coordinate system - four-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set extended axis coordinate system reference point - four-point method
    * @param [in]  pointNum Point number [1-4]
    * @return Error code
    */
    int ExtAxisSetRefPoint(int pointNum);

Calculate extended axis coordinate system - four-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Calculate extended axis coordinate system - four-point method
    * @param [out]  coord Coordinate system value
    * @return Error code
    */
    int ExtAxisComputeECoordSys(DescPose& coord);

Apply extended axis coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Apply extended axis coordinate system
    * @param [in]  applyAxisId Extended axis ID bit0-bit3 correspond to extended axis ID 1-4. For example, if extended axes 1 and 3 are applied, the value is 0b 0000 0101, which is 5
    * @param [in]  axisCoordNum Extended axis coordinate system ID
    * @param [in]  coord Coordinate system value
    * @param [in]  calibFlag Calibration flag 0-no, 1-yes
    * @return Error code
    */
    int ExtAxisActiveECoordSys(int applyAxisId, int axisCoordNum, DescPose coord, int calibFlag);

Set the calibration reference point in the end coordinate system of the positioner
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set the calibration reference point in the end position coordinate system of the positioner
    * @param [in] pos Position value
    * @return Error code
    */
    int SetRefPointInExAxisEnd(DescPose pos);

Set the reference point of the positioner coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set the reference point in the positioner coordinate system
    * @param [in]  pointNum Point number [1-4]
    * @return Error code
    */
    int PositionorSetRefPoint(int pointNum);

Positioner coordinate system calculation - four-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Positioner coordinate system calculation - four-point method
    * @param [out] coord Coordinate system value
    * @return Error code
    */
    int PositionorComputeECoordSys(DescPose& coord);

Get extended axis coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get extended axis coordinate system
    * @param [out] coord Extended axis coordinate system
    * @return Error code
    */
    int ExtAxisGetCoord(ref DescPose coord);

Extended axis coordinate system calibration code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    private void button66_Click(object sender, EventArgs e)
    {
        int rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5,1);
        Console.WriteLine("ExtDevSetUDPComParam rtn is " + rtn);
        string ip = ""; int port = 0; int period = 0; int lossPkgTime = 0; int lossPkgNum = 0; int disconnectTime = 0; int reconnectEnable = 0; int reconnectPeriod = 0; int reconnectNum = 0;
        rtn = robot.ExtDevGetUDPComParam(ref ip, ref port, ref period, ref lossPkgTime, ref lossPkgNum, ref disconnectTime, ref reconnectEnable, ref reconnectPeriod, ref reconnectNum);
        string param = "\nip " + ip + "\nport " + port.ToString() + "\nperiod  " + period.ToString() + "\nlossPkgTime " + lossPkgTime.ToString() + "\nlossPkgNum  " + lossPkgNum.ToString() + "\ndisConntime  " + disconnectTime.ToString() + "\nreconnecable  " + reconnectEnable.ToString() + "\nreconnperiod  " + reconnectPeriod.ToString() + "\nreconnnun  " + reconnectNum.ToString();
        Console.WriteLine("ExtDevGetUDPComParam rtn is " + rtn + param);

        robot.ExtDevLoadUDPDriver();

        rtn = robot.ExtAxisServoOn(1, 1);
        Console.WriteLine("ExtAxisServoOn axis id 1 rtn is " + rtn);
        rtn = robot.ExtAxisServoOn(2, 1);
        Console.WriteLine("ExtAxisServoOn axis id 2 rtn is " + rtn);
        Thread.Sleep(2000);

        robot.ExtAxisSetHoming(1, 0, 10, 2);
        Thread.Sleep(2000);
        rtn = robot.ExtAxisSetHoming(2, 0, 10, 2);
        Console.WriteLine("ExtAxisSetHoming rtnn is  " + rtn);

        Thread.Sleep(4000);

        rtn = robot.SetRobotPosToAxis(1);
        Console.WriteLine("SetRobotPosToAxis rtn is " + rtn);
        rtn = robot.SetAxisDHParaConfig(1, 128.5f, 206.4f, 0, 0, 0, 0, 0, 0);
        Console.WriteLine("SetAxisDHParaConfig rtn is " + rtn);
        rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905f, 262144, 200, 1, 0, 0);
        Console.WriteLine("ExtAxisParamConfig axis 1 rtn is " + rtn);
        rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444f, 262144, 200, 1, 0, 0);
        Console.WriteLine("ExtAxisParamConfig axis 1 rtn is " + rtn);

        DescPose toolCoord = new DescPose(0, 0, 210, 0, 0, 0);
        robot.SetToolCoord(1, toolCoord, 0, 0, 1, 0);

        JointPos jSafe = new JointPos(115.193f, -96.149f, 92.489f, -87.068f, -89.15f, -83.488f);
        JointPos j1 = new JointPos(117.559f, -92.624f, 100.329f, -96.909f, -94.057f, -83.488f);
        JointPos j2 = new JointPos(112.239f, -90.096f, 99.282f, -95.909f, -89.824f, -83.488f);
        JointPos j3 = new JointPos(110.839f, -83.473f, 93.166f, -89.22f, -90.499f, -83.487f);
        JointPos j4 = new JointPos(107.935f, -83.572f, 95.424f, -92.873f, -87.933f, -83.488f);

        DescPose descSafe = new DescPose();
        DescPose desc1 = new DescPose();
        DescPose desc2 = new DescPose();
        DescPose desc3 = new DescPose();
        DescPose desc4 = new DescPose();
        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

        robot.GetForwardKin( jSafe,  ref descSafe);
        robot.MoveJ( jSafe,  descSafe, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        Thread.Sleep(2000);

        robot.GetForwardKin( j1, ref desc1);
        robot.MoveJ( j1,  desc1, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        Thread.Sleep(2000);

        DescPose actualTCPPos = new DescPose();
        robot.GetActualTCPPose(0, ref actualTCPPos);
        robot.SetRefPointInExAxisEnd(actualTCPPos);
        rtn = robot.PositionorSetRefPoint(1);
        Console.WriteLine("PositionorSetRefPoint 1 rtn is " + rtn);
        Thread.Sleep(2000);

        robot.MoveJ( jSafe,  descSafe, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.ExtAxisStartJog(1, 0, 50, 50, 10);
        Thread.Sleep(1000);
        robot.ExtAxisStartJog(2, 0, 50, 50, 10);
        Thread.Sleep(1000);
        robot.GetForwardKin( j2, ref desc2);
        rtn = robot.MoveJ( j2,  desc2, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        rtn = robot.PositionorSetRefPoint(2);
        Console.WriteLine("PositionorSetRefPoint 2 rtn is " + rtn);
        Thread.Sleep(2000);

        robot.MoveJ( jSafe,  descSafe, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.ExtAxisStartJog(1, 0, 50, 50, 10);
        Thread.Sleep(1000);
        robot.ExtAxisStartJog(2, 0, 50, 50, 10);
        Thread.Sleep(1000);
        robot.GetForwardKin( j3, ref desc3);
        robot.MoveJ( j3,  desc3, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        rtn = robot.PositionorSetRefPoint(3);
        Console.WriteLine("PositionorSetRefPoint 3 rtn is " + rtn);
        Thread.Sleep(2000);

        robot.MoveJ( jSafe,  descSafe, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.ExtAxisStartJog(1, 0, 50, 50, 10);
        Thread.Sleep(1000);
        robot.ExtAxisStartJog(2, 0, 50, 50, 10);
        Thread.Sleep(1000);
        robot.GetForwardKin(j4, ref desc4);
        robot.MoveJ(j4, desc4, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        rtn = robot.PositionorSetRefPoint(4);
        Console.WriteLine("PositionorSetRefPoint 4 rtn is " + rtn);
        Thread.Sleep(2000);

        DescPose axisCoord = new DescPose();
        robot.PositionorComputeECoordSys(ref axisCoord);
        robot.MoveJ(jSafe, descSafe, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        Console.WriteLine("PositionorComputeECoordSys rtn is {0} {1} {2} {3} {4} {5}", axisCoord.tran.x, axisCoord.tran.y, axisCoord.tran.z, axisCoord.rpy.rx, axisCoord.rpy.ry, axisCoord.rpy.rz);
        rtn = robot.ExtAxisActiveECoordSys(3, 1, axisCoord, 1);
        Console.WriteLine("ExtAxisActiveECoordSys rtn is " + rtn);
    }

Extended axis motion via UDP
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.5  Web-3.8.4

.. code-block:: C#
    :linenos:

    /**
    * @brief Extended axis motion via UDP
    * @param [in] pos Target position
    * @param [in] ovl Speed percentage
    * @param [in] blend Smoothing parameter (mm or ms)
    * @return Error code
    */
    int ExtAxisMove(ExaxisPos pos, double ovl, double blend=-1);

UDP extended axis movement code example
++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: C#
    :linenos:

    private void button66_Click(object sender, EventArgs e)
    {
        ExaxisPos axisPos;
        axisPos.ePos[0] = 20;
        axisPos.ePos[1] = 0;
        axisPos.ePos[2] = 0;
        axisPos.ePos[3] = 0;
        robot.ExtAxisMove(axisPos, 50);
    }

UDP extended axis and robot joint motion synchronization
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief UDP extension axis and robot joint motion synchronization
    * @param [in] joint_pos Target joint position, unit deg
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool Tool coordinate number, range [0~14]
    * @param [in] user Workpiece coordinate number, range [0~14]
    * @param [in] vel Speed percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not currently available
    * @param [in] ovl Velocity scaling factor, range [0~100]
    * @param [in] epos Extended axis position, unit mm
    * @param [in] blendT [-1.0]-Move to position (blocked), [0~500.0]-Smooth time (non-blocked), unit ms
    * @param [in] offset_flag 0-no offset, 1-offset relative to base coordinate system/workpiece coordinate system, 2-offset relative to tool coordinate system
    * @param [in] offset_pos Pose offset
    * @return Error code
    */
    int ExtAxisSyncMoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos epos, float blendT, byte offset_flag, DescPose offset_pos);

Code example
++++++++++++++++++++++++++++++++++++++

.. code-block:: C#
    :linenos:
    
    private void btnSyncMoveJ_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");
        //1. Calibrate and apply the robot tool coordinate system. You can use the four-point method or six-point method to calibrate and apply the tool coordinate system. The interfaces involved in calibrating the tool coordinate system are as follows:

        //    int SetToolPoint(int point_num);  //Set tool reference point - six-point method
        //    int ComputeTool(ref DescPose tcp_pose);  //Calculate tool coordinate system
        //    int SetTcp4RefPoint(int point_num);    //Set tool reference point - four-point method

        //    int ComputeTcp4(ref DescPose tcp_pose);   // Calculate tool coordinate system - four-point method
        //    int SetToolCoord(int id, DescPose coord, int type, int install);  // Set application tool coordinate system
        //    int SetToolList(int id, DescPose coord, int type, int install);   // Set application tool coordinate system list

        //2. Set UDP communication parameters and load UDP communication 
        robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevLoadUDPDriver();
        //3. Set extended axis parameters, including extended axis type, extended axis driver parameters, and extended axis DH parameters
        robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //Single-axis positioner and DH parameters
        robot.SetRobotPosToAxis(1);  //Extended axis installation position
        robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); // Servo drive parameters. In this example, a single-axis positioner is used, so only one drive parameter needs to be set. If you select an extended axis type with multiple axes, each axis must have its drive parameters set.
        //4. Enable the selected axis and return to zero
        robot.ExtAxisServoOn(1, 0);

        robot.ExtAxisSetHoming(1, 0, 20, 3);
        //5. Perform extended axis coordinate system calibration and application (Note: The calibration interfaces for positioners and linear rails are different. The following is the calibration interface for positioners)
        DescPose pos = new DescPose(/* Enter your calibration point coordinates */);

        robot.SetRefPointInExAxisEnd(pos);
        robot.PositionorSetRefPoint(1); /* You need to calibrate the extended axis using four points at different positions, so this interface must be called four times to complete the calibration */
        DescPose coord = new DescPose( );
        robot.PositionorComputeECoordSys(ref coord); // Calculate the extended axis calibration results

        robot.ExtAxisActiveECoordSys(1, 1, coord, 1);  //Apply the calibration results to the extended axis coordinate system
        //6. Calibrate the workpiece coordinate system on the extended axis. You need to use the following interfaces
        //int SetWObjCoordPoint(int point_num);

        //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
        //int SetWObjCoord(int id, DescPose coord);
        //int SetWObjList(int id, DescPose coord);
        //7. Record the starting point of your synchronized joint motion

        DescPose startdescPose = new DescPose(/*Enter your coordinates*/);
        JointPos startjointPos = new JointPos(/*Enter your coordinates*/);
        ExaxisPos startexaxisPos = new ExaxisPos(/* Enter your extended axis starting point coordinates */);
        //8. Record the endpoint coordinates of your synchronized joint motion

        DescPose enddescPose = new DescPose(/*enter your coordinates*/);
        JointPos endjointPos = new JointPos(/*enter your coordinates*/);
        ExaxisPos endexaxisPos = new ExaxisPos(/* Enter your extended axis endpoint coordinates */);
        //9. Write the synchronized motion program
        //Move to the starting point, assuming the tool coordinate system and workpiece coordinate system are both 1
        robot.ExtAxisMove(startexaxisPos, 20);

        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
        robot.MoveJ(startjointPos, startdescPose, 1, 1, 100, 100, 100, startexaxisPos, 0, 0, offdese);

        //Start synchronized movement
        robot.ExtAxisSyncMoveJ(endjointPos, enddescPose, 1, 1, 100, 100, 100, endexaxisPos, -1, 0, offdese);
    }

UDP extended axis and robot linear motion synchronization
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief UDP extended axis and robot linear motion synchronization
    * @param [in] joint_pos  Target joint position, unit: deg
    * @param [in] desc_pos   Target Cartesian pose
    * @param [in] tool  Tool coordinate number, range[0~14]
    * @param [in] user  Workpiece coordinate index, range [0~14]
    * @param [in] vel  Velocity percentage, range [0~100]
    * @param [in] acc  Acceleration percentage, range [0~100], not currently available
    * @param [in] ovl  Velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-Move to position (blocked), [0~1000.0]-Smoothing radius
    (non-blocking), unit mm
    * @param [in] epos  Extended axis position, unit mm
    * @param [in] offset_flag  0-no offset, 1-offset relative to base coordinate system/workpiece coordinate system, 2-offset relative to tool coordinate system
    * @param [in] offset_pos  Position offset
    */
    int ExtAxisSyncMoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, int offset_flag, DescPose offset_pos);

Code example
++++++++++++++++++++++++++++++++++++++

.. code-block:: C#
    :linenos:

    private void btnSyncMoveL_Click(object sender, EventArgs e)
    {
    Robot robot = new Robot();
    robot.RPC("192.168.58.2");

    //1. Calibrate and apply the robot tool coordinate system. You can use the four-point method or six-point method to calibrate and apply the tool coordinate system. The interfaces involved in calibrating the tool coordinate system are as follows:
        //    int SetToolPoint(int point_num);  //Set the tool reference point - six-point method
        
    //    int ComputeTool(ref DescPose tcp_pose);  // Calculate tool coordinate system
        //    int SetTcp4RefPoint(int point_num);    // Set tool reference point - four-point method
        
    //    int ComputeTcp4(ref DescPose tcp_pose);   // Calculate tool coordinate system - four-point method
        //    int SetToolCoord(int id, DescPose coord, int type, int install);  // Set application tool coordinate system
    //    int SetToolList(int id, DescPose coord, int type, int install);   // Set application tool coordinate system list
        
    //2. Set UDP communication parameters and load UDP communication
    robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
    robot.ExtDevLoadUDPDriver();
    //3. Set extended axis parameters, including extended axis type, extended axis driver parameters, and extended axis DH parameters
    robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //Single-axis positioner and DH parameters
    robot.SetRobotPosToAxis(1);  // Extended axis installation position 
    robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //Servo drive parameters. In this example, a single-axis positioner is used, so only one drive parameter needs to be set. If you select an extended axis type with multiple axes, each axis must have its drive parameters set.
    //4.Enable the selected axis and set it to home position. 
    robot.ExtAxisServoOn(1, 0);
    robot.ExtAxisSetHoming(1, 0, 20, 3);
    //5. Perform extended axis coordinate system calibration and apply 
    DescPose pos = new DescPose(/* Enter your calibration point coordinates */);
    robot.SetRefPointInExAxisEnd(pos);
    robot.PositionorSetRefPoint(1); /* You need to calibrate the extended axis using four different points, so you must call this interface four times to complete the calibration */
        
    DescPose coord = new DescPose();
    robot.PositionorComputeECoordSys(ref coord); // Calculate the extended axis calibration results
    robot.ExtAxisActiveECoordSys(1, 1, coord, 1);  //Apply the calibration results to the extended axis coordinate system

    DescPose startdescPose = new DescPose(/*Enter your coordinates*/);
        
    JointPos startjointPos = new JointPos(/*Enter your coordinates*/);
        ExaxisPos startexaxisPos = new ExaxisPos(/* Enter your extended axis starting point coordinates */);
    //8. Record the endpoint coordinates of your synchronized linear motion
    DescPose enddescPose = new DescPose(/*Enter your coordinates*/);
        
    JointPos endjointPos = new JointPos(/*Enter your coordinates*/);
    ExaxisPos endexaxisPos = new ExaxisPos(/* Enter your extended axis endpoint coordinates */);
    //9. Write the synchronized motion program
    //Move to the starting point, assuming that the tool coordinate system and workpiece coordinate system are both 1
    robot.ExtAxisMove(startexaxisPos, 20);
    DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
    robot.MoveJ(startjointPos, startdescPose, 1, 1, 100, 100, 100, startexaxisPos, 0, 0, offdese);
        
    //Start synchronized movement
    robot.ExtAxisSyncMoveL(endjointPos, enddescPose, 1, 1, 100, 100, 100, 0, endexaxisPos, 0, offdese);
    }
    
UDP extended axis and robot arc motion synchronization
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:
    
    /**
    * @brief UDP extended axis and robot arc motion synchronization
    * @param [in] joint_pos_p  Path point joint position, unit deg
    * @param [in] desc_pos_p   Path point Cartesian pose
    * @param [in] ptool  Tool coordinate number, range [0~14]
    * @param [in] puser  Workpiece coordinate number, range [0~14]
    * @param [in] pvel  Speed percentage, range[0~100]
    * @param [in] pacc  Acceleration percentage, range [0~100], not currently available
    * @param [in] epos_p  Intermediate point extension axis position, unit mm
    * @param [in] poffset_flag  0-no offset, 1-offset relative to base coordinate system/workpiece coordinate system, 2-offset relative to tool coordinate system
    * @param [in] offset_pos_p  Position offset
    * @param [in] joint_pos_t  Target point joint position, unit deg
    * @param [in] desc_pos_t   Target point Cartesian position
    * @param [in] ttool  Tool coordinate number, range [0~14]
    * @param [in] tuser  Workpiece coordinate index, range [0~14]
    * @param [in] tvel  Velocity percentage, range [0~100]
    * @param [in] tacc  Acceleration percentage, range [0~100], not currently available
    * @param [in] epos_t  Extended axis position, unit mm
    * @param [in] toffset_flag  0-no offset, 1-offset relative to base coordinate system/workpiece coordinate system, 2-offset relative to tool coordinate system
    * @param [in] offset_pos_t Pose offset    
    * @param [in] ovl Speed scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-Move to position (blocked), [0~1000.0]-Smooth radius (unblocked), unit mm
    * @return Error code
    */
    int ExtAxisSyncMoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos epos_p, int poffset_flag, DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos epos_t, int toffset_flag, DescPose offset_pos_t, float ovl, float blendR);

Code example
++++++++++++++++++++++++++++++++++++++

.. code-block:: C#
    :linenos:
    
    private void btnSyncMoveC_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");
        //1. Calibrate and apply the robot tool coordinate system. You can use the four-point method or six-point method to calibrate and apply the tool coordinate system. The interfaces involved in calibrating the tool coordinate system are as follows:

        //    int SetToolPoint(int point_num);  //Set tool reference point - six-point method
        //    int ComputeTool(ref DescPose tcp_pose);  //Compute tool coordinate system
        //    int SetTcp4RefPoint(int point_num);    //Set tool reference point - four-point method

        //    int ComputeTcp4(ref DescPose tcp_pose);   // Calculate tool coordinate system - four-point method
        //    int SetToolCoord(int id, DescPose coord, int type, int install);  // Set application tool coordinate system
        //    int SetToolList(int id, DescPose coord, int type, int install);   // Set application tool coordinate system list

        //2. Set UDP communication parameters and load UDP communication
        robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevLoadUDPDriver();

        //3. Set the extended axis parameters, including extended axis type, extended axis driver parameters, and extended axis DH parameters
        robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //Single-axis positioner and DH parameters
        robot.SetRobotPosToAxis(1);  // Extended axis installation position 
        robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); // Servo drive parameters. In this example, it is a single-axis positioner, so only one drive parameter needs to be set. If you select an extended axis type with multiple axes, each axis must have its drive parameters set

        //4. Enable the selected axis and set it to home position
        robot.ExtAxisServoOn(1, 0);
        robot.ExtAxisSetHoming(1, 0, 20, 3);
        //5. Perform extended axis coordinate system calibration and application

        DescPose pos = new DescPose(/* Enter your calibration point coordinates */);
        robot.SetRefPointInExAxisEnd(pos);
        robot.PositionorSetRefPoint(1); /*You need to calibrate the extended axis using four different points, so you need to call this interface four times to complete the calibration */
        DescPose coord = new DescPose();

        robot.PositionorComputeECoordSys(ref coord); // Calculate the extended axis calibration results
        robot.ExtAxisActiveECoordSys(1, 1, coord, 1);  // Apply the calibration results to the extended axis coordinate system
        //6. Calibrate the workpiece coordinate system on the extended axis. You will need the following interfaces

        //int SetWObjCoordPoint(int point_num);
        //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
        //int SetWObjCoord(int id, DescPose coord);
        //int SetWObjList(int id, DescPose coord);
        //7. Record the starting point of your synchronized arc motion

        DescPose startdescPose = new DescPose(/*Enter your coordinates*/);
        JointPos startjointPos = new JointPos(/*Enter your coordinates*/);
        ExaxisPos startexaxisPos = new ExaxisPos(/* Enter your extended axis starting point coordinates */);
        //8. Record the endpoint coordinates of your synchronized circular arc motion

        DescPose endDescPose = new DescPose(/*Enter your coordinates*/);
        JointPos endJointPos = new JointPos(/*Enter your coordinates*/);

        ExaxisPos endexaxisPos = new ExaxisPos(/* Enter your extended axis endpoint coordinates */);
        //8. Record your synchronized arc motion midpoint coordinates
        DescPose middescPose = new DescPose(/* Enter your coordinates */);
        JointPos midjointPos = new JointPos(/* Enter your coordinates */);

        ExaxisPos midexaxisPos = new ExaxisPos(/* Enter the extended axis coordinates when the robot is at the arc midpoint */);
        //9. Write the synchronized motion program

        //Move to the starting point, assuming the tool coordinate system and workpiece coordinate system are both 1
        robot.ExtAxisMove(startexaxisPos, 20);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

        robot.MoveJ(startjointPos, startdescPose, 1, 1, 100, 100, 100, startexaxisPos, 0, 0, offdese);
        //Start synchronized motion
        robot.ExtAxisSyncMoveC(midjointPos, middescPose, 1, 1, 100, 100, midexaxisPos, 0, offdese, endjointPos, enddescPose, 1, 1, 100, 100, endexaxisPos, 0, offdese, 100, 0);
    }

Set extended DO
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set extended DO
    * @param [in] DONum DO number
    * @param [in] bOpen switch true-on; false-off
    * @param [in] smooth whether to smooth
    * @param [in] block whether to block
    * @return Error code
    */
    int SetAuxDO(int DONum, bool bOpen, bool smooth, bool block);

Set extended AO
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set extended AO
    * @param [in] AONum AO number
    * @param [in] value Analog value [0-4095]
    * @param [in] block Whether to block
    * @return Error code
    */
    int SetAuxAO(int AONum, double value, bool block);
            
Set extended DI input filter time
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set extended DI input filter time
    * @param [in] filterTime Filter time (ms)
    * @return Error code
    */
    int SetAuxDIFilterTime(int filterTime);

Set extended AI input filter time
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set extended AI input filter time
    * @param [in] filterTime Filter time (ms)
    * @return Error code
    */
    int SetAuxAIFilterTime(int filterTime);

Wait for extended DI input
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Wait for extended DI input
    * @param [in] DINum DI number
    * @param [in] bOpen Switch 0-off; 1-on
    * @param [in] time Maximum wait time (ms)
    * @param [in] errorAlarm Whether to continue movement
    * @return Error code
    */
    int WaitAuxDI(int DINum, bool bOpen, int time, bool errorAlarm);

Wait for extended AI input
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Wait for extended AI input
    * @param [in] AINum AI number
    * @param [in] sign 0-greater than; 1-less than
    * @param [in] value AI value
    * @param [in] time Maximum wait time (ms)
    * @param [in] errorAlarm Continue movement
    * @return Error code
    */
    int WaitAuxAI(int AINum, int sign, int value, int time, bool errorAlarm);

Get extended DI value
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Get extended DI value
    * @param [in] DINum DI number
    * @param [in] isNoBlock Whether to block
    * @param [out] isOpen 0-off; 1-on
    * @return Error code
    */
    int GetAuxDI(int DINum, bool isNoBlock, bool& isOpen);

Get extended AI value
+++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Get extended AI value
    * @param [in] AINum AI number
    * @param [in] isNoBlock Whether to block
    * @param [in] value Input value
    * @return Error code
    */
    int GetAuxAI(int AINum, bool isNoBlock, int& value);

Extended IO code example
++++++++++++++++++++++++++++++++++++++

.. code-block:: C#
    :linenos:

    private void btnAODO_Click(object sender, EventArgs e)
    {
        int rtn;
        for (int i = 0; i < 128; i++)
        {
            robot.SetAuxDO(i, true, false, true);
            Thread.Sleep(100);
        }
        for (int i = 0; i < 128; i++)
        {
            robot.SetAuxDO(i, false, false, true);
            Thread.Sleep(100);
        }

        for (int i = 0; i < 409; i++)
        {
            robot.SetAuxAO(0, i * 10, true);
            robot.SetAuxAO(1, 4095 - i * 10, true);
            robot.SetAuxAO(2, i * 10, true);
            robot.SetAuxAO(3, 4095 - i * 10, true);
            Thread.Sleep(10);
        }

        robot.SetAuxDIFilterTime(10);
        robot.SetAuxAIFilterTime(0, 10);

        for (int i = 0; i < 20; i++)
        {
            bool curValue = false;
            rtn = robot.GetAuxDI(i, false, ref curValue);
            Console.WriteLine("DI" + i + "   " + curValue);
        }
        int curValueAI = -1;
        for (int i = 0; i < 4; i++)
        {
            rtn = robot.GetAuxAI(i, true, ref curValueAI);
        }

        robot.WaitAuxDI(1, false, 1000, false);
        robot.WaitAuxAI(1, 1, 132, 1000, false);
    }

Enable movable device
+++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * @brief Enable movable device
    * @param enable false-disable; true-enable
    * @return Error code
    */
    int TractorEnable(bool enable);

Stop movable device movement
+++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * @brief Stop the movement of the movable device
    * @return Error code
    */
    int TractorStop();

Reset the movable device
+++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * @brief Movable device returns to zero
    * @return Error code
    */
    int TractorHoming();

Movable device moves in a straight line
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * @brief Movable device moves in a straight line
    * @param distance Linear movement distance (mm)
    * @param vel Linear movement speed percentage (0-100)
    * @return Error code
    */
    int TractorMoveL(double distance, double vel);

Circular motion of a movable device
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * @brief Circular motion of a movable device
    * @param radio Circular motion radius (mm)
    * @param angle Arc movement angle (°)
    * @param vel Linear movement speed percentage (0-100)
    * @return Error code
    */
    int TractorMoveC(double radio, double angle, double vel);

Code example
++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    private void button6_Click(object sender, EventArgs e)
    {
        int rtn;
        robot.ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10,1);
        robot.ExtDevLoadUDPDriver();
        rtn = robot.ExtAxisServoOn(1, 1);
        rtn = robot.ExtAxisServoOn(2, 1);
        Thread.Sleep(2000);
        robot.ExtAxisSetHoming(1, 0, 10, 2);
        Thread.Sleep(2000);
        rtn = robot.ExtAxisSetHoming(2, 0, 10, 2);
        Thread.Sleep(4000);
        robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280f, 16384, 200, 0, 0, 0);
        robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280f, 16384, 200, 0, 0, 0);
        robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0);
        robot.TractorEnable(false);
        Thread.Sleep(2000);
        robot.TractorEnable(true);
        Thread.Sleep(2000);
        robot.TractorHoming();
        Thread.Sleep(2000);
        robot.TractorMoveL(100, 2);
        Thread.Sleep(5000);
        robot.TractorStop();
        robot.TractorMoveL(-100, 20);
        Thread.Sleep(5000);
        robot.TractorMoveC(300, 90, 20);
        Thread.Sleep(10000);
        robot.TractorMoveC(300, -90, 20);
        Thread.Sleep(1000);
        robot.TractorStop();    
    }

Set the synchronous motion strategy of the extension axis and the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:


    /**
    * @brief Set the synchronous movement strategy of the extended axis with the robot
    * @param strategy Strategy; 0 - Robot as the main; 1 - Extended axis synchronous with the robot
    * @return Error code */

    int SetExAxisRobotPlan(int strategy)

Code example for setting up the extended axis to move synchronously with the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:


    private void button94_Click(object sender, EventArgs e)
    {
        JointPos joint_pos1 = new JointPos(-22.016, -49.217, 124.714, -161.100, -85.108, -0.333);
        JointPos joint_pos2 = new JointPos(-21.083, -46.613, 110.079, -147.796, -80.757, -0.330);
        JointPos joint_pos3 = new JointPos(-25.572, -60.090, 135.397, -163.889, -82.489, -0.345);
        DescPose desc_pos1 = new DescPose(2.637, -0.001, 30.673, 178.786, -4.134, 68.326);
        DescPose desc_pos2 = new DescPose(213.812, -1.440, 47.311, 177.410, 0.166, 68.946);
        DescPose desc_pos3 = new DescPose(444.342, -12.723, 82.470, -177.701, -1.325, 65.151);   
        ExaxisPos epos1 = new ExaxisPos(0.001, 0.000, 0.000, 0.000);
        ExaxisPos epos2 = new ExaxisPos(299.977, 0.000, 0.000, 0.000);
        ExaxisPos epos3 = new ExaxisPos(399.969, 0.000, 0.000, 0.000);      
        DescPose offset_pos = new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        int rtn = robot.SetExAxisRobotPlan(0);
        Console.WriteLine($"SetExAxisRobotPlan rtn is {rtn}");
        Thread.Sleep(1000);
        rtn = robot.ExtAxisSyncMoveL(joint_pos1, desc_pos1, 1, 0, 100, 100, 100, -1, epos1, 0, offset_pos);
        Console.WriteLine($"ExtAxisSyncMoveL 1 rtn is {rtn}");

        rtn = robot.ExtAxisSyncMoveL(joint_pos2, desc_pos2, 1, 0, 100, 100, 100, -1, epos2, 0, offset_pos);
        Console.WriteLine($"ExtAxisSyncMoveL 2 rtn is {rtn}");
        rtn = robot.ExtAxisSyncMoveL(joint_pos3, desc_pos3, 1, 0, 100, 100, 100, -1, epos3, 0, offset_pos);
        Console.WriteLine($"ExtAxisSyncMoveL 3 rtn is {rtn}");
        Thread.Sleep(8000);
    }

UDP Extension Axis Positioning Completion Time Setting
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief UDP extension axis positioning completion time setting
    * @param [in] time Positioning completion time [ms]
    * @return Error code
    */
    public int SetExAxisCmdDoneTime(double time)
