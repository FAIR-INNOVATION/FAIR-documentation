extended axis
=================

.. toctree:: 
    :maxdepth: 5

Setting the 485 extension axis parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos.

    /** 
    * @brief Setting 485 extended axis parameters
    * @param [in] servoId Servo drive ID, range [1-15], corresponds to slave ID. 
    * @param [in] servoCompany Servo Drive Manufacturer, 1 - Dynatec
    * @param [in] servoModel Servo Drive Model, 1-FD100-750C
    * @param [in] servoSoftVersion Servo drive software version, 1-V1.0
    * @param [in] servoResolution encoder resolution
    * @param [in] axisMechTransRatio Mechanical Transmission Ratio
    * @return error code 
    */
    int AuxServoSetParam(int servoId, int servoCompany, int servoModel, int servoSoftVersion, int servoResolution, double axisMechTransRatio);
    
Getting 485 Extended Axis Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Getting 485 Extended Axis Configuration Parameters
    * @param [in] servoId Servo drive ID, range [1-15], corresponds to slave ID. 
    * @param [out] servoCompany Servo Drive Manufacturer, 1 - Dynatec
    * @param [out] servoModel Servo drive model, 1-FD100-750C
    * @param [out] servoSoftVersion Servo drive software version, 1-V1.0
    * @param [out] servoResolution encoder resolution
    * @param [out] axisMechTransRatio Mechanical Transmission Ratio
    * @return error code 
    */
    int AuxServoGetParam(int servoId, ref int servoCompany, ref int servoModel, ref int servoSoftVersion, ref int servoResolution, ref double axisMechTransRatio);
    
Setting the 485 expansion axis enable/disable
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting 485 extension axis enable/disable
    * @param [in] servoId Servo drive ID, range [1-15], corresponds to slave ID. 
    * @param [in] status enable status, 0-de-enable, 1-enable
    * @return error code 
    */
    int AuxServoEnable(int servoId, int status).
        
Setting the 485 extended axis control mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting the 485 Extended Axis Control Mode
    * @param [in] servoId Servo drive ID, range [1-15], corresponds to slave ID. 
    * @param [in] mode Control mode, 0-position mode, 1-velocity mode
    * @return error code 
    */
    int AuxServoSetControlMode(int servoId, int mode).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    private void btnWeldStart_Click(object sender, EventArgs e)
    {
    Robot robot = new Robot();
    robot.RPC("192.168.58.2");

    robot.AuxServoSetParam(1, 1, 1, 1, 1, 131072, 36);//set configuration parameters
    int ID = -1, company = -1, model = -1, soft = -1, servoResolution= -1;
    int radio = -1;
        robot.AuxServoGetParam(1, ref company, ref model, ref soft, ref servoResolution, ref radio);//get configuration parameters
        
        Thread.Sleep(100);
        robot.AuxServoEnable(1, 0);//de-enable servo
        Thread.Sleep(100);
        robot.AuxServoSetControlMode(1, 0);//set position mode
        Thread.Sleep(100);
        robot.AuxServoEnable(1, 1);//enable servo
    }

Setting the 485 extended axis back to zero
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting 485 extension axis back to zero.
    * @param [in] servoId Servo drive ID, range [1-15], corresponds to slave ID. 
    * @param [in] mode Return to zero mode, 1 - current position return to zero, 2 - negative limit return to zero, 3 - positive limit return to zero
    * @param [in] searchVel Return-to-zero velocity, mm/s or °/s
    * @param [in] latchVel Hoop speed, mm/s or °/s
    * @return error code 
    */
    int AuxServoHoming(int servoId, int mode, double searchVel, double latchVel);

Setting the 485 extended axis target position (position mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting the 485 extended axis target position (position mode)
    * @param [in] servoId Servo drive ID, range [1-15], corresponds to slave ID. 
    * @param [in] pos target position, mm or °
    * @param [in] speed Target speed, mm/s or °/s
    * @return error code 
    */
    int AuxServoSetTargetPos(int servoId, double pos, double speed);

Setting the 485 extended axis target speed (velocity mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting the 485 extended axis target speed (velocity mode)
    * @param [in] servoId Servo drive ID, range [1-15], corresponds to slave ID. 
    * @param [in] speed Target speed, mm/s or °/s
    * @return error code 
    */
    int AuxServoSetTargetSpeed(int servoId, double speed).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    private void btnWeldStart_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        robot.AuxServoEnable(1, 0);//de-enable
        Thread.Sleep(100);
        robot.AuxServoSetControlMode(1, 0);//set position mode
        Thread.Sleep(100);
        robot.AuxServoEnable(1, 1);//enable
        Thread.Sleep(100);
        robot.AuxServoHoming(1, 1, 20, 5);//back to zero
        Thread.Sleep(4000);//servo back to zero needs some time
                
        robot.AuxServoSetTargetPos(1, 1000, 100);//set target position
        Thread.Sleep(1000);
        robot.AuxServoSetTargetPos(1, 0, 100);//set target position again
        Thread.Sleep(1000);

        robot.AuxServoEnable(1, 0);//de-enable
        Thread.Sleep(100);
        robot.AuxServoSetControlMode(1, 1);//set speed mode
        Thread.Sleep(100);
        robot.AuxServoEnable(1, 1);//enable
        Thread.Sleep(100);
        robot.AuxServoHoming(1, 1, 20, 5);//back to zero
        Thread.Sleep(4000);//returning to zero takes some time
        robot.AuxServoSetTargetSpeed(1, 50);//set target speed
        Thread.Sleep(3000);

        robot.AuxServoSetTargetSpeed(1, -300);//set target speed
        Thread.Sleep(3000);
        robot.AuxServoSetTargetSpeed(1, 0);//Servo Stop
        Thread.Sleep(100);
    }
    
Setting the 485 extended axis target torque (torque mode) - not yet available
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Setting the 485 extended axis target torque (torque mode) - not yet available
    * @param [in] servoId Servo drive ID, range [1-15], corresponds to slave ID. 
    * @param [in] torque Target torque, Nm
    * @return error code 
    */
    int AuxServoSetTargetTorque(int servoId, double torque).
        
Clearing 485 Expansion Axis Error Messages
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Clear 485 extended axis error message
    * @param [in] servoId Servo drive ID, range [1-15], corresponds to slave ID. 
    * @return error code 
    */
    int AuxServoClearError(int servoId).

Get 485 Extended Axis Servo Status
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Getting 485 extended axis servo status
    * @param [in] servoId Servo drive ID, range [1-15], corresponds to slave ID. 
    * @param [out] servoErrCode Servo Drive Error Code
    * @param [out] servoState servo drive state bit0:0-not enabled; 1-enabled; bit1:0-not moving; 1-moving; bit2 0-positive limit not triggered; 1-positive limit triggered; bit3 0-negative limit not triggered; 1-negative limit triggered; bit4 0-positioning not completed; 1-positioning complete; bit5:0 -not returned to zero; 1-return to zero complete
    * @param [out] servoPos Servo current position mm or °.
    * @param [out] servoSpeed servo current speed mm/s or °/s
    * @param [out] servoTorque Servo current torque Nm
    * @return error code 
    */
    int AuxServoGetStatus(int servoId, ref int servoErrCode, ref int servoState, ref double servoPos, ref double servoSpeed, ref double servoTorque);
    
Setting the 485 Extended Axis Data Axis Number in Status Feedback
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Sets the 485 extended axis data axis number in the status feedback.
    * @param [in] servoId Servo drive ID, range [1-15], corresponds to slave ID. 
    * @return error code 
    */
    int AuxServosetStatusID(int servoId).

code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    private void btnWeldStart_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

            robot.AuxServoClearError(1);
        int errCode = 0;
        int servoState = 0;
        double pos = 0;
        double speed = 0;
        double torque = 0;
        robot.AuxServoGetStatus(1, ref errCode, ref servoState, ref pos, ref speed, ref torque);

        robot.AuxServosetStatusID(1);
        ROBOT_STATE_PKG pKG = new ROBOT_STATE_PKG();
        robot.GetRobotRealTimeState(ref pKG);
        Console.WriteLine($"the state is {pKG.auxState.servoPos}");
    }

Configuration of UDP extended axis communication parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief UDP extended axis communication parameter configuration
    * @param [in] ip PLC IP address
    * @param [in] port	 Port number
    * @param [in] period	 Communication period (ms, default is 2, please do not modify this parameter)
    * @param [in] lossPkgTime	 Packet loss detection time (ms)
    * @param [in] lossPkgNum	 Number of packets lost
    * @param [in] disconnectTime	 Disconnect time.
    * @param [in] reconnectEnable	 Auto reconnect enable for communication disconnection 0-don't enable 1-enable
    * @param [in] reconnectPeriod	 reconnect period interval (ms)
    * @param [in] reconnectNum	 reconnectNum
    * @return error code
    */
    int ExtDevSetUDPComParam(std::string ip, int port, int period, int lossPkgTime, int lossPkgNum, int disconnectTime, int reconnectEnable, int reconnectPeriod, int reconnectNum);
        
Get UDP extended axis communication parameter configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Get UDP extended axis communication parameters.
    * @param [out] ip PLC IP address
    * @param [out] port	 Port number.
    * @param [out] period	 Communication period (ms, default is 2, please do not modify this parameter)
    * @param [out] lossPkgTime	 Packet loss detection time (ms)
    * @param [out] lossPkgNum	 Number of packets lost
    * @param [out] disconnectTime	 Communication disconnect time
    * @param [out] reconnectEnable	 Auto reconnect enable for communication disconnection 0-don't enable 1-enable
    * @param [out] reconnectPeriod	 reconnect period interval (ms)
    * @param [out] reconnectNum	 reconnectNum
    * @return error code
    */
    int ExtDevGetUDPComParam(std::string& ip, int& port, int& period, int& lossPkgTime, int& lossPkgNum, int& disconnectTime, int& reconnectEnable, int& reconnectPeriod, int& reconnectNum);
        
Load UDP communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * :: @brief Load UDP communications
    * @return error code
    */
    int ExtDevLoadUDPDriver(); int ExtDevLoadUDPDriver().

Offloading UDP communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Uninstalling UDP communication
    * @return error code
    */
    int ExtDevUnloadUDPDriver();

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++****

.. code-block:: c#
    :linenos:

    private void btnSetParam_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        string ip = "";
        int port = 0;
        int period = 0;
        int checktime = 0;
        int checknum = 0;
        int disconntime = 0;
        int reconnenable = 0;
        int reconntime = 0;
        int reconnnum = 0;
        robot.ExtDevGetUDPComParam(ref ip, ref port, ref period, ref checktime, ref checknum, ref disconntime, ref reconntime, ref reconnenable, ref reconnnum);
        Console.Writeline($"{ip} {port} {period} {checktime} {checknum} {disconntime} {reconnenable} {reconntime} {reconnnum}");

        robot.ExtDevLoadUDPDriver();
        Thread.Sleep(1000 * 10);
        robot.ExtDevUnloadUDPDriver();
    }

Recovering connection after abnormal disconnection of UDP extension axis communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * :: @brief UDP Extended Axis communication restored after abnormal disconnection
    * @return error code
    */
    int ExtDevUDPClientComReset();

UDP extension axis communication is closed after abnormal disconnection.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief UDP extended axis communication closed after abnormal disconnection of communication
    * @return error code
    */
    int ExtDevUDPClientComClose();

UDP Extended Axis Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief UDP extended axis parameter configuration
    * @param [in] axisID axis number
    * @param [in] axisType Extended axis type 0-translation; 1-rotation
    * @param [in] axisDirection extended axis direction 0-positive; 1-direction 
    * @param [in] axisMax Extended axis max position mm
    * @param [in] axisMin Extended axis minimum position mm
    * @param [in] axisVel velocity mm/s
    * @param [in] axisAcc Acceleration mm/s2
    * @param [in] axisLead guide mm
    * @param [in] encResolution encoder resolution
    * @param [in] axisOffect Extended axis offset from weld start point
    * @param [in] axisCompany drive manufacturers 1-Hochuan; 2-Huichuan; 3-Panasonic
    * @param [in] axisModel Drive Model 1-Hochuan-SV-XD3EA040L-E, 2-Hochuan-SV-X2EA150A-A, 1-HuiChuan-SV620PT5R4I, 1-Panasonic-MADLN15SG, 2-Panasonic-MSDLN25SG, 3-Panasonic-MCDLN35SG
    * @param [in] axisEncType Encoder type 0-incremental; 1-absolute
    * @return error code
    */
    int ExtAxisParamConfig(int axisID, int axisType, int axisDirection, double axisMax, double axisMin, double axisVel, double axisAcc, double axisLead, long encResolution, double axisOffect, int axisCompany, int axisModel, int axisEncType);

Setting the Extension Shaft Mounting Position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the Extension Shaft Mounting Position
    * @param [in] installType 0 - robot mounted on external axis, 1 - robot mounted outside external axis
    * @return error code
    */
    int SetRobotPosToAxis(int installType).

Setting the extended axis system DH parameter configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Set up extended axis system DH parameter configuration
    * @param [in] axisConfig external axis configuration, 0 - single degree of freedom linear slide, 1 - two degree of freedom L-type translator, 2 - three degrees of freedom, 3 - four degrees of freedom, 4 - single degree of freedom translator
    * @param [in] axisDHd1 external axisDH parameter d1 mm
    * @param [in] axisDHd2 external axisDH parameter d2 mm
    * @param [in] axisDHd3 external axisDH parameter d3 mm
    * @param [in] axisDHd4 external axisDH parameter d4 mm
    * @param [in] axisDHa1 external axisDH parameter11 mm
    * @param [in] axisDHa2 external axisDH parameter a2 mm
    * @param [in] axisDHa3 external axisDH parameter a3 mm
    * @param [in] axisDHa4 external axisDH parameter a4 mm
    * @return error code
    */
    int SetAxisDHParaConfig(int axisConfig, double axisDHd1, double axisDHd2, double axisDHd3, double axisDHd4, double axisDHa1, double axisDHa2, double axisDHa3, double axisDHa4);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    private void btnSetAxisParam_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        int rtn = 0;
        rtn = robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0);
        Console.WriteLine($"SetAxisDHParaConfig rtn is {rtn}");
        rtn = robot.SetRobotPosToAxis(1);
        Console.WriteLine($"SetRobotPosToAxis rtn is {rtn}");
        rtn = robot.ExtAxisParamConfig(1,0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0);
        Console.WriteLine($"ExtAxisParamConfig rtn is {rtn}");
    }

Setting the reference point of the extended axis coordinate system - four-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * :: @brief Setting the reference point of the extended axis coordinate system -- four-point method
    * @param [in] pointNum pointNum [1-4]
    * @return error code
    */
    int ExtAxisSetRefPoint(int pointNum).

Calculating Extended Axis Coordinate Systems - Four Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * :: @brief Calculation of the extended axis coordinate system -- four-point method
    * @param [out] coord coordinate system value
    * @return error code
    */
    int ExtAxisComputeECoordSys(DescPose& coord);

Applying the Extended Axis Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * :: @brief Applying the extended axis coordinate system
    * @param [in] applyAxisId extension axis number bit0-bit3 corresponds to extension axis number 1-4, e.g. 0b 0000 0101 for application of extension axes 1 and 3; that is, 5
    * @param [in] axisCoordNum extended axis coordinate system number
    * @param [in] coord coordinate system value
    * @param [in] calibFlag calibration flag 0-no, 1-yes
    * @return error code
    */
    int ExtAxisActiveECoordSys(int applyAxisId, int axisCoordNum, DescPose coord, int calibFlag);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**

.. code-block:: c#
    :linenos:

    private void btnCoordCalib_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        robot.ExtAxisSetRefPoint(1);
        //robot.ExtAxisSetRefPoint(2);
        //robot.ExtAxisSetRefPoint(3);
        //robot.ExtAxisSetRefPoint(4);
        //DescPose pos = new DescPose();
        //robot.ExtAxisComputeECoordSys(ref pos);
    }

Setting of the calibration reference point in the position in the coordinate system of the end of the translator
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting of the calibration reference point in the position in the coordinate system of the end of the translator
    * @param [in] pos position value
    * @return error code
    */
    int SetRefPointInExAxisEnd(DescPose pos).

Setting of reference points for the coordinate system of the indexing machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting of reference points for the coordinate system of the translocator
    * @param [in] pointNum pointNum [1-4]
    * @return error code
    */
    int PositionorSetRefPoint(int pointNum).

Shifter Coordinate System Calculation - Four Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Translator Coordinate System Calculation - Four Point Method
    * @param [out] coord coordinate system value
    * @return error code
    */
    int PositionorComputeECoordSys(DescPose& coord);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**

.. code-block:: c#
    :linenos:

    private void btnCoordCalib_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        DescPose refPointPos = new DescPose(122.0, 312.0, 0, 0, 0, 0);
        robot.SetRefPointInExAxisEnd(refPointPos);

        robot.PositionorSetRefPoint(1);
        //robot.PositionorSetRefPoint(2);
        //robot.PositionorSetRefPoint(3);
        //robot.PositionorSetRefPoint(4);

        //DescPose coord = new DescPose();
        //robot.PositionorComputeECoordSys(ref coord);
    }

UDP Extended Axis Enable
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief UDP Extended Axis Enabled
    * @param [in] axisID axis number [1-4]
    * @param [in] status 0-de-enable; 1-enable
    * @return error code
    */
    int ExtAxisServoOn(int axisID, int status).

UDP extension axis back to zero
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief UDP extension axis back to zero.
    * @param [in] axisID axis number [1-4]
    * @param [in] mode Zero return mode 0 - current position zero return, 1 - negative limit zero return, 2 - positive limit zero return
    * @param [in] searchVel Zero search velocity (mm/s)
    * @param [in] latchVel Zeroing hoop speed (mm/s)
    * @return error code
    */
    int ExtAxisSetHoming(int axisID, int mode, double searchVel, double latchVel);

UDP Extended Axis Tap Start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief UDP Extended Axis Tap Starts
    * @param [in] axisID axis number [1-4]
    * @param [in] direction direction of rotation 0-reverse; 1-forward
    * @param [in] vel velocity (mm/s)
    * @param [in] acc Acceleration (mm/s2)
    * @param [in] maxDistance Maximum pointing distance
    * @return error code
    */
    int ExtAxisStartJog(int axisID, int direction, double vel, double acc, double maxDistance);
    
UDP Extended Axis Tap Stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief UDP Extended Axis Tap Stop
    * @param [in] axisID axis number [1-4]
    * @return error code
    */
    int ExtAxisStopJog(int axisID).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**

.. code-block:: c#
    :linenos:

    private void btnJog_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        robot.ExtAxisServoOn(1, 1);
        robot.ExtAxisSetHoming(1, 0, 10, 3);
        robot.ExtAxisStartJog(1, 1, 100, 100, 100, 20);
        Thread.Sleep(1000 * 2);
        robot.ExtAxisStopJog(1);
        robot.ExtAxisServoOn(1, 0);
    }

UDP Extended Axis Motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief UDP Extended Axis Motion
    * @param [in] pos target position
    * @param [in] ovl speed percentage
    * @return error code
    */
    int ExtAxisMove(ExaxisPos pos, double ovl);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**

.. code-block:: c#
    :linenos:

    private void btnAxisMove_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        ExaxisPos pos = new ExaxisPos(10, 0, 0, 0);
        robot.ExtAxisMove(pos, 10);
    }

Synchronised motion of UDP extension axes with robot joint motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief UDP extension axis synchronised motion with robot joint motion
    * @param [in] joint_pos Target joint position in deg.
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool tool coordinate number in the range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] epos Extended axis position in mm
    * @param [in] blendT [-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in ms
    * @param [in] offset_flag 0-no offset, 1-base/workpiece coordinate system offset, 2-tool coordinate system offset
    * @param [in] offset_pos Bit position offset
    * @return error code
    */
    int ExtAxisSyncMoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos epos, float blendT, byte offset_flag, DescPose offset_pos);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**

.. code-block:: c#
    :linenos:

    private void btnSyncMoveJ_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        //1. Calibrate and apply the robot's tool coordinate system. You can use the four-point or six-point method for calibrating and applying the tool coordinate system. The interfaces involved in calibrating the tool coordinate system are as follows:
        // int SetToolPoint(int point_num); // Set tool reference point - six-point method
        // int ComputeTool(ref DescPose tcp_pose); // Compute Tool coordinate system
        // int SetTcp4RefPoint(int point_num); // Set tool reference point - four point method
        // int ComputeTcp4(ref DescPose tcp_pose); // Compute tool coordinate system - four-point method
        // int SetToolCoord(int id, DescPose coord, int type, int install); // set the application tool coordinate system
        // int SetToolList(int id, DescPose coord, int type, int install); // set the list of application tool coordinate systems

        //2. Setting UDP communication parameters and loading UDP communication
        robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevLoadUDPDriver();

        //3. Set the extended axis parameters, including extended axis type, extended axis driver parameters, and extended axis DH parameters.
        robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0, 0); //Single-axis variator and DH parameters
        robot.SetRobotPosToAxis(1); //extended axis mounting position
        robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //servo drive parameters, this example is a single-axis variant of the positioner, so you need to set up only one drive parameter, if you choose to include multiple axes of the type of extended axis, you need to set up the drive parameters for each axis

        //4. Setting the selected axis to enable and return to zero.
        robot.ExtAxisServoOn(1, 0);
        robot.ExtAxisSetHoming(1, 0, 20, 3);

        /5. Extended axis coordinate system calibration and application (Note: the calibration interface of the indexing machine and the linear slide is different, the following is the calibration interface of the indexing machine).
        DescPose pos = new DescPose(/* Enter the coordinates of your calibration point */);
        robot.SetRefPointInExAxisEnd(pos);
        robot.PositionorSetRefPoint(1); /* You need to calibrate the extended axis by four differently positioned points, so you need to call this interface 4 times to complete the calibration */
        DescPose coord = new DescPose( );
        robot.PositionorComputeECoordSys(ref coord); //calculate extended axis calibration results
        robot.ExtAxisActiveECoordSys(1, 1, coord, 1); //apply the calibration results to the extended axis coordinate system

        //6. To calibrate the workpiece coordinate system in the extended axes, you need the following interfaces
        //int SetWObjCoordPoint(int point_num);
        //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
        //int SetWObjCoord(int id, DescPose coord);
        //int SetWObjList(int id, DescPose coord);

        //7. Record your synchronised joint movement starting point
        DescPose startdescPose = new DescPose(/* enter your coordinates**/);
        JointPos startjointPos = new JointPos(/* enter your coordinates**/);
        ExaxisPos startexaxisPos = new ExaxisPos(/* Enter the coordinates of your extended axis start point */);

        //8. Record the coordinates of the end point of your synchronised joint movement.
        DescPose enddescPose = new DescPose(/* enter your coordinates**/);
        JointPos endjointPos = new JointPos(/* enter your coordinates**/);
        ExaxisPos endexaxisPos = new ExaxisPos(/* Enter your extended axis endpoint coordinates */);

        //9. Writing synchronous motion programmes
        //Movement to the starting point, assuming that the applied tool and workpiece coordinate systems are 1
        robot.ExtAxisMove(startexaxisPos, 20);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);
        robot.MoveJ(startjointPos, startdescPose, 1, 1, 100, 100, 100, 100, startexaxisPos, 0, 0, offdese);

        // Start of synchronised movement
        robot.ExtAxisSyncMoveJ(endjointPos, enddescPose, 1, 1, 100, 100, 100, 100, endexaxisPos, -1, 0, offdese);
    }

Synchronised motion of the UDP extension axes with the linear motion of the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief UDP extended axes synchronised motion with robot linear motion
    * @param [in] joint_pos Target joint position in deg.
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool tool coordinate number in the range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm
    * @param [in] epos Extended axis position in mm
    * @param [in] offset_flag 0-no offset, 1-base/workpiece coordinate system offset, 2-tool coordinate system offset
    * @param [in] offset_pos Bit position offset
    * @return error code
    */
    int ExtAxisSyncMoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, int offset_flag, DescPose offset_pos).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**

.. code-block:: c#
    :linenos:

    private void btnSyncMoveL_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

    //1. Calibrate and apply the robot's tool coordinate system. You can use the four-point or six-point method for calibrating and applying the tool coordinate system. The interfaces involved in calibrating the tool coordinate system are as follows:
        // int SetToolPoint(int point_num); // Set tool reference point - six-point method
        // int ComputeTool(ref DescPose tcp_pose); // Compute Tool coordinate system
        // int SetTcp4RefPoint(int point_num); // Set tool reference point - four point method
        // int ComputeTcp4(ref DescPose tcp_pose); // Compute tool coordinate system - four-point method
        // int SetToolCoord(int id, DescPose coord, int type, int install); // set the application tool coordinate system
        // int SetToolList(int id, DescPose coord, int type, int install); // set the list of application tool coordinate systems

        //2. Setting UDP communication parameters and loading UDP communication
        robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevLoadUDPDriver();

        //3. Set the extended axis parameters, including extended axis type, extended axis driver parameters, and extended axis DH parameters.
        robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0, 0); //Single-axis variator and DH parameters
        robot.SetRobotPosToAxis(1); //extended axis mounting position
        robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //servo drive parameters, this example is a single-axis variant of the positioner, so you need to set up only one drive parameter, if you choose to include multiple axes of the type of extended axis, you need to set up the drive parameters for each axis

        //4. Setting the selected axis to enable and return to zero.
        robot.ExtAxisServoOn(1, 0);
        robot.ExtAxisSetHoming(1, 0, 20, 3);

        /5. Carrying out calibration and application of extended axis coordinate systems
        DescPose pos = new DescPose(/* Enter the coordinates of your calibration point */);
        robot.SetRefPointInExAxisEnd(pos);
        robot.PositionorSetRefPoint(1); /* You need to calibrate the extended axis by four differently positioned points, so you need to call this interface 4 times to complete the calibration */
        DescPose coord = new DescPose();
        robot.PositionorComputeECoordSys(ref coord); //calculate extended axis calibration results
        robot.ExtAxisActiveECoordSys(1, 1, coord, 1); //apply the calibration results to the extended axis coordinate system

        //6. To calibrate the workpiece coordinate system in the extended axes, you need the following interfaces
        //int SetWObjCoordPoint(int point_num);
        //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
        //int SetWObjCoord(int id, DescPose coord);
        //int SetWObjList(int id, DescPose coord);

        //7. Record the start point of your synchronised linear motion
        DescPose startdescPose = new DescPose(/* enter your coordinates**/);
        JointPos startjointPos = new JointPos(/* enter your coordinates**/);
        ExaxisPos startexaxisPos = new ExaxisPos(/* Enter the coordinates of your extended axis start point */);

        //8. Record the coordinates of the end point of your synchronised linear motion.
        DescPose enddescPose = new DescPose(/* enter your coordinates**/);
        JointPos endjointPos = new JointPos(/* enter your coordinates**/);
        ExaxisPos endexaxisPos = new ExaxisPos(/* enter your extended axis endpoint coordinates */);

        //9. Writing synchronous motion programmes
        //Movement to the starting point, assuming that the applied tool and workpiece coordinate systems are 1
        robot.ExtAxisMove(startexaxisPos, 20);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0, 0);
        robot.MoveJ(startjointPos, startdescPose, 1, 1, 100, 100, 100, 100, startexaxisPos, 0, 0, offdese);

        // Start of synchronised movement
        robot.ExtAxisSyncMoveL(endjointPos, enddescPose, 1, 1, 100, 100, 100, 0, endexaxisPos, 0, offdese);
    }
    
UDP extension axes synchronised with robot circular motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief UDP extension axis synchronised with robot circular motion
    * @param [in] joint_pos_p Path point joint position in deg.
    * @param [in] desc_pos_p Path point Cartesian pose
    * @param [in] ptool tool coordinate number, range [0~14]
    * @param [in] puser Workpiece coordinate number, range [0~14].
    * @param [in] pvel speed percentage, range [0~100]
    * @param [in] pacc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos_p Extended axis position of the midpoint in mm
    * @param [in] poffset_flag 0-no offset, 1-base/workpiece coordinate system offset, 2-tool coordinate system offset
    * @param [in] offset_pos_p bit position offset
    * @param [in] joint_pos_t Joint position of target point, in deg.
    * @param [in] desc_pos_t Target point Cartesian pose
    * @param [in] ttool tool coordinate number, in the range [0 to 14].
    * @param [in] tuser Workpiece coordinate number, range [0~14].
    * @param [in] tvel speed percentage, range [0~100]
    * @param [in] tacc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos_t Extended axis position in mm
    * @param [in] toffset_flag 0-no offset, 1-base/workpiece coordinate system offset, 2-tool coordinate system offset
    * @param [in] offset_pos_t bit position offset	 
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm
    * @return error code
    */
    int ExtAxisSyncMoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos epos_p, int poffset_flag, DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos epos_t, int toffset_flag, DescPose offset_pos_t, float ovl, float blendR).
    
code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**

.. code-block:: c#
    :linenos:

    private void btnSyncMoveC_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

    //1. Calibrate and apply the robot's tool coordinate system. You can use the four-point or six-point method for calibrating and applying the tool coordinate system. The interfaces involved in calibrating the tool coordinate system are as follows:
        // int SetToolPoint(int point_num); // Set tool reference point - six-point method
        // int ComputeTool(ref DescPose tcp_pose); // Compute Tool coordinate system
        // int SetTcp4RefPoint(int point_num); // Set tool reference point - four point method
        // int ComputeTcp4(ref DescPose tcp_pose); // Compute tool coordinate system - four-point method
        // int SetToolCoord(int id, DescPose coord, int type, int install); // set the application tool coordinate system
        // int SetToolList(int id, DescPose coord, int type, int install); // set the list of application tool coordinate systems

        //2. Setting UDP communication parameters and loading UDP communication
        robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevLoadUDPDriver();

        //3. Set the extended axis parameters, including extended axis type, extended axis driver parameters, and extended axis DH parameters.
        robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0, 0); //Single-axis variator and DH parameters
        robot.SetRobotPosToAxis(1); //extended axis mounting position
        robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //servo drive parameters, this example is a single-axis variant of the positioner, so you need to set up only one drive parameter, if you choose to include multiple axes of the type of extended axis, you need to set up the drive parameters for each axis

        //4. Setting the selected axis to enable and return to zero.
        robot.ExtAxisServoOn(1, 0);
        robot.ExtAxisSetHoming(1, 0, 20, 3);

        /5. Perform extended axis coordinate system calibration and application
        DescPose pos = new DescPose(/* Enter the coordinates of your calibration point */);
        robot.SetRefPointInExAxisEnd(pos);
        robot.PositionorSetRefPoint(1); /* You need to calibrate the extended axis by four differently positioned points, so you need to call this interface 4 times to complete the calibration */
        DescPose coord = new DescPose();
        robot.PositionorComputeECoordSys(ref coord); //calculate extended axis calibration results
        robot.ExtAxisActiveECoordSys(1, 1, coord, 1); //apply the calibration results to the extended axis coordinate system

        //6. To calibrate the workpiece coordinate system in the extended axes, you need the following interfaces
        //int SetWObjCoordPoint(int point_num);
        //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
        //int SetWObjCoord(int id, DescPose coord);
        //int SetWObjList(int id, DescPose coord);

        //7. Record the start point of your synchronised circular motion
        DescPose startdescPose = new DescPose(/* enter your coordinates**/);
        JointPos startjointPos = new JointPos(/* enter your coordinates**/);
        ExaxisPos startexaxisPos = new ExaxisPos(/* Enter the coordinates of your extended axis start point */);

        //8. Record the coordinates of the end point of your synchronised circular motion.
        DescPose enddescPose = new DescPose(/* enter your coordinates**/);
        JointPos endjointPos = new JointPos(/* enter your coordinates**/);
        ExaxisPos endexaxisPos = new ExaxisPos(/* enter your extended axis endpoint coordinates */);

        //8. Record the coordinates of the middle point of your synchronised circular motion.
        DescPose middescPose = new DescPose(/* Enter your coordinates */);
        JointPos midjointPos = new JointPos(/* enter your coordinates**/);
        ExaxisPos midexaxisPos = new ExaxisPos(/* Enter the coordinates of the extended axes at the midpoint of the robot arc */);

        //9. Writing synchronous motion programmes
        //Movement to the starting point, assuming that the applied tool and workpiece coordinate systems are 1
        robot.ExtAxisMove(startexaxisPos, 20);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0, 0);
        robot.MoveJ(startjointPos, startdescPose, 1, 1, 100, 100, 100, 100, startexaxisPos, 0, 0, offdese);

        // Start of synchronised movement
        robot.ExtAxisSyncMoveC(midjointPos, middescPose, 1, 1, 100, 100, midexaxisPos, 0, offdese, endjointPos, enddescPose, 1, 1, 100, 100, endexaxisPos , 0, offdese, 100, 0).
    }

Setting Up the Weld Wire Seek Expansion IO Port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting up the Weld Wire Seeker Expansion IO Port
    * @param searchDoneDINum Solder Wire Successful DO Port (0-127)
    * @param searchStartDONum Welding wire seek start/stop control DO port (0-127)
    * @return error code
    */
    int SetWireSearchExtDIONum(int searchDoneDINum, int searchStartDONum);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**
.. versionchanged:: C#SDK-v1.0.9
    
.. code-block:: c#
    :linenos:

    private void button7_Click(object sender, EventArgs e)
    {
        /UDP wire seeker
        robot.ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10);
        robot.ExtDevLoadUDPDriver();
        robot.SetWireSearchExtDIONum(0, 0);

        int rtn0, rtn1, rtn2 = 0;
        ExaxisPos exaxisPos = new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offdese = new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        DescPose descStart = new DescPose(-158.767, -510.596, 271.709, -179.427, -0.745, -137.349);
        JointPos jointStart = new JointPos(61.667, -79.848, 108.639, -119.682, -89.700, -70.985);

        DescPose descEnd = new DescPose(0.332, -516.427, 270.688, 178.165, 0.017, -119.989);
        JointPos jointEnd = new JointPos(79.021, -81.839, 110.752, -118.298, -91.729, -70.981);

        robot.MoveL(jointStart, descStart, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);
        robot.MoveL(jointEnd, descEnd, 1, 0, 100, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);

        DescPose descREF0A = new DescPose(-66.106, -560.746, 270.381, 176.479, -0.126, -126.745);
        JointPos jointREF0A = new JointPos(73.531, -75.588, 102.941, -116.250, -93.347, -69.689);

        DescPose descREF0B = new DescPose(-66.109, -528.440, 270.407, 176.479, -0.129, -126.744);
        JointPos jointREF0B = new JointPos(72.534, -79.625, 108.046, -117.379, -93.366, -70.687);

        DescPose descREF1A = new DescPose(72.975, -473.242, 270.399, 176.479, -0.129, -126.744);
        JointPos jointREF1A = new JointPos(87.169, -86.509, 115.710, -117.341, -92.993, -56.034);
        DescPose descREF1B = new DescPose(31.355, -473.238, 270.405, 176.480, -0.130, -126.745);
        JointPos jointREF1B = new JointPos(82.117, -87.146, 116.470, -117.737, -93.145, -61.090);
        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF0A, descREF0A, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese); //starting point
        robot.MoveL(jointREF0B, descREF0B, 1, 0, 10, 100, 100, -1, exaxisPos, 1, 0, offdese); //direction point
        rtn1 = robot.WireSearchWait("REF0");
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF1A, descREF1A, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese); //starting point
        robot.MoveL(jointREF1B, descREF1B, 1, 0, 10, 100, 100, -1, exaxisPos, 1, 0, offdese); //direction point
        rtn1 = robot.WireSearchWait("REF1");
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF0A, descREF0A, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese); //starting point
        robot.MoveL(jointREF0B, descREF0B, 1, 0, 10, 100, 100, -1, exaxisPos, 1, 0, offdese); //direction point
        rtn1 = robot.WireSearchWait("RES0");
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF1A, descREF1A, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese); //starting point
        robot.MoveL(jointREF1B, descREF1B, 1, 0, 10, 100, 100, -1, exaxisPos, 1, 0, offdese); //direction point
        rtn1 = robot.WireSearchWait("RES1");
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
        List<string> varNameRef1 = new List<string> { "REF0", "REF1", "#", "#", "#", "#" };
        List<string> varNameRes1 = new List<string> { "RES0", "RES1", "#", "#", "#", "#" };
        string[] varNameRef = varNameRef1.ToArray();
        string[] varNameRes = varNameRes1.ToArray();
        int offectFlag = 0;
        DescPose offectPos = new DescPose(0, 0, 0, 0, 0, 0, 0);
        rtn0 = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes, ref offectFlag, ref offectPos);
        robot.PointsOffsetEnable(0, offectPos);
        robot.MoveL(jointStart, descStart, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);
        robot.MoveL(jointEnd, descEnd, 1, 0, 100, 100, 100, -1, exaxisPos, 1, 0, offdese);
        robot.PointsOffsetDisable();
    }

Set welder control mode to expand DO port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting up welder control mode to expand DO ports
    * @param DONum Welder control mode DO port (0-127)
    * @return error code
    */
    int SetWeldMachineCtrlModeExtDoNum(int DONum).

Setting the welder control mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the welder control mode
    * @param mode Welder control mode;0-unitary
    * @return error code
    */
    int SetWeldMachineCtrlMode(int mode).

code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9
    
.. code-block:: c#
    :linenos:

    private void button8_Click(object sender, EventArgs e)
    {
        robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10);
        robot.ExtDevLoadUDPDriver();

        robot.SetWeldMachineCtrlModeExtDoNum(17);
        for (int i = 0; i < 5; i++)
        {
            robot.SetWeldMachineCtrlMode(1);
            Thread.Sleep(500);
            robot.SetWeldMachineCtrlMode(0);
            Thread.Sleep(500);
        }

        robot.SetWeldMachineCtrlModeExtDoNum(18);
        for (int i = 0; i < 5; i++)
        {
            robot.SetWeldMachineCtrlMode(1);
            Thread.Sleep(500);
            robot.SetWeldMachineCtrlMode(0);
            Thread.Sleep(500);
        }
        robot.SetWeldMachineCtrlModeExtDoNum(19);
        for (int i = 0; i < 5; i++)
        {
            robot.SetWeldMachineCtrlMode(1);
            Thread.Sleep(500);
            robot.SetWeldMachineCtrlMode(0);
            Thread.Sleep(500);
        }
    }

Removable Device Enable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * @brief Removable device enable
    * @param enable false-de-enable; true-enable
    * @return error code
    */
    int TractorEnable(bool enable).

Stopping motion of movable units
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * :: @brief Stopping motion of movable devices
    * @return error code
    */
    int TractorStop();

Zeroing of removable units
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * @brief Zeroing of removable devices
    * @return error code
    */
    int TractorHoming().

Movable unit linear motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * @brief Movable unit linear motion
    * @param distance Linear motion distance (mm)
    * :: @param vel Percentage of linear motion speed (0-100)
    * @return error code
    */
    int TractorMoveL(double distance, double vel).

Movable unit circular motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9

.. code-block:: c#
    :linenos:

    /**
    * @brief Movable unit circular motion
    * :: @param radio radius of circular motion (mm)
    * @param angle Angle of circular motion (°)
    * :: @param vel Percentage of linear motion speed (0-100)
    * @return error code
    */
    int TractorMoveC(double radio, double angle, double vel).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.9
    
.. code-block:: c#
    :linenos:

    private void button6_Click(object sender, EventArgs e)
    {
        robot.ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10);
        int tru = robot.ExtDevLoadUDPDriver();
        Thread.Sleep(2000);
        Console.WriteLine("tru" + tru);
        robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
        robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
        robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        int tru1 = robot.TractorEnable(true);
        Thread.Sleep(3000);
        robot.TractorHoming().
        Thread.Sleep(2000);
        robot.TractorMoveL(100, 20);
        Thread.Sleep(2000);
        robot.TractorMoveL(-100, 20);
        Thread.Sleep(2000);
        robot.TractorMoveC(50, 60, 20).
        Thread.Sleep(2000);
        robot.TractorMoveC(50, -60, 20).
        Thread.Sleep(1000);
        robot.TractorStop();//stop in the middle of the process
    }

Setting the Extended DO
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * :: @brief set up extended DO
    * @param [in] DONum DO No.
    * @param [in] bOpen switch true-on; false-off
    * @param [in] smooth Whether smooth or not
    * @param [in] block whether to block or not
    * @return error code
    */
    int SetAuxDO(int DONum, bool bOpen, bool smooth, bool block);
        
Setting up Extended AO
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * :: @brief sets up extended AO
    * @param [in] AONum AO number 
    * @param [in] value Analogue value [0-4095]
    * @param [in] block whether to block or not
    * @return error code
    */
    int SetAuxAO(int AONum, double value, bool block);
    
code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    private void btnAODO_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        for (int i = 0; i < 128; i++)
        {
            robot.SetAuxDO(i, true, false, true);
            Thread.Sleep(200);
        }

        for(int i = 0; i < 409; i++)
        {
            robot.SetAuxAO(0, i * 10, true);
            robot.SetAuxAO(1, 4095 - i * 10, true);
            robot.SetAuxAO(2, i * 10, true);
            robot.SetAuxAO(3, 4095 - i * 10, true);
            Thread.Sleep(10);
        }
    }
            
Setting the Extended DI Input Filter Time
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the extended DI input filter time
    * @param [in] filterTime filterTime (ms)
    * @return error code
    */
    int SetAuxDIFilterTime(int filterTime).

Setting the extended AI input filter time
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the extended AI input filter time
    * @param [in] filterTime filterTime (ms)
    * @return error code
    */
    int SetAuxAIFilterTime(int filterTime).

Waiting for extended DI input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Waiting for extended DI input
    * @param [in] DINum DI number
    * @param [in] bOpen switch 0-Off; 1-On
    * @param [in] time Maximum wait time (ms)
    * @param [in] errorAlarm whether to continue the movement or not
    * @return error code
    */
    int WaitAuxDI(int DINum, bool bOpen, int time, bool errorAlarm);
    
Waiting for extended AI input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Waiting for extended AI input
    * @param [in] AINum AI number
    * @param [in] sign 0 - greater than; 1 - less than
    * @param [in] value AI value
    * @param [in] time Maximum wait time (ms)
    * @param [in] errorAlarm whether to continue the movement or not
    * @return error code
    */
    int WaitAuxAI(int AINum, int sign, int value, int time, bool errorAlarm);
        
Get Extended DI Value
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Get extended DI value
    * @param [in] DINum DI number
    * @param [in] isNoBlock whether to block or not
    * @param [out] isOpen 0-Off; 1-Open
    * @return error code
    */
    int GetAuxDI(int DINum, bool isNoBlock, bool& isOpen);
            
Get Extended AI Value
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.7

.. code-block:: c#
    :linenos:

    /**
    * @brief Get extended AI value
    * @param [in] AINum AI number
    * @param [in] isNoBlock whether to block or not
    * @param [in] value Input value
    * @return error code
    */
    int GetAuxAI(int AINum, bool isNoBlock, int& value);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnAIDI_Click(object sender, EventArgs e)
    {
    Robot robot = new Robot();
    robot.RPC("192.168.58.2");

    robot.SetAuxDIFilterTime(10);
    robot.SetAuxAIFilterTime(10);

    for (int i = 0; i < 20; i++)
    {
        bool curValue = false;
        int rtn = robot.GetAuxDI(i, false, ref curValue);
        txtRtn.Text = rtn.ToString();
        Console.Write($"DI{i} {curValue} ");
        Console.WriteLine(" ");
    }

    int curValue = -1;
    int rtn = 0;
    for (int i = 0; i < 4; i++)
    {
        rtn = robot.GetAuxAI(i, true, ref curValue);
        txtRtn.Text = rtn.ToString();
        Console.Write($"AI{i} {curValue} rtn is {rtn} ");
        Console.WriteLine(" ");
    }

    robot.WaitAuxDI(1, true, 1000, false);
    robot.WaitAuxAI(1, 1, 132, 1000, false);
    }