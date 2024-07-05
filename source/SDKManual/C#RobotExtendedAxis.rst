Extended Axis
=================

.. toctree:: 
    :maxdepth: 5

Set 485 extended axis parameters
++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6

.. code-block:: c#
    :linenos:
    
    /** 
    * @brief Set 485 extended axis parameters
    * @param [in] servoId servoId servo drive ID, range [1-15], corresponding slave ID 
    * @param [in] servoCompany Servo drive manufacturer, 1-Dynatec
    * @param [in] servoModel servo drive model, 1-FD100-750C
    * @param [in] servoSoftVersion servo driver software version, 1-V1.0
    * @param [in] servoResolution encoder resolution
    * @param [in] axisMechTransRatio mechanical transmission ratio
    * @return error code 
    */
    int AuxServoSetParam(int servoId, int servoCompany, int servoModel, int servoSoftVersion, int servoResolution, double axisMechTransRatio);

Get 485 extended axis configuration parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get 485 extended axis configuration parameters
    * @param [in] servoId servoId servo drive ID, range [1-15], corresponding slave ID 
    * @param [out] servoCompany Servo drive manufacturer, 1-Dynatec
    * @param [out] servoModel servo drive model, 1-FD100-750C
    * @param [out] servoSoftVersion servo driver software version, 1-V1.0
    * @param [out] servoResolution encoder resolution
    * @param [out] axisMechTransRatio mechanical transmission ratio
    * @return error code 
    */
    int AuxServoGetParam(int servoId, ref int servoCompany, ref int servoModel, ref int servoSoftVersion, ref int servoResolution, ref double axisMechTransRatio);

Set 485 expansion axis enable/disable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set 485 expansion axis enable/disable
    * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID 
    * @param [in] status enable status, 0-disabled, 1-enabled
    * @return error code 
    */
    int AuxServoEnable(int servoId, int status);

Set 485 extended axis control mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set 485 extended axis control mode
    * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID 
    * @param [in] mode control mode, 0-position mode, 1-speed mode
    * @return error code
    */
    int AuxServoSetControlMode(int servoId, int mode);

Code example
****************

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    private void btnWeldStart_Click(object sender, EventArgs e)
    {
    Robot robot = new Robot();
    robot.RPC("192.168.58.2");

    robot.AuxServoSetParam(1, 1, 1, 1, 131072, 36);//set param
    int ID = -1, company = -1, model = -1, soft = -1, servoResolution= -1;
    int radio = -1;
        robot.AuxServoGetParam(1, ref company, ref model, ref soft, ref servoResolution, ref radio);//get param
        
        Thread.Sleep(100);
        robot.AuxServoEnable(1, 0);//unable
        Thread.Sleep(100);
        robot.AuxServoSetControlMode(1, 0);//set position mode
        Thread.Sleep(100);
        robot.AuxServoEnable(1, 1);//enable
    }


Set the 485 extended axis target position (position mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set the 485 extended axis target position (position mode)
    * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID 
    * @param [in] pos pos target position, mm or °
    * @param [in] speed speed target speed, mm/s or °/s
    * @return error code
    */
    int AuxServoSetTargetPos(int servoId, double pos, double speed);

Set the 485 extended axis target speed (speed mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set the 485 extended axis target speed (speed mode)
    * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID 
    * @param [in] speed target speed, mm/s or °/s
    * @return error code
    */
    int AuxServoSetTargetSpeed(int servoId, double speed);

Code example
****************
.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    private void btnWeldStart_Click(object sender, EventArgs e)
    {
    Robot robot = new Robot();
    robot.RPC("192.168.58.2");

        robot.AuxServoEnable(1, 0);//unable
        Thread.Sleep(100);
        robot.AuxServoSetControlMode(1, 0);//set position mode
        Thread.Sleep(100);
        robot.AuxServoEnable(1, 1);//enbale
        Thread.Sleep(100);
        robot.AuxServoHoming(1, 1, 20, 5);//homing
        Thread.Sleep(4000);//It takes time to homing
                
        robot.AuxServoSetTargetPos(1, 1000, 100);//set target pos
        Thread.Sleep(1000);
        robot.AuxServoSetTargetPos(1, 0, 100);//set target pos
        Thread.Sleep(1000);

        robot.AuxServoEnable(1, 0);//unable
        Thread.Sleep(100);
        robot.AuxServoSetControlMode(1, 1);//set speed mode
        Thread.Sleep(100);
        robot.AuxServoEnable(1, 1);//enbale
        Thread.Sleep(100);
        robot.AuxServoHoming(1, 1, 20, 5);//homing
        Thread.Sleep(4000);//It takes time to homing
        robot.AuxServoSetTargetSpeed(1, 50);//set target speed
        Thread.Sleep(3000);

        robot.AuxServoSetTargetSpeed(1, -300);//set target speed
        Thread.Sleep(3000);
        robot.AuxServoSetTargetSpeed(1, 0);//servo stop
        Thread.Sleep(100);
    }


Set 485 extended axis target torque (torque mode)-- Not open yet
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set 485 extended axis target torque (torque mode)-- Not open yet
    * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID 
    * @param [in] torque target torque, Nm
    * @return error code
    */
    int AuxServoSetTargetTorque(int servoId, double torque);

Set 485 extended axis servo homing
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set 485 extended axis servo homing
    * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID
    * @param [in] mode homing mode, 0-current position homing; 1-limit homing
    * @param [in] searchVel search homing speed, mm/s or °/s
    * @param [in] latchVel latch homing speed，mm/s或°/s
    * @return error code
    */
    int AuxServoHoming(int servoId, int mode, double searchVel, double latchVel);

Clear 485 extended axis error message
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Clear 485 extended axis error message
    * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID 
    * @return error code
    */
    int AuxServoClearError(int servoId);

Get 485 extended axis servo status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get 485 extended axis servo status
    * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID 
    * @param [out] servoErrCode servo drive error code
    * @param [out] servoState servo drive status [decimal number converted to binary, bit0-bit5: servo enable-servo running-positive limittrigger-negative limit trigger-positioning completed-zero return completed
    * @param [out] servoPos servo current position mm or °
    * @param [out] servoSpeed Servo current speed mm/s or °/s
    * @param [out] servoTorque Servo current torque Nm
    * @return error code
    */
    int AuxServoGetStatus(int servoId, ref int servoErrCode, ref int servoState, ref double servoPos, ref double servoSpeed, ref double servoTorque);

Set the 485 extended axis data axis number in status feedback
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set the 485 extended axis data axis number in status feedback
    * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID 
    * @return error code
    */
    int AuxServosetStatusID(int servoId);

Code example
+++++++++++++++

.. versionadded:: C# SDK-v1.0.6
    
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

Configure UDP extension axis communication parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Configure UDP extension axis communication parameters
    * @param [in] ip PLC IP address
    * @param [in] port	 port num
    * @param [in] period	Communication period(ms，default 2ms)
    * @param [in] lossPkgTime	Packet loss detection time(ms)
    * @param [in] lossPkgNum	the number of packet loss times
    * @param [in] disconnectTime	the duration of communication disconnection confirmation
    * @param [in] reconnectEnable	 Automatic reconnection when communication is disconnected Enable;0-Disable, 1-Enable
    * @param [in] reconnectPeriod	 Reconnection period(ms)
    * @param [in] reconnectNum Reconnection times
    * @return error code
    */
    int ExtDevSetUDPComParam(std::string ip, int port, int period, int lossPkgTime, int lossPkgNum, int disconnectTime, int reconnectEnable, int reconnectPeriod, int reconnectNum);
        
Get the UDP extension axis communication parameter configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Get the UDP extension axis communication parameter configuration
    * @param [out] ip PLC IP address
    * @param [out] port port num
    * @param [out] period	Communication period(ms，default 2ms)
    * @param [out] lossPkgTime Packet loss detection time(ms)
    * @param [out] lossPkgNum	 Indicates the number of packet loss times
    * @param [out] disconnectTime	 the duration of communication disconnection confirmation
    * @param [out] reconnectEnable Automatic reconnection when communication is disconnected Enable;0-Disable, 1-Enable
    * @param [out] reconnectPeriod Reconnection period(ms)
    * @param [out] reconnectNum	Reconnection times
    * @return error code
    */
    int ExtDevGetUDPComParam(std::string& ip, int& port, int& period, int& lossPkgTime, int& lossPkgNum, int& disconnectTime, int& reconnectEnable, int& reconnectPeriod, int& reconnectNum);
        
Load the UDP communication connection
+++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Load the UDP communication connection
    * @return error code
    */
    int ExtDevLoadUDPDriver();

Unload the UDP communication connection
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Unload the UDP communication connection
    * @return error code
    */
    int ExtDevUnloadUDPDriver();

Code example
**************

.. code-block:: C#
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
        Console.Writeline($"{ip}  {port}  {period} {checktime}  {checknum}  {disconntime}  {reconnenable}  {reconntime}  {reconnnum}");

        robot.ExtDevLoadUDPDriver();
        Thread.Sleep(1000 * 10);
        robot.ExtDevUnloadUDPDriver();
    }

Reconnect UDP communication after abnormal disconnected
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Reconnect UDP communication after abnormal disconnected
    * @return error code
    */
    int ExtDevUDPClientComReset();

Close UDP communication after abnormal disconnected
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Close UDP communication after abnormal disconnected
    * @return error code
    */
    int ExtDevUDPClientComClose();

Configure UDP extension axis parameters
+++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Configure UDP extension axis parameters
    * @param [in] axisID Axis number [1-4]
    * @param [in] axisType Extended axis type; 0-translation, 1-rotation
    * @param [in] axisDirection Axis direction; 0-forward; 1-reverse
    * @param [in] axisMax The maximum position of the extension axis(mm)
    * @param [in] axisMin Minimum position of the extension axis (mm)
    * @param [in] axisVel Speed mm/s
    * @param [in] axisAcc Acceleration mm/s2
    * @param [in] axisLead Lead mm
    * @param [in] encResolution Encoder resolution
    * @param [in] axisOffect The start point of the weld extension axis offset
    * @param [in] axisCompany Driver manufacturer 1-Hechuan; 2- Huichuan; 3- Panasonic
    * @param [in] axisModel Driver models 1-Hechuan SV-XD3EA040L-E, 2-Hechuan SV-X2EA150A-A, 1-Huichuan SV620PT5R4I, 1-Matsushita MADLN15SG, 2-Matsushita MSDLN25SG, 3-Matsushita MCDLN35SG
    * @param [in] axisEncType Encoder type 0-increments; 1- absolute value
    * @return error code
    */
    int ExtAxisParamConfig(int axisID, int axisType, int axisDirection, double axisMax, double axisMin, double axisVel, double axisAcc, double axisLead, long encResolution, double axisOffect, int axisCompany, int axisModel, int axisEncType);

Set the installation position of the expansion shaft
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set the installation position of the expansion shaft
    * @param [in] installType  0-The robot is installed on the external axis, 1-the robot is installed outside the external axis
    * @return error code
    */
    int SetRobotPosToAxis(int installType);

Set the extended shaft system DH parameters
+++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set the extended shaft system DH parameters
    * @param [in] axisConfig 0-single DOF linear slide, 1-2 DOF L-type positioner, 2-3 DOF, 3-4 DOF, 4-single DOF positioner
    * @param [in] axisDHd1 External axisDH parameter d1 mm
    * @param [in] axisDHd2 External axisDH parameter d2 mm
    * @param [in] axisDHd3 External axisDH parameter d3 mm
    * @param [in] axisDHd4 External axisDH parameter d4 mm
    * @param [in] axisDHa1 External axisDH parameter 11 mm
    * @param [in] axisDHa2 External axisDH parameter a2 mm
    * @param [in] axisDHa3 External axisDH parameter a3 mm
    * @param [in] axisDHa4 External axisDH parameter a4 mm
    * @return error code
    */
    int SetAxisDHParaConfig(int axisConfig, double axisDHd1, double axisDHd2, double axisDHd3, double axisDHd4, double axisDHa1, double axisDHa2, double axisDHa3, double axisDHa4);

Code example
***************

.. code-block:: C#
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

Set the reference point of the extended axis coordinate system - four-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set the reference point of the extended axis coordinate system - four-point method
    * @param [in] pointNum Point number [1-4]
    * @return error code 
    */
    int ExtAxisSetRefPoint(int pointNum);

Calculation of extended axis coordinate system - four-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Calculation of extended axis coordinate system - four-point method
    * @param [out] coord coordinate values
    * @return error code
    */
    int ExtAxisComputeECoordSys(ref DescPose& coord);

Apply the extended axis coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Apply the extended axis coordinate system
    * @param [in]  applyAxisId Extended shaft number BIT0-BIT3 corresponds to the extension axis number 1-4, such as the application extension axis 1 and 3,then 0B 0000 0101, that is 5;
    * @param [in]  axisCoordNum Extended shaft coordinate system number
    * @param [in]  coord coordinate values
    * @param [in]  calibFlag calibflag 0-No, 1-yes
    * @return error code
    */
    int ExtAxisActiveECoordSys(int applyAxisId, int axisCoordNum, DescPose coord, int calibFlag);

Code example
************

.. code-block:: C#
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

Set the pose of the calibration reference point in the end coordinate system of the positioner
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set the pose of the calibration reference point in the end coordinate system of the positioner
    * @param [in] pos Position value
    * @return error code
    */
    int SetRefPointInExAxisEnd(DescPose pos);

Positioner coordinate system reference point setting - four-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Positioner coordinate system reference point setting - four-point method
    * @param [in] pointNum Point number[1-4]
    * @return error code 
    */
    int PositionorSetRefPoint(int pointNum);

Coordinate system calculation of positioner - four-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Coordinate system calculation of positioner - four-point method
    * @param [out] coord coordinate values
    * @return error code
    */
    int PositionorComputeECoordSys(DescPose& coord);

Code example
************

.. code-block:: C#
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

Enable the UDP extension axis
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Enable the UDP extension axis
    * @param [in] axisID Axis number [1-4]
    * @param [in] status 0-Disable, 1-Enable
    * @return error code
    */
    int ExtAxisServoOn(int axisID, int status);

Set the UDP extension axis homing
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set the UDP extension axis homing
    * @param [in] axisID Axis number[1-4]
    * @param [in] mode homing mode; 0-Current position homing, 1-negative limit homing, 2-positive limit homing 
    * @param [in] searchVel homing velocity(mm/s)
    * @param [in] latchVel homing latch velocity(mm/s)
    * @return error code
    */
    int ExtAxisSetHoming(int axisID, int mode, double searchVel, double latchVel);

UDP extension axis jog start
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief UDP extension axis jog start
    * @param [in] axisID Axis number[1-4]
    * @param [in] direction Rotation direction 0- reverse; 1-forward
    * @param [in] vel velocity(mm/s)
    * @param [in] acc Acceleration  (mm/s2)
    * @param [in] maxDistance maximum jog distance(mm)
    * @return error code
    */
    int ExtAxisStartJog(int axisID, int direction, double vel, double acc, double maxDistance);
    
UDP extension axis jog stop
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief UDP extension axis jog stop
    * @param [in] axisID Axis number[1-4]
    * @return error code
    */
    int ExtAxisStopJog(int axisID);

Code example
************

.. code-block:: C#
    :linenos:

    private void btnJog_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        robot.ExtAxisServoOn(1, 1);
        robot.ExtAxisSetHoming(1, 0, 10, 3);
        robot.ExtAxisStartJog(1, 1, 100, 100, 20);
        Thread.Sleep(1000 * 2);
        robot.ExtAxisStopJog(1);
        robot.ExtAxisServoOn(1, 0);
    }

UDP extension axis movement
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief UDP extension axis movement
    * @param [in] pos target position
    * @param [in] ovl Speed percentage
    * @return error code
    */
    int ExtAxisMove(ExaxisPos pos, double ovl);

Code example
************

.. code-block:: C#
    :linenos:

    private void btnAxisMove_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        ExaxisPos pos = new ExaxisPos(10, 0, 0, 0);
        robot.ExtAxisMove(pos, 10);
    }

The UDP expansion axis moves synchronously with the robot joint movement
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief The UDP expansion axis moves synchronously with the robot joint movement
    * @param [in] joint_pos Position of the target joint, unit[°]
    * @param [in] desc_pos target Cartesian pose, unit[mm]
    * @param [in] tool Tool number [0~14]
    * @param [in] user workpiece number [0~14]
    * @param [in] vel Speed percentage [0~100]
    * @param [in] acc Acceleration percentage[0~100]
    * @param [in] ovl Speed scaling factor[0~100]
    * @param [in] epos the external axis position[mm]
    * @param [in] blendT [1.0] - movement in place (block), [0 ~ 500.0]-smooth time (non-blocking), unit[ms]
    * @param [in] offset_flag 0-No offset, 1-Offset in the job/base coordinate system, 2-Offset in the tool coordinate system, defaults to 0
    * @param [in] offset_pos position offset
    * @return error code
    */
    int ExtAxisSyncMoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos epos, float blendT, byte offset_flag, DescPose offset_pos);

Code example
************

.. code-block:: C#
    :linenos:

    private void btnSyncMoveJ_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

    //1.Calibrate and apply the robot tool coordinate system. You can use the four-point method or the six-point method to calibrate and apply the tool coordinate system. The interfaces involved in the calibration of the tool coordinate system are as follows:
        //    int SetToolPoint(int point_num);  //Set tool reference point - six point method
        //    int ComputeTool(ref DescPose tcp_pose);  //Computational tool coordinate system
        //    int SetTcp4RefPoint(int point_num);    //Set tool reference point - four point method
        //    int ComputeTcp4(ref DescPose tcp_pose);   //Calculating tool coordinate system - four-point method
        //    int SetToolCoord(int id, DescPose coord, int type, int install);  //Set the application tool coordinate system
        //    int SetToolList(int id, DescPose coord, int type, int install);   //Sets the list of application tool coordinate systems

        //2.Set UDP communication parameters and load UDP communication
        robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevLoadUDPDriver();

        //3.Set the extension shaft parameters, including the extension shaft type, extension shaft drive parameters, and extension shaft DH parameters
        robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //Single axis positioner and DH parameters
        robot.SetRobotPosToAxis(1);  //Expansion shaft mounting position
        robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //Servo drive parameters, this example is a single-axis positioner, so only one drive parameter needs to be set. If you choose an extension shaft type with multiple axes, you need to set the drive parameters for each axis

        //4.Set the selected axis to enable and homing
        robot.ExtAxisServoOn(1, 0);
        robot.ExtAxisSetHoming(1, 0, 20, 3);

        //5.Carry out calibration and application of extended axis coordinate system
        DescPose pos = new DescPose(/* Enter your marker coordinates */);
        robot.SetRefPointInExAxisEnd(pos);
        robot.PositionorSetRefPoint(1); /*You need to calibrate the extension axis through four points in different locations, so you need to call this interface four times to complete the calibration */
        DescPose coord = new DescPose();
        robot.PositionorComputeECoordSys(ref coord); //Calculate the calibration results of the extension axis
        robot.ExtAxisActiveECoordSys(1, 1, coord, 1);  //The calibration results are applied to the extended axis coordinate system

        //6.To calibrate the workpiece coordinate system on the extension axis, you need the following interfaces
        //int SetWObjCoordPoint(int point_num);
        //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
        //int SetWObjCoord(int id, DescPose coord);
        //int SetWObjList(int id, DescPose coord);

        //7.Record the start point of your synchronous joint movement
        DescPose startdescPose = new DescPose(/*Enter your coordinates*/);
        JointPos startjointPos = new JointPos(/*Enter your coordinates*/);
        ExaxisPos startexaxisPos = new ExaxisPos(/* Enter your extension axis start point coordinates */);

        //8.Record the coordinates of the end point of your synchronous joint movement
        DescPose enddescPose = new DescPose(/*Enter your coordinates*/);
        JointPos endjointPos = new JointPos(/*Enter your coordinates*/);
        ExaxisPos endexaxisPos = new ExaxisPos(/* Enter your extension axis endpoint coordinates */);

        //9.Write synchronous motion program
        //Move to the starting point, assuming that the tool coordinate system and the workpiece coordinate system are both 1
        robot.ExtAxisMove(startexaxisPos, 20);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
        robot.MoveJ(startjointPos, startdescPose, 1, 1, 100, 100, 100, startexaxisPos, 0, 0, offdese);

        //Start synchronized motion
        robot.ExtAxisSyncMoveJ(endjointPos, enddescPose, 1, 1, 100, 100, 100, endexaxisPos, -1, 0, offdese);
    }

The UDP extension axis moves synchronously with the robot’s linear motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief  The UDP extension axis moves synchronously with the robot’s linear motion
    * @param [in] joint_pos Position of the target joint, unit[°]
    * @param [in] desc_pos target Cartesian pose, unit[mm]
    * @param [in] tool Tool number [0~14]
    * @param [in] user workpiece number [0~14]
    * @param [in] vel Speed percentage [0~100]
    * @param [in] acc Acceleration percentage[0~100]
    * @param [in] ovl Speed scaling factor[0~100]
    * @param [in] blendR 1.0-movement in place (block), [0 ~ 1000] - smooth radius (non-blocking), unit (mm) 1.0 by default
    * @param [in] epos the external axis position[mm]
    * @param [in] offset_flag  0-No offset, 1-Offset in the job/base coordinate system, 2-Offset in the tool coordinate system, defaults to 0
    * @param [in] offset_pos position offset
    * @return error code
    */
    int ExtAxisSyncMoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, int offset_flag, DescPose offset_pos);

Code example
************

.. code-block:: C#
    :linenos:

    private void btnSyncMoveL_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        //1.Calibrate and apply the robot tool coordinate system. You can use the four-point method or the six-point method to calibrate and apply the tool coordinate system. The interfaces involved in the calibration of the tool coordinate system are as follows:
        //    int SetToolPoint(int point_num);  //Set tool reference point - six point method
        //    int ComputeTool(ref DescPose tcp_pose);  //Computational tool coordinate system
        //    int SetTcp4RefPoint(int point_num);    //Set tool reference point - four point method
        //    int ComputeTcp4(ref DescPose tcp_pose);   //Calculating tool coordinate system - four-point method
        //    int SetToolCoord(int id, DescPose coord, int type, int install);  //Set the application tool coordinate system
        //    int SetToolList(int id, DescPose coord, int type, int install);   //Sets the list of application tool coordinate systems

        //2.Set UDP communication parameters and load UDP communication
        robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevLoadUDPDriver();

        //3.Set the extension shaft parameters, including the extension shaft type, extension shaft drive parameters, and extension shaft DH parameters
        robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //Single axis positioner and DH parameters
        robot.SetRobotPosToAxis(1);  //Expansion shaft mounting position
        robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //Servo drive parameters, this example is a single-axis positioner, so only one drive parameter needs to be set. If you choose an extension shaft type with multiple axes, you need to set the drive parameters for each axis

        //4.Set the selected axis to enable and homing
        robot.ExtAxisServoOn(1, 0);
        robot.ExtAxisSetHoming(1, 0, 20, 3);

        //5.Carry out calibration and application of extended axis coordinate system
        DescPose pos = new DescPose(/* Enter your marker coordinates */);
        robot.SetRefPointInExAxisEnd(pos);
        robot.PositionorSetRefPoint(1); /*You need to calibrate the extension axis through four points in different locations, so you need to call this interface four times to complete the calibration */
        DescPose coord = new DescPose();
        robot.PositionorComputeECoordSys(ref coord); //Calculate the calibration results of the extension axis
        robot.ExtAxisActiveECoordSys(1, 1, coord, 1);  //The calibration results are applied to the extended axis coordinate system

        //6.To calibrate the workpiece coordinate system on the extension axis, you need the following interfaces
        //int SetWObjCoordPoint(int point_num);
        //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
        //int SetWObjCoord(int id, DescPose coord);
        //int SetWObjList(int id, DescPose coord);

        //7.Record the start point of your synchronous line movement
        DescPose startdescPose = new DescPose(/*Enter your coordinates*/);
        JointPos startjointPos = new JointPos(/*Enter your coordinates*/);
        ExaxisPos startexaxisPos = new ExaxisPos(/* Enter your extension axis start point coordinates */);

        //8.Record the coordinates of the end point of your synchronous line movement
        DescPose enddescPose = new DescPose(/*Enter your coordinates*/);
        JointPos endjointPos = new JointPos(/*Enter your coordinates*/);
        ExaxisPos endexaxisPos = new ExaxisPos(/* Enter your extension axis endpoint coordinates */);

        //9.Write synchronous motion program
        //Move to the starting point, assuming that the tool coordinate system and the workpiece coordinate system are both 1
        robot.ExtAxisMove(startexaxisPos, 20);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
        robot.MoveJ(startjointPos, startdescPose, 1, 1, 100, 100, 100, startexaxisPos, 0, 0, offdese);

        //Start synchronized motion
        robot.ExtAxisSyncMoveL(endjointPos, enddescPose, 1, 1, 100, 100, 100, 0, endexaxisPos, 0, offdese);
    }
        
The UDP extension axis moves synchronously with the robot arc motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief The UDP extension axis moves synchronously with the robot arc motion
    * @param [in] joint_pos_p joint position of a pathpoint [°]
    * @param [in] desc_pos_p path point Cartesian pose[mm]
    * @param [in] ptool  path point tool number[0~14]
    * @param [in] puser  path point workpiece number[0~14]
    * @param [in] pvel  Speed percentage [0~100]
    * @param [in] pacc  Acceleration percentage[0~100]
    * @param [in] epos_p Pathpoint external axis position mm
    * @param [in] poffset_flag 0-No offset, 1-Offset in the job/base coordinate system, 2-Offset in the tool coordinate system, defaults to 0
    * @param [in] offset_pos_p  path point position offset
    * @param [in] joint_pos_t joint position of the target point[°]
    * @param [in] desc_pos_t Cartesian position of the target point[mm]
    * @param [in] ttool  target point tool number[0~14]
    * @param [in] tuser  target point workpiece number[0~14]
    * @param [in] tvel  Speed percentage[0~100]
    * @param [in] tacc  Acceleration percentage[0~100]
    * @param [in] epos_t target point external axis position mm
    * @param [in] toffset_flag 0-No offset, 1-Offset in the job/base coordinate system, 2-Offset in the tool coordinate system, defaults to 0
    * @param [in] offset_pos_t target point position offset 
    * @param [in] ovl Speed scaling factor [0~100]
    * @param [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm  
    * @return error code
    */
    int ExtAxisSyncMoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos epos_p, int poffset_flag, DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos epos_t, int toffset_flag, DescPose offset_pos_t, float ovl, float blendR);
    
Code example
************

.. code-block:: C#
    :linenos:

    private void btnSyncMoveC_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        //1.Calibrate and apply the robot tool coordinate system. You can use the four-point method or the six-point method to calibrate and apply the tool coordinate system. The interfaces involved in the calibration of the tool coordinate system are as follows:
        //    int SetToolPoint(int point_num);  //Set tool reference point - six point method
        //    int ComputeTool(ref DescPose tcp_pose);  //Computational tool coordinate system
        //    int SetTcp4RefPoint(int point_num);    //Set tool reference point - four point method
        //    int ComputeTcp4(ref DescPose tcp_pose);   //Calculating tool coordinate system - four-point method
        //    int SetToolCoord(int id, DescPose coord, int type, int install);  //Set the application tool coordinate system
        //    int SetToolList(int id, DescPose coord, int type, int install);   //Sets the list of application tool coordinate systems

        //2.Set UDP communication parameters and load UDP communication
        robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevLoadUDPDriver();

        //3.Set the extension shaft parameters, including the extension shaft type, extension shaft drive parameters, and extension shaft DH parameters
        robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //Single axis positioner and DH parameters
        robot.SetRobotPosToAxis(1);  //Expansion shaft mounting position
        robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //Servo drive parameters, this example is a single-axis positioner, so only one drive parameter needs to be set. If you choose an extension shaft type with multiple axes, you need to set the drive parameters for each axis

        //4.Set the selected axis to enable and homing
        robot.ExtAxisServoOn(1, 0);
        robot.ExtAxisSetHoming(1, 0, 20, 3);

        //5.Carry out calibration and application of extended axis coordinate system
        DescPose pos = new DescPose(/* Enter your marker coordinates */);
        robot.SetRefPointInExAxisEnd(pos);
        robot.PositionorSetRefPoint(1); /*You need to calibrate the extension axis through four points in different locations, so you need to call this interface four times to complete the calibration */
        DescPose coord = new DescPose();
        robot.PositionorComputeECoordSys(ref coord); //Calculate the calibration results of the extension axis
        robot.ExtAxisActiveECoordSys(1, 1, coord, 1);  //The calibration results are applied to the extended axis coordinate system

        //6.To calibrate the workpiece coordinate system on the extension axis, you need the following interfaces
        //int SetWObjCoordPoint(int point_num);
        //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
        //int SetWObjCoord(int id, DescPose coord);
        //int SetWObjList(int id, DescPose coord);

        //7.Record the start point of your synchronous line movement
        DescPose startdescPose = new DescPose(/*Enter your coordinates*/);
        JointPos startjointPos = new JointPos(/*Enter your coordinates*/);
        ExaxisPos startexaxisPos = new ExaxisPos(/* Enter your extension axis start point coordinates */);

        //8.Record the coordinates of the end point of your synchronous line movement
        DescPose enddescPose = new DescPose(/*Enter your coordinates*/);
        JointPos endjointPos = new JointPos(/*Enter your coordinates*/);
        ExaxisPos endexaxisPos = new ExaxisPos(/* Enter your extension axis endpoint coordinates */);

        //9.Record the coordinates of the intermediate point of your synchronous circular motion
        DescPose middescPose = new DescPose(/*Enter your coordinates*/);
        JointPos midjointPos = new JointPos(/*Enter your coordinates*/);
        ExaxisPos midexaxisPos = new ExaxisPos(/* Expand axis coordinates when entering the robot arc midpoint */);

        //10.Write synchronous motion program
        //Move to the starting point, assuming that the tool coordinate system and the workpiece coordinate system are both 1
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
    * @param [in] bOpen True- on,False- off
    * @param [in] smooth whether it is smooth; True-Yes, False-no
    * @param [in] block True-block, False-no block
    * @return error code
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
    * @param [in] value analog quantity value [0-4095]
    * @param [in] block True-block, False-no block
    * @return error code
    */
    int SetAuxAO(int AONum, double value, bool block);
    
Code example
************

.. code-block:: C#
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
            
Set the extended DI input filtering time
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set the extended DI input filtering time
    * @param [in] filterTime DI input filtering time(ms)
    * @return error code
    */
    int SetAuxDIFilterTime(int filterTime);

Set the extended AI input filtering time
++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Set the extended AI input filtering time
    * @param [in] filterTime AI input filtering time(ms)
    * @return error code
    */
    int SetAuxAIFilterTime(int filterTime);

Wait for the extended DI input
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Wait for the extended DI input
    * @param [in] DINum DI number
    * @param [in] bOpen True- on,False- off
    * @param [in] time Maximum waiting time(ms)
    * @param [in] errorAlarm Whether to continue a motion. True- Yes,False- no
    * @return error code
    */
    int WaitAuxDI(int DINum, bool bOpen, int time, bool errorAlarm);
    
Wait for the extended AI input
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Wait for the extended AI input
    * @param [in] AINum AI number
    * @param [in] sign 0-greater than, 1-less than
    * @param [in] value AI value
    * @param [in] time Maximum waiting time(ms)
    * @param [in] errorAlarm Whether to continue a motion. True- Yes,False- no
    * @return error code
    */
    int WaitAuxAI(int AINum, int sign, int value, int time, bool errorAlarm);
        
Gets the extended DI value
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Gets the extended DI value
    * @param [in] DINum DI number
    * @param [in] isNoBlock True-block, False-no block
    * @param [out] isOpen True- on,False- off
    * @return error code
    */
    int GetAuxDI(int DINum, bool isNoBlock, bool& isOpen);
            
Gets the extended AI value
++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7

.. code-block:: C#
    :linenos:

    /**
    * @brief Gets the extended AI value
    * @param [in] AINum AI number
    * @param [in] isNoBlock True-block, False-no block
    * @param [in] value input value
    * @return error code
    */
    int GetAuxAI(int AINum, bool isNoBlock, int& value);

Code example
***************
.. code-block:: C#
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
        Console.Write($"DI{i}  {curValue}  ");
        Console.WriteLine("  ");
    }

    int curValue = -1;
    int rtn = 0;
    for (int i = 0; i < 4; i++)
    {
        rtn = robot.GetAuxAI(i, true, ref curValue);
        txtRtn.Text = rtn.ToString();
        Console.Write($"AI{i} {curValue}   rtn is {rtn} ");
        Console.WriteLine("  ");
    }

    robot.WaitAuxDI(1, true, 1000, false);
    robot.WaitAuxAI(1, 1, 132, 1000, false);
    }