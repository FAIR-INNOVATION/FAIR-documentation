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
+++++++++++++++++++

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
+++++++++++++++++

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
