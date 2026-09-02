Robot Safety Settings
==================================

.. toctree:: 
    :maxdepth: 5

Set collision level
++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set collision level
    * @param  [in]  mode  0-level, 1-percentage
    * @param  [in]  level Collision threshold, level corresponds to range [], percentage corresponds to range [0~1]
    * @param  [in]  config 0-do not update configuration file, 1-update configuration file
    * @return  Error code
    */
    int SetAnticollision(int mode, double[] level, int config);
 
Set collision post-strategy
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set collision post-strategy
    * @param  [in] strategy  0 - Pause on error; 1 - Continue running; 2 - Stop on error; 3 - Torque mode; 4 - Oscillation response mode; 5 - Collision rebound mode
    * @param  [in] safeTime  safe stop time [1000 - 2000]ms
    * @param  [in] safeDistance  Safe stop distance [1-150] mm
    * @param  [in] safeVel  TCP safe stop speed [50-250] mm/s
    * @param  [in] safetyMargin  j1-j6 safety factor [1-10]
    * @return Error code
    */
    int SetCollisionStrategy(int strategy, int safeTime, int safeDistance, int safeVel,int[] safetyMargin);

Custom collision detection threshold function start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Custom collision detection threshold function starts, sets the collision detection threshold for the joint end and TCP end
    * @param  [in] flag 1-only joint detection enabled; 2-only TCP detection enabled; 3-joint and TCP detection enabled simultaneously
    * @param  [in] jointDetectionThreshould Joint collision detection threshold j1-j6
    * @param  [in] tcpDetectionThreshould TCP collision detection threshold, xyzabc
    * @param  [in] block 0-non-blocking; 1-blocking
    * @return  Error code
    */
    int CustomCollisionDetectionStart(int flag, double[] jointDetectionThreshould, double[] tcpDetectionThreshould, int block);

Custom collision detection threshold function disabled
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Custom collision detection threshold function disabled
    * @return Error code
    */
    int CustomCollisionDetectionEnd();

Robot collision level settings code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button24_Click(object sender, EventArgs e)
    {
        int mode = 0;
        int config = 1;
        double[] level1 = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f };
        double[] level2 = { 50.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f };

        int rtn = robot.SetAnticollision(mode, level1, config);
        Console.WriteLine($"SetAnticollision mode 0 rtn is {rtn}");
        mode = 1;
        rtn = robot.SetAnticollision(mode, level2, config);
        Console.WriteLine($"SetAnticollision mode 1 rtn is {rtn}");

        JointPos p1Joint = new JointPos(-11.904f, -99.669f, 117.473f, -108.616f, -91.726f, 74.256f);
        JointPos p2Joint = new JointPos(-45.615f, -106.172f, 124.296f, -107.151f, -91.282f, 74.255f);

        DescPose p1Desc = new DescPose(-419.524f, -13.000f, 351.569f, -178.118f, 0.314f, 3.833f);
        DescPose p2Desc = new DescPose(-321.222f, 185.189f, 335.520f, -179.030f, -1.284f, -29.869f);

        ExaxisPos exaxisPos = new ExaxisPos(0.0f, 0.0f, 0.0f, 0.0f);
        DescPose offdese = new DescPose(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        robot.MoveL( p2Joint,  p2Desc, 0, 0, 100, 100, 100, 2,  exaxisPos, 0, 0,  offdese);
        robot.ResetAllError();
        int[] safety = { 5, 5, 5, 5, 5, 5 };
        rtn = robot.SetCollisionStrategy(3, 1000, 150, 250, safety);
        Console.WriteLine($"SetCollisionStrategy rtn is {rtn}");

        double[] jointDetectionThreshould = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };
        double[] tcpDetectionThreshould = { 60, 60, 60, 60, 60, 60 };
        rtn = robot.CustomCollisionDetectionStart(3, jointDetectionThreshould, tcpDetectionThreshould, 0);
        Console.WriteLine($"CustomCollisionDetectionStart rtn is {rtn}");

        robot.MoveL( p1Joint,  p1Desc, 0, 0, 100, 100, 100, -1,  exaxisPos, 0, 0,  offdese);
        robot.MoveL( p2Joint,  p2Desc, 0, 0, 100, 100, 100, -1,  exaxisPos, 0, 0,  offdese);
        rtn = robot.CustomCollisionDetectionEnd();
        Console.WriteLine($"CustomCollisionDetectionEnd rtn is {rtn}");
    }

Set positive limit
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set positive limit
    * @param  [in] limit Six joint positions, units in deg
    * @return  Error code
    */
    int SetLimitPositive(double[] limit);
 
Set negative limit
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set negative limit
    * @param  [in] limit Six joint positions, units in deg
    * @return  Error code
    */
    int SetLimitNegative(double[] limit);
 
Get joint soft limit angles
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get joint soft limit angles
    * @param  [in] flag 0-block, 1-non-block
    * @param  [out] negative  Negative limit angle, in degrees
    * @param  [out] positive  Positive limit angle, in degrees
    * @return  Error code
    */
    int GetJointSoftLimitDeg(byte flag, ref double[] negative, ref double[] positive);

Robot limit setting code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnRobotSafetySet_Click(object sender, EventArgs e)
    {
        double[] plimit = { 170.0f, 80.0f, 150.0f, 80.0f, 170.0f, 160.0f };
        robot.SetLimitPositive(plimit);
        double[] nlimit = { -170.0f, -260.0f, -150.0f, -260.0f, -170.0f, -160.0f };
        robot.SetLimitNegative(nlimit);

        double[] neg_deg = new double[6] {0,0,0,0,0,0 };
        double[] pos_deg = new double[6] { 0, 0, 0, 0, 0, 0 };
        robot.GetJointSoftLimitDeg(0, ref neg_deg,ref pos_deg);
        Console.WriteLine($"neg limit deg:{neg_deg[0]},{neg_deg[1]},{neg_deg[2]},{neg_deg[3]},{neg_deg[4]},{neg_deg[5]}");
        Console.WriteLine($"pos limit deg:{pos_deg[0]},{pos_deg[1]},{pos_deg[2]},{pos_deg[3]},{pos_deg[4]},{pos_deg[5]}");
    }

Set robot collision detection method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set robot collision detection method
    * @param  [in] method Collision detection method: 0-current mode; 1-dual encoder; 2-current and dual encoder enabled simultaneously
    * @param [in] thresholdMode Collision level threshold mode; 0 - fixed collision level threshold mode; 1 - custom collision detection threshold 
    * @return  Error code
    */
    int SetCollisionDetectionMethod(int method, int thresholdMode=0);

Set collision detection start/stop in static mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set collision detection start/stop in static mode
    * @param  [in] status 0-disabled; 1-enabled
    * @return  Error code
    */
    int SetStaticCollisionOnOff(int status);

Code example for setting the robot collision detection method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button26_Click(object sender, EventArgs e)
    {
        int rtn = robot.SetCollisionDetectionMethod(0, 0);

        rtn = robot.SetStaticCollisionOnOff(1);
        Console.WriteLine($"SetStaticCollisionOnOff On rtn is {rtn}");
        Thread.Sleep(5000);
        rtn = robot.SetStaticCollisionOnOff(0);
        Console.WriteLine($"SetStaticCollisionOnOff Off rtn is {rtn}");
    }

Joint Torque Power Detection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Joint Torque Power Detection
    * @param  [in] status 0-off; 1-on
    * @param  [in] power Set maximum power (W)
    * @return Error code
    */
    int SetPowerLimit(int status, double power);

Joint torque power detection code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button26_Click(object sender, EventArgs e)
    {
        robot.DragTeachSwitch(1);
        robot.SetPowerLimit(1, 200);
        double[] torques = { 0, 0, 0, 0, 0, 0 };
        robot.GetJointTorques(1, torques);

        int count = 100;
        robot.ServoJTStart();
        int error = 0;
        while (count > 0)
        {
            error = robot.ServoJT(torques, 0.001f);
            count--;
            Thread.Sleep(1);
        }
        error = robot.ServoJTEnd();
        robot.DragTeachSwitch(0);
    }

Set Safety Speed Parameters
++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set safety speed parameters
    * @param [in] enable 0-off; 1-enabled in manual mode; 2-enabled in all modes (automatic speed limiting not supported)
    * @param [in] maxTCPVel Maximum TCP speed limit; [0-1000] mm/s
    * @param [in] strategy Strategy after overspeed; 0-stop with alarm; 1-automatic speed limiting; 2-stop with alarm and disable
    * @param [in] maxJointVel Maximum speed for 6 joints (°/s), default 45°/s
    * @return Error code
    */
    public int SetVelReducePara(int enable, double maxTCPVel, int strategy, double[] maxJointVel = null)
    
SDK Code Example for Setting Safety Speed Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    public int TestSetVelReducePara()
    {
        int rtn = 0;
        JointPos j1 = new JointPos(10.220, -11.121, -118.086, -46.739, 82.036, 131.503);
        JointPos j2 = new JointPos(89.782, -11.122, -118.086, -46.740, 82.036, 131.504);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        double[] maxJointVel = new double[] { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 };

        robot.SetSpeed(20);
        rtn = robot.SetVelReducePara(0, 200, 0, maxJointVel);
        robot.MoveJ(j2, 1, 2, 100, 100, 100, epos, -1, 0, offset_pos);

        // 1st
        rtn = robot.SetVelReducePara(2, 200, 0, maxJointVel);
        Console.WriteLine($"SetVelReduceParaA param error rtn is {rtn}");
        robot.MoveJ(j1, 1, 2, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.MoveJ(j2, 1, 2, 100, 100, 100, epos, -1, 0, offset_pos);

        // 2rd
        maxJointVel = new double[] { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 };
        rtn = robot.SetVelReducePara(2, 200, 0, maxJointVel);
        Console.WriteLine($"SetVelReduceParaB reduce vel rtn is {rtn}");
        robot.MoveJ(j1, 1, 2, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.MoveJ(j2, 1, 2, 100, 100, 100, epos, -1, 0, offset_pos);
        return 0; 
    }
    
Get Safety Configuration Parameter Checksum
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:  

    /**
    * @brief  Get safety configuration parameter checksum
    * @param  [out] status Verification status, 0-valid, 1-verifying, 2-verification failed
    * @param  [out] checksum Checksum, 8-digit hexadecimal
    * @return  Error code
    */
    public int GetSafetyParamsCheckSum(ref int status, ref uint checksum)
        
Safety Operation Password Verification
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:  

    /**
    * @brief  Safety operation password verification
    * @param  [in] status Verification, 0-enable, 1-disable
    * @param  [in] password Password
    * @return  Error code
    */
    public int SafetyOPPasswordCheck(int status, string password)
            
Code Example for Getting Safety Configuration Parameter Checksum and Safety Operation Password Verification
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:  

    public void TestSafetyParamsCheckSum()
    {
        int status = 0;
        uint checksum = 0;

        int error = robot.GetSafetyParamsCheckSum(ref status, ref checksum);
        Console.WriteLine("GetSafetyParamsCheckSum: error={0}, status={1}, hex_code={2:X8}", error, status, checksum);
        Thread.Sleep(3000);

        error = robot.SafetyOPPasswordCheck(0, "12345678");
        Console.WriteLine("SafetyOPPasswordCheck: error={0}", error);

        if (error == 0)
        {
            error = robot.SetAnticollision(0, new double[] { 2.0, 2.0, 2.0, 2.0, 2.0, 2.0 }, 1);
            Console.WriteLine("SetAnticollision: error={0}", error);

            error = robot.SetCollisionStrategy(0, 1000, 150, 0, new int[] { 10, 10, 10, 10, 10, 10 });
            Console.WriteLine("SetCollisionStrategy: error={0}", error);
        }

        Thread.Sleep(1000);

        error = robot.GetSafetyParamsCheckSum(ref status, ref checksum);
        Console.WriteLine("GetSafetyParamsCheckSum(again): error={0}, status={1}, hex_code={2:X8}", error, status, checksum);
    }    