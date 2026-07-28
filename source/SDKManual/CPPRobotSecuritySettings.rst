Security settings
========================

.. toctree:: 
    :maxdepth: 5

Set Collision Level
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Set collision level
    * @param  [in]  mode  0-Level, 1-Percentage
    * @param  [in]  level Collision threshold, level range [], percentage range [0~1]
    * @param  [in]  config 0-Don't update config file, 1-Update config file
    * @return  Error code
    */
    errno_t  SetAnticollision(int mode, float level[6], int config);

Set Post-Collision Strategy
++++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
	 * @brief  Set post-collision strategy
	 * @param  [in] strategy  0-Error pause; 1-Continue running; 2-Error stop; 3-Gravity torque mode; 4-Oscillation response mode; 5-Collision rebound mode
	 * @param  [in] safeTime  Safe stop time [1000 - 2000]ms
	 * @param  [in] safeDistance  Safe stop distance [1-150]mm
	 * @param  [in] safetyMargin  J1-J6 safety factor [1-10]
	 * @return  Error code
	 */
	errno_t SetCollisionStrategy(int strategy, int safeTime, int safeDistance, int safetyMargin[]);

Custom Collision Detection Threshold Start
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.0-3.8.0

.. code-block:: c++
    :linenos:

	 /**
	 * @brief  Start custom collision detection threshold, set joint and TCP collision detection thresholds
	 * @param  [in] flag 1-Joint detection only; 2-TCP detection only; 3-Both joint and TCP detection
	 * @param  [in] jointDetectionThreshould Joint collision detection threshold j1-j6
	 * @param  [in] tcpDetectionThreshould TCP collision detection threshold, xyzabc
	 * @param  [in] block 0-Non-blocking; 1-Blocking
	 * @return  Error code
	 */
	errno_t CustomCollisionDetectionStart(int flag, double jointDetectionThreshould[6], double tcpDetectionThreshould[6], int block);

Custom Collision Detection Threshold End
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.0-3.8.0

.. code-block:: c++
    :linenos:

	/**
	 * @brief  Close custom collision detection threshold function
	 * @return  Error code
	 */
	errno_t CustomCollisionDetectionEnd();

Robot Collision Level Setting Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestCollision(void)
     {
         ROBOT_STATE_PKG pkg = {};
         FRRobot robot;
         robot.LoggerInit();
         robot.SetLoggerLevel(1);
         int rtn = robot.RPC("192.168.58.2");
         if (rtn != 0)
         {
             return -1;
         }
         robot.SetReConnectParam(true, 30000, 500);
         int mode = 0;
         int config = 1;
         float level1[6] = { 1.0,2.0,3.0,4.0,5.0,6.0 };
         float level2[6] = { 50.0,20.0,30.0,40.0,50.0,60.0 };
         rtn = robot.SetAnticollision(mode, level1, config);
         printf("SetAnticollision mode 0 rtn is %d\n", rtn);
         mode = 1;
         rtn = robot.SetAnticollision(mode, level2, config);
         printf("SetAnticollision mode 1 rtn is %d\n", rtn);
         JointPos p1Joint(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
         JointPos p2Joint(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
         DescPose p1Desc(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
         DescPose p2Desc(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
         ExaxisPos exaxisPos(0.0, 0.0, 0.0, 0.0);
         DescPose offdese(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
         robot.MoveL(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, 2, &exaxisPos, 0, 0, &offdese);
         robot.ResetAllError();
         int safety[6] = { 5,5,5,5,5,5 };
         rtn = robot.SetCollisionStrategy(3, 1000, 150, 250, safety);
         printf("SetCollisionStrategy rtn is %d\n", rtn);
         double jointDetectionThreshould[6] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };
         double tcpDetectionThreshould[6] = { 60,60,60,60,60,60 };
         rtn = robot.CustomCollisionDetectionStart(3, jointDetectionThreshould, tcpDetectionThreshould, 0);
         cout << "CustomCollisionDetectionStart rtn is " << rtn << endl;
         robot.MoveL(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
         robot.MoveL(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
         rtn = robot.CustomCollisionDetectionEnd();
         cout << "CustomCollisionDetectionEnd rtn is " << rtn << endl;
         robot.CloseRPC();
         return 0;
     }

Set Positive Limit
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set positive limit
    * @param  [in] limit Six joint positions, unit deg
    * @return  Error code
    */
    errno_t  SetLimitPositive(float limit[6]);

Set Negative Limit
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set negative limit
    * @param  [in] limit Six joint positions, unit deg
    * @return  Error code
    */
    errno_t  SetLimitNegative(float limit[6]);   

Get Joint Soft Limit Angles
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get joint soft limit angles
    * @param  [in] flag 0-Blocking, 1-Non-blocking    
    * @param  [out] negative  Negative limit angle, unit deg
    * @param  [out] positive  Positive limit angle, unit deg
    * @return  Error code
    */
    errno_t  GetJointSoftLimitDeg(uint8_t flag, float negative[6], float positive[6]);
    
Robot Limit Setting Code Example
++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestLimit(void)
    {
      ROBOT_STATE_PKG pkg = {};
      FRRobot robot;
      robot.LoggerInit();
      robot.SetLoggerLevel(1);
      int rtn = robot.RPC("192.168.58.2");
      if (rtn != 0)
      {
        return -1;
      }
      robot.SetReConnectParam(true, 30000, 500);
      float plimit[6] = { 170.0,80.0,150.0,80.0,170.0,160.0 };
      robot.SetLimitPositive(plimit);
      float nlimit[6] = { -170.0,-260.0,-150.0,-260.0,-170.0,-160.0 };
      robot.SetLimitNegative(nlimit);
      float neg_deg[6] = { 0.0 }, pos_deg[6] = { 0.0 };
      robot.GetJointSoftLimitDeg(0, neg_deg, pos_deg);
      printf("neg limit deg:%f,%f,%f,%f,%f,%f\n", neg_deg[0], neg_deg[1], neg_deg[2], neg_deg[3], neg_deg[4], neg_deg[5]);
      printf("pos limit deg:%f,%f,%f,%f,%f,%f\n", pos_deg[0], pos_deg[1], pos_deg[2], pos_deg[3], pos_deg[4], pos_deg[5]);
      robot.CloseRPC();
      return 0;
    }

Set Robot Collision Detection Method
++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set robot collision detection method
    * @param [in] method Collision detection method: 0-Current mode; 1-Dual encoder; 2-Both current and dual encoder
    * @param [in] thresholdMode Collision level threshold method: 0-Fixed collision level threshold; 1-Custom collision detection threshold
    * @return  Error code
    */
    errno_t SetCollisionDetectionMethod(int method, int thresholdMode = 0);

Set Static Collision Detection On/Off
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Set static collision detection on/off
     * @param [in] status 0-Off; 1-On
     * @return Error code
     */
    errno_t SetStaticCollisionOnOff(int status);

Robot Collision Detection Method Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int TestCollisionMethod(void)
    {
      ROBOT_STATE_PKG pkg = {};
      FRRobot robot;
      robot.LoggerInit();
      robot.SetLoggerLevel(1);
      int rtn = robot.RPC("192.168.58.2");
      if (rtn != 0)
      {
        return -1;
      }
      robot.SetReConnectParam(true, 30000, 500);
      rtn = robot.SetCollisionDetectionMethod(0, 0);
      printf("SetCollisionDetectionMethod rtn is %d\n", rtn);
      rtn = robot.SetStaticCollisionOnOff(1);
      printf("SetStaticCollisionOnOff On rtn is %d\n", rtn);
      rtn = robot.Sleep(5000);
      rtn = robot.SetStaticCollisionOnOff(0);
      printf("SetStaticCollisionOnOff Off rtn is %d\n", rtn);
      robot.CloseRPC();
      return 0;
    }

Joint Torque Power Detection
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Joint torque power detection
     * @param [in] status 0-Off; 1-On
     * @param [in] power Set maximum power (W);
     * @return Error code
     */
    errno_t SetPowerLimit(int status, double power);

Joint Torque Power Detection Code Example
++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestPowerLimit(void)
    {
       ROBOT_STATE_PKG pkg = {};
       FRRobot robot;
       robot.LoggerInit();
       robot.SetLoggerLevel(1);
       int rtn = robot.RPC("192.168.58.2");
       if (rtn != 0)
       {
          return -1;
       }
       robot.SetReConnectParam(true, 30000, 500);
       robot.DragTeachSwitch(1);
       robot.SetPowerLimit(1, 200);
       float torques[] = { 0, 0, 0, 0, 0, 0 };
       robot.GetJointTorques(1, torques);
       int count = 100;
       robot.ServoJTStart(); 
       int error = 0;
       while (count > 0)
       {
          error = robot.ServoJT(torques, 0.001);
          count = count - 1;
          robot.Sleep(1);
       }
       error = robot.ServoJTEnd();
       robot.DragTeachSwitch(0);
       robot.CloseRPC();
       return 0;
    }

Set Safety Speed Parameters
++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set safety speed parameters
    * @param [in] enable 0-off; 1-manual mode enable; 2-all modes enable
    * @param [in] maxTCPVel Maximum TCP speed limit; [0-1000] mm/s
    * @param [in] strategy Post-overspeed strategy; 0-stop and alarm; 1-auto speed limit; 2-stop and alarm with disable
    * @param [in] maxJointVel Maximum speed for 6 joints (°/s), default 45°/s
    * @return Error code
    */
    errno_t SetVelReducePara(int enable, double maxTCPVel, int strategy, std::vector<double> maxJointVel = {45.0, 45.0, 45.0, 45.0, 45.0, 45.0});
        
SDK Code Example for Setting Safety Speed Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestSetVelReducePara()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos j1(0, -90, 90, 0, 0, 0);
        JointPos j2(90, -90, 90, 0, 0, 0);
        ExaxisPos epos(0, 0, 0, 0);
        DescPose offset_pos(0, 0, 0, 0, 0, 0);
        robot.SetSpeed(80);
        rtn = robot.SetVelReducePara(2, 30, 1);
        printf("SetVelReducePara param error rtn is %d\n", rtn);
        rtn = robot.SetVelReducePara(0, 30, 1);
        printf("SetVelReducePara disable reduce vel rtn is %d\n", rtn);
        robot.MoveJ(&j1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.MoveJ(&j2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        rtn = robot.SetVelReducePara(1, 30, 1);
        printf("SetVelReducePara reduce vel rtn is %d\n", rtn);
        robot.MoveJ(&j1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.MoveJ(&j2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        rtn = robot.SetVelReducePara(2, 30, 2);
        printf("SetVelReducePara disable robot rtn is %d\n", rtn);
        robot.MoveJ(&j1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.MoveJ(&j2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(2000);
        robot.ResetAllError();
        robot.RobotEnable(1);
        robot.Sleep(1000);
        rtn = robot.SetVelReducePara(2, 30, 0);
        printf("SetVelReducePara report error rtn is %d\n", rtn);
        robot.MoveJ(&j1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.MoveJ(&j2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.CloseRPC();
        robot.Sleep(1000);
        return 0;
    }

Code Example for Setting Robot Joint Safety Speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:    

    int TestSetJointVelReducePara()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        JointPos j1(10.220, -11.121, -118.086, -46.739, 82.036, 131.503);
        JointPos j2(89.782, -11.122, -118.086, -46.740, 82.036, 131.504);
        ExaxisPos epos(0, 0, 0, 0);
        DescPose offset_pos(0, 0, 0, 0, 0, 0);
        robot.SetSpeed(20);

        std::vector<double> maxJointVelA = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0 };
        rtn = robot.SetVelReducePara(2, 200, 0, maxJointVelA);
        printf("SetVelReducePara param error rtn is %d\n", rtn);
        robot.MoveJ(&j1, 1, 2, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.MoveJ(&j2, 1, 2, 100, 100, 100, &epos, -1, 0, &offset_pos);
        std::vector<double> maxJointVelB = { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 };
        rtn = robot.SetVelReducePara(2, 200, 0, maxJointVelB);
        printf("SetVelReducePara reduce vel rtn is %d\n", rtn);
        robot.MoveJ(&j1, 1, 2, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.MoveJ(&j2, 1, 2, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(2000);
        robot.CloseRPC();
        return 0;
    }