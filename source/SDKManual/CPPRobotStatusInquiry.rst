Status query
=====================

.. toctree:: 
    :maxdepth: 5

Get Current Joint Positions (Degrees)
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get current joint positions (degrees)
    * @param  [in] flag 0-Blocking, 1-Non-blocking
    * @param  [out] jPos Six joint positions, unit deg
    * @return  Error code
    */
    errno_t  GetActualJointPosDegree(uint8_t flag, JointPos *jPos);

Get Joint Feedback Speed
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get joint feedback speed - deg/s
     * @param  [in] flag 0-Blocking, 1-Non-blocking
     * @param  [out] speed Six joint speeds
     * @return  Error code 
     */ 
    errno_t  GetActualJointSpeedsDegree(uint8_t flag, float speed[6]);

Get Joint Feedback Acceleration
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get joint feedback acceleration - deg/s^2
     * @param  [in] flag 0-Blocking, 1-Non-blocking
     * @param  [out] acc Six joint accelerations
     * @return  Error code 
     */ 
    errno_t  GetActualJointAccDegree(uint8_t flag, float acc[6]);   

Get TCP Command Composite Speed
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get TCP command composite speed
     * @param  [in] flag 0-Blocking, 1-Non-blocking
     * @param  [out] tcp_speed Linear speed
     * @param  [out] ori_speed Orientation speed
     * @return  Error code 
     */
    errno_t  GetTargetTCPCompositeSpeed(uint8_t flag, float *tcp_speed, float *ori_speed);

Get TCP Feedback Composite Speed
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get TCP feedback composite speed
     * @param  [in] flag 0-Blocking, 1-Non-blocking
     * @param  [out] tcp_speed Linear speed
     * @param  [out] ori_speed Orientation speed
     * @return  Error code 
     */ 
    errno_t  GetActualTCPCompositeSpeed(uint8_t flag, float *tcp_speed, float *ori_speed);

Get TCP Command Speed
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get TCP command speed
     * @param  [in] flag 0-Blocking, 1-Non-blocking
     * @param  [out] speed [x,y,z,rx,ry,rz] speed
     * @return  Error code 
     */ 
    errno_t  GetTargetTCPSpeed(uint8_t flag, float speed[6]);

Get TCP Feedback Speed
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get TCP feedback speed
     * @param  [in] flag 0-Blocking, 1-Non-blocking
     * @param  [out] speed [x,y,z,rx,ry,rz] speed
     * @return  Error code 
     */ 
    errno_t  GetActualTCPSpeed(uint8_t flag, float speed[6]);

Get Current Tool Pose
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get current tool pose
    * @param  [in] flag  0-Blocking, 1-Non-blocking
    * @param  [out] desc_pos  Tool pose
    * @return  Error code
    */
    errno_t  GetActualTCPPose(uint8_t flag, DescPose *desc_pos);

Get Current Tool Frame Number
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get current tool frame number
    * @param  [in] flag  0-Blocking, 1-Non-blocking
    * @param  [out] id  Tool frame number
    * @return  Error code
    */
    errno_t  GetActualTCPNum(uint8_t flag, int *id);

Get Current Work Object Frame Number
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get current work object frame number
    * @param  [in] flag  0-Blocking, 1-Non-blocking
    * @param  [out] id  Work object frame number
    * @return  Error code
    */
    errno_t  GetActualWObjNum(uint8_t flag, int *id);  

Get Current End Flange Pose
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get current end flange pose
    * @param  [in] flag  0-Blocking, 1-Non-blocking
    * @param  [out] desc_pos  Flange pose
    * @return  Error code
    */
    errno_t  GetActualToolFlangePose(uint8_t flag, DescPose *desc_pos);  

Get Current Joint Torque
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Get current joint torque
    * @param  [in] flag 0-Blocking, 1-Non-blocking
    * @param  [out] torques Joint torque
    * @return  Error code
    */
    errno_t  GetJointTorques(uint8_t flag, float torques[6]);

Get System Time
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get system time
    * @param  [out] t_ms Unit ms
    * @return  Error code
    */
    errno_t  GetSystemClock(float *t_ms);

Check If Robot Motion Is Complete
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Check if robot motion is complete
    * @param  [out]  state  0-Not complete, 1-Complete
    * @return  Error code
    */   
    errno_t  GetRobotMotionDone(uint8_t *state);

Query Robot Motion Queue Length
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Query robot motion queue length
     * @param  [out]  len  Queue length
     * @return  Error code
     */ 
    errno_t  GetMotionQueueLength(int *len);

Get Robot Emergency Stop Status
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Get robot emergency stop status
    * @param [out] state Emergency stop status, 0-Normal, 1-Emergency stop
    * @return Error code 
    */
    errno_t GetRobotEmergencyStopState(uint8_t *state);

Get SDK-Robot Communication Status
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Get SDK-robot communication status
    * @param [out] state Communication status, 0-Normal, 1-Abnormal
    */
    errno_t GetSDKComState(int *state);

Get Safety Stop Signal
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Get safety stop signal
    * @param [out] si0_state Safety stop signal SI0, 0-Invalid, 1-Valid
    * @param [out] si1_state Safety stop signal SI1, 0-Invalid, 1-Valid
    */
    errno_t GetSafetyStopState(uint8_t *si0_state, uint8_t *si1_state);

Get Robot Joint Driver Temperature(℃)
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get robot joint driver temperature(℃)
    * @return Error code
    */
    errno_t GetJointDriverTemperature(double temperature[]);

Get Robot Joint Driver Torque(Nm)
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get robot joint driver torque(Nm)
    * @return Error code
    */
    errno_t GetJointDriverTorque(double torque[]);
        
Get Robot Real-time Status Structure
+++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get robot real-time status structure
    * @param [out] pkg Robot real-time status structure
    * @return Error code
    */
    errno_t GetRobotRealTimeState(ROBOT_STATE_PKG *pkg);

Robot Status Query Code Example
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos: 

    int TestGetStatus(void)
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
      float yangle, zangle;
      robot.GetRobotInstallAngle(&yangle, &zangle);
      printf("yangle:%f,zangle:%f\n", yangle, zangle);
      JointPos j_deg = {};
      robot.GetActualJointPosDegree(0, &j_deg);
      printf("joint pos deg:%f,%f,%f,%f,%f,%f\n", j_deg.jPos[0], j_deg.jPos[1], j_deg.jPos[2], j_deg.jPos[3], j_deg.jPos[4], j_deg.jPos[5]);
      float jointSpeed[6] = { 0.0 };
      robot.GetActualJointSpeedsDegree(0, jointSpeed);
      printf("joint speeds deg:%f,%f,%f,%f,%f,%f\n", jointSpeed[0], jointSpeed[1], jointSpeed[2], jointSpeed[3], jointSpeed[4], jointSpeed[5]);
      float jointAcc[6] = { 0.0 };
      robot.GetActualJointAccDegree(0, jointAcc);
      printf("joint acc deg:%f,%f,%f,%f,%f,%f\n", jointAcc[0], jointAcc[1], jointAcc[2], jointAcc[3], jointAcc[4], jointAcc[5]);
      float tcp_speed = 0.0;
      float ori_speed = 0.0;
      robot.GetTargetTCPCompositeSpeed(0, &tcp_speed, &ori_speed);
      printf("GetTargetTCPCompositeSpeed tcp %f; ori %f\n", tcp_speed, ori_speed);
      robot.GetActualTCPCompositeSpeed(0, &tcp_speed, &ori_speed);
      printf("GetActualTCPCompositeSpeed tcp %f; ori %f\n", tcp_speed, ori_speed);
      float targetSpeed[6] = { 0.0 };
      robot.GetTargetTCPSpeed(0, targetSpeed);
      printf("GetTargetTCPSpeed %f,%f,%f,%f,%f,%f\n", targetSpeed[0], targetSpeed[1], targetSpeed[2], targetSpeed[3], targetSpeed[4], targetSpeed[5]);
      float actualSpeed[6] = { 0.0 };
      robot.GetActualTCPSpeed(0, actualSpeed);
      printf("GetTargetTCPSpeed %f,%f,%f,%f,%f,%f\n", actualSpeed[0], actualSpeed[1], actualSpeed[2], actualSpeed[3], actualSpeed[4], actualSpeed[5]);
      DescPose tcp = {};
      robot.GetActualTCPPose(0, &tcp);
      printf("tcp pose:%f,%f,%f,%f,%f,%f\n", tcp.tran.x, tcp.tran.y, tcp.tran.z, tcp.rpy.rx, tcp.rpy.ry, tcp.rpy.rz);
      DescPose flange = {};
      robot.GetActualToolFlangePose(0, &flange);
      printf("flange pose:%f,%f,%f,%f,%f,%f\n", flange.tran.x, flange.tran.y, flange.tran.z, flange.rpy.rx, flange.rpy.ry, flange.rpy.rz);
      int id = 0;
      robot.GetActualTCPNum(0, &id);
      printf("tcp num:%d\n", id);
      robot.GetActualWObjNum(0, &id);
      printf("wobj num:%d\n", id);
      float jtorque[6] = { 0.0 };
      robot.GetJointTorques(0, jtorque);
      printf("torques:%f,%f,%f,%f,%f,%f\n", jtorque[0], jtorque[1], jtorque[2], jtorque[3], jtorque[4], jtorque[5]);
      float t_ms = 0.0;
      robot.GetSystemClock(&t_ms);
      printf("system clock:%f\n", t_ms);
      int config = 0;
      robot.GetRobotCurJointsConfig(&config);
      printf("joint config:%d\n", config);
      uint8_t motionDone = 0;
      robot.GetRobotMotionDone(&motionDone);
      printf("GetRobotMotionDone :%d\n", motionDone);
      int len = 0;
      robot.GetMotionQueueLength(&len);
      printf("GetMotionQueueLength :%d\n", len);
      uint8_t emergState = 0;
      robot.GetRobotEmergencyStopState(&emergState);
      printf("GetRobotEmergencyStopState :%d\n", emergState);
      int comstate = 0;
      robot.GetSDKComState(&comstate);
      printf("GetSDKComState :%d\n", comstate);
      uint8_t si0_state, si1_state;
      robot.GetSafetyStopState(&si0_state, &si1_state);
      printf("GetSafetyStopState :%d %d\n", si0_state, si1_state);
      double temp[6] = { 0.0 };
      robot.GetJointDriverTemperature(temp);
      printf("Temperature:%f,%f,%f,%f,%f,%f\n", temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]);
      double torque[6] = { 0.0 };
      robot.GetJointDriverTorque(torque);
      printf("torque:%f,%f,%f,%f,%f,%f\n", torque[0], torque[1], torque[2], torque[3], torque[4], torque[5]);
      robot.GetRobotRealTimeState(&pkg);
      robot.CloseRPC();
      return 0;
    }

Inverse Kinematics Calculation
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Inverse kinematics calculation
    * @param  [in] type 0-Absolute pose (base frame), 1-Incremental pose (base frame), 2-Incremental pose (tool frame)
    * @param  [in] desc_pos Cartesian pose
    * @param  [in] config Joint space configuration, [-1]-Calculate based on current joint position, [0~7]-Solve according to specific joint space configuration
    * @param  [out] joint_pos Joint position
    * @return  Error code
    */
    errno_t  GetInverseKin(int type, DescPose *desc_pos, int config, JointPos *joint_pos);

Inverse Kinematics Calculation (Reference Position)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Inverse kinematics calculation, solve based on specified joint position
    * @param  [in] type 0-Absolute pose (base frame), 1-Incremental pose (base frame), 2-Incremental pose (tool frame)
    * @param  [in] desc_pos Cartesian pose
    * @param  [in] joint_pos_ref Reference joint position
    * @param  [out] joint_pos Joint position
    * @return  Error code
    */   
    errno_t  GetInverseKinRef(int type, DescPose *desc_pos, JointPos *joint_pos_ref, JointPos *joint_pos);

Check If Inverse Kinematics Has Solution
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Inverse kinematics calculation, check if solution exists based on specified joint position
    * @param  [in] type 0-Absolute pose (base frame), 1-Incremental pose (base frame), 2-Incremental pose (tool frame)
    * @param  [in] desc_pos Cartesian pose
    * @param  [in] joint_pos_ref Reference joint position
    * @param  [out] result 0-No solution, 1-Solution exists
    * @return  Error code
    */   
    errno_t  GetInverseKinHasSolution(int type, DescPose *desc_pos, JointPos *joint_pos_ref, uint8_t *result);

Forward Kinematics Calculation
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Forward kinematics calculation
    * @param  [in] joint_pos Joint position
    * @param  [out] desc_pos Cartesian pose
    * @return  Error code
    */
    errno_t  GetForwardKin(JointPos *joint_pos, DescPose *desc_pos);

Robot Kinematics Calculation Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestInverseKin(void)
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
      JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
      DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
      JointPos inverseRtn = {};
      robot.GetInverseKin(0, &desc_pos1, -1, &inverseRtn);
      printf("dcs1 GetInverseKin rtn is %f %f %f %f %f %f \n", inverseRtn.jPos[0], inverseRtn.jPos[1], inverseRtn.jPos[2], inverseRtn.jPos[3], inverseRtn.jPos[4], inverseRtn.jPos[5]);
      robot.GetInverseKinRef(0, &desc_pos1, &j1, &inverseRtn);
      printf("dcs1 GetInverseKinRef rtn is %f %f %f %f %f %f \n", inverseRtn.jPos[0], inverseRtn.jPos[1], inverseRtn.jPos[2], inverseRtn.jPos[3], inverseRtn.jPos[4], inverseRtn.jPos[5]);
      uint8_t hasResut = 0;
      robot.GetInverseKinHasSolution(0, &desc_pos1, &j1, &hasResut);
      printf("dcs1 GetInverseKinRef result %d\n", hasResut);
      DescPose forwordResult = {};
      robot.GetForwardKin(&j1, &forwordResult);
      printf("jpos1 forwordResult rtn is %f %f %f %f %f %f \n", forwordResult.tran.x, forwordResult.tran.y, forwordResult.tran.z, forwordResult.rpy.rx, forwordResult.rpy.ry, forwordResult.rpy.rz);
      robot.CloseRPC();
      return 0;
    }

Query Robot Teaching Point Data
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Query robot teaching point data
     * @param  [in]  name  Point name
     * @param  [out]  data   Point data
     * @return  Error code
     */ 
    errno_t  GetRobotTeachingPoint(char name[64], float data[20]);

Get Robot DH Parameter Compensation Values
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get robot DH parameter compensation values
    * @param [out] dhCompensation Robot DH parameter compensation values(mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]
    * @return Error code
    */
    errno_t GetDHCompensation(double dhCompensation[6]);

Get Controller SN Code
+++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.1-3.8.1

.. code-block:: c++
    :linenos:

    /**
    * @brief Get controller SN code
    * @param [out] SNCode Controller SN code
    * @return Error code
    */
    errno_t GetRobotSN(std::string& SNCode);

Query Robot Teaching Point Data Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestGetTeachPoint(void)
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
      char name[64] = "P1";
      float data[20] = { 0 };
      rtn = robot.GetRobotTeachingPoint(name, data);
      printf(" %d name is: %s \n", rtn, name);
      for (int i = 0; i < 20; i++)
      {
        printf("data is: %f \n", data[i]);
      }
      int que_len = 0;
      rtn = robot.GetMotionQueueLength(&que_len);
      printf("GetMotionQueueLength rtn is: %d, queue length is: %d \n", rtn, que_len);
      double dh[6] = { 0 };
      int retval = 0;
      retval = robot.GetDHCompensation(dh);
      cout << "retval is: " << retval << endl;
      cout << "dh is: " << dh[0] << " " << dh[1] << " " << dh[2] << " " << dh[3] << " " << dh[4] << " " << dh[5] << endl;
      string SN = "";
      robot.GetRobotSN(SN);
      cout << "robot SN is " << SN << endl;
      robot.CloseRPC();
      return 0;
    }