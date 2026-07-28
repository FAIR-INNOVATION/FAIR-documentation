Robot Peripheral Devices
========================

.. toctree:: 
    :maxdepth: 5

Configure Gripper
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Configure gripper
    * @param  [in] company  Gripper manufacturer (to be determined)
    * @param  [in] device  Device number (currently unused, default 0)
    * @param  [in] softvesion  Software version (currently unused, default 0)
    * @param  [in] bus  Device bus location at end (currently unused, default 0)
    * @return  Error code
    */
    errno_t  SetGripperConfig(int company, int device, int softvesion, int bus);

Get Gripper Configuration
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get gripper configuration
    * @param  [in] company  Gripper manufacturer (to be determined)
    * @param  [in] device  Device number (currently unused, default 0)
    * @param  [in] softvesion  Software version (currently unused, default 0)
    * @param  [in] bus  Device bus location at end (currently unused, default 0)
    * @return  Error code
    */
    errno_t  GetGripperConfig(int *company, int *device, int *softvesion, int *bus);

Activate Gripper
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Activate gripper
    * @param  [in] index  Gripper number
    * @param  [in] act  0-reset, 1-activate
    * @return  Error code
    */
    errno_t  ActGripper(int index, uint8_t act);

Control Gripper
++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
	 * @brief  Control gripper
	 * @param  [in] index  Gripper number
	 * @param  [in] pos  Position percentage [0~100]
	 * @param  [in] vel  Speed percentage [0~100]
	 * @param  [in] force  Torque percentage [0~100]
	 * @param  [in] max_time  Maximum wait time [0~30000], unit ms
	 * @param  [in] block  0-blocking, 1-non-blocking
	 * @param  [in] type  Gripper type, 0-parallel gripper; 1-rotary gripper
	 * @param  [in] rotNum  Rotation turns
	 * @param  [in] rotVel  Rotation speed percentage [0-100]
	 * @param  [in] rotTorque  Rotation torque percentage [0-100]
	 * @return  Error code
	 */
	errno_t MoveGripper(int index, int pos, int vel, int force, int max_time, uint8_t block, int type, double rotNum, int rotVel, int rotTorque);

Get Gripper Motion Status
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get gripper motion status
     * @param  [out] fault  0-no error, 1-error
     * @param  [out] staus  0-motion not completed, 1-motion completed
     * @return  Error code
     */
    errno_t  GetGripperMotionDone(uint16_t *fault, uint8_t *status);

Get Gripper Activation Status
++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get gripper activation status
     * @param  [out] fault  0-no error, 1-error
     * @param  [out] status  bit0~bit15 correspond to gripper numbers 0~15, bit=0 not activated, bit=1 activated
     * @return  Error code
     */
    errno_t  GetGripperActivateStatus(uint16_t *fault, uint16_t *status);

Get Gripper Position
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get gripper position
     * @param  [out] fault  0-no error, 1-error
     * @param  [out] position  Position percentage, range 0~100%
     * @return  Error code
     */
    errno_t  GetGripperCurPosition(uint16_t *fault, uint8_t *position);

Get Gripper Speed
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get gripper speed
     * @param  [out] fault  0-no error, 1-error
     * @param  [out] speed  Speed percentage, range 0~100%
     * @return  Error code
     */
    errno_t  GetGripperCurSpeed(uint16_t *fault, int8_t *speed);

Get Gripper Current
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get gripper current
     * @param  [out] fault  0-no error, 1-error
     * @param  [out] current  Current percentage, range 0~100%
     * @return  Error code
     */
    errno_t  GetGripperCurCurrent(uint16_t *fault, int8_t *current);

Get Gripper Voltage
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get gripper voltage
     * @param  [out] fault  0-no error, 1-error
     * @param  [out] voltage  Voltage, unit 0.1V
     * @return  Error code
     */
    errno_t  GetGripperVoltage(uint16_t *fault, int *voltage);

Get Gripper Temperature
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get gripper temperature
     * @param  [out] fault  0-no error, 1-error
     * @param  [out] temp  Temperature, unit ℃
     * @return  Error code
     */
    errno_t  GetGripperTemp(uint16_t *fault, int *temp);

Calculate Pre-Pick Point - Vision
++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Calculate pre-pick point - vision
     * @param  [in] desc_pos  Pick point Cartesian pose
     * @param  [in] zlength  Z-axis offset
     * @param  [in] zangle   Rotation offset around Z-axis
     * @return  Error code 
     */
    errno_t  ComputePrePick(DescPose *desc_pos, double zlength, double zangle, DescPose *pre_pos);

Calculate Retreat Point - Vision
++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Calculate retreat point - vision
     * @param  [in] desc_pos  Pick point Cartesian pose
     * @param  [in] zlength  Z-axis offset
     * @param  [in] zangle   Rotation offset around Z-axis
     * @return  Error code 
     */
    errno_t  ComputePostPick(DescPose *desc_pos, double zlength, double zangle, DescPose *post_pos);

Robot Gripper Operation Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestGripper(void)
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
      int company = 4;
      int device = 0;
      int softversion = 0;
      int bus = 2;
      int index = 2;
      int act = 0;
      int max_time = 30000;
      uint8_t block = 0;
      uint8_t status;
      uint16_t fault;
      uint16_t active_status = 0;
      uint8_t current_pos = 0;
      int8_t current = 0;
      int voltage = 0;
      int temp = 0;
      int8_t speed = 0;
      robot.SetGripperConfig(company, device, softversion, bus);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      robot.GetGripperConfig(&company, &device, &softversion, &bus);
      printf("gripper config:%d,%d,%d,%d\n", company, device, softversion, bus);
      robot.ActGripper(index, act);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      act = 1;
      robot.ActGripper(index, act);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      robot.MoveGripper(index, 100, 50, 50, max_time, block, 0, 0, 0, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      robot.MoveGripper(index, 0, 50, 0, max_time, block, 0, 0, 0, 0);
      robot.GetGripperMotionDone(&fault, &status);
      printf("motion status:%u,%u\n", fault, status);
      robot.GetGripperActivateStatus(&fault, &active_status);
      printf("gripper active fault is: %u, status is: %u\n", fault, active_status);
      robot.GetGripperCurPosition(&fault, &current_pos);
      printf("fault is:%u, current position is: %u\n", fault, current_pos);
      robot.GetGripperCurCurrent(&fault, &current);
      printf("fault is:%u, current current is: %d\n", fault, current);
      robot.GetGripperVoltage(&fault, &voltage);
      printf("fault is:%u, current voltage is: %d \n", fault, voltage);
      robot.GetGripperTemp(&fault, &temp);
      printf("fault is:%u, current temperature is: %d\n", fault, temp);
      robot.GetGripperCurSpeed(&fault, &speed);
      printf("fault is:%u, current speed is: %d\n", fault, speed);
      int retval = 0;
      DescPose prepick_pose = {};
      DescPose postpick_pose = {};
      DescPose p1Desc(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
      DescPose p2Desc(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
      retval = robot.ComputePrePick(&p1Desc, 10, 0, &prepick_pose);
      printf("ComputePrePick retval is: %d\n", retval);
      printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", prepick_pose.tran.x, prepick_pose.tran.y, prepick_pose.tran.z, prepick_pose.rpy.rx, prepick_pose.rpy.ry, prepick_pose.rpy.rz);
      retval = robot.ComputePostPick(&p2Desc, -10, 0, &postpick_pose);
      printf("ComputePostPick retval is: %d\n", retval);
      printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", postpick_pose.tran.x, postpick_pose.tran.y, postpick_pose.tran.z, postpick_pose.rpy.rx, postpick_pose.rpy.ry, postpick_pose.rpy.rz);
      robot.CloseRPC();
      return 0;
    }

Get Rotation Count of Rotary Gripper
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: Version 3.7.6

.. code-block:: c++
    :linenos:

    /**
	 * @brief  Get rotation count of rotary gripper
	 * @param  [out] fault  0-no error, 1-error
	 * @param  [out] num  Rotation count
	 * @return  Error code
	 */
	errno_t GetGripperRotNum(uint16_t* fault, double* num);

Get Rotation Speed of Rotary Gripper
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: V3.7.6

.. code-block:: c++
    :linenos:

    /**
	 * @brief  Get rotation speed of rotary gripper
	 * @param  [out] fault  0-no error, 1-error
	 * @param  [out] speed  Rotation speed percentage
	 * @return  Error code
	 */
	errno_t GetGripperRotSpeed(uint16_t* fault, int* speed);

Get Rotation Torque of Rotary Gripper
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: V3.7.6

.. code-block:: c++
    :linenos:

    /**
	 * @brief  Get rotation torque of rotary gripper
	 * @param  [out] fault  0-no error, 1-error
	 * @param  [out] torque  Rotation torque percentage
	 * @return  Error code
	 */
	errno_t GetGripperRotTorque(uint16_t* fault, int* torque);

Rotary Gripper Status Code Example
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestRotGripperState(void)
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
      uint16_t fault = 0;
      double rotNum = 0.0;
      int rotSpeed = 0;
      int rotTorque = 0;
      robot.GetGripperRotNum(&fault, &rotNum);
      robot.GetGripperRotSpeed(&fault, &rotSpeed);
      robot.GetGripperRotTorque(&fault, &rotTorque);
      printf("gripper rot num : %lf, gripper rotSpeed : %d, gripper rotTorque : %d\n", rotNum, rotSpeed, rotTorque);
      robot.CloseRPC();
      return 0;
    }


Conveyor Start/Stop
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Conveyor start/stop
    * @param [in] status State, 1-start, 0-stop 
    * @return Error code
    */
    errno_t ConveyorStartEnd(uint8_t status);

Record IO Detection Point
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Record IO detection point
    * @return Error code
    */
    errno_t ConveyorPointIORecord();

Record Point A
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Record point A
    * @return Error code
    */
    errno_t ConveyorPointARecord();

Record Reference Point
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Record reference point
    * @return Error code
    */
    errno_t ConveyorRefPointRecord();

Record Point B
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Record point B
    * @return Error code
    */
    errno_t ConveyorPointBRecord();

Conveyor Workpiece IO Detection
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Conveyor workpiece IO detection
    * @param [in] max_t Maximum detection time, unit ms
    * @return Error code
    */
    errno_t ConveyorIODetect(int max_t);

Get Current Object Position
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Get current object position
    * @param [in] mode 
    * @return Error code
    */
    errno_t ConveyorGetTrackData(int mode);

Conveyor Tracking Start
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Conveyor tracking start
    * @param [in] status State, 1-start, 0-stop 
    * @return Error code
    */
    errno_t ConveyorTrackStart(uint8_t status);

Conveyor Tracking Stop
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Conveyor tracking stop
    * @return Error code
    */
    errno_t ConveyorTrackEnd();

Conveyor Parameter Configuration
+++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.2.1-3.8.1

.. code-block:: c++
    :linenos:

    /**
    * @brief Conveyor parameter configuration
    * @param [in] para[0] Encoder channel 1~2
    * @param [in] para[1] Pulses per encoder revolution
    * @param [in] para[2] Conveyor travel distance per encoder revolution
    * @param [in] para[3] Workpiece coordinate system number (0 for tracking capture and TPD tracking)
    * @param [in] para[4] Vision configuration 0-not configured 1-configured
    * @param [in] para[5] Speed ratio for conveyor tracking capture (1-100), default 1 for other options
    * @return Error code
    */
    errno_t ConveyorSetParam(float para[6], int followType = 0, int startDis = 0, int endDis = 100);

Conveyor Capture Point Compensation
+++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

	/**
	 * @brief Conveyor capture point compensation
	 * @param [in] cmp Compensation position double[3]{x, y, z}
	 * @return Error code
	 */
    errno_t ConveyorCatchPointComp(double cmp[3]);

Conveyor Linear Motion
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Conveyor linear motion
    * @param [in] status State, 1-start, 0-stop 
    * @return Error code
    */
    errno_t TrackMoveL(char name[32], int tool, int wobj, float vel, float acc, float ovl, float blendR, uint8_t flag, uint8_t type);

Conveyor Communication Input Detection
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.1-3.8.1
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Conveyor communication input detection
    * @param [in] timeout Wait timeout time (ms)
    * @return Error code
    */
    errno_t ConveyorComDetect(int timeout);

Conveyor Communication Input Detection Trigger
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.1-3.8.1
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Conveyor communication input detection trigger
    * @return Error code
    */
    errno_t ConveyorComDetectTrigger();

Robot Conveyor Operation Example Program
++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestConveyor(void)
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
      int retval = 0;
      retval = robot.ConveyorStartEnd(1);
      printf("ConveyorStartEnd retval is: %d\n", retval);
      retval = robot.ConveyorPointIORecord();
      printf("ConveyorPointIORecord retval is: %d\n", retval);
      retval = robot.ConveyorPointARecord();
      printf("ConveyorPointARecord retval is: %d\n", retval);
      retval = robot.ConveyorRefPointRecord();
      printf("ConveyorRefPointRecord retval is: %d\n", retval);
      retval = robot.ConveyorPointBRecord();
      printf("ConveyorPointBRecord retval is: %d\n", retval);
      retval = robot.ConveyorStartEnd(0);
      printf("ConveyorStartEnd retval is: %d\n", retval);
      retval = 0;
      float param[6] = { 1,10000,200,0,0,20 };
      retval = robot.ConveyorSetParam(param);
      printf("ConveyorSetParam retval is: %d\n", retval);
      double cmp[3] = { 0.0, 0.0, 0.0 };
      retval = robot.ConveyorCatchPointComp(cmp);
      printf("ConveyorCatchPointComp retval is: %d\n", retval);
      int index = 1;
      int max_time = 30000;
      uint8_t block = 0;
      retval = 0;
      DescPose p1Desc(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
      DescPose p2Desc(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
      retval = robot.MoveCart(&p1Desc, 1, 0, 100.0, 100.0, 100.0, -1.0, -1);
      printf("MoveCart retval is: %d\n", retval);
      retval = robot.WaitMs(1);
      printf("WaitMs retval is: %d\n", retval);
      retval = robot.ConveyorIODetect(10000);
      printf("ConveyorIODetect retval is: %d\n", retval);
      retval = robot.ConveyorGetTrackData(1);
      printf("ConveyorGetTrackData retval is: %d\n", retval);
      retval = robot.ConveyorTrackStart(1);
      printf("ConveyorTrackStart retval is: %d\n", retval);
      retval = robot.TrackMoveL("cvrCatchPoint", 1, 0, 100, 100, 100, -1.0, 0, 0);
      printf("TrackMoveL retval is: %d\n", retval);
      retval = robot.MoveGripper(index, 51, 40, 30, max_time, block, 0, 0, 0, 0);
      printf("MoveGripper retval is: %d\n", retval);
      retval = robot.TrackMoveL("cvrRaisePoint", 1, 0, 100, 100, 100, -1.0, 0, 0);
      printf("TrackMoveL retval is: %d\n", retval);
      retval = robot.ConveyorTrackEnd();
      printf("ConveyorTrackEnd retval is: %d\n", retval);
      robot.MoveCart(&p2Desc, 1, 0, 100.0, 100.0, 100.0, -1.0, -1);
      retval = robot.MoveGripper(index, 100, 40, 10, max_time, block, 0, 0, 0, 0);
      printf("MoveGripper retval is: %d\n", retval);
      rtn = robot->ConveyorComDetect(1000 * 10);
      printf("ConveyorComDetect rtn is: %d\n", rtn);
      robot.Sleep(2000);
      rtn = robot->ConveyorComDetectTrigger();
      printf("ConveyorComDetectTrigger rtn is: %d\n", rtn);
      robot.CloseRPC();
      return 0;
    }

Conveyor Belt In-Place Tracking Parameter Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.9.8
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Configure conveyor belt in-place tracking parameters
    * @param [in] trackMode 0-time; 1-distance; 2-time and distance, either condition satisfied
    * @param [in] trackTime Tracking time, unit s
    * @param [in] trackDis Tracking distance, unit mm
    * @return Error code
    */
    int SetStationaryTrackPara(int trackMode, double trackTime, int trackDis);
    
Conveyor Belt In-Place Tracking Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.9.8
    
.. code-block:: c++
    :linenos:

    int TestStationaryTrack()
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
        printf("\n========== Conveyor Stationary Tracking Test ==========");
        JointPos j1(-35.146, -102.684, 120.805, -100.401, -90.295, 150.105);
        DescPose d1(-121.814, -348.341, 209.978, -173.152, -3.585, -5.446);
        ExaxisPos ex(0, 0, 0, 0);
        DescPose zeroOff(0, 0, 0, 0, 0, 0);
        int tool = 1;
        int workpiece = 1;
        float conveyorParam[6] = { 0, 10000, 200, 0, 0, 10 };
        rtn = robot.ConveyorSetParam(conveyorParam);
        robot.MoveJ(&j1, &d1, tool, workpiece, 100, 100, 100, &ex, -1, 0, &zeroOff);
        // Step 1: SetDO control signal
        printf("--- Step 1: SetDO(6,1) ---\n");
        rtn = robot.SetDO(6, 1, 0, 0);
        printf("  SetDO(6,1) rtn={0}\n", rtn);
        // Step 2: Conveyor tracking start
        printf("--- Step 2: ConveyorTrackStart(2) ---\n");
        rtn = robot.ConveyorTrackStart(2);
        printf("  ConveyorTrackStart(2) rtn={0}\n", rtn);
        // Step 3: Workpiece IO detection
        printf("--- Step 3: ConveyorIODetect(10000) ---\n");
        rtn = robot.ConveyorIODetect(10000);
        printf("  ConveyorIODetect(10000) rtn={0}\n", rtn);
        // Step 4: Get tracking data
        printf("--- Step 4: ConveyorGetTrackData(2) ---\n");
        rtn = robot.ConveyorGetTrackData(2);
        printf("  ConveyorGetTrackData(2) rtn={0}\n", rtn);
        // Step 5: Stationary tracking parameter configuration (time mode, 200s, distance 5)
        printf("--- Step 5: SetStationaryTrackPara(0,200,5) ---\n");
        rtn = robot.SetStationaryTrackPara(0, 5, 5);
        printf("  SetStationaryTrackPara(0,200,5) rtn={0}\n", rtn);
        // Step 6: Execute stationary tracking motion
        printf("--- Step 6: MoveStationary() ---\n");
        rtn = robot.MoveStationary();
        rtn = robot.WaitStationaryMotionDone();
        printf("  MoveStationary() rtn={0}\n", rtn);
        // Step 7: Conveyor tracking end
        printf("--- Step 7: ConveyorTrackEnd() ---\n");
        rtn = robot.ConveyorTrackEnd();
        printf("  ConveyorTrackEnd() rtn={0}\n", rtn);
        // Step 8: SetDO turn off signal
        printf("--- Step 8: SetDO(6,0) ---\n");
        rtn = robot.SetDO(6, 0, 0, 0);
        printf("  SetDO(6,0) rtn={0}\n", rtn);
        printf("\n========== Stationary Tracking Test Complete ==========\n");
        return 0;
    }

End Sensor Configuration
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief End sensor configuration
    * @param [in] idCompany Manufacturer, 18-JUNKONG; 25-HUIDE
    * @param [in] idDevice Type, 0-JUNKONG/RYR6T.V1.0
    * @param [in] idSoftware Software version, 0-J1.0/HuiDe1.0 (not yet available)
    * @param [in] idBus Mounting position, 1-End port 1; 2-End port 2...8-End port 8 (not yet available)
    * @return Error code
    */
    errno_t AxleSensorConfig(int idCompany, int idDevice, int idSoftware, int idBus);

Get End Sensor Configuration
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Get end sensor configuration
     * @param [out] idCompany Manufacturer, 18-JUNKONG; 25-HUIDE
     * @param [out] idDevice Type, 0-JUNKONG/RYR6T.V1.0
     * @return Error code
     */
    errno_t AxleSensorConfigGet(int& idCompany, int& idDevice);

End Sensor Activation
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief End sensor activation
     * @param [in] actFlag 0-Reset; 1-Activate
     * @return Error code
     */
    errno_t AxleSensorActivate(int actFlag);

End Sensor Register Write
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief End sensor register write
     * @param [in] devAddr Device address number 0-255
     * @param [in] regHAddr Register address high 8 bits
     * @param [in] regLAddr Register address low 8 bits
     * @param [in] regNum Register count 0-255
     * @param [in] data1 Write register value 1
     * @param [in] data2 Write register value 2
     * @param [in] isNoBlock 0-Blocking; 1-Non-blocking
     * @return Error code
     */
    errno_t AxleSensorRegWrite(int devAddr, int regHAddr, int regLAddr, int regNum, int data1, int data2, int isNoBlock);

End Sensor Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestAxleSensor(void)
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
      robot.AxleSensorConfig(18, 0, 0, 1);
      int company = -1;
      int type = -1;
      robot.AxleSensorConfigGet(company, type);
      printf("company is %d, type is %d\n", company, type);
      rtn = robot.AxleSensorActivate(1);
      printf("AxleSensorActivate rtn is %d\n", rtn);
      robot.Sleep(1000);
      rtn = robot.AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0);
      printf("AxleSensorRegWrite rtn is %d\n", rtn);
      robot.CloseRPC();
      return 0;
    }
        
Get Robot Peripheral Protocol
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get robot peripheral protocol
    * @param [out] protocol Robot peripheral protocol number 4096-Extended axis control card; 4097-ModbusSlave; 4098-ModbusMaster
    * @return Error code
    */
    errno_t GetExDevProtocol(int *protocol);

Set Robot Peripheral Protocol
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set robot peripheral protocol
    * @param [in] protocol Robot peripheral protocol number 4096-Extended axis control card; 4097-ModbusSlave; 4098-ModbusMaster
    * @return Error code
    */
    errno_t SetExDevProtocol(int protocol);

Set Robot Peripheral Protocol Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:
    
    int TestExDevProtocol(void)
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
      int protocol = 4096;
      rtn = robot.SetExDevProtocol(protocol);
      std::cout << "SetExDevProtocol rtn " << rtn << std::endl;
      rtn = robot.GetExDevProtocol(&protocol);
      std::cout << "GetExDevProtocol rtn " << rtn << " protocol is: " << protocol << std::endl;
      robot.CloseRPC();
      return 0;
    }

Get End Communication Parameters
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get end communication parameters
    * @param param End communication parameters
    * @return  Error code
    */
    errno_t GetAxleCommunicationParam(AxleComParam* param);

Set End Communication Parameters
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set end communication parameters
    * @param param  End communication parameters
    * @return  Error code
    */
    errno_t SetAxleCommunicationParam(AxleComParam param);

Set End File Transfer Type
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set end file transfer type
    * @param type 1-MCU upgrade file; 2-LUA file
    * @return  Error code
    */
    errno_t SetAxleFileType(int type);

Set Enable End LUA Execution
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set enable end LUA execution
    * @param enable 0-Disable; 1-Enable
    * @return  Error code
    */
    errno_t SetAxleLuaEnable(int enable);

End LUA File Error Recovery
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief End LUA file error recovery
    * @param status 0-No recovery; 1-Recovery
    * @return  Error code
    */
    errno_t SetRecoverAxleLuaErr(int status);

Get End LUA Execution Enable Status
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get end LUA execution enable status
    * @param status status[0]: 0-Not enabled; 1-Enabled
    * @return  Error code
    */
    errno_t GetAxleLuaEnableStatus(int status[]);

Set the enabled device types for the end-effector LUA
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set the enabled device types for the end-effector LUA
    * @param forceSensorEnable Force sensor enable status, 0-disabled; 1-enabled
    * @param gripperEnable Gripper enable status, 0-disabled; 1-enabled
    * @param IOEnable IO device enable status, 0-disabled; 1-enabled
    * @param dexhandEnable Dexterous hand enable status, 0-disabled; 1-enabled
    * @return  Error code
    */
    errno_t SetAxleLuaEnableDeviceType(int forceSensorEnable, int gripperEnable, int IOEnable, int dexhandEnable);

Get the enabled device types for the end-effector LUA
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:
        
    /**
    * @brief Get the enabled device types for the end-effector LUA
    * @param enable enable[0]:forceSensorEnable Force sensor enable status, 0-disabled; 1-enabled
    * @param enable enable[1]:gripperEnable Gripper enable status, 0-disabled; 1-enabled
    * @param enable enable[2]:IOEnable IO device enable status, 0-disabled; 1-enabled
    * @param enable enable[3]:dexhandEnable Dexterous hand enable status, 0-disabled; 1-enabled
    * @return Error code
    */
    errno_t GetAxleLuaEnableDeviceType(int* forceSensorEnable, int* gripperEnable, int* IOEnable, int* dexhandEnable);

Get the currently configured end-effector devices
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get the currently configured end-effector devices
    * @param [out] forceSensorEnable Force sensor enabled device number, 0-disabled; 1-enabled
    * @param [out] gripperEnable Gripper enabled device number, 0-disabled; 1-enabled
    * @param [out] IODeviceEnable IO device enabled device number, 0-disabled; 1-enabled
    * @param [out] decHandEnable Dexterous hand enabled device number, 0-disabled; 1-enabled
    * @return Error code
    */
    errno_t GetAxleLuaEnableDevice(int forceSensorEnable[], int gripperEnable[], int IODeviceEnable[], int decHandEnable[]);

Set the enabled gripper action control functions
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set the enabled gripper action control functions
    * @param id Gripper device number
    * @param func func[0]-gripper enable; func[1]-gripper initialization; func[2]-position setting; func[3]-speed setting; func[4]-torque setting; func[6]-read gripper status;
        func[7]-read initialization status; func[8]-read fault code; func[9]-read position; func[10]-read speed; func[11]-read torque; func[12]-set rotation count for rotary gripper;
        func[13]-set rotation speed for rotary gripper; func[14]-set rotation torque for rotary gripper; func[15]-read rotary gripper status; func[16]-read rotary gripper initialization status;
        func[17]-read rotary gripper rotation count; func[18]-read rotary gripper speed; func[19]-read rotary gripper torque; func[20]-multi-axis synchronous motion setting; func[21]-fault clear command;
        func[22]-single-axis running status; func[23]-all-axis running status;
    * @return  Error code
    */
    errno_t SetAxleLuaGripperFunc(int id, int func[32]);

Get the enabled gripper action control functions
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get the enabled gripper action control functions
    * @param id Gripper device number
    * @param func func[0]-gripper enable; func[1]-gripper initialization; func[2]-position setting; func[3]-speed setting; func[4]-torque setting; func[6]-read gripper status;
        func[7]-read initialization status; func[8]-read fault code; func[9]-read position; func[10]-read speed; func[11]-read torque; func[12]-set rotation count for rotary gripper;
        func[13]-set rotation speed for rotary gripper; func[14]-set rotation torque for rotary gripper; func[15]-read rotary gripper status; func[16]-read rotary gripper initialization status;
        func[17]-read rotary gripper rotation count; func[18]-read rotary gripper speed; func[19]-read rotary gripper torque; func[20]-multi-axis synchronous motion setting; func[21]-fault clear command;
        func[22]-single-axis running status; func[23]-all-axis running status;
    * @return  Error code
    */
    errno_t GetAxleLuaGripperFunc(int id, int func[32]);

Robot Ethercat Slave File Write
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Robot Ethercat slave file write
    * @param type Slave file type, 1-Upgrade slave file; 2-Upgrade slave configuration file
    * @param slaveID Slave number
    * @param fileName Upload file name
    * @return  Error code
    */
    errno_t SlaveFileWrite(int type, int slaveID, std::string fileName);

Upload End Lua Open Protocol File
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Upload end Lua open protocol file
    * @param filePath Local lua file path name ".../AXLE_LUA_End_DaHuan.lua"
    * @return Error code
    */
    errno_t AxleLuaUpload(std::string filePath);

Robot Ethercat Slave Enter Boot Mode
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Robot Ethercat slave enter boot mode
    * @return  Error code
    */
    errno_t SetSysServoBootMode();

Robot End LUA File Operation Code Example
++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestAxleLua(void)
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
        robot.AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan.lua");
        AxleComParam param(7, 8, 1, 0, 5, 3, 1);
        robot.SetAxleCommunicationParam(param);
        AxleComParam getParam;
        robot.GetAxleCommunicationParam(&getParam);
        printf("GetAxleCommunicationParam param is %d %d %d %d %d %d %d\n", getParam.baudRate, getParam.dataBit, getParam.stopBit, getParam.verify, getParam.timeout, getParam.timeoutTimes, getParam.period);
        robot.SetAxleLuaEnable(1);
        int luaEnableStatus = 0;
        robot.GetAxleLuaEnableStatus(&luaEnableStatus);
        robot.SetAxleLuaEnableDeviceType(0, 1, 0, 0);
        int forceEnable = 0;
        int gripperEnable = 0;
        int ioEnable = 0;
        int dexhandEnable = 0;
        robot.GetAxleLuaEnableDeviceType(&forceEnable, &gripperEnable, &ioEnable, &dexhandEnable);
        printf("GetAxleLuaEnableDeviceType param is %d %d %d %d\n", forceEnable, gripperEnable, ioEnable, dexhandEnable);
        int func[32] = { 0,1,1,1,1,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
        rtn = robot.SetAxleLuaGripperFunc(2, func);
        printf("SetAxleLuaGripperFunc rtn is %d\n", rtn);
        int getFunc[32] = { 0 };
        rtn = robot.GetAxleLuaGripperFunc(2, getFunc);
        printf("GetAxleLuaGripperFunc rtn is %d\n", rtn);
        int getforceEnable[16] = { 0 };
        int getgripperEnable[16] = { 0 };
        int getioEnable[16] = { 0 };
        int dexhandEnable1[16] = { 0 };
        robot.GetAxleLuaEnableDevice(getforceEnable, getgripperEnable, getioEnable, dexhandEnable1);
        printf("\ngetforceEnable status : ");
        for (int i = 0; i < 16; i++)
        {
            printf("%d,", getforceEnable[i]);
        }
        printf("\ngetgripperEnable status : ");
        for (int i = 0; i < 16; i++)
        {
            printf("%d,", getgripperEnable[i]);
        }
        printf("\ngetioEnable status : ");
        for (int i = 0; i < 16; i++)
        {
            printf("%d,", getioEnable[i]);
        }
        printf("\n");
        robot.ActGripper(2, 0);
        robot.Sleep(2000);
        robot.ActGripper(2, 1);
        robot.Sleep(2000);
        robot.MoveGripper(2, 1, 10, 100, 50000, 0, 0, 0, 0, 0);
        int pos = 0;
        while (true)
        {
            robot.GetRobotRealTimeState(&pkg);
            printf("gripper pos is %u\n", pkg.gripper_position);
            robot.Sleep(100);
        }
        robot.CloseRPC();
        return 0;
    }

Get SmartTool Button Status
++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Get SmartTool button status
    * @param [out] state SmartTool handle button status; (bit0:0-Communication normal; 1-Communication lost; bit1-Undo; bit2-Clear program;
    bit3-A button; bit4-B button; bit5-C button; bit6-D button; bit7-E button; bit8-IO button; bit9-Manual/Auto; bit10-Start)
    * @return Error code
    */
    errno_t GetSmarttoolBtnState(int& state);
    
SmartTool Button Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int main(void)
    {
      ROBOT_STATE_PKG pkg = {};
      FRRobot robot;

      robot.LoggerInit();
      robot.SetLoggerLevel(1);
      int rtn = robot.RPC("192.168.58.2");
      robot.SetReConnectParam(true, 30000, 500);

      while (true)
      {
        int btn = 0;
        robot.GetSmarttoolBtnState(btn);
        cout << "smarttool " << std::bitset<sizeof(btn) * 8>(btn) << endl;

        Sleep(100);
      }
    }

Control Array Suction Cup
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Control Array Suction Cup
    * @param [in] slaveID Slave Station ID
    * @param [in] len Length
    * @param [in] ctrlValue Control Value
    * @return Error Code
    */
    errno_t FRRobot::SetSuckerCtrl(uint8_t slaveID, uint8_t len, uint8_t ctrlValue[20]);

Get Array Suction Cup Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Get Array Suction Cup Status
    * @param [in] slaveID Slave Station ID
    * @param [out] state Adsorption Status 0-Release Object 1-Workpiece Detected, Adsorption Successful 2-No Object Adsorbed 3-Object Detached
    * @param [out] pressValue Current Vacuum Level Unit kpa
    * @param [out] error Current Error Code of the Suction Cup
    * @return Error Code
    */
	errno_t FRRobot::GetSuckerState(uint8_t slaveID, uint8_t* state, int* pressValue, int* error);

Wait for Suction Cup Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Wait for Suction Cup Status
    * @param [in] slaveID Slave Station ID
    * @param [in] state Adsorption Status 0-Release Object 1-Workpiece Detected, Adsorption Successful 2-No Object Adsorbed 3-Object Detached
    * @param [in] ms Maximum Wait Time (ms)
    * @return Error Code
    */
    errno_t FRRobot::WaitSuckerState(uint8_t slaveID, uint8_t state, int ms);

Array Suction Cup Control Command Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    void testSucker()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        uint8_t ctrl[20];
        uint8_t state;
        int pressValue; // Corrected typo from pressVlaue
        int error;

        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return;
        }
        robot.SetReConnectParam(true, 30000, 500);
        //Upload and load the open protocol file
        robot.OpenLuaUpload("E://Project/Peripheral SDK/CtrlDev_sucker.lua");
        robot.Sleep(2000);
        robot.SetCtrlOpenLUAName(1, "CtrlDev_sucker.lua");
        robot.UnloadCtrlOpenLUA(1);
        robot.LoadCtrlOpenLUA(1);
        robot.Sleep(1000);

        //Control suction cup in broadcast mode, adsorb at maximum capacity
        ctrl[0] = 1;
        robot.SetSuckerCtrl(0, 1, ctrl);

        //Loop to monitor the status of suction cup 1 and suction cup 12
        for (int i = 0; i < 100; i++)
        {
            robot.GetSuckerState(1, &state, &pressValue, &error);
            printf("sucker1 state is %d, pressValue is %d, error num is %d\n", state, pressValue, error);
            robot.GetSuckerState(12, &state, &pressValue, &error);
            printf("sucker12 state is %d, pressValue is %d, error num is %d\n", state, pressValue, error);
            robot.Sleep(100);
        }

        //Wait for suction cup 1 to reach the state of having adsorbed an object, wait time 100ms
        int ret = robot.WaitSuckerState(1, 1, 100);
        printf("WaitSuckerState result is  %d\n", ret);

        //Turn off suction cup 1 and 12 in unicast mode
        ctrl[0] = 3;
        robot.SetSuckerCtrl(1, 1, ctrl);
        robot.SetSuckerCtrl(12, 1, ctrl);

        robot.CloseRPC();
    }

Upload Peripheral Open Protocol LUA File
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

	/**
	 * @brief Upload Lua File
	 * @param [in] filePath Local lua file path name
	 * @return Error Code
	 */
    errno_t OpenLuaUpload(std::string filePath);

Get Slave Board Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief  Get Slave Board Parameters
    * @param  [out] type  0-Ethercat，1-CClink, 3-Ethercat, 4-EIP
    * @param  [out] version  Protocol Version
    * @param  [out] connState  0-Not Connected 1-Connected
    * @return  Error Code
    */
    errno_t GetFieldBusConfig(uint8_t* type, uint8_t* version, uint8_t* connState);

Write Slave DO
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief  Write Slave DO
    * @param  [in] DOIndex  DO Number
    * @param  [in] writeNum  Number to Write // Corrected typo from wirteNum
    * @param  [in] status[8] Value to Write, maximum 8
    * @return  Error Code
    */
    errno_t FieldBusSlaveWriteDO(uint8_t DOIndex, uint8_t writeNum, uint8_t status[8]);

Write Slave AO
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief  Write Slave AO
    * @param  [in] AOIndex  AO Number
    * @param  [in] writeNum  Number to Write // Corrected typo from wirteNum
    * @param  [in] status[8] Value to Write, maximum 8
    * @return  Error Code
    */
    errno_t FieldBusSlaveWriteAO(uint8_t AOIndex, uint8_t writeNum, double status[8]);

Read Slave DI
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief  Read Slave DI
    * @param  [in] DIIndex  DI Number // Corrected parameter name from DOIndex
    * @param  [in] readNum  Number to Read // Corrected typo from readeNum
    * @param  [out] status[8] Value Read, maximum 8
    * @return  Error Code
    */
    errno_t FieldBusSlaveReadDI(uint8_t DIIndex, uint8_t readNum, uint8_t status[8]);

Read Slave AI
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief  Read Slave AI
    * @param  [in] AIIndex  AI Number // Corrected parameter name from AOIndex
    * @param  [in] readNum  Number to Read // Corrected typo from readeNum
    * @param  [out] status[8] Value Read, maximum 8
    * @return  Error Code
    */
    errno_t FieldBusSlaveReadAI(uint8_t AIIndex, uint8_t readNum, double status[8]);

Wait for Extended DI Input
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Wait for Extended DI Input
    * @param [in] DIIndex DI Number
    * @param [in] status 0-Low Level; 1-High Level
    * @param [in] waitMs Maximum Wait Time (ms)
    * @return Error Code
    */
    errno_t FRRobot::FieldBusSlaveWaitDI(uint8_t DIIndex, bool status, int waitMs);

Wait for Extended AI Input
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Wait for Extended AI Input
    * @param [in] AIIndex AI Number
    * @param [in] waitType 0-Greater Than; 1-Less Than
    * @param [in] value AI Value
    * @param [in] waitMs Maximum Wait Time (ms)
    * @return Error Code
    */
    errno_t FRRobot::FieldBusSlaveWaitAI(uint8_t AIIndex, uint8_t waitType, double value, int waitMs);

Slave Mode Related Interface Command Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    void testFieldBusBoard()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        uint8_t type = 0, version = 0, connState = 0;
        uint8_t ctrl[8];
        double ctrlAO[8];
        static uint8_t DI[8];
        static double AI[8];

        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return;
        }
        robot.SetReConnectParam(true, 30000, 500);
        //Upload and load the open protocol file
        robot.OpenLuaUpload("E://Project/Peripheral SDK/CtrlDev_field.lua");
        robot.Sleep(2000);
        robot.SetCtrlOpenLUAName(3, "CtrlDev_field.lua");
        robot.UnloadCtrlOpenLUA(3);
        robot.LoadCtrlOpenLUA(3);
        robot.Sleep(8000);

        //Get the protocol type, software version, and connection status with the PLC of the slave board
        robot.GetFieldBusConfig(&type, &version, &connState);
        printf("type is %d, version is %d,connState is %d\n", type, version, connState);

        //Write DO0 = 1, DO1 = 0, DO2 = 1
        ctrl[0] = 1; // Corrected value from 0 to 1 based on comment (DO0=1)
        ctrl[1] = 0;
        ctrl[2] = 1;
        robot.FieldBusSlaveWriteDO(0, 3, ctrl);

        //Write AO2 = 0x1005 (value from ctrlAO[0])
        ctrlAO[0] = 0x1005;
        robot.FieldBusSlaveWriteAO(2, 1, ctrlAO);

        //Loop to monitor DI0~DI3 AI0~AI2
        for (int i = 0; i < 100; i++)
        {
            robot.FieldBusSlaveReadDI(0, 4, DI);
            printf("DI0 is %d, DI1 is %d,DI2 is %d,DI3 is %d\n", DI[0], DI[1], DI[2], DI[3]);
            robot.FieldBusSlaveReadAI(0, 3, AI);
            printf("AI0 is %d, AI1 is %d,AI2 is %d\n", AI[0], AI[1], AI[2]);
            robot.Sleep(10);
        }

        //Wait for DI0 to become 1, wait time 100ms, and print the result
        int ret = robot.FieldBusSlaveWaitDI(0, 1, 100);
        printf("FieldBusSlaveWaitDI result is  %d\n", ret);

        //Wait for AI0 to be greater than 400, wait time 100ms, and print the result
        ret = robot.FieldBusSlaveWaitAI(0,0,400.00,100);
        printf("FieldBusSlaveWaitAI result is  %d\n", ret);

        robot.CloseRPC();
    }

Laser Peripheral On/Off
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Laser Peripheral On/Off Function
     * @param [in] OnOff 0- Off 1- On
     * @param [in] weldId Weld ID, default is 0
     * @return Error code
     */
    errno_t LaserTrackingLaserOnOff(int OnOff,int weldId);
        
Laser Tracking Start/Stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Laser Tracking Start/Stop Function
     * @param [in] OnOff 0- Stop 1- Start
     * @param [in] coordId Laser peripheral tool coordinate system ID
     * @return Error code
     */
     errno_t LaserTrackingTrackOnOff(int OnOff, int coordId); 
            
Laser Seam Search Start - Fixed Direction
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Laser Seam Search - Fixed Direction
    * @param [in] direction 0- X+ 1- X- 2- Y+ 3- Y- 4- Z+ 5- Z-
    * @param [in] vel Speed, unit %
    * @param [in] distance Maximum search distance, unit mm
    * @param [in] timeout Search timeout time, unit ms
    * @param [in] posSensorNum Laser calibrated tool coordinate number
    * @return Error code
    */
    errno_t LaserTrackingSearchStart_xyz(int direction, int vel, int distance, int timeout, int posSensorNum);
                
Laser Seam Search Start - Arbitrary Point Direction
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Laser Seam Search - Arbitrary Direction
     * @param [in] directionPoint Search input point's xyz coordinates
     * @param [in] vel Speed, unit %
     * @param [in] distance Maximum search distance, unit mm
     * @param [in] timeout Search timeout time, unit ms
     * @param [in] posSensorNum Laser calibrated tool coordinate number
     * @return Error code
     */
    errno_t LaserTrackingSearchStart_point(DescTran directionPoint, int vel, int distance, int timeout, int posSensorNum);
                    
Laser Seam Search Stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Laser Seam Search Stop
     * @return Error code
     */
    errno_t LaserTrackingSearchStop();

Laser Network Parameter Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Laser Network Parameter Configuration
     * @param [in] ip Laser peripheral IP address
     * @param [in] port Laser peripheral port number
     * @return Error code
     */
    errno_t LaserTrackingSensorConfig(std::string ip, int port);
    
Laser Peripheral Sampling Period Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Laser Peripheral Sampling Period Configuration
    * @param [in] period Laser peripheral sampling period, unit ms
    * @return Error code
    */
    errno_t LaserTrackingSensorSamplePeriod(int period);
        
Laser Peripheral Driver Load
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Laser Peripheral Driver Load
     * @param [in] type Laser peripheral driver protocol type 101- Ruiniu 102- Chuangxiang 103- Quanshi 104- Tongzhou 105- Aotai
     * @return Error code
     */
    errno_t LoadPosSensorDriver(int type);
            
Laser Peripheral Driver Unload
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Laser Peripheral Driver Unload
     * @return Error code
     */
    errno_t UnLoadPosSensorDriver();
                
Laser Seam Trajectory Recording
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Laser Seam Trajectory Recording
    * @param [in] status 0- Stop recording 1- Real-time tracking  2- Start recording
    * @param [in] delayTime Delay time, unit ms
    * @return Error code
    */
    errno_t LaserSensorRecord1(int status, int delayTime); 
                    
Laser Seam Trajectory Replay
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Laser Seam Trajectory Replay
     * @param [in] delayTime Delay time, unit ms
     * @param [in] speed Speed, unit %
     * @return Error code
     */
    errno_t LaserSensorReplay(int delayTime, double speed);
                        
Laser Tracking Replay
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Laser Tracking Replay
     * @return Error code
     */
    errno_t MoveLTR();
                            
Laser Seam Trajectory Recording and Replay
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Laser Seam Trajectory Recording and Replay
    * @param [in] delayMode Mode 0-Delay Time 1-Delay Distance
    * @param [in] delayTime Delay time in milliseconds (ms)
    * @param [in] delayDisExAxisNum Extended Axis Number
    * @param [in] delayDis Delay distance in millimeters (mm)
    * @param [in] sensitivePara Compensation Sensitivity Coefficient
    * @param [in] trackMode Fixed-point Tracking Type. 0-Extended Axis Asynchronous Motion; 1-Robot
    * @param [in] triggerMode Fixed-point Tracking Trigger Method. 0-Tracking Duration; 1-IO
    * @param [in] runTime Robot Fixed-point Tracking Duration in seconds (s)
    * @param [in] speed Speed in percentage (%)
    * @return Error Code
    */
    errno_t LaserSensorRecordandReplay(int delayMode, int delayTime, int delayDisExAxisNum, double delayDis, double sensitivePara, int trackMode, int triggerMode, double runTime, double speed);
                                
Move to Laser Recorded Start Point
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Move to Laser Recorded Start Point
    * @param [in] moveType 0- moveJ 1- moveL
    * @param [in] ovl Speed, unit %
    * @return Error code
    */
    errno_t MoveToLaserRecordStart(int moveType, double ovl);
                                    
Move to Laser Recorded End Point
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Move to Laser Recorded End Point
    * @param [in] moveType 0- moveJ 1- moveL
    * @param [in] ovl Speed, unit %
    * @return Error code
    */
    errno_t MoveToLaserRecordEnd(int moveType, double ovl);
                                        
Move to Laser Sensor Seam Search Point
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Move to Laser Sensor Seam Search Point
    * @param [in] moveFlag Motion type: 0- PTP; 1- LIN
    * @param [in] ovl Speed scaling factor, 0-100
    * @param [in] dataFlag Seam buffer data selection: 0- Execute planned data; 1- Execute recorded data
    * @param [in] plateType Plate type: 0- Corrugated plate; 1- Corrugated cardboard; 2- Fence plate; 3- Oil drum; 4- Corrugated shell steel
    * @param [in] trackOffectType Laser sensor offset type: 0- No offset; 1- Base coordinate system offset; 2- Tool coordinate system offset; 3- Laser sensor raw data offset
    * @param [in] offset Offset amount
    * @return Error code
    */
    errno_t MoveToLaserSeamPos(int moveFlag, double ovl, int dataFlag, int plateType, int trackOffectType, DescPose offset);
                                            
Get Laser Sensor Seam Search Point Coordinates
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get Laser Sensor Seam Search Point Coordinate Information
    * @param [in] trackOffectType Laser sensor offset type: 0- No offset; 1- Base coordinate system offset; 2- Tool coordinate system offset; 3- Laser sensor raw data offset
    * @param [in] offset Offset amount
    * @param [out] jPos Joint position [°]
    * @param [out] descPos Cartesian position [mm]
    * @param [out] tool Tool coordinate system
    * @param [out] user Work object coordinate system
    * @param [out] exaxis Extended axis position [mm]
    * @return Error code
    */
    errno_t GetLaserSeamPos(int trackOffectType, DescPose offset, JointPos& jPos, DescPose& descPos, int& tool, int& user, ExaxisPos& exaxis); 
                                                
Laser Peripheral Sensor Parameter Configuration and Debugging Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    void testLaserConfig()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        uint8_t ctrl[20];
        uint8_t state;
        int pressVlaue;
        int error;
        robot.CloseRPC();
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return;
        }
        robot.SetReConnectParam(true, 30000, 500);
        //Set IP address and port number
        robot.LaserTrackingSensorConfig("192.168.58.20", 5020);
        //Set sampling period
        robot.LaserTrackingSensorSamplePeriod(20);
        //Load driver
        robot.LoadPosSensorDriver(101);
        //Turn off laser peripheral
        robot.LaserTrackingLaserOnOff(0,0);
        robot.Sleep(3000);
        //Turn on laser peripheral
        robot.LaserTrackingLaserOnOff(1, 0);
        robot.CloseRPC();
    }
                                                    
Laser Trajectory Scanning and Trajectory Replay Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    void testLaserRecordAndReplay()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        uint8_t ctrl[20];
        uint8_t state;
        int pressVlaue;
        int error;
        robot.CloseRPC();
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return;
        }
        robot.SetReConnectParam(true, 30000, 500);

        //Upload and load open protocol file
        robot.OpenLuaUpload("E://openlua/CtrlDev_laser_ruiniu-0117.lua");
        robot.Sleep(2000);
        robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua");
        robot.UnloadCtrlOpenLUA(0);
        robot.LoadCtrlOpenLUA(0);
        robot.Sleep(8000);
        int cnt = 1;
        while(cnt<31)
        { 
            //Move to scan start point
            JointPos startjointPos(56.205, -117.951, 141.872, -118.149, -94.217, -122.176);
            DescPose startdescPose(-97.552, -282.855, 26.675, 174.182, -1.338, -91.707);
            ExaxisPos exaxisPos(0, 0, 0, 0);
            DescPose offdese(0, 0, 0, 0, 0, 0);
            robot.MoveL(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
            //Start trajectory recording
            robot.LaserSensorRecord1(2, 10);
            //Move to the end point to be recorded
            JointPos endjointPos(68.809, -87.100, 121.120, -127.233, -95.038, -109.555);
            DescPose enddescPose(-103.555, -464.234, 13.076, 174.179, -1.344, -91.709);
            robot.MoveL(&endjointPos, &enddescPose, 1, 0, 30, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
            //Stop recording
            robot.LaserSensorRecord1(0, 10);
            //Move to the recorded seam start point
            robot.MoveToLaserRecordStart(1, 30);
            //Start trajectory replay
            robot.LaserSensorReplay(10, 100);
            robot.MoveLTR();
            //Stop trajectory replay
            robot.LaserSensorRecord1(0, 10);
            printf("Laser scanning + trajectory replay stability test, %dth time\n", cnt);
            cnt++;
        }
        robot.CloseRPC();
    }
                                                        
Laser Seam Search and Real-time Tracking Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    void testLasertrack()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        uint8_t ctrl[20];
        uint8_t state;
        int pressVlaue;
        int error;
        robot.CloseRPC();
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");

        if (rtn != 0)
        {
            return;
        }
        robot.SetReConnectParam(true, 30000, 500);

        //Upload and load open protocol file
        robot.OpenLuaUpload("E://openlua/CtrlDev_laser_ruiniu-0117.lua");
        robot.Sleep(2000);
        robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua");
        robot.UnloadCtrlOpenLUA(0);
        robot.LoadCtrlOpenLUA(0);
        robot.Sleep(8000);
        int cnt = 1;
        while (cnt < 2)
        {
            //Move to the starting point for seam search
            JointPos startjointPos(58.337, -119.628, 146.037, -116.358, -92.224, -117.654);
            DescPose startdescPose(-53.375, -255.363, 0.919, 178.054, 1.077, -94.026);
            ExaxisPos exaxisPos(0, 0, 0, 0);
            DescPose offdese(0, 0, 0, 0, 0, 0);
            DescTran directionPoint;
            robot.MoveL(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);

            //Start search along the -Y direction
            int ret = robot.LaserTrackingSearchStart_xyz(3, 100, 300, 1000, 2);
            robot.LaserTrackingSearchStop();
            //If search is successful
            if (ret == 0)
            {
                //Move to the search point
                robot.MoveToLaserSeamPos(1, 30, 0, 0, 0, offdese);
                //Start laser tracking along the search point
                robot.LaserTrackingTrackOnOff(1, 2);
                JointPos endjointPos(70.580, -90.918, 126.593, -125.154, -92.162, -105.403);
                DescPose enddescPose(-53.375, -419.020, 0.920, 178.054, 1.076, -94.026);
                robot.MoveL(&endjointPos, &enddescPose, 1, 0, 20, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
                //Stop tracking
                robot.LaserTrackingTrackOnOff(0, 2);

            }
            cnt++;
        }
        robot.CloseRPC();
    }
                                                            
Extended Axis and Robot Synchronized Laser Tracking Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    void testLasertrackandExitAxis()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        uint8_t ctrl[20];
        uint8_t state;
        int pressVlaue;
        int error;
        robot.CloseRPC();
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");

        if (rtn != 0)
        {
            return;
        }
        robot.SetReConnectParam(true, 30000, 500);

        ExaxisPos startexaxisPos = { 0,0,0,0 };
        ExaxisPos seamexaxisPos = { -10,0,0,0 };
        ExaxisPos endexaxisPos = { -30, 0, 0, 0 };
        DescPose offdese = { 0, 0, 0, 0, 0, 0 };
        JointPos seamjointPos(0, 0, 0, 0, 0, 0);
        DescPose seamdescPose(0, 0, 0, 0, 0, 0);
        
        int cnt = 1;
        while (cnt < 31)
        {
            //Move to the starting point for seam search
            JointPos startjointPos(58.337, -119.628, 146.037, -116.358, -92.224, -117.654);
            DescPose startdescPose(-53.375, -255.363, 0.919, 178.054, 1.077, -94.026);
            robot.ExtAxisSyncMoveJ(startjointPos, startdescPose, 1, 0, 100, 100, 100, startexaxisPos, -1, 0, offdese);

            //Start search along the -Y direction
            int ret = robot.LaserTrackingSearchStart_xyz(3, 100, 300, 1000, 2);
            robot.LaserTrackingSearchStop();
            int tool = 0;
            int user = 0;
            robot.GetLaserSeamPos(0, offdese, seamjointPos, seamdescPose, tool, user, startexaxisPos);
            printf("%f, %f, %f,%f, %f, %f,%f, %f, %f,%f, %f, %f\n", seamjointPos.jPos[0], seamjointPos.jPos[1], seamjointPos.jPos[2], seamjointPos.jPos[3], seamjointPos.jPos[4], seamjointPos.jPos[5], seamdescPose.tran.x, seamdescPose.tran.y, seamdescPose.tran.z, seamdescPose.rpy.rx, seamdescPose.rpy.ry, seamdescPose.rpy.rz);

            //If search is successful
            if (ret == 0)
            {
                //Robot and extended axis synchronously move to the search point
                robot.ExtAxisSyncMoveJ(seamjointPos, seamdescPose, 1, 0, 100, 100, 100, seamexaxisPos, -1, 0, offdese);

                //Start laser tracking along the search point and synchronously move with the extended axis
                robot.LaserTrackingTrackOnOff(1, 2);
                JointPos endjointPos(70.580, -90.918, 126.593, -125.154, -92.162, -105.403);
                DescPose enddescPose(-53.375, -419.020, 0.920, 178.054, 1.076, -94.026);
                robot.ExtAxisSyncMoveL(endjointPos, enddescPose, 1, 0, 20, 100, 100, -1, endexaxisPos, 0, offdese);;
                //Stop tracking
                robot.LaserTrackingTrackOnOff(0, 2);
            }
            cnt++;
            printf("Extended axis and robot synchronized laser tracking, %dth time\n", cnt);
        }
        robot.CloseRPC();
    } 

End-Effector Transparent Transmission Function Enable/Disable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Enable/disable end-effector transparent transmission function
    * @param [in] enable, 0-disable, 1-enable
    * @return Error code
    */
    errno_t SetAxleGenComEnable(int mode);
                                                            
End-Effector Transparent Transmission Function Non-Periodic Data Transmission and Reception
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:
    
    /**
    * @brief End-effector transparent transmission function non-periodic data transmission and reception
    * @param [in] len_snd Length of data to send
    * @param [in] sndBuff Data to send
    * @param [in] len_rcv Length of data to receive
    * @param [out] rcvBuff Response data
    * @return Error code
    */
    errno_t SndRcvAxleGenComCmdData(int lenSnd, int sndBuff[130], int lenRcv, int rcvData[130]);
                                                                
Code Example for Non-Periodic Data Communication of DIO Health Care Moxibustion Head Based on End-Effector Transparent Transmission Function
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int testAxleGenCom()
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
      int led_on[6] = { 0xAB, 0xBA, 0x12, 0x01, 0x01, 0x79 };
      int led_off[6] = { 0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78 };
      int version[5] = { 0xAB, 0xBA, 0x11, 0x00, 0x76 };
      int state[6] = { 0xAB, 0xBA, 0x1B, 0x01, 0xAA, 0x2B };
      int cycleState[6] = { 0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78 };
      int rcvdata[16] = {0};
      int ret = 0;
      int cnt = 1;
      JointPos p1Joint(88.708, -86.178, 140.989, -141.825, -89.162, -49.879);
      DescPose p1Desc(188.007, -377.850, 260.207, 178.715, 2.823, -131.466);
      JointPos p2Joint(112.131, -75.554, 126.989, -139.027, -88.044, -26.477);
      DescPose p2Desc(368.003, -377.848, 260.211, 178.715, 2.823, -131.465);
      ExaxisPos exaxisPos(0, 0, 0, 0);
      DescPose offdese(0, 0, 0, 0, 0, 0);
      //Enable end-effector transparent transmission function
      robot.SetAxleGenComEnable(1);
      robot.SetAxleLuaEnable(1);
      while (cnt <= 10000)
      {
        //Read version number
        ret = robot.SndRcvAxleGenComCmdData(5, version, 10, rcvdata);
        printf(" hard version : %d,hard code:%d, soft version:%d %d, soft code:%d \n", rcvdata[4], rcvdata[5], rcvdata[6] ,rcvdata[7], rcvdata[8]);
        if (ret != 0)
        {
          break;
        }
        robot.Sleep(1000);
        //Read moxibustion head presence status
        ret = robot.SndRcvAxleGenComCmdData(6, state, 6, rcvdata);
        printf(" state : %d \n", rcvdata[4]);
        robot.Sleep(1000);
        //Turn on moxibustion head laser
        ret = robot.SndRcvAxleGenComCmdData(6, led_on, 6, rcvdata);
        printf("led on rcv data is: %d, %d, %d, %d, %d, %d  \n", rcvdata[0], rcvdata[1], rcvdata[2], rcvdata[3], rcvdata[4], rcvdata[5]);
        robot.MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.Sleep(4000);
        //Turn off moxibustion head laser
        ret = robot.SndRcvAxleGenComCmdData(6, led_off, 6, rcvdata);
        printf("led off rcv data is: %d, %d, %d, %d, %d, %d \n", rcvdata[0], rcvdata[1], rcvdata[2], rcvdata[3], rcvdata[4], rcvdata[5]);
        robot.MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.Sleep(1000);
        printf("***********************complate No. %d SDK test*****************************\n", cnt);
        cnt++;
      }
      robot.CloseRPC();
    }

Download Open Protocol Lua File
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Download open protocol Lua file
    * @param [in] fileName Open protocol file name "CtrlDev_XXX.lua"
    * @param [in] savePath Path to save the open protocol file
    * @return Error code
    */
    errno_t OpenLuaDownload(std::string fileName, std::string savePath);
    
Delete Open Protocol Lua File
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Delete open protocol Lua file
    * @param [in] fileName Name of the open protocol Lua file to delete "CtrlDev_XXX.lua"
    * @return Error code
    */
    errno_t OpenLuaDelete(std::string fileName);
        
Delete All Open Protocol Lua Files
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Delete all open protocol Lua files
    * @return Error code
    */
    errno_t AllOpenLuaDelete();

Code Example for Controller Peripheral Open Protocol Upload, Download, and Delete
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestCtrlOpenLuaOperate()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return 0;
        }
        robot.SetReConnectParam(true, 30000, 500);
        rtn = robot.OpenLuaUpload("D://zUP/openlua/CtrlDev_WELDING_A.lua");
        printf("OpenLuaUpload rtn is %d\n", rtn);
        rtn = robot.OpenLuaUpload("D://zUP/openlua/CtrlDev_SWDPOLISH.lua");
        printf("OpenLuaUpload rtn is %d\n", rtn);
        rtn = robot.OpenLuaDownload("CtrlDev_WELDING_A.lua", "D://zDOWN/");
        printf("OpenLuaDownload rtn is %d\n", rtn);
        rtn = robot.OpenLuaDownload("CtrlDev_SWDPOLISH.lua", "D://zDOWN/");
        printf("OpenLuaDownload rtn is %d\n", rtn);
        rtn = robot.SetCtrlOpenLUAName(0, "CtrlDev_WELDING_A.lua");
        printf("SetCtrlOpenLUAName rtn is %d\n", rtn);
        rtn = robot.SetCtrlOpenLUAName(1, "CtrlDev_SWDPOLISH.lua");
        printf("SetCtrlOpenLUAName rtn is %d\n", rtn);
        std::string name[4] = {};
        rtn = robot.GetCtrlOpenLUAName(name);
        printf("ctrl open lua names : %s, %s, %s, %s\n", name[0].c_str(), name[1].c_str(), name[2].c_str(), name[3].c_str());
        rtn = robot.LoadCtrlOpenLUA(1);
        printf("LoadCtrlOpenLUA rtn is %d\n", rtn);
        robot.Sleep(2000);
        rtn = robot.UnloadCtrlOpenLUA(1);
        printf("UnloadCtrlOpenLUA rtn is %d\n", rtn);
        rtn = robot.OpenLuaDelete("CtrlDev_WELDING_A.lua");
        printf("OpenLuaDelete rtn is %d\n", rtn);
        rtn = robot.AllOpenLuaDelete();
        printf("AllOpenLuaDelete rtn is %d\n", rtn);
        robot.CloseRPC();
        robot.Sleep(1000);
        return 0;
    }

Control Dexterous Hand Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Control dexterous hand motion
    * @param [in] idstart Starting slave station number
    * @param [in] slaveNum Number of slaves
    * @param [in] pos Position array, length 16, range (-360~360)
    * @param [in] speed Speed percentage array, length 16, range [0~100]
    * @param [in] force Torque percentage array, length 16, range [0~100]
    * @param [in] max_time Maximum wait time, range [0~30000], unit ms
    * @return Error code, 0 on success
    */
    errno_t SetDexterousHandsMove(int idstart, int slaveNum, double pos[16], int speed[16], int force[16], int max_time);
        
Control Dexterous Hand Reset and Activation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Control dexterous hand reset and activation
    * @param [in] id Slave station number
    * @param [in] act 0-reset 1-activate
    * @return Error code, 0 on success
    */
    errno_t SetDexterousHandsAct(int id, int act);
            
Clear Dexterous Hand Error
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Clear dexterous hand error
    * @return Error code, 0 on success
    */
    errno_t ClearDexterousHandsError();
                
Set Enabled Dexterous Hand Action Control Functions
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set enabled dexterous hand action control functions
    * @param [in] id Dexterous hand device number
    * @param [in] func 0-grip trigger, 1-gripper initialization, 2-position setting, 3-speed setting, 4-torque setting, 6-read gripper status, 7-read initialization status, 8-read fault code, 9-read position, 10-read speed, 11-read torque, 12-rotation count setting, 13-rotation speed setting, 14-rotation torque setting, 15-read rotary gripper status, 16-read rotary initialization status, 17-read rotation count, 18-read rotation speed, 19-read rotation torque, 20-multi-axis synchronous motion setting, 21-fault clear command, 22-single-axis running status, 23-all-axis running status
    * @return Error code
    */
    errno_t SetDexterousHandsFunc(int id, int func[32]);
                    
Get Enabled Dexterous Hand Action Control Functions
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get enabled dexterous hand action control functions
    * @param [in] id Dexterous hand device number
    * @param [out] func 0-grip trigger, 1-gripper initialization, 2-position setting, 3-speed setting, 4-torque setting, 6-read gripper status, 7-read initialization status, 8-read fault code, 9-read position, 10-read speed, 11-read torque, 12-rotation count setting, 13-rotation speed setting, 14-rotation torque setting, 15-read rotary gripper status, 16-read rotary initialization status, 17-read rotation count, 18-read rotation speed, 19-read rotation torque, 20-multi-axis synchronous motion setting, 21-fault clear command, 22-single-axis running status, 23-all-axis running status
    * @return Error code
    */
    errno_t GetDexterousHandsFunc(int id, int func[32]);
                        
End-Effector Dexterous Hand Configuration and Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestDexterousHands()
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
    int id = 1;        // Slave station number
    int slaveNum = 4;     // Control 4 fingers
    int max_time = 8000;   // Maximum wait time 8 seconds
    int speed[16] = {0};   // Speed array, all 0 means use default speed
    int force[16] = {0};   // Torque array
    // Initialize torque array: first 4 fingers set to 50%, the rest 0 (values sent via Move command)
    for (int i = 0; i < 16; i++)
    {
        force[i] = (i < 4) ? 50 : 0;
    }
    // Helper function: set position array (only first 4 fingers are effective)
    double pos[16] = {0.0};
    JointPos j1(-91.876, -85.920, 109.279, -86.239, -96.664, -28.563);
    JointPos j2(-40.954, -85.920, 109.279, -86.239, -96.664, -28.563);
    ExaxisPos epos(0, 0, 0, 0);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    printf("===== Dexterous Hand Full Function Test Started =====\n");
    // 1. Clear error
    int ret = robot.ClearDexterousHandsError();
    printf("ClearDexterousHandsError rtn %d\n", ret);
    // ========== 2. Set function switches ==========
    int setFunc[32] = {0};
    setFunc[2] = 1;  // Enable position setting function
    setFunc[4] = 1;  // Enable torque setting function
    setFunc[9] = 1;  // Read position
    setFunc[10] = 1;  // Read torque
    setFunc[11] = 1;  // Read status
    setFunc[22] = 1;  // Single-axis motion status
    ret = robot.SetDexterousHandsFunc(id, setFunc);
    printf("SetDexterousHandsFunc(enable + init + position/torque functions enabled) rtn %d\n", ret);
    // ========== 3. Read function status (verify settings took effect) ==========
    int getFunc[32] = {0}; // GetDexterousHandsFunc returns 32 integers
    ret = robot.GetDexterousHandsFunc(id, getFunc);
    printf("GetDexterousHandsFunc rtn %d\n", ret);
    if (ret == 0)
    {
        // Print all 32 values
        printf("All 32 values returned by GetDexterousHandsFunc:");
        for (int i = 0; i < 32; i++)
        {
        printf(" [%d]={%d}", i, getFunc[i]);
        if ((i + 1) % 8 == 0)
        {
            printf("\n");
        } 
        else if (i < 31)
        {
            printf(", ");
        }
        }
    }
    // ========== 4. Activate dexterous hand ==========
    ret = robot.SetDexterousHandsAct(id, 1);
    printf("SetDexterousHandsAct(activate) rtn %d\n", ret);
    if (ret != 0)
    {
        printf("Activation failed, test aborted");
        return -1;
    }
    // ========== 5. Initial move to 20° (send position and torque values via Move command) ==========
    memset(pos, 0, sizeof(pos));
    pos[0] = 20;
    pos[1] = 20;
    pos[2] = 20;
    pos[3] = 20;
    ret = robot.SetDexterousHandsMove(id, slaveNum, pos, speed, force, max_time);
    printf("Initial move to 20° -> %d\n", ret);
    robot.Sleep(5000);
    // ========== 6. Reciprocating motion 10 times (10° ↔ 50°) ==========
    printf("Starting 10 reciprocating motions...");
    for (int iteration = 1; iteration <= 10; iteration++)
    {
        robot.MoveJ(&j1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        memset(pos, 0, sizeof(pos));
        pos[0] = 10;
        pos[1] = 10;
        pos[2] = 10;
        pos[3] = 10;
        ret = robot.SetDexterousHandsMove(id, slaveNum, pos, speed, force, max_time);
        printf("Move to 10° -> %d\n", ret);
        robot.Sleep(1000);
        robot.MoveJ(&j2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        memset(pos, 0, sizeof(pos));
        pos[0] = 50;
        pos[1] = 50;
        pos[2] = 50;
        pos[3] = 50;
        ret = robot.SetDexterousHandsMove(id, slaveNum, pos, speed, force, max_time);
        printf("Move to 50° -> %d\n", ret);
        robot.Sleep(1000);
    }
    printf("Test completed (function switch set/read + activation + 10 reciprocating motions).");
    return 0;
    }    