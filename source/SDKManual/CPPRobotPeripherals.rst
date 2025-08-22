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
    * @param [in] followType Tracking motion type, 0-tracking motion; 1-chase inspection motion
    * @param [in] startDis For chase capture: tracking start distance (-1: auto calculate), unit mm, default 0
    * @param [in] endDis For chase capture: tracking end distance, unit mm, default 100
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

Set End LUA End Device Enable Type
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set end LUA end device enable type
    * @param forceSensorEnable Force sensor enable status, 0-Disable; 1-Enable
    * @param gripperEnable Gripper enable status, 0-Disable; 1-Enable
    * @param IOEnable IO device enable status, 0-Disable; 1-Enable
    * @return  Error code
    */
    errno_t SetAxleLuaEnableDeviceType(int forceSensorEnable, int gripperEnable, int IOEnable);

Get End LUA End Device Enable Type
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:
        
    /**
    * @brief Get end LUA end device enable type
    * @param enable enable[0]:forceSensorEnable Force sensor enable status, 0-Disable; 1-Enable
    * @param enable enable[1]:gripperEnable Gripper enable status, 0-Disable; 1-Enable
    * @param enable enable[2]:IOEnable IO device enable status, 0-Disable; 1-Enable
    * @return  Error code
    */
    errno_t GetAxleLuaEnableDeviceType(int* forceSensorEnable, int* gripperEnable, int* IOEnable);

Get Currently Configured End Devices
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get currently configured end devices
    * @param forceSensorEnable Force sensor enable device number 0-Not enabled; 1-Enabled
    * @param gripperEnable Gripper enable device number, 0-Disable; 1-Enable
    * @param IODeviceEnable IO device enable device number, 0-Disable; 1-Enable
    * @return  Error code
    */
    errno_t GetAxleLuaEnableDevice(int forceSensorEnable[], int gripperEnable[], int IODeviceEnable[]);

Set Enable Gripper Action Control Function
+++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set enable gripper action control function
    * @param id Gripper device number
    * @param func func[0]-Gripper enable; func[1]-Gripper initialization; 2-Position setting; 3-Speed setting; 4-Torque setting; 6-Read gripper status; 7-Read initialization status; 8-Read fault code; 9-Read position; 10-Read speed; 11-Read torque
    * @return  Error code
    */
    errno_t SetAxleLuaGripperFunc(int id, int func[]);

Get Enable Gripper Action Control Function
+++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get enable gripper action control function
    * @param id Gripper device number
    * @param func func[0]-Gripper enable; func[1]-Gripper initialization; 2-Position setting; 3-Speed setting; 4-Torque setting; 6-Read gripper status; 7-Read initialization status; 8-Read fault code; 9-Read position; 10-Read speed; 11-Read torque
    * @return  Error code
    */
    errno_t GetAxleLuaGripperFunc(int id, int func[]);

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
      robot.SetAxleLuaEnableDeviceType(0, 1, 0);
      int forceEnable = 0;
      int gripperEnable = 0;
      int ioEnable = 0;
      robot.GetAxleLuaEnableDeviceType(&forceEnable, &gripperEnable, &ioEnable);
      printf("GetAxleLuaEnableDeviceType param is %d %d %d\n", forceEnable, gripperEnable, ioEnable);
      int func[16] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
      robot.SetAxleLuaGripperFunc(1, func);
      int getFunc[16] = { 0 };
      robot.GetAxleLuaGripperFunc(1, getFunc);
      int getforceEnable[16] = { 0 };
      int getgripperEnable[16] = { 0 };
      int getioEnable[16] = { 0 };
      robot.GetAxleLuaEnableDevice(getforceEnable, getgripperEnable, getioEnable);
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
      robot.ActGripper(1, 0);
      robot.Sleep(2000);
      robot.ActGripper(1, 1);
      robot.Sleep(2000);
      robot.MoveGripper(1, 90, 10, 100, 50000, 0, 0, 0, 0, 0);
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
    errno_t FieldBusSlaveWriteAO(uint8_t AOIndex, uint8_t writeNum, int status[8]);

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
    errno_t FieldBusSlaveReadAI(uint8_t AIIndex, uint8_t readNum, int status[8]);

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
        int ctrlAO[8];
        static uint8_t DI[8];
        static int AI[8];

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