Force control
=======================

.. toctree:: 
    :maxdepth: 5

Force sensor configuration
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief force sensor configuration
    * @param [in] company force sensor manufacturer, 17-Kunwei Technology, 19-Aerospace 11th Institute, 20-ATI Sensor, 21-Zhongke Midian, 22-Weihang Minxin, 23-NBIT, 24-Xinjingcheng (XJC), 26-NSR
    * @param [in] device device number, Kunwei (0-KWR75B), Aerospace 11th Institute (0-MCS6A-200-4), ATI (0-AXIA80-M8), Zhongke Midian (0-MST2010), Weihang Minxin (0-WHC6L-YB-10A), NBIT (0-XLH93003ACS), Xinjingcheng XJC (0-XJC-6F-D82), NSR (0-NSR-FTSensorA)
    * @param [in] softvesion Software version number, not used yet, default is 0
    * @param [in] bus device is hung at the end bus position, not used yet, default is 0
    * @return error code
    */
    errno_t  FT_SetConfig(int company, int device, int softvesion, int bus);

Get the force sensor configuration
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the force sensor configuration
    * @param  [in] company  Force sensor manufacturer, to be determined
    * @param  [in] device  Device number, not used yet. The default value is 0
    * @param  [in] softvesion  Software version. The value is not used. The default value is 0
    * @param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    * @return  Error code
    */
    errno_t  FT_GetConfig(int *company, int *device, int *softvesion, int *bus);

Force Sensor Activation
+++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Force sensor activation
    * @param  [in] act  0- reset, 1- activate
    * @return  Error code
    */
    errno_t  FT_Activate(uint8_t act);

Force sensor calibration
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Force sensor calibration
    * @param  [in] act  0- zero removal, 1- zero correction
    * @return  Error code
    */
    errno_t  FT_SetZero(uint8_t act);   

Set Force Sensor Reference Coordinate System
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set force sensor reference coordinate system
    * @param  [in] ref  0-Tool coordinate system, 1-Base coordinate system
    * @return  Error code
    */
    errno_t  FT_SetRCS(uint8_t ref); 

Set Payload Weight Under Force Sensor
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Set payload weight under force sensor
     * @param [in] weight  Payload weight in kg
     * @return  Error code
     */
    errno_t SetForceSensorPayload(double weight);

Set Payload Center of Gravity Under Force Sensor
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Set payload center of gravity under force sensor
     * @param [in] x  Payload COG x in mm
     * @param [in] y  Payload COG y in mm
     * @param [in] z  Payload COG z in mm
     * @return  Error code
     */
    errno_t SetForceSensorPayloadCog(double x, double y, double z);

Get Payload Weight Under Force Sensor
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:
    
    /**
     * @brief Get payload weight under force sensor
     * @param [in] weight  Payload weight in kg
     * @return  Error code
     */
    errno_t GetForceSensorPayload(double& weight);

Get Payload Center of Gravity Under Force Sensor
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Get payload center of gravity under force sensor
     * @param [out] x  Payload COG x in mm
     * @param [out] y  Payload COG y in mm
     * @param [out] z  Payload COG z in mm
     * @return  Error code
     */
    errno_t GetForceSensorPayloadCog(double& x, double& y, double& z);

Force Sensor Auto Zero Calibration
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Force sensor auto zero calibration
     * @param [out] weight  Sensor mass in kg
     * @param [out] pos  Sensor COG in mm
     * @return  Error code
     */
    errno_t ForceSensorAutoComputeLoad(double& weight, DescTran& pos);

Get Force/Torque Data in Reference Coordinate System
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get force/torque data in reference coordinate system
    * @param  [out] ft  Force/torque, fx, fy, fz, tx, ty, tz
    * @return  Error code
    */   
    errno_t  FT_GetForceTorqueRCS(ForceTorque *ft); 

Get Raw Force/Torque Data from Force Sensor
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get raw force/torque data from force sensor
    * @param  [out] ft  Force/torque, fx, fy, fz, tx, ty, tz
    * @return  Error code
    */   
    errno_t  FT_GetForceTorqueOrigin(ForceTorque *ft); 

Force Sensor Configuration and Auto Zero Calibration Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestFTInit(void)
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
      int company = 24;
      int device = 0;
      int softversion = 0;
      int bus = 1;
      int index = 1;
      robot.FT_SetConfig(company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_GetConfig(&company, &device, &softversion, &bus);
      printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_Activate(0);
      robot.Sleep(1000);
      robot.FT_Activate(1);
      robot.Sleep(1000);
      robot.Sleep(1000);
      robot.FT_SetZero(0);
      robot.Sleep(1000);
      ForceTorque ft;
      memset(&ft, 0, sizeof(ForceTorque));
      robot.FT_GetForceTorqueOrigin(0, &ft);
      printf("ft origin:%f,%f,%f,%f,%f,%f\n", ft.fx, ft.fy, ft.fz, ft.tx, ft.ty, ft.tz);
      robot.FT_SetZero(1);
      robot.Sleep(1000);
      DescPose ftCoord = {};
      robot.FT_SetRCS(0, ftCoord);
      robot.SetForceSensorPayload(0.824);
      robot.SetForceSensorPayloadCog(0.778, 2.554, 48.765);
      double weight = 0;
      double x = 0, y = 0, z = 0;
      robot.GetForceSensorPayload(weight);
      robot.GetForceSensorPayloadCog(x, y, z);
      printf("the FT load is %lf, %lf %lf %lf\n", weight, x, y, z);
      robot.SetForceSensorPayload(0);
      robot.SetForceSensorPayloadCog(0, 0, 0);
      double computeWeight = 0;
      DescTran tran = {};
      robot.ForceSensorAutoComputeLoad(weight, tran);
      cout << "the result is weight " << weight << " pos is " << tran.x << " " << tran.y << " " << tran.z << endl;
      robot.CloseRPC();
      return 0;
    }


Payload Weight Identification Record
+++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Payload weight identification record
    * @param  [in] id  Sensor coordinate system number, range [1~14]
    * @return  Error code
    */
    errno_t  FT_PdIdenRecord(int id);   

Payload Weight Identification Calculation
+++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Payload weight identification calculation
    * @param  [out] weight  Payload weight in kg
    * @return  Error code
    */   
    errno_t  FT_PdIdenCompute(float *weight);

Payload COG Identification Record
+++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Payload COG identification record
    * @param  [in] id  Sensor coordinate system number, range [1~14]
    * @param  [in] index  Point number, range [1~3]
    * @return  Error code
    */
    errno_t  FT_PdCogIdenRecord(int id, int index);    

Payload COG Identification Calculation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Payload COG identification calculation
    * @param  [out] cog  Payload COG in mm
    * @return  Error code
    */   
    errno_t  FT_PdCogIdenCompute(DescTran *cog); 

Force Sensor Payload Identification Code Example
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestFTLoadCompute(void)
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
      int company = 24;
      int device = 0;
      int softversion = 0;
      int bus = 1;
      int index = 1;
      robot.FT_SetConfig(company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_GetConfig(&company, &device, &softversion, &bus);
      printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_Activate(0);
      robot.Sleep(1000);
      robot.FT_Activate(1);
      robot.Sleep(1000);
      robot.Sleep(1000);
      robot.FT_SetZero(0);
      robot.Sleep(1000);
      ForceTorque ft;
      memset(&ft, 0, sizeof(ForceTorque));
      robot.FT_GetForceTorqueOrigin(0, &ft);
      printf("ft origin:%f,%f,%f,%f,%f,%f\n", ft.fx, ft.fy, ft.fz, ft.tx, ft.ty, ft.tz);
      robot.FT_SetZero(1);
      robot.Sleep(1000);
      DescPose tcoord = {};
      tcoord.tran.z = 35.0;
      robot.SetToolCoord(10, &tcoord, 1, 0, 0, 0);
      robot.FT_PdIdenRecord(10);
      robot.Sleep(1000);
      float weight = 0.0;
      robot.FT_PdIdenCompute(&weight);
      printf("payload weight:%f\n", weight);
      DescPose desc_p1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
      DescPose desc_p2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
      DescPose desc_p3(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
      robot.MoveCart(&desc_p1, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
      robot.Sleep(1000);
      robot.FT_PdCogIdenRecord(10, 1);
      robot.MoveCart(&desc_p2, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
      robot.Sleep(1000);
      robot.FT_PdCogIdenRecord(10, 2);
      robot.MoveCart(&desc_p3, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
      robot.Sleep(1000);
      robot.FT_PdCogIdenRecord(10, 3);
      robot.Sleep(1000);
      DescTran cog;
      memset(&cog, 0, sizeof(DescTran));
      robot.FT_PdCogIdenCompute(&cog);
      printf("cog:%f,%f,%f\n", cog.x, cog.y, cog.z);
      robot.CloseRPC();
      return 0;
    }

Collision Guard
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Collision guard
    * @param  [in] flag 0-Turn off collision guard, 1-Turn on collision guard
    * @param  [in] sensor_id  Force sensor number
    * @param  [in] select  Select whether to detect collision in six degrees of freedom, 0-Do not detect, 1-Detect
    * @param  [in] ft  Collision force/torque, fx, fy, fz, tx, ty, tz
    * @param  [in] max_threshold  Maximum threshold
    * @param  [in] min_threshold  Minimum threshold
    * @note   Force/torque detection range: (ft-min_threshold, ft+max_threshold)
    * @return  Error code
    */   
    errno_t  FT_Guard(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float max_threshold[6], float min_threshold[6]); 

Collision Guard Code Example
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestFTGuard(void)
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
      int company = 24;
      int device = 0;
      int softversion = 0;
      int bus = 1;
      int index = 1;
      robot.FT_SetConfig(company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_GetConfig(&company, &device, &softversion, &bus);
      printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_Activate(0);
      robot.Sleep(1000);
      robot.FT_Activate(1);
      robot.Sleep(1000);
      robot.Sleep(1000);
      robot.FT_SetZero(0);
      robot.Sleep(1000);
      uint8_t sensor_id = 1;
      uint8_t select[6] = { 1,1,1,1,1,1 };
      float max_threshold[6] = { 10.0,10.0,10.0,10.0,10.0,10.0 };
      float min_threshold[6] = { 5.0,5.0,5.0,5.0,5.0,5.0 };
      ForceTorque ft;
      DescPose desc_p1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
      DescPose desc_p2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
      DescPose desc_p3(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
      robot.FT_Guard(1, sensor_id, select, &ft, max_threshold, min_threshold);
      robot.MoveCart(&desc_p1, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
      robot.MoveCart(&desc_p2, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
      robot.MoveCart(&desc_p3, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
      robot.FT_Guard(0, sensor_id, select, &ft, max_threshold, min_threshold);
      robot.CloseRPC();
      return 0;
    }

Constant force control
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    *@brief  constant control
    *@param [ in ] flag 0-off constant force control, 1-on constant force control
    *@param [ in ] sensor number
    *@param [ in ] select six degrees of freedom whether to detect collision, 0-no detection, 1-detection
    *@param [ in ] ft impact torque, FX, F Y, F Z, TX, Ty, TZ
    *@param [ in ] ft force PID parameter, moment PID parameter
    *@param [ in ] adj adaptive start-stop control, 0-off, 1-on
    *@param [ in ] ILC start-stop control, 0-stop, 1-training, 2-operation
    *@param [ in ] Max adjusted distance, in mm
    *@param [ in ] max
    *@param [ in ] m quality parameters
    *@param [ in ] b damping parameter
    *@param [ in ] Polish radio radius, mm
    *@param [ in ] filter on flag 0-off, 1-on, off by default
    *@param [ in ] POSADAPT gesture 0-off, 1-on, off by default
    *@param [ in ] is noblock flag, 0-blocking, 1-nonblocking
    *@return  error code
    */
    errno_t FT_Control(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque* ft, float ft_pid[6], uint8_t adj_sign, uint8_t ILC_sign, float max_dis, float max_ang, double M[2], double B[2], double polishRadio = 0.0, int filter_Sign = 0, int posAdapt_sign = 0, int isNoBlock = 0); 

Example of constant force control code with damping
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestFTControlWithDamping(void)
    {
      ROBOT_STATE_PKG pkg = {};
      FRRobot robot;
      robot.LoggerInit();
      robot.SetLoggerLevel(3);
      int rtn = robot.RPC("192.168.58.2");
      if (rtn != 0)
      {
        return -1;
      }
      robot.SetReConnectParam(true, 30000, 500);
      uint8_t sensor_id = 10;
      uint8_t select[6] = { 0,0,1,0,0,0 };
      float ft_pid[6] = { 0.0008, 0.0, 0.0, 0.0, 0.0, 0.0 };
      uint8_t adj_sign = 0;
      uint8_t ILC_sign = 0;
      float max_dis = 100.0;
      float max_ang = 20;
      ForceTorque ft = {};
      ft.fz = -10.0;
      ExaxisPos epos(0, 0, 0, 0);
      JointPos j1(-118.985, -86.882, -118.139, -65.019, 90.002, 54.951);
      JointPos j2(-77.055, -77.218, -126.219, -66.591, 90.028, 96.881);
      DescPose desc_p1(-300.856, -332.618, 309.240, 179.976, -0.031, 96.065);
      DescPose desc_p2(-16.399, -383.760, 309.312, 179.975, -0.031, 96.064);
      DescPose offset_pos(0, 0, 0, 0, 0, 0);
      double M[2] = {2.0, 2.0};
      double B[2] = {8.0, 8.0};
      double polishRadio;
      int filter_Sign;
      int posAdapt_sign;
      int isNoBlock;
      DescPose ftCoord = {};
      robot.FT_SetRCS(2, ftCoord);
      rtn = robot.FT_Control(1, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, 0, 0, 1, 0);
      printf("FT_Control start rtn is %d\n", rtn);
      rtn = robot.MoveL(&j1, &desc_p1, 0, 0, 100.0, 100.0, 20.0, -1.0, &epos, 0, 0, &offset_pos);
      rtn = robot.MoveL(&j2, &desc_p2, 0, 0, 100.0, 100.0, 20.0, -1.0, &epos, 0, 0, &offset_pos);
      rtn = robot.FT_Control(1, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, 0, 0, 1, 0);
      printf("FT_Control end rtn is %d\n", rtn);
      robot.CloseRPC();
      return 0;
    }

Spiral Search
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Spiral search
    * @param  [in] rcs  Reference coordinate system, 0-Tool coordinate system, 1-Base coordinate system
    * @param  [in] dr  Radius feed per circle
    * @param  [in] ft  Force/torque threshold, fx, fy, fz, tx, ty, tz, range [0~100]
    * @param  [in] max_t_ms  Maximum search time in ms
    * @param  [in] max_vel  Maximum linear velocity in mm/s
    * @return  Error code
    */   
    errno_t  FT_SpiralSearch(int rcs, float dr, float ft, float max_t_ms, float max_vel);  

Rotary Insertion
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Rotary insertion
    * @param  [in] rcs  Reference coordinate system, 0-Tool coordinate system, 1-Base coordinate system
    * @param  [in] angVelRot  Rotational angular velocity in deg/s
    * @param  [in] ft  Force/torque threshold, fx, fy, fz, tx, ty, tz, range [0~100]
    * @param  [in] max_angle  Maximum rotation angle in deg
    * @param  [in] orn  Force/torque direction, 1-Along z-axis, 2-Around z-axis
    * @param  [in] max_angAcc  Maximum rotational acceleration in deg/s^2, not currently used, default is 0
    * @param  [in] rotorn  Rotation direction, 1-Clockwise, 2-Counterclockwise
    * @return  Error code
    */   
    errno_t  FT_RotInsertion(int rcs, float angVelRot, float ft, float max_angle, uint8_t orn, float max_angAcc, uint8_t rotorn);    

Linear Insertion
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Linear insertion
    * @param  [in] rcs  Reference coordinate system, 0-Tool coordinate system, 1-Base coordinate system
    * @param  [in] ft  Force/torque threshold, fx, fy, fz, tx, ty, tz, range [0~100]
    * @param  [in] lin_v  Linear velocity in mm/s
    * @param  [in] lin_a  Linear acceleration in mm/s^2, not currently used
    * @param  [in] max_dis  Maximum insertion distance in mm
    * @param  [in] linorn  Insertion direction, 0-Negative direction, 1-Positive direction
    * @return  Error code
    */   
    errno_t  FT_LinInsertion(int rcs, float ft, float lin_v, float lin_a, float max_dis, uint8_t linorn);    

Spiral Search, Linear Insertion and Other Instruction Code Examples
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestFTSearch(void)
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
      int company = 24;
      int device = 0;
      int softversion = 0;
      int bus = 1;
      int index = 1;
      robot.FT_SetConfig(company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_GetConfig(&company, &device, &softversion, &bus);
      printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_Activate(0);
      robot.Sleep(1000);
      robot.FT_Activate(1);
      robot.Sleep(1000);
      robot.Sleep(1000);
      robot.FT_SetZero(0);
      robot.Sleep(1000);
      uint8_t status = 1; 
      int sensor_num = 1; 
      float gain[6] = { 0.0001,0.0,0.0,0.0,0.0,0.0 }; 
      uint8_t adj_sign = 0; 
      uint8_t ILC_sign = 0; 
      float max_dis = 100.0; 
      float max_ang = 5.0; 
      ForceTorque ft;
      memset(&ft, 0, sizeof(ForceTorque));
      int rcs = 0; 
      float dr = 0.7; 
      float fFinish = 1.0; 
      float t = 60000.0;
      float vmax = 3.0;
      float force_goal = 20.0; 
      float lin_v = 0.0;
      float lin_a = 0.0;
      float disMax = 100.0;
      uint8_t linorn = 1; 
      float angVelRot = 2.0; 
      float forceInsertion = 1.0; 
      int angleMax = 45; 
      uint8_t orn = 1;
      float angAccmax = 0.0; 
      uint8_t rotorn = 1; 
      uint8_t select1[6] = { 0,0,1,1,1,0 }; 
      ft.fz = -10.0;
      robot.FT_Control(status, sensor_num, select1, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
      rtn = robot.FT_SpiralSearch(rcs, dr, fFinish, t, vmax);
      printf("FT_SpiralSearch rtn is %d\n", rtn);
      status = 0;
      robot.FT_Control(status, sensor_num, select1, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
      uint8_t select2[6] = { 1,1,1,0,0,0 }; 
      gain[0] = 0.00005;
      ft.fz = -30.0;
      status = 1;
      robot.FT_Control(status, sensor_num, select2, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
      rtn = robot.FT_LinInsertion(rcs, force_goal, lin_v, lin_a, disMax, linorn);
      printf("FT_LinInsertion rtn is %d\n", rtn);
      status = 0;
      robot.FT_Control(status, sensor_num, select2, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
      uint8_t select3[6] = { 0,0,1,1,1,0 }; 
      ft.fz = -10.0;
      gain[0] = 0.0001;
      status = 1;
      robot.FT_Control(status, sensor_num, select3, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
      rtn = robot.FT_RotInsertion(rcs, angVelRot, forceInsertion, angleMax, orn, angAccmax, rotorn);
      printf("FT_RotInsertion rtn is %d\n", rtn);
      status = 0;
      robot.FT_Control(status, sensor_num, select3, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
      uint8_t select4[6] = { 1,1,1,0,0,0 }; 
      ft.fz = -30.0;
      status = 1;
      robot.FT_Control(status, sensor_num, select4, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
      rtn = robot.FT_LinInsertion(rcs, force_goal, lin_v, lin_a, disMax, linorn);
      printf("FT_LinInsertion rtn is %d\n", rtn);
      status = 0;
      robot.FT_Control(status, sensor_num, select4, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
      robot.CloseRPC();
      return 0;
    }

Surface Localization
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Surface localization
    * @param  [in] rcs  Reference coordinate system, 0-Tool coordinate system, 1-Base coordinate system
    * @param  [in] dir  Movement direction, 1-Positive direction, 2-Negative direction 
    * @param  [in] axis  Movement axis, 1-x axis, 2-y axis, 3-z axis
    * @param  [in] lin_v  Search linear velocity in mm/s
    * @param  [in] lin_a  Search linear acceleration in mm/s^2, not currently used, default is 0
    * @param  [in] max_dis  Maximum search distance in mm
    * @param  [in] ft  Action termination force/torque threshold, fx, fy, fz, tx, ty, tz  
    * @return  Error code
    */   
    errno_t  FT_FindSurface(int rcs, uint8_t dir, uint8_t axis, float lin_v, float lin_a, float max_dis, float ft);   

Calculate Middle Plane Position - Start
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Calculate middle plane position - start
    * @return  Error code
    */   
    errno_t  FT_CalCenterStart();

Calculate Middle Plane Position - End
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Calculate middle plane position - end
    * @param  [out] pos  Middle plane pose
    * @return  Error code
    */      
    errno_t  FT_CalCenterEnd(DescPose *pos);

Surface Localization Code Example
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestSurface(void)
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
      int company = 24;
      int device = 0;
      int softversion = 0;
      int bus = 1;
      int index = 1;
      robot.FT_SetConfig(company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_GetConfig(&company, &device, &softversion, &bus);
      printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_Activate(0);
      robot.Sleep(1000);
      robot.FT_Activate(1);
      robot.Sleep(1000);
      robot.Sleep(1000);
      robot.FT_SetZero(0);
      robot.Sleep(1000);
      int rcs = 0;
      uint8_t dir = 1;
      uint8_t axis = 1;
      float lin_v = 3.0;
      float lin_a = 0.0;
      float maxdis = 50.0;
      float ft_goal = 2.0;
      DescPose desc_pos(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
      DescPose xcenter(0, 0, 0, 0, 0, 0);
      DescPose ycenter(0, 0, 0, 0, 0, 0);
      ForceTorque ft;
      memset(&ft, 0, sizeof(ForceTorque));
      ft.fx = -2.0;
      robot.MoveCart(&desc_pos, 9, 0, 100.0, 100.0, 100.0, -1.0, -1);
      robot.FT_CalCenterStart();
      robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
      robot.MoveCart(&desc_pos, 9, 0, 100.0, 100.0, 100.0, -1.0, -1);
      robot.WaitMs(1000);
      dir = 2;
      robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
      robot.FT_CalCenterEnd(&xcenter);
      printf("xcenter:%f,%f,%f,%f,%f,%f\n", xcenter.tran.x, xcenter.tran.y, xcenter.tran.z, xcenter.rpy.rx, xcenter.rpy.ry, xcenter.rpy.rz);
      robot.MoveCart(&xcenter, 9, 0, 60.0, 50.0, 50.0, -1.0, -1);
      robot.FT_CalCenterStart();
      dir = 1;
      axis = 2;
      lin_v = 6.0;
      maxdis = 150.0;
      robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
      robot.MoveCart(&desc_pos, 9, 0, 100.0, 100.0, 100.0, -1.0, -1);
      robot.WaitMs(1000);
      dir = 2;
      robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
      robot.FT_CalCenterEnd(&ycenter);
      printf("ycenter:%f,%f,%f,%f,%f,%f\n", ycenter.tran.x, ycenter.tran.y, ycenter.tran.z, ycenter.rpy.rx, ycenter.rpy.ry, ycenter.rpy.rz);
      robot.MoveCart(&ycenter, 9, 0, 60.0, 50.0, 50.0, 0.0, -1);
      robot.CloseRPC();
      return 0;
    }

Compliance Control Activation
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Compliance control activation
    * @param  [in] p  Position adjustment coefficient or compliance coefficient
    * @param  [in] force  Compliance activation force threshold in N
    * @return  Error code
    */   
    errno_t  FT_ComplianceStart(float p, float force); 

Compliance Control Deactivation
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Compliance control deactivation
    * @return  Error code
    */   
    errno_t  FT_ComplianceStop(); 

Compliance Control Code Example
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestCompliance(void)
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
      int company = 24;
      int device = 0;
      int softversion = 0;
      int bus = 1;
      int index = 1;
      robot.FT_SetConfig(company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_GetConfig(&company, &device, &softversion, &bus);
      printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_Activate(0);
      robot.Sleep(1000);
      robot.FT_Activate(1);
      robot.Sleep(1000);
      robot.Sleep(1000);
      robot.FT_SetZero(0);
      robot.Sleep(1000);
      uint8_t flag = 1;
      int sensor_id = 1;
      uint8_t select[6] = { 1,1,1,0,0,0 };
      float ft_pid[6] = { 0.0005,0.0,0.0,0.0,0.0,0.0 };
      uint8_t adj_sign = 0;
      uint8_t ILC_sign = 0;
      float max_dis = 100.0;
      float max_ang = 0.0;
      ForceTorque ft;
      DescPose offset_pos(0, 0, 0, 0, 0, 0);
      ExaxisPos epos(0, 0, 0, 0);
      memset(&ft, 0, sizeof(ForceTorque));
      JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
      JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
      DescPose desc_p1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
      DescPose desc_p2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
      ft.fx = -10.0;
      ft.fy = -10.0;
      ft.fz = -10.0;
      robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
      float p = 0.00005;
      float force = 30.0;
      rtn = robot.FT_ComplianceStart(p, force);
      printf("FT_ComplianceStart rtn is %d\n", rtn);
      int count = 15;
      while (count)
      {
        robot.MoveL(&j1, &desc_p1, 0, 0, 100.0, 180.0, 100.0, -1.0, &epos, 0, 1, &offset_pos);
        robot.MoveL(&j2, &desc_p2, 0, 0, 100.0, 180.0, 100.0, -1.0, &epos, 0, 0, &offset_pos);
        count -= 1;
      }
      robot.FT_ComplianceStop();
      printf("FT_ComplianceStop rtn is %d\n", rtn);
      flag = 0;
      robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
      robot.CloseRPC();
      return 0;
    }

Payload Identification Initialization
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Payload identification initialization
    * @return Error code
    */
    errno_t LoadIdentifyDynFilterInit();

Payload Identification Initialization
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Payload identification initialization
    * @return Error code
    */
    errno_t LoadIdentifyDynVarInit();

Payload Identification Main Program
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Payload identification main program
    * @param [in] joint_torque  Joint torque
    * @param [in] joint_pos  Joint position
    * @param [in] t  Sampling period
    * @return Error code
    */
    errno_t LoadIdentifyMain(double joint_torque[6], double joint_pos[6], double t);

Get Payload Identification Result
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Get payload identification result
    * @param [in] gain 
    * @param [out] weight  Payload weight
    * @param [out] cog  Payload COG
    * @return Error code
    */
    errno_t LoadIdentifyGetResult(double gain[12], double *weight, DescTran *cog);

Robot Payload Identification Code Example
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestIdentify(void)
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
      retval = robot.LoadIdentifyDynFilterInit();
      printf("LoadIdentifyDynFilterInit retval is: %d \n", retval);
      retval = robot.LoadIdentifyDynVarInit();
      printf("LoadIdentifyDynVarInit retval is: %d \n", retval);
      JointPos posJ = {};
      DescPose posDec = {};
      float joint_toq[6] = { 0.0 };
      robot.GetActualJointPosDegree(0, &posJ);
      posJ.jPos[1] = posJ.jPos[1] + 10;
      robot.GetJointTorques(0, joint_toq);
      joint_toq[1] = joint_toq[1] + 2;
      double tmpTorque[6] = { 0.0 };
      for (int i = 0; i < 6; i++)
      {
        tmpTorque[i] = joint_toq[i];
      }
      retval = robot.LoadIdentifyMain(tmpTorque, posJ.jPos, 1);
      printf("LoadIdentifyMain retval is: %d \n", retval);
      double gain[12] = { 0,0.05,0,0,0,0,0,0.02,0,0,0,0 };
      double weight = 0;
      DescTran load_pos;
      memset(&load_pos, 0, sizeof(DescTran));
      retval = robot.LoadIdentifyGetResult(gain, &weight, &load_pos);
      printf("LoadIdentifyGetResult retval is: %d ; weight is %f cog is %f %f %f \n", retval, weight, load_pos.x, load_pos.y, load_pos.z);
      robot.CloseRPC();
      return 0;
    }

Force Sensor Assisted Drag
+++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.2.0-3.8.0
    
.. code-block:: c++
    :linenos:

    /**
	  * @brief  Force sensor assisted drag
	  * @param  [in] status  Control status, 0-Off; 1-On
	  * @param  [in] asaptiveFlag  Adaptive activation flag, 0-Off; 1-On
	  * @param  [in] interfereDragFlag  Interference area drag flag, 0-Off; 1-On
	  * @param  [in] ingularityConstraintsFlag  Singularity strategy, 0-Avoid; 1-Cross
	  * @param  [in] forceCollisionFlag Robot collision detection sign for auxiliary dragging; 0-off; 1-on
	  * @param  [in] M  Inertia coefficient
	  * @param  [in] B  Damping coefficient
	  * @param  [in] K  Stiffness coefficient
	  * @param  [in] F  Drag six-dimensional force threshold
	  * @param  [in] Fmax  Maximum drag force limit
	  * @param  [in] Vmax  Maximum joint velocity limit
      * @return  error code
      */
      errno_t EndForceDragControl(int status, int asaptiveFlag, int interfereDragFlag, int ingularityConstraintsFlag, int forceCollisionFlag, std::vector<double> M, std::vector<double> B, std::vector<double> K, std::vector<double> F, double Fmax, double Vmax);

Get Force Sensor Drag Switch Status
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Get force sensor drag switch status
     * @param [out] dragState  Force sensor assisted drag control status, 0-Off; 1-On
     * @param [out] sixDimensionalDragState  Six-dimensional force assisted drag control status, 0-Off; 1-On
     * @return Error code
     */
    errno_t GetForceAndTorqueDragState(int& dragState, int& sixDimensionalDragState);

Force Sensor Auto Activation After Error Clear
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Auto activate force sensor after error clear
     * @param [in] status  Control status, 0-Off; 1-On
     * @return Error code
     */
    errno_t SetForceSensorDragAutoFlag(int status);

Force Sensor Assisted Drag Code Example
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestEndForceDragCtrl(void)
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
      robot.SetForceSensorDragAutoFlag(1);
      vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
      vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
      vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
      vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
      robot.EndForceDragControl(1, 0, 0, 0, 1, M, B, K, F, 50, 100);
      robot.Sleep(5000);
      int dragState = 0;
      int sixDimensionalDragState = 0;
      robot.GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
      printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);
      robot.EndForceDragControl(0, 0, 0, 0, 1, M, B, K, F, 50, 100);
      robot.CloseRPC();
      return 0;
    }

Set Six-Dimensional Force and Joint Impedance Hybrid Drag Switch and Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Set six-dimensional force and joint impedance hybrid drag switch and parameters
     * @param [in] status  Control status, 0-Off; 1-On
     * @param [in] impedanceFlag  Impedance activation flag, 0-Off; 1-On
     * @param [in] lamdeDain  Drag gain
     * @param [in] KGain  Stiffness gain
     * @param [in] BGain  Damping gain
     * @param [in] dragMaxTcpVel  Maximum end linear velocity limit for drag
     * @param [in] dragMaxTcpOriVel  Maximum end angular velocity limit for drag
     * @return Error code
     */
    errno_t ForceAndJointImpedanceStartStop(int status, int impedanceFlag, std::vector<double> lamdeDain, std::vector<double> KGain, std::vector<double> BGain, double dragMaxTcpVel, double dragMaxTcpOriVel);

Six-Dimensional Force and Joint Impedance Hybrid Drag Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int TestForceAndJointImpedance(void)
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
      vector <double> lamdeDain = { 3.0, 2.0, 2.0, 2.0, 2.0, 3.0 };
      vector <double> KGain = { 0, 0, 0, 0, 0, 0 };
      vector <double> BGain = { 150, 150, 150, 5.0, 5.0, 1.0 };
      rtn = robot.ForceAndJointImpedanceStartStop(1, 0, lamdeDain, KGain, BGain, 1000, 180);
      printf("ForceAndJointImpedanceStartStop rtn is %d\n", rtn);
      robot.Sleep(5000);
      robot.DragTeachSwitch(0);
      rtn = robot.ForceAndJointImpedanceStartStop(0, 0, lamdeDain, KGain, BGain, 1000, 180);
      printf("ForceAndJointImpedanceStartStop rtn is %d\n", rtn);
      robot.CloseRPC();
      return 0;
    }

Impedance Start/Stop Control
++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Impedance Start/Stop Control
    * @param [in] status 0: Stop; 1- Start
    * @param [in] workSpace 0- Joint Space; 1- Cartesian Space
    * @param [in] forceThreshold Trigger force threshold (N)
    * @param [in] m Mass parameter
    * @param [in] b Damping parameter
    * @param [in] k Stiffness parameter
    * @param [in] maxV Maximum linear velocity (mm/s)
    * @param [in] maxVA Maximum linear acceleration (mm/s²)
    * @param [in] maxW Maximum angular velocity (°/s)
    * @param [in] maxWA Maximum angular acceleration (°/s²)
    * @return Error code
    */
    errno_t ImpedanceControlStartStop(int status, int workSpace, double forceThreshold[6], double m[6], double b[6], double k[6], double maxV, double maxVA, double maxW, double maxWA);

Robot Impedance Start/Stop Control Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    int TestImpedanceControl()
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
        return 0;
      }
      robot.SetReConnectParam(true, 30000, 500);
      JointPos j1(102.622, -135.990, 120.769, -73.950, -90.848, 35.507);
      JointPos j2(93.674, -80.062, 82.947, -92.199, -90.967, 26.559);
      DescPose desc_pos1(136.552, -149.799, 449.532, 179.817, -1.172, 157.123);
      DescPose desc_pos2(136.540, -561.048, 449.542, 179.819, -1.172, 157.122);
      DescPose offset_pos(0, 0, 0, 0, 0, 0);
      ExaxisPos epos(0, 0, 0, 0);
      int tool = 0;
      int user = 0;
      float vel = 100.0;
      float acc = 200.0;
      float ovl = 100.0;
      float blendT = -1.0;
      float blendR = -1.0;
      uint8_t flag = 0;
      uint8_t search = 0;
      robot.SetSpeed(20);
      int company = 17;
      int device = 0;
      int softversion = 0;
      int bus = 1;
      robot.FT_SetConfig(company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_GetConfig(&company, &device, &softversion, &bus);
      printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
      robot.Sleep(1000);
      robot.FT_Activate(0);
      robot.Sleep(1000);
      robot.FT_Activate(1);
      robot.Sleep(1000);
      robot.Sleep(1000);
      robot.FT_SetZero(0);
      robot.Sleep(1000);
      robot.FT_SetZero(1);
      robot.Sleep(1000);
      double forceThreshold[6] = { 30,30,30,5,5,5 };
      double m[6] = { 0.1,0.1,0.1,0.02,0.02,0.02 };
      double b[6] = { 1,1,1,0.08,0.08,0.08 };
      double k[6] = { 0,0,0,0,0,0 };
      rtn = robot.ImpedanceControlStartStop(1, 1, forceThreshold, m, b, k, 1000, 500, 100, 100);
      printf("ImpedanceControlStartStop errcode:%d\n", rtn);
      rtn = robot.MoveL(&desc_pos1, tool, user, vel, acc, ovl, blendR, 0, &epos, search, flag, &offset_pos, -1, 1);
      rtn = robot.MoveL(&desc_pos2, tool, user, vel, acc, ovl, blendR, 0, &epos, search, flag, &offset_pos, -1, 1);
      rtn = robot.MoveL(&desc_pos1, tool, user, vel, acc, ovl, blendR, 0, &epos, search, flag, &offset_pos, -1, 1);
      rtn = robot.MoveL(&desc_pos2, tool, user, vel, acc, ovl, blendR, 0, &epos, search, flag, &offset_pos, -1, 1);
      printf("movel errcode:%d\n", rtn);
      robot.ImpedanceControlStartStop(0, 1, forceThreshold, m, b, k, 1000, 500, 100, 100);    
      robot.CloseRPC();
      return 0;
    }
            