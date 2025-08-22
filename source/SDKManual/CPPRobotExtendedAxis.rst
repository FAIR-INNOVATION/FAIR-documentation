Extended Axis
=============

.. toctree:: 
    :maxdepth: 5

Set 485 Extended Axis Parameters
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set 485 extended axis parameters
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @param [in] servoCompany Servo driver manufacturer, 1-Dynatek
    * @param [in] servoModel Servo driver model, 1-FD100-750C
    * @param [in] servoSoftVersion Servo driver software version, 1-V1.0
    * @param [in] servoResolution Encoder resolution
    * @param [in] axisMechTransRatio Mechanical transmission ratio
    * @return Error code
    */
    errno_t AuxServoSetParam(int servoId, int servoCompany, int servoModel, int servoSoftVersion, int servoResolution, double axisMechTransRatio);

Get 485 Extended Axis Configuration Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get 485 extended axis configuration parameters
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @param [out] servoCompany Servo driver manufacturer, 1-Dynatek
    * @param [out] servoModel Servo driver model, 1-FD100-750C
    * @param [out] servoSoftVersion Servo driver software version, 1-V1.0
    * @param [out] servoResolution Encoder resolution
    * @param [out] axisMechTransRatio Mechanical transmission ratio
    * @return Error code
    */
    errno_t AuxServoGetParam(int servoId, int* servoCompany, int* servoModel, int* servoSoftVersion, int* servoResolution, double* axisMechTransRatio);

Set 485 Extended Axis Enable/Disable
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set 485 extended axis enable/disable
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @param [in] status Enable status, 0-disable, 1-enable
    * @return Error code
    */
    errno_t AuxServoEnable(int servoId, int status);

Set 485 Extended Axis Control Mode
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set 485 extended axis control mode
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @param [in] mode Control mode, 0-position mode, 1-velocity mode
    * @return Error code
    */
    errno_t AuxServoSetControlMode(int servoId, int mode);

Set 485 Extended Axis Target Position (Position Mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set 485 extended axis target position (position mode)
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @param [in] pos Target position, mm or °
    * @param [in] speed Target speed, mm/s or °/s
    * @return Error code
    */
    errno_t AuxServoSetTargetPos(int servoId, double pos, double speed);

Set 485 Extended Axis Target Torque (Torque Mode) - Temporarily Unavailable
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set 485 extended axis target torque (torque mode)
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @param [in] torque Target torque, Nm
    * @return Error code
    */
    errno_t AuxServoSetTargetTorque(int servoId, double torque);

Set 485 Extended Axis Homing
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set 485 extended axis homing
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @param [in] mode Homing mode, 0-current position homing; 1-limit homing
    * @param [in] searchVel Homing speed, mm/s or °/s
    * @param [in] latchVel Latch speed, mm/s or °/s
    * @return Error code
    */
    errno_t AuxServoHoming(int servoId, int mode, double searchVel, double latchVel);

Clear 485 Extended Axis Error Information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Clear 485 extended axis error information
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @return Error code
    */
    errno_t AuxServoClearError(int servoId);

Get 485 Extended Axis Servo Status
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get 485 extended axis servo status
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @param [out] servoErrCode Servo driver error code
    * @param [out] servoState Servo driver state [Decimal converted to binary, bit0-bit5: Servo enable-Servo running-Positive limit triggered-Negative limit triggered-Positioning complete-Homing complete]
    * @param [out] servoPos Current servo position, mm or °
    * @param [out] servoSpeed Current servo speed, mm/s or °/s
    * @param [out] servoTorque Current servo torque, Nm
    * @return Error code
    */
    errno_t AuxServoGetStatus(int servoId, int* servoErrCode, int* servoState, double* servoPos, double* servoSpeed, double* servoTorque);

Set 485 Extended Axis Target Speed (Velocity Mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set 485 extended axis target speed (velocity mode)
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @param [in] speed Target speed, mm/s or °/s
    * @return Error code
    */
    errno_t AuxServoSetTargetSpeed(int servoId, double speed);

Set Status Feedback 485 Extended Axis Data Axis Number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set status feedback 485 extended axis data axis number
    * @param [in] servoId Servo driver ID, range [1-15], corresponding to slave ID
    * @return Error code
    */
    errno_t AuxServosetStatusID(int servoId);

Set 485 Extended Axis Motion Acceleration/Deceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set 485 extended axis motion acceleration/deceleration
    * @param [in] acc 485 extended axis motion acceleration
    * @param [in] dec 485 extended axis motion deceleration
    * @return Error code
    */
    errno_t AuxServoSetAcc(double acc, double dec);

Set 485 Extended Axis Emergency Stop Acceleration/Deceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set 485 extended axis emergency stop acceleration/deceleration
    * @param [in] acc 485 extended axis emergency stop acceleration
    * @param [in] dec 485 extended axis emergency stop deceleration
    * @return Error code
    */
    errno_t AuxServoSetEmergencyStopAcc(double acc, double dec);

Get 485 Extended Axis Motion Acceleration/Deceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get 485 extended axis motion acceleration/deceleration
    * @param [out] acc 485 extended axis motion acceleration
    * @param [out] dec 485 extended axis motion deceleration
    * @return Error code
    */
    errno_t AuxServoGetAcc(double& acc, double& dec);

Get 485 Extended Axis Emergency Stop Acceleration/Deceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get 485 extended axis emergency stop acceleration/deceleration
    * @param [out] acc 485 extended axis emergency stop acceleration
    * @param [out] dec 485 extended axis emergency stop deceleration
    * @return Error code
    */
    errno_t AuxServoGetEmergencyStopAcc(double& acc, double& dec);

Extended Axis Control Code Example
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:
        
    int Test485Auxservo(void)
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
      int retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 15.45);
      std::cout << "AuxServoSetParam is: " << retval << std::endl;
      int servoCompany;
      int servoModel;
      int servoSoftVersion;
      int servoResolution;
      double axisMechTransRatio;
      retval = robot.AuxServoGetParam(1, &servoCompany, &servoModel, &servoSoftVersion, &servoResolution, &axisMechTransRatio);
      std::cout << "servoCompany " << servoCompany << "\n"
        << "servoModel " << servoModel << "\n"
        << "servoSoftVersion " << servoSoftVersion << "\n"
        << "servoResolution " << servoResolution << "\n"
        << "axisMechTransRatio " << axisMechTransRatio << "\n"
        << std::endl;
      retval = robot.AuxServoSetParam(1, 10, 11, 12, 13, 14);
      std::cout << "AuxServoSetParam is: " << retval << std::endl;
      retval = robot.AuxServoGetParam(1, &servoCompany, &servoModel, &servoSoftVersion, &servoResolution, &axisMechTransRatio);
      std::cout << "servoCompany " << servoCompany << "\n"
        << "servoModel " << servoModel << "\n"
        << "servoSoftVersion " << servoSoftVersion << "\n"
        << "servoResolution " << servoResolution << "\n"
        << "axisMechTransRatio " << axisMechTransRatio << "\n"
        << std::endl;
      retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 36);
      std::cout << "AuxServoSetParam is: " << retval << std::endl;
      robot.Sleep(3000);
      robot.AuxServoSetAcc(3000, 3000);
      robot.AuxServoSetEmergencyStopAcc(5000, 5000);
      robot.Sleep(1000);
      double emagacc = 0, acc = 0;
      double emagdec = 0, dec = 0;
      robot.AuxServoGetEmergencyStopAcc(emagacc, emagdec);
      printf("emergency acc is %f dec is %f \n", emagacc, emagdec);
      robot.AuxServoGetAcc(acc, dec);
      printf("acc is %f dec is %f \n", acc, dec);
      robot.AuxServoSetControlMode(1, 0);
      robot.Sleep(2000);
      retval = robot.AuxServoEnable(1, 0);
      std::cout << "AuxServoEnable disenable " << retval << std::endl;
      robot.Sleep(1000);
      int servoerrcode = 0;
      int servoErrCode;
      int servoState;
      double servoPos;
      double servoSpeed;
      double servoTorque;
      retval = robot.AuxServoGetStatus(1, &servoErrCode, &servoState, &servoPos, &servoSpeed, &servoTorque);
      std::cout << "AuxServoGetStatus servoState " << servoState << std::endl;
      robot.Sleep(1000);;
      retval = robot.AuxServoEnable(1, 1);
      std::cout << "AuxServoEnable enable " << retval << std::endl;
      robot.Sleep(1000);
      retval = robot.AuxServoGetStatus(1, &servoErrCode, &servoState, &servoPos, &servoSpeed, &servoTorque);
      std::cout << "AuxServoGetStatus servoState " << servoState << std::endl;
      robot.Sleep(1000);
      retval = robot.AuxServoHoming(1, 1, 5, 1);
      std::cout << "AuxServoHoming " << retval << std::endl;
      robot.Sleep(3000);
      retval = robot.AuxServoSetTargetPos(1, 200, 30);
      std::cout << "AuxServoSetTargetPos " << retval << std::endl;
      robot.Sleep(1000);
      retval = robot.AuxServoGetStatus(1, &servoErrCode, &servoState, &servoPos, &servoSpeed, &servoTorque);
      std::cout << "AuxServoGetStatus servoSpeed " << servoSpeed << std::endl;
      robot.Sleep(8000);
      robot.AuxServoSetControlMode(1, 1);
      robot.Sleep(2000);
      robot.AuxServoEnable(1, 0);
      robot.Sleep(1000);
      robot.AuxServoEnable(1, 1);
      robot.Sleep(1000);
      robot.AuxServoSetTargetSpeed(1, 100, 80);
      robot.Sleep(5000);
      robot.AuxServoSetTargetSpeed(1, 0, 80);
      robot.CloseRPC();
      return 0;
    }

UDP Extended Axis Communication Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP extended axis communication parameter configuration
    * @param [in] ip PLC IP address
    * @param [in] port  Port number
    * @param [in] period    Communication cycle (ms, default is 2, do not modify this parameter)
    * @param [in] lossPkgTime   Packet loss detection time (ms)
    * @param [in] lossPkgNum    Packet loss count
    * @param [in] disconnectTime    Communication disconnection confirmation duration
    * @param [in] reconnectEnable   Communication disconnection auto-reconnect enable, 0-disable, 1-enable
    * @param [in] reconnectPeriod   Reconnect cycle interval (ms)
    * @param [in] reconnectNum  Reconnect count
    * @param [in] selfConnect Auto-establish connection after power restart; 0-do not establish; 1-establish
    * @return Error code
    */
    errno_t ExtDevSetUDPComParam(std::string ip, int port, int period, int lossPkgTime, int lossPkgNum, int disconnectTime, int reconnectEnable, int reconnectPeriod, int reconnectNum, int selfConnect = 1);
        
Get UDP Extended Axis Communication Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get UDP extended axis communication parameters
    * @param [out] ip PLC IP address
    * @param [out] port Port number
    * @param [out] period Communication cycle (ms, default is 2, do not modify this parameter)
    * @param [out] lossPkgTime Packet loss detection time (ms)
    * @param [out] lossPkgNum Packet loss count
    * @param [out] disconnectTime Communication disconnection confirmation duration
    * @param [out] reconnectEnable Communication disconnection auto-reconnect enable, 0-disable, 1-enable
    * @param [out] reconnectPeriod Reconnect cycle interval (ms)
    * @param [out] reconnectNum Reconnect count
    * @return Error code
    */
    errno_t ExtDevGetUDPComParam(std::string& ip, int& port, int& period, int& lossPkgTime, int& lossPkgNum, int& disconnectTime, int& reconnectEnable, int& reconnectPeriod, int& reconnectNum);
        
Load UDP Communication
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Load UDP communication
    * @return Error code
    */
    errno_t ExtDevLoadUDPDriver();

Unload UDP Communication
++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Unload UDP communication
    * @return Error code
    */
    errno_t ExtDevUnloadUDPDriver();

UDP Extended Axis Communication Exception Disconnection Recovery
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP extended axis communication exception disconnection recovery
    * @return Error code
    */
    errno_t ExtDevUDPClientComReset();

UDP Extended Axis Communication Exception Disconnection Close Communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP extended axis communication exception disconnection close communication
    * @return Error code
    */
    errno_t ExtDevUDPClientComClose();

UDP Extended Axis Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP extended axis parameter configuration
    * @param [in] axisID Axis number
    * @param [in] axisType Extended axis type, 0-translation; 1-rotation
    * @param [in] axisDirection Extended axis direction, 0-forward; 1-reverse 
    * @param [in] axisMax Extended axis maximum position, mm
    * @param [in] axisMin Extended axis minimum position, mm
    * @param [in] axisVel Speed, mm/s
    * @param [in] axisAcc Acceleration, mm/s²
    * @param [in] axisLead Lead, mm
    * @param [in] encResolution Encoder resolution
    * @param [in] axisOffect Weld starting point extended axis offset
    * @param [in] axisCompany Driver manufacturer, 1-Hiwin; 2-Inovance; 3-Panasonic
    * @param [in] axisModel Driver model, 1-Hiwin-SV-XD3EA040L-E, 2-Hiwin-SV-X2EA150A-A, 1-Inovance-SV620PT5R4I, 1-Panasonic-MADLN15SG, 2-Panasonic-MSDLN25SG, 3-Panasonic-MCDLN35SG
    * @param [in] axisEncType Encoder type, 0-incremental; 1-absolute
    * @return Error code
    */
    errno_t ExtAxisParamConfig(int axisID, int axisType, int axisDirection, double axisMax, double axisMin, double axisVel, double axisAcc, double axisLead, long encResolution, double axisOffect, int axisCompany, int axisModel, int axisEncType);

Set Extended Axis Installation Position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set extended axis installation position
    * @param [in] installType 0-Robot installed on external axis, 1-Robot installed outside external axis
    * @return Error code
    */
    errno_t SetRobotPosToAxis(int installType);

Set Extended Axis System DH Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set extended axis system DH parameter configuration
    * @param [in] axisConfig External axis configuration, 0-single degree of freedom linear slide, 1-two degree of freedom L-type positioner, 2-three degree of freedom, 3-four degree of freedom, 4-single degree of freedom positioner
    * @param [in] axisDHd1 External axis DH parameter d1, mm
    * @param [in] axisDHd2 External axis DH parameter d2, mm
    * @param [in] axisDHd3 External axis DH parameter d3, mm
    * @param [in] axisDHd4 External axis DH parameter d4, mm
    * @param [in] axisDHa1 External axis DH parameter a1, mm
    * @param [in] axisDHa2 External axis DH parameter a2, mm
    * @param [in] axisDHa3 External axis DH parameter a3, mm
    * @param [in] axisDHa4 External axis DH parameter a4, mm
    * @return Error code
    */
    errno_t SetAxisDHParaConfig(int axisConfig, double axisDHd1, double axisDHd2, double axisDHd3, double axisDHd4, double axisDHa1, double axisDHa2, double axisDHa3, double axisDHa4);

UDP Extended Axis Enable
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP extended axis enable
    * @param [in] axisID Axis number [1-4]
    * @param [in] status 0-disable; 1-enable
    * @return Error code
    */
    errno_t ExtAxisServoOn(int axisID, int status);

UDP Extended Axis Homing
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP extended axis homing
    * @param [in] axisID Axis number [1-4]
    * @param [in] mode Homing method, 0-current position homing, 1-negative limit homing, 2-positive limit homing
    * @param [in] searchVel Homing speed (mm/s)
    * @param [in] latchVel Homing latch speed (mm/s)
    * @return Error code
    */
    errno_t ExtAxisSetHoming(int axisID, int mode, double searchVel, double latchVel);

UDP Extended Axis Jog Start
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP extended axis jog start
    * @param [in] axisID Axis number [1-4]
    * @param [in] direction Rotation direction, 0-reverse; 1-forward
    * @param [in] vel Speed (mm/s)
    * @param [in] acc Acceleration (mm/s²)
    * @param [in] maxDistance Maximum jog distance
    * @return Error code
    */
    errno_t ExtAxisStartJog(int axisID, int direction, double vel, double acc, double maxDistance);
    
UDP Extended Axis Jog Stop
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP extended axis jog stop
    * @param [in] axisID Axis number [1-4]
    * @return Error code
    */
    errno_t ExtAxisStopJog(int axisID);

UDP Extended Axis Configuration and Jog Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestUDPAxis(void)
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
      rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1);
      cout << "ExtDevSetUDPComParam rtn is " << rtn << endl;
      string ip = ""; int port = 0; int period = 0; int lossPkgTime = 0; int lossPkgNum = 0; int disconnectTime = 0; int reconnectEnable = 0; int reconnectPeriod = 0; int reconnectNum = 0;
      rtn = robot.ExtDevGetUDPComParam(ip, port, period, lossPkgTime, lossPkgNum, disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum);
      string patam = "\nip " + ip + "\nport " + to_string(port) + "\nperiod " + to_string(period) + "\nlossPkgTime " + to_string(lossPkgTime) + "\nlossPkgNum " + to_string(lossPkgNum) + "\ndisConntime " + to_string(disconnectTime) + "\nreconnecable " + to_string(reconnectEnable) + "\nreconnperiod " + to_string(reconnectPeriod) + "\nreconnnun " + to_string(reconnectNum);
      cout << "ExtDevGetUDPComParam rtn is " << rtn << patam << endl;
      robot.ExtDevLoadUDPDriver();
      rtn = robot.ExtAxisServoOn(1, 1);
      cout << "ExtAxisServoOn axis id 1 rtn is " << rtn << endl;
      rtn = robot.ExtAxisServoOn(2, 1);
      cout << "ExtAxisServoOn axis id 2 rtn is " << rtn << endl;
      robot.Sleep(2000);
      robot.ExtAxisSetHoming(1, 0, 10, 2);
      robot.Sleep(2000);
      rtn = robot.ExtAxisSetHoming(2, 0, 10, 2);
      cout << "ExtAxisSetHoming rtnn is " << rtn << endl;
      robot.Sleep(4000);
      rtn = robot.SetRobotPosToAxis(1);
      cout << "SetRobotPosToAxis rtn is " << rtn << endl;
      rtn = robot.SetAxisDHParaConfig(10, 20, 0, 0, 0, 0, 0, 0, 0);
      cout << "SetAxisDHParaConfig rtn is " << rtn << endl;
      rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0);
      cout << "ExtAxisParamConfig axis 1 rtn is " << rtn << endl;
      rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0);
      cout << "ExtAxisParamConfig axis 1 rtn is " << rtn << endl;
      robot.Sleep(1000 * 3);
      robot.ExtAxisStartJog(1, 0, 10, 10, 30);
      robot.Sleep(1000 * 1);
      robot.ExtAxisStopJog(1);
      robot.Sleep(1000 * 3);
      robot.ExtAxisServoOn(1, 0);
      robot.Sleep(1000 * 3);
      robot.ExtAxisStartJog(2, 0, 10, 10, 30);
      robot.Sleep(1000 * 1);
      robot.ExtAxisStopJog(2);
      robot.Sleep(1000 * 3);
      robot.ExtAxisServoOn(2, 0);
      robot.ExtDevUnloadUDPDriver();
      robot.CloseRPC();
      return 0;
    }

Set Extended Axis Coordinate System Reference Point - Four-Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set extended axis coordinate system reference point - four-point method
    * @param [in] pointNum Point number [1-4]
    * @return Error code
    */
    errno_t ExtAxisSetRefPoint(int pointNum);

Calculate Extended Axis Coordinate System - Four-Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Calculate extended axis coordinate system - four-point method
    * @param [out] coord Coordinate system value
    * @return Error code
    */
    errno_t ExtAxisComputeECoordSys(DescPose& coord);

Positioner Coordinate System Reference Point Setting
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Positioner coordinate system reference point setting
    * @param [in] pointNum Point number [1-4]
    * @return Error code
    */
    errno_t PositionorSetRefPoint(int pointNum);

Positioner Coordinate System Calculation - Four-Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Positioner coordinate system calculation - four-point method
    * @param [out] coord Coordinate system value
    * @return Error code
    */
    errno_t PositionorComputeECoordSys(DescPose& coord);

Set Calibration Reference Point Pose in Extended Axis End Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set calibration reference point pose in extended axis end coordinate system
    * @param [in] pos Pose value
    * @return Error code
    */
    errno_t SetRefPointInExAxisEnd(DescPose pos);

Apply Extended Axis Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Apply extended axis coordinate system
    * @param [in] applyAxisId Extended axis number, bit0-bit3 corresponds to extended axis number 1-4, e.g., to apply extended axis 1 and 3, use 0b 0000 0101; which is 5
    * @param [in] axisCoordNum Extended axis coordinate system number
    * @param [in] coord Coordinate system value
    * @param [in] calibFlag Calibration flag, 0-no, 1-yes
    * @return Error code
    */
    errno_t ExtAxisActiveECoordSys(int applyAxisId, int axisCoordNum, DescPose coord, int calibFlag);

Get Extended Axis Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Get extended axis coordinate system
    * @param [out] coord Extended axis coordinate system
    * @return Error code
    */
    errno_t ExtAxisGetCoord(DescPose& coord);

Extended Axis Coordinate System Calibration Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestUDPAxisCalib(void)
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
       rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1);
       cout << "ExtDevSetUDPComParam rtn is " << rtn << endl;
       string ip = ""; int port = 0; int period = 0; int lossPkgTime = 0; int lossPkgNum = 0; int disconnectTime = 0; int reconnectEnable = 0; int reconnectPeriod = 0; int reconnectNum = 0;
       rtn = robot.ExtDevGetUDPComParam(ip, port, period, lossPkgTime, lossPkgNum, disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum);
       string patam = "\nip " + ip + "\nport " + to_string(port) + "\nperiod " + to_string(period) + "\nlossPkgTime " + to_string(lossPkgTime) + "\nlossPkgNum " + to_string(lossPkgNum) + "\ndisConntime " + to_string(disconnectTime) + "\nreconnecable " + to_string(reconnectEnable) + "\nreconnperiod " + to_string(reconnectPeriod) + "\nreconnnun " + to_string(reconnectNum);
       cout << "ExtDevGetUDPComParam rtn is " << rtn << patam << endl;
       robot.ExtDevLoadUDPDriver();
       rtn = robot.ExtAxisServoOn(1, 1);
       cout << "ExtAxisServoOn axis id 1 rtn is " << rtn << endl;
       rtn = robot.ExtAxisServoOn(2, 1);
       cout << "ExtAxisServoOn axis id 2 rtn is " << rtn << endl;
       robot.Sleep(2000);
       robot.ExtAxisSetHoming(1, 0, 10, 2);
       robot.Sleep(2000);
       rtn = robot.ExtAxisSetHoming(2, 0, 10, 2);
       cout << "ExtAxisSetHoming rtnn is " << rtn << endl;
       robot.Sleep(4000);
       rtn = robot.SetRobotPosToAxis(1);
       cout << "SetRobotPosToAxis rtn is " << rtn << endl;
       rtn = robot.SetAxisDHParaConfig(1, 128.5, 206.4, 0, 0, 0, 0, 0, 0);
       cout << "SetAxisDHParaConfig rtn is " << rtn << endl;
       rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0);
       cout << "ExtAxisParamConfig axis 1 rtn is " << rtn << endl;
       rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0);
       cout << "ExtAxisParamConfig axis 1 rtn is " << rtn << endl;
       DescPose toolCoord(0, 0, 210, 0, 0, 0);
       robot.SetToolCoord(1, &toolCoord, 0, 0, 1, 0);
       JointPos jSafe(115.193, -96.149, 92.489, -87.068, -89.15, -83.488);
       JointPos j1(117.559, -92.624, 100.329, -96.909, -94.057, -83.488);
       JointPos j2(112.239, -90.096, 99.282, -95.909, -89.824, -83.488);
       JointPos j3(110.839, -83.473, 93.166, -89.22, -90.499, -83.487);
       JointPos j4(107.935, -83.572, 95.424, -92.873, -87.933, -83.488);
       DescPose descSafe = {};
       DescPose desc1 = {};
       DescPose desc2 = {};
       DescPose desc3 = {};
       DescPose desc4 = {};
       ExaxisPos exaxisPos = { 0, 0, 0, 0 };
       DescPose offdese = { 0, 0, 0, 0, 0, 0 };
       robot.GetForwardKin(&jSafe, &descSafe);
       robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
       robot.Sleep(2000);
       robot.GetForwardKin(&j1, &desc1);
       robot.MoveJ(&j1, &desc1, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
       robot.Sleep(2000);
       DescPose actualTCPPos = {};
       robot.GetActualTCPPose(0, &actualTCPPos);
       robot.SetRefPointInExAxisEnd(actualTCPPos);
       rtn = robot.PositionorSetRefPoint(1);
       cout << "PositionorSetRefPoint 1 rtn is " << rtn << endl;
       robot.Sleep(2000);
       robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
       robot.ExtAxisStartJog(1, 0, 50, 50, 10);
       robot.Sleep(1000);
       robot.ExtAxisStartJog(2, 0, 50, 50, 10);
       robot.Sleep(1000);
       robot.GetForwardKin(&j2, &desc2);
       rtn = robot.MoveJ(&j2, &desc2, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
       rtn = robot.PositionorSetRefPoint(2);
       cout << "PositionorSetRefPoint 2 rtn is " << rtn << endl;
       robot.Sleep(2000);
       robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
       robot.ExtAxisStartJog(1, 0, 50, 50, 10);
       robot.Sleep(1000);
       robot.ExtAxisStartJog(2, 0, 50, 50, 10);
       robot.Sleep(1000);
       robot.GetForwardKin(&j3, &desc3);
       robot.MoveJ(&j3, &desc3, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
       rtn = robot.PositionorSetRefPoint(3);
       cout << "PositionorSetRefPoint 3 rtn is " << rtn << endl;
       robot.Sleep(2000);
       robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
       robot.ExtAxisStartJog(1, 0, 50, 50, 10);
       robot.Sleep(1000);
       robot.ExtAxisStartJog(2, 0, 50, 50, 10);
       robot.Sleep(1000);
       robot.GetForwardKin(&j4, &desc4);
       robot.MoveJ(&j4, &desc4, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
       rtn = robot.PositionorSetRefPoint(4);
       cout << "PositionorSetRefPoint 4 rtn is " << rtn << endl;
       robot.Sleep(2000);
       DescPose axisCoord = {};
       robot.PositionorComputeECoordSys(axisCoord);
       robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
       printf("PositionorComputeECoordSys rtn is %f %f %f %f %f %f\n", axisCoord.tran.x, axisCoord.tran.y, axisCoord.tran.z, axisCoord.rpy.rx, axisCoord.rpy.ry, axisCoord.rpy.rz);
       rtn = robot.ExtAxisActiveECoordSys(3, 1, axisCoord, 1);
       cout << "ExtAxisActiveECoordSys rtn is " << rtn << endl;
    DescPose getCoord(0, 0, 0, 0, 0, 0);
    rtn = robot.ExtAxisGetCoord(getCoord);
    printf("ExtAxisGetCoord rtn is %f %f %f %f %f %f\n", getCoord.tran.x, getCoord.tran.y, getCoord.tran.z, getCoord.rpy.rx, getCoord.rpy.ry, getCoord.rpy.rz);
    robot.CloseRPC();
    return 0;
    }

UDP Extended Axis Motion
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP extended axis motion
    * @param [in] pos Target position
    * @param [in] ovl Speed percentage
    * @param [in] blend Smoothing parameter (mm or ms); -1: Wait for motion completion
    * @return Error code
    */
        errno_t ExtAxisMove(ExaxisPos pos, double ovl, double blend = -1);

UDP Extended Axis Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestUDPAxisCalib(void)
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
      ExaxisPos axisPos;
      axisPos.ePos[0] = 20;
      axisPos.ePos[1] = 0;
      axisPos.ePos[2] = 0;
      axisPos.ePos[3] = 0;
      robot.ExtAxisMove(axisPos, 50);
      robot.CloseRPC();
      return 0;
    }

UDP Extended Axis and Robot Joint Motion Synchronous Motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP extended axis and robot joint motion synchronous motion
    * @param [in] joint_pos Target joint position, unit deg
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool Tool coordinate number, range [0~14]
    * @param [in] user Workpiece coordinate number, range [0~14]
    * @param [in] vel Speed percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], temporarily unavailable
    * @param [in] ovl Speed scaling factor, range [0~100]
    * @param [in] epos Extended axis position, unit mm
    * @param [in] blendT [-1.0]-Motion complete (blocking), [0~500.0]-Smoothing time (non-blocking), unit ms
    * @param [in] offset_flag 0-No offset, 1-Base coordinate/workpiece coordinate offset, 2-Tool coordinate offset
    * @param [in] offset_pos Pose offset
    * @return Error code
    */
    errno_t ExtAxisSyncMoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos epos, float blendT, byte offset_flag, DescPose offset_pos);

UDP Extended Axis and Robot Joint Motion Synchronization (Automatic Forward Kinematics Calculation)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP Extended Axis and Robot Joint Motion Synchronization (Automatic Forward Kinematics Calculation)
    * @param [in] joint_pos Target joint position, unit: deg
    * @param [in] tool Tool coordinate number, range [0~14]
    * @param [in] user Workpiece coordinate number, range [0~14]
    * @param [in] vel Velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], temporarily not available
    * @param [in] ovl Velocity scaling factor, range [0~100]
    * @param [in] epos Extended axis position, unit: mm
    * @param [in] blendT [-1.0]-Move to position (blocking), [0~500.0]-Smoothing time (non-blocking), unit: ms
    * @param [in] offset_flag 0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos Pose offset value
    * @return Error code
    */
    errno_t ExtAxisSyncMoveJ(JointPos joint_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos epos, float blendT, uint8_t offset_flag, DescPose offset_pos);

UDP Extended Axis and Robot Joint Motion Synchronous Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int testSyncMoveJ()
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
      //1. Calibrate and apply robot tool coordinate system. You can use the four-point or six-point method for tool coordinate system calibration and application. The interfaces involved in tool coordinate system calibration are as follows:
      //  int SetToolPoint(int point_num); //Set tool reference point - six-point method
      //  int ComputeTool(ref DescPose tcp_pose); //Calculate tool coordinate system
      //  int SetTcp4RefPoint(int point_num);  //Set tool reference point - four-point method
      //  int ComputeTcp4(ref DescPose tcp_pose);  //Calculate tool coordinate system - four-point method
      //  int SetToolCoord(int id, DescPose coord, int type, int install); //Set application tool coordinate system
      //  int SetToolList(int id, DescPose coord, int type, int install);  //Set application tool coordinate system list
      //2. Set UDP communication parameters and load UDP communication
      robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
      robot.ExtDevLoadUDPDriver();
      //3. Set extended axis parameters, including extended axis type, extended axis driver parameters, extended axis DH parameters
      robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //Single-axis positioner and DH parameters
      robot.SetRobotPosToAxis(1); //Extended axis installation position
      robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //Servo driver parameters. This example is for a single-axis positioner, so only one driver parameter needs to be set. If you choose an extended axis type with multiple axes, you need to set driver parameters for each axis.
      //4. Set the selected axis enable and homing
      robot.ExtAxisServoOn(1, 0);
      robot.ExtAxisSetHoming(1, 0, 20, 3);
      //5. Perform extended axis coordinate system calibration and application
      DescPose pos = {/* Enter your calibration point coordinates */ };
      robot.SetRefPointInExAxisEnd(pos);
      robot.PositionorSetRefPoint(1); /* You need to call this interface 4 times to complete the calibration of the extended axis using four different position points */
      DescPose coord = {};
      robot.PositionorComputeECoordSys(coord); //Calculate extended axis calibration result
      robot.ExtAxisActiveECoordSys(1, 1, coord, 1); //Apply the calibration result to the extended axis coordinate system
      //6. Calibrate the workpiece coordinate system on the extended axis. You need to use the following interfaces:
      //int SetWObjCoordPoint(int point_num);
      //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
      //int SetWObjCoord(int id, DescPose coord);
      //int SetWObjList(int id, DescPose coord);
      //7. Record your synchronous joint motion starting point
      DescPose startdescPose = {/*Enter your coordinates*/ };
      JointPos startjointPos = {/*Enter your coordinates*/ };
      ExaxisPos startexaxisPos = {/* Enter your extended axis starting point coordinates */ };
      //8. Record your synchronous joint motion end point coordinates
      DescPose enddescPose = {/*Enter your coordinates*/ };
      JointPos endjointPos = {/*Enter your coordinates*/ };
      ExaxisPos endexaxisPos = {/* Enter your extended axis end point coordinates */ };
      //9. Write synchronous motion program
      //Move to starting point, assuming the applied tool coordinate system and workpiece coordinate system are both 1
      robot.ExtAxisMove(startexaxisPos, 20);
      DescPose offdese = { 0, 0, 0, 0, 0, 0 };
      robot.MoveJ(&startjointPos, &startdescPose, 1, 1, 100, 100, 100, &startexaxisPos, 0, 0, &offdese);
      //Start synchronous motion
      robot.ExtAxisSyncMoveJ(endjointPos, enddescPose, 1, 1, 100, 100, 100, endexaxisPos, -1, 0, offdese);
      robot.MoveJ(&startjointPos, 1, 1, 100, 100, 100, &startexaxisPos, 0, 0, &offdese);
      // Start synchronous motion
      robot.ExtAxisSyncMoveJ(endjointPos, 1, 1, 100, 100, 100, endexaxisPos, -1, 0, offdese);
      robot.CloseRPC();
    }

UDP Extended Axis and Robot Linear Motion Synchronous Motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP extended axis and robot linear motion synchronous motion
    * @param [in] joint_pos Target joint position, unit deg
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool Tool coordinate number, range [0~14]
    * @param [in] user Workpiece coordinate number, range [0~14]
    * @param [in] vel Speed percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], temporarily unavailable
    * @param [in] ovl Speed scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-Motion complete (blocking), [0~1000.0]-Smoothing radius (non-blocking), unit mm
    * @param [in] epos Extended axis position, unit mm
    * @param [in] offset_flag 0-No offset, 1-Base coordinate/workpiece coordinate offset, 2-Tool coordinate offset
    * @param [in] offset_pos Pose offset
    * @return Error code
    */
    errno_t ExtAxisSyncMoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, int offset_flag, DescPose offset_pos);

UDP Extended Axis and Robot Linear Motion Synchronization (Automatic Inverse Kinematics Calculation)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP Extended Axis and Robot Linear Motion Synchronization (Automatic Inverse Kinematics Calculation)
    * @param [in] desc_pos  Target Cartesian pose
    * @param [in] tool Tool coordinate number, range [0~14]
    * @param [in] user Workpiece coordinate number, range [0~14]
    * @param [in] vel Velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], temporarily not available
    * @param [in] ovl Velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-Move to position (blocking), [0~1000.0]-Smoothing radius (non-blocking), unit: mm
    * @param [in] epos Extended axis position, unit: mm
    * @param [in] offset_flag 0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos Pose offset value
    * @param [in] config Inverse kinematics joint space configuration, [-1]-Calculate based on current joint position, [0~7]-Solve based on specific joint space configuration
    * @return Error code
    */
    errno_t ExtAxisSyncMoveL(DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, uint8_t offset_flag, DescPose offset_pos, int config = -1);

UDP Extended Axis and Robot Linear Motion Synchronous Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int testSyncMoveL()
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
      //1. Calibrate and apply robot tool coordinate system. You can use the four-point or six-point method for tool coordinate system calibration and application. The interfaces involved in tool coordinate system calibration are as follows:
      //  int SetToolPoint(int point_num); //Set tool reference point - six-point method
      //  int ComputeTool(ref DescPose tcp_pose); //Calculate tool coordinate system
      //  int SetTcp4RefPoint(int point_num);  //Set tool reference point - four-point method
      //  int ComputeTcp4(ref DescPose tcp_pose);  //Calculate tool coordinate system - four-point method
      //  int SetToolCoord(int id, DescPose coord, int type, int install); //Set application tool coordinate system
      //  int SetToolList(int id, DescPose coord, int type, int install);  //Set application tool coordinate system list
      //2. Set UDP communication parameters and load UDP communication
      robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
      robot.ExtDevLoadUDPDriver();
      //3. Set extended axis parameters, including extended axis type, extended axis driver parameters, extended axis DH parameters
      robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //Single-axis positioner and DH parameters
      robot.SetRobotPosToAxis(1); //Extended axis installation position
      robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //Servo driver parameters. This example is for a single-axis positioner, so only one driver parameter needs to be set. If you choose an extended axis type with multiple axes, you need to set driver parameters for each axis.
      //4. Set the selected axis enable and homing
      robot.ExtAxisServoOn(1, 0);
      robot.ExtAxisSetHoming(1, 0, 20, 3);
      //5. Perform extended axis coordinate system calibration and application
      DescPose pos = {/* Enter your calibration point coordinates */ };
      robot.SetRefPointInExAxisEnd(pos);
      robot.PositionorSetRefPoint(1); /* You need to call this interface 4 times to complete the calibration of the extended axis using four different position points */
      DescPose coord = {};
      robot.PositionorComputeECoordSys(coord); //Calculate extended axis calibration result
      robot.ExtAxisActiveECoordSys(1, 1, coord, 1); //Apply the calibration result to the extended axis coordinate system
      //6. Calibrate the workpiece coordinate system on the extended axis. You need to use the following interfaces:
      //int SetWObjCoordPoint(int point_num);
      //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
      //int SetWObjCoord(int id, DescPose coord);
      //int SetWObjList(int id, DescPose coord);
      //7. Record your synchronous linear motion starting point
      DescPose startdescPose = {/*Enter your coordinates*/ };
      JointPos startjointPos = {/*Enter your coordinates*/ };
      ExaxisPos startexaxisPos = {/* Enter your extended axis starting point coordinates */ };
      //8. Record your synchronous linear motion end point coordinates
      DescPose enddescPose = {/*Enter your coordinates*/ };
      JointPos endjointPos = {/*Enter your coordinates*/ };
      ExaxisPos endexaxisPos = {/* Enter your extended axis end point coordinates */ };
      //9. Write synchronous motion program
      //Move to starting point, assuming the applied tool coordinate system and workpiece coordinate system are both 1
      robot.ExtAxisMove(startexaxisPos, 20);
      DescPose offdese = { 0, 0, 0, 0, 0, 0 };
      robot.MoveJ(&startjointPos, &startdescPose, 1, 1, 100, 100, 100, &startexaxisPos, 0, 0, &offdese);
      //Start synchronous motion
      robot.ExtAxisSyncMoveL(endjointPos, enddescPose, 1, 1, 100, 100, 100, 0, endexaxisPos, 0, offdese);
      robot.MoveJ(&startjointPos, 1, 1, 100, 100, 100, &startexaxisPos, 0, 0, &offdese);
      // Start synchronous motion
      robot.ExtAxisSyncMoveL(enddescPose, 1, 1, 100, 100, 100, 0, endexaxisPos, 0, offdese);
      robot.CloseRPC();
    }

UDP Extended Axis and Robot Arc Motion Synchronous Motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP extended axis and robot arc motion synchronous motion
    * @param [in] joint_pos_p Path point joint position, unit deg
    * @param [in] desc_pos_p Path point Cartesian pose
    * @param [in] ptool Tool coordinate number, range [0~14]
    * @param [in] puser Workpiece coordinate number, range [0~14]
    * @param [in] pvel Speed percentage, range [0~100]
    * @param [in] pacc Acceleration percentage, range [0~100], temporarily unavailable
    * @param [in] epos_p Intermediate point extended axis position, unit mm
    * @param [in] poffset_flag 0-No offset, 1-Base coordinate/workpiece coordinate offset, 2-Tool coordinate offset
    * @param [in] offset_pos_p Pose offset
    * @param [in] joint_pos_t Target point joint position, unit deg
    * @param [in] desc_pos_t Target point Cartesian pose
    * @param [in] ttool Tool coordinate number, range [0~14]
    * @param [in] tuser Workpiece coordinate number, range [0~14]
    * @param [in] tvel Speed percentage, range [0~100]
    * @param [in] tacc Acceleration percentage, range [0~100], temporarily unavailable
    * @param [in] epos_t Extended axis position, unit mm
    * @param [in] toffset_flag 0-No offset, 1-Base coordinate/workpiece coordinate offset, 2-Tool coordinate offset
    * @param [in] offset_pos_t Pose offset	 
    * @param [in] ovl Speed scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-Motion complete (blocking), [0~1000.0]-Smoothing radius (non-blocking), unit mm
    * @return Error code
    */
    errno_t ExtAxisSyncMoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos epos_p, int poffset_flag, DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos epos_t, int toffset_flag, DescPose offset_pos_t, float ovl, float blendR);

UDP Extended Axis and Robot Circular Motion Synchronization (Automatic Inverse Kinematics Calculation)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief UDP Extended Axis and Robot Circular Motion Synchronization (Automatic Inverse Kinematics Calculation)
    * @param [in] desc_pos_p  Path point Cartesian pose
    * @param [in] ptool Tool coordinate number, range [0~14]
    * @param [in] puser Workpiece coordinate number, range [0~14]
    * @param [in] pvel Velocity percentage, range [0~100]
    * @param [in] pacc Acceleration percentage, range [0~100], temporarily not available
    * @param [in] epos_p Extended axis position, unit: mm
    * @param [in] poffset_flag 0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos_p Pose offset value
    * @param [in] desc_pos_t  Target point Cartesian pose
    * @param [in] ttool Tool coordinate number, range [0~14]
    * @param [in] tuser Workpiece coordinate number, range [0~14]
    * @param [in] tvel Velocity percentage, range [0~100]
    * @param [in] tacc Acceleration percentage, range [0~100], temporarily not available
    * @param [in] epos_t Extended axis position, unit: mm
    * @param [in] toffset_flag 0-No offset, 1-Offset in base coordinate system/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos_t Pose offset value
    * @param [in] ovl Velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-Move to position (blocking), [0~1000.0]-Smoothing radius (non-blocking), unit: mm
    * @param [in] config Inverse kinematics joint space configuration, [-1]-Calculate based on current joint position, [0~7]-Solve based on specific joint space configuration
    * @return Error code
    */
    errno_t ExtAxisSyncMoveC(DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos epos_p, uint8_t poffset_flag, DescPose offset_pos_p, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos epos_t, uint8_t toffset_flag, DescPose offset_pos_t, float ovl, float blendR, int config = -1);    

UDP Extended Axis and Robot Arc Motion Synchronous Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int testSyncMoveC()
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
      //1. Calibrate and apply robot tool coordinate system. You can use the four-point or six-point method for tool coordinate system calibration and application. The interfaces involved in tool coordinate system calibration are as follows:
      //  int SetToolPoint(int point_num); //Set tool reference point - six-point method
      //  int ComputeTool(ref DescPose tcp_pose); //Calculate tool coordinate system
      //  int SetTcp4RefPoint(int point_num);  //Set tool reference point - four-point method
      //  int ComputeTcp4(ref DescPose tcp_pose);  //Calculate tool coordinate system - four-point method
      //  int SetToolCoord(int id, DescPose coord, int type, int install); //Set application tool coordinate system
      //  int SetToolList(int id, DescPose coord, int type, int install);  //Set application tool coordinate system list
      //2. Set UDP communication parameters and load UDP communication
      robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
      robot.ExtDevLoadUDPDriver();
      //3. Set extended axis parameters, including extended axis type, extended axis driver parameters, extended axis DH parameters
      robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //Single-axis positioner and DH parameters
      robot.SetRobotPosToAxis(1); //Extended axis installation position
      robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //Servo driver parameters. This example is for a single-axis positioner, so only one driver parameter needs to be set. If you choose an extended axis type with multiple axes, you need to set driver parameters for each axis.
      //4. Set the selected axis enable and homing
      robot.ExtAxisServoOn(1, 0);
      robot.ExtAxisSetHoming(1, 0, 20, 3);
      //5. Perform extended axis coordinate system calibration and application
      DescPose pos = {/* Enter your calibration point coordinates */ };
      robot.SetRefPointInExAxisEnd(pos);
      robot.PositionorSetRefPoint(1); /* You need to call this interface 4 times to complete the calibration of the extended axis using four different position points */
      DescPose coord = {};
      robot.PositionorComputeECoordSys(coord); //Calculate extended axis calibration result
      robot.ExtAxisActiveECoordSys(1, 1, coord, 1); //Apply the calibration result to the extended axis coordinate system
      //6. Calibrate the workpiece coordinate system on the extended axis. You need to use the following interfaces:
      //int SetWObjCoordPoint(int point_num);
      //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
      //int SetWObjCoord(int id, DescPose coord);
      //int SetWObjList(int id, DescPose coord);
      //7. Record your synchronous arc motion starting point
      DescPose startdescPose = {/*Enter your coordinates*/ };
      JointPos startjointPos = {/*Enter your coordinates*/ };
      ExaxisPos startexaxisPos = {/* Enter your extended axis starting point coordinates */ };
      //8. Record your synchronous arc motion end point coordinates
      DescPose enddescPose = {/*Enter your coordinates*/ };
      JointPos endjointPos = {/*Enter your coordinates*/ };
      ExaxisPos endexaxisPos = {/* Enter your extended axis end point coordinates */ };
      //9. Record your synchronous arc motion intermediate point coordinates
      DescPose middescPose = {/*Enter your coordinates*/ };
      JointPos midjointPos = {/*Enter your coordinates*/ };
      ExaxisPos midexaxisPos = {/* Enter the extended axis coordinates at the robot arc intermediate point */ };
      //10. Write synchronous motion program
      //Move to starting point, assuming the applied tool coordinate system and workpiece coordinate system are both 1
      robot.ExtAxisMove(startexaxisPos, 20);
      DescPose offdese = { 0, 0, 0, 0, 0, 0 };
      robot.MoveJ(&startjointPos, &startdescPose, 1, 1, 100, 100, 100, &startexaxisPos, 0, 0, &offdese);
      //Start synchronous motion
      robot.ExtAxisSyncMoveC(midjointPos, middescPose, 1, 1, 100, 100, midexaxisPos, 0, offdese, endjointPos, enddescPose, 1, 1, 100, 100, endexaxisPos, 0, offdese, 100, 0);
      robot.MoveJ(&startjointPos, 1, 1, 100, 100, 100, &startexaxisPos, 0, 0, &offdese);
      // Start synchronous motion
      robot.ExtAxisSyncMoveC(middescPose, 1, 1, 100, 100, midexaxisPos, 0, offdese, enddescPose, 1, 1, 100, 100, endexaxisPos, 0, offdese, 100, 0);
      robot.CloseRPC();
    }
    
Set Extended DO
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set extended DO
    * @param [in] DONum DO number
    * @param [in] bOpen Switch, true-on; false-off
    * @param [in] smooth Whether to smooth
    * @param [in] block Whether to block
    * @return Error code
    */
    errno_t SetAuxDO(int DONum, bool bOpen, bool smooth, bool block);
        
Set Extended AO
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set extended AO
    * @param [in] AONum AO number 
    * @param [in] value Analog value [0-4095]
    * @param [in] block Whether to block
    * @return Error code
    */
    errno_t SetAuxAO(int AONum, double value, bool block);
  
Set Extended DI Input Filter Time
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set extended DI input filter time
    * @param [in] filterTime Filter time (ms)
    * @return Error code
    */
    errno_t SetAuxDIFilterTime(int filterTime);

Set Extended AI Input Filter Time
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set extended AI input filter time
    * @param [in] filterTime Filter time (ms)
    * @return Error code
    */
    errno_t SetAuxAIFilterTime(int filterTime);

Wait for Extended DI Input
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Wait for extended DI input
    * @param [in] DINum DI number
    * @param [in] bOpen Switch, 0-off; 1-on
    * @param [in] time Maximum wait time (ms)
    * @param [in] errorAlarm Whether to continue motion
    * @return Error code
    */
    errno_t WaitAuxDI(int DINum, bool bOpen, int time, bool errorAlarm);
    
Wait for Extended AI Input
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Wait for extended AI input
    * @param [in] AINum AI number
    * @param [in] sign 0-Greater than; 1-Less than
    * @param [in] value AI value
    * @param [in] time Maximum wait time (ms)
    * @param [in] errorAlarm Whether to continue motion
    * @return Error code
    */
    errno_t WaitAuxAI(int AINum, int sign, int value, int time, bool errorAlarm);
        
Get Extended DI Value
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get extended DI value
    * @param [in] DINum DI number
    * @param [in] isNoBlock Whether to block
    * @param [out] isOpen 0-off; 1-on
    * @return Error code
    */
    errno_t GetAuxDI(int DINum, bool isNoBlock, bool& isOpen);
            
Get Extended AI Value
+++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.4.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get extended AI value
    * @param [in] AINum AI number
    * @param [in] isNoBlock Whether to block
    * @param [in] value Input value
    * @return Error code
    */
    errno_t GetAuxAI(int AINum, bool isNoBlock, int& value);

Extended IO Code Example
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestAuxDOAO(void)
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
      for (int i = 0; i < 128; i++)
      {
        robot.SetAuxDO(i, true, false, true);
        Sleep(100);
      }
      for (int i = 0; i < 128; i++)
      {
        robot.SetAuxDO(i, false, false, true);
        Sleep(100);
      }
      for (int i = 0; i < 409; i++)
      {
        robot.SetAuxAO(0, i * 10, true);
        robot.SetAuxAO(1, 4095 - i * 10, true);
        robot.SetAuxAO(2, i * 10, true);
        robot.SetAuxAO(3, 4095 - i * 10, true);
        Sleep(10);
      }
      robot.SetAuxDIFilterTime(10);
      robot.SetAuxAIFilterTime(0, 10);
      for (int i = 0; i < 20; i++)
      {
        bool curValue = false;
        int rtn = robot.GetAuxDI(i, false, curValue);
        cout << "DI" << i << "  " << curValue << endl;
      }
      int curValue = -1;
      for (int i = 0; i < 4; i++)
      {
        rtn = robot.GetAuxAI(i, true, curValue);
      }
      robot.WaitAuxDI(1, false, 1000, false);
      robot.WaitAuxAI(1, 1, 132, 1000, false);
      robot.CloseRPC();
      return 0;
    }

Mobile Device Enable
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Mobile device enable
    * @param enable false-disable; true-enable
    * @return Error code
    */
    errno_t TractorEnable(bool enable);

Mobile Device Homing
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Mobile device homing
    * @return Error code
    */
    errno_t TractorHoming();

Mobile Device Linear Motion
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:
    
    /**
    * @brief Mobile device linear motion
    * @param distance Linear motion distance (mm)
    * @param vel Linear motion speed percentage (0-100)
    * @return Error code
    */
    errno_t TractorMoveL(double distance, double vel);

Mobile Device Arc Motion
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Mobile device arc motion
    * @param radio Arc motion radius (mm)
    * @param angle Arc motion angle (°)
    * @param vel Linear motion speed percentage (0-100)
    * @return Error code
    */
    errno_t TractorMoveC(double radio, double angle, double vel);

Mobile Device Stop Motion
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Mobile device stop motion
    * @return Error code
    */
    errno_t TractorStop();

Mobile Device Code Example
+++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestTractor(void)
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
      robot.ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10, 1);
      robot.ExtDevLoadUDPDriver();
      rtn = robot.ExtAxisServoOn(1, 1);
      rtn = robot.ExtAxisServoOn(2, 1);
      robot.Sleep(2000);
      robot.ExtAxisSetHoming(1, 0, 10, 2);
      robot.Sleep(2000);
      rtn = robot.ExtAxisSetHoming(2, 0, 10, 2);
      robot.Sleep(4000);
      robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
      robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
      robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0);
      robot.TractorEnable(false);
      robot.Sleep(2000);
      robot.TractorEnable(true);
      robot.Sleep(2000);
      robot.TractorHoming();
      robot.Sleep(2000);
      robot.TractorMoveL(100, 2);
      robot.Sleep(5000);
      robot.TractorStop();
      robot.TractorMoveL(-100, 20);
      robot.Sleep(5000);
      robot.TractorMoveC(300, 90, 20);
      robot.Sleep(10000);
      robot.TractorMoveC(300, -90, 20);
      robot.Sleep(1);
      robot.CloseRPC();
      return 0;
    }