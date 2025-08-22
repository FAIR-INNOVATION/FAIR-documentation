Robot Welding
======================

.. toctree:: 
    :maxdepth: 5

Set Welding Process Curve Parameters
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Set welding process curve parameters
     * @param [in] id Welding process ID (1-99)
     * @param [in] startCurrent Arc starting current (A)
     * @param [in] startVoltage Arc starting voltage (V)
     * @param [in] startTime Arc starting time (ms)
     * @param [in] weldCurrent Welding current (A)
     * @param [in] weldVoltage Welding voltage (V)
     * @param [in] endCurrent Arc ending current (A)
     * @param [in] endVoltage Arc ending voltage (V)
     * @param [in] endTime Arc ending time (ms)
     * @return Error code
     */
    errno_t WeldingSetProcessParam(int id, double startCurrent, double startVoltage, double startTime, double weldCurrent, double weldVoltage, double endCurrent, double endVoltage, double endTime);

Get Welding Process Curve Parameters
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Get welding process curve parameters
     * @param [in] id Welding process ID (1-99)
     * @param [out] startCurrent Arc starting current (A)
     * @param [out] startVoltage Arc starting voltage (V)
     * @param [out] startTime Arc starting time (ms)
     * @param [out] weldCurrent Welding current (A)
     * @param [out] weldVoltage Welding voltage (V)
     * @param [out] endCurrent Arc ending current (A)
     * @param [out] endVoltage Arc ending voltage (V)
     * @param [out] endTime Arc ending time (ms)
     * @return Error code
     */
    errno_t WeldingGetProcessParam(int id, double& startCurrent, double& startVoltage, double& startTime, double& weldCurrent, double& weldVoltage, double& endCurrent, double& endVoltage, double& endTime);

Set Welding Current to Analog Output Relationship
+++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set welding current to analog output relationship
    * @param [in] currentMin Current value at left point of linear relationship (A)
    * @param [in] currentMax Current value at right point of linear relationship (A)
    * @param [in] outputVoltageMin Analog output voltage at left point (V)
    * @param [in] outputVoltageMax Analog output voltage at right point (V)
    * @return Error code
    */
    errno_t WeldingSetCurrentRelation(double currentMin, double currentMax, double outputVoltageMin, double outputVoltageMax);

Set Welding Voltage to Analog Output Relationship
+++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set welding voltage to analog output relationship
    * @param [in] weldVoltageMin Welding voltage at left point of linear relationship (A)
    * @param [in] weldVoltageMax Welding voltage at right point of linear relationship (A)
    * @param [in] outputVoltageMin Analog output voltage at left point (V)
    * @param [in] outputVoltageMax Analog output voltage at right point (V)
    * @return Error code
    */
    errno_t WeldingSetVoltageRelation(double weldVoltageMin, double weldVoltageMax, double outputVoltageMin, double outputVoltageMax);

Get Welding Current to Analog Output Relationship
+++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get welding current to analog output relationship
    * @param [out] currentMin Current value at left point of linear relationship (A)
    * @param [out] currentMax Current value at right point of linear relationship (A)
    * @param [out] outputVoltageMin Analog output voltage at left point (V)
    * @param [out] outputVoltageMax Analog output voltage at right point (V)
    * @return Error code
    */
    errno_t WeldingGetCurrentRelation(double *currentMin, double *currentMax, double *outputVoltageMin, double *outputVoltageMax);

Get Welding Voltage to Analog Output Relationship
+++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Get welding voltage to analog output relationship
    * @param [out] weldVoltageMin Welding voltage at left point of linear relationship (A)
    * @param [out] weldVoltageMax Welding voltage at right point of linear relationship (A)
    * @param [out] outputVoltageMin Analog output voltage at left point (V)
    * @param [out] outputVoltageMax Analog output voltage at right point (V)
    * @return Error code
    */
    errno_t WeldingGetVoltageRelation(double *weldVoltageMin, double *weldVoltageMax, double *outputVoltageMin, double *outputVoltageMax);

Set Welding Current
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set welding current
    * @param [in] ioType Control IO type 0-Control box IO; 1-Extension IO
    * @param [in] current Welding current value (A)
    * @param [in] AOIndex Welding current control box analog output port (0-1)
    * @param [in] blend Whether to smooth 0-Not smooth; 1-Smooth
    * @return Error code
    */
    errno_t WeldingSetCurrent(int ioType, double current, int AOIndex, int blend);

Set Welding Voltage
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set welding voltage
    * @param [in] ioType Control IO type 0-Control box IO; 1-Extension IO
    * @param [in] voltage Welding voltage value (V)
    * @param [in] AOIndex Welding voltage control box analog output port (0-1)
    * @param [in] blend Whether to smooth 0-Not smooth; 1-Smooth
    * @return Error code
    */
    errno_t WeldingSetVoltage(int ioType, double voltage, int AOIndex, int blend);

Set Weaving Parameters
+++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
     * @brief Set weaving parameters
     * @param [in] weaveNum Weaving parameter configuration ID
     * @param [in] weaveType Weaving type: 
     *        0-Horizontal triangular wave weaving;
     *        1-Vertical L-shaped triangular wave weaving;
     *        2-Clockwise circular weaving;
     *        3-Counter-clockwise circular weaving;
     *        4-Horizontal sine wave weaving;
     *        5-Vertical L-shaped sine wave weaving;
     *        6-Vertical triangular wave weaving;
     *        7-Vertical sine wave weaving
     * @param [in] weaveFrequency Weaving frequency (Hz)
     * @param [in] weaveIncStayTime Wait mode: 
     *        0-Period excludes wait time;
     *        1-Period includes wait time
     * @param [in] weaveRange Weaving amplitude (mm)
     * @param [in] weaveLeftRange Left chord length for vertical triangular weaving (mm)
     * @param [in] weaveRightRange Right chord length for vertical triangular weaving (mm)
     * @param [in] additionalStayTime Vertical triangular weaving apex dwell time (mm)
     * @param [in] weaveLeftStayTime Left dwell time (ms)
     * @param [in] weaveRightStayTime Right dwell time (ms)
     * @param [in] weaveCircleRadio Circular weaving-callback ratio (0-100%)
     * @param [in] weaveStationary Position wait during weaving:
     *        0-Position continues moving during wait time;
     *        1-Position remains static during wait time
     * @param [in] weaveYawAngle Weaving direction azimuth angle (rotation around Z-axis), in °
     * @param [in] weaveRotAngle Weaving direction tilt angle (deflection around X-axis), in °
     * @return Error code
     */
    errno_t WeaveSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, double weaveLeftRange, double weaveRightRange, int additionalStayTime, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary, double weaveYawAngle, double weaveRotAngle = 0);

Welding Parameter Setup Code Example
++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestSetWeldParam(void)
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
      robot.WeldingSetProcessParam(1, 177, 27, 1000, 178, 28, 176, 26, 1000);
      robot.WeldingSetProcessParam(2, 188, 28, 555, 199, 29, 133, 23, 333);
      double startCurrent = 0;
      double startVoltage = 0;
      double startTime = 0;
      double weldCurrent = 0;
      double weldVoltage = 0;
      double endCurrent = 0;
      double endVoltage = 0;
      double endTime = 0;
      robot.WeldingGetProcessParam(1, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);
      cout << "the Num 1 process param is " << startCurrent << " " << startVoltage << " " << startTime << " " << weldCurrent << " " << weldVoltage << " " << endCurrent << " " << endVoltage << " " << endTime << endl;
      robot.WeldingGetProcessParam(2, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);
      cout << "the Num 2 process param is " << startCurrent << " " << startVoltage << " " << startTime << " " << weldCurrent << " " << weldVoltage << " " << endCurrent << " " << endVoltage << " " << endTime << endl;
      rtn = robot.WeldingSetCurrentRelation(0, 400, 0, 10, 0);
      cout << "WeldingSetCurrentRelation rtn is: " << rtn << endl;
      rtn = robot.WeldingSetVoltageRelation(0, 40, 0, 10, 1);
      cout << "WeldingSetVoltageRelation rtn is: " << rtn << endl;
      double current_min = 0;
      double current_max = 0;
      double vol_min = 0;
      double vol_max = 0;
      double output_vmin = 0;
      double output_vmax = 0;
      int curIndex = 0;
      int volIndex = 0;
      rtn = robot.WeldingGetCurrentRelation(&current_min, &current_max, &output_vmin, &output_vmax, &curIndex);
      cout << "WeldingGetCurrentRelation rtn is: " << rtn << endl;
      cout << "current min " << current_min << " current max " << current_max << " output vol min " << output_vmin << " output vol max " << output_vmax << endl;
      rtn = robot.WeldingGetVoltageRelation(&vol_min, &vol_max, &output_vmin, &output_vmax, &volIndex);
      cout << "WeldingGetVoltageRelation rtn is: " << rtn << endl;
      cout << "vol min " << vol_min << " vol max " << vol_max << " output vol min " << output_vmin << " output vol max " << output_vmax << endl;
      rtn = robot.WeldingSetCurrent(1, 100, 0, 0);
      cout << "WeldingSetCurrent rtn is: " << rtn << endl;
      this_thread::sleep_for(chrono::seconds(3));
      rtn = robot.WeldingSetVoltage(1, 10, 0, 0);
      cout << "WeldingSetVoltage rtn is: " << rtn << endl;
      rtn = robot.WeaveSetPara(0, 0, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 60.000000);
      cout << "rtn is: " << rtn << endl;
      robot.WeaveOnlineSetPara(0, 0, 1, 0, 20, 0, 0, 0, 0);
      rtn = robot.WeldingSetCheckArcInterruptionParam(1, 200);
      printf("WeldingSetCheckArcInterruptionParam  %d\n", rtn);
      rtn = robot.WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0);
      printf("WeldingSetReWeldAfterBreakOffParam  %d\n", rtn);
      int enable = 0;
      double length = 0;
      double velocity = 0;
      int moveType = 0;
      int checkEnable = 0;
      int arcInterruptTimeLength = 0;
      rtn = robot.WeldingGetCheckArcInterruptionParam(&checkEnable, &arcInterruptTimeLength);
      printf("WeldingGetCheckArcInterruptionParam checkEnable %d  arcInterruptTimeLength %d\n", checkEnable, arcInterruptTimeLength);
      rtn = robot.WeldingGetReWeldAfterBreakOffParam(&enable, &length, &velocity, &moveType);
      printf("WeldingGetReWeldAfterBreakOffParam enable = %d, length = %lf, velocity = %lf, moveType = %d\n", enable, length, velocity, moveType);
      robot.SetWeldMachineCtrlModeExtDoNum(17);
      for (int i = 0; i < 5; i++)
      {
        robot.SetWeldMachineCtrlMode(0);
        robot.Sleep(1000);
        robot.SetWeldMachineCtrlMode(1);
        robot.Sleep(1000);
      }
      robot.CloseRPC();
      return 0;
    }

Instantaneously Set Weaving Parameters
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Instantly set weaving parameters
    * @param [in] weaveNum Weaving parameter configuration number
    * @param [in] weaveType Weaving type 0-Flat triangular wave; 1-Vertical L-shaped triangular wave; 2-Clockwise circular; 3-Counter-clockwise circular; 4-Flat sine wave; 5-Vertical L-shaped sine wave; 6-Vertical triangular wave; 7-Vertical sine wave
    * @param [in] weaveFrequency Weaving frequency (Hz)
    * @param [in] weaveIncStayTime Wait mode 0-Cycle excludes wait time; 1-Cycle includes wait time
    * @param [in] weaveRange Weaving amplitude (mm)
    * @param [in] weaveLeftStayTime Left dwell time (ms)
    * @param [in] weaveRightStayTime Right dwell time (ms)
    * @param [in] weaveCircleRadio Circular weaving - callback ratio (0-100%)
    * @param [in] weaveStationary Weaving position wait, 0-Position continues moving during wait; 1-Position remains stationary during wait
    * @return Error code
    */
    errno_t WeaveOnlineSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary);

Set Robot Welding Arc Unexpected Interruption Detection Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
	 * @brief Set robot welding arc unexpected interruption detection parameters
	 * @param [in] checkEnable Enable detection; 0-Disable; 1-Enable
	 * @param [in] arcInterruptTimeLength Arc interruption confirmation duration (ms)
	 * @return Error code
    */
	errno_t WeldingSetCheckArcInterruptionParam(int checkEnable, int arcInterruptTimeLength);

Get Robot Welding Arc Unexpected Interruption Detection Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
	 * @brief Get robot welding arc unexpected interruption detection parameters
	 * @param [out] checkEnable Whether detection is enabled; 0-Disabled; 1-Enabled
	 * @param [out] arcInterruptTimeLength Arc interruption confirmation duration (ms)
	 * @return Error code
    */
	errno_t WeldingGetCheckArcInterruptionParam(int* checkEnable, int* arcInterruptTimeLength);

Set Robot Welding Interruption Recovery Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
	 * @brief Set robot welding interruption recovery parameters
	 * @param [in] enable Whether to enable welding interruption recovery
	 * @param [in] length Weld overlap distance (mm)
	 * @param [in] velocity Robot return to re-arc point speed percentage (0-100)
	 * @param [in] moveType Robot movement to re-arc point method; 0-LIN; 1-PTP
	 * @return Error code
    */
	errno_t WeldingSetReWeldAfterBreakOffParam(int enable, double length, double velocity, int moveType);
    
Get Robot Welding Interruption Recovery Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
	 * @brief Get robot welding interruption recovery parameters
	 * @param [out] enable Whether welding interruption recovery is enabled
	 * @param [out] length Weld overlap distance (mm)
	 * @param [out] velocity Robot return to re-arc point speed percentage (0-100)
	 * @param [out] moveType Robot movement to re-arc point method; 0-LIN; 1-PTP
	 * @return Error code
    */
	errno_t WeldingGetReWeldAfterBreakOffParam(int* enable, double* length, double* velocity, int* moveType);

Set Welder Control Mode Extended DO Port
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set welder control mode extended DO port
    * @param DONum Welder control mode DO port (0-127)
    * @return Error code
    */
    errno_t SetWeldMachineCtrlModeExtDoNum(int DONum);

Set Welder Control Mode
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set welder control mode
    * @param mode Welder control mode; 0-Unary
    * @return Error code
    */
    errno_t SetWeldMachineCtrlMode(int mode);

Welding Start
++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Welding start
    * @param [in] ioType IO type 0-Controller IO; 1-Extended IO
    * @param [in] arcNum Welder configuration file number
    * @param [in] timeout Arc ignition timeout
    * @return Error code
    */
    errno_t ARCStart(int ioType, int arcNum, int timeout);

Welding End
++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Welding end
    * @param [in] ioType IO type 0-Controller IO; 1-Extended IO
    * @param [in] arcNum Welder configuration file number
    * @param [in] timeout Arc extinguishing timeout
    * @return Error code
    */
    errno_t ARCEnd(int ioType, int arcNum, int timeout);

Weaving Start
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Weaving start
    * @param [in] weaveNum Weaving parameter configuration number
    * @return Error code
    */
    errno_t WeaveStart(int weaveNum);

Weaving End
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Weaving end
    * @param [in] weaveNum Weaving parameter configuration number
    * @return Error code
    */
    errno_t WeaveEnd(int weaveNum);

Forward Wire Feeding
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Forward wire feeding
    * @param [in] ioType IO type 0-Controller IO; 1-Extended IO
    * @param [in] wireFeed Wire feed control 0-Stop feeding; 1-Feed wire
    * @return Error code
    */
    errno_t SetForwardWireFeed(int ioType, int wireFeed);

Reverse Wire Feeding
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Reverse wire feeding
    * @param [in] ioType IO type 0-Controller IO; 1-Extended IO
    * @param [in] wireFeed Wire feed control 0-Stop feeding; 1-Feed wire
    * @return Error code
    */
    errno_t SetReverseWireFeed(int ioType, int wireFeed);

Gas Feeding
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Gas feeding
    * @param [in] ioType IO type 0-Controller IO; 1-Extended IO
    * @param [in] airControl Gas feed control 0-Stop feeding; 1-Feed gas
    * @return Error code
    */
    errno_t SetAspirated(int ioType, int airControl);

Set Robot to Resume Welding After Interruption
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
	 * @brief Set robot to resume welding after interruption
	 * @return Error code
    */
	errno_t WeldingStartReWeldAfterBreakOff();

Set Robot to Abort Welding After Interruption
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
	 * @brief Set robot to abort welding after interruption
	 * @return Error code
	 */
	errno_t WeldingAbortWeldAfterBreakOff();

Robot Welding Control Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestWelding(void)
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
      robot.SetForwardWireFeed(0, 1);
      robot.Sleep(1000);
      robot.SetForwardWireFeed(0, 0);
      robot.SetReverseWireFeed(0, 1);
      robot.Sleep(1000);
      robot.SetReverseWireFeed(0, 0);
      robot.SetAspirated(0, 1);
      robot.Sleep(1000);
      robot.SetAspirated(0, 0);
      robot.WeldingSetCurrent(1, 230, 0, 0);
      robot.WeldingSetVoltage(1, 24, 0, 1);
      DescPose p1Desc(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
      JointPos p1Joint(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);
      DescPose p2Desc(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
      JointPos p2Joint(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);
      ExaxisPos exaxisPos(0, 0, 0, 0);
      DescPose offdese(0, 0, 0, 0, 0, 0);
      robot.MoveJ(&p1Joint, &p1Desc, 13, 0, 20, 100, 100, &exaxisPos, -1, 0, &offdese);
      robot.ARCStart(1, 0, 10000);
      robot.WeaveStart(0);
      robot.MoveL(&p2Joint, &p2Desc, 13, 0, 20, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
      robot.ARCEnd(1, 0, 10000);
      robot.WeaveEnd(0);
      robot.WeldingStartReWeldAfterBreakOff();
      robot.WeldingAbortWeldAfterBreakOff();
      robot.CloseRPC();
      return 0;
    }


Segment Welding Start
+++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Segment welding start
    * @param [in] startDesePos Starting point Cartesian position
    * @param [in] endDesePos Ending point Cartesian pose
    * @param [in] startJPos Starting point joint pose
    * @param [in] endJPos Ending point joint pose
    * @param [in] weldLength Welding segment length (mm)
    * @param [in] noWeldLength Non-welding segment length (mm)
    * @param [in] weldIOType Welding IO type (0-Control box IO; 1-Extended IO)
    * @param [in] arcNum Welder configuration file number
    * @param [in] weldTimeout Arc ignition/extinguishing timeout
    * @param [in] isWeave Whether to weave
    * @param [in] weaveNum Weaving parameter configuration number
    * @param [in] tool Tool coordinate number, range [0~14]
    * @param [in] user Workpiece coordinate number, range [0~14]
    * @param [in] vel Speed percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not currently open
    * @param [in] ovl Speed scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-Move to position (blocking), [0~1000.0]-Smoothing radius (non-blocking), unit mm
    * @param [in] epos Extended axis position, unit mm
    * @param [in] search 0-No wire search, 1-Wire search
    * @param [in] offset_flag 0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos Pose offset
    * @return Error code
    */
    errno_t SegmentWeldStart(DescPose *startDesePos, DescPose *endDesePos, JointPos *startJPos, JointPos *endJPos, double weldLength, double noWeldLength, int weldIOType, int arcNum, int weldTimeout, bool isWeave, int weaveNum, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos);

Robot Segment Welding Code Example
+++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    int TestSegWeld(void)
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
      robot.WeldingSetCurrent(1, 230, 0, 0);
      robot.WeldingSetVoltage(1, 24, 0, 1);
      DescPose p1Desc(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
      JointPos p1Joint(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);
      DescPose p2Desc(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
      JointPos p2Joint(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);
      ExaxisPos exaxisPos(0, 0, 0, 0);
      DescPose offdese(0, 0, 0, 0, 0, 0);
      rtn = robot.SegmentWeldStart(&p1Desc, &p2Desc, &p1Joint, &p2Joint, 20, 20, 0, 0, 5000, 0, 0, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
      printf("SegmentWeldStart rtn is %d\n", rtn);
      robot.CloseRPC();
      return 0;
    }


Simulation Weaving Start
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Simulation weaving start
     * @param [in] weaveNum Weaving parameter number
     * @return Error code
     */
    errno_t WeaveStartSim(int weaveNum);

Simulation Weaving End
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Simulation weaving end
     * @param [in] weaveNum Weaving parameter number
     * @return Error code
     */
    errno_t WeaveEndSim(int weaveNum);

Start Trajectory Detection Warning (No Movement)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Start trajectory detection warning (no movement)
     * @param [in] weaveNum  Weaving parameter number
     * @return Error code
     */
    errno_t WeaveInspectStart(int weaveNum);

End Trajectory Detection Warning (No Movement)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief End trajectory detection warning (no movement)
     * @param [in] weaveNum  Weaving parameter number
     * @return Error code
     */
    errno_t WeaveInspectEnd(int weaveNum);

Weaving Gradual Change Start
++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
     * @brief Weaving gradual change start
     * @param [in] weaveChangeFlag 1-Change weaving parameters; 2-Change weaving parameters + welding speed
     * @param [in] weaveNum Weaving number 
     * @param [in] velStart Welding start speed (cm/min)
     * @param [in] velEnd Welding end speed (cm/min)
     * @return Error code
     */
     errno_t WeaveChangeStart(int weaveChangeFlag, int weaveNum, double velStart, double velEnd);

Robot Weaving Gradual Change Welding Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:
    
    int TestWeave(void)
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
      DescPose p1Desc(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
      JointPos p1Joint(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);
      DescPose p2Desc(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
      JointPos p2Joint(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);
      ExaxisPos exaxisPos(0, 0, 0, 0);
      DescPose offdese(0, 0, 0, 0, 0, 0);
      robot.MoveJ(&p1Joint, &p1Desc, 13, 0, 20, 100, 100, &exaxisPos, -1, 0, &offdese);
      robot.WeaveStartSim(0);
      robot.MoveL(&p2Joint, &p2Desc, 13, 0, 20, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
      robot.WeaveEndSim(0);
      robot.MoveJ(&p1Joint, &p1Desc, 13, 0, 20, 100, 100, &exaxisPos, -1, 0, &offdese);
      robot.WeaveInspectStart(0);
      robot.MoveL(&p2Joint, &p2Desc, 13, 0, 20, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
      robot.WeaveInspectEnd(0);
      robot.WeldingSetVoltage(1, 19, 0, 0);
      robot.WeldingSetCurrent(1, 190, 0, 0);
      robot.MoveL(&p1Joint, &p1Desc, 1, 1, 100, 100, 50, -1, &exaxisPos, 0, 0, &offdese);
      robot.ARCStart(1, 0, 10000);
      robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
      robot.WeaveStart(0);
      robot.WeaveChangeStart(1, 0, 50, 30);
      robot.MoveL(&p2Joint, &p2Desc, 1, 1, 100, 100, 1, -1, &exaxisPos, 0, 0, &offdese);
      robot.WeaveChangeEnd();
      robot.WeaveEnd(0);
      robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
      robot.ARCEnd(1, 0, 10000);
      robot.CloseRPC();
      return 0;
    }

Weaving Gradual Change End
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.0-3.8.0

.. code-block:: c++
    :linenos:

	/**
	 * @brief  Weaving gradual change end
	 * @return  Error code
	 */
    errno_t WeaveChangeEnd();

Extended IO-Configure Welder Gas Detection Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Extended IO-Configure welder gas detection signal
     * @param [in] DONum Gas detection signal extended DO number
     * @return Error code
     */
    errno_t SetAirControlExtDoNum(int DONum);

Extended IO-Configure Welder Arc Start Signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Extended IO-Configure welder arc start signal
     * @param [in] DONum Welder arc start signal extended DO number
     * @return Error code
     */
    errno_t SetArcStartExtDoNum(int DONum);

Extended IO-Configure Welder Reverse Wire Feed Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Extended IO-Configure welder reverse wire feed signal
     * @param [in] DONum Reverse wire feed signal extended DO number
     * @return Error code
     */
    errno_t SetWireReverseFeedExtDoNum(int DONum);

Extended IO-Configure Welder Forward Wire Feed Signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Extended IO-Configure welder forward wire feed signal
     * @param [in] DONum Forward wire feed signal extended DO number
     * @return Error code
     */
    errno_t SetWireForwardFeedExtDoNum(int DONum);

Extended IO-Configure Welder Arc Success Signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Extended IO-Configure welder arc success signal
     * @param [in] DINum Arc success signal extended DI number
     * @return Error code
     */
    errno_t SetArcDoneExtDiNum(int DINum);

Extended IO-Configure Welder Ready Signal
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Extended IO-Configure welder ready signal
     * @param [in] DINum Welder ready signal extended DI number
     * @return Error code
     */
    errno_t SetWeldReadyExtDiNum(int DINum);

Extended IO-Configure Welding Interruption Recovery Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Extended IO-Configure welding interruption recovery signal
     * @param [in] reWeldDINum Resume welding after interruption signal extended DI number
     * @param [in] abortWeldDINum Abort welding after interruption signal extended DI number
     * @return Error code
     */
    errno_t SetExtDIWeldBreakOffRecover(int reWeldDINum, int abortWeldDINum);

Set Extended IO Welding Signal Code Example
+++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestExtDIConfig(void)
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
      robot.SetArcStartExtDoNum(10);
      robot.SetAirControlExtDoNum(20);
      robot.SetWireForwardFeedExtDoNum(30);
      robot.SetWireReverseFeedExtDoNum(40);
      robot.SetWeldReadyExtDiNum(50);
      robot.SetArcDoneExtDiNum(60);
      robot.SetExtDIWeldBreakOffRecover(70, 80);
      robot.SetWireSearchExtDIONum(0, 1);
      robot.CloseRPC();
      return 0;
    }

Arc Tracking Control
+++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

      /**
      * @brief  Arc tracking control
   	  * @param  [in] flag Switch, 0-Off; 1-On
  	  * @param  [in] dalayTime Lag time, unit ms
  	  * @param  [in] isLeftRight Left-right deviation compensation
  	  * @param  [in] klr Left-right adjustment coefficient (sensitivity);
  	  * @param  [in] tStartLr Left-right start compensation time cyc
  	  * @param  [in] stepMaxLr Left-right maximum compensation per cycle mm
  	  * @param  [in] sumMaxLr Left-right total maximum compensation mm
  	  * @param  [in] isUpLow Up-down deviation compensation
  	  * @param  [in] kud Up-down adjustment coefficient (sensitivity);
  	  * @param  [in] tStartUd Up-down start compensation time cyc
  	  * @param  [in] stepMaxUd Up-down maximum compensation per cycle mm
  	  * @param  [in] sumMaxUd Up-down total maximum compensation
  	  * @param  [in] axisSelect Up-down coordinate system selection, 0-Weaving; 1-Tool; 2-Base
  	  * @param  [in] referenceType Up-down reference current setting method, 0-Feedback; 1-Constant
  	  * @param  [in] referSampleStartUd Up-down reference current sampling start count (feedback);, cyc
  	  * @param  [in] referSampleCountUd Up-down reference current sampling cycle count (feedback);, cyc
  	  * @param  [in] referenceCurrent Up-down reference current mA
  	  * @param  [in] offsetType Offset tracking type, 0-No offset; 1-Sampling; 2-Percentage
  	  * @param  [in] offsetParameter Offset parameter; Sampling (offset sampling start time, default one cycle); Percentage (offset percentage (-100 ~ 100))
  	  * @return  Error code
  	  */
	 errno_t ArcWeldTraceControl(int flag, double delaytime, int isLeftRight, double klr, double tStartLr, double stepMaxLr, double sumMaxLr, int isUpLow, double kud, double tStartUd, double stepMaxUd, double sumMaxUd, int axisSelect, int referenceType, double referSampleStartUd, double referSampleCountUd, double referenceCurrent, int offsetType = 0, int offsetParameter = 0);

Set Arc Tracking Input Signal Port
++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Set arc tracking input signal port
      * @param  [in] channel Arc tracking AI channel selection,[0-3]
      * @return  Error code
      */
     errno_t ArcWeldTraceExtAIChannelConfig(int channel);


Arc Tracking + Multi-layer Multi-pass Compensation Start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Arc tracking + multi-layer multi-pass compensation start
    * @return Error code
    */
    errno_t ArcWeldTraceReplayStart();

Arc Tracking + Multi-layer Multi-pass Compensation End
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Arc tracking + multi-layer multi-pass compensation end
    * @return Error code
    */
    errno_t ArcWeldTraceReplayEnd();

Offset Coordinate Transformation - Multi-layer Multi-pass Welding
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Offset coordinate transformation - multi-layer multi-pass welding
    * @return Error code
    */
    errno_t MultilayerOffsetTrsfToBase(DescTran pointO, DescTran pointX, DescTran pointZ, double dx, double dy, double db, DescPose& offset);

Multi-layer Multi-pass Welding Arc Tracking Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestArcWeldTrace(void)
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
      JointPos mulitilineorigin1_joint(-24.090, -63.501, 84.288, -111.940, -93.426, 57.669);
      DescPose mulitilineorigin1_desc(-677.559, 190.951, -1.205, 1.144, -41.482, -82.577);
      DescTran mulitilineX1_desc;
      mulitilineX1_desc.x = -677.556;
      mulitilineX1_desc.y = 211.949;
      mulitilineX1_desc.z = -1.206;
      DescTran mulitilineZ1_desc;
      mulitilineZ1_desc.x = -677.564;
      mulitilineZ1_desc.y = 190.956;
      mulitilineZ1_desc.z = 19.817;
      JointPos mulitilinesafe_joint(-25.734, -63.778, 81.502, -108.975, -93.392, 56.021);
      DescPose mulitilinesafe_desc(-677.561, 211.950, 19.812, 1.144, -41.482, -82.577);
      JointPos mulitilineorigin2_joint(-29.743, -75.623, 101.241, -116.354, -94.928, 55.735);
      DescPose mulitilineorigin2_desc(-563.961, 215.359, -0.681, 2.845, -40.476, -87.443);
      DescTran mulitilineX2_desc;
      mulitilineX2_desc.x = -563.965;
      mulitilineX2_desc.y = 220.355;
      mulitilineX2_desc.z = -0.680;
      DescTran mulitilineZ2_desc;
      mulitilineZ2_desc.x = -563.968;
      mulitilineZ2_desc.y = 215.362;
      mulitilineZ2_desc.z = 4.331;
      ExaxisPos epos(0, 0, 0, 0);
      DescPose offset(0, 0, 0, 0, 0, 0);
      robot.Sleep(10);
      int error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
      printf("MoveJ return: %d\n", error);
      error = robot.MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
      printf("MoveL return: %d\n", error);
      error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
      printf("MoveJ return: %d\n", error);
      error = robot.MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
      printf("MoveL return: %d\n", error);
      error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
      printf("MoveJ return: %d\n", error);
      error = robot.MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
      printf("MoveL return: %d\n", error);
      error = robot.ARCStart(1, 0, 3000);
      printf("ARCStart return: %d\n", error);
      error = robot.WeaveStart(0);
      printf("WeaveStart return: %d\n", error);
      error = robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10);
      printf("ArcWeldTraceControl return: %d\n", error);
      error = robot.MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 1, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
      printf("MoveL return: %d\n", error);
      error = robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10);
      printf("ArcWeldTraceControl return: %d\n", error);
      error = robot.WeaveEnd(0);
      printf("WeaveEnd return: %d\n", error);
      error = robot.ARCEnd(1, 0, 10000);
      printf("ARCEnd return: %d\n", error);
      error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
      printf("MoveJ return: %d\n", error);
      error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 10.0, 0.0, 0.0, offset);
      printf("MultilayerOffsetTrsfToBase return: %d offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);
      error = robot.MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
      printf("MoveL return: %d\n", error);
      error = robot.ARCStart(1, 0, 3000);
      printf("ARCStart return: %d\n", error);
      error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 10, 0, 0, offset);
      printf("MultilayerOffsetTrsfToBase return: %d offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);
      error = robot.ArcWeldTraceReplayStart();
      printf("ArcWeldTraceReplayStart return: %d\n", error);
      error = robot.MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 2, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
      printf("MoveL return: %d\n", error);
      error = robot.ArcWeldTraceReplayEnd();
      printf("ArcWeldTraceReplayEnd return: %d\n", error);
      error = robot.ARCEnd(1, 0, 10000);
      printf("ARCEnd return: %d\n", error);
      error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
      printf("MoveJ return: %d\n", error);
      error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 0, 10, 0, offset);
      printf("MultilayerOffsetTrsfToBase return: %d offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);
      error = robot.MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
      printf("MoveL return: %d\n", error);
      error = robot.ARCStart(1, 0, 3000);
      printf("ARCStart return: %d\n", error);
      error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 0, 10, 0, offset);
      printf("MultilayerOffsetTrsfToBase return: %d offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);
      error = robot.ArcWeldTraceReplayStart();
      printf("MoveJ return: %d\n", error);
      error = robot.MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 2, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
      printf("MoveL return: %d\n", error);
      error = robot.ArcWeldTraceReplayEnd();
      printf("ArcWeldTraceReplayEnd return: %d\n", error);
      error = robot.ARCEnd(1, 0, 3000);
      printf("ARCEnd return: %d\n", error);
      error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
      printf("MoveJ return: %d\n", error);
      robot.CloseRPC();
      return 0;
    }

Arc Tracking Welder Current Feedback AI Channel Selection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Arc tracking welder current feedback AI channel selection
     * @param [in]  channel Channel; 0-Extended AI0; 1-Extended AI1; 2-Extended AI2; 3-Extended AI3; 4-Control box AI0; 5-Control box AI1
     * @return Error code
     */
     errno_t ArcWeldTraceAIChannelCurrent(int channel);

Arc Tracking Welder Voltage Feedback AI Channel Selection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Arc tracking welder voltage feedback AI channel selection
     * @param [in]  channel Channel; 0-Extended AI0; 1-Extended AI1; 2-Extended AI2; 3-Extended AI3; 4-Control box AI0; 5-Control box AI1
     * @return Error code
     */
     errno_t ArcWeldTraceAIChannelVoltage(int channel);

Arc Tracking Welder Current Feedback Conversion Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

     /**
      * @brief Arc tracking welder current feedback conversion parameters
      * @param [in] AILow AI channel lower limit, default 0V, range [0-10V]
      * @param [in] AIHigh AI channel upper limit, default 10V, range [0-10V]
      * @param [in] currentLow AI channel lower limit corresponding welder current value, default 0V, range [0-200V]
      * @param [in] currentHigh AI channel upper limit corresponding welder current value, default 100V, range [0-200V]
      * @return Error code
      */
     errno_t ArcWeldTraceCurrentPara(float AILow, float AIHigh, float currentLow, float currentHigh);

Arc Tracking Welder Voltage Feedback Conversion Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

     /**
    * @brief Arc tracking welder voltage feedback conversion parameters
    * @param [in] AILow AI channel lower limit, default 0V, range [0-10V]
    * @param [in] AIHigh AI channel upper limit, default 10V, range [0-10V]
    * @param [in] voltageLow AI channel lower limit corresponding welder voltage value, default 0V, range [0-200V]
    * @param [in] voltageHigh AI channel upper limit corresponding welder voltage value, default 100V, range [0-200V]
    * @return Error code
    */
    errno_t ArcWeldTraceVoltagePara(float AILow, float AIHigh, float voltageLow, float voltageHigh);

Arc Tracking Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int WeldTraceControlWithCtrlBoxAI(FRRobot* robot)
    {
      DescPose startdescPose = { -473.86, 257.879, -20.849, -37.317, -42.021, 2.543 };
      JointPos startjointPos = { -43.487, -76.526, 95.568, -104.445, -89.356, 3.72 };

      DescPose enddescPose = { -499.844, 141.225, 7.72, -34.856, -40.17, 13.13 };
      JointPos endjointPos = { -31.305, -82.998, 99.401, -104.426, -89.35, 3.696 };

      DescPose safedescPose = { -504.043, 275.181, 40.908, -28.002, -42.025, -14.044 };
      JointPos safejointPos = { -39.078, -76.732, 87.227, -99.47, -94.301, 18.714 };

      ExaxisPos exaxisPos = { 0, 0, 0, 0 };
      DescPose offdese = { 0, 0, 0, 0, 0, 0 };

      robot->WeldingSetCurrentRelation(0, 495, 1, 10, 0);
      robot->WeldingSetVoltageRelation(10, 45, 1, 10, 1);

      robot->WeldingSetVoltage(0, 25, 1, 0);// ----Set voltage
      robot->WeldingSetCurrent(0, 260, 0, 0);// ----Set current

      int rtn = robot->ArcWeldTraceAIChannelCurrent(4);
      cout << "ArcWeldTraceAIChannelCurrent rtn is " << rtn << endl;
      rtn = robot->ArcWeldTraceAIChannelVoltage(5);
      cout << "ArcWeldTraceAIChannelVoltage rtn is " << rtn << endl;
      rtn = robot->ArcWeldTraceCurrentPara(0, 5, 0, 500);
      cout << "ArcWeldTraceCurrentPara rtn is " << rtn << endl;
      rtn = robot->ArcWeldTraceVoltagePara(1.018, 10, 0, 50);
      cout << "ArcWeldTraceVoltagePara rtn is " << rtn << endl;
      robot->MoveJ(&safejointPos, &safedescPose, 1, 0, 5, 100, 100, &exaxisPos, -1, 0, &offdese);
      robot->MoveJ(&startjointPos, &startdescPose, 1, 0, 5, 100, 100, &exaxisPos, -1, 0, &offdese);
      rtn = robot->ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
      cout << "ArcWeldTraceControl rtn is " << rtn << endl;
      robot->ARCStart(0, 0, 10000);
      robot->WeaveStart(0);
      robot->MoveL(&endjointPos, &enddescPose, 1, 0, 100, 100, 2, -1, &exaxisPos, 0, 0, &offdese);
      robot->ARCEnd(0, 0, 10000);
      robot->WeaveEnd(0);
      robot->ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
      return 0;
    }

Wire Search Start
+++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief  Wire search start
    * @param  [in] refPos  1-Reference point 2-Contact point
    * @param  [in] searchVel   Search speed %
    * @param  [in] searchDis  Search distance mm
    * @param  [in] autoBackFlag Auto return flag, 0-No auto; -Auto
    * @param  [in] autoBackVel  Auto return speed %
    * @param  [in] autoBackDis  Auto return distance mm
    * @param  [in] offectFlag  1-Search with offset; 2-Teach point search
    * @return  Error code
    */
     errno_t WireSearchStart(int refPos, float searchVel, int searchDis, int autoBackFlag, float autoBackVel, int autoBackDis, int offectFlag);

Wire Search End
+++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

     /**
      * @brief  Wire search end
      * @param  [in] refPos  1-Reference point 2-Contact point
      * @param  [in] searchVel   Search speed %
      * @param  [in] searchDis  Search distance mm
      * @param  [in] autoBackFlag Auto return flag, 0-No auto; -Auto
      * @param  [in] autoBackVel  Auto return speed %
      * @param  [in] autoBackDis  Auto return distance mm
      * @param  [in] offectFlag  1-Search with offset; 2-Teach point search
      * @return  Error code
      */
     errno_t WireSearchEnd(int refPos, float searchVel, int searchDis, int autoBackFlag, float autoBackVel, int autoBackDis, int offectFlag);

Calculate Wire Search Offset
++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

     /**
      * @brief  Calculate wire search offset
      * @param  [in] seamType  Weld seam type
      * @param  [in] method   Calculation method
      * @param  [in] varNameRef Reference point 1-6, "#" means no point variable
      * @param  [in] varNameRes Contact point 1-6, "#" means no point variable
      * @param  [out] offectFlag 0-Offset directly added to command point; 1-Offset requires coordinate transformation of command point
      * @param  [out] offect Offset pose [x, y, z, a, b, c]
      * @return  Error code
      */
     errno_t GetWireSearchOffset(int seamType, int method, std::vector<std::string> varNameRef, std::vector<std::string> varNameRes, int& offectFlag, DescPose& offect);

Wait for Wire Search Completion
++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

     /**
      * @brief  Wait for wire search completion
      * @return  Error code
      */
     errno_t WireSearchWait(std::string varName);

Write Wire Search Contact Point to Database
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Write wire search contact point to database
      * @param  [in] varName  Contact point name "RES0" ~ "RES99"
      * @param  [in] pos  Contact point data [x, y, x, a, b, c]
      * @return  Error code
      */
     errno_t SetPointToDatabase(std::string varName, DescPose pos);

Robot Wire Search Code Example
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int TestWireSearch(void)
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
      DescPose toolCoord(0, 0, 200, 0, 0, 0);
      robot.SetToolCoord(1, &toolCoord, 0, 0, 1, 0);
      DescPose wobjCoord(0, 0, 0, 0, 0, 0);
      robot.SetWObjCoord(1, &wobjCoord, 0);
      int rtn0, rtn1, rtn2 = 0;
      ExaxisPos exaxisPos = { 0, 0, 0, 0 };
      DescPose offdese = { 0, 0, 0, 0, 0, 0 };
      DescPose descStart = { 216.543, 445.175, 93.465, 179.683, 1.757, -112.641 };
      JointPos jointStart = { -128.345, -86.660, 114.679, -119.625, -89.219, 74.303 };
      DescPose descEnd = { 111.143, 523.384, 87.659, 179.703, 1.835, -97.750 };
      JointPos jointEnd = { -113.454, -81.060, 109.328, -119.954, -89.218, 74.302 };
      robot.MoveL(&jointStart, &descStart, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
      robot.MoveL(&jointEnd, &descEnd, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
      DescPose descREF0A = { 142.135, 367.604, 86.523, 179.728, 1.922, -111.089 };
      JointPos jointREF0A = { -126.794, -100.834, 128.922, -119.864, -89.218, 74.302 };
      DescPose descREF0B = { 254.633, 463.125, 72.604, 179.845, 2.341, -114.704 };
      JointPos jointREF0B = { -130.413, -81.093, 112.044, -123.163, -89.217, 74.303 };
      DescPose descREF1A = { 92.556, 485.259, 47.476, -179.932, 3.130, -97.512 };
      JointPos jointREF1A = { -113.231, -83.815, 119.877, -129.092, -89.217, 74.303 };
      DescPose descREF1B = { 203.103, 583.836, 63.909, 179.991, 2.854, -103.372 };
      JointPos jointREF1B = { -119.088, -69.676, 98.692, -121.761, -89.219, 74.303 };
      rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
      robot.MoveL(&jointREF0A, &descREF0A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese); //Start point
      robot.MoveL(&jointREF0B, &descREF0B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese); //Direction point
      rtn1 = robot.WireSearchWait("REF0");
      rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
      rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
      robot.MoveL(&jointREF1A, &descREF1A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese); //Start point
      robot.MoveL(&jointREF1B, &descREF1B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese); //Direction point
      rtn1 = robot.WireSearchWait("REF1");
      rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
      rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
      robot.MoveL(&jointREF0A, &descREF0A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese); //Start point
      robot.MoveL(&jointREF0B, &descREF0B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese); //Direction point
      rtn1 = robot.WireSearchWait("RES0");
      rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
      rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
      robot.MoveL(&jointREF1A, &descREF1A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese); //Start point
      robot.MoveL(&jointREF1B, &descREF1B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese); //Direction point
      rtn1 = robot.WireSearchWait("RES1");
      rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
      vector <string> varNameRef = { "REF0", "REF1", "#", "#", "#", "#" };
      vector <string> varNameRes = { "RES0", "RES1", "#", "#", "#", "#" };
      int offectFlag = 0;
      DescPose offectPos = { 0, 0, 0, 0, 0, 0 };
      rtn0 = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes, offectFlag, offectPos);
      robot.PointsOffsetEnable(0, &offectPos);
      robot.MoveL(&jointStart, &descStart, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
      robot.MoveL(&jointEnd, &descEnd, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);
      robot.PointsOffsetDisable();
      robot.CloseRPC();
      return 0;

Set Welding Voltage Gradual Change Start
+++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

     /**
    * @brief Set welding voltage gradual change start
    * @param [in] IOType Control type; 0-Control box IO; 1-Digital communication protocol (UDP);2-Digital communication protocol (ModbusTCP)
    * @param [in] voltageStart Starting welding voltage (V)
    * @param [in] voltageEnd Ending welding voltage (V)
    * @param [in] AOIndex Control box AO port number (0-1)
    * @param [in] blend Whether to smooth 0-No smoothing; 1-Smooth
    * @return Error code
    */
    errno_t WeldingSetVoltageGradualChangeStart(int IOType, double voltageStart, double voltageEnd, int AOIndex, int blend);

Set Welding Voltage Gradual Change End
+++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

     /**
      * @brief Set welding voltage gradual change end
      * @return Error code
      */
     errno_t WeldingSetVoltageGradualChangeEnd();

Set Welding Current Gradual Change Start
+++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

     /**
      * @brief Set welding current gradual change start
      * @param [in] IOType Control type; 0-Control box IO; 1-Digital communication protocol (UDP);2-Digital communication protocol (ModbusTCP)
      * @param [in] voltageStart Starting welding current (A)
      * @param [in] voltageEnd Ending welding current (A)
      * @param [in] AOIndex Control box AO port number (0-1)
      * @param [in] blend Whether to smooth 0-No smoothing; 1-Smooth
      * @return Error code
      */
     errno_t WeldingSetCurrentGradualChangeStart(int IOType, double currentStart, double currentEnd, int AOIndex, int blend);

Set Welding Current Gradual Change End
+++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Set welding current gradual change end
     * @return Error code
     */
    errno_t WeldingSetCurrentGradualChangeEnd();
    
Robot Welding Current Voltage Gradual Change Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int WeldparamChange(FRRobot* robot)
    {
      DescPose startdescPose = { -484.707, 276.996, -14.013, -37.657, -40.508, -1.548 };
      JointPos startjointPos = { -45.421, -75.673, 93.627, -104.302, -87.938, 6.005 };
      
      DescPose enddescPose = { -508.767, 137.109, -13.966, -37.639, -40.508, -1.559 };
      JointPos endjointPos = { -32.768, -80.947, 100.254, -106.201, -87.201, 18.648 };

      DescPose safedescPose = { -484.709, 294.436, 13.621, -37.660, -40.508, -1.545 };
      JointPos safejointPos = { -46.604, -75.410, 89.109, -100.003, -88.012, 4.823 };
      ExaxisPos exaxisPos = { 0, 0, 0, 0 };
      DescPose offdese = { 0, 0, 0, 0, 0, 0 };

      robot->WeldingSetCurrentRelation(0, 495, 1, 10, 0);
      robot->WeldingSetVoltageRelation(10, 45, 1, 10, 1);
      robot->MoveJ(&safejointPos, &safedescPose, 1, 0, 5, 100, 100, &exaxisPos, -1, 0, &offdese);
      int rtn = robot->WeldingSetCurrentGradualChangeStart(0, 260, 220, 0, 0);
      cout << "WeldingSetCurrentGradualChangeStart rtn is " << rtn << endl;
      rtn = robot->WeldingSetVoltageGradualChangeStart(0, 25, 22, 1, 0);
      cout << "WeldingSetVoltageGradualChangeStart rtn is " << rtn << endl;
      rtn = robot->ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
      cout << "ArcWeldTraceControl rtn is " << rtn << endl;
      robot->MoveJ(&startjointPos, &startdescPose, 1, 0, 5, 100, 100, &exaxisPos, -1, 0, &offdese);
      
      robot->ARCStart(0, 0, 10000);
      robot->WeaveStart(0);
      robot->WeaveChangeStart(2, 1, 24, 36);
      robot->MoveL(&endjointPos, &enddescPose, 1, 0, 100, 100, 2, -1, &exaxisPos, 0, 0, &offdese);
      robot->ARCEnd(0, 0, 10000);
      robot->WeaveChangeEnd();
      robot->WeaveEnd(0);
      robot->ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
      robot->WeldingSetCurrentGradualChangeEnd();
      robot->WeldingSetVoltageGradualChangeEnd();
      return 0;
    }