Robotic Welding
=====================

.. toctree:: 
    :maxdepth: 5

Setting the welding process parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Sets the welding process curve parameters.
    * @param [in] id Welding process number (1-99)
    * @param [in] startCurrent startCurrent(A)
    * @param [in] startVoltage startVoltage(V)
    * @param [in] startTime startTime(ms)
    * @param [in] weldCurrent weldCurrent(A)
    * @param [in] weldVoltage Welding voltage (V)
    * @param [in] endCurrent Welding current(A)
    * @param [in] endVoltage weldVoltage(V)
    * @param [in] endTime Arc recovery time (ms)
    * @return Error code
    */
    int WeldingSetProcessParam(int id, double startCurrent, double startVoltage, double startTime, double weldCurrent, double weldVoltage, double endCurrent, double endVoltage, double endTime);

Get weld process curve parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get weld process curve parameters.
    * @param [in] id Welding process number (1-99)
    * @param [out] startCurrent startCurrent (A)
    * @param [out] startVoltage startVoltage(V)
    * @param [out] startTime startTime(ms)
    * @param [out] weldCurrent weldCurrent(A)
    * @param [out] weldVoltage Welding voltage (V)
    * @param [out] endCurrent Welding current(A)
    * @param [out] endVoltage weldVoltage(V)
    * @param [out] endTime Arc recovery time (ms)
    * @return Error code
    */
    int WeldingGetProcessParam(int id, ref double startCurrent, ref double startVoltage, ref double startTime, ref double weldCurrent, ref double weldVoltage, ref double endCurrent, ref double endVoltage, ref double endTime);

Setting the weld current in relation to the output analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set weld current to output analog.
    * @param [in] currentMin Welding current-analog output linear relationship left point current value (A)
    * @param [in] currentMax Welding current - analog output linear relationship right point current value (A)
    * @param [in] outputVoltageMin Welding current - analog output linear relationship left point analog output voltage value (V)
    * @param [in] outputVoltageMax Welding current - analog output linear relationship of the right point analog output voltage value (V)
    * @return Error Code
    */
    int WeldingSetCurrentRelation(double currentMin, double currentMax, double outputVoltageMin, double outputVoltageMax);

Set Welding Voltage and Output Analog Relation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set weld voltage to output analog.
    * @param [in] weldVoltageMin weldVoltageMin Welding voltage-analog output linear relationship between the left point weld voltage value (A)
    * @param [in] weldVoltageMax Welding voltage - analog output linear relationship between the right point welding voltage value (A)
    * @param [in] outputVoltageMin Welding voltage-analog output linear relationship left point analog output voltage value (V)
    * @param [in] outputVoltageMax Welding voltage - analog output linear relationship of the right point analog output voltage value (V)
    * @return Error Code
    */
    int WeldingSetVoltageRelation(double weldVoltageMin, double weldVoltageMax, double outputVoltageMin, double outputVoltageMax);

Get the relationship between the welding current and the output analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Get weld current to output analog.
    * @param [out] currentMin Welding current-analog output linear relationship of the left point current value (A).
    * @param [out] currentMax Welding current - analog output linear relationship right point current value (A)
    * @param [out] outputVoltageMin Welding current - analog output linear relationship left point analog output voltage value (V)
    * @param [out] outputVoltageMax Welding current - analog output linear relationship between the right point analog output voltage value (V)
    * @return Error Code
    */
    int WeldingGetCurrentRelation(ref double currentMin, ref double currentMax, ref double outputVoltageMin, ref double outputVoltageMax);

Get Welding Voltage Relation to Output Analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Get weld voltage to output analog
    * @param [out] weldVoltageMin Welding Voltage - Analog Output Linear Relationship Left Point Welding Voltage Value (A)
    * @param [out] weldVoltageMax Welding Voltage - Analog Output Linear Relationship Welding Voltage Value (A) at the Right Point
    * @param [out] outputVoltageMin Welding voltage-analog output linear relationship left point analog output voltage value (V)
    * @param [out] outputVoltageMax Welding voltage - analog output linear relationship right point analog output voltage value (V)
    * @return Error Code
    */
    int WeldingGetVoltageRelation(ref double weldVoltageMin, ref double weldVoltageMax, ref double outputVoltageMin, ref double outputVoltageMax);

Set weld current
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set weld current.
    * @param [in] ioType Control IO type 0-control box IO; 1-expansion IO
    * @param [in] current Welding current value (A)
    * @param [in] AOIndex Welding current control box analog output port (0-1)
    * @return Error Code
    */
    int WeldingSetCurrent(int ioType, double current, int AOIndex);

Set Welding Voltage
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set the solder voltage.
    * @param [in] ioType Control IO type 0-Control box IO; 1-Extension IO
    * @param [in] voltage Welding voltage value (A)
    * @param [in] AOIndex Welding voltage control box analog output port (0-1)
    * @return Error Code
    */
    int WeldingSetVoltage(int ioType, double voltage, int AOIndex);

Set the swing parameter
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set the swing parameter
    * @param [in] weaveNum The configuration number of the weave parameter.
    * @param [in] weaveType Oscillation type 0-planar triangular oscillation; 1-vertical L-shaped triangular oscillation; 2-clockwise circular oscillation; 3-counterclockwise circular oscillation; 4-planar sinusoidal oscillation; 5-vertical L-shaped sinusoidal oscillation; 6-vertical triangular oscillation; 7-vertical sine wave oscillation
    * @param [in] weaveFrequency swing frequency (Hz)
    * @param [in] weaveIncStayTime wait mode 0-cycle without wait time; 1-cycle with wait time
    * @param [in] weaveRange Weave range (mm)
    * @param [in] weaveLeftRange vertical triangle swing left chord length (mm)
    * @param [in] weaveRightRange the length of the right chord of the vertical triangle swing (mm)
    * @param [in] additionalStayTime The stay time of the vertical triangle (mm).
    * @param [in] weaveLeftStayTime weaveLeftStayTime (ms)
    * @param [in] weaveRightStayTime weaveRightStayTime (ms)
    * @param [in] weaveCircleRadio Circle swing-back ratio (0-100%)
    * @param [in] weaveStationary swing position wait, 0 - wait time for position to continue moving; 1 - wait time for position to be stationary
    * @param [in] weaveYawAngle swing direction azimuth (rotation around swing Z-axis), in °.
    * @return Error code 
    */
    int WeaveSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, double weaveLeftRange, double weaveRightRange, int additionalStayTime, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary, double weaveYawAngle, double weaveRotAngle=0);

Sample code for setting welding parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V3.9.8
    
.. code-block:: c#
    :linenos:

    private void button42_Click(object sender, EventArgs e)
    {
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

        robot.WeldingGetProcessParam(1, ref startCurrent, ref startVoltage, ref startTime, ref weldCurrent, ref weldVoltage, ref endCurrent, ref endVoltage, ref endTime);
        Console.WriteLine("the Num 1 process param is " + startCurrent + " " + startVoltage + " " + startTime + " " + weldCurrent + " " + weldVoltage + " " + endCurrent + " " + endVoltage + " " + endTime);
        robot.WeldingGetProcessParam(2, ref startCurrent, ref startVoltage, ref startTime, ref weldCurrent, ref weldVoltage, ref endCurrent, ref endVoltage, ref endTime);
        Console.WriteLine("the Num 2 process param is " + startCurrent + " " + startVoltage + " " + startTime + " " + weldCurrent + " " + weldVoltage + " " + endCurrent + " " + endVoltage + " " + endTime);

        int rtn = robot.WeldingSetCurrentRelation(0, 400, 0, 10, 0);
        Console.WriteLine("WeldingSetCurrentRelation rtn is: " + rtn);

        rtn = robot.WeldingSetVoltageRelation(0, 40, 0, 10, 1);
        Console.WriteLine("WeldingSetVoltageRelation rtn is: " + rtn);

        double current_min = 0;
        double current_max = 0;
        double vol_min = 0;
        double vol_max = 0;
        double output_vmin = 0;
        double output_vmax = 0;
        int curIndex = 0;
        int volIndex = 0;
        rtn = robot.WeldingGetCurrentRelation(ref current_min, ref current_max, ref output_vmin, ref output_vmax, ref curIndex);
        Console.WriteLine("WeldingGetCurrentRelation rtn is: " + rtn);
        Console.WriteLine("current min " + current_min + " current max " + current_max + " output vol min " + output_vmin + " output vol max " + output_vmax);

        rtn = robot.WeldingGetVoltageRelation(ref vol_min, ref vol_max, ref output_vmin, ref output_vmax, ref volIndex);
        Console.WriteLine("WeldingGetVoltageRelation rtn is: " + rtn);
        Console.WriteLine("vol min " + vol_min + " vol max " + vol_max + " output vol min " + output_vmin + " output vol max " + output_vmax);

        rtn = robot.WeldingSetCurrent(1, 100, 0, 0);
        Console.WriteLine("WeldingSetCurrent rtn is: " + rtn);

        System.Threading.Thread.Sleep(3000);

        rtn = robot.WeldingSetVoltage(1, 10, 0, 0);
        Console.WriteLine("WeldingSetVoltage rtn is: " + rtn);

        rtn = robot.WeaveSetPara(0, 0, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 60.000000);
        Console.WriteLine("rtn is: " + rtn);

        robot.WeaveOnlineSetPara(0, 0, 1, 0, 20, 0, 0, 0, 0);

        rtn = robot.WeldingSetCheckArcInterruptionParam(1, 200);
        Console.WriteLine("WeldingSetCheckArcInterruptionParam    " + rtn);
        rtn = robot.WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0);
        Console.WriteLine("WeldingSetReWeldAfterBreakOffParam    " + rtn);
        int enable = 0;
        double length = 0;
        double velocity = 0;
        int moveType = 0;
        int checkEnable = 0;
        int arcInterruptTimeLength = 0;
        rtn = robot.WeldingGetCheckArcInterruptionParam(ref checkEnable, ref arcInterruptTimeLength);
        Console.WriteLine("WeldingGetCheckArcInterruptionParam  checkEnable  " + checkEnable + "   arcInterruptTimeLength  " + arcInterruptTimeLength);
        rtn = robot.WeldingGetReWeldAfterBreakOffParam(ref enable, ref length, ref velocity, ref moveType);
        Console.WriteLine("WeldingGetReWeldAfterBreakOffParam  enable = " + enable + ", length = " + length + ", velocity = " + velocity + ", moveType = " + moveType);

        robot.SetWeldMachineCtrlModeExtDoNum(17);
        for (int i = 0; i < 5; i++)
        {
            int getCtrlMode = -1;
            robot.SetWeldMachineCtrlMode(0);
            robot.GetWeldMachineCtrlMode(ref getCtrlMode);
            Console.WriteLine("GetWeldMachineCtrlMode {0}", getCtrlMode);
            Thread.Sleep(1000);
            robot.SetWeldMachineCtrlMode(1);
            robot.GetWeldMachineCtrlMode(ref getCtrlMode);
            Console.WriteLine("GetWeldMachineCtrlMode {0}", getCtrlMode);
            Thread.Sleep(1000);
        }
    }

Setting the swing parameters on the fly
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Instant setup of swing parameters
    * @param [in] weaveNum The configuration number of the weave parameter.
    * @param [in] weaveType Oscillation type 0-planar triangular oscillation; 1-vertical L-shaped triangular oscillation; 2-clockwise circular oscillation; 3-counterclockwise circular oscillation; 4-planar sinusoidal oscillation; 5-vertical L-shaped sinusoidal oscillation; 6-vertical triangular oscillation; 7-vertical sinusoidal oscillation
    * @param [in] weaveFrequency swing frequency (Hz)
    * @param [in] weaveIncStayTime wait mode 0-cycle without wait time; 1-cycle with wait time
    * @param [in] weaveRange Weave range (mm)
    * @param [in] weaveLeftStayTime weaveLeftStayTime (ms)
    * @param [in] weaveRightStayTime weaveRightStayTime(ms)
    * @param [in] weaveCircleRadio Circle swing-back ratio (0-100%)
    * @param [in] weaveStationary swing position wait, 0 - wait time for position to continue moving; 1 - wait time for position to be stationary
    * @return error code
    */
    int WeaveOnlineSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary).

Set parameters for detecting unexpected interruptions of the robot's welding arc
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting parameters for detecting unexpected interruptions of a robotic welding arc.
    * @param [in] checkEnable whether or not to enable detection; 0-don't enable; 1-enable
    * @param [in] arcInterruptTimeLength arcInterruptAcknowledgmentTimeLength(ms)
    * @return Error code
    */
    int WeldingSetCheckArcInterruptionParam(int checkEnable, int arcInterruptTimeLength)

Get parameters for detecting accidental interruptions of the robot's welding arc
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get parameters for detecting unexpected interruptions of a robotic welding arc.
    * @param [out] checkEnable Whether to enable detection; 0-don't enable; 1-enable
    * @param [out] arcInterruptTimeLength arcInterruptAcknowledgmentTimeLength(ms)
    * @return Error code
    */
    int WeldingGetCheckArcInterruptionParam(ref int checkEnable, ref int arcInterruptTimeLength)

Set the robot welding interruption recovery parameter
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the robot weld interrupt recovery parameters
    * @param[in] enable Whether to enable weld interrupt recovery or not.
    * @param[in] length The overlap distance of the weld (mm).
    * @param[in] velocity the speed of the robot to return to the restart point (0-100)
    * @param[in] moveType Robot movement to restart point type; 0-LIN; 1-PTP
    * @return Error code
    */
    int WeldingSetReWeldAfterBreakOffParam(int enable, double length, double velocity, int moveType)

Get the robot welding break recovery parameter
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get robot weld interrupt recovery parameters.
    * @param [out] enable Whether to enable weld interrupt recovery.
    * @param [out] length The overlap distance of the weld (mm).
    * @param [out] velocity the speed of the robot to return to the restart point (0-100)
    * @param [out] moveType the way the robot moves to the restart point; 0-LIN; 1-PTP
    * @return Error code
    */
    int WeldingGetReWeldAfterBreakOffParam(ref int enable, ref double length, ref double velocity, ref int moveType)

Set welder control mode extension DO port
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set welder control mode extended DO port.
    * @param DONum Welder control mode DO port (0-127)
    * @return Error code
    */
    int SetWeldMachineCtrlModeExtDoNum(int DONum).

Set Welding Machine Control Mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set welding machine control mode
    * @param [in] mode Welding machine control mode; 0-DC one-knob mode; 1-Pulse one-knob mode; 2-JOB mode; 3-Local control mode; 4-Separate mode; 5-CC/CV mode; 6-TIG; 7-CMT
    * @param [in] ioType Control type; 0-Control box IO; 1-Digital communication protocol (UDP); 2-Digital communication protocol (ModbusTCP)
    * @return Error code
    */
    public int SetWeldMachineCtrlMode(int mode,int ioType = 1)

Get Welding Machine Control Mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get welding machine control mode
    * @param [out] mode Welding machine control mode; 0-DC one-knob mode; 1-pulse one-knob mode; 2-JOB mode; 3-local control mode; 4-separate mode; 5-CC/CV mode; 6-TIG; 7-CMT
    * @return Error code
    */
    public int GetWeldMachineCtrlMode(ref int mode)

Weld start
++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Welding start
    * @param [in] ioType io type 0-controller IO; 1-extended IO
    * @param [in] arcNum Welder profile number
    * @param [in] timeout Arc start timeout
    * @return Error code
    */
    int ARCStart(int ioType, int arcNum, int timeout);

Weld End
++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief End of welding.
    * @param [in] ioType io type 0-controller IO; 1-extended IO
    * @param [in] arcNum Welder profile number
    * @param [in] timeout Arc-off timeout timeout
    * @return Error code
    */
    int ARCEnd(int ioType, int arcNum, int timeout).

Oscillation start
++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief The start of the swing.
    * @param [in] weaveNum Pendulum weld parameter configuration number
    * @return ErrorCode
    */
    int WeaveStart(int weaveNum).

WeaveStart(int weaveNum); int WeaveStart(int weaveNum).
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief End of swing
    * @param [in] weaveNum Pendulum weld parameter configuration number
    * @return ErrorCode
    */
    int WeaveEnd(int weaveNum).

Forward Wire Feed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Forward wire feed
    * @param [in] ioType io type 0-controller IO; 1-extended IO
    * @param [in] wireFeed wire feed control 0-stop wire feed; 1-wire feed
    * @return Error code
    */
    int SetForwardWireFeed(int ioType, int wireFeed).  

Reverse wire feed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Reverse wire feed
    * @param [in] ioType ioType 0-Controller IO; 1-Extended IO
    * @param [in] wireFeed wireFeedControl 0-Stop wire feed; 1-Wire feed
    * @return Error code
    */
    int SetReverseWireFeed(int ioType, int wireFeed).

Wire Feed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief air delivery
    * @param [in] ioType io type 0-controller IO; 1-extended IO
    * @param [in] airControl airControl 0-stop air delivery; 1-feed air
    * @return Error code
    */
    int SetAspirated(int ioType, int airControl).

Setting the robot to resume welding after an interruption
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the robot to resume welding after a weld interruption.
    * @return Error code
    */
    int WeldingStartReWeldAfterBreakOff()

Sets the robot to exit welding after a weld break
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the robot to quit welding after a weld interruption.
    * @return Error code
    */
    int WeldingAbortWeldAfterBreakOff()

Code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
   :linenos:

   private void button7_Click(object sender, EventArgs e)
   {
      robot.WeldingSetCurrent(1, 230, 0, 0);
      robot.WeldingSetVoltage(1, 24, 0, 1);

      DescPose p1Desc = new DescPose(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
      JointPos p1Joint = new JointPos(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);

      DescPose p2Desc = new DescPose(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
      JointPos p2Joint = new JointPos(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);

      ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
      DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

      robot.MoveJ(p1Joint, p1Desc, 13, 0, 20, 100, 100, exaxisPos, -1, 0, offdese);
      robot.ARCStart(1, 0, 10000);
      robot.WeaveStart(0);
      robot.MoveL (p2Joint, p2Desc, 13, 0, 20, 100, 100, -1, 0, exaxisPos, 0, 0, offdese);
      robot.ARCEnd(1, 0, 10000);
      robot.WeaveEnd(0);
   }

Start of segment welding
++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /** 
    * @brief Start of segment welding
    * @param [in] startDesePos start point Cartesian position
    * @param [in] endDesePos end point Cartesian position
    * @param [in] startJPos start point joint position
    * @param [in] endJPos end point joint position
    * @param [in] weldLength weldLength (mm)
    * @param [in] noWeldLength length (mm)
    * @param [in] weldIOType Weld IO type (0-control box IO; 1-expansion IO)
    * @param [in] arcNum Welder profile number
    * @param [in] weldTimeout start/recovery arc timeout time
    * @param [in] isWeave whether or not to wiggle
    * @param [in] weaveNum weave parameter configuration number
    * @param [in] tool tool number
    * @param [in] user Workpiece number
    * @param [in] vel velocity percentage, range [0~100] * @param [in] acc.
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm    
    * @param [in] epos Extended axis position in mm
    * @param [in] search 0-no wire seek, 1-wire seek
    * @param [in] offset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos Position offset
    * @return Error code 
    */
    int SegmentWeldStart(DescPose startDesePos, DescPose endDesePos, JointPos startJPos, JointPos endJPos, double weldLength, double noWeldLength, int weldIOType, int arcNum, int weldTimeout,bool isWeave, int weaveNum, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, byte search, byte offset_flag, DescPose offset_pos);

Robot segment welding code example
++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
   :linenos:

   private void btnWeldStart_Click(object sender, EventArgs e)
   {
      robot.WeldingSetCurrent(1, 230, 0, 0);
      robot.WeldingSetVoltage(1, 24, 0, 1);

      DescPose p1Desc = new DescPose(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
      JointPos p1Joint = new JointPos(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);

      DescPose p2Desc = new DescPose(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
      JointPos p2Joint = new JointPos(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);

      ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
      DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

      int rtn = robot.SegmentWeldStart( p1Desc,  p2Desc,  p1Joint,  p2Joint, 20, 20, 0, 0, 5000, false, 0, 0, 0, 100, 100, 100, -1,  exaxisPos, 0, 0,  offdese);
      Console.WriteLine("SegmentWeldStart rtn is {0}", rtn);
   }

Simulate the start of the swing
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Simulate swing start
    * @param [in] weaveNum swing parameter number
    * @return ErrorCode
    */
    int WeaveStartSim(int weaveNum).

End of swing simulation
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Simulate end of swing
    * @param [in] weaveNum swing parameter number
    * @return ErrorCode
    */
    int WeaveEndSim(int weaveNum).

Start trajectory detection warning (no motion)
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Start trajectory detection warning (no motion).
    * @param [in] weaveNum swing parameter number
    * @return Error code
    */
    int WeaveInspectStart(int weaveNum).

End trajectory detection warning (no motion)
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief End trajectory detection warning (no motion).
    * @param [in] weaveNum swing parameter number
    * @return Error code.
    */
    int WeaveInspectEnd(int weaveNum).

Weave fading start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Swinging gradient start
    * @param [in] weaveChangeFlag 1-change swing parameter; 2-change swing parameter + weld speed
    * @param [in] weaveNum weaveNumber 
    * @param [in] velStart Welding start speed, (cm/min)
    * @param [in] velEnd Weld end speed, (cm/min)
    * @return Error code
    */
    int WeaveChangeStart(int weaveChangeFlag, int weaveNum, double velStart, double velEnd);

Weave fade end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:


    /**
    * @brief End of swing gradient
    * @return Error code
    */
    int WeaveChangeEnd()

Sample Welding Code for Robot Weave Change
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
   :linenos:

   private void btnweld_Click(object sender, EventArgs e)
   {
      DescPose p1Desc = new DescPose(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
      JointPos p1Joint = new JointPos(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);

      DescPose p2Desc = new DescPose(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
      JointPos p2Joint = new JointPos(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);

      ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
      DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

      robot.MoveJ(p1Joint, p1Desc, 13, 0, 20, 100, 100, exaxisPos, -1, 0, offdese);
      robot.WeaveStartSim(0);
      robot.MoveL(p2Joint, p2Desc, 13, 0, 20, 100, 100, -1, 0, exaxisPos, 0, 0, offdese);
      robot.WeaveEndSim(0);
      robot.MoveJ(p1Joint, p1Desc, 13, 0, 20, 100, 100, exaxisPos, -1, 0, offdese);
      robot.WeaveInspectStart(0);
      robot.MoveL(p2Joint, p2Desc, 13, 0, 20, 100, 100, -1, 0, exaxisPos, 0, 0, offdese);
      robot.WeaveInspectEnd(0);

      robot.WeldingSetVoltage(1, 19, 0, 0);
      robot.WeldingSetCurrent(1, 190, 0, 0);
      robot.MoveL( p1Joint,  p1Desc, 1, 1, 100, 100, 50, -1,  exaxisPos, 0, 0,  offdese);
      robot.ARCStart(1, 0, 10000);
      robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
      robot.WeaveStart(0);
      robot.WeaveChangeStart(1, 0, 50, 30);
      robot.MoveL( p2Joint,  p2Desc, 1, 1, 100, 100, 1, -1,  exaxisPos, 0, 0,  offdese);
      robot.WeaveChangeEnd();
      robot.WeaveEnd(0);
      robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
      robot.ARCEnd(1, 0, 10000);
   }


Extended IO-Configuration of Welder Gas Detection Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration Welder Gas Detection Signal
    * @param [in] DONum Gas detection signal extended DO number
    * @return error-code
    */
    int SetAirControlExtDoNum(int DONum).

Extended IO-Configuration of the welder arc start signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration Welder Arc Start Signal
    * @param [in] DONum Welder arc start signal extended DO number
    * @return Error code.
    */
    int SetArcStartExtDoNum(int DONum).

Extended IO-Configuring the Welder Reverse Wire Feed Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration of the welder reverse wire feed signal.
    * @param [in] DONum Reverse Wire Feed Signal Extension DO Number
    * @return Error code.
    */
    int SetWireReverseFeedExtDoNum(int DONum).

Extended IO-Configuration Welder Forward Wire Feed Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration of the welder's forward wire feed signal.
    * @param [in] DONum Forward Wire Feed Signal Extension DO Number
    * @return Error code.
    */
    int SetWireForwardFeedExtDoNum(int DONum);

Extended IO-Configuration Welder Arc Start Success Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration Welder Arc Start Success Signal
    * @param [in] DINum Extended DI number for arc success signaling.
    * @return Error code.
    */
    int SetArcDoneExtDiNum(int DINum).

Extended IO-Configuration Welder Ready Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration Welder Ready Signal
    * @param [in] DINum Welder ready signal extended DI number
    * @return error-code
    */
    int SetWeldReadyExtDiNum(int DINum).

Extended IO-Configure weld interrupt recovery signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration Weld Interrupt Recovery Signal
    * @param [in] reWeldDINum Resume Weld Signal after Weld Interrupt Extended DI Number
    * @param [in] abortWeldDINum Exit Weld Signal Extended DI Number after a Weld Interrupt
    * @return Error Code
    */
    int SetExtDIWeldBreakOffRecover(int reWeldDINum, int abortWeldDINum);

Configure and Get Extended IO Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button51_Click(object sender, EventArgs e)
    {
        robot.SetArcStartExtDoNum(10);
        robot.SetAirControlExtDoNum(20);
        robot.SetWireForwardFeedExtDoNum(30);
        robot.SetWireReverseFeedExtDoNum(40);

        robot.SetWeldReadyExtDiNum(50);
        robot.SetArcDoneExtDiNum(60);
        robot.SetExtDIWeldBreakOffRecover(70, 80);
        robot.SetWireSearchExtDIONum(0, 1);

        int[] DIConfig = new int[16];
        int[] DOConfig = new int[16];
        int rtn = robot.GetExtDIConfig(ref DIConfig);
        Console.WriteLine("GetExtDIConfig rtn={0}, welder ready={1}, arc done={2}, reweld start={3}, abort reweld={4}, wiresearch done={5}, laser state={6}, laser err={7}",
            rtn, DIConfig[0], DIConfig[1], DIConfig[2], DIConfig[3], DIConfig[4], DIConfig[5], DIConfig[6]);
        rtn = robot.GetExtDOConfig(ref DOConfig);
        Console.WriteLine("GetExtDOConfig rtn={0}, arc start={1}, air test={2}, wire forward={3}, wire inverse={4}, wiresearch={5}, weld mode={6}, laser enable={7}, laser on={8}, laser reset={9}",
            rtn, DOConfig[0], DOConfig[1], DOConfig[2], DOConfig[3], DOConfig[4], DOConfig[5], DOConfig[6], DOConfig[7], DOConfig[8]);
    }

Get Extended DI Function Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get extended DI function configuration
    * @param [out] DIConfig Extended DI input configuration; [0]-welder ready; [1]-arc start success; [2]-weld interrupt resume; [3]-weld interrupt exit; [4]-wire search success; [5]-laser welder running status; [6]-laser welder fault status; [7-15]-reserved
    * @return  Error code
    */
    public int GetExtDIConfig(ref int[] DIConfig)

Get Extended DO Function Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get extended DO function configuration
    * @param [out] DOConfig Extended DO input configuration; [0]-welder arc start; [1]-gas detection; [2]-forward wire feed; [3]-reverse wire feed; [4]-wire search; [5]-welder control mode; [6]-laser welder enable; [7]-laser welder start; [8]-laser welder reset; [9-15]-reserved
    * @return  Error code
    */
    public int GetExtDOConfig(ref int[] DOConfig)

Arc tracking control
++++++++++++++++++++++++++++++++++
.. code-block:: c#
   :linenos:

    /**
    * @brief Arc tracking control
    * @param [in] flag switch, 0-off; 1-on
    * @param [in] dalayTime hysteresis time, in ms
    * @param [in] isLeftRight left-right deviation compensation
    * @param [in] klr left/right adjustment factor (sensitivity)
    * @param [in] tStartLr left/right start time cyc
    * @param [in] stepMaxLr Maximum compensation in mm per pass * @param [in] stepMaxLr Maximum compensation in mm per pass
    * @param [in] sumMaxLr total max compensation mm
    * @param [in] isUpLow Compensation for up and down deviation
    * @param [in] kud up/down adjustment factor (sensitivity)
    * @param [in] tStartUd up/down compensation time cyc
    * @param [in] stepMaxUd Maximum compensation in mm for each step.
    * @param [in] sumMaxUd up and down total max. compensation mm
    * @param [in] axisSelect upper and lower coordinate system selection, 0-swing; 1-tool; 2-base
    * @param [in] referenceType upper and lower reference current setting mode, 0-feedback; 1-constant
    * @param [in] referSampleStartUd upper and lower reference current sample start count (feedback), cyc
    * @param [in] referSampleCountUd upper and lower reference current sampling cycle count (feedback), cyc
    * @param [in] referenceCurrent upper and lower reference current mA
    * @param [in] offsetType offset tracking type, 0-no offset; 1-sample; 2-percentage /version 3.7.9
    * @param [in] offsetParameter offset parameter; sampling (offset sampling start time, default one cycle); percentage (offset percentage (-100 ~ 100)) /version 3.7.9
    * @return Error code
    */
    int ArcWeldTraceControl(int flag, double delaytime, int isLeftRight, double klr, double tStartLr, double stepMaxLr, double sumMaxLr, int isUpLow, double kud, double tStartUd, double stepMaxUd, double sumMaxUd, int axisSelect, int referenceType, double referSampleStartUd, double referSampleCountUd, double referenceCurrent, int offsetType, int offsetParameter).

Arc tracking AI passband selection
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Arc-tracking AI passband selection.
    * @param [in] channel Arc-tracking AI passband selection, [0-3].
    * @return Error code.
    */
    int ArcWeldTraceExtAIChannelConfig(int channel).

Arc Trace + Multi-Layer Multi-Channel Compensation on
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Arc tracing + multilayer compensation on.
    * @return Error code
    */
    int ArcWeldTraceReplayStart();

ArcWeldTrace + MultiLayerMultiChannelCompensation OFF
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
   :linenos:

   /**
   * @brief Arc tracing + multilayer multichannel compensation off .
   * @return Error code
   */
   int ArcWeldTraceReplayEnd();

Offset Coordinate Change - Multi-Layer Multi-Pass Welding
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     /**
     * @brief Offset Coordinate Change - Multi-Layer Multi-Pass Weld
     * @param [in] pointO Cartesian orientation of datum.
     * @param [in] pointX Cartesian position of datum X in offset direction.
     * @param [in] pointZ Cartesian position of the datum Z to the offset direction point.
     * @param [in] dx x-direction offset (mm)
     * @param [in] z offset (mm)
     * @param [in] offset around y-axis (°)
     * @param [out] Calculation result offset (mm)
     * @return Error Code
     */
    int MultilayerOffsetTrsfToBase(DescTran pointO, DescTran pointX, DescTran pointZ, double dx, double dy, double db, ref DescPose offset);

Multi-layer multi-pass welding arc tracking code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
   :linenos:

   private void button52_Click(object sender, EventArgs e)
   {
      JointPos mulitilineorigin1_joint = new JointPos(-24.090, -63.501, 84.288, -111.940, -93.426, 57.669);
      DescPose mulitilineorigin1_desc = new DescPose(-677.559, 190.951, -1.205, 1.144, -41.482, -82.577);

      DescTran mulitilineX1_desc = new DescTran();
      mulitilineX1_desc.x = -677.556;
      mulitilineX1_desc.y = 211.949;
      mulitilineX1_desc.z = -1.206;

      DescTran mulitilineZ1_desc = new DescTran();
      mulitilineZ1_desc.x = -677.564;
      mulitilineZ1_desc.y = 190.956;
      mulitilineZ1_desc.z = 19.817;

      JointPos mulitilinesafe_joint = new JointPos(-25.734, -63.778, 81.502, -108.975, -93.392, 56.021);
      DescPose mulitilinesafe_desc = new DescPose(-677.561, 211.950, 19.812, 1.144, -41.482, -82.577);
      JointPos mulitilineorigin2_joint = new JointPos(-29.743, -75.623, 101.241, -116.354, -94.928, 55.735);
      DescPose mulitilineorigin2_desc = new DescPose(-563.961, 215.359, -0.681, 2.845, -40.476, -87.443);

      DescTran mulitilineX2_desc = new DescTran();
      mulitilineX2_desc.x = -563.965;
      mulitilineX2_desc.y = 220.355;
      mulitilineX2_desc.z = -0.680;

      DescTran mulitilineZ2_desc = new DescTran();
      mulitilineZ2_desc.x = -563.968;
      mulitilineZ2_desc.y = 215.362;
      mulitilineZ2_desc.z = 4.331;

      ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
      DescPose offset = new DescPose(0, 0, 0, 0, 0, 0);

      Thread.Sleep(10);
      int error = robot.MoveJ( mulitilinesafe_joint,  mulitilinesafe_desc, 13, 0, 10, 100, 100,  epos, -1, 0,  offset);
      Console.WriteLine("MoveJ return: {0}", error);

      error = robot.MoveL( mulitilineorigin1_joint,  mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1,  epos, 0, 0,  offset, 0, 100);
      Console.WriteLine("MoveL return: {0}", error);

      error = robot.MoveJ(mulitilinesafe_joint, mulitilinesafe_desc, 13, 0, 10, 100, 100, epos, -1, 0, offset);
      Console.WriteLine("MoveJ return: {0}", error);

      error = robot.MoveL(mulitilineorigin2_joint, mulitilineorigin2_desc, 13, 0, 10, 100, 100, -1,  epos, 0, 0,  offset, 0, 100);
      Console.WriteLine("MoveL return: {0}", error);

      error = robot.MoveJ(mulitilinesafe_joint, mulitilinesafe_desc, 13, 0, 10, 100, 100, epos, -1, 0, offset);
      Console.WriteLine("MoveJ return: {0}", error);

      error = robot.MoveL( mulitilineorigin1_joint,  mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1,  epos, 0, 0,  offset, 0, 100);
      Console.WriteLine("MoveL return: {0}", error);

      error = robot.ARCStart(1, 0, 3000);
      Console.WriteLine("ARCStart return: {0}", error);

      error = robot.WeaveStart(0);
      Console.WriteLine("WeaveStart return: {0}", error);

      error = robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10);
      Console.WriteLine("ArcWeldTraceControl return: {0}", error);

      error = robot.MoveL( mulitilineorigin2_joint,  mulitilineorigin2_desc, 13, 0, 1, 100, 100, -1,  epos, 0, 0,  offset, 0, 100);
      Console.WriteLine("MoveL return: {0}", error);

      error = robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10);
      Console.WriteLine("ArcWeldTraceControl return: {0}", error);

      error = robot.WeaveEnd(0);
      Console.WriteLine("WeaveEnd return: {0}", error);

      error = robot.ARCEnd(1, 0, 10000);
      Console.WriteLine("ARCEnd return: {0}", error);

      error = robot.MoveJ(mulitilinesafe_joint, mulitilinesafe_desc, 13, 0, 10, 100, 100, epos, -1, 0, offset);
      Console.WriteLine("MoveJ return: {0}", error);

      error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 10.0, 0.0, 0.0, ref offset);
      Console.WriteLine("MultilayerOffsetTrsfToBase return: {0}  offect is {1} {2} {3}", error, offset.tran.x, offset.tran.y, offset.tran.z);

      error = robot.MoveL( mulitilineorigin1_joint,  mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1,  epos, 0, 1,  offset, 0, 100);
      Console.WriteLine("MoveL return: {0}", error);

      error = robot.ARCStart(1, 0, 3000);
      Console.WriteLine("ARCStart return: {0}", error);

      error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 10, 0, 0, ref offset);
      Console.WriteLine("MultilayerOffsetTrsfToBase return: {0}  offect is {1} {2} {3}", error, offset.tran.x, offset.tran.y, offset.tran.z);

      error = robot.ArcWeldTraceReplayStart();
      Console.WriteLine("ArcWeldTraceReplayStart return: {0}", error);

      error = robot.MoveL( mulitilineorigin2_joint,  mulitilineorigin2_desc, 13, 0, 2, 100, 100, -1,  epos, 0, 1,  offset, 0, 100);
      Console.WriteLine("MoveL return: {0}", error);

      error = robot.ArcWeldTraceReplayEnd();
      Console.WriteLine("ArcWeldTraceReplayEnd return: {0}", error);

      error = robot.ARCEnd(1, 0, 10000);
      Console.WriteLine("ARCEnd return: {0}", error);

      error = robot.MoveJ(mulitilinesafe_joint, mulitilinesafe_desc, 13, 0, 10, 100, 100, epos, -1, 0, offset);
      Console.WriteLine("MoveJ return: {0}", error);

      error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 0, 10, 0, ref offset);
      Console.WriteLine("MultilayerOffsetTrsfToBase return: {0}  offect is {1} {2} {3}", error, offset.tran.x, offset.tran.y, offset.tran.z);

      error = robot.MoveL( mulitilineorigin1_joint,  mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1,  epos, 0, 1,  offset, 0, 100);
      Console.WriteLine("MoveL return: {0}", error);

      error = robot.ARCStart(1, 0, 3000);
      Console.WriteLine("ARCStart return: {0}", error);

      error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 0, 10, 0, ref offset);
      Console.WriteLine("MultilayerOffsetTrsfToBase return: {0}  offect is {1} {2} {3}", error, offset.tran.x, offset.tran.y, offset.tran.z);

      error = robot.ArcWeldTraceReplayStart();
      Console.WriteLine("MoveJ return: {0}", error);

      error = robot.MoveL(mulitilineorigin2_joint, mulitilineorigin2_desc, 13, 1, 2, 100, 100, -1, epos, 1, 1, offset, 1, 100);
      Console.WriteLine("MoveL return: {0}", error);

      error = robot.ArcWeldTraceReplayEnd();
      Console.WriteLine("ArcWeldTraceReplayEnd return: {0}", error);

      error = robot.ARCEnd(1, 0, 3000);
      Console.WriteLine("ARCEnd return: {0}", error);

      error = robot.MoveJ(mulitilinesafe_joint, mulitilinesafe_desc, 13, 0, 10, 100, 100, epos, -1, 0, offset);
      Console.WriteLine("MoveJ return: {0}", error);
   }

Arc Tracking Welder Current Feedback AI Channel Selection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
   :linenos:

    /**
    * @brief Arc tracking welder current feedback AI channel selection
    * @param [in] channel channel; 0-extension AI0; 1-extension AI1; 2-extension AI2; 3-extension AI3; 4-control box AI0; 5-control box AI1
    * @return error code
    */
    int ArcWeldTraceAIChannelCurrent(int channel).

Arc tracking welder voltage feedback AI channel selection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
   :linenos:

    /**
    * @brief Arc Tracking Welder Voltage Feedback AI Channel Selection
    * @param [in] channel channel; 0-extension AI0; 1-extension AI1; 2-extension AI2; 3-extension AI3; 4-control box AI0; 5-control box AI1
    * @return error code
    */
    int ArcWeldTraceAIChannelVoltage(int channel).

Arc tracking welder current feedback conversion parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
   :linenos:

    /**
    * @brief Arc tracking welder current feedback conversion parameters
    * @param [in] AILow AI channel lower limit, default value 0V, range [0-10V].
    * @param [in] AIHigh AI channel upper limit, default value 10V, range [0-10V]
    * @param [in] currentLow AI channel lower limit corresponds to the welder current value, default value 0V, range [0-200V].
    * @param [in] currentHigh AI channel upper limit corresponds to the welder current value, default value 100V, range [0-200V].
    * @return Error code
    */
    public int ArcWeldTraceCurrentPara(double AILow, double AIHigh, double currentLow, double currentHigh)

Arc Trace Welder Voltage Feedback Conversion Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
   :linenos:

    /**
    * @brief Arc Tracker Welder Voltage Feedback Conversion Parameters
    * @param [in] AILow AI channel lower limit, default value 0V, range [0-10V].
    * @param [in] AIHigh AI channel upper limit, default value 10V, range [0-10V].
    * @param [in] voltageLow AI channel lower limit corresponds to the welder voltage value, default value 0V, range [0-200V].
    * @param [in] voltageHigh AI channel upper limit corresponds to the welder voltage value, default value 100V, range [0-200V] * @param [in] voltageHigh AI channel upper limit corresponds to the welder voltage value, default value 100V, range [0-200V
    * @return Error code
    */
    public int ArcWeldTraceVoltagePara(double AILow, double AIHigh, double voltageLow, double voltageHigh)

Arc Trace Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
   :linenos:

   private void button8_Click(object sender, EventArgs e)
   {
      DescPose startdescPose = new DescPose(441.901, 416.508, -51.979, -179.234, 0.718, -115.305);
      JointPos startjointPos = new JointPos(-146.22, -60.551, 104.859, -135.317, -90.289, 59.088);

      DescPose enddescPose = new DescPose(441.901, 615.317, -51.979, -179.234, 0.718, -115.305);
      JointPos endjointPos = new JointPos(-133.22, -44.193, 74.934, -121.661, -90.509, 72.087);

      DescPose safetydescPose = new DescPose(441.901, 416.508, -51.979, -179.234, 0.718, -115.305);
      JointPos safetyjointPos = new JointPos(-146.22, -60.551, 104.859, -135.317, -90.289, 59.088);

      ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
      DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
      robot.MoveJ(safetyjointPos, safetydescPose, 1, 0, 20, 100, 100, exaxisPos, -1, 0, offdese);

      robot.WeldingSetCurrentRelation(0, 495, 1, 10, 0);
      robot.WeldingSetVoltageRelation(10, 45, 1, 10, 1);
      robot.WeldingSetVoltage(0, 25, 1, 0); 
      robot.WeldingSetCurrent(0, 260, 0, 0); 

      int rtn = robot.ArcWeldTraceAIChannelCurrent(4);
      Console.WriteLine("ArcWeldTraceAIChannelCurrent rtn is " + rtn);
      rtn = robot.ArcWeldTraceAIChannelVoltage(5);
      Console.WriteLine("ArcWeldTraceAIChannelVoltage rtn is " + rtn);
      rtn = robot.ArcWeldTraceCurrentPara((double)0, (double)5, (double)0, (double)500);
      Console.WriteLine("ArcWeldTraceCurrentPara rtn is " + rtn);
      rtn = robot.ArcWeldTraceVoltagePara((double)1.018, (double)10, (double)0, (double)50);
      Console.WriteLine("ArcWeldTraceVoltagePara rtn is " + rtn);

      robot.MoveJ(startjointPos, startdescPose, 1, 0, 20, 100, 100, exaxisPos, -1, 0, offdese);
      robot.ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
      robot.ARCStart(0, 0, 10000);
      robot.WeaveStart(0);
         robot.MoveL(endjointPos, enddescPose, 1, 0, 100, 100, 2, -1, 0,exaxisPos, 0, 0, offdese);
      robot.ARCEnd(0, 0, 10000);
      robot.WeaveEnd(0);
      robot.ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
      robot.MoveJ(safetyjointPos, safetydescPose, 1, 0, 20, 100, 100, exaxisPos, -1, 0, offdese);

   }

Start of wire position finding
++++++++++++++++++++++++++++++++++
.. code-block:: c#
   :linenos:

    /**
    * @brief Welding wire seek start.
    * @param [in] refPos 1-datum point 0-contact point
    * @param [in] searchVel SearchVelocity %.
    * @param [in] searchDis Search distance mm
    * @param [in] autoBackFlag autoBackFlag, 0 - not auto; - auto
    * @param [in] autoBackVel autoBackVelocity % * @param [in] searchDis searchDistance mm
    * @param [in] autoBackDis autoBack distance mm
    * @param [in] offectFlag 1-with offset seek; 0-teach point seek
    * @return error code
    */
    int WireSearchStart(int refPos, double searchVel, int searchDis, int autoBackFlag, double autoBackVel, int autoBackDis, int offectFlag);

End of wire search
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief End of wire seek.
    * @param [in] refPos 1-datum point 2-contact point
    * @param [in] searchVel searchVelocity %
    * @param [in] searchDis Search distance mm
    * @param [in] autoBackFlag autoBackFlag, 0 - not auto; - auto
    * @param [in] autoBackVel autoBackVelocity % * @param [in] searchDis searchDistance mm
    * @param [in] autoBackDis autoBack distance mm
    * @param [in] offectFlag 1-with offset seek; 2-teach point seek
    * @return error code
    */
    int WireSearchEnd(int refPos, double searchVel, int searchDis, int autoBackFlag, double autoBackVel, int autoBackDis, int offectFlag);

Calculate the wire seek offset
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Calculate wire-seeking offsets.
    * @param [in] seamType weldType
    * @param [in] method Calculate the method.
    * @param [in] varNameRef datums 1-6, "#" means no point variable
    * @param [in] varNameRes contact points 1-6, "#" means no point variable
    * @param [out] offectFlag 0-offset directly superimposed on the command point; 1-offset requires a coordinate transformation of the command point
    * @param [out] offect Offset pose [x, y, z, a, b, c]
    * @return Error code
    */
    int GetWireSearchOffset(int seamType, int method, string[] varNameRef, string[] varNameRes, ref int offsetFlag, ref DescPose offset);;

Wait for the wire search to complete
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Waiting for wire seek to complete.
    * @return Error code.
    */
    int WireSearchWait(string name).

Write wire search contact to database.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Welding wire seek contacts written to database.
    * @param [in] varName Contact name "RES0" ~ "RES99".
    * @param [in] pos contact data [x, y, x, a, b, c]
    * @return Error Code
    */
    int SetPointToDatabase(string varName, DescPose pos);

Robot Welding Wire Position Finding Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
   :linenos:

   private void button53_Click(object sender, EventArgs e)
   {
      DescPose toolCoord=new DescPose(0, 0, 200, 0, 0, 0);
      robot.SetToolCoord(1, toolCoord, 0, 0, 1, 0);
      DescPose wobjCoord=new DescPose(0, 0, 0, 0, 0, 0);
      robot.SetWObjCoord(1, wobjCoord, 0);

      int rtn0, rtn1, rtn2 = 0;
      ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
      DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

      DescPose descStart = new DescPose(216.543, 445.175, 93.465, 179.683, 1.757, -112.641);
      JointPos jointStart = new JointPos(-128.345, -86.660, 114.679, -119.625, -89.219, 74.303);

      DescPose descEnd = new DescPose(111.143, 523.384, 87.659, 179.703, 1.835, -97.750);
      JointPos jointEnd = new JointPos(-113.454, -81.060, 109.328, -119.954, -89.218, 74.302);

      robot.MoveL(jointStart, descStart, 1, 1, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);
      robot.MoveL(jointEnd, descEnd, 1, 1, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);

      DescPose descREF0A = new DescPose(142.135, 367.604, 86.523, 179.728, 1.922, -111.089);
      JointPos jointREF0A = new JointPos(-126.794, -100.834, 128.922, -119.864, -89.218, 74.302);

      DescPose descREF0B = new DescPose(254.633, 463.125, 72.604, 179.845, 2.341, -114.704);
      JointPos jointREF0B = new JointPos(-130.413, -81.093, 112.044, -123.163, -89.217, 74.303);

      DescPose descREF1A = new DescPose(92.556, 485.259, 47.476, -179.932, 3.130, -97.512);
      JointPos jointREF1A = new JointPos(-113.231, -83.815, 119.877, -129.092, -89.217, 74.303);

      DescPose descREF1B = new DescPose(203.103, 583.836, 63.909, 179.991, 2.854, -103.372);
      JointPos jointREF1B = new JointPos(-119.088, -69.676, 98.692, -121.761, -89.219, 74.303);

      rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
      robot.MoveL(jointREF0A, descREF0A, 1, 1, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);  //起点
      robot.MoveL(jointREF0B, descREF0B, 1, 1, 100, 100, 100, -1, exaxisPos, 1, 0, offdese);  //方向点
      rtn1 = robot.WireSearchWait("REF0");
      rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

      rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
      robot.MoveL(jointREF1A, descREF1A, 1, 1, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);  //起点
      robot.MoveL(jointREF1B, descREF1B, 1, 1, 100, 100, 100, -1, exaxisPos, 1, 0, offdese);  //方向点
      rtn1 = robot.WireSearchWait("REF1");
      rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

      rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
      robot.MoveL(jointREF0A, descREF0A, 1, 1, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);  //起点
      robot.MoveL(jointREF0B, descREF0B, 1, 1, 100, 100, 100, -1, exaxisPos, 1, 0, offdese);  //方向点
      rtn1 = robot.WireSearchWait("RES0");
      rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

      rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
      robot.MoveL(jointREF1A, descREF1A, 1, 1, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);  //起点
      robot.MoveL(jointREF1B, descREF1B, 1, 1, 100, 100, 100, -1, exaxisPos, 1, 0, offdese);  //方向点
      rtn1 = robot.WireSearchWait("RES1");
      rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

      string[] varNameRef = { "REF0", "REF1", "#", "#", "#", "#" };
      string[] varNameRes = { "RES0", "RES1", "#", "#", "#", "#" };
      int offectFlag = 0;
      DescPose offectPos = new DescPose(0, 0, 0, 0, 0, 0);
      rtn0 = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes, ref offectFlag, ref offectPos);
      robot.PointsOffsetEnable(0, offectPos);
      robot.MoveL(jointStart, descStart, 1, 1, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);
      robot.MoveL(jointEnd, descEnd, 1, 1, 100, 100, 100, -1, exaxisPos, 1, 0, offdese);
      robot.PointsOffsetDisable();
   }

Setting the end of the welding voltage gradient
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Sets the weld voltage gradient to start.
    * @param [in] IOType Control type; 0-control box IO; 1-digital communication protocol (UDP); 2-digital communication protocol (ModbusTCP)
    * @param [in] voltageStart Start welding voltage (V)
    * @param [in] voltageEnd End weld voltage (V)
    * @param [in] AOIndex control box AO port number (0-1)
    * @param [in] blend Whether smooth or not 0-not smooth; 1-smooth
    * @return Error code
    */
    int WeldingSetVoltageGradualChangeStart(int IOType, double voltageStart, double voltageEnd, int AOIndex, int blend);

Set weld voltage gradual change end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the end of the welding voltage gradient.
    * @return Error Code
    */
    int WeldingSetVoltageGradualChangeEnd();

Sets the weld current gradual change end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the weld current gradient to start.
    * @param [in] IOType Control type; 0-control box IO; 1-digital communication protocol (UDP); 2-digital communication protocol (ModbusTCP)
    * @param [in] voltageStart Start welding current (A)
    * @param [in] voltageEnd End weld current (A)
    * @param [in] AOIndex Control box AO port number (0-1)
    * @param [in] blend Whether smooth or not 0-not smooth; 1-smooth
    * @return Error code
    */
    int WeldingSetCurrentGradualChangeStart(int IOType, double currentStart, double currentEnd, int AOIndex, int blend);

Set weld current gradual change end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the end of the welding current gradient.
    * @return Error code
    */
    int WeldingSetCurrentGradualChangeEnd();

Robot Welding Current Voltage Gradual Change Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

      private void btnweld_Click(object sender, EventArgs e)
      {
         DescPose startdescPose = new DescPose(-319.303, -240.689, 116.379, -175.879, -0.337, 148.239);
         JointPos startjointPos = new JointPos(20.474, -103.554, 126.774, -116.682, -87.746, -37.709);

         DescPose enddescPose = new DescPose(-454.166, -327.159, 62.217, 177.199, -2.276, 154.955);
         JointPos endjointPos = new JointPos(27.176, -74.423, 104.557, -119.315, -93.514, -37.698);

         DescPose safedescPose = new DescPose(-375.533, -543.319, 19.798, 177.486, -2.489, 175.825);
         JointPos safejointPos = new JointPos(48.074, -59.714, 89.955, -119.777, -93.508, -37.683);

         ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
         DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

         robot.WeldingSetCurrentRelation(0, 495, 1, 10, 0);
         robot.WeldingSetVoltageRelation(10, 45, 1, 10, 1);

         robot.WeldingSetVoltage(0, 25, 1, 0);//
         robot.WeldingSetCurrent(0, 260, 0, 0);// 

         robot.MoveJ(safejointPos, safedescPose, 1, 0, 5, 100, 100, exaxisPos, -1, 0, offdese);

         int rtn = robot.WeldingSetCurrentGradualChangeStart(0, 260, 220, 0, 0);
         Console.WriteLine($"WeldingSetCurrentGradualChangeStart rtn is {rtn}");
         rtn = robot.WeldingSetVoltageGradualChangeStart(0, 25, 22, 1, 0);
         Console.WriteLine($"WeldingSetVoltageGradualChangeStart rtn is {rtn}");

         rtn = robot.ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
         Console.WriteLine($"ArcWeldTraceControl rtn is {rtn}");

         robot.MoveJ(startjointPos, startdescPose, 1, 0, 5, 100, 100, exaxisPos, -1, 0, offdese);

         robot.ARCStart(0, 0, 10000);
         robot.WeaveStart(0);
         rtn = robot.WeaveChangeStart(2, 1, 24, 36);
         Console.WriteLine($"WeaveChangeStart rtn is {rtn}");
         //robot.MoveL(endjointPos, enddescPose, 1, 0, 100, 100, 2, -1, exaxisPos, 0, 0, offdese);
         robot.ARCEnd(0, 0, 10000);
         robot.WeaveChangeEnd();
         robot.WeaveEnd(0);
         robot.ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
         robot.WeldingSetCurrentGradualChangeEnd();
         robot.WeldingSetVoltageGradualChangeEnd();
      }

Set custom weaving parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Set custom weaving parameters
    * @param [in] id Custom weaving number: 0-2
    * @param [in] pointNum Number of weaving points: 0-10
    * @param [in] point Moving endpoint data x,y,z
    * @param [in] stayTime Weaving dwell time in ms
    * @param [in] frequency Weaving frequency in Hz
    * @param [in] incStayType Wait mode: 0-cycle does not include wait time; 1-cycle includes wait time
    * @param [in] stationary Weaving position wait: 0-continue motion during wait time; 1-position static during wait time
    * @return  Error code
    */
    public int CustomWeaveSetPara(int id, int pointNum, DescTran[] points, double[] stayTimes, double frequency, int incStayType, int stationary)

Get custom weaving parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Get custom weaving parameters
    * @param [in] id Custom weaving number: 0-2
    * @param [out] pointNum Number of weaving points: 0-10
    * @param [out] point Moving endpoint data x,y,z
    * @param [out] stayTime Weaving dwell time in ms
    * @param [out] frequency Weaving frequency in Hz
    * @param [out] incStayType Wait mode: 0-cycle does not include wait time; 1-cycle includes wait time
    * @param [out] stationary Weaving position wait: 0-continue motion during wait time; 1-position static during wait time
    * @return  Error code
    */
    public int CustomWeaveGetPara(int id, ref int pointNum, ref DescTran[] points, ref double[] stayTimes, ref double frequency, ref int incStayType, ref int stationary)

Custom Weaving Parameters Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    public void TestCoordMain5()
    {  
        DescTran[] points = new DescTran[10];
        for (int i = 0; i < 10; i++)
        {
            points[i] = new DescTran();
        }
        points[0].x = -3;
        points[0].y = -3;
        points[0].z = 0;
        points[1].x = -6;
        points[1].y = 0;
        points[1].z = 0;
        points[2].x = -3;
        points[2].y = 3;
        points[2].z = 0;
        points[3].x = 0;
        points[3].y = 0;
        points[3].z = 0;
        double[] stayTimes = new double[10] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        int rtn = robot.CustomWeaveSetPara(2, 4, points, stayTimes, 1.000, 0, 0);
        Console.WriteLine($"CustomWeaveSetPara rtn is {rtn}");
        System.Threading.Thread.Sleep(1000);
        int pointNum = 0;
        double frequency = 0;
        int incStayType = 0;
        int stationary = 0;
        rtn = robot.CustomWeaveGetPara(2, ref pointNum, ref points, ref stayTimes, ref frequency, ref incStayType, ref stationary);
        Console.WriteLine($"pointNum is {pointNum}");
        for (int i = 0; i < pointNum; i++)
        {
            Console.WriteLine($"point {i}, point x y z {points[i].x:F6} {points[i].y:F6} {points[i].z:F6}");
        }
        Console.WriteLine($"fre is {frequency:F6}, stay is {incStayType} {stationary}");
        robot.WeaveSetPara(0, 9, 1.000000, 1, 5.000000, 6.000000, 5.000000, 50, 100, 100, 0, 1, 0.000000, 0.000000);
        DescPose desc_p1 = new DescPose(-288.650, 367.807, 288.404, 0.000, -0.001, 0.001);
        DescPose desc_p2 = new DescPose(-431.714, 367.815, 288.415, 0.001, 0.001, 0.000);    
        DescPose desc_p3 = new DescPose(-348.666, 427.798, 288.404, -0.000, -0.000, 0.001);
        JointPos j1 = new JointPos(140.656,  -84.560,  -91.707, -93.734,  90.000,50.655 );
        JointPos j2 = new JointPos ( 149.873, -98.298, -77.599,  -94.103,  90.000,  59.873 );
        JointPos j3 = new JointPos (139.773,  -96.173, -80.014,  -93.814,  90.000,  49.772 );
        ExaxisPos epos = new ExaxisPos(0,0,0,0);
        DescPose offset_pos = new DescPose(0,0,0,0,0,0);
        robot.MoveJ(j1, desc_p1, 3, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.WeaveStart(0);
        robot.Circle(j3, desc_p3, 3, 0, 100, 100, epos, j2, desc_p2, 3, 0, 100, 100, epos, 10, -1, offset_pos, 100, -1, 0);
        robot.WeaveEnd(0);
        robot.MoveJ(j1, desc_p1, 3, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.WeaveStart(0);
        robot.MoveC(j3, desc_p3, 3, 0, 100, 100, epos, 0, offset_pos, j2, desc_p2, 3, 0, 100, 100, epos, 0, offset_pos, 10, -1, 0);
        robot.WeaveEnd(0);
        robot.MoveJ(j1, desc_p1, 3, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.WeaveStart(0);
        robot.MoveL(j2, desc_p2, 3, 0, 100, 100, 10, -1, epos, 0, 0, offset_pos, 0, 0, 10);
        robot.WeaveEnd(0);
    }

Laser Welding Machine Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Write configuration parameters for one of the 10 process groups of the laser welding machine and configure the welding machine
    * @param[in] io_type Communication type 0-IO 1-UDP
    * @param[in] num Group number to be set (1~10)
    * @param[in] scanSpeed Scanning speed
    * @param[in] scanWidth Scanning width
    * @param[in] peakPower Peak power
    * @param[in] dutyCycle Duty cycle
    * @param[in] freq Frequency
    * @return Error code
    */
    public int SetLaserWeldingParam(int io_type, int num, int scanSpeed, int scanWidth, int peakPower, int dutyCycle, int freq)

Set Laser Welding Start/Stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Set laser welding machine on/off
    * @param[in] io_type Communication type 0-IO 1-UDP
    * @param[in] status Control word 0-laser off 1-laser on
    * @param[in] max_waittime Maximum wait time
    * @return Error code
    */
    public int SetLaserWeldingStartEnd(int io_type, int status, int max_waittime)

Laser Welding Machine Enable/Disable
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Enable/disable laser welding machine
    * @param[in] io_type Communication type 0-IO 1-UDP
    * @param[in] status 0-disable 1-enable
    * @return Error code
    */
    public int SetLaserWeldingEnable(int io_type, int status)

Laser Welding Machine Fault Reset
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Reset laser welding machine fault
    * @param[in] io_type Communication type 0-IO 1-UDP
    * @param[in] status Control word 0-invalid 1-fault reset
    * @return Error code
    */
    public int ResetLaserWeldingErr(int io_type, int status)

Get Laser Welding Machine Running Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Get laser welding machine running status
    * @param[in] io_type Communication type 0-IO 1-UDP
    * @param[out] status Control word 0-stopped 1-running
    * @return Error code
    */
    public int GetLaserWeldingRunningState(int io_type, ref int status)

Get Laser Welding Machine Fault Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Get laser welding machine fault status
    * @param[in] io_type Communication type 0-IO 1-UDP
    * @param[out] status 0-no fault 1-fault present
    * @return Error code
    */
    public int GetLaserWeldingErrState(int io_type, ref int status)

Get Laser Welding Machine Configured Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Get configuration parameters for one of the 10 process groups of the laser welding machine
    * @param[in] num Group number to be set (1~10)
    * @param[out] scanSpeed Scanning speed
    * @param[out] scanWidth Scanning width
    * @param[out] peakPower Peak power
    * @param[out] dutyCycle Duty cycle
    * @param[out] freq Frequency
    * @return Error code
    */
    public int GetLaserWeldingParamTarget(int num, ref int scanSpeed, ref int scanWidth, ref int peakPower, ref int dutyCycle, ref int freq)

Get Currently Active Configuration Parameters of the Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Get the currently active configuration parameters of the laser welding machine
    * @param[in] io_type Communication type 0-IO 1-UDP
    * @param[out] scanSpeed Scanning speed
    * @param[out] scanWidth Scanning width
    * @param[out] peakPower Peak power
    * @param[out] dutyCycle Duty cycle
    * @param[out] freq Frequency
    * @return Error code
    */
    public int GetLaserWeldingParamActual(int io_type, ref int scanSpeed, ref int scanWidth, ref int peakPower, ref int dutyCycle, ref int freq)
    
Configure Laser Welding Machine Extended IO Enable DO Port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Set the extended IO DO port for laser welding machine enable
    * @param[in] ctrlModeDONum Extended DO port number for laser welding machine enable
    * @return Error code
    */
    public int SetLaserWeldingEnableExtDoNum(int ctrlModeDONum)

Configure Laser Welding Machine Extended IO Start DO Port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Set the extended IO DO port for laser welding machine start (laser on/off)
    * @param[in] ctrlModeDONum Extended DO port number for laser welding machine start (laser on/off)
    * @return Error code
    */
    public int SetLaserWeldingStartExtDoNum(int ctrlModeDONum)

Configure Laser Welding Machine Extended IO Fault Reset DO Port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Set the extended IO DO port for laser welding machine fault reset
    * @param[in] ctrlModeDONum Extended DO port number for laser welding machine fault reset
    * @return Error code
    */
    public int SetLaserWeldingErrResetExtDoNum(int ctrlModeDONum)

Configure Laser Welding Machine Running Status (Laser On Status) Extended DI
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Configure the extended DI for laser welding machine running status (laser on status)
    * @param[in] diNum Extended DI port number for laser welding machine running status (laser on status)
    * @return Error code
    */
    public int SetLaserWeldingRunningStateExtDiNum(int diNum)
    
Configure Laser Welding Machine Extended IO Fault Status DI Port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Configure the extended DI for laser welding machine fault status
    * @param[in] diNum Extended DI port number for laser welding machine fault status
    * @return Error code
    */
    public int SetLaserWeldingErrStateExtDiNum(int diNum)
        
Laser Welding Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    private void btnLaserWeld_Click(object sender, EventArgs e)
    {

        int rtn = -1;
        // Load UDP driver
        rtn = robot.ExtDevLoadUDPDriver();
        if (rtn != 0)
        {
            Console.WriteLine("Failed to load UDP driver, error code: " + rtn);
        }
        Thread.Sleep(1000);

        // Set laser welding parameters: io_type=1, num=3, scanSpeed=2000, scanWidth=3, peakPower=1500, dutyCycle=100, freq=1000
        rtn = robot.SetLaserWeldingParam(1, 3, 2000, 3, 1500, 100, 1000);
        if (rtn != 0)
        {
            Console.WriteLine("SetLaserWeldingParam failed, error code: " + rtn);
        }
        else
        {
            Console.WriteLine("SetLaserWeldingParam success");
        }

        // Set the start DO port number
        rtn = robot.SetLaserWeldingStartExtDoNum(1);
        if (rtn != 0)
        {
            Console.WriteLine("SetLaserWeldingStartExtDoNum failed, error code: " + rtn);
        }

        // Set to mode 0 (teaching mode)
        rtn = robot.Mode(0);
        if (rtn != 0)
        {
            Console.WriteLine("Set mode 0 failed, error code: " + rtn);
        }
        Thread.Sleep(1000);


        DescPose desc_pos1 = new DescPose(-303.721, -206.960, 297.105, 152.209, 19.857, 109.166);
        DescPose desc_pos2 = new DescPose(-301.575, -254.888, 284.786, 155.919, 26.946, 111.629);
        DescPose desc_safe = new DescPose(-344.386, -280.830, 435.073, 173.835, 15.333, 124.931);


        ExaxisPos exaxis = new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offset = new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Move to the first welding point
        int error = robot.MoveL(desc_pos1, 0, 0, 100, 100, 100, -1, 0, exaxis, 0, 0, offset, -1, 0);
        Console.WriteLine("MoveL to pos1 return: " + error);

        // Start laser (laser on)
        rtn = robot.SetLaserWeldingStartEnd(1, 1, 10000);
        if (rtn != 0)
        {
            Console.WriteLine("SetLaserWeldingStartEnd (start) failed, error code: " + rtn);
        }
        else
        {
            Console.WriteLine("Laser started");
        }

        // Move to the second welding point (during welding)
        rtn = robot.MoveL(desc_pos2, 0, 0, 30, 100, 100, -1, 0, exaxis, 0, 0, offset, -1, 0);
        Console.WriteLine("MoveL to pos2 return: " + rtn);

        Thread.Sleep(500);
        // Stop laser (laser off)
        rtn = robot.SetLaserWeldingStartEnd(1, 0, 10000);
        if (rtn != 0)
        {
            Console.WriteLine("SetLaserWeldingStartEnd (stop) failed, error code: " + rtn);
        }
        else
        {
            Console.WriteLine("Laser stopped");
        }

        // Move to safety point
        rtn = robot.MoveL(desc_safe, 0, 0, 100, 100, 100, -1, 0, exaxis, 0, 0, offset, -1, 0);
        Console.WriteLine("MoveL to safe_pos return: " + rtn);

        // Set to mode 1 (remote mode)
        rtn = robot.Mode(1);
        if (rtn != 0)
        {
            Console.WriteLine("Set mode 1 failed, error code: " + rtn);
        }
        Thread.Sleep(1000);

        // Close connection
        robot.CloseRPC();
        Thread.Sleep(1000);

        Console.WriteLine("Test completed");

        return ;
    }

Set Weave End Return to Cycle Zero Point
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief  Set whether to return to the cycle zero point at the end of weaving
    * @param [in] flag Whether to return to cycle zero point at weave end; 0-do not return; 1-return to cycle zero point
    * @return  Error code
    */
    public int SetWeavebackCenterConfig(int flag) 
           
Get Weave End Return to Cycle Zero Point Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the weave end return to cycle zero point parameters
    * @param [out] flag Whether to return to cycle zero point at weave end; 0-do not return; 1-return to cycle zero point
    * @return  Error code
    */
    public int GetWeavebackCenterConfig(ref int flag)
           
Weave End Return to Cycle Zero Point Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    public void TestSplineWeave()
    {
        int rtn;

        // Weave return to center configuration
        robot.SetWeavebackCenterConfig(1);
        int weaveBackConfig = 0;
        robot.GetWeavebackCenterConfig(ref weaveBackConfig);
        Console.WriteLine("GetWeavebackCenterConfig: {0}", weaveBackConfig);

        JointPos j1 = new JointPos(9.000, -66.067, 67.706, -103.217, -90.151, 100.669);
        JointPos j2 = new JointPos(-4.660, -107.973, 103.734, -76.214, -89.999, 90.886);
        JointPos j3 = new JointPos(-36.762, -77.380, 91.364, -127.159, -90.024, 54.833);
        JointPos j4 = new JointPos(-62.875, -89.460, 86.437, -77.030, -90.012, 31.539);
        DescPose desc_pos1 = new DescPose(-654.129, -235.344, 246.543, 6.010, -11.535, -176.787);
        DescPose desc_pos2 = new DescPose(-273.710, -100.871, 280.935, 5.692, 9.522, 179.512);
        DescPose desc_pos3 = new DescPose(-566.093, 311.278, 215.008, -10.453, -17.486, -174.209);
        DescPose desc_pos4 = new DescPose(-246.558, 328.240, 292.173, 13.912, 4.437, -179.067);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        int tool = 2;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 20.0f;
        float blendT = 0.0f;
        float blendR = 0.0f;
        byte flag = 0;

        robot.SetSpeed(1);

        // Move to starting point j1
        rtn = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, 100.0f, epos, blendT, flag, offset_pos);
        Console.WriteLine("MoveJ to j1 rtn: {0}", rtn);

        // Weave + spline curve motion
        robot.WeaveStart(0);
        robot.NewSplineStart(0, 6000);
        robot.NewSplinePoint(j1, desc_pos1, tool, user, vel, acc, ovl, -1.0f, 0);
        robot.NewSplinePoint(j2, desc_pos2, tool, user, vel, acc, ovl, -1.0f, 0);
        robot.NewSplinePoint(j3, desc_pos3, tool, user, vel, acc, ovl, -1.0f, 0);
        robot.NewSplinePoint(j4, desc_pos4, tool, user, vel, acc, ovl, -1.0f, 1);
        robot.NewSplineEnd();

        Console.WriteLine("TestSplineWeave completed");
    }