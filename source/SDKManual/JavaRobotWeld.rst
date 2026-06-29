Robot Welding  
=================

.. toctree::  
    :maxdepth: 5  


Set welding process curve parameters  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set welding process curve parameters  
    * @param [in] id Welding process ID (1-99)  
    * @param [in] param Welding process parameters  
    * @return Error code  
    */  
    int WeldingSetProcessParam(int id, WeldingProcessParam param);  

Get welding process curve parameters  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get welding process curve parameters  
    * @param [in] id Welding process ID (1-99)  
    * @param [out] param Welding process parameters  
    * @return Error code  
    */  
    int WeldingGetProcessParam(int id, WeldingProcessParam param);  

Set welding current to analog output relation  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set welding current to analog output relation  
    * @param [in] relation Relation value  
    * @return Error code  
    */  
    int WeldingSetCurrentRelation(WeldCurrentAORelation relation);  

Set welding voltage to analog output relation  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set welding voltage to analog output relation  
    * @param [in] relation Welding voltage-analog output relation value  
    * @return Error code  
    */  
    int WeldingSetVoltageRelation(WeldVoltageAORelation relation);  

Get welding current to analog output relation  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get welding current to analog output relation  
    * @param [out] relation Relation value  
    * @return Error code  
    */  
    int WeldingGetCurrentRelation(WeldCurrentAORelation relation);  

Get welding voltage to analog output relation  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get welding voltage to analog output relation  
    * @param [out] relation Welding voltage-analog output relation value  
    * @return Error code  
    */  
    int WeldingGetVoltageRelation(WeldVoltageAORelation relation);  

Set welding current  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set welding current  
    * @param [in] ioType Control IO type: 0-Controller IO; 1-Extension IO  
    * @param [in] current Welding current value (A)  
    * @param [in] AOIndex Controller analog output port (0-1)  
    * @param [in] blend Smoothing: 0-No smoothing; 1-Smoothing  
    * @return Error code  
    */  
    int WeldingSetCurrent(int ioType, double current, int AOIndex, int blend);  

Set welding voltage  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set welding voltage  
    * @param [in] ioType Control IO type: 0-Controller IO; 1-Extension IO  
    * @param [in] voltage Welding voltage value (V)  
    * @param [in] AOIndex Controller analog output port (0-1)  
    * @param [in] blend Smoothing: 0-No smoothing; 1-Smoothing  
    * @return Error code  
    */  
    int WeldingSetVoltage(int ioType, double voltage, int AOIndex, int blend);  

Set weaving parameters  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Java SDK-v1.0.5-3.8.2  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set weaving parameters  
    * @param [in] weaveNum Weaving parameter configuration ID  
    * @param [in] weaveType Weaving type:  
    *   0-Horizontal triangular wave  
    *   1-Vertical L-shaped triangular wave  
    *   2-Clockwise circular  
    *   3-Counter-clockwise circular  
    *   4-Horizontal sine wave  
    *   5-Vertical L-shaped sine wave  
    *   6-Vertical triangular wave  
    *   7-Vertical sine wave  
    * @param [in] weaveFrequency Weaving frequency (Hz)  
    * @param [in] weaveIncStayTime Wait mode:  
    *   0-Cycle excludes wait time  
    *   1-Cycle includes wait time  
    * @param [in] weaveRange Weaving amplitude (mm)  
    * @param [in] weaveLeftRange Vertical triangular wave left chord length (mm)  
    * @param [in] weaveRightRange Vertical triangular wave right chord length (mm)  
    * @param [in] additionalStayTime Vertical triangular wave apex dwell time (ms)  
    * @param [in] weaveLeftStayTime Left dwell time (ms)  
    * @param [in] weaveRightStayTime Right dwell time (ms)  
    * @param [in] weaveCircleRadio Circular weaving callback ratio (0-100%)  
    * @param [in] weaveStationary Position wait:  
    *   0-Continue moving during wait  
    *   1-Stationary during wait  
    * @param [in] weaveYawAngle Weaving direction azimuth angle (around Z-axis) in °  
    * @param [in] weaveRotAngle Weaving direction azimuth angle (around X-axis) in °  
    * @return Error code  
    */  
    int WeaveSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, double weaveLeftRange, double weaveRightRange, int additionalStayTime, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary, double weaveYawAngle, double weaveRotAngle);  

Welding parameter setting code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestSetWeldParam(Robot robot)  
    {  
        WeldingProcessParam para1 = new WeldingProcessParam(177, 27, 1000, 178, 28, 176, 26, 1000);  
        WeldingProcessParam para2 = new WeldingProcessParam(188, 28, 555, 199, 29, 133, 23, 333);  

        robot.WeldingSetProcessParam(1, para1);  
        robot.WeldingSetProcessParam(2, para2);  

        double startCurrent = 0;  
        double startVoltage = 0;  
        int startTime = 0;  
        double weldCurrent = 0;  
        double weldVoltage = 0;  
        double endCurrent = 0;  
        double endVoltage = 0;  
        int endTime = 0;  

        WeldingProcessParam param = new WeldingProcessParam(startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);  
        robot.WeldingGetProcessParam(1, param);  
        robot.WeldingGetProcessParam(2, param);  

        WeldCurrentAORelation rela1 = new WeldCurrentAORelation(0, 400, 0, 10, 0);  
        int rtn = robot.WeldingSetCurrentRelation(rela1);  

        WeldVoltageAORelation rela2 = new WeldVoltageAORelation(0, 40, 0, 10, 1);  
        rtn = robot.WeldingSetVoltageRelation(rela2);  

        double current_min = 0;  
        double current_max = 0;  
        double vol_min = 0;  
        double vol_max = 0;  
        double output_vmin = 0;  
        double output_vmax = 0;  
        int curIndex = 0;  
        int volIndex = 0;  
        WeldCurrentAORelation rela3 = new WeldCurrentAORelation(current_min, current_max, output_vmin, output_vmax, curIndex);  
        rtn = robot.WeldingGetCurrentRelation(rela3);  

        WeldVoltageAORelation rela4 = new WeldVoltageAORelation(0, 0, 0, 0, 0);  
        rtn = robot.WeldingGetVoltageRelation(rela4);  

        rtn = robot.WeldingSetCurrent(0, 100, 0, 0);  

        robot.Sleep(3000);  

        rtn = robot.WeldingSetVoltage(0, 10, 0, 0);  

        rtn = robot.WeaveSetPara(0, 0, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 60.000000, 0);  

        robot.WeaveOnlineSetPara(0, 0, 1, 0, 20, 0, 0, 0, 0);  

        rtn = robot.WeldingSetCheckArcInterruptionParam(1, 200);  
        rtn = robot.WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0);  
        int enable = 0;  
        double length = 0;  
        double velocity = 0;  
        int moveType = 0;  
        int checkEnable = 0;  
        int arcInterruptTimeLength = 0;  
        List<Integer> inter = new ArrayList<>();  
        List<Number> num = new ArrayList<>();  

        inter = robot.WeldingGetCheckArcInterruptionParam();  
        num = robot.WeldingGetReWeldAfterBreakOffParam();  

        robot.SetWeldMachineCtrlModeExtDoNum(17);  
        for (int i = 0; i < 5; i++) {  
            robot.SetWeldMachineCtrlMode(0);  
            robot.Sleep(1000);  
            robot.SetWeldMachineCtrlMode(1);  
            robot.Sleep(1000);  
        }  
        return 0;  
    }  

Real-time weaving parameter setting  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Real-time weaving parameter setting  
    * @param [in] weaveNum Weaving parameter configuration ID  
    * @param [in] weaveType Weaving type:  
    *   0-Horizontal triangular wave  
    *   1-Vertical L-shaped triangular wave  
    *   2-Clockwise circular  
    *   3-Counter-clockwise circular  
    *   4-Horizontal sine wave  
    *   5-Vertical L-shaped sine wave  
    *   6-Vertical triangular wave  
    *   7-Vertical sine wave  
    * @param [in] weaveFrequency Weaving frequency (Hz)  
    * @param [in] weaveIncStayTime Wait mode:  
    *   0-Cycle excludes wait time  
    *   1-Cycle includes wait time  
    * @param [in] weaveRange Weaving amplitude (mm)  
    * @param [in] weaveLeftStayTime Left dwell time (ms)  
    * @param [in] weaveRightStayTime Right dwell time (ms)  
    * @param [in] weaveCircleRadio Circular weaving callback ratio (0-100%)  
    * @param [in] weaveStationary Position wait:  
    *   0-Continue moving during wait  
    *   1-Stationary during wait  
    * @return Error code  
    */  
    int WeaveOnlineSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary);  

Set welding arc interruption detection parameters  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set welding arc interruption detection parameters  
    * @param [in] checkEnable Enable detection: 0-Disable; 1-Enable  
    * @param [in] arcInterruptTimeLength Arc interruption confirmation duration (ms)  
    * @return Error code  
    */  
    int WeldingSetCheckArcInterruptionParam(int checkEnable, int arcInterruptTimeLength);  

Get welding arc interruption detection parameters  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get welding arc interruption detection parameters  
    * @return List[0]: Error code; List[1]: double Detection enable (0-Disable; 1-Enable); List[2]: Arc interruption confirmation duration (ms)  
    */  
    List<Integer> WeldingGetCheckArcInterruptionParam();  

Set welding interruption recovery parameters  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set welding interruption recovery parameters  
    * @param [in] enable Enable welding recovery  
    * @param [in] length Weld overlap distance (mm)  
    * @param [in] velocity Robot return speed percentage (0-100)  
    * @param [in] moveType Robot movement type to restart point: 0-LIN; 1-PTP  
    * @return Error code  
    */  
    int WeldingSetReWeldAfterBreakOffParam(int enable, double length, double velocity, int moveType);  

Get welding interruption recovery parameters  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get welding interruption recovery parameters  
    * @return List[0]: Error code; List[1]: int Enable welding recovery; List[2]: double Weld overlap distance (mm);  
    * @return List[3]: double Robot return speed percentage (0-100); List[4]: int Robot movement type: 0-LIN; 1-PTP  
    */  
    List<Number> WeldingGetReWeldAfterBreakOffParam();  

Set welder control mode extension DO port  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set welder control mode extension DO port  
    * @param [in] DONum Welder control mode DO port (0-127)  
    * @return Error code  
    */  
    int SetWeldMachineCtrlModeExtDoNum(int DONum);  

Set welder control mode  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**
    * @brief Set welding machine control mode
    * @param mode Welding machine control mode; 0-DC one-knob mode; 1-Pulse one-knob mode; 2-JOB mode; 3-Local control mode; 4-Separate mode; 5-CC/CV mode; 6-TIG; 7-CMT
    * @param ioType Control type; 0-Control box IO; 1-Digital communication protocol (UDP); 2-Digital communication protocol (ModbusTCP)
    * @return Error code
    */
    public int SetWeldMachineCtrlMode(int mode, int ioType)

Welding start  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Welding start  
    * @param [in] ioType IO type: 0-Controller IO; 1-Extension IO  
    * @param [in] arcNum Welder configuration file ID  
    * @param [in] timeout Arc start timeout  
    * @return Error code  
    */  
    int ARCStart(int ioType, int arcNum, int timeout);  

Welding end  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Welding end  
    * @param [in] ioType IO type: 0-Controller IO; 1-Extension IO  
    * @param [in] arcNum Welder configuration file ID  
    * @param [in] timeout Arc end timeout  
    * @return Error code  
    */  
    int ARCEnd(int ioType, int arcNum, int timeout);  

Weaving start  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Weaving start  
    * @param [in] weaveNum Weaving parameter configuration ID  
    * @return Error code  
    */  
    int WeaveStart(int weaveNum);  

Weaving end  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Weaving end  
    * @param [in] weaveNum Weaving parameter configuration ID  
    * @return Error code  
    */  
    int WeaveEnd(int weaveNum);  

Forward wire feeding  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Forward wire feeding  
    * @param [in] ioType IO type: 0-Controller IO; 1-Extension IO  
    * @param [in] wireFeed Wire feed control: 0-Stop; 1-Feed  
    * @return Error code  
    */  
    int SetForwardWireFeed(int ioType, int wireFeed);  

Reverse wire feeding  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Reverse wire feeding  
    * @param [in] ioType IO type: 0-Controller IO; 1-Extension IO  
    * @param [in] wireFeed Wire feed control: 0-Stop; 1-Feed  
    * @return Error code  
    */  
    int SetReverseWireFeed(int ioType, int wireFeed);  

Gas feeding  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Gas feeding  
    * @param [in] ioType IO type: 0-Controller IO; 1-Extension IO  
    * @param [in] airControl Gas control: 0-Stop; 1-Feed  
    * @return Error code  
    */  
    int SetAspirated(int ioType, int airControl);  

Set robot to resume welding after interruption  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set robot to resume welding after interruption  
    * @return Error code  
    */  
    int WeldingStartReWeldAfterBreakOff();  

Set robot to abort welding after interruption  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set robot to abort welding after interruption  
    * @return Error code  
    */  
    int WeldingAbortWeldAfterBreakOff();  

Robot welding control code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestWelding(Robot robot)  
    {  
        robot.WeldingSetCurrent(0, 230, 0, 0);  
        robot.WeldingSetVoltage(0, 24, 0, 1);  

        DescPose p1Desc = new DescPose(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);  
        JointPos p1Joint = new JointPos(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);  

        DescPose p2Desc = new DescPose(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);  
        JointPos p2Joint = new JointPos(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);  

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);  
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);  

        robot.MoveJ(p1Joint, p1Desc, 13, 0, 20, 100, 100, exaxisPos, -1, 0, offdese);  
        robot.ARCStart(1, 0, 10000);  
        robot.WeaveStart(0);  
        robot.MoveL(p2Joint, p2Desc, 13, 0, 20, 100, 100, -1, 0, exaxisPos, 0, 0, offdese, 0, 10);  
        robot.ARCEnd(1, 0, 10000);  
        robot.WeaveEnd(0);  
        return 0;  
    }  

Segment welding start  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Segment welding start  
    * @param [in] startDesePos Start point Cartesian position  
    * @param [in] endDesePos End point Cartesian pose  
    * @param [in] startJPos Start point joint pose  
    * @param [in] endJPos End point joint pose  
    * @param [in] weldLength Weld segment length (mm)  
    * @param [in] noWeldLength Non-weld segment length (mm)  
    * @param [in] weldIOType Welding IO type (0-Controller IO; 1-Extension IO)  
    * @param [in] arcNum Welder configuration file ID  
    * @param [in] weldTimeout Arc start/end timeout  
    * @param [in] isWeave Enable weaving  
    * @param [in] weaveNum Weaving parameter configuration ID  
    * @param [in] tool Tool number  
    * @param [in] user Workpiece number  
    * @param [in] vel Speed percentage (0~100)  
    * @param [in] acc Acceleration percentage (0~100) (Not currently available)  
    * @param [in] ovl Speed scaling factor (0~100)  
    * @param [in] blendR [-1.0]-Move to position (blocking); [0~1000.0]-Smoothing radius (non-blocking) in mm  
    * @param [in] epos External axis position in mm  
    * @param [in] search 0-No wire search; 1-Wire search  
    * @param [in] offset_flag 0-No offset; 1-Offset in base/workpiece frame; 2-Offset in tool frame  
    * @param [in] offset_pos Pose offset  
    * @return Error code  
    */  
    int SegmentWeldStart(DescPose startDesePos, DescPose endDesePos, JointPos startJPos, JointPos endJPos, double weldLength, double noWeldLength, int weldIOType, int arcNum, int weldTimeout, boolean isWeave, int weaveNum, int tool, int user, double vel, double acc, double ovl, double blendR, ExaxisPos epos, int search, int offset_flag, DescPose offset_pos);  

Robot segment welding code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestSegWeld(Robot robot)  
    {  
        robot.WeldingSetCurrent(0, 230, 0, 0);  
        robot.WeldingSetVoltage(0, 24, 0, 1);  

        DescPose p1Desc = new DescPose(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);  
        JointPos p1Joint = new JointPos(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);  

        DescPose p2Desc = new DescPose(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);  
        JointPos p2Joint = new JointPos(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);  

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);  
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);  

        robot.GetForwardKin(p1Joint, p1Desc);  
        robot.GetForwardKin(p2Joint, p2Desc);  

        int rtn = robot.SegmentWeldStart(p1Desc, p2Desc, p1Joint, p2Joint, 20, 20, 0, 0, 5000, true, 0, 1, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese);  
        return 0;  
    }  

Simulation weaving start  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Simulation weaving start  
    * @param [in] weaveNum Weaving parameter ID  
    * @return Error code  
    */  
    int WeaveStartSim(int weaveNum);  

Simulation weaving end  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Simulation weaving end  
    * @param [in] weaveNum Weaving parameter ID  
    * @return Error code  
    */  
    int WeaveEndSim(int weaveNum);  

Start trajectory inspection warning (no movement)  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Start trajectory inspection warning (no movement)  
    * @param [in] weaveNum Weaving parameter ID  
    * @return Error code  
    */  
    int WeaveInspectStart(int weaveNum);  

End trajectory inspection warning (no movement)  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief End trajectory inspection warning (no movement)  
    * @param [in] weaveNum Weaving parameter ID  
    * @return Error code  
    */  
    int WeaveInspectEnd(int weaveNum);  

Weaving transition start  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Java SDK-v1.0.5-3.8.2  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Weaving transition start  
    * @param [in] weaveChangeFlag 1-Change weaving parameters; 2-Change weaving parameters + welding speed  
    * @param [in] weaveNum Weaving ID  
    * @param [in] velStart Welding start speed (cm/min)  
    * @param [in] velEnd Welding end speed (cm/min)  
    * @return Error code  
    */  
    int WeaveChangeStart(int weaveChangeFlag, int weaveNum, double velStart, double velEnd);  

Robot weaving transition welding code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestWeave(Robot robot)  
    {  
        DescPose p1Desc = new DescPose(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);  
        JointPos p1Joint = new JointPos(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);  

        DescPose p2Desc = new DescPose(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);  
        JointPos p2Joint = new JointPos(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);  

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);  
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);  

        robot.MoveJ(p1Joint, p1Desc, 13, 0, 20, 100, 100, exaxisPos, -1, 0, offdese);  
        robot.WeaveStartSim(0);  
        robot.MoveL(p2Joint, p2Desc, 13, 0, 20, 100, 100, -1, 0, exaxisPos, 0, 0, offdese, 0, 10);  
        robot.WeaveEndSim(0);  
        robot.MoveJ(p1Joint, p1Desc, 13, 0, 20, 100, 100, exaxisPos, -1, 0, offdese);  
        robot.WeaveInspectStart(0);  
        robot.MoveL(p2Joint, p2Desc, 13, 0, 20, 100, 100, -1, 0, exaxisPos, 0, 0, offdese, 0, 10);  
        robot.WeaveInspectEnd(0);  

        robot.WeldingSetVoltage(1, 19, 0, 0);  
        robot.WeldingSetCurrent(1, 190, 0, 0);  
        robot.MoveL(p1Joint, p1Desc, 1, 1, 100, 100, 50, -1, 0, exaxisPos, 0, 0, offdese, 0, 10);  
        robot.ARCStart(1, 0, 10000);  
        robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);  
        robot.WeaveStart(0);  
        robot.WeaveChangeStart(1, 0, 50, 30);  
        robot.MoveL(p2Joint, p2Desc, 1, 1, 100, 100, 1, -1, 0, exaxisPos, 0, 0, offdese, 0, 10);  
        robot.WeaveChangeEnd();  
        robot.WeaveEnd(0);  
        robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);  
        robot.ARCEnd(1, 0, 10000);  
        return 0;  
    }  

Weaving transition end  
+++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.2-3.7.9  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Weaving transition end  
    * @return Error code  
    */  
    int WeaveChangeEnd();  

Extension IO - Configure welder gas detection signal  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Extension IO - Configure welder gas detection signal  
    * @param [in] DONum Gas detection signal extension DO number  
    * @return Error code  
    */  
    int SetAirControlExtDoNum(int DONum);  

Extension IO - Configure welder arc start signal  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Extension IO - Configure welder arc start signal  
    * @param [in] DONum Welder arc start signal extension DO number  
    * @return Error code  
    */  
    int SetArcStartExtDoNum(int DONum);  

Extension IO - Configure welder reverse wire feed signal  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Extension IO - Configure welder reverse wire feed signal  
    * @param [in] DONum Reverse wire feed signal extension DO number  
    * @return Error code  
    */  
    int SetWireReverseFeedExtDoNum(int DONum);  

Extension IO - Configure welder forward wire feed signal  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Extension IO - Configure welder forward wire feed signal  
    * @param [in] DONum Forward wire feed signal extension DO number  
    * @return Error code  
    */  
    int SetWireForwardFeedExtDoNum(int DONum);  

Extension IO - Configure welder arc success signal  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Extension IO - Configure welder arc success signal  
    * @param [in] DINum Arc success signal extension DI number  
    * @return Error code  
    */  
    int SetArcDoneExtDiNum(int DINum);  

Extension IO - Configure welder ready signal  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Extension IO - Configure welder ready signal  
    * @param [in] DINum Welder ready signal extension DI number  
    * @return Error code  
    */  
    int SetWeldReadyExtDiNum(int DINum);  

Extension IO - Configure welding interruption recovery signal  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Extension IO - Configure welding interruption recovery signal  
    * @param [in] reWeldDINum Resume welding after interruption signal extension DI number  
    * @param [in] abortWeldDINum Abort welding after interruption signal extension DI number  
    * @return Error code  
    */  
    int SetExtDIWeldBreakOffRecover(int reWeldDINum, int abortWeldDINum);  

Set extension IO welding signal code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestExtDIConfig(Robot robot)  
    {  
        robot.SetArcStartExtDoNum(10);  
        robot.SetAirControlExtDoNum(20);  
        robot.SetWireForwardFeedExtDoNum(30);  
        robot.SetWireReverseFeedExtDoNum(40);  

        robot.SetWeldReadyExtDiNum(50);  
        robot.SetArcDoneExtDiNum(60);  
        robot.SetExtDIWeldBreakOffRecover(70, 80);  
        robot.SetWireSearchExtDIONum(0, 1);  

        return 0;  
    }  

Arc tracking control  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Java SDK-v1.0.2-3.7.9  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Arc tracking control  
    * @param [in] flag Switch: 0-Off; 1-On  
    * @param [in] delaytime Lag time in ms  
    * @param [in] isLeftRight Left/right deviation compensation  
    * @param [in] klr Left/right adjustment coefficient (sensitivity)  
    * @param [in] tStartLr Left/right start compensation time in cycles  
    * @param [in] stepMaxLr Left/right maximum compensation per step in mm  
    * @param [in] sumMaxLr Left/right total maximum compensation in mm  
    * @param [in] isUpLow Up/down deviation compensation  
    * @param [in] kud Up/down adjustment coefficient (sensitivity)  
    * @param [in] tStartUd Up/down start compensation time in cycles  
    * @param [in] stepMaxUd Up/down maximum compensation per step in mm  
    * @param [in] sumMaxUd Up/down total maximum compensation in mm  
    * @param [in] axisSelect Up/down coordinate system selection: 0-Weaving; 1-Tool; 2-Base  
    * @param [in] referenceType Up/down reference current setting method: 0-Feedback; 1-Constant  
    * @param [in] referSampleStartUd Up/down reference current sampling start count (feedback) in cycles  
    * @param [in] referSampleCountUd Up/down reference current sampling cycle count (feedback) in cycles  
    * @param [in] referenceCurrent Up/down reference current in mA  
    * @param [in] offsetType Offset tracking type: 0-No offset; 1-Sampling; 2-Percentage  
    * @param [in] offsetParameter Offset parameters:  
    *   Sampling (offset sampling start time, default one cycle)  
    *   Percentage (offset percentage (-100 ~ 100))  
    * @return Error code  
    */  
    int ArcWeldTraceControl(int flag, double delaytime, int isLeftRight, double klr, double tStartLr, double stepMaxLr, double sumMaxLr, int isUpLow, double kud, double tStartUd, double stepMaxUd, double sumMaxUd, int axisSelect, int referenceType, double referSampleStartUd, double referSampleCountUd, double referenceCurrent, int offsetType, int offsetParameter);  

Arc tracking AI channel selection  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Arc tracking AI channel selection  
    * @param channel Arc tracking AI channel selection [0-3]  
    * @return Error code  
    */  
    public int ArcWeldTraceExtAIChannelConfig(int channel);  

Arc tracking + multi-layer multi-pass compensation start  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Arc tracking + multi-layer multi-pass compensation start  
    * @return Error code  
    */  
    public int ArcWeldTraceReplayStart();  

Arc tracking + multi-layer multi-pass compensation end  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Arc tracking + multi-layer multi-pass compensation end  
    * @return Error code  
    */  
    public int ArcWeldTraceReplayEnd();  

Offset coordinate transformation - multi-layer multi-pass welding  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Offset coordinate transformation - multi-layer multi-pass welding  
    * @param pointO Reference point Cartesian pose  
    * @param pointX Reference point X-direction offset point Cartesian pose  
    * @param pointZ Reference point Z-direction offset point Cartesian pose  
    * @param dx X-direction offset (mm)  
    * @param dz Z-direction offset (mm)  
    * @param dry Rotation around Y-axis offset (°)  
    * @param offset Calculated offset  
    * @return Error code  
    */  
    public int MultilayerOffsetTrsfToBase(DescTran pointO, DescTran pointX, DescTran pointZ, double dx, double dz, double dry, DescPose offset);  

Multi-layer multi-pass welding arc tracking code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestArcWeldTrace(Robot robot)  
    {  
        JointPos mulitilineorigin1_joint = new JointPos(-24.090, -63.501, 84.288, -111.940, -93.426, 57.669);  
        DescPose mulitilineorigin1_desc = new DescPose(-677.559, 190.951, -1.205, 1.144, -41.482, -82.577);  

        DescTran mulitilineX1_desc = new DescTran(0, 0, 0);  
        mulitilineX1_desc.x = -677.556;  
        mulitilineX1_desc.y = 211.949;  
        mulitilineX1_desc.z = -1.206;  

        DescTran mulitilineZ1_desc = new DescTran(0, 0, 0);  
        mulitilineZ1_desc.x = -677.564;  
        mulitilineZ1_desc.y = 190.956;  
        mulitilineZ1_desc.z = 19.817;  

        JointPos mulitilinesafe_joint = new JointPos(-25.734, -63.778, 81.502, -108.975, -93.392, 56.021);  
        DescPose mulitilinesafe_desc = new DescPose(-677.561, 211.950, 19.812, 1.144, -41.482, -82.577);  
        JointPos mulitilineorigin2_joint = new JointPos(-29.743, -75.623, 101.241, -116.354, -94.928, 55.735);  
        DescPose mulitilineorigin2_desc = new DescPose(-563.961, 215.359, -0.681, 2.845, -40.476, -87.443);  

        DescTran mulitilineX2_desc = new DescTran(0, 0, 0);  
        mulitilineX2_desc.x = -563.965;  
        mulitilineX2_desc.y = 220.355;  
        mulitilineX2_desc.z = -0.680;  

        DescTran mulitilineZ2_desc = new DescTran(0, 0, 0);  
        mulitilineZ2_desc.x = -563.968;  
        mulitilineZ2_desc.y = 215.362;  
        mulitilineZ2_desc.z = 4.331;  

        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);  
        DescPose offset = new DescPose(0, 0, 0, 0, 0, 0);  

        robot.Sleep(10);  
        int error = robot.MoveJ(mulitilinesafe_joint, mulitilinesafe_desc, 13, 0, 10, 100, 100, epos, -1, 0, offset);  

        error = robot.MoveL(mulitilineorigin1_joint, mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, 0, epos, 0, 0, offset, 0, 100);  

        error = robot.MoveJ(mulitilinesafe_joint, mulitilinesafe_desc, 13, 0, 10, 100, 100, epos, -1, 0, offset);  

        error = robot.MoveL(mulitilineorigin2_joint, mulitilineorigin2_desc, 13, 0, 10, 100, 100, -1, 0, epos, 0, 0, offset, 0, 100);  

        error = robot.MoveJ(mulitilinesafe_joint, mulitilinesafe_desc, 13, 0, 10, 100, 100, epos, -1, 0, offset);  

        error = robot.MoveL(mulitilineorigin1_joint, mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, 0, epos, 0, 0, offset, 0, 100);  

        error = robot.ARCStart(1, 0, 3000);  

        error = robot.WeaveStart(0);  

        error = robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10, 0, 0);  

        error = robot.MoveL(mulitilineorigin2_joint, mulitilineorigin2_desc, 13, 0, 1, 100, 100, -1, 0, epos, 0, 0, offset, 0, 100);  

        error = robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10, 0, 0);  

        error = robot.WeaveEnd(0);  

        error = robot.ARCEnd(1, 0, 10000);  

        error = robot.MoveJ(mulitilinesafe_joint, mulitilinesafe_desc, 13, 0, 10, 100, 100, epos, -1, 0, offset);  

        error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 10.0, 0.0, 0.0, offset);  

        error = robot.MoveL(mulitilineorigin1_joint, mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, 0, epos, 0, 1, offset, 0, 100);  

        error = robot.ARCStart(1, 0, 3000);  

        error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 10, 0, 0, offset);  

        error = robot.ArcWeldTraceReplayStart();  

        error = robot.MoveL(mulitilineorigin2_joint, mulitilineorigin2_desc, 13, 0, 2, 100, 100, -1, 0, epos, 0, 1, offset, 0, 100);  

        error = robot.ArcWeldTraceReplayEnd();  

        error = robot.ARCEnd(1, 0, 10000);  

        error = robot.MoveJ(mulitilinesafe_joint, mulitilinesafe_desc, 13, 0, 10, 100, 100, epos, -1, 0, offset);  

        error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 0, 10, 0, offset);  

        error = robot.MoveL(mulitilineorigin1_joint, mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, 0, epos, 0, 1, offset, 0, 100);  

        error = robot.ARCStart(1, 0, 3000);  

        error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 0, 10, 0, offset);  

        error = robot.ArcWeldTraceReplayStart();  

        error = robot.MoveL(mulitilineorigin2_joint, mulitilineorigin2_desc, 13, 0, 2, 100, 100, -1, 0, epos, 0, 1, offset, 0, 100);  

        error = robot.ArcWeldTraceReplayEnd();  

        error = robot.ARCEnd(1, 0, 3000);  

        error = robot.MoveJ(mulitilinesafe_joint, mulitilinesafe_desc, 13, 0, 10, 100, 100, epos, -1, 0, offset);  

        robot.CloseRPC();  
        return 0;  
    }  

Arc tracking welder current feedback AI channel selection  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.5-3.8.2  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Arc tracking welder current feedback AI channel selection  
    * @param [in] channel Channel:  
    *   0-Extension AI0  
    *   1-Extension AI1  
    *   2-Extension AI2  
    *   3-Extension AI3  
    *   4-Controller AI0  
    *   5-Controller AI1  
    * @return Error code  
    */  
    int ArcWeldTraceAIChannelCurrent(int channel);  

Arc tracking welder voltage feedback AI channel selection  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.5-3.8.2  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Arc tracking welder voltage feedback AI channel selection  
    * @param [in] channel Channel:  
    *   0-Extension AI0  
    *   1-Extension AI1  
    *   2-Extension AI2  
    *   3-Extension AI3  
    *   4-Controller AI0  
    *   5-Controller AI1  
    * @return Error code  
    */  
    int ArcWeldTraceAIChannelVoltage(int channel);  

Arc tracking welder current feedback conversion parameters  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.5-3.8.2  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Arc tracking welder current feedback conversion parameters  
    * @param [in] AILow AI channel lower limit (default 0V, range [0-10V])  
    * @param [in] AIHigh AI channel upper limit (default 10V, range [0-10V])  
    * @param [in] currentLow AI channel lower limit corresponding welder current (default 0V, range [0-200V])  
    * @param [in] currentHigh AI channel upper limit corresponding welder current (default 100V, range [0-200V])  
    * @return Error code  
    */  
    int ArcWeldTraceCurrentPara(double AILow, double AIHigh, double currentLow, double currentHigh);  

Arc tracking welder voltage feedback conversion parameters  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.5-3.8.2  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Arc tracking welder voltage feedback conversion parameters  
    * @param [in] AILow AI channel lower limit (default 0V, range [0-10V])  
    * @param [in] AIHigh AI channel upper limit (default 10V, range [0-10V])  
    * @param [in] voltageLow AI channel lower limit corresponding welder voltage (default 0V, range [0-200V])  
    * @param [in] voltageHigh AI channel upper limit corresponding welder voltage (default 100V, range [0-200V])  
    * @return Error code  
    */  
    int ArcWeldTraceVoltagePara(double AILow, double AIHigh, double voltageLow, double voltageHigh);  

Arc tracking code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static void WeldTraceControlWithCtrlBoxAI(Robot robot)  
    {  
        DescPose startdescPose = new DescPose(-473.86, 257.879, -20.849, -37.317, -42.021, 2.543);  
        JointPos startjointPos = new JointPos(-43.487, -76.526, 95.568, -104.445, -89.356, 3.72);  

        DescPose safedescPose = new DescPose(-504.043, 275.181, 40.908, -28.002, -42.025, -14.044);  
        JointPos safejointPos = new JointPos(-39.078, -76.732, 87.227, -99.47, -94.301, 18.714);  

        DescPose enddescPose = new DescPose(-499.844, 141.225, 7.72, -34.856, -40.17, 13.13);  
        JointPos endjointPos = new JointPos(-31.305, -82.998, 99.401, -104.426, -89.35, 3.696);  

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);  
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);  
        // Move to safe point  
        robot.MoveJ(safejointPos, safedescPose, 1, 0, 5, 20, 100, exaxisPos, -1, 0, offdese);  

        WeldCurrentAORelation current = new WeldCurrentAORelation(0, 495, 1, 10, 0);  
        WeldVoltageAORelation voltage = new WeldVoltageAORelation(10, 45, 1, 10, 1);  
        robot.WeldingSetCurrentRelation(current); // Current to analog output relation  
        robot.WeldingSetVoltageRelation(voltage); // Voltage to analog output relation  
        robot.WeldingSetVoltage(0, 25, 1, 0); // Set voltage  
        robot.WeldingSetCurrent(0, 260, 0, 0); // Set current  

        int rtn = robot.ArcWeldTraceAIChannelCurrent(4);  
        System.out.println("ArcWeldTraceAIChannelCurrent rtn is " + rtn);  

        rtn = robot.ArcWeldTraceAIChannelVoltage(5);  
        System.out.println("ArcWeldTraceAIChannelVoltage rtn is " + rtn);  

        rtn = robot.ArcWeldTraceCurrentPara(0.0, 5, 0, 500);  
        System.out.println("ArcWeldTraceCurrentPara rtn is " + rtn);  

        rtn = robot.ArcWeldTraceVoltagePara(1.018, 10, 0, 50);  
        System.out.println("ArcWeldTraceVoltagePara rtn is " + rtn);  

        robot.MoveJ(startjointPos, startdescPose, 1, 0, 20, 20, 100, exaxisPos, -1, 0, offdese);  
        robot.ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);  
        robot.ARCStart(0, 0, 10000);  
        robot.WeaveStart(0);  
        robot.MoveL(endjointPos, enddescPose, 1, 0, 100, 100, 2, -1, 0, exaxisPos, 0, 0, offdese, 0, 10);  
        robot.ARCEnd(0, 0, 10000);  
        robot.WeaveEnd(0);  
        robot.ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);  

        robot.MoveJ(safejointPos, safedescPose, 1, 0, 20, 20, 100, exaxisPos, -1, 0, offdese);  
    }  

Set Wire Search Extension IO Ports
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Sets the wire search extension IO ports
    * @param [in] searchDoneDINum Wire search success DO port (0-127)
    * @param [in] searchStartDONum Wire search start/stop control DO port (0-127)
    * @return Error code
    */
    int SetWireSearchExtDIONum(int searchDoneDINum, int searchStartDONum);

Wire search start  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Wire search start  
    * @param [in] refPos 1-Reference point; 0-Contact point  
    * @param [in] searchVel Search speed %  
    * @param [in] searchDis Search distance mm  
    * @param [in] autoBackFlag Auto return flag: 0-No auto; 1-Auto  
    * @param [in] autoBackVel Auto return speed %  
    * @param [in] autoBackDis Auto return distance mm  
    * @param [in] offectFlag 1-Search with offset; 0-Teach point search  
    * @return Error code  
    */  
    int WireSearchStart(int refPos, double searchVel, int searchDis, int autoBackFlag, double autoBackVel, int autoBackDis, int offectFlag);  

Wire search end  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Wire search end  
    * @param [in] refPos 1-Reference point; 2-Contact point  
    * @param [in] searchVel Search speed %  
    * @param [in] searchDis Search distance mm  
    * @param [in] autoBackFlag Auto return flag: 0-No auto; 1-Auto  
    * @param [in] autoBackVel Auto return speed %  
    * @param [in] autoBackDis Auto return distance mm  
    * @param [in] offectFlag 1-Search with offset; 2-Teach point search  
    * @return Error code  
    */  
    int WireSearchEnd(int refPos, double searchVel, int searchDis, int autoBackFlag, double autoBackVel, int autoBackDis, int offectFlag);  

Calculate wire search offset  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Calculate wire search offset  
    * @param [in] seamType Weld seam type  
    * @param [in] method Calculation method  
    * @param [in] varNameRef Reference points 1-6, "#" indicates no point variable  
    * @param [in] varNameRes Contact points 1-6, "#" indicates no point variable  
    * @param [out] offset Offset pose [x, y, z, a, b, c] and offset method  
    * @return Error code  
    */  
    int GetWireSearchOffset(int seamType, int method, String[] varNameRef, String[] varNameRes, DescOffset offset);  

Wait for wire search completion  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Wait for wire search completion  
    * @return Error code  
    */  
    int WireSearchWait(String name);  

Write wire search contact point to database  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Write wire search contact point to database  
    * @param [in] varName Contact point name "RES0" ~ "RES99"  
    * @param [in] pos Contact point data [x, y, z, a, b, c]  
    * @return Error code  
    */  
    int SetPointToDatabase(String varName, DescPose pos);  

Robot wire search code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestWireSearch(Robot robot)  
    {  
        DescPose toolCoord = new DescPose(0, 0, 200, 0, 0, 0);  
        robot.SetToolCoord(1, toolCoord, 0, 0, 1, 0);  
        DescPose wobjCoord = new DescPose(0, 0, 0, 0, 0, 0);  
        robot.SetWObjCoord(1, wobjCoord, 0);  

        int rtn0, rtn1, rtn2 = 0;  
        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);  
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);  

        DescPose descStart = new DescPose(216.543, 445.175, 93.465, 179.683, 1.757, -112.641);  
        JointPos jointStart = new JointPos(-128.345, -86.660, 114.679, -119.625, -89.219, 74.303);  

        DescPose descEnd = new DescPose(111.143, 523.384, 87.659, 179.703, 1.835, -97.750);  
        JointPos jointEnd = new JointPos(-113.454, -81.060, 109.328, -119.954, -89.218, 74.302);  

        robot.MoveL(jointStart, descStart, 1, 1, 100, 100, 100, -1, 0, exaxisPos, 0, 0, offdese, 0, 100);  
        robot.MoveL(jointEnd, descEnd, 1, 1, 100, 100, 100, -1, 0, exaxisPos, 0, 0, offdese, 0, 100);  

        DescPose descREF0A = new DescPose(142.135, 367.604, 86.523, 179.728, 1.922, -111.089);  
        JointPos jointREF0A = new JointPos(-126.794, -100.834, 128.922, -119.864, -89.218, 74.302);  

        DescPose descREF0B = new DescPose(254.633, 463.125, 72.604, 179.845, 2.341, -114.704);  
        JointPos jointREF0B = new JointPos(-130.413, -81.093, 112.044, -123.163, -89.217, 74.303);  

        DescPose descREF1A = new DescPose(92.556, 485.259, 47.476, -179.932, 3.130, -97.512);  
        JointPos jointREF1A = new JointPos(-113.231, -83.815, 119.877, -129.092, -89.217, 74.303);  

        DescPose descREF1B = new DescPose(203.103, 583.836, 63.909, 179.991, 2.854, -103.372);  
        JointPos jointREF1B = new JointPos(-119.088, -69.676, 98.692, -121.761, -89.219, 74.303);  

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);  
        robot.MoveL(jointREF0A, descREF0A, 1, 1, 100, 100, 100, -1, 0, exaxisPos, 0, 0, offdese, 0, 10);  // Start point  
        robot.MoveL(jointREF0B, descREF0B, 1, 1, 100, 100, 100, -1, 0, exaxisPos, 1, 0, offdese, 0, 10);  // Direction point  
        rtn1 = robot.WireSearchWait("REF0");  
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);  

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);  
        robot.MoveL(jointREF1A, descREF1A, 1, 1, 100, 100, 100, -1, 0, exaxisPos, 0, 0, offdese, 0, 10);  // Start point  
        robot.MoveL(jointREF1B, descREF1B, 1, 1, 100, 100, 100, -1, 0, exaxisPos, 1, 0, offdese, 0, 10);  // Direction point  
        rtn1 = robot.WireSearchWait("REF1");  
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);  

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);  
        robot.MoveL(jointREF0A, descREF0A, 1, 1, 100, 100, 100, -1, 0, exaxisPos, 0, 0, offdese, 0, 10);  // Start point  
        robot.MoveL(jointREF0B, descREF0B, 1, 1, 100, 100, 100, -1, 0, exaxisPos, 1, 0, offdese, 0, 10);  // Direction point  
        rtn1 = robot.WireSearchWait("RES0");  
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);  

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);  
        robot.MoveL(jointREF1A, descREF1A, 1, 1, 100, 100, 100, -1, 0, exaxisPos, 0, 0, offdese, 0, 10);  // Start point  
        robot.MoveL(jointREF1B, descREF1B, 1, 1, 100, 100, 100, -1, 0, exaxisPos, 1, 0, offdese, 0, 10);  // Direction point  
        rtn1 = robot.WireSearchWait("RES1");  
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);  

        String[] varNameRef = new String[]{"REF0", "REF1", "#", "#", "#", "#"};  
        String[] varNameRes = new String[]{"RES0", "RES1", "#", "#", "#", "#"};  
        int offectFlag = 0;  

        DescPose pos = new DescPose(0, 0, 0, 0, 0, 0);  
        DescOffset offectPos = new DescOffset();  
        offectPos.offset = pos;  
        offectPos.offsetFlag = 0;  

        rtn0 = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes, offectPos);  
        robot.PointsOffsetEnable(0, pos);  
        robot.MoveL(jointStart, descStart, 1, 1, 100, 100, 100, -1, 0, exaxisPos, 0, 0, offdese, 0, 10);  
        robot.MoveL(jointEnd, descEnd, 1, 1, 100, 100, 100, -1, 0, exaxisPos, 1, 0, offdese, 0, 10);  
        robot.PointsOffsetDisable();  

        robot.CloseRPC();  
        return 0;  
    }  

Set welding voltage gradual change start  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.5-3.8.2  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set welding voltage gradual change start  
    * @param [in] IOType Control type:  
    *   0-Controller IO  
    *   1-Digital communication protocol (UDP)  
    *   2-Digital communication protocol (ModbusTCP)  
    * @param [in] voltageStart Starting welding voltage (V)  
    * @param [in] voltageEnd Ending welding voltage (V)  
    * @param [in] AOIndex Controller AO port number (0-1)  
    * @param [in] blend Smoothing: 0-No smoothing; 1-Smoothing  
    * @return Error code  
    */  
    int WeldingSetVoltageGradualChangeStart(int IOType, double voltageStart, double voltageEnd, int AOIndex, int blend);  

Set welding voltage gradual change end  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.5-3.8.2  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set welding voltage gradual change end  
    * @return Error code  
    */  
    int WeldingSetVoltageGradualChangeEnd();  

Set welding current gradual change start  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.5-3.8.2  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set welding current gradual change start  
    * @param [in] IOType Control type:  
    *   0-Controller IO  
    *   1-Digital communication protocol (UDP)  
    *   2-Digital communication protocol (ModbusTCP)  
    * @param [in] currentStart Starting welding current (A)  
    * @param [in] currentEnd Ending welding current (A)  
    * @param [in] AOIndex Controller AO port number (0-1)  
    * @param [in] blend Smoothing: 0-No smoothing; 1-Smoothing  
    * @return Error code  
    */  
    int WeldingSetCurrentGradualChangeStart(int IOType, double currentStart, double currentEnd, int AOIndex, int blend);  

Set welding current gradual change end  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.5-3.8.2  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set welding current gradual change end  
    * @return Error code  
    */  
    int WeldingSetCurrentGradualChangeEnd();  

Robot welding current/voltage gradual change code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static void WeldparamChange(Robot robot)  
    {  
        DescPose startdescPose = new DescPose(-484.707, 276.996, -14.013, -37.657, -40.508, -1.548);  
        JointPos startjointPos = new JointPos(-45.421, -75.673, 93.627, -104.302, -87.938, 6.005);  

        DescPose enddescPose = new DescPose(-508.767, 137.109, -13.966, -37.639, -40.508, -1.559);  
        JointPos endjointPos = new JointPos(-32.768, -80.947, 100.254, -106.201, -87.201, 18.648);  

        DescPose safedescPose = new DescPose(-484.709, 294.436, 13.621, -37.660, -40.508, -1.545);  
        JointPos safejointPos = new JointPos(-46.604, -75.410, 89.109, -100.003, -88.012, 4.823);  

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);  
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);  

        WeldCurrentAORelation cur = new WeldCurrentAORelation(0, 495, 1, 10, 0);  
        WeldVoltageAORelation vol = new WeldVoltageAORelation(10, 45, 1, 10, 1);
        robot.WeldingSetCurrentRelation(cur);
        robot.WeldingSetVoltageRelation(vol);

        robot.WeldingSetVoltage(0, 25, 1, 0);
        robot.WeldingSetCurrent(0, 260, 0, 0);

        robot.MoveJ(safejointPos, safedescPose, 1, 0, 5, 100, 100, exaxisPos, -1, 0, offdese);

        robot.WeldingSetCurrentGradualChangeStart(0, 260, 220, 0, 0);
        robot.WeldingSetVoltageGradualChangeStart(0, 25, 22, 1, 0);
        int rtn = robot.ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);

        robot.MoveJ(startjointPos, startdescPose, 1, 0, 5, 100, 100, exaxisPos, -1, 0, offdese);
        System.out.println("ArcWeldTraceControl rtn is " + rtn);

        robot.ARCStart(0, 0, 10000);
        robot.WeaveStart(0);
        robot.WeaveChangeStart(2, 1, 24, 36);
        robot.MoveL(endjointPos, enddescPose, 1, 0, 100, 100, 2, -1, 0, exaxisPos, 0, 0, offdese, 0, 10);
        robot.ARCEnd(0, 0, 10000);
        robot.WeaveChangeEnd();
        robot.WeaveEnd(0);
        robot.ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
        robot.WeldingSetCurrentGradualChangeEnd();
        robot.WeldingSetVoltageGradualChangeEnd();
    }

Set Custom Weaving Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
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
   public int CustomWeaveSetPara(int id, int pointNum, DescTran[] point, double[] stayTime, double frequency, int incStayType, int stationary)

Get Custom Weaving Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: java
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
   public int CustomWeaveGetPara(int id, int[] pointNum, DescTran[] point, double[] stayTime, double[] frequency, int[] incStayType, int[] stationary)

Custom Weaving Parameters Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: java
   :linenos:

   public static void TestCustomWeaveSetPara(Robot robot)
   {
       DescTran[] point = new DescTran[10];
       point[0]=new DescTran();
       point[0].x = -3;
       point[0].y = -3;
       point[0].z = 0;

       point[1]=new DescTran();
       point[1].x = -6;
       point[1].y = 0;
       point[1].z = 0;

       point[2]=new DescTran();
       point[2].x = -3;
       point[2].y = 3;
       point[2].z = 0;

       point[3]=new DescTran();
       point[3].x = 0;
       point[3].y = 0;
       point[3].z = 0;
       point[4]=new DescTran(0,0,0);
       point[5]=new DescTran(0,0,0);
       point[6]=new DescTran(0,0,0);
       point[7]=new DescTran(0,0,0);
       point[8]=new DescTran(0,0,0);
       point[9]=new DescTran(0,0,0);

       double[] stayTime =new double[] { 0,0,0,0,0,0,0,0,0,0 };
       int rtn = robot.CustomWeaveSetPara(2, 4, point, stayTime, 1.000, 0, 0);
       System.out.println("CustomWeaveSetPara rtn is: "+ rtn);
       robot.Sleep(1000);

       int[] pointNum = new int[1];
       double[] frequency=new double[1];
       int[] incStayType=new int[1];
       int[] stationary=new int[1];
       robot.CustomWeaveGetPara(2, pointNum, point, stayTime, frequency, incStayType, stationary);
       System.out.println("pointNum is: "+ pointNum[0]);
       for (int i = 0; i < pointNum[0]; i++)
       {
           System.out.println("point: "+i+", "+ point[i].x+", "+ point[i].y+", "+ point[i].z);
       }
       System.out.println("frequency is: "+ frequency[0]+", incStayType is: "+ incStayType[0]+", stationary is: "+ stationary[0]);

       robot.WeaveSetPara(0, 9, 1.000000, 1, 5.000000,
               6.000000, 5.000000, 50, 100, 100,
               0, 1, 0.000000, 0.000000);

       DescPose desc_p1 =new DescPose(-288.650, 367.807, 288.404, 0.000, -0.001, 0.001 );
       DescPose desc_p2 = new DescPose( -431.714, 367.815, 288.415, 0.001, 0.001, 0.000 );
       DescPose desc_p3 = new DescPose( -348.666, 427.798, 288.404, -0.000, -0.000, 0.001 );
       JointPos j1 = new JointPos( 140.656, -84.560, -91.707, -93.734, 90.000, 50.655 );
       JointPos j2 = new JointPos( 149.873, -98.298, -77.599, -94.103, 90.000, 59.873 );
       JointPos j3 = new JointPos( 139.773, -96.173, -80.014, -93.814, 90.000, 49.772 );

       ExaxisPos epos = new ExaxisPos();
       DescPose offset_pos = new DescPose();

       robot.MoveJ(j1, desc_p1, 3, 0, 100, 100,100, epos, -1, 0, offset_pos);
       robot.WeaveStart(0);
       robot.Circle(j3, desc_p3, 3, 0, 100, 100, epos, j2, desc_p2, 3, 0, 100, 100, epos, 10, -1, offset_pos,0,-1,0);
       robot.WeaveEnd(0);
       robot.MoveJ(j1, desc_p1, 3, 0, 100, 100, 100, epos, -1, 0, offset_pos);
       robot.WeaveStart(0);
       robot.MoveC(j3, desc_p3, 3, 0, 100, 100, epos, 0, offset_pos, j2, desc_p2, 3, 0, 100, 100, epos, 0, offset_pos, 10, -1,0);
       robot.WeaveEnd(0);
       robot.MoveJ(j1, desc_p1, 3, 0, 100, 100, 100, epos, -1, 0, offset_pos);
       robot.WeaveStart(0);
       robot.MoveL(j2, desc_p2, 3, 0, 100, 100, 10, -1,epos, 0, 0, offset_pos, 0,0, 100);
       robot.WeaveEnd(0);

       robot.CloseRPC();
   }

Laser Welding Machine Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Laser welding machine parameter configuration
    * @param  io_type Communication type 0-IO 1-UDP
    * @param  num Group number to set (1~10)
    * @param  scanSpeed Scanning speed
    * @param  scanWidth Scanning width
    * @param  peakPower Peak power
    * @param  dutyCycle Duty cycle
    * @param  freq Frequency
    * @return Error code
    */
    public int SetLaserWeldingParam(int io_type, int num, int scanSpeed, int scanWidth, int peakPower, int dutyCycle, int freq);

Start/Stop Laser Welding
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Start/stop laser welding
    * @param io_type Communication type 0-IO 1-UDP
    * @param status Control word 0-laser off 1-laser on
    * @param max_waittime Maximum wait time in milliseconds, default 10000
    * @return Error code
    */
    public int SetLaserWeldingStartEnd(int io_type, int status, int max_waittime)

Enable/Disable Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Enable/disable laser welding machine
    * @param io_type Communication type 0-IO 1-UDP
    * @param status 0-disable 1-enable
    * @return Error code
    */
    public int SetLaserWeldingEnable(int io_type, int status)

Laser Welding Machine Fault Reset
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Laser welding machine fault reset
    * @param io_type Communication type 0-IO 1-UDP
    * @param status Control word 0-invalid 1-fault reset
    * @return Error code
    */
    public int ResetLaserWeldingErr(int io_type, int status)

Get Laser Welding Machine Running Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get laser welding machine running status
    * @param io_type Communication type 0-IO 1-UDP
    * @param  status Control word 0-stopped 1-running
    * @return Error code
    */
    public int GetLaserWeldingRunningState(int io_type, int[] status)

Get Laser Welding Machine Fault Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get laser welding machine fault status
    * @param io_type Communication type 0-IO 1-UDP
    * @param  status 0-no fault 1-fault present
    * @return Error code
    */
    public int GetLaserWeldingErrState(int io_type, int[] status)

Get Laser Welding Machine Configuration Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get configuration parameters of one of the 10 process groups of the laser welding machine
    * @param num Group number to set (1~10)
    * @param params Output parameter array: [scanSpeed, scanWidth, peakPower, dutyCycle, freq]
    * @return Error code
    */
    public int GetLaserWeldingParamTarget(int num, int[] params)

Get Currently Active Configuration Parameters of Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get currently active configuration parameters of the laser welding machine
    * @param io_type Communication type 0-IO 1-UDP
    * @param params Output parameter array: [scanSpeed, scanWidth, peakPower, dutyCycle, freq]
    * @return Error code
    */
    public int GetLaserWeldingParamActual(int io_type, int[] params)

Configure Extended IO Enable DO Port for Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Configure extended IO enable DO port for laser welding machine
    * @param ctrlModeDONum Extended DO port number for laser welding machine enable
    * @return Error code
    */
    public int SetLaserWeldingEnableExtDoNum(int ctrlModeDONum)

Configure Extended IO Start DO Port for Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Configure extended IO start DO port for laser welding machine
    * @param ctrlModeDONum Extended DO port number for laser welding machine start/stop
    * @return Error code
    */
    public int SetLaserWeldingStartExtDoNum(int ctrlModeDONum)

Configure Extended IO Fault Reset DO Port for Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Configure extended IO fault reset DO port for laser welding machine
    * @param ctrlModeDONum Extended DO port number for laser welding machine fault reset
    * @return Error code
    */
    public int SetLaserWeldingErrResetExtDoNum(int ctrlModeDONum)

Configure Extended IO Running Status (Laser On Status) DI Port for Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Configure extended IO running status (laser on status) DI port for laser welding machine
    * @param diNum Extended DI port number for laser welding machine running status (laser on status)
    * @return Error code, 0 indicates success, non-zero indicates failure
    */
    public int SetLaserWeldingRunningStateExtDiNum(int diNum);

Configure Extended IO Fault Status DI Port for Laser Welding Machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Configure extended IO fault status DI port for laser welding machine
    * @param diNum Extended DI port number for laser welding machine fault status
    * @return Error code, 0 indicates success, non-zero indicates failure
    */
    public int SetLaserWeldingErrStateExtDiNum(int diNum);

Laser Welding Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int testLsaerWeld(Robot robot) {
        int rtn = -1;
        rtn = robot.ExtDevLoadUDPDriver();
        if (rtn != 0) {
            System.out.println("Failed to load UDP driver, error code: " + rtn);
        }
        robot.Sleep(1000);
        rtn = robot.SetLaserWeldingParam(1, 3, 2000, 3, 1500, 100, 1000);
        if (rtn != 0) {
            System.out.println("SetLaserWeldingParam failed, error code: " + rtn);
        } else {
            System.out.println("SetLaserWeldingParam success");
        }
        rtn = robot.SetLaserWeldingStartExtDoNum(1);
        if (rtn != 0) {
            System.out.println("SetLaserWeldingStartExtDoNum failed, error code: " + rtn);
        }
        rtn = robot.Mode(0);
        if (rtn != 0) {
            System.out.println("Set mode 0 failed, error code: " + rtn);
        }
        robot.Sleep(1000);
        DescPose desc_pos1 = new DescPose(-303.721, -206.960, 297.105, 152.209, 19.857, 109.166);
        DescPose desc_pos2 = new DescPose(-301.575, -254.888, 284.786, 155.919, 26.946, 111.629);
        DescPose desc_safe = new DescPose(-344.386, -280.830, 435.073, 173.835, 15.333, 124.931);

        JointPos jointPos1 = new JointPos(9.827, -99.740, 120.088, -78.900, -77.241, -17.904);
        JointPos jointPos2 = new JointPos(15.251, -96.456, 120.138, -84.664, -68.542, -17.843);
        JointPos jointSafe = new JointPos(19.142, -98.078, 101.493, -83.078, -77.070, -17.794);

        ExaxisPos exaxis = new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offset = new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        int error = robot.MoveL(desc_pos1, 0, 0, 100, 100, 100, -1, 0, exaxis, 0, 0, offset, -1, 0,0,0);
        System.out.println("MoveL to pos1 return: " + error);
        rtn = robot.SetLaserWeldingStartEnd(1, 1, 10000);
        if (rtn != 0) {
            System.out.println("SetLaserWeldingStartEnd (start) failed, error code: " + rtn);
        } else {
            System.out.println("Laser started");
        }
        rtn = robot.MoveL(desc_pos2, 0, 0, 30, 100, 100, -1, 0, exaxis, 0, 0, offset, -1, 0,0, 0);
        System.out.println("MoveL to pos2 return: " + rtn);
        rtn = robot.SetLaserWeldingStartEnd(1, 0, 10000);
        if (rtn != 0) {
            System.out.println("SetLaserWeldingStartEnd (stop) failed, error code: " + rtn);
        } else {
            System.out.println("Laser stopped");
        }
        robot.Sleep(500);
        rtn = robot.MoveL(desc_safe, 0, 0, 100, 100, 100, -1, 0, exaxis, 0, 0, offset, -1, 0,0,0);
        System.out.println("MoveL to safe_pos return: " + rtn);
        rtn = robot.Mode(1);
        if (rtn != 0) {
            System.out.println("Set mode 1 failed, error code: " + rtn);
        }
        robot.Sleep(1000);
        robot.CloseRPC();
        robot.Sleep(1000);

        System.out.println("Test completed");

        return 0;
    }

Set Weave End Return to Cycle Zero Point
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set whether to return to the cycle zero point at the end of weaving
    * @param flag Whether to return to cycle zero point at weave end; 0-do not return; 1-return to cycle zero point
    * @return Error code
    */
    public int SetWeaveBackCenterConfig(int flag)
        
Get Weave End Return to Cycle Zero Point Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get the weave end return to cycle zero point parameters
    * @param flag Whether to return to cycle zero point at weave end; 0-do not return; 1-return to cycle zero point
    * @return Error code
    */
    public int GetWeaveBackCenterConfig(int[] flag)
            
Weave End Return to Cycle Zero Point Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void TestSplineWeave(Robot robot)
    {
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
        float oacc = 100.0f;
        float blendT = 0.0f;
        float blendR = 0.0f;
        int flag = 0;
        int search = 0;
        int blendMode = 0;
        int velAccMode = 0;

        robot.WeaveEnd(0);
        robot.SetSpeed(1);

        robot.SetWeaveBackCenterConfig(1);
        int[] weaveBackConfig = new int[1];
        robot.GetWeaveBackCenterConfig(weaveBackConfig);
        System.out.printf("GetWeaveBackCenterConfig:  %d \n", weaveBackConfig[0]);

        robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, 100.0f, epos, blendT, flag, offset_pos);

        robot.WeaveStart(0);
        robot.NewSplineStart(0, 6000);
        robot.NewSplinePoint(j1, desc_pos1, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(j2, desc_pos2, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(j3, desc_pos3, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(j4, desc_pos4, tool, user, vel, acc, ovl, -1, 1);
        robot.NewSplineEnd();
        robot.WeaveEnd(0);
    }
                
Real-time Speed Setting (Command Frame, Low Latency)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set speed (command frame, low latency)
    * @param vel Speed percentage, range [0~100]
    * @return Error code
    */
    public int SetSpeedInstant(int vel)
                    
Set Real-Time Weave Offset
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set real-time weave offset
    * @param [in] offset Real-time offset [mm, °]
    * @return Error code
    */
    public int SetWeaveOffsetRT(DescPose offset)
                        
Weave Speed and Offset Test Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: Java
    :linenos:

    public static void TestWeaveSpeedAndOffset(Robot robot) {
    if (robot == null) {
        System.out.println("ERROR: connect fail");
        return;
    }
    int rtn;
    ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
     ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
     DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);

     JointPos j1 = new JointPos(5.027, -84.331, -75.139, -103.690, 86.379, 20.794);
     DescPose d1 = new DescPose(324.752, -83.339, 366.314, -172.321, -0.936, -106.047);

     JointPos j2 = new JointPos(-35.335, -117.598, -57.174, -95.234, 90.001, -19.560);
     DescPose d2 = new DescPose(324.999, -355.439, 260.000, 179.995, 0.003, -105.775);

     JointPos j3 = new JointPos(59.787, -117.594, -57.183, -95.222, 90.006, 75.562);
     DescPose d3 = new DescPose(324.998, 355.441, 260.002, 179.995, 0.003, -105.775);

     System.out.println("\nStep 1: MoveJ to start point");
     rtn = robot.MoveJ(j1, d1, 1, 0, 100, 100, 50, epos, -1, 0, offset_pos);
     System.out.println("  MoveJ(j1) rtn=" + rtn);
     try {
         Thread.sleep(500);
     } catch (InterruptedException e) {
         Thread.currentThread().interrupt();
     }

     System.out.println("\nStep 2: MoveJ to weave entry point");
     rtn = robot.MoveJ(j2, d2, 1, 0, 100, 100, 50, epos, -1, 0, offset_pos);
     System.out.println("  MoveJ(j2) rtn=" + rtn);
     try {
         Thread.sleep(500);
     } catch (InterruptedException e) {
         Thread.currentThread().interrupt();
     }

     System.out.println("\nStep 3: WeaveStart + MoveL in background thread");
     robot.WeaveStart(0);

     final boolean[] weaveRunning = {true};
     final int[] threadRtn = {0};
     Thread weaveThread = new Thread(new Runnable() {
         @Override
         public void run() {
             threadRtn[0] = robot.MoveL(j3, d3, 1, 0, 100, 100, 5, -1, 0, epos, 0, 0, offset_pos, 5, 0, 0, 10);
             System.out.println("  MoveL(weave) thread finished, rtn=" + threadRtn[0]);
             weaveRunning[0] = false;
         }
     });
     weaveThread.setDaemon(true);
     weaveThread.start();
     try {
         Thread.sleep(500);
     } catch (InterruptedException e) {
         Thread.currentThread().interrupt();
     }

     System.out.println("\nStep 4: SetSpeed test during weaving");
     int[] speedValues = { 20, 50, 80, 30, 60, 10 };
     for (int speed : speedValues) {
         if (!weaveRunning[0]) break;
         rtn = robot.SetSpeedInstant(speed);
         pkg = robot.GetRobotRealTimeState();
         System.out.println("  SetSpeed(" + speed + ") -> rtn=" + rtn + ", TCP_CmpSpeed=" + pkg.target_TCP_CmpSpeed);
         try {
             Thread.sleep(5000);
         } catch (InterruptedException e) {
             Thread.currentThread().interrupt();
         }
     }

     try {
         Thread.sleep(5000);
     } catch (InterruptedException e) {
         Thread.currentThread().interrupt();
     }

     System.out.println("\nStep 5: SetWeaveOffsetRT test (50 iterations, delta=0.1)");
     double accumOffset = 0.0;
     for (int i = 0; i < 50 && weaveRunning[0]; i++) {
         accumOffset += 0.1;
         DescPose weaveOffset = new DescPose(0, 0, accumOffset, 0, 0, 0);
         rtn = robot.SetWeaveOffsetRT(weaveOffset);
         pkg = robot.GetRobotRealTimeState();
         System.out.printf("  [%d/50] SetWeaveOffsetRT(x=%.1f) -> rtn=%d, TCP_pos=(%.2f,%.2f,%.2f)\n",
             i + 1, accumOffset, rtn,
             pkg.tl_cur_pos[0], pkg.tl_cur_pos[1], pkg.tl_cur_pos[2]);
         try {
             Thread.sleep(100);
         } catch (InterruptedException e) {
             Thread.currentThread().interrupt();
         }
     }

     System.out.println("\nStep 6: Wait for weave MoveL, then WeaveEnd");
      try {
          weaveThread.join();
      } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
      }
      robot.WeaveEnd(0);
      try {
          Thread.sleep(500);
      } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
      }

      System.out.println("\nStep 7: MoveL back to start");
      rtn = robot.MoveL(j1, d1, 1, 0, 100, 100, 50, -1, 0, epos, 0, 0, offset_pos, 50, 0, 0, 10);
      System.out.println("  MoveL(back) rtn=" + rtn);

      pkg = robot.GetRobotRealTimeState();
      System.out.println("\n  Final robot state: main_code=" + pkg.main_code + ", sub_code=" + pkg.sub_code);
    }