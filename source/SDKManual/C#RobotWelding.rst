Robotic welding
==========================
.. toctree:: 
    :maxdepth: 5


Start of welding
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Welding begins
    * @param [in] ioType io type 0-controller IO; 1-extended IO
    * @param [in] arcNum Welder profile number
    * @param [in] timeout Arc start timeout
    * @return error code
    */
    int ARCStart(int ioType, int arcNum, int timeout);

End of welding
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief End of welding
    * @param [in] ioType io type 0-controller IO; 1-extended IO
    * @param [in] arcNum Welder profile number
    * @param [in] timeout Arc off timeout
    * @return error code
    */
    int ARCEnd(int ioType, int arcNum, int timeout);

Setting of welding current and output analogue correspondence
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set weld current to analogue output
    * @param [in] currentMin Welding current - analogue output linear relationship left point current value (A)
    * @param [in] currentMax Welding current - analogue output linear relationship right point current value (A)
    * @param [in] outputVoltageMin Welding current - analogue output linear relationship left point analogue output voltage value (V)
    * @param [in] outputVoltageMax The analogue output voltage value (V) at the right point of the weld current-analog output linear relationship.
    * @return error code
    */
    int WeldingSetCurrentRelation(double currentMin, double currentMax, double outputVoltageMin, double outputVoltageMax);

Setting the welding voltage and output analogue correspondence
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the weld voltage to output analogue correspondence
    * @param [in] weldVoltageMin Welding Voltage - Analogue Output Linear Relationship Left Point Welding Voltage Value (A)
    * @param [in] weldVoltageMax Welding Voltage - Analogue Output Linear Relationship Right Point Welding Voltage Value (A)
    * @param [in] outputVoltageMin Welding Voltage - Analogue Output Linear Relationship Left Point Analogue Output Voltage Value (V)
    * @param [in] outputVoltageMax Analogue output voltage value (V) at the right point of the weld voltage-analogue output linear relation
    * @return error code
    */
    int WeldingSetVoltageRelation(double weldVoltageMin, double weldVoltageMax, double outputVoltageMin, double outputVoltageMax);

Getting the correspondence between welding current and output analogue
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Get weld current to output analogue
    * @param [out] currentMin Welding current - analogue output linear relationship left point current value (A)
    * @param [out] currentMax Welding current - analogue output linear relationship right point current value (A)
    * @param [out] outputVoltageMin Welding current - analogue output linear relationship left point analogue output voltage value (V)
    * @param [out] outputVoltageMax Analogue output voltage value (V) at the right point of the weld current-analog output linear relationship
    * @return error code
    */
    int WeldingGetCurrentRelation(ref double currentMin, ref double currentMax, ref double outputVoltageMin, ref double outputVoltageMax);

Getting welding voltage and output analogue correspondence
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Get weld voltage to analogue output
    * @param [out] weldVoltageMin Welding Voltage - Analogue Output Linear Relationship Left Point Welding Voltage Value (A)
    * @param [out] weldVoltageMax Welding Voltage - Analogue Output Linear Relationship Right Point Welding Voltage Value (A)
    * @param [out] outputVoltageMin Welding voltage - analogue output linear relationship left point analogue output voltage value (V)
    * @param [out] outputVoltageMax Analogue output voltage value (V) at the right point of the weld voltage-analogue output linearity relation
    * @return error code
    */
    int WeldingGetVoltageRelation(ref double weldVoltageMin, ref double weldVoltageMax, ref double outputVoltageMin, ref double outputVoltageMax);

Setting the welding current
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set weld current
    * @param [in] ioType Control IO type 0-Control box IO; 1-Extended IO
    * @param [in] current Welding current value (A)
    * @param [in] AOIndex Welding current control box analogue output port (0-1)
    * @return error code
    */
    int WeldingSetCurrent(int ioType, double current, int AOIndex).

Setting the welding voltage
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set weld voltage
    * @param [in] ioType Control IO type 0-Control box IO; 1-Extended IO
    * @param [in] voltage Welding voltage value (A)
    * @param [in] AOIndex Analogue output port of welding voltage control box (0-1)
    * @return error code
    */
    int WeldingSetVoltage(int ioType, double voltage, int AOIndex);

Setting the oscillation parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * :: @brief Setting the swing parameters
    * @param [in] weaveNum weave parameter configuration number
    * @param [in] weaveType weaveType 0-planar triangular wave swing; 1-vertical L-shaped triangular wave swing; 2-clockwise circular swing; 3-counterclockwise circular swing; 4-planar sine wave swing; 5-vertical L-shaped sine wave swing; 6-vertical triangular wave swing; 7-vertical sine wave swing
    * @param [in] weaveFrequency swing frequency (Hz)
    * @param [in] weaveIncStayTime wait mode 0-cycle without wait time; 1-cycle with wait time
    * @param [in] weaveRange swing range (mm)
    * @param [in] weaveLeftStayTime weaveLeftStayTime (ms)
    * @param [in] weaveRightStayTime weaveRightStayTime (ms)
    * @param [in] weaveCircleRadio Circle swing-back ratio (0-100%)
    * @param [in] weaveStationary swing position wait, 0 - position continues to move during wait time; 1 - position is stationary during wait time
    * @return error code
    */
    int WeaveSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary).

Instant setting of swing parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Instant setup of swing parameters
    * @param [in] weaveNum weave parameter configuration number
    * @param [in] weaveType weaveType 0-planar triangular wave swing; 1-vertical L-shaped triangular wave swing; 2-clockwise circular swing; 3-counterclockwise circular swing; 4-planar sine wave swing; 5-vertical L-shaped sine wave swing; 6-vertical triangular wave swing; 7-vertical sine wave swing
    * @param [in] weaveFrequency swing frequency (Hz)
    * @param [in] weaveIncStayTime wait mode 0-cycle without wait time; 1-cycle with wait time
    * @param [in] weaveRange swing range (mm)
    * @param [in] weaveLeftStayTime weaveLeftStayTime (ms)
    * @param [in] weaveRightStayTime weaveRightStayTime (ms)
    * @param [in] weaveCircleRadio Circle swing-back ratio (0-100%)
    * @param [in] weaveStationary swing position wait, 0 - position continues to move during wait time; 1 - position is stationary during wait time
    * @return error code
    */
    int WeaveOnlineSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary).

swing start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief The swing is on
    * @param [in] weaveNum weave parameter configuration number
    * @return error code
    */
    int WeaveStart(int weaveNum).

end of swing
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief End of swing
    * @param [in] weaveNum weave parameter configuration number
    * @return error code
    */
    int WeaveEnd(int weaveNum).

Positive wire feed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Forward wire feed
    * @param [in] ioType io type 0-Controller IO; 1-Extended IO
    * @param [in] wireFeed Wire Feed Control 0-Stop Wire Feed; 1-Feed Wire
    * @return error code
    */
    int SetForwardWireFeed(int ioType, int wireFeed). 	

Reverse wire feed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Reverse wire feed
    * @param [in] ioType io type 0-Controller IO; 1-Extended IO
    * @param [in] wireFeed Wire Feed Control 0-Stop Wire Feed; 1-Feed Wire
    * @return error code
    */
    int SetReverseWireFeed(int ioType, int wireFeed).

Aspiration (phonetics, explosion of breath on consonants distinguishing Chinese p, t from b, d)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief sends gas
    * @param [in] ioType io type 0-Controller IO; 1-Extended IO
    * @param [in] airControl airControl 0-stop air delivery; 1-feed air
    * @return error code
    */
    int SetAspirated(int ioType, int airControl).

Segment welding
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /** 
    * :: Start of @brief segment welding
    * @param [in] startDesePos Starting point Cartesian position
    * @param [in] endDesePos End Point Cartesian Positions
    * @param [in] startJPos Starting point joint position
    * @param [in] endJPos endJoint Positions
    * @param [in] weldLength Weld section length (mm)
    * @param [in] noWeldLength Non-Weld Length (mm)
    * @param [in] weldIOType Weld IO type (0-control box IO; 1-extended IO)
    * @param [in] arcNum Welder profile number
    * @param [in] weldTimeout 起/收弧超时时间时间
    * @param [in] isWeave isWeave or not?
    * @param [in] weaveNum weave parameter configuration number
    * @param [in] tool tool number
    * @param [in] user Workpiece number
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm	 
    * @param [in] epos Extended axis position in mm
    * @param [in] search 0-no wire search, 1-wire search
    * @param [in] offset_flag 0-no offset, 1-offset in base/workpiece coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos Bit position offset
    * @return error code 
    */
    int SegmentWeldStart(DescPose startDesePos, DescPose endDesePos, JointPos startJPos, JointPos endJPos, double weldLength, double noWeldLength, int weldIOType, int arcNum, int weldTimeout,bool isWeave, int weaveNum, int tool, int user, float vel, float acc, float ovl, float int weldIOType, int arcNum, int weldTimeout,bool isWeave, int weaveNum, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, byte search, byte offset_flag, DescPose offset_pos);

Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    private void btnWeldStart_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");
        DescPose startdescPose = new DescPose(-525.55, 562.3, 417.199, -178.325, 0.847, 31.109);
        JointPos startjointPos = new JointPos(-58.978, -76.817, 112.494, -127.348, -89.145, -0.063);
        DescPose enddescPose = new DescPose(-345.155, 535.733, 421.269, 179.475, 0.571, 18.332);
        JointPos endjointPos = new JointPos(-71.746, -87.177, 123.953, -126.25, -89.429, -0.089);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0, 0);

        robot.WeldingSetCurrentRelation(0, 400, 0, 10);
        robot.WeldingSetVoltageRelation(0, 40, 0, 10);
        double curmin = 0;
        double curmax = 0;
        double vurvolmin = 0;
        double curvolmax = 0;
        double volmax = 0;
        double volmin = 0;
        double volvolmin = 0;
        double volvolmax = 0;

        robot.WeldingGetCurrentRelation(ref curmin, ref curmax, ref vurvolmin, ref curvolmax);
        robot.WeldingGetVoltageRelation(ref volmin, ref volmax, ref volvolmin, ref volvolmax);

        robot.WeldingSetCurrent(0, 100, 0); 
        robot.WeldingSetVoltage(0, 19, 1);

        robot.WeaveSetPara(0,0,1,0,10,100,100,0,0);

        robot.SetForwardWireFeed(0, 1);
        Thread.Sleep(1000);
        robot.SetForwardWireFeed(0, 0);
        robot.SetReverseWireFeed(0, 1);
        Thread.Sleep(1000);
        robot.SetReverseWireFeed(0, 0);
        robot.SetAspirated(0, 1);
        Thread.Sleep(1000);
        robot.SetAspirated(0, 0);

        robot.SetSpeed(5);
        robot.MoveL(startjointPos, startdescPose, 1, 0, 100, 100, 100, 100, 0, exaxisPos, 0, 0, offdese);
        robot.ARCStart(0, 0, 1000);
        robot.WeaveStart(0);
        robot.MoveL(endjointPos, enddescPose, 1, 0, 100, 100, 100, 100, 0, exaxisPos, 0, 0, offdese);
        robot.ARCEnd(0, 0, 1000);
        robot.WeaveEnd(0);
    }

Welding Wire Finding Position Begins
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief Solder wire seek start
    * @param [in] refPos 1-datum point 2-contact point
    * @param [in] searchVel searchVelocity % * @param [in] searchDel
    * @param [in] searchDis Search distance mm
    * @param [in] autoBackFlag autoBackFlag, 0 - not auto; - auto
    * @param [in] autoBackVel autoBackVelocity % * @param [in] searchDis searchDistance mm
    * @param [in] autoBackDis autoBack distance mm
    * @param [in] offectFlag 1-with offset seek; 2-teach point seek
    * @return error code
    */
    int WireSearchStart(int refPos, double searchVel, int searchDis, int autoBackFlag, double autoBackVel, int autoBackDis, int offectFlag);

End of wire search
++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8

.. code-block:: c#
    :linenos:

    /**
    * @brief End of solder wire seek
    * @param [in] refPos 1-datum point 2-contact point
    * @param [in] searchVel searchVelocity %
    * @param [in] searchDis Search distance in mm
    * @param [in] autoBackFlag autoBackFlag, 0 - not auto; - auto
    * @param [in] autoBackVel autoBackVelocity % * @param [in] searchDis searchDistance mm
    * @param [in] autoBackDis autoBack distance mm
    * @param [in] offectFlag 1-with offset seek; 2-teach point seek
    * @return error code
    */
    int WireSearchEnd(int refPos, double searchVel, int searchDis, int autoBackFlag, double autoBackVel, int autoBackDis, int offectFlag);

Calculating wire finding offsets
++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8

.. code-block:: c#
    :linenos:

    /**
    * @brief Calculate weld wire seek offset.
    * @param [in] seamType weldType
    * @param [in] method Calculation method
    * @param [in] varNameRef datums 1-6, ‘#’ means no point variable
    * @param [in] varNameRes contact points 1-6, ‘#’ means no point variable
    * @param [out] offectFlag 0-offset directly superimposed on the command point; 1-offset requires a coordinate transformation of the command point
    * @param [out] offect offset position [x, y, z, a, b, c]
    * @return Error code
    */
    int GetWireSearchOffset(int seamType, int method, string[] varNameRef, string[] varNameRes, ref int offsetFlag, ref DescPose offset);;

Wait for the wire search to complete
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief Waiting for wire seek to complete.
    * @return Error code.
    */
    int WireSearchWait(string name).

Write wire search contact to database
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief Welding wire seek contact written to database.
    * @param [in] varName Contact name ‘RES0’ ~ ‘RES99’.
    * @param [in] pos contact data [x, y, x, a, b, c]
    * @return Error code
    */
    int SetPointToDatabase(string varName, DescPose pos);

Arc tracking control
++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8

.. code-block:: c#
    :linenos:

    / * *
    * @brief arc tracking control
    * @param [in] flag Switch, 0- off. 1-on
    * @param [in] dalayTime Delay time (unit: ms)
    * @param [in] isLeftRight left and right deviation compensation
    * @param [in] klr adjustment coefficient (sensitivity)
    * @param [in] tStartLr or around start compensation time cyc
    * @param [in] stepMaxLr About the maximum compensation amount mm each time
    * @param [in] sumMaxLr Maximum total compensation mm
    * @param [in] isUpLow up-and-down deviation compensation
    * @param [in] kud up-down adjustment factor (sensitivity)
    * @param [in] tStartUd start up and down compensation time cyc
    * @param [in] stepMaxUd Maximum compensation amount mm each time
    * @param [in] sumMaxUd Maximum total compensation
    * @param [in] axisSelect upper and lower coordinate system selection, 0-swing; 1- Tools; 2- Base
    * @param [in] referenceType Reference current setting mode, 0- feedback; 1-constant
    * @param [in] referSampleStartUd Reference current sampling starts counting (feedback), cyc
    * @param [in] referSampleCountUd Reference current sampling cycle count (feedback), cyc
    * @param [in] referenceCurrent Upper and lower reference current mA
    * @param [in] offsetType Indicates the offset tracking type. 0- no offset. 1- Sampling; 2- percentage 
    * @param [in] offsetParameter Offset parameter; Sampling (offset sampling start time, default sampling cycle); Percentage (offset percentage (-100 to 100)) /version 3.8.0
    * @return Error code
    * /
    int ArcWeldTraceControl(int flag, double delaytime, int isLeftRight, double klr, double tStartLr, double stepMaxLr, double sumMaxLr, int isUpLow, double kud, double tStartUd, double stepMaxUd, double sumMaxUd, int axisSelect, int referenceType, double referSampleStartUd, double referSampleCountUd, double referenceCurrent, int offsetType, int offsetParameter);

Code example
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.1.0

.. code-block:: c#
    :linenos:

    private void btnweld_Click(object sender, EventArgs e)
    {

        //Arc Tracking
        DescPose p1Desc = new DescPose(-72.912, -587.664, 31.849, 43.283, -6.731, 15.068);
        JointPos p1Joint = new JointPos(74.620, -80.903, 94.608, -109.882, -90.436, -13.432);

        DescPose p2Desc = new DescPose(-104.915, -483.712, -25.231, 42.228, -6.572, 18.433);
        JointPos p2Joint = new JointPos(66.431, -92.875, 116.362, -120.516, -88.627, -24.731);

        DescPose p3Desc = new DescPose(-242.834, -498.697, -23.681, 46.576, -5.286, 8.318);
        JointPos p3Joint = new JointPos(57.153, -82.046, 104.060, -116.659, -92.478, -24.735);
        ExaxisPos exaxisPos = new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offdese = new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        robot.WeldingSetVoltage(1, 19, 0, 0);
        robot.WeldingSetCurrent(1, 190, 0, 0);
        robot.MoveJ(p1Joint, p1Desc, 1, 1, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveL(p2Joint, p2Desc, 1, 1, 100, 100, 50, -1, exaxisPos, 0, 0, offdese);
        robot.ARCStart(1, 0, 10000);
        robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 60, 0, 0, 4, 1, 10, 2, 2);
        robot.WeaveStart(0);
        robot.MoveL(p3Joint, p3Desc, 1, 1, 100, 100, 1, -1, exaxisPos, 0, 0, offdese);
        robot.WeaveEnd(0);
        robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 60, 0, 0, 4, 1, 10, 2, 2);
        robot.ARCEnd(1, 0, 10000);
    }

Arc tracking control
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Arc tracking control
    * @param [in] flag switch, 0-off; 1-on
    * @param [in] dalayTime hysteresis time in ms
    * @param [in] isLeftRight left-right deviation compensation
    * @param [in] klr left/right adjustment factor (sensitivity)
    * @param [in] tStartLr Left/Right compensation time cyc
    * @param [in] stepMaxLr Maximum compensation in mm per shift * @param [in] stepMaxLr Maximum compensation in mm per shift
    * @param [in] sumMaxLr Total max compensation in mm
    * @param [in] isUpLow Compensation for up and down deviation
    * @param [in] kud up/down adjustment factor (sensitivity)
    * @param [in] tStartUd up/down compensation time cyc
    * @param [in] stepMaxUd Maximum compensation in mm for each step.
    * @param [in] sumMaxUd up and down total max. compensation mm
    * @param [in] axisSelect upper and lower coordinate system selection, 0-swing; 1-tool; 2-base
    * @param [in] referenceType upper and lower reference current setting mode, 0-feedback; 1-constant
    * @param [in] referSampleStartUd upper and lower reference current sample start count (feedback), cyc
    * @param [in] referSampleCountUd upper and lower reference current sampling cycle count (feedback), cyc
    * @param [in] referenceCurrent upper and lower reference currents mA
    * @return Error code
    */
    int ArcWeldTraceControl(int flag, double delaytime, int isLeftRight, double klr, double tStartLr, double stepMaxLr, double sumMaxLr, int isUpLow, double kud, double tStartUd, double stepMaxUd, double sumMaxUd, int axisSelect, int referenceType, double referSampleStartUd, double referSampleCountUd, double referenceCurrent).

Arc tracking AI passband selection
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief Arc-tracking AI passband selection.
    * @param [in] channel Arc-tracking AI passband selection, [0-3].
    * @return Error code.
    */
    int ArcWeldTraceExtAIChannelConfig(int channel);

Simulated Swing Start
++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8

.. code-block:: c#
    :linenos:

    /**
    * @brief Simulation of swing start
    * @param [in] weaveNum swing parameter number
    * @return ErrorCode
    */
    int WeaveStartSim(int weaveNum).

End of simulation
++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8

.. code-block:: c#
    :linenos:

    /**
    * @brief Simulation of end of swing
    * @param [in] weaveNum swing parameter number
    * @return ErrorCode
    */
    int WeaveEndSim(int weaveNum).

Start trajectory detection warning (no motion)
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief start trajectory detection warning (no motion)
    * @param [in] weaveNum swing parameter number
    * @return Error code
    */
    int WeaveInspectStart(int weaveNum).

End trajectory detection warning (no motion)
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief end trajectory detection warning (no motion)
    * @param [in] weaveNum swing parameter number
    * @return Error code
    */
    int WeaveInspectEnd(int weaveNum).

Extended IO-Configuration Welder Gas Detection Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration Welder Gas Detection Signal
    * @param [in] DONum gas detection signal extended DO number
    * @return error-code
    */
    int SetAirControlExtDoNum(int DONum).

Extended IO-Configuration Welder Arc Start Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration of the welder arc start signal.
    * @param [in] DONum Welder Arc Start Signal Extended DO Number
    * @return Error code.
    */
    int SetArcStartExtDoNum(int DONum).

Extended IO-Configuring the Welder Reverse Wire Feed Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration Welder Reverse Wire Feed Signal
    * @param [in] DONum Reverse Wire Feed Signal Extension DO Number
    * @return Error Code
    */
    int SetWireReverseFeedExtDoNum(int DONum).

Extended IO-Configuration Welder Forward Wire Feed Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Config welder positive wire feed signal.
    * @param [in] DONum Forward Wire Feed Signal Extension DO Number
    * @return Error Code
    */
    int SetWireForwardFeedExtDoNum(int DONum).

Extended IO-Configuration Welder Arc Start Success Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration Welder Arc Success Signal
    * @param [in] DINum Extended DI number for arc success signal.
    * @return error-code
    */
    int SetArcDoneExtDiNum(int DINum).

Extended IO-Configuration Welder Ready Signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration Welder Ready Signal
    * @param [in] DINum Welder ready signal extended DI number
    * @return error-code
    */
    int SetWeldReadyExtDiNum(int DINum).

Extended IO-Configure weld interrupt recovery signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief Extended IO-Configuration Weld Interrupt Recovery Signal
    * @param [in] reWeldDINum Resume weld signal after weld interrupt Extended DI number
    * @param [in] abortWeldDINum Exit Weld Signal Extended DI Number after a Weld Interrupt
    * @return Error Code
    */
    nt SetExtDIWeldBreakOffRecover(int reWeldDINum, int abortWeldDINum);

Arc Tracking + Multi-Layer Multi-Pass Compensation On
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

    /**
    * @brief Arc tracking + multilayer multichannel compensation turned on
    * @return Error code
    */
    int ArcWeldTraceReplayStart();

ArcWeldTrace + MultiLayerMultiChannelCompensation OFF
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

        /**
         * @brief Arc tracking + multilayer multichannel compensation off
         * @return Error code
         */
    int ArcWeldTraceReplayEnd();

Offset Coordinate Change - Multi-Layer Multi-Pass Welding
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8
.. code-block:: c#
    :linenos:

     /**
     * @brief offset coordinate change - multilayer multi-pass welding
     * @param [in] pointO Cartesian orientation of datum.
     * @param [in] pointX Cartesian position of datum X in the direction of the offset.
     * @param [in] pointZ Cartesian position of the datum Z to the offset direction point.
     * @param [in] dx x-direction offset (mm)
     * @param [in] dz z-direction offset (mm)
     * @param [in] dry offset around y-axis (°)
     * @param [out] offset Offset of the result of the calculation.
     * @return Error code
     */
    int MultilayerOffsetTrsfToBase(DescTran pointO, DescTran pointX, DescTran pointZ, double dx, double dz, double dry, ref DescPose offset);

Setting parameters for detecting unexpected interruptions of the robotic welding arc
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting parameters for detecting unexpected interruptions of the robotic welding arc
    * @param [in] checkEnable whether to enable detection; 0-don't enable; 1-enable
    * @param [in] arcInterruptTimeLength arcInterruptAcknowledgmentTimeLength(ms)
    * @return Error code
    */
    int WeldingSetCheckArcInterruptionParam(int checkEnable, int arcInterruptTimeLength)

Get parameters for detecting unexpected interruptions of the robotic welding arc
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8

.. code-block:: c#
    :linenos:

    /**
    * @brief Get parameters for detecting unexpected interruptions of the robotic welding arc
    * @param [out] checkEnable whether to enable detection; 0-don't enable; 1-enable
    * @param [out] arcInterruptTimeLength arcInterruptAcknowledgmentTimeLength(ms)
    * @return Error code
    */
    int WeldingGetCheckArcInterruptionParam(ref int checkEnable, ref int arcInterruptTimeLength)

Setting the robot weld interrupt recovery parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the robot weld interrupt recovery parameters
    * @param[in] enable Whether to enable weld interrupt recovery.
    * @param[in] length The overlap distance of the weld (mm).
    * @param[in] velocity the speed of the robot to return to the restart point (0-100)
    * @param[in] moveType Robot movement to restart point type; 0-LIN; 1-PTP
    * @return Error code
    */
    int WeldingSetReWeldAfterBreakOffParam(int enable, double length, double velocity, int moveType)

Get robot weld interrupt recovery parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8

.. code-block:: c#
    :linenos:

    /**
    * @brief Get robot weld interrupt recovery parameters.
    * @param [out] enable Whether to enable weld interrupt recovery or not.
    * @param [out] length The overlap distance of the weld (mm).
    * @param [out] velocity the speed of the robot to return to the restart point (0-100)
    * @param [out] moveType the way the robot moves to the restart point; 0-LIN; 1-PTP
    * @return Error code
    */
    int WeldingGetReWeldAfterBreakOffParam(ref int enable, ref double length, ref double velocity, ref int moveType)

Setting the robot to resume welding after a welding interruption
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the robot to resume welding after a welding interruption
    * @return Error code
    */
    int WeldingStartReWeldAfterBreakOff()

Setting the robot to exit welding after a weld interruption
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8

.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the robot to exit welding after a weld interruption
    * @return Error code
    */
    int WeldingAbortWeldAfterBreakOff()

code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: c# SDK-v1.1.0-3.7.8

.. code-block:: c#
    :linenos:

    private void button7_Click(object sender, EventArgs e)
    {
        int rtn = -1;
        rtn = robot.WeldingSetCheckArcInterruptionParam(1, 200);
        Console.WriteLine("WeldingSetCheckArcInterruptionParam  {0}", rtn);
        rtn = robot.WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0);
        Console.WriteLine("WeldingSetReWeldAfterBreakOffParam {0}", rtn);
        int enable = 0;
        double length = 0;
        double velocity = 0;
        int moveType = 0;
        int checkEnable = 0;
        int arcInterruptTimeLength = 0;
        rtn = robot.WeldingGetCheckArcInterruptionParam(ref checkEnable, ref arcInterruptTimeLength);
        Console.WriteLine($"WeldingGetCheckArcInterruptionParam  checkEnable {checkEnable} - arcInterruptTimeLength {arcInterruptTimeLength}");

        rtn = robot.WeldingGetReWeldAfterBreakOffParam(ref enable, ref length, ref velocity,ref moveType);
        Console.WriteLine("WeldingGetReWeldAfterBreakOffParam  enable = {0}, length = {1}, velocity = {2}, moveType = {3}", enable, length, velocity, moveType);

        robot.ProgramLoad("/fruser/test.lua");
        robot.ProgramRun();

        Thread.Sleep(5000);

        while (true)
        {
            ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG { };
            robot.GetRobotRealTimeState(ref pkg);
            Console.WriteLine("welding breakoff state is     {0}", pkg.weldingBreakOffState.breakOffState);
            if (pkg.weldingBreakOffState.breakOffState == 1)
            {
                Console.WriteLine("welding breakoff ! \n");
                Thread.Sleep(2000);
                rtn = robot.WeldingStartReWeldAfterBreakOff();
                Console.WriteLine("WeldingStartReWeldAfterBreakOff    %d\n", rtn);
                break;
            }
            Thread.Sleep(100);
        }
    }

Swing gradient begins
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Swing gradient begins
    * @param [in] weaveNum Swing number
    * @return Error code
    * /
    int WeaveChangeStart(int weaveNum)

Swing gradient ends
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    / * *
    * @brief Swing gradient ends
    * @return Error code
    * /
    int WeaveChangeEnd()

Code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnweld_Click(object sender, EventArgs e)
    {
        //摆动渐变
        DescPose p1Desc = new DescPose(-72.912, -587.664, 31.849, 43.283, -6.731, 15.068);
        JointPos p1Joint = new JointPos(74.620, -80.903, 94.608, -109.882, -90.436, -13.432);

        DescPose p2Desc = new DescPose(-104.915, -483.712, -25.231, 42.228, -6.572, 18.433);
        JointPos p2Joint = new JointPos(66.431, -92.875, 116.362, -120.516, -88.627, -24.731);

        DescPose p3Desc = new DescPose(-240.651, -483.840, -7.161, 46.577, -5.286, 8.318);
        JointPos p3Joint = new JointPos(56.457, -84.796, 104.618, -114.497, -92.422, -25.430);

        ExaxisPos exaxisPos = new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offdese = new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        robot.WeldingSetVoltage(1, 19, 0, 0);
        robot.WeldingSetCurrent(1, 190, 0, 0);
        robot.MoveJ(p1Joint, p1Desc, 1, 1, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveL(p2Joint, p2Desc, 1, 1, 100, 100, 50, -1, exaxisPos, 0, 0, offdese);
        robot.ARCStart(1, 0, 10000);
        robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
        robot.WeaveStart(0);
        robot.WeaveChangeStart(1);
        robot.MoveL(p3Joint, p3Desc, 1, 1, 100, 100, 1, -1, exaxisPos, 0, 0, offdese);
        robot.WeaveChangeEnd();
        robot.WeaveEnd(0);
        robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
        robot.ARCEnd(1, 0, 10000);
    }