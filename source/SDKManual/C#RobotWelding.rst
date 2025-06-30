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

.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the swing parameters
    * @param [in] weaveNum swing welding parameter configuration number
    * @param [in] weaveType Swing type 0- Planar triangular wave swing; 1- Vertical L-shaped triangular wave swing 2- Clockwise circular swing; 3- Counterclockwise circular swing; 4- Planar sine wave swing 5- Vertical L-shaped sine wave swing 6- Vertical triangular wave swing 7- Vertical sine wave swing
    * @param [in] weaveFrequency (Hz)
    * @param [in] weaveIncStayTime Waiting mode 0- cycle does not include waiting time; 1- The cycle includes waiting time
    * @param [in] weaveRange Swing amplitude (mm)
    * @param [in] weaveLeftRange Vertical triangular swing left chord Length (mm)
    * @param [in] weaveRightRange Vertical triangular swing right chord Length (mm)
    * @param [in] additionalStayTime Vertical triangular swing Vertical triangular point stay Time (mm)
    * @param [in] weaveLeftStayTime Swing left stay Time (ms)
    * @param [in] weaveRightStayTime Swing right stay Time (ms)
    * @param [in] weaveCircleRadio Circular Swing - Callback Ratio (0-100%)
    * @param [in] weaveStationary swing position waiting, 0- the position continues to move during the waiting time; The position remains stationary during the waiting time
    * @param [in] weaveYawAngle Swing direction azimuth Angle (rotation around the z-axis of swing), unit °
    * @return error code
    */
    int WeaveSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, double weaveLeftRange, double weaveRightRange, int additionalStayTime, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary, double weaveYawAngle, double weaveRotAngle=0);

Code Example
++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    private void button7_Click(object sender, EventArgs e)
    {
        DescPose startdescPose = new DescPose(146.273, -208.110, 270.102, 177.523, -3.782, -158.101);
        JointPos startjointPos = new JointPos(98.551, -128.309, 127.341, -87.490, -94.249, -13.208);
        DescPose enddescPose = new DescPose(146.272, -476.204, 270.102, 177.523, -3.781, -158.101);
        JointPos endjointPos = new JointPos(93.931, -89.722, 102.216, -101.300, -94.359, -17.840);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
        robot.WeaveSetPara(0, 3, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 0, 0);
        robot.MoveL(startjointPos, startdescPose, 2, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);
        robot.WeaveStart(0);
        robot.MoveL(endjointPos, enddescPose, 2, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);
        robot.WeaveEnd(0);

        robot.WeaveSetPara(0, 3, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 0, 30);
        robot.MoveL(startjointPos, startdescPose, 2, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);
        robot.WeaveStart(0);
        robot.MoveL(endjointPos, enddescPose, 2, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);
        robot.WeaveEnd(0);

    }

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
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Swing gradient begins
    * @param [in] weaveChangeFlag 1- Variable swing parameter; 2- Variable swing parameters + welding speed
    * @param [in] weaveNum swing number
    * @param [in] velStart welding start speed, (cm/min)
    * @param [in] velEnd welding end speed, (cm/min)
    * @return error code
    */
    int WeaveChangeStart(int weaveChangeFlag, int weaveNum, double velStart, double velEnd);

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

Arc Tracking Welding machine Current Feedback AI Channel selection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Arc Tracking Welding machine Current Feedback AI Channel selection
    * @param [in] channel; 0- Expand AI0; 1- Expand AI1; 2- Expand AI2; 3- Expand AI3; 4- Control Box AI0 5- Control Box AI1
    * @return error code
    */
    int ArcWeldTraceAIChannelCurrent(int channel);

Arc Tracking Welding Machine Voltage Feedback AI Channel Selection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Arc Tracking Welding Machine Voltage Feedback AI Channel Selection
    * @param [in] channel; 0- Expand AI0; 1- Expand AI1; 2- Expand AI2; 3- Expand AI3; 4- Control Box AI0 5- Control Box AI1
    * @return error code
    */
    int ArcWeldTraceAIChannelVoltage(int channel);

Current feedback Conversion parameters of Arc tracking Welding machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Current feedback Conversion parameters of Arc tracking Welding machine
    * @param [in] AILow AI channel lower limit, default value 0V, range [0-10V]
    * @param [in] AIHigh AI channel upper limit, default value 10V, range [0-10V]
    * @param [in] The lower limit of the currentLow AI channel corresponds to the current value of the welding machine. The default value is 0V, and the range is [0-200V]
    * @param [in] currentHigh AI channel upper limit corresponding welder current value, default value 100V, range [0-200V]
    * @return error code
    */
    int ArcWeldTraceCurrentPara(float AILow, float AIHigh, float currentLow, float currentHigh);

Voltage feedback Conversion Parameters of Arc Tracking Welding machine
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Voltage feedback Conversion Parameters of Arc Tracking Welding machine
    * @param [in] AILow AI channel lower limit, default value 0V, range [0-10V]
    * @param [in] AIHigh AI channel upper limit, default value 10V, range [0-10V]
    * @param [in] The lower limit of the voltageLow AI channel corresponds to the welding machine voltage value. The default value is 0V, and the range is [0-200V]
    * @param [in] The upper limit of the voltageHigh AI channel corresponds to the voltage value of the welding machine. The default value is 100V, and the range is [0-200V]
    * @return error code
    */
    int ArcWeldTraceVoltagePara(float AILow, float AIHigh, float voltageLow, float voltageHigh);

Code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    private void btnweld_Click(object sender, EventArgs e)
    {
        DescPose safetydescPose = new DescPose(-504.043, 275.181, 40.908, -28.002, -42.025, -14.044);
        JointPos safetyjointPos = new JointPos(-39.078, -76.732, 87.227, -99.47, -94.301, 18.714);
        DescPose startdescPose = new DescPose(-473.86, 257.879, -20.849, -37.317, -42.021, 2.543);
        JointPos startjointPos = new JointPos(-43.487, -76.526, 95.568, -104.445, -89.356, 3.72);

        DescPose enddescPose = new DescPose(-499.844, 141.225, 7.72, -34.856, -40.17, 13.13);
        JointPos endjointPos = new JointPos(-31.305, -82.998, 99.401, -104.426, -89.35, 3.696);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
        robot.MoveJ(safetyjointPos, safetydescPose, 1, 0, 20, 100, 100, exaxisPos, -1, 0, offdese);

        robot.WeldingSetCurrentRelation(0, 495, 1, 10, 0);
        robot.WeldingSetVoltageRelation(10, 45, 1, 10, 1);
        robot.WeldingSetVoltage(0, 25, 1, 0);//
        robot.WeldingSetCurrent(0, 260, 0, 0);//

        int rtn = robot.ArcWeldTraceAIChannelCurrent(4);
        Console.WriteLine("ArcWeldTraceAIChannelCurrent rtn is " + rtn);
        rtn = robot.ArcWeldTraceAIChannelVoltage(5);
        Console.WriteLine("ArcWeldTraceAIChannelVoltage rtn is " + rtn);
        rtn = robot.ArcWeldTraceCurrentPara((float)0, (float)5, (float)0, (float)500);
        Console.WriteLine("ArcWeldTraceCurrentPara rtn is " + rtn);
        rtn = robot.ArcWeldTraceVoltagePara((float)1.018, (float)10, (float)0, (float)50);
        Console.WriteLine("ArcWeldTraceVoltagePara rtn is " + rtn);

        robot.MoveJ(startjointPos, startdescPose, 1, 0, 20, 100, 100, exaxisPos, -1, 0, offdese);
        robot.ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
        robot.ARCStart(0, 0, 10000);
        robot.WeaveStart(0);
        // robot.MoveL(endjointPos, enddescPose, 1, 0, 100, 100, 2, -1, exaxisPos, 0, 0, offdese);
        robot.ARCEnd(0, 0, 10000);
        robot.WeaveEnd(0);
        robot.ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
        robot.MoveJ(safetyjointPos, safetydescPose, 1, 0, 20, 100, 100, exaxisPos, -1, 0, offdese);
    }

Set the welding voltage to start gradually
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the welding voltage to start gradually
    * @param [in] IOType control type; 0- Control Box IO 1- Digital Communication Protocol (UDP) 2- Digital Communication Protocol (ModbusTCP)
    * @param [in] voltageStart Initial Welding Voltage (V)
    * @param [in] voltageEnd Stop welding Voltage (V)
    * @param [in] AOIndex control box AO port number (0-1)
    * @param [in] Is blend smooth? 0- Not smooth; 1- Smooth
    * @return error code
    */
    int WeldingSetVoltageGradualChangeStart(int IOType, double voltageStart, double voltageEnd, int AOIndex, int blend);

Set the welding voltage gradient to end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief: Set the welding voltage gradient to end
    * @return error code
    */
    int WeldingSetVoltageGradualChangeEnd();

Set the welding current to start gradually
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the welding current to start gradually
    * @param [in] IOType control type; 0- Control Box IO 1- Digital Communication Protocol (UDP) 2- Digital Communication Protocol (ModbusTCP)
    * @param [in] voltageStart Initial welding Current (A)
    * @param [in] voltageEnd Stop welding current (A)
    * @param [in] AOIndex control box AO port number (0-1)
    * @param [in] Is blend smooth? 0- Not smooth; 1- Smooth
    * @return error code
    */
    int WeldingSetCurrentGradualChangeStart(int IOType, double currentStart, double currentEnd, int AOIndex, int blend);

Set the welding current gradient to end
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the welding current gradient to end
    * @return error code
    */
    int WeldingSetCurrentGradualChangeEnd();

Code example
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

        robot.WeldingSetVoltage(0, 25, 1, 0);
        robot.WeldingSetCurrent(0, 260, 0, 0);

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