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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

aspiration (phonetics, explosion of breath on consonants distinguishing Chinese p, t from b, d)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

segment welding
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

code example
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