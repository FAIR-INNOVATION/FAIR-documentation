Robot Welding
==================

.. toctree:: 
    :maxdepth: 5

Welding Start
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Welding Start
    * @param [in] ioType  0-control box IO； 1-extend IO
    * @param [in] arcNum Welder configuration file number
    * @param [in] timeout Arcing timeout time
    * @return Error code
    */
    int ARCStart(int ioType, int arcNum, int timeout);

Welding end
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Welding end
    * @param [in] ioType 0-control box IO； 1-extend IO
    * @param [in] arcNum Welder configuration file number
    * @param [in] timeout Quenching arc timeout time
    * @return Error code
    */
    int ARCEnd(int ioType, int arcNum, int timeout);

Set the relationship between welding current and output analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set the relationship between welding current and output analog
    * @param [in] currentMin Welding current-analog output linear relationship left point current value(A)
    * @param [in] currentMax Welding current-analog output linear relationship right point current value(A)
    * @param [in] outputVoltageMin Welding current-analog output linear relationship left point analog output voltage value(V)
    * @param [in] outputVoltageMax Welding current-analog output linear relationship right point analog output voltage value(V)
    * @return Error code
    */
    int WeldingSetCurrentRelation(double currentMin, double currentMax, double outputVoltageMin, double outputVoltageMax);

Set the relationship between welding voltage and output analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set the relationship between welding voltage and output analog
    * @param [in] weldVoltageMin Welding voltage - analog output linear relationship left spot welding voltage value(V)
    * @param [in] weldVoltageMax Welding voltage - analog output linear relationship right spot welding voltage value(V)
    * @param [in] outputVoltageMin Welding voltage-analog output linear relationship left point analog output voltage value(V)
    * @param [in] outputVoltageMax Welding voltage-analog output linear relationship right point analog output voltage value(V)
    * @return Error code
    */
    int WeldingSetVoltageRelation(double weldVoltageMin, double weldVoltageMax, double outputVoltageMin, double outputVoltageMax);

Obtain the corresponding relationship between welding current and output analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Obtain the corresponding relationship between welding current and output analog
    * @param [out] currentMin Welding current-analog output linear relationship left point current value(A)
    * @param [out] currentMax Welding current-analog output linear relationship right point current value(A)
    * @param [out] outputVoltageMin Welding current-analog output linear relationship left point analog output voltage value(V)
    * @param [out] outputVoltageMax Welding current-analog output linear relationship right point analog output voltage value(V)
    * @return Error code
    */
    int WeldingGetCurrentRelation(ref double currentMin, ref double currentMax, ref double outputVoltageMin, ref double outputVoltageMax);

Obtain the corresponding relationship between welding voltage and output analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief The corresponding relationship between welding voltage and output analog is obtained
    * @param [out] weldVoltageMin Welding voltage - analog output linear relationship left spot welding voltage value(V)
    * @param [out] weldVoltageMax Welding voltage - analog output linear relationship right spot welding voltage value(V)(V)
    * @param [out] outputVoltageMin Welding voltage-analog output linear relationship left point analog output voltage value(V)
    * @param [out] outputVoltageMax Welding voltage-analog output linear relationship right point analog output voltage value(V)
    * @return Error code
    */
    int WeldingGetVoltageRelation(ref double weldVoltageMin, ref double weldVoltageMax, ref double outputVoltageMin, ref double outputVoltageMax);

Set welding current
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set welding current
    * @param [in] ioType 0-control box IO； 1-extend IO
    * @param [in] current welding current(A)
    * @param [in] AOIndex Welding current control box analog output port(0-1)
    * @return Error code
    */
    int WeldingSetCurrent(int ioType, double current, int AOIndex);

Set welding voltage
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set welding voltage
    * @param [in] ioType 0-control box IO； 1-extend IO
    * @param [in] voltage welding voltage(V)
    * @param [in] AOIndex Welding voltage control box analog output port(0-1)
    * @return Error code
    */
    int WeldingSetVoltage(int ioType, double voltage, int AOIndex);

Set weave parameters
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set weave parameters
    * @param [in] weaveNum parameters number
    * @param [in] weaveType weave type：0- plane triangular weave ; 1- vertical L-shaped triangular weave; 2- clockwise circular weave; 3-counterclockwise circular weave; 4-plane sine weave; 5-vertical L-shaped sine weave; 6- vertical triangular weave; 7- Vertical sine weave
    * @param [in] weaveFrequency weave frequency(Hz)
    * @param [in] weaveIncStayTime Wait mode 0- period does not contain wait time; 1- Period contains the wait time
    * @param [in] weaveRange weave amplitude(mm)
    * @param [in] weaveLeftStayTime weave left residence time(ms)
    * @param [in] weaveRightStayTime weave right residence time(ms)
    * @param [in] weaveCircleRadio Circular wiggle-pullback ratio(0-100%)
    * @param [in] weaveStationary weave position wait, 0- position continue to move within the waiting time; 1- The position is stationary during the waiting time
    * @return Error code
    */
    int WeaveSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary);

Set weave parameters in real time
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Set weave parameters in real time
    * @param [in] weaveNum parameters number
    * @param [in] weaveType weave type：0- plane triangular weave ; 1- vertical L-shaped triangular weave; 2- clockwise circular weave; 3-counterclockwise circular weave; 4-plane sine weave; 5-vertical L-shaped sine weave; 6- vertical triangular weave; 7- Vertical sine weave
    * @param [in] weaveFrequency weave frequency(Hz)
    * @param [in] weaveIncStayTime Wait mode 0- period does not contain wait time; 1- Period contains the wait time
    * @param [in] weaveRange weave amplitude(mm)
    * @param [in] weaveLeftStayTime weave left residence time(ms)
    * @param [in] weaveRightStayTime weave right residence time(ms)
    * @param [in] weaveCircleRadio Circular wiggle-pullback ratio(0-100%)
    * @param [in] weaveStationary weave position wait, 0- position continue to move within the waiting time; 1- The position is stationary during the waiting time
    * @return Error code
    */
    int WeaveOnlineSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary);

Weave start
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Weave start
    * @param [in] weaveNum Weave welding parameter configuration number
    * @return Error code
    */
    int WeaveStart(int weaveNum);

Weave end
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Weave end
    * @param [in] weaveNum Weave welding parameter configuration number
    * @return Error code
    */
    int WeaveEnd(int weaveNum);

Forward wire feed
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Forward Wire Feed
    * @param [in] ioType 0-control box IO； 1-extend IO
    * @param [in] wireFeed wire control: 0-stop wire feed ；1-wire feed 
    * @return Error code
    */
    int SetForwardWireFeed(int ioType, int wireFeed);

Reverse wire feed
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief Reverse wire feed
    * @param [in] ioType 0-control box IO； 1-extend IO
    * @param [in] wireFeed wire control: 0-stop wire feed ；1-wire feed 
    * @return Error code
    */
    int SetReverseWireFeed(int ioType, int wireFeed);

Aspirated
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /**
    * @brief aspirated
    * @param [in] ioType 0-control box IO； 1-extend IO
    * @param [in] airControl aspirated control: 0-stop aspirated；1-aspirated
    * @return Error code
    */
    int SetAspirated(int ioType, int airControl);

Segment weld start
++++++++++++++++++++++++++++++++++

.. versionadded:: C#SDK-v1.0.4

.. code-block:: c#
    :linenos:

    /** 
    * @brief Segment weld start
    * @param [in] startDesePos Starting point Cartesian position
    * @param [in] endDesePos Ending point Cartesian position
    * @param [in] startJPos Starting point joint position
    * @param [in] endJPos Ending point joint position
    * @param [in] weldLength Weld length(mm)
    * @param [in] noWeldLength Length of unwelded section(mm)
    * @param [in] weldIOType 0-control box IO； 1-extend IO
    * @param [in] arcNum Welder configuration file number
    * @param [in] timeout Arcing timeout time
    * @param [in] isWeave is Weave or not 
    * @param [in] weaveNum Weave welding parameter configuration number
    * @param  [in] tool  Tool coordinate number, range [1~15]
    * @param  [in] user  Workpiece coordinate number, range [1~15] 7
    * @param  [in] vel  Percentage of speed, range [0~100] 
    * @param  [in] acc  Acceleration percentage, range [0~100], not open for now * @param  [in] ovl  Velocity scaling factor, range[0~100]
    * @param  [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm
    * @param  [in] epos  Position of expansion shaft, unit: mm
    * @param  [in] search  0- no wire seeking, 1- wire seeking
    * @param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos  The pose offset
    * @return Error code 
    */
    int SegmentWeldStart(DescPose startDesePos, DescPose endDesePos, JointPos startJPos, JointPos endJPos, double weldLength, double noWeldLength, int weldIOType, int arcNum, int weldTimeout,bool isWeave, int weaveNum, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, byte search, byte offset_flag, DescPose offset_pos);

Code example:
++++++++++++++++++++++++++++++++++

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
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

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
        robot.MoveL(startjointPos, startdescPose, 1, 0, 100, 100, 100, 0, exaxisPos, 0, 0, offdese);
        robot.ARCStart(0, 0, 1000);
        robot.WeaveStart(0);
        robot.MoveL(endjointPos, enddescPose, 1, 0, 100, 100, 100, 0, exaxisPos, 0, 0, offdese);
        robot.ARCEnd(0, 0, 1000);
        robot.WeaveEnd(0);
    }
