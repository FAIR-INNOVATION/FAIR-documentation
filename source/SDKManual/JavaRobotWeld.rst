Robotic welding
=====================================

.. toctree:: 
    :maxdepth: 5

Welding Start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Welding started
    * @param [in] ioType io type 0-controller IO; 1-extended IO
    * @param [in] arcNum Welder profile number
    * @param [in] timeout Arc start timeout
    * @return error code
    */
    int ARCStart(int ioType, int arcNum, int timeout);

End of welding
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief End of welding
    * @param [in] ioType io type 0-controller IO; 1-extended IO
    * @param [in] arcNum Welder profile number
    * @param [in] timeout Arc off timeout
    * @return error code
    */
    int ARCEnd(int ioType, int arcNum, int timeout);

Setting of welding current and output analog correspondences
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set weld current to output analog
    * @param [in] relation relation value
    * @return error code
    */
    int WeldingSetCurrentRelation(WeldCurrentAORelation relation).

Setting the welding voltage and output analog correspondence
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the weld voltage to output analog
    * @param [in] relation Welding voltage-analog output relation value
    * @return error code
    */
    int WeldingSetVoltageRelation(WeldVoltageAORelation relation).

Acquiring the correspondence between welding current and output analog quantity
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get weld current to analog output
    * @param [out] relation relation value
    * @return error code
    */
    int WeldingGetCurrentRelation(WeldCurrentAORelation relation).

Getting welding voltage and output analog correspondence
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get weld voltage to output analog correspondence
    * @param [out] relation Welding voltage-analog output relation value
    * @return error code
    */
    int WeldingGetVoltageRelation(WeldVoltageAORelation relation).

code example
++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        DescPose desc_p1, desc_p2.
        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0, 0);

        JointPos j1 = new JointPos(-28.529,-140.397,-81.08,-30.934,92.34,-5.629);
        JointPos j2 = new JointPos(-11.045,-130.984,-104.495,-12.854,92.475,-5.547);

        robot.GetForwardKin(j1,desc_p1);
        robot.GetForwardKin(j2,desc_p2);

        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();
        robot.MoveL(j1, desc_p1,0, 0, 20, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
        robot.ARCStart(0, 0, 10000);//welding start
        robot.MoveL(j2, desc_p2,0, 0, 20, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
        robot.ARCEnd(0, 0, 10000);//end of welding

        WeldCurrentAORelation currentRelation = new WeldCurrentAORelation(0, 1000, 0, 10, 0);
        robot.WeldingSetCurrentRelation(currentRelation).
        WeldVoltageAORelation voltageAORelation = new WeldVoltageAORelation(0, 100, 0, 10, 1);
        robot.WeldingSetVoltageRelation(voltageAORelation).

        WeldCurrentAORelation tmpCur = new WeldCurrentAORelation();
        WeldVoltageAORelation tmpVol = new WeldVoltageAORelation();
        robot.WeldingGetCurrentRelation(tmpCur).
        robot.WeldingGetVoltageRelation(tmpVol).
    } 

Setting the welding current
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set weld current
    * @param [in] ioType Control IO type 0-Control box IO; 1-Extended IO
    * @param [in] current Welding current value (A)
    * @param [in] AOIndex Welding current control box analog output port (0-1)
    * @param [in] blend whether smooth or not 0-not smooth; 1-smooth
    * @return error code
    */
    int WeldingSetCurrent(int ioType, double current, int AOIndex, int blend);

Setting the welding voltage
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set weld voltage
    * @param [in] ioType Control IO type 0-Control box IO; 1-Extended IO
    * @param [in] voltage Welding voltage value (A)
    * @param [in] AOIndex Welding voltage control box analog output port (0-1)
    * @param [in] blend whether smooth or not 0-not smooth; 1-smooth
    * @return error code
    */
    int WeldingSetVoltage(int ioType, double voltage, int AOIndex, int blend);

Setting Oscillation Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the swing parameters
    * @param [in] weaveNum weave parameter configuration number
    * @param [in] weaveType weaveType weaveType 0-planar triangular weave; 1-vertical L-shaped triangular weave; 2-clockwise circular weave; 3-counterclockwise circular weave; 4-planar sine weave; 5-vertical L-shaped sine weave; 6-vertical triangular weave; 7-vertical sine weave
    * @param [in] weaveFrequency swing frequency (Hz)
    * @param [in] weaveIncStayTime wait mode 0-cycle without wait time; 1-cycle with wait time
    * @param [in] weaveRange swing range (mm)
    * @param [in] weaveLeftRange Vertical triangle swing left chord length (mm)
    * @param [in] weaveRightRange Vertical triangle weave right chord length (mm)
    * @param [in] additionalStayTime Vertical triangle swing vertical triangle point stay time (mm)
    * @param [in] weaveLeftStayTime weaveLeftStayTime (ms)
    * @param [in] weaveRightStayTime weaveRightStayTime (ms)
    * @param [in] weaveCircleRadio Circle swing-back ratio (0-100%)
    * @param [in] weaveStationary swing position wait, 0 - wait time for position to continue moving; 1 - wait time for position to be stationary
    * @param [in] weaveYawAngle swing direction azimuth (rotation around swing Z axis) in °.
    * @return error code
    */
    int WeaveSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, double weaveLeftRange, double weaveRightRange, int additionalStayTime, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary, double weaveYawAngle).

Instant setup of swing parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Instant setup of swing parameters
    * @param [in] weaveNum weave parameter configuration number
    * @param [in]weaveType swing type 0-planar triangular swing; 1-vertical L-shaped triangular swing; 2-clockwise circular swing; 3-counterclockwise circular swing; 4-planar sinusoidal swing; 5-vertical L-shaped sinusoidal swing; 6-vertical triangular swing; 7-vertical sinusoidal swing
    * @param [in]weaveFrequency swing frequency (Hz)
    * @param [in]weaveIncStayTime wait mode 0-cycle without wait time; 1-cycle with wait time
    * @param [in]weaveRange swing range (mm)
    * @param [in]weaveLeftStayTime swing left stay time (ms)
    * @param [in]weaveRightStayTime weaveRightStayTime (ms)
    * @param [in]weaveCircleRadio Circle swing-back ratio (0-100%)
    * @param [in]weaveStationary swing position wait, 0 - position continues to move during wait time; 1 - position is stationary during wait time
    * @return error code
    */
    int WeaveOnlineSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary).

swing start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief The swing is on
    * @param [in] weaveNum weave parameter configuration number
    * @return error code
    */
    int WeaveStart(int weaveNum).

end of swing (math.)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief End of swing
    * @param [in] weaveNum weave parameter configuration number
    * @return error code
    */
    int WeaveEnd(int weaveNum).

code example
++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.WeldingSetCurrent(0, 500, 0, 0);
        robot.WeldingSetVoltage(0, 60, 1, 0);
        robot.WeaveSetPara(0,0, 2.0, 0, 0.0, 0, 0, 0, 100, 100, 50, 50,1);

        DescPose desc_p1 = new DescPose(688.259,-566.358,-139.354,-158.206,0.324,-117.817);
        DescPose desc_p2 = new DescPose(700.078,-224.752,-149.191,-158.2,0.239,-94.978);


        JointPos j1 = new JointPos(0,0,0,0,0,0,0);
        JointPos j2 = new JointPos(0,0,0,0,0,0,0);

        robot.GetInverseKin(0, desc_p1, -1, j1);
        robot.GetInverseKin(0, desc_p2, -1, j2);

        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();

        robot.MoveL(j1, desc_p1,3, 0, 30, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
        robot.WeaveSetPara(0,0, 1.0, 0, 10.0, 0, 0, 0, 100, 100, 50, 50,1);
        robot.ARCStart(0, 0, 10000);
        robot.WeaveStart(0);
        robot.MoveL(j2, desc_p2,3, 0, 10, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
        robot.ARCEnd(0, 0, 10000);
        robot.WeaveEnd(0);
    } 

Positive wire feed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Positive wire feed
    * @param [in] ioType io type 0-Controller IO; 1-Extended IO
    * @param [in] wireFeed Wire Feed Control 0-Stop Wire Feed; 1-Feed Wire
    * @return error code
    */
    int SetForwardWireFeed(int ioType, int wireFeed); 	

Reverse wire feed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Reverse Wire Feed
    * @param [in] ioType io type 0-Controller IO; 1-Extended IO
    * @param [in] wireFeed Wire Feed Control 0-Stop Wire Feed; 1-Feed Wire
    * @return error code
    */
    int SetReverseWireFeed(int ioType, int wireFeed);

aspiration (phonetics, explosion of breath on consonants distinguishing Chinese p, t from b, d)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief sends gas
    * @param [in] ioType io type 0-Controller IO; 1-Extended IO
    * @param [in] airControl airControl 0-stop air delivery; 1-feed air
    * @return error code
    */
    int SetAspirated(int ioType, int airControl).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.SetForwardWireFeed(0, 1);
        robot.Sleep(2000);
        robot.SetForwardWireFeed(0, 0);
        robot.Sleep(2000);
        robot.SetReverseWireFeed(0, 1);
        robot.Sleep(2000);
        robot.SetReverseWireFeed(0, 0);
        robot.Sleep(2000);

        robot.SetAspirated(0,1);
        robot.Sleep(2000);
        robot.SetAspirated(0,0);
    }

segment welding
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Start of segment welding
    * @param [in] startDesePos Starting point Cartesian position
    * @param [in] endDesePos endDesePos Cartesian Positions
    * @param [in] startJPos Starting point joint position
    * @param [in] endJPos endJoint Positions
    * @param [in] weldLength Welding section length (mm)
    * @param [in] noWeldLength Non-Weld Length (mm)
    * @param [in] weldIOType Weld IO type (0-control box IO; 1-extended IO)
    * @param [in] arcNum Welder profile number
    * @param [in] weldTimeout 起/收弧超时时间时间
    * @param [in] isWeave whether or not to swing
    * @param [in] weaveNum weave parameter configuration number
    * @param [in] tool tool number
    * @param [in] user Workpiece number
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0 to 100].
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm
    * @param [in] epos Extended axis position in mm
    * @param [in] search 0-no wire seek, 1-wire seek
    * @param [in] offset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos bit position offset
    * @return error code 
    */
    int SegmentWeldStart(DescPose startDesePos, DescPose endDesePos, JointPos startJPos, JointPos endJPos, double weldLength, double noWeldLength, int weldIOType,int arcNum, int weldTimeout, boolean isWeave, int weaveNum, int tool, int user, double vel, double acc, double ovl, double blendR, ExaxisPos epos, int search, int offset_flag, DescPose offset_pos);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        DescPose startdescPose = new DescPose(185.859,-520.154,193.129,-177.129,1.339,-137.789);
        JointPos startjointPos = new JointPos(-60.989,-94.515,-89.479,-83.514,91.957,-13.124);

        DescPose enddescPose = new DescPose( -243.7033,-543.868,143.199,-177.954,1.528,177.758);
        JointPos endjointPos = new JointPos(-105.479,-101.919,-87.979,-78.455,91.955,-13.183);

        ExaxisPos exaxisPos = new ExaxisPos( 0, 0, 0, 0, 0 );
        DescPose offdese = new DescPose( 0, 0, 0, 0, 0, 0, 0, 0 );

        robot.SegmentWeldStart(startdescPose, enddescPose, startjointPos, endjointPos, 80, 40, 0, 0, 5000, true, 0, 3, 0, 30, 30, 100, -1, exaxisPos, 0, 0, offdese).
    }

Welding wire position finding start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Welding wire seek start
    * @param [in] refPos 1-datum 2-contact point
    * @param [in] searchVel searchVelocity %
    * @param [in] searchDis seek distance mm
    * @param [in] autoBackFlag autoBackFlag, 0-not automatic; - automatic
    * @param [in] autoBackVel autoBackVel %
    * @param [in] autoBackDis autoBackDistance mm
    * @param [in] offectFlag 1 - offset seek; 2 - teach point seek
    * @return error code 
    */
    int WireSearchStart(int refPos, double searchVel, int searchDis, int autoBackFlag, double autoBackVel, int autoBackDis, int offectFlag);;

End of wire position finding
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief End of wire seek.
    * @param [in] refPos 1-datum 2-contact point
    * @param [in] searchVel searchVelocity %
    * @param [in] searchDis seek distance mm
    * @param [in] autoBackFlag autoBackFlag, 0-not automatic; - automatic
    * @param [in] autoBackVel autoBackVel %
    * @param [in] autoBackDis autoBackDistance mm
    * @param [in] offectFlag 1 - offset seek; 2 - teach point seek
    * @return error code 
    */
    int WireSearchEnd(int refPos, double searchVel, int searchDis, int autoBackFlag, double autoBackVel, int autoBackDis, int offectFlag);;

Calculate the wire finding offset
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Calculate wire seek offset
    * @param [in] seamType Weld Type
    * @param [in] method Calculation method
    * @param [in] varNameRef datums 1-6, "#" denotes a pointless variable
    * @param [in] varNameRes Contacts 1-6, "#" denotes dotless variable
    * @param [out] offset offset position [x, y, z, a, b, c] and offset mode
    * @return error code 
    */
    int GetWireSearchOffset(int seamType, int method, String[] varNameRef, String[] varNameRes, DescOffset offset);

Waiting for wire finding to complete
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Waiting for the wire to complete its search.
    * @return error code 
    */
    int WireSearchWait(String name).

Welding wire seek contact points written to database
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Welding wire seeker contacts written to database
    * @param [in] varName Contact name "RES0" ~ "RES99"
    * @param [in] pos contact point data [x, y, x, a, b, c]
    * @return error code 
    */
    int SetPointToDatabase(String varName, DescPose pos);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection successful");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);

        DescPose descStart = new DescPose(153.736,-715.249,-295.037,-179.829,2.613,-155.615);
        JointPos jointStart = new JointPos(0,0,0,0,0,0,0,0);

        DescPose descEnd = new DescPose(73.748,-645.825,-295.016,-179.828,2.608,-155.614);
        JointPos jointEnd = new JointPos(0,0,0,0,0,0,0,0);

        robot.GetInverseKin(0, descStart, -1, jointStart);
        robot.GetInverseKin(0, descEnd, -1, jointEnd);

        robot.MoveL(jointStart, descStart, 3, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);
        robot.MoveL(jointEnd, descEnd, 3, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese,0, 100);

        DescPose descREF0A = new DescPose(273.716,-723.539,-295.075,-179.829,2.608,-155.614);
        JointPos jointREF0A = new JointPos(0,0,0,0,0,0,0,0);

        DescPose descREF0B = new DescPose(202.588,-723.543,-295.039,-179.829,2.609,-155.614);
        JointPos jointREF0B = new JointPos(0,0,0,0,0,0,0,0);

        DescPose descREF1A = new DescPose(75.265,-525.091,-295.059,-179.83,2.609,-155.616);
        JointPos jointREF1A = new JointPos(0,0,0,0,0,0,0,0);

        DescPose descREF1B = new DescPose(75.258,-601.157,-295.075,-179.834,2.609,-155.616);
        JointPos jointREF1B = new JointPos(0,0,0,0,0,0,0,0);

        robot.GetInverseKin(0, descREF0A, -1, jointREF0A);
        robot.GetInverseKin(0, descREF0B, -1, jointREF0B);
        robot.GetInverseKin(0, descREF1A, -1, jointREF1A);
        robot.GetInverseKin(0, descREF1B, -1, jointREF1B);

        robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF0A, descREF0A, 3, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100); //starting point
        robot.MoveL(jointREF0B, descREF0B, 3, 0, 30, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100); //direction point
        robot.WireSearchWait("REF0");
        robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF1A, descREF1A, 3, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100); //starting point
        robot.MoveL(jointREF1B, descREF1B, 3, 0, 30, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100); //direction point
        robot.WireSearchWait("REF1");
        robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);


        robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF0A, descREF0A, 3, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100); //starting point
        robot.MoveL(jointREF0B, descREF0B, 3, 0, 30, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100); //direction point
        robot.WireSearchWait("RES0");
        robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF1A, descREF1A, 3, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100); //starting point
        robot.MoveL(jointREF1B, descREF1B, 3, 0, 30, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100); //direction point
        robot.WireSearchWait("RES1");
        robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        String[] varNameRef = { "REF0", "REF1", "#", "#", "#", "#", "#" };
        String[] varNameRes = { "RES0", "RES1", "#", "#", "#", "#" };

        DescOffset offectPos = new DescOffset();
        robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes, offectPos);
        robot.PointsOffsetEnable(offectPos.offsetFlag, offectPos.offset);
        robot.MoveL(jointStart, descStart, 3, 1, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);
        robot.MoveL(jointEnd, descEnd, 3, 1, 30, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);
        robot.PointsOffsetDisable(); robot.
    }

Arc tracking control
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Arc tracking control
    * @param [in] flag switch, 0-off; 1-on
    * @param [in] delaytime lag time in ms
    * @param [in] isLeftRight Left/Right deviation compensation
    * @param [in] klr left/right adjustment factor (sensitivity)
    * @param [in] tStartLr left/right start compensation time cyc
    * @param [in] stepMaxLr Maximum compensation in mm at a time for left and right.
    * @param [in] sumMaxLr Left/Right Total Maximum Compensation mm
    * @param [in] isUpLow Up and down deviation compensation
    * @param [in] kud up/down adjustment factor (sensitivity)
    * @param [in] tStartUd up and down start compensation time cyc
    * @param [in] stepMaxUd Maximum compensation in mm for each up and down step.
    * @param [in] sumMaxUd up and down total maximum compensation
    * @param [in] axisSelect upper and lower coordinate system selection, 0-swing; 1-tool; 2-base
    * @param [in] referenceType upper and lower reference current setting mode, 0-feedback; 1-constant
    * @param [in] referSampleStartUd upper and lower reference current sample start count (feedback), cyc
    * @param [in] referSampleCountUd Upper and lower reference current sampling cycle count (feedback), cyc
    * @param [in] referenceCurrent upper and lower reference current mA
    * @return error code 
    */
    int ArcWeldTraceControl(int flag, double delaytime, int isLeftRight, double klr, double tStartLr, double stepMaxLr, double sumMaxLr, int isUpLow, double kud, double tStartUd, double stepMaxUd, double sumMaxUd, int axisSelect, int referenceType, double referSampleStartUd, double referSampleCountUd, double referenceCurrent).

Simulated swing start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief simulation of the start of the swing
    * @param [in] weaveNum swing parameter number
    * @return error code 
    */
    int WeaveStartSim(int weaveNum).

End of simulation swing
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Simulation of the end of the swing
    * @param [in] weaveNum swing parameter number
    * @return error code 
    */
    int WeaveEndSim(int weaveNum).

Start trajectory detection warning (no movement)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Start trajectory detection warning (no movement)
    * @param [in] weaveNum swing parameter number
    * @return error code 
    */
    int WeaveInspectStart(int weaveNum).

End trajectory detection warning (no movement)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief End trajectory detection warning (no movement)
    * @param [in] weaveNum swing parameter number
    * @return error code 
    */
    int WeaveInspectEnd(int weaveNum).

Setting Welding Process Curve Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting welding process profile parameters
    * @param [in] id Welding process number (1-99)
    * @param [in] param Welding process parameters
    * @return error code 
    */
    int WeldingSetProcessParam(int id, WeldingProcessParam param);

Obtaining Welding Process Curve Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Obtain welding process curve parameters
    * @param [in] id Welding process number (1-99)
    * @param [out] param Welding process parameters
    * @return error code 
    */
    int WeldingGetProcessParam(int id, WeldingProcessParam param);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        WeldingProcessParam param = new WeldingProcessParam(177.0,27.0,1000,178.0,28.0,176.0,26.0,1000);
        robot.WeldingSetProcessParam(1, param);

        WeldingProcessParam getParam = new WeldingProcessParam();
        robot.WeldingGetProcessParam(1, getParam);
    }

Extended IO-Configuration Welder Gas Detection Signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Extended IO-Configuration Welder Gas Detection Signal
    * @param [in] DONum Gas Detection Signal Extension DO Number
    * @return error code 
    */
    int SetAirControlExtDoNum(int DONum).

Extended IO-Configuration of welder arc start signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Extended IO-Configuration welder arc start signaling
    * @param [in] DONum Welder Arc Start Signal Extension DO Number
    * @return error code 
    */
    int SetArcStartExtDoNum(int DONum).

Extended IO-Configuration of the welder's reverse wire feed signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Extended IO-Configuration Welder Reverse Wire Feed Signal
    * @param [in] DONum Reverse Wire Feed Signal Extension DO No.
    * @return error code 
    */
    int SetWireReverseFeedExtDoNum(int DONum).

Extended IO-Configuration of the welder's forward wire feed signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Extended IO-Configuration Welder Positive Wire Feed Signal
    * @param [in] DONum Forward Wire Feed Signal Extension DO No.
    * @return error code 
    */
    int SetWireForwardFeedExtDoNum(int DONum).

Extended IO-Configuration of the welder arc start success signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Extended IO-Configuration Welder Arc Start Success Signal
    * @param [in] DINum Arc Success Signal Extension DI Number
    * @return error code 
    */
    int SetArcDoneExtDiNum(int DINum).

Extended IO-Configuration Welder Ready Signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Extended IO-Configuration Welder Ready Signal
    * @param [in] DINum Welder Ready Signal Extension DI Number
    * @return error code 
    */
    int SetWeldReadyExtDiNum(int DINum).

Extended IO-Configuration Weld Interrupt Recovery Signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Extended IO-Configuration Weld Interrupt Recovery Signaling
    * @param [in] reWeldDINum Resume Weld Signal Extension DI Number after Weld Interruption
    * @param [in] abortWeldDINum Exit Weld Signal Expansion DI Number after Weld Interruption
    * @return error code 
    */
    int SetExtDIWeldBreakOffRecover(int reWeldDINum, int abortWeldDINum);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.SetArcStartExtDoNum(1);
        robot.SetAirControlExtDoNum(2);
        robot.SetWireForwardFeedExtDoNum(3);
        robot.SetWireReverseFeedExtDoNum(4);

        robot.SetWeldReadyExtDiNum(5);
        robot.SetArcDoneExtDiNum(6);
        robot.SetExtDIWeldBreakOffRecover(7, 8);
    }

Setting up the Weld Wire Seek Expansion IO Port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting up the Weld Wire Seeker Expansion IO Port
    * @param [in] searchDoneDINum Welding Wire Successful DO Port (0-127)
    * @param [in] searchStartDONum Welding wire seek start/stop control DO port (0-127)
    * @return error code
    */
    int SetWireSearchExtDIONum(int searchDoneDINum, int searchStartDONum);

Set welder control mode to expand DO port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting up welder control mode to expand DO ports
    * @param [in] DONum Welder Control Mode DO Port (0-127)
    * @return error code 
    */
    int SetWeldMachineCtrlModeExtDoNum(int DONum).

Setting the welder control mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief set welder control mode
    * @param [in] mode Welder control mode;0-unitary
    * @return error code 
    */
    int SetWeldMachineCtrlMode(int mode).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        UDPComParam param = new UDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10);
        robot.ExtDevSetUDPComParam(param);//udp extended axis communication
        robot.ExtDevLoadUDPDriver();

        robot.SetWeldMachineCtrlModeExtDoNum(17);//DO
        for (int i = 0; i < 5; i++)
        {
            robot.SetWeldMachineCtrlMode(0);//set weld machine control mode
            robot.Sleep(500);
            robot.SetWeldMachineCtrlMode(1);
            robot.Sleep(500);
        }

        robot.SetWeldMachineCtrlModeExtDoNum(18);
        for (int i = 0; i < 5; i++)
        {
            robot.SetWeldMachineCtrlMode(0);
            robot.Sleep(500);
            robot.SetWeldMachineCtrlMode(1);
            robot.Sleep(500);
        }
    }