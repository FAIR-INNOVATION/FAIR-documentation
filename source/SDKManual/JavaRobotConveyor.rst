conveyor belts
=====================================

.. toctree:: 
    :maxdepth: 5

Drive belt start and stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Drive belt start, stop
    * @param [in] status status, 1-start, 0-stop
    * @return error code
    */
    int ConveyorStartEnd(int status).

Record IO detection points
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Record IO detection points
    * @return error code
    */
    int ConveyorPointIORecord().

Record point A
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Record point A
    * @return error code
    */
    int ConveyorPointARecord(). 

Recording reference points
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Record reference points
    * @return error code
    */
    int ConveyorRefPointRecord();

Record point B
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Record point B
    * @return error code
    */
    int ConveyorPointBRecord(). 

Drive Belt Parameter Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Drive belt parameterization
    * @param [in] encChannel encoder channel 1~2
    * @param [in] resolution Number of pulses for one revolution of the encoder
    * @param [in] lead Distance traveled by the encoder for one conveyor revolution
    * @param [in] wpAxis Workpiece coordinate system number Select the workpiece coordinate system number for the tracking motion function, and set the tracking grab and TPD tracking to 0.
    * @param [in] vision whether to match vision 0 no 1 yes
    * @param [in] speedRadio speedRatio For conveyor tracking grab options (1-100) Other options default to 1
    * @return error code
    */
    int ConveyorSetParam(int encChannel, int resolution, double lead, int wpAxis, int vision, double speedRadio); 

Setting the drive belt gripping point compensation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting drive belt grab point compensation
    * @param [in] cmp compensation position double[3]{x, y, z}
    * @return error code 
    */ 
    int ConveyorCatchPointComp(Object[] cmp).

Conveyorized workpiece IO inspection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Conveyorized Workpiece IO Inspection
    * @param [in] max_t Maximum detection time in ms
    * @return error code 
    */ 
    int ConveyorIODetect(int max_t).

Get the current position of the object
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get the current position of the object
    * @param [in] mode 1-tracking grab, 2-tracking motion, 3-TPD tracking
    * @return error code 
    */ 
    int ConveyorGetTrackData(int mode).

Drive belt tracking started
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Drivetrain tracking begins *
    * @param [in] status status, 1-start, 0-stop
    * @return error code 
    */ 
    int ConveyorTrackStart(int status).

Belt tracking stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Drive belt tracking stop
    * @return error code 
    */ 
    int ConveyorTrackEnd().

linear motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Linear motion
    * @param [in] name Campaign point description
    * @param [in] tool tool coordinate number, range [0~14].
    * @param [in] wobj Workpiece coordinate number, range [0~14]
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm
    * @return error code 
    */ 
    int ConveyorTrackMoveL(String name, int tool, int wobj, double vel, double acc, double ovl, double blendR);   

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
        int rtn = -1;
        rtn = robot.ConveyorPointIORecord();//record the IO entry point
        System.out.println("ConveyorPointIORecord: rtn " + rtn);

        rtn = robot.ConveyorPointARecord();//record point A
        System.out.println("ConveyorPointARecord: rtn " + rtn);

        rtn = robot.ConveyorRefPointRecord();//record reference points
        System.out.println("ConveyorRefPointRecord: rtn " + rtn);

        rtn = robot.ConveyorPointBRecord();//record point B
        System.out.println("ConveyorPointBRecord: rtn " + rtn);

        //Configuration of conveyor belts
        robot.ConveyorSetParam(1, 10000, 2.0, 1, 1, 20);
        System.out.println("ConveyorSetParam: rtn " + rtn);
        //Conveyor Tracking Grab
        DescPose pos1 = new DescPose(-351.549,87.914,354.176,-179.679,-0.134,2.468);
        DescPose pos2 = new DescPose(-351.203,-213.393,351.054,-179.932,-0.508,2.472);

        Object[] cmp = {0.0, 0.0, 0.0};
        rtn = robot.ConveyorCatchPointComp(cmp);//set drive belt catch point compensation
        if(rtn ! = 0)
        {
            return;
        }
        System.out.println("ConveyorCatchPointComp: rtn " + rtn);

        rtn = robot.MoveCart(pos1, 1, 0, 30.0, 180.0, 100.0, -1.0, -1);
        System.out.println("MoveCart: rtn " + rtn);

        rtn = robot.ConveyorIODetect(10000);// conveyor workpiece IO detection
        System.out.println("ConveyorIODetect: rtn " + rtn);

        robot.ConveyorGetTrackData(1);//configure the conveyor belt to track the capture
        rtn = robot.ConveyorTrackStart(1);//tracking start
        System.out.println("ConveyorTrackStart: rtn " + rtn);

        rtn = robot.ConveyorTrackMoveL("cvrCatchPoint", 1, 0, 100.0, 0.0, 100.0, -1.0);
        System.out.println("ConveyorTrackMoveL: rtn " + rtn);

        rtn = robot.MoveGripper(1, 60, 60, 30, 30000, 0);
        System.out.println("MoveGripper: rtn {rtn}");

        rtn = robot.ConveyorTrackMoveL("cvrRaisePoint", 1, 0, 100.0, 0.0, 100.0, -1.0);
        System.out.println("ConveyorTrackMoveL: rtn " + rtn);

        rtn = robot.ConveyorTrackEnd();// conveyor tracking stops
        System.out.println("ConveyorTrackEnd: rtn " + rtn);

        rtn = robot.MoveCart(pos2, 1, 0, 30.0, 180.0, 100.0, -1.0, -1);
        System.out.println("MoveCart: rtn " + rtn);

        rtn = robot.MoveGripper(1, 100, 60, 30, 30000, 0);
        System.out.println("MoveGripper: rtn " + rtn);
    } 