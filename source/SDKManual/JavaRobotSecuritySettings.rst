Robot Security Settings
=====================================

.. toctree:: 
    :maxdepth: 5

Setting the collision level
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the collision level
    * @param [in] mode 0-rank, 1-percentage
    * @param [in] level collision threshold, level corresponds to the range [1 - 10 corresponds to level 1-10, 100 - off], percentage corresponds to the range [0 to 10 corresponds to 0% - 100%].
    * @param [in] config 0-does not update the config file, 1-updates the config file
    * @return error code
    */
    int SetAnticollision(int mode, Object[] level, int config); 

Setting the post-collision strategy
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting up post-collision strategies
    * @param [in] strategy 0-stop with error, 1-keep running
    * @param [in] safeTime safe stop time [1000 - 2000] ms
    * @param [in] safeDistance Safe stopping distance [1-150]mm
    * @param [in] safetyMargin j1-j6 safety factor [1-10]
    * @return error code  
    */
    int SetCollisionStrategy(int strategy, int safeTime, int safeDistance, int safetyMargin[]); 

Setting the positive limit
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting positive limits
    * @param [in] limit Six joint positions in deg.
    * @return error code
    */
    int SetLimitPositive(Object[] limit). 

Setting the negative limit
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting negative limits
    * @param [in] limit Six joint positions in deg.
    * @return error code
    */
    int SetLimitNegative(Object[] limit). 

error state clearing
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Error status clearing
    * @return error code
    */
    int ResetAllError(); 

Code example
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
        Object[] config = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
        robot.SetAnticollision(0, config, 1);
        int safetyMargin[]={10,10,10,10,10,10,10};
        robot.SetCollisionStrategy(0,1000,10,safetyMargin);

        robot.ProgramLoad("/fruser/test.lua");
        robot.ProgramRun();//run lua file

        Object[] plimit = { 170.0, 80.0, 150.0, 80.0, 170.0, 160.0 };
        robot.SetLimitPositive(plimit);

        Object[] nlimit = { -170.0, -260.0, -150.0, -260.0, -170.0, -160.0 };
        robot.SetLimitNegative(nlimit);

        robot.SetLoadWeight(123.0);
        robot.Sleep(3000);
        robot.ResetAllError();
    }

Customized collision detection threshold function starts
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.3-3.8.0

.. code-block:: Java
    :linenos:

    /**
    * @brief Customize collision detection threshold function start, set collision detection threshold for joint side and TCP side
    * @param [in] flag 1-Joint detection only on; 2-TCP detection only on; 3-Joint and TCP detection both on
    * @param [in] jointDetectionThreshould Joint collision detection threshold j1-j6
    * @param [in] tcpDetectionThreshould TCP collision detection threshold, xyzabc
    * @param [in] block 0-non-blocking; 1-blocking
    * @return error code
    */   
    public int CustomCollisionDetectionStart(int flag, double[] jointDetectionThreshould, double[] tcpDetectionThreshould, int block);

Customized collision detection threshold function off
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.3-3.8.0

.. code-block:: Java
    :linenos:

    /**
    * @brief  Customized collision detection threshold function off
    * @return  error code
    */   
    public int CustomCollisionDetectionEnd();

Code Example
+++++++++++++++++++++++++++++++++++++++++++++
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
        int[] safety = { 5,5,5,5,5,5 };
        robot.SetCollisionStrategy(3, 1000, 150, 250, safety);
        double[] jointDetectionThreshould= { 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
        double[] tcpDetectionThreshould = { 60,60,60,60,60,60 };
        int rtn = robot.CustomCollisionDetectionStart(1, jointDetectionThreshould, tcpDetectionThreshould, 1);

        DescPose p1Desc=new DescPose(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
        JointPos p1Joint=new JointPos(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);

        DescPose p2Desc=new DescPose(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
        JointPos p2Joint=new JointPos(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);

        ExaxisPos exaxisPos=new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offdese=new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        for(int i=0;i<10;++i) {
            robot.MoveL(p1Joint, p1Desc, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 10);
            robot.MoveL(p2Joint, p2Desc, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 10);
        }
        rtn = robot.CustomCollisionDetectionEnd();
    }

Joint Friction Compensation Switch
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Joint Friction Compensation Switch 
    * @param [in] state 0-off, 1-on
    * @return error code 
    */ 
    int FrictionCompensationOnOff(int state). 

Setting the joint friction compensation coefficients - positive loading
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the joint friction compensation factor - formal wear
    * @param [in] coeff Six joint compensation coefficients, range [0~1].
    * @return error code
    */
    int SetFrictionValue_level(Object[] coeff).

Setting the joint friction compensation coefficient - side mounting
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the joint friction compensation factor - side mounted
    * @param [in] coeff Six joint compensation coefficients, range [0~1].
    * @return error code
    */
    int SetFrictionValue_wall(Object[] coeff). 

Setting the Joint Friction Compensation Factor - Inverted
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the joint friction compensation factor - inverted
    * @param [in] coeff Six joint compensation coefficients, range [0~1].
    * @return error code
    */
    int SetFrictionValue_ceiling(Object[] coeff).

Setting the joint friction compensation factor - free mounting
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the joint friction compensation factor - free mounting
    * @param [in] coeff Six joint compensation coefficients, range [0~1].
    * @return error code
    */
    int SetFrictionValue_freedom(Object[] coeff).

Code example
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
        Object[] lcoeff = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
        Object[] wcoeff = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
        Object[] ccoeff = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
        Object[] fcoeff = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };

        robot.FrictionCompensationOnOff(1).

        robot.SetFrictionValue_level(lcoeff);//formalwear

        robot.SetFrictionValue_wall(wcoeff);// side loading

        robot.SetFrictionValue_ceiling(ccoeff);//inversion

        robot.SetFrictionValue_freedom(fcoeff);//freedom installation
    }

Starting odd position protection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief begins odd-position protection
    * @param [in] protectMode singular protection mode, 0: articulated mode; 1 - Cartesian mode
    * @param [in] minShoulderPos Shoulder singularity adjustment range (mm), default 100
    * @param [in] minElbowPos elbow singularity adjustment range (mm), default 50
    * @param [in] minWristPos wrist singularity adjustment range (°), default 10
    * @return error code
    */
    int SingularAvoidStart(int protectMode, double minShoulderPos, double minElbowPos, double minWristPos);;

Stop odd position protection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief stops odd-position protection
    * @return error code
    */
    int SingularAvoidEnd().

Code example
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
        DescPose startdescPose=new DescPose(-402.473, -185.876, 103.985, -175.367, 59.682, 94.221);
        JointPos startjointPos=new JointPos(-0.095, -50.828, 109.737, -150.708, -30.225, -0.623);

        DescPose enddescPose=new DescPose(-399.264, -184.434, 296.022, -4.402, 58.061, -94.161);
        JointPos endjointPos=new JointPos(-0.095, -65.547, 105.145, -131.397, 31.851, -0.622);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0, 0, 0);

        robot.MoveL(startjointPos, startdescPose, 0, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        robot.SingularAvoidStart(0, 150, 50, 20);
        robot.MoveL(endjointPos, enddescPose, 0, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        robot.SingularAvoidEnd();
    }
