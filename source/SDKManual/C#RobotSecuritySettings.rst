Robot Security Settings
==============================
.. toctree:: 
    :maxdepth: 5


Setting the collision level
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief Setting the collision level
    * @param [in] mode 0-rank, 1-percentage
    * @param [in] level Collision threshold, level corresponds to range [], percentage corresponds to range [0~1].
    * @param [in] config 0-does not update the config file, 1-updates the config file
    * @return error code
    */
    int SetAnticollision(int mode, double[] level, int config); 

Setting the post-collision strategy
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief Setting up post-collision strategies
    * @param [in] strategy 0-stop with error, 1-keep running
    * @param [in] safeTime safe stop time [1000 - 2000] ms
    * @param [in] safeDistance Safe stopping distance [1-150]mm
    * @param [in] safeVel tcp safe stopping speed [50-250]mm/s
    * @param [in] safetyMargin j1-j6 safety factor [1-10]
    * @return error code
    */
    int SetCollisionStrategy(int strategy, int safeTime, int safeDistance,  int safeVel, int[] safetyMargin); 

Setting the positive limit
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting positive limits
    * @param [in] limit Six joint positions in deg.
    * @return error code
    */
    int SetLimitPositive(double[] limit). 

Setting the negative limit
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting negative limits
    * @param [in] limit Six joint positions in deg.
    * @return error code
    */
    int SetLimitNegative(double[] limit). 

error state clearing
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Error status clearing
    * @return error code
    */
    int ResetAllError(). 

Joint Friction Compensation Switch
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Joint friction compensation switch 
    * @param [in] state 0-off, 1-on 
    * @return error code 
    */ 
    int FrictionCompensationOnOff(byte state). 

Setting the joint friction compensation coefficients - positive loading
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the joint friction compensation factor - formal wear
    * @param [in] coeff Six joint compensation coefficients, range [0 to 1].
    * @return error code
    */
    int SetFrictionValue_level(double[] coeff).

Setting the joint friction compensation coefficient - side mounting
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the joint friction compensation factor - side mounted
    * @param [in] coeff Six joint compensation coefficients, range [0 to 1].
    * @return error code
    */
    int SetFrictionValue_wall(double[] coeff). 

Setting the joint friction compensation factor - inverted
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the joint friction compensation factor - inverted
    * @param [in] coeff Six joint compensation coefficients, range [0 to 1].
    * @return error code
    */
    int SetFrictionValue_ceiling(double[] coeff).

Setting the joint friction compensation factor - free mounting
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the joint friction compensation factor - free mounting
    * @param [in] coeff Six joint compensation coefficients, range [0 to 1].
    * @return error code
    */
    int SetFrictionValue_freedom(double[] coeff).

Code Example
+++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnRobotSafetySet_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        int mode = 0;
        int config = 1;
        double[] level1 = new double[6] { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
        double[] level2 = new double[6] { 0.5, 0.2, 0.3, 0.4, 0.5, 0.12 };

        robot.SetAnticollision(mode, level1, config);
        mode = 1;
        robot.SetAnticollision(mode, level2, config);
        int[] safetyMargin = { 1, 1, 1, 1, 1, 1 };
        robot.SetCollisionStrategy(5, 1000, 150,150,safetyMargin);

        double[] plimit = new double[6] { 170.0, 80.0, 150.0, 80.0, 170.0, 160.0 };
        int rtn = robot.SetLimitPositive(plimit);
        Console.WriteLine($"SetLimitPositive rtn {rtn}");
        double[] nlimit = new double[6] { -170.0, -260.0, -150.0, -260.0, -170.0, -160.0 };
        rtn = robot.SetLimitNegative(nlimit);
        Console.WriteLine($"SetLimitNegative rtn {rtn}");

        robot.ResetAllError();

        double[] lcoeff = new double[6] { 0.9, 0.9, 0.9, 0.9, 0.9, 0.9 };
        double[] wcoeff = new double[6] { 0.4, 0.4, 0.4, 0.4, 0.4, 0.4 };
        double[] ccoeff = new double[6] { 0.6, 0.6, 0.6, 0.6, 0.6, 0.6 };
        double[] fcoeff = new double[6] { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
        robot.FrictionCompensationOnOff(1).
        rtn = robot.SetFrictionValue_level(lcoeff);
        Console.WriteLine($"SetFrictionValue_level rtn {rtn}");
        rtn = robot.SetFrictionValue_wall(wcoeff);
        Console.WriteLine($"SetFrictionValue_wall rtn {rtn}");
        rtn = robot.SetFrictionValue_ceiling(ccoeff);
        Console.WriteLine($"SetFrictionValue_ceiling rtn {rtn}");
        rtn = robot.SetFrictionValue_freedom(fcoeff);
        Console.WriteLine($"SetFrictionValue_freedom rtn {rtn}");
    }

Customized Collision Detection Thresholds Feature Begins
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     /**
    * @brief Customize the collision detection threshold function to start, set the collision detection threshold on the joint side and the TCP side
    * @param [in] flag 1-joint detection only on; 2-TCP detection only on; 3-joint and TCP detection both on
    * @param [in] jointDetectionThreshould Joint collision detection threshold j1-j6
    * @param [in] tcpDetectionThreshould TCP collision detection threshold, xyzabc
    * @param [in] block 0-non-blocking; 1-blocking
    * @return Error code
    */
    int CustomCollisionDetectionStart(int flag, double[] jointDetectionThreshould, double[] tcpDetectionThreshould, int block);

Customize collision detection threshold function off
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Customize collision detection threshold function off
    * @return Error code
    */
    int CustomCollisionDetectionEnd()

Code Example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnRobotSafetySet_Click(object sender, EventArgs e)
    {
        while (true)
        {
            int[] safety = { 5, 5, 5, 5, 5, 5 };
            robot.SetCollisionStrategy(3, 1000, 150, 250, safety);

            double[] jointDetectionThreshold = { 0.3, 0.3, 0.3, 0.3, 0.3, 0.3 };
            double[] tcpDetectionThreshold = { 80, 80, 80, 80, 80, 80 };
            int rtn = robot.CustomCollisionDetectionStart(3, jointDetectionThreshold, tcpDetectionThreshold, 0);
            Console.WriteLine($"CustomCollisionDetectionStart rtn is {rtn}");

            DescPose p1Desc = new DescPose(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
            JointPos p1Joint = new JointPos(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);

            DescPose p2Desc = new DescPose(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
            JointPos p2Joint = new JointPos(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);

            ExaxisPos exaxisPos = new ExaxisPos(0.0, 0.0, 0.0, 0.0);
            DescPose offdese = new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

            // Assuming the signature of the MoveL method is as follows:
            robot.MoveL(p1Joint, p1Desc, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);
            robot.MoveL(p2Joint, p2Desc, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese);

            rtn = robot.CustomCollisionDetectionEnd();
            Console.WriteLine($"CustomCollisionDetectionEnd rtn is {rtn}");
        }
    }