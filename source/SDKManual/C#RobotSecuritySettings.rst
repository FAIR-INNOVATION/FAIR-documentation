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
    * @param [in] safetyMargin j1-j6 safety factor [1-10]
    * @return error code
    */
    int SetCollisionStrategy(int strategy, int safeTime, int safeDistance, int[] safetyMargin); 

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

code example
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
        robot.SetCollisionStrategy(2);

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