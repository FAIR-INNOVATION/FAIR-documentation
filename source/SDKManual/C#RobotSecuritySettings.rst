Security settings
========================

.. toctree:: 
    :maxdepth: 5

Set collision level
++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set collision level
    * @param  [in]  mode  0- grade, 1- percentage
    * @param  [in]  level Collision threshold, grade range [], percentage range [0~1]
    * @param  [in]  config 0- Do not update the configuration file. 1- Update the configuration file
    * @return  Error code
    */
    int SetAnticollision(int mode, double[] level, int config); 

Set the post-collision policy
++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the post-collision policy
    * @param  [in] strategy  0- Error stop, 1- Continue running
    * @return  Error code  
    */
    int SetCollisionStrategy(int strategy); 

Set the positive limit
++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the positive limit
    * @param  [in] limit Six joint positions, unit: deg
    * @return  Error code
    */
    int SetLimitPositive(double[] limit); 

Set the negative limit
++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the negative limit
    * @param  [in] limit Six joint positions, unit: deg
    * @return  Error code
    */
    int SetLimitNegative(double[] limit);   

Error status clearing
++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Error status clearing
    * @return  Error code
    */
    int ResetAllError(); 

Joint friction compensation switch
+++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Joint friction compensation switch
    * @param  [in]  state  0- off, 1- on
    * @return  Error code
    */
    int FrictionCompensationOnOff(byte state); 

Set joint friction compensation coefficient - formal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set joint friction compensation coefficient - formal
    * @param  [in]  coeff Six joint compensation coefficients, range [0~1]
    * @return  Error code
    */
    int SetFrictionValue_level(double[] coeff); 

Set joint friction compensation coefficient - side mount
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set joint friction compensation coefficient - side mount
    * @param  [in]  coeff Six joint compensation coefficients, range [0~1]
    * @return  Error code
    */
    int SetFrictionValue_wall(double[] coeff); 

Set joint friction compensation coefficient - reverse mount
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set joint friction compensation coefficient - inverted
    * @param  [in]  coeff Six joint compensation coefficients, range [0~1]
    * @return  Error code
    */
    int SetFrictionValue_ceiling(double[] coeff); 

Set joint friction compensation coefficient - free mount
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set joint friction compensation coefficient - free mount
    * @param  [in]  coeff Six joint compensation coefficients, range [0~1]
    * @return  Error code
    */
    int SetFrictionValue_freedom(double[] coeff); 

Code example
++++++++++++++
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
        Console.WriteLine($"SetLimitPositive  rtn  {rtn}");
        double[] nlimit = new double[6] { -170.0, -260.0, -150.0, -260.0, -170.0, -160.0 };
        rtn = robot.SetLimitNegative(nlimit);
        Console.WriteLine($"SetLimitNegative  rtn  {rtn}");

        robot.ResetAllError();

        double[] lcoeff = new double[6] { 0.9, 0.9, 0.9, 0.9, 0.9, 0.9 };
        double[] wcoeff = new double[6] { 0.4, 0.4, 0.4, 0.4, 0.4, 0.4 };
        double[] ccoeff = new double[6] { 0.6, 0.6, 0.6, 0.6, 0.6, 0.6 };
        double[] fcoeff = new double[6] { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
        robot.FrictionCompensationOnOff(1);
        rtn = robot.SetFrictionValue_level(lcoeff);
        Console.WriteLine($"SetFrictionValue_level  rtn  {rtn}");
        rtn = robot.SetFrictionValue_wall(wcoeff);
        Console.WriteLine($"SetFrictionValue_wall  rtn  {rtn}");
        rtn = robot.SetFrictionValue_ceiling(ccoeff);
        Console.WriteLine($"SetFrictionValue_ceiling  rtn  {rtn}");
        rtn = robot.SetFrictionValue_freedom(fcoeff);
        Console.WriteLine($"SetFrictionValue_freedom  rtn  {rtn}");
    }