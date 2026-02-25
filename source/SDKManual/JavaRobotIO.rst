Robot IO
========================

.. toctree:: 
    :maxdepth: 5

Set control box digital output
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Set control box digital output
    * @param  [in] id  IO number, range [0~15]
    * @param  [in] status 0-off, 1-on
    * @param  [in] smooth 0-no smoothing, 1-smoothing
    * @param  [in] block  0-blocking, 1-non-blocking
    * @return  Error code
    */
    int SetDO(int id, int status, int smooth, int block); 

Set tool digital output
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Set tool digital output
    * @param  [in] id  IO number, range [0~1]
    * @param  [in] status 0-off, 1-on
    * @param  [in] smooth 0-no smoothing, 1-smoothing
    * @param  [in] block  0-blocking, 1-non-blocking
    * @return  Error code
    */
    int SetToolDO(int id, int status, int smooth, int block); 

Set control box analog output
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Set control box analog output
    * @param  [in] id  IO number, range [0~1]
    * @param  [in] value Current or voltage value percentage, range [0~100] corresponding to current [0~20mA] or voltage [0~10V]
    * @param  [in] block  0-blocking, 1-non-blocking
    * @return  Error code
    */
    int SetAO(int id, double value, int block); 

Set tool analog output
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Set tool analog output
    * @param  [in] id  IO number, range [0]
    * @param  [in] value Current or voltage value percentage, range [0~100] corresponding to current [0~20mA] or voltage [0~10V]
    * @param  [in] block  0-blocking, 1-non-blocking
    * @return  Error code
    */
    int SetToolAO(int id, double value, int block); 

Digital and analog output setting example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestAODO(Robot robot)
    {
        int status = 1;
        int smooth = 0;
        int block = 0;

        for (int i = 0; i < 16; i++)
        {
            robot.SetDO(i, status, smooth, block);
            robot.Sleep(300);
        }

        status = 0;

        for (int i = 0; i < 16; i++)
        {
            robot.SetDO(i, status, smooth, block);
            robot.Sleep(300);
        }

        status = 1;

        for (int i = 0; i < 2; i++)
        {
            robot.SetToolDO(i, status, smooth, block);
            robot.Sleep(1000);
        }

        status = 0;

        for (int i = 0; i < 2; i++)
        {
            robot.SetToolDO(i, status, smooth, block);
            robot.Sleep(1000);
        }

        for (int i = 0; i < 100; i++)
        {
            robot.SetAO(0, i, block);
            robot.Sleep(30);
        }

        for (int i = 0; i < 100; i++)
        {
            robot.SetToolAO(0, i, block);
            robot.Sleep(30);
        }

        robot.CloseRPC();
        return 0;
    }

Get control box digital input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get control box digital input
    * @param  [in] id  IO number, range [0~15]
    * @param  [in] block  0-blocking, 1-non-blocking
    * @param  [out] level  0-low level, 1-high level
    * @return  Error code
    */   
    int GetDI(int id, int block, int[] level);

Get tool digital input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get tool digital input
    * @param  [in] id    IO number, range [0~1]
    * @param  [in] block  0-blocking, 1-non-blocking
    * @param  [out] level 0-low level, 1-high level
    * @return  Error code
    */   
    int GetToolDI(int id, int block, int[] level);

Get control box analog input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get control box analog input
    * @param  [in] id  IO number, range [0~1]
    * @param  [in] block  0-blocking, 1-non-blocking
    * @param  [out] persent Input current or voltage value percentage, range [0~100] corresponding to current [0~20mA] or voltage [0~10V]
    * @return  Error code
    */   
    int GetAI(int id, int block, double[] persent)

Get tool analog input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get tool analog input
    * @param  [in] id  IO number, range [0]
    * @param  [in] block  0-blocking, 1-non-blocking
    * @param  [out] persent Input current or voltage value percentage, range [0~100] corresponding to current [0~20mA] or voltage [0~10V]
    * @return  Error code
    */   
    int GetToolAI(int id, int block, double[] persent)

Get robot end-point record button state
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get robot end-point record button state
    * @param  [out] state Button state, 0-pressed, 1-released
    * @return  Error code
    */   
    int GetAxlePointRecordBtnState(int[] state)

Get robot end DO output state
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get robot end DO output state
    * @param  [out] do_state DO output state, do0~do1 corresponds to bit1~bit2, starting from bit0
    * @return  Error code
    */   
    int GetToolDO(int[] do_state)

Get robot controller DO output state
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief  Get robot controller DO output state
    * @param  [out] do_state_h DO output state, co0~co7 corresponds to bit0~bit7
    * @param  [out] do_state_l DO output state, do0~do7 corresponds to bit0~bit7
    * @return  Error code
    */   
    int GetDO(int[] do_state_h, int[] do_state_l)

Get robot DI/DO state example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestGetDIAI(Robot robot)
    {
        int status = 1;
        int smooth = 0;
        int block = 0;
        int[] di =new int[]{0}, tool_di =new int[] {0};
        double[] ai =new double[] {0}, tool_ai = new double[]{0};
        double value = 0.0;

        robot.GetDI(0, block, di);
        System.out.println("di0:"+di[0]);

        robot.GetToolDI(1, block, tool_di);
        System.out.println("tool_di1:"+ tool_di[0]);

        robot.GetAI(0, block, ai);
        System.out.println("ai0:"+ ai[0]);

        robot.GetToolAI(0, block, tool_ai);
        System.out.println("tool_ai0:"+ tool_ai[0]);

        int[] _button_state=new int[]{0};
        robot.GetAxlePointRecordBtnState(_button_state);
        System.out.println("_button_state is: "+ _button_state[0]);

        int[] tool_do_state=new int[]{0};
        robot.GetToolDO(tool_do_state);
        System.out.println("tool DO state is: "+ tool_do_state[0]);

        int[] do_state_h=new int[]{0};
        int[] do_state_l=new int[]{0};
        robot.GetDO(do_state_h, do_state_l);
        System.out.println("DO state high is: "+do_state_h[0]+", DO state low is: "+ do_state_l[0]);

        return 0;
    }

Wait for control box digital input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Wait for control box digital input
    * @param  [in]  id  IO number, range [0~15]
    * @param  [in]  status 0-off, 1-on
    * @param  [in]  max_time  Maximum wait time in ms
    * @param  [in]  opt  Timeout strategy, 0-program stops and prompts timeout, 1-ignore timeout and continue, 2-wait indefinitely
    * @return  Error code
    */
    int WaitDI(int id, int status, int max_time, int opt); 

Wait for multiple control box digital inputs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Wait for multiple control box digital inputs
    * @param  [in] mode 0-multi AND, 1-multi OR
    * @param  [in] id  IO number, bit0~bit7 corresponds to DI0~DI7, bit8~bit15 corresponds to CI0~CI7
    * @param  [in] status 0-off, 1-on
    * @param  [in] max_time  Maximum wait time in ms
    * @param  [in] opt  Timeout strategy, 0-program stops and prompts timeout, 1-ignore timeout and continue, 2-wait indefinitely
    * @return  Error code
    */
    int WaitMultiDI(int mode, int id, int status, int max_time, int opt); 

Wait for tool digital input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Wait for tool digital input
    * @param  [in]  id  IO number, range [0~1]
    * @param  [in]  status 0-off, 1-on
    * @param  [in]  max_time  Maximum wait time in ms
    * @param  [in]  opt  Timeout strategy, 0-program stops and prompts timeout, 1-ignore timeout and continue, 2-wait indefinitely
    * @return  Error code
    */
    int WaitToolDI(int id, int status, int max_time, int opt); 

Wait for control box analog input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Wait for control box analog input
    * @param  [in]  id  IO number, range [0~1]
    * @param  [in]  sign 0-greater than, 1-less than
    * @param  [in]  value Input current or voltage value percentage, range [0~100] corresponding to current [0~20mA] or voltage [0~10V]
    * @param  [in]  max_time  Maximum wait time in ms
    * @param  [in]  opt  Timeout strategy, 0-program stops and prompts timeout, 1-ignore timeout and continue, 2-wait indefinitely
    * @return  Error code
    */
    int WaitAI(int id, int sign, double value, int max_time, int opt);   

Wait for tool analog input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Wait for tool analog input
    * @param  [in]  id  IO number, range [0]
    * @param  [in]  sign 0-greater than, 1-less than
    * @param  [in]  value Input current or voltage value percentage, range [0~100] corresponding to current [0~20mA] or voltage [0~10V]
    * @param  [in]  max_time  Maximum wait time in ms
    * @param  [in]  opt  Timeout strategy, 0-program stops and prompts timeout, 1-ignore timeout and continue, 2-wait indefinitely
    * @return  Error code
    */
    int WaitToolAI(int id, int sign, double value, int max_time, int opt); 

Wait for digital/analog input signal example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestWaitDIAI(Robot robot)
    {
        int rtn=-1;

        int status = 1;
        int smooth = 0;
        int block = 0;
        int di = 0, tool_di = 0;
        double ai = 0.0, tool_ai = 0.0;
        double value = 0.0;

        rtn = robot.WaitDI(0, 1, 1000, 1);
        System.out.println("WaitDI over; rtn is: "+ rtn);

        robot.WaitMultiDI(1, 3, 3, 1000, 1);
        System.out.println("WaitDI over; rtn is: "+ rtn);

        robot.WaitToolDI(1, 1, 1000, 1);
        System.out.println("WaitDI over; rtn is: " + rtn);

        robot.WaitAI(0, 0, 50, 1000, 1);
        System.out.println("WaitDI over; rtn is: " + rtn);

        robot.WaitToolAI(0, 0, 50, 1000, 1);
        System.out.println("WaitDI over; rtn is: " + rtn);
        return 0;
    }

Set Whether Control Box DO Output Resets After Stop/Pause
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set whether control box DO output resets after stop/pause
    * @param resetFlag 0-Do not reset; 1-Reset
    * @param reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetCtlBoxDO(int resetFlag, int reloadFlag)

Set Whether Control Box AO Output Resets After Stop/Pause
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set whether control box AO output resets after stop/pause
    * @param resetFlag 0-Do not reset; 1-Reset
    * @param reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetCtlBoxAO(int resetFlag, int reloadFlag)

Set Whether End Tool DO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set whether end tool DO output resets after stop/pause
    * @param resetFlag 0-Do not reset; 1-Reset
    * @param reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetAxleDO(int resetFlag, int reloadFlag)

Set Whether End Tool AO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set whether end tool AO output resets after stop/pause
    * @param resetFlag 0-Do not reset; 1-Reset
    * @param reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetAxleAO(int resetFlag, int reloadFlag)
    
Set Whether Extended DO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set whether extended DO output resets after stop/pause
    * @param resetFlag 0-Do not reset; 1-Reset
    * @param reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetExtDO(int resetFlag, int reloadFlag)
    
Set Whether Extended AO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set whether extended AO output resets after stop/pause
    * @param resetFlag 0-Do not reset; 1-Reset
    * @param reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetExtAO(int resetFlag, int reloadFlag)

Set Whether SmartTool Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set whether SmartTool output resets after stop/pause
    * @param resetFlag 0-Do not reset; 1-Reset
    * @param reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetSmartToolDO(int resetFlag, int reloadFlag)

Code Example for Setting Output Reset After Lua Program Stop/Pause
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void TestDOReset(Robot robot)
    {
        for (int i = 0; i < 16; i++)
        {
            robot.SetDO(i, 1, 0, 0);
            robot.Sleep(200);
        }
        int resetFlag = 1;
        int resumeReloadFlag = 1;
        int rtn = robot.SetOutputResetCtlBoxDO(resetFlag, resumeReloadFlag);
        robot.SetOutputResetCtlBoxAO(resetFlag, resumeReloadFlag);
        robot.SetOutputResetAxleDO(resetFlag, resumeReloadFlag);
        robot.SetOutputResetAxleAO(resetFlag, resumeReloadFlag);
        robot.SetOutputResetExtDO(resetFlag, resumeReloadFlag);
        robot.SetOutputResetExtAO(resetFlag, resumeReloadFlag);
        robot.SetOutputResetSmartToolDO(resetFlag, resumeReloadFlag);
        robot.ProgramLoad("/fruser/test.lua");
        robot.ProgramRun();
        robot.Sleep(2000);
        robot.PauseMotion();
        robot.Sleep(2000);
        robot.ResumeMotion();
        robot.Sleep(2000);
        robot.CloseRPC();
    }