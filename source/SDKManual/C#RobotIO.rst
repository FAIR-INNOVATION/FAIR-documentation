Robot IO
============

.. toctree:: 
    :maxdepth: 5

Setting the control box digital outputs
++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting up the control box digital outputs.
    * @param [in] id io number, range [0~15]
    * @param [in] status 0-off, 1-on
    * @param [in] smooth 0-not smooth, 1-smooth
    * @param [in] block 0-Blocking, 1-Non-blocking
    * @return Error code
    */
    int SetDO(int id, byte status, byte smooth, byte block); 

Set the tool digital output
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the tool digital output
    * @param [in] id io number, range [0~1]
    * @param [in] status 0-off, 1-on
    * @param [in] smooth 0-not smooth, 1-smooth
    * @param [in] block 0-Blocking, 1-Non-blocking
    * @return Error code
    */
    int SetToolDO(int id, byte status, byte smooth, byte block); 

Set the control box analog output
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting up the control box analog outputs.
    * @param [in] id io number, range [0~1].
    * @param [in] value Percentage of current or voltage value, range [0~100] Corresponding to current value [0~20mA] or voltage [0~10V].
    * @param [in] block 0-blocking, 1-non-blocking
    * @return Error code
    */
    int SetAO(int id, float value, byte block). 

Set the tool analog output
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief set tool analog output
    * @param [in] id io number, range [0].
    * @param [in] value Percentage of current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    * @param [in] block 0-blocking, 1-non-blocking
    * @return Error code
    */
    int SetToolAO(int id, float value, byte block).

Set digital, analog output code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button14_Click(object sender, EventArgs e)
    {
        byte status = 1;
        byte smooth = 0;
        byte block = 0;
        byte di = 0, tool_di = 0;
        float ai = 0.0f, tool_ai = 0.0f;
        float value = 0.0f;


        for (int i = 0; i < 16; i++)
        {
            robot.SetDO(i, status, smooth, block);
            Thread.Sleep(300);
        }

        status = 0;

        for (int i = 0; i < 16; i++)
        {
            robot.SetDO(i, status, smooth, block);
            Thread.Sleep(300);
        }

        status = 1;

        for (int i = 0; i < 2; i++)
        {
            robot.SetToolDO(i, status, smooth, block);
            Thread.Sleep(1000);
        }

        status = 0;

        for (int i = 0; i < 2; i++)
        {
            robot.SetToolDO(i, status, smooth, block);
            Thread.Sleep(1000);
        }

        for (int i = 0; i < 100; i++)
        {
            robot.SetAO(0, i, block);
            Thread.Sleep(30);
        }

        for (int i = 0; i < 100; i++)
        {
            robot.SetToolAO(0, i, block);
            Thread.Sleep(30);
        }

    }

etting control box digital inputs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get control box digital inputs.
    * @param [in] id io number, range [0~15]
    * @param [in] block 0-blocking, 1-non-blocking
    * @param [out] result 0-low level, 1-high level
    * @return error code
    */    
    int GetDI(int id, byte block, ref byte level).

Get tool digital input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get instrumented digital input
    * @param [in] id io number, range [0~1]
    * @param [in] block 0-blocking, 1-non-blocking
    * @param [out] result 0-low level, 1-high level
    * @return error code
    */    
    int GetToolDI(int id, byte block, ref byte level); 

Get control box analog input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get control box analog inputs.
    * @param [in] id io number, range [0~1]
    * @param [in] block 0-blocking, 1-non-blocking
    * @param [out] result Input current or voltage value percentage, range [0~100] corresponding to current value [0~20mS] or voltage [0~10V]
    * @return error code
    */    
    int GetAI(int id, byte block, ref float persent). 

Get tool analog input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get tool analog input
    * @param [in] id io number, range [0]
    * @param [in] block 0-blocking, 1-non-blocking
    * @param [out] result Input current or voltage value percentage, range [0~100] corresponds to current value [0~20mS] or voltage [0~10V]
    * @return error code
    */    
    int GetToolAI(int id, byte block, ref float persent). 
 
Get the robot end record button status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get end-of-robot record button state.
    * @param [out] state Button state, 0-pressed, 1-unpressed.
    * @return Error code. 
    */ 
    int GetAxlePointRecordBtnState(ref byte state). 

Get the robot end DO output state
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get robot end DO output state. 
    * @param [out] do_state DO output state, do0~do1 corresponds to bit1~bit2, start from bit0. 
    * @return Error code 
    */ 
    int GetToolDO(ref byte do_state).

Get the DO output state of the machine controller
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get robot controller DO output state. 
    * @param [out] do_state_h DO output state, co0~co7 corresponds to bit0~bit7 
    * @param [out] do_state_l DO output state, do0~do7 corresponds to bit0~bit7
    * @return Error code 
    */ 
    int GetDO(ref int do_state_h, ref int do_state_l);   

Get robot DI, DO state code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button15_Click(object sender, EventArgs e)
    {
        byte status = 1;
        byte smooth = 0;
        byte block = 0;
        byte di = 0, tool_di = 0;
        float ai = 0.0f, tool_ai = 0.0f;
        float value = 0.0f;

        robot.GetDI(0, block, ref di);
        Console.WriteLine($"di0: {di}");

        tool_di = (byte)robot.GetToolDI(1, block, ref tool_di);
        Console.WriteLine($"tool_di1: {tool_di}");

        robot.GetAI(0, block, ref ai);
        Console.WriteLine($"ai0: {ai}");

        tool_ai = robot.GetToolAI(0, block, ref tool_ai);
        Console.WriteLine($"tool_ai0: {tool_ai}");

        byte _button_state = 0;
        robot.GetAxlePointRecordBtnState(ref _button_state);
        Console.WriteLine($"_button_state is: {_button_state}");

        byte tool_do_state = 0;
        robot.GetToolDO(ref tool_do_state);
        Console.WriteLine($"tool DO state is: {tool_do_state}");

        int do_state_h = 0;
        int do_state_l = 0;
        robot.GetDO(ref do_state_h, ref do_state_l);
        Console.WriteLine($"DO state high is: {do_state_h}\n DO state low is: {do_state_l}");
    }

Wait for control box digital input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Wait for control box digital input.
    * @param [in] id io number, range [0~15]
    * @param [in] status 0-off, 1-on
    * @param [in] max_time Maximum waiting time in ms
    * @param [in] opt Timeout policy, 0-program stops and prompts for timeout, 1-ignore timeout prompts and continue execution, 2-wait all the time.
    * @return error_code
    */
    int WaitDI(int id, byte status, int max_time, int opt). 

Wait for multiple digital inputs to the control box
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Waiting for control box multiplexed digital inputs.
    * @param [in] mode 0-multiplex with, 1-multiplex or
    * @param [in] id io number, bit0~bit7 corresponds to DI0~DI7, bit8~bit15 corresponds to CI0~CI7
    * @param [in] status 0-off, 1-on
    * @param [in] max_time Maximum wait time in ms.
    * @param [in] opt Timeout policy, 0 - program stops and prompts for timeout, 1 - ignores timeout prompts and continues execution, 2 - waits forever.
    * @return error_code
    */
    int WaitMultiDI(int mode, int id, byte status, int max_time, int opt). 

Wait for tool digital input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Waiting for instrumented digital input
    * @param [in] id io number, range [0~1]
    * @param [in] status 0-off, 1-on
    * @param [in] max_time Maximum wait time in ms
    * @param [in] opt Timeout policy, 0-program stops and prompts for timeout, 1-ignore timeout prompts and continue execution, 2-wait all the time.
    * @return error_code
    */
    int WaitToolDI(int id, byte status, int max_time, int opt); 

Wait for control box analog input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Waiting for control box analog input.
    * @param [in] id io number, range [0~1]
    * @param [in] sign 0-greater than, 1-less than
    * @param [in] value Input current or voltage value percentage, range [0~100] corresponding to current value [0~20mS] or voltage [0~10V]
    * @param [in] max_time Maximum wait time in ms
    * @param [in] opt Policy after timeout, 0-program stops and prompts for timeout, 1-ignore timeout and prompt program to continue, 2-always wait
    * @return error_code
    */
    int WaitAI(int id, int sign, float value, int max_time, int opt);   

Wait for tool analog input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Waiting for tool analog input
    * @param [in] id io number, range [0]
    * @param [in] sign 0-greater than, 1-less than
    * @param [in] value Input current or voltage value percentage, range [0~100] corresponding to current value [0~20mS] or voltage [0~10V]
    * @param [in] max_time Maximum wait time in ms
    * @param [in] opt Policy after timeout, 0-program stops and prompts for timeout, 1-ignore timeout and prompt program to continue, 2-always wait
    * @return error_code
    */
    int WaitToolAI(int id, int sign, float value, int max_time, int opt). 

Wait for the control box digital, analog input signal code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnIOTest_Click(object sender, EventArgs e)
    {
        byte status = 1;
        byte smooth = 0;
        byte block = 0;
        byte di = 0, tool_di = 0;
        float ai = 0.0f, tool_ai = 0.0f;
        float value = 0.0f;

        int rtn = robot.WaitDI(0, 1, 1000, 1);
        Console.WriteLine("WaitDI over; rtn is: " + rtn);

        robot.WaitMultiDI(1, 3, 3, 1000, 1);
        Console.WriteLine("WaitMultiDI over; rtn is: " + rtn);

        robot.WaitToolDI(1, 1, 1000, 1);
        Console.WriteLine("WaitToolDI over; rtn is: " + rtn);

        robot.WaitAI(0, 0, 50, 1000, 1);
        Console.WriteLine("WaitAI over; rtn is: " + rtn);

        robot.WaitToolAI(0, 0, 50, 1000, 1);
        Console.WriteLine("WaitToolAI over; rtn is: " + rtn);
    }

Set Whether Control Box DO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set whether control box DO output resets after stop/pause
    * @param [in] resetFlag 0-Do not reset; 1-Reset
    * @param [in] reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetCtlBoxDO(int resetFlag, int reloadFlag);

Set Whether Control Box AO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set whether control box AO output resets after stop/pause
    * @param [in] resetFlag 0-Do not reset; 1-Reset
    * @param [in] reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetCtlBoxAO(int resetFlag, int reloadFlag);

Set Whether End Tool DO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set whether end tool DO output resets after stop/pause
    * @param [in] resetFlag 0-Do not reset; 1-Reset
    * @param [in] reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetAxleDO(int resetFlag, int reloadFlag);

Set Whether End Tool AO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set whether end tool AO output resets after stop/pause
    * @param [in] resetFlag 0-Do not reset; 1-Reset
    * @param [in] reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetAxleAO(int resetFlag, int reloadFlag);

Set Whether Extended DO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set whether extended DO output resets after stop/pause
    * @param [in] resetFlag 0-Do not reset; 1-Reset
    * @param [in] reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetExtDO(int resetFlag, int reloadFlag);

Set Whether Extended AO Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set whether extended AO output resets after stop/pause
    * @param [in] resetFlag 0-Do not reset; 1-Reset
    * @param [in] reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetExtAO(int resetFlag, int reloadFlag);

Set Whether SmartTool Output Resets After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set whether SmartTool output resets after stop/pause
    * @param [in] resetFlag 0-Do not reset; 1-Reset
    * @param [in] reloadFlag Whether to reload after pause resume, 0-Do not load; 1-Load
    * @return Error code
    */
    public int SetOutputResetSmartToolDO(int resetFlag, int reloadFlag);

Code Example for Setting Output Reset After Lua Program Stop/Pause
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    public void TestDOReset()
    {
        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();

        for (int i = 0; i < 16; i++)
        {
            robot.SetDO(i, 1, 0, 0);
            Thread.Sleep(200);
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

        Thread.Sleep(2000);
        robot.PauseMotion();
        Thread.Sleep(2000);
        robot.ResumeMotion();
        Thread.Sleep(2000);
    }