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
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

Set Configurable CI Port Functions of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set configurable CI port functions of the control box
    * @param [in] config CI0-CI7 function codes;
    * 0-None;1-Arc start success;2-Welder ready;3-Conveyor detection;4-Pause;5-Resume;6-Start;7-Stop;
    8-Pause/Resume;9-Start/Stop;10-Pedal drag;11-Move to home position;12-Manual/Auto switch;
    13-Wire search success;14-Motion interrupt;15-Start main program;16-Start rewind;17-Start confirmation;
    18-Photoelectric detection signal X;19-Photoelectric detection signal Y;20-External emergency stop input signal 1;21-External emergency stop input signal 2;
    22-Level 1 reduction mode;23-Level 2 reduction mode;24-Level 3 reduction mode (Stop);25-Resume welding;26-Terminate welding;
    27-Assist drag enable;28-Assist drag disable;29-Assist drag enable/disable;30-Clear all errors;
    31-Manual/Auto switch (high/low level);32-Enable;33-Disable;34-Enable/Disable (rising/falling edge);35-Fixed-point tracking start/end
    * @return Error code
    */
    public int SetDIConfig(int[] config)

Get Configurable CI Port Functions of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get configurable CI port functions of the control box
    * @param [in] config CI0-CI7 function codes;
    * 0-None;1-Arc start success;2-Welder ready;3-Conveyor detection;4-Pause;5-Resume;6-Start;7-Stop;
    8-Pause/Resume;9-Start/Stop;10-Pedal drag;11-Move to home position;12-Manual/Auto switch;
    13-Wire search success;14-Motion interrupt;15-Start main program;16-Start rewind;17-Start confirmation;
    18-Photoelectric detection signal X;19-Photoelectric detection signal Y;20-External emergency stop input signal 1;21-External emergency stop input signal 2;
    22-Level 1 reduction mode;23-Level 2 reduction mode;24-Level 3 reduction mode (Stop);25-Resume welding;26-Terminate welding;
    27-Assist drag enable;28-Assist drag disable;29-Assist drag enable/disable;30-Clear all errors;
    31-Manual/Auto switch (high/low level);32-Enable;33-Disable;34-Enable/Disable (rising/falling edge);35-Fixed-point tracking start/end
    * @return Error code
    */
    public int GetDIConfig(out int[] config)

Set Configurable CO Port Functions of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set configurable CO port functions of the control box
    * @param [out] config CO0-CO7 function codes;
    * 0-None;1-Robot error;2-Robot in motion;3-Spray start/stop;4-Spray gun cleaning;5-Gas supply signal;6-Arc start signal;7-Jog wire feed;
    8-Reverse wire feed;9-JOB input port 1;10-JOB input port 2;11-JOB input port 3;12-Conveyor start/stop control;13-Robot paused;14-Reached home position;
    15-Reached interference area;16-Wire search start/stop control;17-Robot start completed;18-Program start/stop;19-Auto/Manual mode;20-Emergency stop output signal 1-Safety;
    21-Emergency stop output signal 2-Safety;22-Lua script program running/stopped;23-Safety status output-Safety;24-Protective stop status output-Safety;
    25-Robot in motion-Safety;26-Robot reduced mode-Safety;27-Robot non-reduced mode-Safety;28-Robot non-stopped;29-Robot error-Instruction point error;
    30-Robot error-Driver error;31-Robot error-Soft limit exceeded error;32-Robot error-Collision error;33-Robot error-Active slave count error;
    34-Robot error-Slave error;35-Robot error-IO error;36-Robot error-Gripper error;37-Robot error-File error;38-Robot error-Singular pose error;
    39-Robot error-Driver communication error;40-Robot error-Parameter error;41-Robot error-External axis soft limit exceeded error;42-Robot warning-Warning;
    43-Robot warning-Safety door warning;44-Robot warning-Motion warning;45-Robot warning-Interference area warning;46-Robot warning-Safety wall warning;
    47-Enable status;48-Auto lift during disconnection;49-Cube 1 interference warning;50-Cube 2 interference warning;51-Cube 3 interference warning;52-Cube 4 interference warning;
    * @return Error code
    */
    public int SetDOConfig(int[] config)

Get Configurable CO Port Functions of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get configurable CO port functions of the control box
    * @param [out] config CO0-CO7 function codes;
    * 0-None;1-Robot error;2-Robot in motion;3-Spray start/stop;4-Spray gun cleaning;5-Gas supply signal;6-Arc start signal;7-Jog wire feed;
    8-Reverse wire feed;9-JOB input port 1;10-JOB input port 2;11-JOB input port 3;12-Conveyor start/stop control;13-Robot paused;14-Reached home position;
    15-Reached interference area;16-Wire search start/stop control;17-Robot start completed;18-Program start/stop;19-Auto/Manual mode;20-Emergency stop output signal 1-Safety;
    21-Emergency stop output signal 2-Safety;22-Lua script program running/stopped;23-Safety status output-Safety;24-Protective stop status output-Safety;
    25-Robot in motion-Safety;26-Robot reduced mode-Safety;27-Robot non-reduced mode-Safety;28-Robot non-stopped;29-Robot error-Instruction point error;
    30-Robot error-Driver error;31-Robot error-Soft limit exceeded error;32-Robot error-Collision error;33-Robot error-Active slave count error;
    34-Robot error-Slave error;35-Robot error-IO error;36-Robot error-Gripper error;37-Robot error-File error;38-Robot error-Singular pose error;
    39-Robot error-Driver communication error;40-Robot error-Parameter error;41-Robot error-External axis soft limit exceeded error;42-Robot warning-Warning;
    43-Robot warning-Safety door warning;44-Robot warning-Motion warning;45-Robot warning-Interference area warning;46-Robot warning-Safety wall warning;
    47-Enable status;48-Auto lift during disconnection;49-Cube 1 interference warning;50-Cube 2 interference warning;51-Cube 3 interference warning;52-Cube 4 interference warning;
    * @return Error code
    */
    public int GetDOConfig(out int[] config)

Set Configurable End-CI Port Functions of the End-Effector
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set configurable End-CI port functions of the end-effector
    * @param [in] config End CI0-CI1 function codes;
    * 0-None;1-Drag teaching tool switch;2-Point recording signal;3-Manual/Auto switch (pulse signal);4-TPD recording start/stop;5-Pause motion;
    6-Resume motion;7-Start;8-Stop;9-Pause/Resume;10-Start/Stop;11-Force sensor assist drag enable;12-Force sensor assist drag disable;
    13-Force sensor assist drag enable/disable;14-Laser detection signal X;15-Laser detection signal Y;16-PTP motion to home position;17-Motion interrupt, stop current motion based on signal;
    18-Start main program;19-Start rewind;20-Start confirmation;21-Resume welding;22-Terminate welding;23-Clear error;24-Manual/Auto switch (high/low level);
    25-Enable;26-Disable;27-Enable/Disable;28-Laser servo tracking start/stop signal;
    * @return Error code
    */
    public int SetToolDIConfig(int[] config)

Get Configurable End-CI Port Functions of the End-Effector
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get configurable End-CI port functions of the end-effector
    * @param [out] config End CI0-CI1 function codes;
    * 0-None;1-Drag teaching tool switch;2-Point recording signal;3-Manual/Auto switch (pulse signal);4-TPD recording start/stop;5-Pause motion;
    6-Resume motion;7-Start;8-Stop;9-Pause/Resume;10-Start/Stop;11-Force sensor assist drag enable;12-Force sensor assist drag disable;
    13-Force sensor assist drag enable/disable;14-Laser detection signal X;15-Laser detection signal Y;16-PTP motion to home position;17-Motion interrupt, stop current motion based on signal;
    18-Start main program;19-Start rewind;20-Start confirmation;21-Resume welding;22-Terminate welding;23-Clear error;24-Manual/Auto switch (high/low level);
    25-Enable;26-Disable;27-Enable/Disable;28-Laser servo tracking start/stop signal;
    * @return Error code
    */
    public int GetToolDIConfig(out int[] config)
    
Set Configurable CI Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set configurable CI active state of the control box
    * @param [in] config CI0-CI7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    public int SetDIConfigLevel(int[] config)
        
Get Configurable CI Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get configurable CI active state of the control box
    * @param [out] config CI0-CI7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    public int GetDIConfigLevel(out int[] config)
        
Set Configurable CO Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set configurable CO active state of the control box
    * @param [in] config CO0-CO7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    public int SetDOConfigLevel(int[] config)

Get Configurable CO Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get configurable CO active state of the control box
    * @param [out] config CO0-CO7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    public int GetDOConfigLevel(out int[] config)
    
Set Configurable CI Active State of the End-Effector
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set configurable CI active state of the end-effector
    * @param [in] config CI0-CI1 port active state; 0-active high; 1-active low
    * @return Error code
    */
    public int SetToolDIConfigLevel(int[] config)
    
Get Configurable CI Active State of the End-Effector
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get configurable CI active state of the end-effector
    * @param [out] config CI0-CI1 port active state; 0-active high; 1-active low
    * @return Error code
    */
    public int GetToolDIConfigLevel(out int[] config)
    
Set Standard DI Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set standard DI active state of the control box
    * @param [in] config DI0-DI7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    public int SetStandardDILevel(int[] config)
    
Get Standard DI Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get standard DI active state of the control box
    * @param [out] config DI0-DI7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    public int GetStandardDILevel(out int[] config)

Set Standard DO Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set standard DO active state of the control box
    * @param [in] config DO0-DO7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    public int SetStandardDOLevel(int[] config)
    
Get Standard DO Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get standard DO active state of the control box
    * @param [out] config DO0-DO7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    public int GetStandardDOLevel(out int[] config)
        
Robot IO Configuration Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    public void TestIOConfig()
    {
        int rtn = 0;

        // ---------- Test configurable CI port functions ----------
        int[] setDIConfig = new int[] { 3, 9, 1, 4, 5, 6, 7, 8 };
        rtn = robot.SetDIConfig(setDIConfig);
        Console.WriteLine($"SetDIConfig rtn is {rtn}");

        // Use out parameter to receive the obtained configuration array
        int[] getDIConfig;
        rtn = robot.GetDIConfig(out getDIConfig);  
        Console.WriteLine($"GetDIConfig rtn is {rtn}, value is {string.Join(" ", getDIConfig)}");

        // ---------- Test configurable CO port functions ----------
        int[] setDOConfig = new int[] { 9, 10, 11, 12, 13, 14, 15, 16 };
        rtn = robot.SetDOConfig(setDOConfig);
        Console.WriteLine($"SetDOConfig rtn is {rtn}");

        int[] getDOConfig;
        rtn = robot.GetDOConfig(out getDOConfig);
        Console.WriteLine($"GetDOConfig rtn is {rtn}, value is {string.Join(" ", getDOConfig)}");

        // ---------- Test configurable End-CI port functions of the end-effector ----------
        int[] setToolDIConfig = new int[] { 17, 18 };
        rtn = robot.SetToolDIConfig(setToolDIConfig);
        Console.WriteLine($"SetToolDIConfig rtn is {rtn}");

        int[] getToolDIConfig;
        rtn = robot.GetToolDIConfig(out getToolDIConfig);
        Console.WriteLine($"GetToolDIConfig rtn is {rtn}, value is {string.Join(" ", getToolDIConfig)}");

        // ---------- Test configurable CI active state of the control box ----------
        int[] setDIConfigLevel = new int[] { 1, 1, 1, 1, 0, 0, 0, 0 };
        rtn = robot.SetDIConfigLevel(setDIConfigLevel);
        Console.WriteLine($"SetDIConfigLevel rtn is {rtn}");

        int[] getDIConfigLevel;
        rtn = robot.GetDIConfigLevel(out getDIConfigLevel);
        Console.WriteLine($"GetDIConfigLevel rtn is {rtn}, value is {string.Join(" ", getDIConfigLevel)}");

        // ---------- Test configurable CO active state of the control box ----------
        int[] setDOConfigLevel = new int[] { 0, 0, 0, 0, 1, 1, 1, 1 };
        rtn = robot.SetDIConfigLevel(setDOConfigLevel);
        Console.WriteLine($"SetDOConfigLevel rtn is {rtn}");

        int[] getDOConfigLevel;
        rtn = robot.GetDOConfigLevel(out getDOConfigLevel);
        Console.WriteLine($"GetDOConfigLevel rtn is {rtn}, value is {string.Join(" ", getDOConfigLevel)}");

        // ---------- Test configurable CI active state of the end-effector ----------
        int[] setToolDIConfigLevel = new int[] { 1, 0 };
        rtn = robot.SetToolDIConfigLevel(setToolDIConfigLevel);
        Console.WriteLine($"SetToolDIConfigLevel rtn is {rtn}");

        int[] getToolDIConfigLevel;
        rtn = robot.GetToolDIConfigLevel(out getToolDIConfigLevel);
        Console.WriteLine($"GetToolDIConfigLevel rtn is {rtn}, value is {string.Join(" ", getToolDIConfigLevel)}");

        // ---------- Test standard DI active state of the control box ----------
        int[] setStandardDILevel = new int[] { 1, 1, 1, 1, 0, 0, 0, 0 };
        rtn = robot.SetStandardDILevel(setStandardDILevel);
        Console.WriteLine($"SetStandardDILevel rtn is {rtn}");

        int[] getStandardDILevel;
        rtn = robot.GetStandardDILevel(out getStandardDILevel);
        Console.WriteLine($"GetStandardDILevel rtn is {rtn}, value is {string.Join(" ", getStandardDILevel)}");

        // ---------- Test standard DO active state of the control box ----------
        int[] setStandardDOLevel = new int[] { 0, 0, 0, 0, 1, 1, 1, 1 };
        rtn = robot.SetStandardDOLevel(setStandardDOLevel);
        Console.WriteLine($"SetStandardDOLevel rtn is {rtn}");

        int[] getStandardDOLevel;
        rtn = robot.GetStandardDOLevel(out getStandardDOLevel);
        Console.WriteLine($"GetStandardDOLevel rtn is {rtn}, value is {string.Join(" ", getStandardDOLevel)}");

    }