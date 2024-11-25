Robotics IO
===============================================

.. toctree:: 
    :maxdepth: 5


Setting up the control box digital outputs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

Set the digital output of the tool
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting up the control box analog outputs
    * @param [in] id io number, range [0~1].
    * @param [in] value Percentage of current or voltage value, range [0~100] Corresponding to current value [0~20mA] or voltage [0~10V].
    * @param [in] block 0-blocking, 1-non-blocking
    * @return Error code
    */
    int SetAO(int id, float value, byte block). 

Set the tool analog output
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the tool analog output
    * @param [in] id io number, range [0].
    * @param [in] value Percentage of current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    * @param [in] block 0-blocking, 1-non-blocking
    * @return Error code
    */
    int SetToolAO(int id, float value, byte block). 

Get the control box digital input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get control box digital inputs.
    * @param [in] id io number, range [0~15]
    * @param [in] block 0-blocking, 1-non-blocking
    * @param [out] result 0-low level, 1-high level
    * @return Error code
    */   
    int GetDI(int id, byte block, ref byte level).

Get tool digital input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the tool digital input
    * @param [in] id io number, range [0~1]
    * @param [in] block 0-blocking, 1-non-blocking
    * @param [out] result 0-low level, 1-high level
    * @return Error code
    */   
    int GetToolDI(int id, byte block, ref byte level); 

Wait for control box digital input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Waiting for control box digital inputs.
    * @param [in] id io number, range [0~15]
    * @param [in] status 0-off, 1-on
    * @param [in] max_time Maximum wait time in ms
    * @param [in] opt Timeout policy, 0-program stops and prompts for timeout, 1-ignore timeout prompts and continue execution, 2-wait all the time.
    * @return error_code
    */
    int WaitDI(int id, byte status, int max_time, int opt). 

Wait for multiple digital inputs to the control box
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Waiting for tool digital input
    * @param [in] id io number, range [0~1]
    * @param [in] status 0-off, 1-on
    * @param [in] max_time Maximum wait time in ms
    * @param [in] opt Timeout policy, 0-program stops and prompts for timeout, 1-ignore timeout prompts and continue execution, 2-wait all the time.
    * @return error_code
    */
    int WaitToolDI(int id, byte status, int max_time, int opt); 

Get the control box analog input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief get control box analog inputs
    * @param [in] id io number, range [0~1]
    * @param [in] block 0-blocking, 1-non-blocking
    * @param [out] result Input current or voltage value percentage, range [0~100] corresponding to current value [0~20mS] or voltage [0~10V]
    * @return error code
    */   
    int GetAI(int id, byte block, ref float persent). 

Get tool analog input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

Get robot end record button status
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get end-of-robot logging button state
    * @param [out] state Button state, 0-pressed, 1-unpressed.
    * @return Error code. 
    */ 
    int GetAxlePointRecordBtnState(ref byte state); 

Wait for control box analog input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief waiting for control box analog inputs
    * @param [in] id io number, range [0~1]
    * @param [in] sign 0-greater than, 1-less than
    * @param [in] value Input current or voltage value percentage, range [0~100] corresponds to current value [0~20mS] or voltage [0~10V]
    * @param [in] max_time Maximum wait time in ms
    * @param [in] opt Policy after timeout, 0-program stops and prompts for timeout, 1-ignore timeout and prompt program to continue, 2-always wait
    * @return error_code
    */
    int WaitAI(int id, int sign, float value, int max_time, int opt);   

Waiting for tool analog inputs
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Waiting for tool analog input
    * @param [in] id io number, range [0]
    * @param [in] sign 0-greater than, 1-less than
    * @param [in] value Percentage of input current or voltage value, range [0~100] corresponding to current value [0~20mS] or voltage [0~10V]
    * @param [in] max_time Maximum wait time in ms
    * @param [in] opt Policy after timeout, 0-program stops and prompts for timeout, 1-ignore timeout and prompt program to continue, 2-always wait
    * @return error_code
    */
    int WaitToolAI(int id, int sign, float value, int max_time, int opt); 

Get the status of the DO output at the end of the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the DO output state of the robot end. 
    * @param [out] do_state DO output state, do0~do1 corresponds to bit1~bit2, start from bit0. 
    * @return Error code 
    */ 
    int GetToolDO(ref byte do_state).

Get the DO output state of the machine controller
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the DO output state of the robot controller. 
    * @param [out] do_state_h DO output state, co0~co7 corresponds to bit0~bit7 
    * @param [out] do_state_l DO output state, do0~do7 corresponds to bit0~bit7
    * @return Error code 
    */ 
    int GetDO(ref int do_state_h, ref int do_state_l);

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnIOTest_Click(object sender, EventArgs e)
    {
        FRRobot robot; 
        robot.RPC("192.168.58.2"); 

        uint8_t status = 1;
        uint8_t smooth = 0;
        uint8_t block  = 0;
        uint8_t di = 0, tool_di = 0;
        float ai = 0.0, tool_ai = 0.0;
        float value = 0.0;
        int i;

        for(i = 0; i < 16; i++)
        {
            robot.SetDO(i, status, smooth, block);
            robot.WaitMs(1000);
        }

        status = 0;

        for(i = 0; i < 16; i++)
        {
            robot.SetDO(i, status, smooth, block);
            robot.WaitMs(1000);
        }

        status = 1;

        for(i = 0; i < 2; i++)
        {
            robot.SetToolDO(i, status, smooth, block);
            robot.WaitMs(1000);
        }

        status = 0;

        for(i = 0; i < 2; i++)
        {
            robot.SetToolDO(i, status, smooth, block);
            robot.WaitMs(1000);
        }

        value = 50.0;
        robot.SetAO(0, value, block);
        value = 100.0;
        robot.SetAO(1, value, block);
        robot.WaitMs(1000);
        value = 0.0;
        robot.SetAO(0, value, block);
        value = 0.0;
        robot.SetAO(1, value, block);

        value = 100.0;
        robot.SetToolAO(0, value, block);
        robot.WaitMs(1000);
        value = 0.0;
        robot.SetToolAO(0, value, block);

        robot.GetDI(0, block, &di);
        printf("di0:%u\n", di);
        robot.WaitDI(0,1,0,2);              // wait for
        robot.WaitMultiDI(1,3,3,10000,2);   // wait for
        tool_di = robot.GetToolDI(1, block, &tool_di);
        printf("tool_di1:%u\n", tool_di);
        robot.WaitToolDI(1,1,0,2);          // wait for

        robot.GetAI(0,block, &ai);
        printf("ai0:%f\n", ai);
        robot.WaitAI(0,0,50,0,2);           // wait for
        robot.WaitToolAI(0,0,50,0,2);       // wait for
        tool_ai = robot.GetToolAI(0,block, &tool_ai);
        printf("tool_ai0:%f\n", tool_ai);

        uint8_t _button_state = 0;
        robot.GetAxlePointRecordBtnState(&_button_state);
        printf("_button_state is: %u\n", _button_state);

        uint8_t tool_do_state = 0;
        robot.GetToolDO(&tool_do_state);
        printf("tool DO state is: %u\n", tool_do_state);

        uint8_t do_state_h = 0;
        uint8_t do_state_l = 0;
        robot.GetDO(&do_state_h, &do_state_l);
        printf("DO state high is: %u \n DO state low is: %u\n", do_state_h, do_state_l);
        return 0;
    }
    
Get the robot software version
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get robot software version information
    * @param [out] robotModel robotModel
    * @param [out] webVersion webVersion
    * @param [out] controllerVersion controllerVersion
    * @return Error code 
    */ 
    int GetSoftwareVersion(ref string robotModel, ref string webVersion, ref string controllerVersion);
    
Get the robot hardware version
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get robot hardware version information
    * @param [out] ctrlBoxBoardVersion ctrlBoxBoardHardwareVersion
    * @param [out] driver1Version driver1HardwareVersion
    * @param [out] driver1Version driver2HardwareVersion
    * @param [out] driver1Version driver3HardwareVersion
    * @param [out] driver1Version driver4HardwareVersion
    * @param [out] driver1Version driver5HardwareVersion
    * @param [out] driver1Version driver6HardwareVersion
    * @param [out] endBoardVersion endBoardHardwareVersion
    * @return Error Code 
    */ 
    int GetHardwareVersion(ref string ctrlBoxBoardVersion, ref string driver1Version, ref string driver2Version, ref string driver3Version,ref string driver4Version, ref string driver5Version, ref string driver6Version, ref string endBoardVersion);

Get the firmware version of the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the robot firmware version information.
    * @param [out] ctrlBoxBoardVersion ctrlBoxBoardFirmwareVersion
    * @param [out] driver1Version driver1FirmwareVersion
    * @param [out] driver1Version driver2FirmwareVersion
    * @param [out] driver1Version driver3FirmwareVersion
    * @param [out] driver1Version driver4FirmwareVersion
    * @param [out] driver1Version driver5FirmwareVersion
    * @param [out] driver1Version driver6FirmwareVersion
    * @param [out] endBoardVersion endBoardFirmwareVersion
    * @return Error Code 
    */ 
    int GetFirmwareVersion(ref string ctrlBoxBoardVersion, ref string driver1Version, ref string driver2Version, ref string driver3Version,ref string driver4Version, ref string driver5Version, ref string driver6Version, ref string endBoardVersion);

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnGetVersions_Click(object sender, EventArgs e)
    {
        string[] ver = new string[20];
        int rtn = 0;
        rtn = robot.GetSoftwareVersion(ref ver[0], ref ver[1], ref ver[2]);
        rtn = robot.GetHardwareVersion(ref ver[3], ref ver[4], ref ver[5], ref ver[6], ref ver[7], ref ver[8], ref ver[9], ref ver[10]);
        rtn = robot.GetFirmwareVersion(ref ver[11], ref ver[12], ref ver[13], ref ver[14], ref ver[15], ref ver[16], ref ver[17], ref ver[18]);
        Console.WriteLine($"robotmodel  is: {ver[0]}");
        Console.WriteLine($"webVersion  is: {ver[1]}");
        Console.WriteLine($"controllerVersion  is: {ver[2]}");
        Console.WriteLine($"Hard ctrlBox Version  is: {ver[3]}");
        Console.WriteLine($"Hard driver1 Version  is: {ver[4]}");
        Console.WriteLine($"Hard driver2 Version  is: {ver[5]}");
        Console.WriteLine($"Hard driver3 Version  is: {ver[6]}");
        Console.WriteLine($"Hard driver4 Version  is: {ver[7]}");
        Console.WriteLine($"Hard driver5 Version  is: {ver[8]}");
        Console.WriteLine($"Hard driver6 Version  is: {ver[9]}");
        Console.WriteLine($"Hard end Version  is: {ver[10]}");
        Console.WriteLine($"Firm ctrlBox Version  is: {ver[11]}");
        Console.WriteLine($"Firm driver1 Version  is: {ver[12]}");
        Console.WriteLine($"Firm driver2 Version  is: {ver[13]}");
        Console.WriteLine($"Firm driver3 Version  is: {ver[14]}");
        Console.WriteLine($"Firm driver4 Version  is: {ver[15]}");
        Console.WriteLine($"Firm driver5 Version  is: {ver[16]}");
        Console.WriteLine($"Firm driver6 Version  is: {ver[17]}");
        Console.WriteLine($"Firm end Version  is: {ver[18]}");
    }