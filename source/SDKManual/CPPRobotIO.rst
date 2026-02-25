IO
============

.. toctree::
    :maxdepth: 5

Set Control Box Digital Output
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Set control box digital output
    @param [in] id IO number, range [0~15]
    @param [in] status 0-off, 1-on
    @param [in] smooth 0-non-smooth, 1-smooth
    @param [in] block 0-blocking, 1-non-blocking
    @return Error code
    */
    errno_t SetDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

Set Tool Digital Output
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Set tool digital output
    @param [in] id IO number, range [0~1]
    @param [in] status 0-off, 1-on
    @param [in] smooth 0-non-smooth, 1-smooth
    @param [in] block 0-blocking, 1-non-blocking
    @return Error code
    */
    errno_t SetToolDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

Set Control Box Analog Output
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Set control box analog output
    @param [in] id IO number, range [0~1]
    @param [in] value Current or voltage percentage, range [0~100] corresponding to current [0~20mA] or voltage [0~10V]
    @param [in] block 0-blocking, 1-non-blocking
    @return Error code
    */
    errno_t SetAO(int id, float value, uint8_t block);

Set Tool Analog Output
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Set tool analog output
    @param [in] id IO number, range [0]
    @param [in] value Current or voltage percentage, range [0~100] corresponding to current [0~20mA] or voltage [0~10V]
    @param [in] block 0-blocking, 1-non-blocking
    @return Error code
    */
    errno_t SetToolAO(int id, float value, uint8_t block);

Code Example for Setting Digital and Analog Outputs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestAODO(void)
    {
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
    return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    uint8_t status = 1;
    uint8_t smooth = 0;
    uint8_t block = 0;
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
    robot.SetAO(0, i * 40.96, block);
    robot.Sleep(30);
    }
    for (int i = 0; i < 100; i++)
    {
    robot.SetToolAO(0, i * 40.96, block);
    robot.Sleep(30);
    }
    robot.CloseRPC();
    return 0;
    }

Get Control Box Digital Input
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Get control box digital input
    @param [in] id IO number, range [0~15]
    @param [in] block 0-blocking, 1-non-blocking
    @param [out] result 0-low level, 1-high level
    @return Error code
    */
    errno_t GetDI(int id, uint8_t block, uint8_t *result);

Get Tool Digital Input
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Get tool digital input
    @param [in] id IO number, range [0~1]
    @param [in] block 0-blocking, 1-non-blocking
    @param [out] result 0-low level, 1-high level
    @return Error code
    */
    errno_t GetToolDI(int id, uint8_t block, uint8_t *result);

Get Control Box Analog Input
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Get control box analog input
    @param [in] id IO number, range [0~1]
    @param [in] block 0-blocking, 1-non-blocking
    @param [out] result Input current or voltage percentage, range [0~100] corresponding to current [0~20mS] or voltage [0~10V]
    @return Error code
    */
    errno_t GetAI(int id, uint8_t block, float *result);

Get Tool Analog Input
+++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Get tool analog input
    @param [in] id IO number, range [0]
    @param [in] block 0-blocking, 1-non-blocking
    @param [out] result Input current or voltage percentage, range [0~100] corresponding to current [0~20mS] or voltage [0~10V]
    @return Error code
    */
    errno_t GetToolAI(int id, uint8_t block, float *result);

Get Robot End Point Record Button Status
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Get robot end point record button status
    @param [out] state Button status, 0-pressed, 1-released
    @return Error code
    */
    errno_t GetAxlePointRecordBtnState(uint8_t *state);

Get Robot End DO Output Status
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Get robot end DO output status
    @param [out] do_state DO output status, do0~do1 correspond to bit1~bit2, starting from bit0
    @return Error code
    */
    errno_t GetToolDO(uint8_t *do_state);

Get Robot Controller DO Output Status
++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Get robot controller DO output status
    @param [out] do_state_h DO output status, co0~co7 correspond to bit0~bit7
    @param [out] do_state_l DO output status, do0~do7 correspond to bit0~bit7
    @return Error code
    */
    errno_t GetDO(uint8_t *do_state_h, uint8_t *do_state_l);

Code Example for Getting Robot DI and DO Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestGetDIAI(void)
    {
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
    return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    uint8_t status = 1;
    uint8_t smooth = 0;
    uint8_t block = 0;
    uint8_t di = 0, tool_di = 0;
    float ai = 0.0, tool_ai = 0.0;
    float value = 0.0;
    robot.GetDI(0, block, &di);
    printf("di0:%u\n", di);
    tool_di = robot.GetToolDI(1, block, &tool_di);
    printf("tool_di1:%u\n", tool_di);
    robot.GetAI(0, block, &ai);
    printf("ai0:%f\n", ai);
    tool_ai = robot.GetToolAI(0, block, &tool_ai);
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
    robot.CloseRPC();
    return 0;
    }

Wait for Control Box Digital Input
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Wait for control box digital input
    @param [in] id IO number, range [0~15]
    @param [in] status 0-off, 1-on
    @param [in] max_time Maximum wait time, unit ms
    @param [in] opt Strategy after timeout, 0-stop program and prompt timeout, 1-ignore timeout and continue, 2-wait indefinitely
    @return Error code
    */
    errno_t WaitDI(int id, uint8_t status, int max_time, int opt);

Wait for Control Box Multi-Channel Digital Input
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Wait for control box multi-channel digital input
    @param [in] mode 0-multi AND, 1-multi OR
    @param [in] id IO number, bit0~bit7 correspond to DI0~DI7, bit8~bit15 correspond to CI0~CI7
    @param [in] status 0-off, 1-on
    @param [in] max_time Maximum wait time, unit ms
    @param [in] opt Strategy after timeout, 0-stop program and prompt timeout, 1-ignore timeout and continue, 2-wait indefinitely
    @return Error code
    */
    errno_t WaitMultiDI(int mode, int id, uint8_t status, int max_time, int opt);

Wait for Tool Digital Input
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Wait for tool digital input
    @param [in] id IO number, range [0~1]
    @param [in] status 0-off, 1-on
    @param [in] max_time Maximum wait time, unit ms
    @param [in] opt Strategy after timeout, 0-stop program and prompt timeout, 1-ignore timeout and continue, 2-wait indefinitely
    @return Error code
    */
    errno_t WaitToolDI(int id, uint8_t status, int max_time, int opt);

Wait for Control Box Analog Input
+++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Wait for control box analog input
    @param [in] id IO number, range [0~1]
    @param [in] sign 0-greater than, 1-less than
    @param [in] value Input current or voltage percentage, range [0~100] corresponding to current [0~20mS] or voltage [0~10V]
    @param [in] max_time Maximum wait time, unit ms
    @param [in] opt Strategy after timeout, 0-stop program and prompt timeout, 1-ignore timeout and continue, 2-wait indefinitely
    @return Error code
    */
    errno_t WaitAI(int id, int sign, float value, int max_time, int opt);

Wait for Tool Analog Input
++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    @brief Wait for tool analog input
    @param [in] id IO number, range [0]
    @param [in] sign 0-greater than, 1-less than
    @param [in] value Input current or voltage percentage, range [0~100] corresponding to current [0~20mS] or voltage [0~10V]
    @param [in] max_time Maximum wait time, unit ms
    @param [in] opt Strategy after timeout, 0-stop program and prompt timeout, 1-ignore timeout and continue, 2-wait indefinitely
    @return Error code
    */
    errno_t WaitToolAI(int id, int sign, float value, int max_time, int opt);

Code Example for Waiting for Control Box Digital and Analog Input Signals
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionchanged:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    int TestWaitDIAI(void)
    {
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
    return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    uint8_t status = 1;
    uint8_t smooth = 0;
    uint8_t block = 0;
    uint8_t di = 0, tool_di = 0;
    float ai = 0.0, tool_ai = 0.0;
    float value = 0.0;
    rtn = robot.WaitDI(0, 1, 1000, 1);
    cout << "WaitDI over; rtn is: " << rtn << endl;
    robot.WaitMultiDI(1, 3, 3, 1000, 1);
    cout << "WaitDI over; rtn is: " << rtn << endl;
    robot.WaitToolDI(1, 1, 1000, 1);
    cout << "WaitDI over; rtn is: " << rtn << endl;
    robot.WaitAI(0, 0, 50, 1000, 1);
    cout << "WaitDI over; rtn is: " << rtn << endl;
    robot.WaitToolAI(0, 0, 50, 1000, 1);
    cout << "WaitDI over; rtn is: " << rtn << endl;
    robot.CloseRPC();
    return 0;
    }

Set Whether to Reset Control Box DO Output After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set whether to reset control box DO output after stop/pause
    * @param [in] resetFlag 0-do not reset; 1-reset
    * @param [in] reloadFlag Whether to reload after pause recovery, 0-do not load; 1-load
    * @return Error code
    */
    errno_t SetOutputResetCtlBoxDO(int resetFlag, int reloadFlag = 0);

Set Whether to Reset Control Box AO Output After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set whether to reset control box AO output after stop/pause
    * @param [in] resetFlag  0-do not reset; 1-reset
    * @param [in] reloadFlag Whether to reload after pause recovery, 0-do not load; 1-load
    * @return Error code
    */
    errno_t SetOutputResetCtlBoxAO(int resetFlag, int reloadFlag = 0);

Set Whether to Reset End Tool DO Output After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set whether to reset end tool DO output after stop/pause
    * @param [in] resetFlag  0-do not reset; 1-reset
    * @param [in] reloadFlag Whether to reload after pause recovery, 0-do not load; 1-load
    * @return Error code
    */
    errno_t SetOutputResetAxleDO(int resetFlag, int reloadFlag = 0);

Set Whether to Reset End Tool AO Output After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set whether to reset end tool AO output after stop/pause
    * @param [in] resetFlag  0-do not reset; 1-reset
    * @param [in] reloadFlag Whether to reload after pause recovery, 0-do not load; 1-load
    * @return  Error code
    */
    errno_t SetOutputResetAxleAO(int resetFlag, int reloadFlag = 0);

Set Whether to Reset Extension DO Output After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set whether to reset extension DO output after stop/pause
    * @param [in] resetFlag  0-do not reset; 1-reset
    * @param [in] reloadFlag Whether to reload after pause recovery, 0-do not load; 1-load
    * @return  Error code
    */
    errno_t SetOutputResetExtDO(int resetFlag, int reloadFlag = 0);

Set Whether to Reset Extension AO Output After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set whether to reset extension AO output after stop/pause
    * @param [in] resetFlag  0-do not reset; 1-reset
    * @param [in] reloadFlag Whether to reload after pause recovery, 0-do not load; 1-load
    * @return  Error code
    */
    errno_t SetOutputResetExtAO(int resetFlag, int reloadFlag = 0);

Set Whether to Reset SmartTool Output After Stop/Pause
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set whether to reset SmartTool output after stop/pause
    * @param [in] resetFlag  0-do not reset; 1-reset
    * @param [in] reloadFlag Whether to reload after pause recovery, 0-do not load; 1-load
    * @return  Error code
    */
    errno_t SetOutputResetSmartToolDO(int resetFlag, int reloadFlag = 0);

Example Code for Setting LUA Program Output Reset After Stop/Pause
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int TestDOReset(void)
    {
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(3);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    for (int i = 0; i < 16; i++)
    {
        robot.SetDO(i, 1, 0, 0);
        robot.Sleep(200);
    }
    int resetFlag = 0;
    int resumeReloadFlag = 0;
    rtn = robot.SetOutputResetCtlBoxDO(resetFlag, resumeReloadFlag);
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
    return 0;
    }