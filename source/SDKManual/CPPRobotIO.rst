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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
    robot.ProgramLoad("test.lua");
    robot.ProgramRun();
    robot.Sleep(2000);
    robot.PauseMotion();
    robot.Sleep(2000);
    robot.ResumeMotion();
    robot.Sleep(2000);
    robot.CloseRPC();
    return 0;
    }

Set Configurable CI Port Functions of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
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
    36-Enter safety speed motion;37-Current loop drag lock;38-Force sensor assisted lock
    * @return Error code
    */
    errno_t SetDIConfig(int config[8]);

Get Configurable CI Port Functions of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
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
    36-Enter safety speed motion;37-Current loop drag lock;38-Force sensor assisted lock
    201-External emergency stop input signal 1-dual channel; 202-External emergency stop input signal 2-dual channel; 203-Level 1 reduced mode-dual channel;
    204-Level 2 reduced mode-dual channel; 205-Level 3 reduced mode-dual channel; 206-Normal stop-dual channel; 207-Safety wall 1-dual channel; 208-Safety wall 2-dual channel;
    209-Safety wall 3-dual channel; 210-Safety wall 4-dual channel; 211-Safety wall 5-dual channel; 212-Safety wall 6-dual channel; 213-Safety wall 7-dual channel;
    214-Safety wall 8-dual channel; 215-Safety stop reset-dual channel;
    * @return Error code
    */
    errno_t GetDIConfig(int config[8]);

Set Configurable CO Port Functions of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
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
    errno_t SetDOConfig(int config[8]);

Get Configurable CO Port Functions of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
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
    201-Emergency stop output signal 1-dual channel; 202-Emergency stop output signal 2-dual channel; 203-Safety status output-dual channel; 204-Protective stop status output-dual channel; 205-Robot in motion-dual channel;
	206-Robot reduced mode-dual channel; 207-Robot non-reduced mode-dual channel;
    * @return Error code
    */
    errno_t GetDOConfig(int config[8]);

Set Configurable End-CI Port Functions of the End-Effector
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
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
    errno_t SetToolDIConfig(int config[2]);

Get Configurable End-CI Port Functions of the End-Effector
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
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
    errno_t GetToolDIConfig(int config[2]);
    
Set Configurable CI Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set configurable CI active state of the control box
    * @param [in] config CI0-CI7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    errno_t SetDIConfigLevel(int config[8]);
        
Get Configurable CI Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get configurable CI active state of the control box
    * @param [out] config CI0-CI7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    errno_t GetDIConfigLevel(int config[8]);
        
Set Configurable CO Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set configurable CO active state of the control box
    * @param [in] config CO0-CO7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    errno_t SetDOConfigLevel(int config[8]);

Get Configurable CO Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get configurable CO active state of the control box
    * @param [out] config CO0-CO7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    errno_t GetDOConfigLevel(int config[8]);
    
Set Configurable CI Active State of the End-Effector
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set configurable CI active state of the end-effector
    * @param [in] config CI0-CI1 port active state; 0-active high; 1-active low
    * @return Error code
    */
    errno_t SetToolDIConfigLevel(int config[2]);
    
Get Configurable CI Active State of the End-Effector
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get configurable CI active state of the end-effector
    * @param [out] config CI0-CI1 port active state; 0-active high; 1-active low
    * @return Error code
    */
    errno_t GetToolDIConfigLevel(int config[2]);
    
Set Standard DI Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set standard DI active state of the control box
    * @param [in] config DI0-DI7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    errno_t SetStandardDILevel(int config[8]);
    
Get Standard DI Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get standard DI active state of the control box
    * @param [out] config DI0-DI7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    errno_t GetStandardDILevel(int config[8]);

Set Standard DO Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set standard DO active state of the control box
    * @param [in] config DO0-DO7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    errno_t SetStandardDOLevel(int config[8]);
    
Get Standard DO Active State of the Control Box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get standard DO active state of the control box
    * @param [out] config DO0-DO7 port active state; 0-active high; 1-active low
    * @return Error code
    */
    errno_t GetStandardDOLevel(int config[8]);
        
Robot IO Configuration Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c++
    :linenos:

    int TestIOConfig()
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
        int setDIConfig[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
        int getDIConfig[8] = { 0 };
        rtn = robot.SetDIConfig(setDIConfig);
        printf("SetDIConfig rtn is %d\n", rtn);
        rtn = robot.GetDIConfig(getDIConfig);
        printf("GetDIConfig rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn, 
            getDIConfig[0], getDIConfig[1], getDIConfig[2], getDIConfig[3], getDIConfig[4], getDIConfig[5], getDIConfig[6], getDIConfig[7]);
        int setDOConfig[8] = { 9, 10, 11, 12, 13, 14, 15, 16 };
        int getDOConfig[8] = { 0 };
        rtn = robot.SetDOConfig(setDOConfig);
        printf("SetDOConfig rtn is %d\n", rtn);
        rtn = robot.GetDOConfig(getDOConfig);
        printf("GetDOConfig rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
            getDOConfig[0], getDOConfig[1], getDOConfig[2], getDOConfig[3], getDOConfig[4], getDOConfig[5], getDOConfig[6], getDOConfig[7]);
        int setToolDIConfig[2] = { 17, 18 };
        int getToolDIConfig[2] = { 0 };
        rtn = robot.SetToolDIConfig(setToolDIConfig);
        printf("SetToolDIConfig rtn is %d\n", rtn);
        rtn = robot.GetToolDIConfig(getToolDIConfig);
        printf("GetToolDIConfig rtn is %d, value is %d %d \n", rtn, getToolDIConfig[0], getToolDIConfig[1]);
        int setDIConfigLevel[8] = { 1, 1, 1, 1, 0, 0, 0, 0 };
        int getDIConfigLevel[8] = { 0 };
        rtn = robot.SetDIConfigLevel(setDIConfigLevel);
        printf("SetDIConfigLevel rtn is %d\n", rtn);
        rtn = robot.GetDIConfigLevel(getDIConfigLevel);
        printf("GetDIConfigLevel rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
            getDIConfigLevel[0], getDIConfigLevel[1], getDIConfigLevel[2], getDIConfigLevel[3], getDIConfigLevel[4], getDIConfigLevel[5], getDIConfigLevel[6], getDIConfigLevel[7]);
        int setDOConfigLevel[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
        int getDOConfigLevel[8] = { 0 };
        rtn = robot.SetDOConfigLevel(setDOConfigLevel);
        printf("SetDOConfigLevel rtn is %d\n", rtn);
        rtn = robot.GetDOConfigLevel(getDOConfigLevel);
        printf("GetDOConfigLevel rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
            getDOConfigLevel[0], getDOConfigLevel[1], getDOConfigLevel[2], getDOConfigLevel[3], getDOConfigLevel[4], getDOConfigLevel[5], getDOConfigLevel[6], getDOConfigLevel[7]);
        int setToolDIConfigLevel[2] = { 1, 0 };
        int getToolDIConfigLevel[2] = { 0 };
        rtn = robot.SetToolDIConfigLevel(setToolDIConfigLevel);
        printf("SetToolDIConfigLevel rtn is %d\n", rtn);
        rtn = robot.GetToolDIConfigLevel(getToolDIConfigLevel);
        printf("GetToolDIConfigLevel rtn is %d, value is %d %d \n", rtn, getToolDIConfigLevel[0], getToolDIConfigLevel[1]);
        int setStandardDILevel[8] = { 1, 1, 1, 1, 0, 0, 0, 0 };
        int getStandardDILevel[8] = { 0 };
        rtn = robot.SetStandardDILevel(setStandardDILevel);
        printf("SetStandardDILevel rtn is %d\n", rtn);
        rtn = robot.GetStandardDILevel(getStandardDILevel);
        printf("GetStandardDILevel rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
            getStandardDILevel[0], getStandardDILevel[1], getStandardDILevel[2], getStandardDILevel[3], getStandardDILevel[4], getStandardDILevel[5], getStandardDILevel[6], getStandardDILevel[7]);
        int setStandardDOLevel[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
        int getStandardDOLevel[8] = { 0 };
        rtn = robot.SetStandardDOLevel(setStandardDOLevel);
        printf("SetStandardDOLevel rtn is %d\n", rtn);
        rtn = robot.GetStandardDOLevel(getStandardDOLevel);
        printf("GetStandsrdDOLevel rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
            getStandardDOLevel[0], getStandardDOLevel[1], getStandardDOLevel[2], getStandardDOLevel[3], getStandardDOLevel[4], getStandardDOLevel[5], getStandardDOLevel[6], getStandardDOLevel[7]);
        robot.Sleep(2000);
        robot.CloseRPC();
        robot.Sleep(1000);
        return 0;
    }