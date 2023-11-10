IO
============

.. toctree:: 
    :maxdepth: 5

Set the control box digital output
+++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set the control box digital output
    * @param  [in] id  I/O number and range[0~15]
    * @param  [in] status 0- off, 1- on
    * @param  [in] smooth 0- Not smooth, 1- smooth
    * @param  [in] block  0- blocking, 1- non-blocking
    * @return  Error code
    */
    int SetDO(int id, byte status, byte smooth, byte block); 

Set tool digital output
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set tool digital output
    * @param  [in] id  I/O number and range[0~1]
    * @param  [in] status 0- off, 1- on
    * @param  [in] smooth 0- not smooth, 1- smooth
    * @param  [in] block  0- blocking, 1- non-blocking
    * @return  Error code
    */
    int SetToolDO(int id, byte status, byte smooth, byte block);

Set control box analog output
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set control box analog output
    * @param  [in] id  I/O number and range[0~1]
    * @param  [in] value Percentage of current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    * @param  [in] block  0- blocking, 1- non-blocking
    * @return  Error code
    */
    int SetAO(int id, float value, byte block); 

Set tool analog output
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Set tool analog output
    * @param  [in] id  I/O number, range [0]
    * @param  [in] value Percentage of current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    * @param  [in] block  0- blocking, 1- non-blocking
    * @return  Error code
    */
    int SetToolAO(int id, float value, byte block); 

Get the control box digital input
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the control box digital input
    * @param  [in] id  I/O number range[0~15]
    * @param  [in] block  0- blocking, 1- non-blocking
    * @param  [out] result  0- low, 1- high
    * @return  Error code
    */   
    int GetDI(int id, byte block, ref byte level); 

Get tool numeric input
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get tool numeric input
    * @param  [in] id  I/O number, range[0~1]
    * @param  [in] block  0- blocking, 1- non-blocking
    * @param  [out] result  0- low, 1- high
    * @return  Error code
    */   
    int GetToolDI(int id, byte block, ref byte level); 

Wait for the control box digital input
++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Wait for the control box digital input
    * @param  [in] id  I/O number，range[0~15]
    * @param  [in]  status 0- off, 1- on
    * @param  [in]  max_time  Maximum waiting time, expressed in ms
    * @param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    * @return  Error code
    */
    int WaitDI(int id, byte status, int max_time, int opt); 

Wait for control box multiplex digital input
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Wait for control box multiplex digital input
    * @param  [in] mode 0- multiplexed and, 1- multiplexed or
    * @param  [in] id  I/O numbers. bit0 to bit7 corresponds to DI0 to DI7, and bit8 to bit15 corresponds to CI0 to CI7
    * @param  [in]  status 0- off, 1- on
    * @param  [in]  max_time  Maximum waiting time, expressed in ms
    * @param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    * @return  Error code
    */
    int WaitMultiDI(int mode, int id, byte status, int max_time, int opt); 

Wait for the tool number to enter
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Wait for the tool number to enter
    * @param  [in] id  I/O numbers，range[0~1]
    * @param  [in]  status 0- off, 1- on
    * @param  [in]  max_time  Maximum waiting time, expressed in ms
    * @param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    * @return  Error code
    */
    int WaitToolDI(int id, byte status, int max_time, int opt);

Get control box analog input
++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief  Get control box analog input
    * @param  [in] id  I/O numbers，range[0~1]
    * @param  [in] block  0- blocking, 1- non-blocking
    * @param  [out] result  Percentage of input current or voltage value, range [0-100] corresponding to current value [0-20ms] or voltage [0-10V]
    * @return  Error code
    */   
    int GetAI(int id, byte block, ref float persent);

Get the tool analog input
+++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the tool analog input
    * @param  [in] id  I/O numbers，range[0~1]
    * @param  [in] block  0- blocking, 1- non-blocking
    * @param  [out] result  Percentage of input current or voltage value, range [0-100] corresponding to current value [0-20ms] or voltage [0-10V]
    * @return  Error code
    */   
    int GetToolAI(int id, byte block, ref float persent);

Gets the status of the tool record button
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Gets the status of the tool record button
    * @param [out] state Button status, 0-press, 1-Release
    * @return Error code
    */ 
    int GetAxlePointRecordBtnState(ref byte state);

Wait for control box analog input
++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Wait for control box analog input
    * @param  [in] id  I/O numbers，range[0~1]
    * @param  [in]  sign 0-greater than，1-less than
    * @param  [in]  value Percentage of input current or voltage value, range [0-100] corresponding to current value [0-20ms] or voltage [0-10V]
    * @param  [in]  max_time Maximum waiting time, expressed in ms
    * @param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    * @return  Error code
    */
    int WaitAI(int id, int sign, float value, int max_time, int opt);  

Wait for tool analog input
+++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Wait for tool analog input
    * @param  [in] id  I/O numbers，range[0~1]
    * @param  [in]  sign 0-greater than，1-less than
    * @param  [in]  value Percentage of input current or voltage value, range [0-100] corresponding to current value [0-20ms] or voltage [0-10V]
    * @param  [in]  max_time  Maximum waiting time, expressed in ms
    * @param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    * @return  Error code
    */
    int WaitToolAI(int id, int sign, float value, int max_time, int opt); 

Get the tool digital output
+++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the tool digital output
    * @param [out] do_state Digital output status，bit1-Do0，bit2-Do1
    * @return Error code 
    */ 
    int GetToolDO(ref byte do_state);

Gets the control box digital output
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Gets the control box digital output 
    * @param [out] do_state_h DO status，co0~co7 for bit0~bit7 
    * @param [out] do_state_l DO status，do0~do7 for bit0~bit7
    * @return Error code
    */ 
    int GetDO(ref int do_state_h, ref int do_state_l); 

Code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnIOTest_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        byte status = 1;
        byte smooth = 0;
        byte block = 0;
        byte di = 0, tool_di = 0;
        float ai = 0.0f, tool_ai = 0.0f;
        float value = 0.0f;
        int i;

        for (i = 0; i < 16; i++)
        {
            robot.SetDO(i, status, smooth, block);
            robot.WaitMs(500);
        }

        status = 0;

        for (i = 0; i < 16; i++)
        {
            robot.SetDO(i, status, smooth, block);
            robot.WaitMs(500);
        }

        status = 1;

        for (i = 0; i < 2; i++)
        {
            robot.SetToolDO(i, status, smooth, block);
            robot.WaitMs(500);
        }
        status = 0;
        for (i = 0; i < 2; i++)
        {
            robot.SetToolDO(i, status, smooth, block);
            robot.WaitMs(500);
        }

        value = 50.0f;
        robot.SetAO(0, value, block);
        value = 100.0f;
        robot.SetAO(1, value, block);
        robot.WaitMs(300);
        value = 0.0f;
        robot.SetAO(0, value, block);
        value = 0.0f;
        robot.SetAO(1, value, block);

        value = 100.0f;
        robot.SetToolAO(0, value, block);
        robot.WaitMs(1000);
        value = 0.0f;
        robot.SetToolAO(0, value, block);

        robot.GetDI(0, block, ref di);
        Console.WriteLine($"di0 : {di}");
        robot.WaitDI(0, 1, 0, 2);       
        Console.WriteLine("wait di success");
        robot.WaitMultiDI(0, 3, 0, 10000, 2);   
        Console.WriteLine("wait multi di success");
        robot.GetToolDI(1, block, ref tool_di);
        Console.WriteLine($"tool_di1 : {tool_di}");
        robot.WaitToolDI(1, 0, 0, 2);          
        Console.WriteLine("wait tool di success");
        robot.GetAI(0, block, ref ai);
        Console.WriteLine($"ai0 : {ai}");
        robot.GetAI(1, block, ref ai);
        Console.WriteLine($"ai1 : {ai}");
        robot.WaitAI(0, 1, 50.0f, 0, 2);    
        Console.WriteLine("wait ai success");
        robot.WaitToolAI(0, 1, 50, 0, 2);   
        Console.WriteLine("wait tool ai success");
        robot.GetToolAI(0, block, ref tool_ai);
        Console.WriteLine($"tool_ai0 : {tool_ai}");
    }