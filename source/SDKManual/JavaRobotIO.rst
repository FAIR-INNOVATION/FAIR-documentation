Robot IO
=====================================

.. toctree:: 
    :maxdepth: 5

Setting the control box digital output
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting up control box digital outputs
    * @param [in] id io number, range [0~15]
    * @param [in] status 0-off, 1-on
    * @param [in] smooth 0-not smooth, 1-smooth
    * @param [in] block 0-blocking, 1-non-blocking
    * @return error code
    */
    int SetDO(int id, int status, int smooth, int block). 

Setting Tool Digital Outputs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief setup tool digital output
    * @param [in] id io number, range [0~1]
    * @param [in] status 0-off, 1-on
    * @param [in] smooth 0-not smooth, 1-smooth
    * @param [in] block 0-blocking, 1-non-blocking
    * @return error code
    */
    int SetToolDO(int id, int status, int smooth, int block); 

Setting the control box analog output
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting up control box analog outputs
    * @param [in] id id io number, range [0~1]
    * @param [in] id value Percentage of current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V].
    * @param [in] id block 0-blocking, 1-non-blocking
    * @return error code
    */
    int SetAO(int id, double value, int block); 

Setting Tool Analog Outputs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting up tool analog outputs
    * @param [in] id io number, range [0]
    * @param [in] value Percentage of current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V].
    * @param [in] block 0-blocking, 1-non-blocking
    * @return error code
    */
    int SetToolAO(int id, double value, int block); 

Waiting for control box digital inputs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief waiting for control box digital inputs
    * @param [in] id io number, range [0~15]
    * @param [in] status 0-off, 1-on
    * @param [in] max_time Maximum wait time in ms
    * @param [in] opt after timeout policy, 0-program stops and prompts for timeout, 1-ignores timeout prompts program continues, 2-always waits
    * @return error code
    */
    int WaitDI(int id, int status, int max_time, int opt); 

Waiting for control box with multiple digital inputs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Waiting for control box with multiple digital inputs
    * @param [in] mode 0-multiple with, 1-multiple or
    * @param [in] id io number, bit0~bit7 corresponds to DI0~DI7, bit8~bit15 corresponds to CI0~CI7
    * @param [in] status 0-off, 1-on
    * @param [in] max_time Maximum wait time in ms
    * @param [in] opt after timeout policy, 0-program stops and prompts for timeout, 1-ignores timeout prompts program continues execution, 2-always waits
    * @return error code
    */
    int WaitMultiDI(int mode, int id, int status, int max_time, int opt); 

Waiting for tool digital inputs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Waiting for tool digital inputs
    * @param [in] id io number, range [0~1]
    * @param [in] status 0-off, 1-on
    * @param [in] max_time Maximum wait time in ms
    * @param [in] opt after timeout policy, 0-program stops and prompts for timeout, 1-ignores timeout prompts program continues, 2-always waits
    * @return error code
    */
    int WaitToolDI(int id, int status, int max_time, int opt); 

Waiting for control box analog inputs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Waiting for control box analog inputs.
    * @param [in] id io number, range [0~1]
    * @param [in] sign 0-more than, 1-less than
    * @param [in] value Input current or voltage value percentage, range [0~100] corresponding to current value [0~20mS] or voltage [0~10V].
    * @param [in] max_time Maximum wait time in ms
    * @param [in] opt after timeout policy, 0-program stops and prompts for timeout, 1-ignores timeout prompts program continues execution, 2-always waits
    * @return error code
    */
    int WaitAI(int id, int sign, double value, int max_time, int opt);   

Waiting for tool analog inputs
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Waiting for tool analog input
    * @param [in] id io number, range [0]
    * @param [in] sign 0 - greater than, 1 - less than
    * @param [in] value Input current or voltage value percentage, range [0~100] corresponding to current value [0~20mS] or voltage [0~10V].
    * @param [in] max_time Maximum wait time in ms
    * @param [in] opt after timeout policy, 0-program stops and prompts for timeout, 1-ignores timeout prompts program continues, 2-always waits
    * @return error code
    */
    int WaitToolAI(int id, int sign, double value, int max_time, int opt); 

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.SetDO(8, 1, 0, 0);
        robot.Sleep(3000);
        robot.SetDO(8, 0, 0, 0);

        robot.SetToolDO(0, 1, 0, 0);
        robot.Sleep(3000);
        robot.SetToolDO(0, 0, 0, 0);

        for(int i = 0; i < 90; i++)
        {
            robot.SetAO(0, i+1, 0);
            robot.SetAO(1, i+1, 0);
            robot.Sleep(50);
        }
        robot.SetAO(0, 0.0, 0);
        robot.SetAO(1, 0.0, 0);

        for(int i = 0; i < 99; i++)
        {
            robot.SetToolAO(0, i+1, 0);
            robot.Sleep(50);
        }
        robot.SetToolAO(0, 0.0, 0);

        System.out.println("wait start ");
        robot.WaitDI(1, 1, 10000, 0);//WaitDI
        robot.WaitMultiDI(0, 6, 6, 10000, 0);//WaitMultiDI
        robot.WaitToolDI(0, 1, 5000, 0);//WaitToolDI
        robot.WaitAI(0, 0, 8.0, 5000, 0);//WaitAI
        robot.WaitToolAI(0, 0, 20, 5000, 0);//WaitToolAI
        System.out.println("wait end ");
    }

Setting whether the output is reset after the control box DO stop/pause
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Sets whether outputs are reset after control box DO stop/pause 
    * @param [in] resetFlag 0-no reset; 1-reset
    * @return error code 
    */ 
    int SetOutputResetCtlBoxDO(int resetFlag);

Setting whether the output is reset after the control box AO stop/pause
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Sets whether outputs are reset after control box AO stop/pause 
    * @param [in] resetFlag 0-no reset; 1-reset
    * @return error code 
    */ 
    int SetOutputResetCtlBoxAO(int resetFlag);

Sets whether the output is reset after the end tool DO stops/pause.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Sets whether output is reset after end tool DO stops/pause
    * @param [in] resetFlag 0-no reset; 1-reset
    * @return error code 
    */ 
    int SetOutputResetAxleDO(int resetFlag);

Set whether the output is reset after the end tool AO stops/pauses
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief sets whether the output is reset after the end tool AO stops/pauses. 
    * @param [in] resetFlag 0-no reset; 1-reset
    * @return error code 
    */ 
    int SetOutputResetAxleAO(int resetFlag);
    
Sets whether the outputs are reset after an extended DO stop/pause.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Sets whether output is reset after extended DO stop/pause
    * @param [in] resetFlag 0-no reset; 1-reset
    * @return error code 
    */ 
    int SetOutputResetExtDO(int resetFlag);
    
Sets whether the output is reset after the expansion AO stops/pause.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Sets whether output is reset after extended AO stop/pause
    * @param [in] resetFlag 0-no reset; 1-reset
    * @return error code 
    */ 
    int SetOutputResetExtAO(int resetFlag);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.SetOutputResetCtlBoxDO(1);
        robot.SetOutputResetAxleDO(1);//tools
        robot.SetOutputResetCtlBoxAO(1);
        robot.SetOutputResetAxleAO(1);//tools
    }