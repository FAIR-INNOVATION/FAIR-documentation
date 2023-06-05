IO
============

.. toctree:: 
    :maxdepth: 5

Set the control box digital output
+++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the control box digital output
    * @param  [in] id  I/O number and range[0~15]
    * @param  [in] status 0- off, 1- on
    * @param  [in] smooth 0- Not smooth, 1- smooth
    * @param  [in] block  0- blocking, 1- non-blocking
    * @return  Error code
    */
    errno_t  SetDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

Set tool digital output
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set tool digital output
    * @param  [in] id  I/O number and range[0~1]
    * @param  [in] status 0- off, 1- on
    * @param  [in] smooth 0- not smooth, 1- smooth
    * @param  [in] block  0- blocking, 1- non-blocking
    * @return  Error code
    */
    errno_t  SetToolDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

Set control box analog output
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set control box analog output
    * @param  [in] id  I/O number and range[0~1]
    * @param  [in] value Percentage of current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    * @param  [in] block  0- blocking, 1- non-blocking
    * @return  Error code
    */
    errno_t  SetAO(int id, float value, uint8_t block);

Set tool analog output
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set tool analog output
    * @param  [in] id  I/O number, range [0]
    * @param  [in] value Percentage of current or voltage value, range [0~100] corresponding to current value [0~20mA] or voltage [0~10V]
    * @param  [in] block  0- blocking, 1- non-blocking
    * @return  Error code
    */
    errno_t  SetToolAO(int id, float value, uint8_t block);

Get the control box digital input
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the control box digital input
    * @param  [in] id  I/O number (range: 0 to 15)
    * @param  [in] block  0- blocking, 1- non-blocking
    * @param  [out] result  0- low, 1- high
    * @return  Error code
    */   
    errno_t  GetDI(int id, uint8_t block, uint8_t *result);

Get tool numeric input
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get tool numeric input
    * @param  [in] id  I/O number, range[0~1]
    * @param  [in] block  0- blocking, 1- non-blocking
    * @param  [out] result  0- low, 1- high
    * @return  Error code
    */   
    errno_t  GetToolDI(int id, uint8_t block, uint8_t *result);

Wait for the control box digital input
++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Wait for the control box digital input
    * @param  [in] id  I/O number，range[0~15]
    * @param  [in]  status 0- off, 1- on
    * @param  [in]  max_time  Maximum waiting time, expressed in ms
    * @param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    * @return  Error code
    */
    errno_t  WaitDI(int id, uint8_t status, int max_time, int opt);

Wait for control box multiplex digital input
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
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
    errno_t  WaitMultiDI(int mode, int id, uint8_t status, int max_time, int opt);

Wait for the tool number to enter
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Wait for the tool number to enter
    * @param  [in] id  I/O numbers，range[0~1]
    * @param  [in]  status 0- off, 1- on
    * @param  [in]  max_time  Maximum waiting time, expressed in ms
    * @param  [in]  opt  After timeout policy, 0- program stops and prompts timeout, 1- ignores timeout prompts and continues execution, 2- waits
    * @return  Error code
    */
    errno_t  WaitToolDI(int id, uint8_t status, int max_time, int opt);

Get control box analog input
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /** 
    * @brief  Get control box analog input
    * @param  [in] id  I/O numbers，range[0~1]
    * @param  [in] block  0- blocking, 1- non-blocking
    * @param  [out] result  Percentage of input current or voltage value, range [0-100] corresponding to current value [0-20ms] or voltage [0-10V]
    * @return  Error code
    */   
    errno_t  GetAI(int id, uint8_t block, float *result); 

Get the tool analog input
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the tool analog input
    * @param  [in] id  I/O numbers，range[0~1]
    * @param  [in] block  0- blocking, 1- non-blocking
    * @param  [out] result  Percentage of input current or voltage value, range [0-100] corresponding to current value [0-20ms] or voltage [0-10V]
    * @return  Error code
    */   
    errno_t  GetToolAI(int id, uint8_t block, float *result);   

Wait for control box analog input
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
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
    errno_t  WaitAI(int id, int sign, float value, int max_time, int opt);  

Wait for tool analog input
+++++++++++++++++++++++++++++
.. code-block:: c++
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
    errno_t  WaitToolAI(int id, int sign, float value, int max_time, int opt); 

Code example
++++++++++++++
.. code-block:: c++
    :linenos:

    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    #include "FRRobot.h"
    #include "RobotTypes.h"

    using namespace std;

    int main(void)
    {
        FRRobot robot;                 //Instantiate the robot object
        robot.RPC("192.168.58.2");     //Establish a communication connection with the robot controller

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
        robot.WaitDI(0,1,0,2);              //Have been waiting
        robot.WaitMultiDI(1,3,3,10000,2);   //Have been waiting
        tool_di = robot.GetToolDI(1, block, &tool_di);
        printf("tool_di1:%u\n", tool_di);
        robot.WaitToolDI(1,1,0,2);          //Have been waiting

        robot.GetAI(0,block, &ai);
        printf("ai0:%f\n", ai);
        robot.WaitAI(0,0,50,0,2);           //Have been waiting
        robot.WaitToolAI(0,0,50,0,2);       //Have been waiting
        tool_ai = robot.GetToolAI(0,block, &tool_ai);
        printf("tool_ai0:%f\n", tool_ai);

        return 0;
    }