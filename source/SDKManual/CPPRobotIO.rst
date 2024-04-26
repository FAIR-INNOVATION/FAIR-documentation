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
    * @param  [in] id  I/O number range[0~15]
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

Get the robot end point record button status
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Get the robot end point record button status
     * @param [out] state button state, 0-pressed, 1-released
     * @return Error code
     */
    errno_t  GetAxlePointRecordBtnState(uint8_t *state);

Get the DO output status at the end of the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get the DO output status at the end of the robot
     * @param [out] do_state DO output state, do0~do1 corresponds to bit1~bit2, starting from bit0
     * @return Error code
     */
    errno_t  GetToolDO(uint8_t *do_state);

Get the DO output status of the robot controller
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get the DO output status of the robot controller
     * @param [out] do_state_h DO output status, co0~co7 corresponds to bit0~bit7
     * @param [out] do_state_l DO output status, do0~do7 correspond to bit0~bit7
     * @return Error code
     */
    errno_t  GetDO(uint8_t *do_state_h, uint8_t *do_state_l);

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

.. versionchanged:: C++ SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    #include "libfairino/robot.h"

    //If using Windows, include the following header files
    #include <string.h>
    #include <windows.h>
    //If using Linux, include the following header files
    /*
    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    */
    #include <chrono>
    #include <thread>

    using namespace std;
    int main(void)
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
        robot.WaitDI(0,1,0,2);              // always waiting
        robot.WaitMultiDI(1,3,3,10000,2);   // always waiting
        tool_di = robot.GetToolDI(1, block, &tool_di);
        printf("tool_di1:%u\n", tool_di);
        robot.WaitToolDI(1,1,0,2);          // always waiting

        robot.GetAI(0,block, &ai);
        printf("ai0:%f\n", ai);
        robot.WaitAI(0,0,50,0,2);           // always waiting
        robot.WaitToolAI(0,0,50,0,2);       // always waiting
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

Get robot software version
+++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Get robot software version
	* @param[out]	robotModel Robot model
	* @param[out]	webversion web version
	* @param[out]	controllerVersion controller version
	* @return Error code 
	*/
	errno_t GetSoftwareVersion(char robotModel[64], char webVersion[64], char controllerVersion[64]);

Get the robot hardware version
++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
	* @brief Get the robot hardware version
	* @param[out] ctrlBoxBoardversion Control box carrier board hardware version
	* @param[out] driver1version Drive 1 Hardware Version
	* @param[out] driver2version Drive 2 Hardware Version
	* @param[out] driver3version Drive 3 Hardware Version
	* @param[out] driver4version Drive 4 Hardware Version
	* @param[out] driver5version Drive 5 Hardware Version
	* @param[out] driver6version Drive 6 Hardware Version
	* @param[out] endBoardversion End version hardware version
	* @return Error code 
	*/
	errno_t GetHardwareVersion(char ctrlBoxBoardversion[128], char driver1version[128], char driver2version[128], char driver3version[128], char driver4version[128], char driver5version[128], char driver6version[128], char endBoardversion[128]);

Get the robot firmware version
++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Get the robot firmware version
	* @param[out] ctrlBoxBoardversion Control box carrier board firmware version
	* @param[out] driver1version Drive 1 firmware version
	* @param[out] driver2version Drive 2 firmware version
	* @param[out] driver3version Drive 3 firmware version
	* @param[out] driver4version Drive 4 firmware version
	* @param[out] driver5version Drive 5 firmware version
	* @param[out] driver6version Drive 6 firmware version
	* @param[out] endBoardversion End version firmware version
	* @return Error code 
	*/
	errno_t GetFirmwareVersion(char ctrlBoxBoardversion[128], char driver1version[128], char driver2version[128], char driver3version[128], char driver4version[128], char driver5version[128], char driver6version[128], char endBoardversion[128]);

Code example
++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    #include "libfairino/robot.h"

    //If using Windows, include the following header files
    #include <string.h>
    #include <windows.h>
    //If using Linux, include the following header files
    /*
    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    */
    #include <chrono>
    #include <thread>

    using namespace std;
    int main(void)
    {
        FRRobot robot; 
        robot.RPC("192.168.58.2"); 

        int retval = 0;
        char robotModel[64] = {0};
        char webversion[64] = {0};
        char controllerVersion[64] = {0};

        char ctrlBoxBoardversion[128] = {0};
        char driver1version[128] = {0};
        char driver2version[128] = {0};
        char driver3version[128] = {0};
        char driver4version[128] = {0};
        char driver5version[128] = {0};
        char driver6version[128] = {0};
        char endBoardversion[128] = {0};

        retval = robot.GetSoftwareVersion(robotModel, webversion, controllerVersion);
        printf("Getsoftwareversion retval is: %d\n", retval);
        printf("robotmodel is: %s, webversion is: %s, controllerVersion is: %s \n", robotModel, webversion, controllerVersion);

        retval = robot.GetHardwareVersion(ctrlBoxBoardversion,  driver1version,  driver2version, driver3version,  driver4version,  driver5version, driver6version,  endBoardversion);
        printf("GetHardwareversion retval is: %d\n", retval);
        printf("GetHardwareversion get hardware versoin is: %s, %s, %s, %s, %s, %s, %s, %s\n", ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);

        retval = robot.GetFirmwareVersion(ctrlBoxBoardversion,  driver1version,  driver2version, driver3version,  driver4version,  driver5version, driver6version, endBoardversion);
        printf("GetFirmwareversion retval is: %d\n", retval);
        printf("GetHardwareversion get hardware versoin is: %s, %s, %s, %s, %s, %s, %s, %s\n", ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);

        return 0;
    }
