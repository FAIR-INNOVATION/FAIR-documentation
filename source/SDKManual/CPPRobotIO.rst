机器人IO
============

.. toctree:: 
    :maxdepth: 5

设置控制箱数字量输出
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  设置控制箱数字量输出
    * @param  [in] id  io编号，范围[0~15]
    * @param  [in] status 0-关，1-开
    * @param  [in] smooth 0-不平滑， 1-平滑
    * @param  [in] block  0-阻塞，1-非阻塞
    * @return  错误码
    */
    errno_t  SetDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

设置工具数字量输出
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  设置工具数字量输出
    * @param  [in] id  io编号，范围[0~1]
    * @param  [in] status 0-关，1-开
    * @param  [in] smooth 0-不平滑， 1-平滑
    * @param  [in] block  0-阻塞，1-非阻塞
    * @return  错误码
    */
    errno_t  SetToolDO(int id, uint8_t status, uint8_t smooth, uint8_t block);

设置控制箱模拟量输出
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  设置控制箱模拟量输出
    * @param  [in] id  io编号，范围[0~1]
    * @param  [in] value 电流或电压值百分比，范围[0~100]对应电流值[0~20mA]或电压[0~10V]
    * @param  [in] block  0-阻塞，1-非阻塞
    * @return  错误码
    */
    errno_t  SetAO(int id, float value, uint8_t block);

设置工具模拟量输出
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  设置工具模拟量输出
    * @param  [in] id  io编号，范围[0]
    * @param  [in] value 电流或电压值百分比，范围[0~100]对应电流值[0~20mA]或电压[0~10V]
    * @param  [in] block  0-阻塞，1-非阻塞
    * @return  错误码
    */
    errno_t  SetToolAO(int id, float value, uint8_t block);

获取控制箱数字量输入
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取控制箱数字量输入
    * @param  [in] id  io编号，范围[0~15]
    * @param  [in] block  0-阻塞，1-非阻塞
    * @param  [out] result  0-低电平，1-高电平
    * @return  错误码
    */   
    errno_t  GetDI(int id, uint8_t block, uint8_t *result);

获取工具数字量输入
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取工具数字量输入
    * @param  [in] id  io编号，范围[0~1]
    * @param  [in] block  0-阻塞，1-非阻塞
    * @param  [out] result  0-低电平，1-高电平
    * @return  错误码
    */   
    errno_t  GetToolDI(int id, uint8_t block, uint8_t *result);

等待控制箱数字量输入
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief 等待控制箱数字量输入
    * @param  [in] id  io编号，范围[0~15]
    * @param  [in]  status 0-关，1-开
    * @param  [in]  max_time  最大等待时间，单位ms
    * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    * @return  错误码
    */
    errno_t  WaitDI(int id, uint8_t status, int max_time, int opt);

等待控制箱多路数字量输入
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief 等待控制箱多路数字量输入
    * @param  [in] mode 0-多路与，1-多路或
    * @param  [in] id  io编号，bit0~bit7对应DI0~DI7，bit8~bit15对应CI0~CI7
    * @param  [in]  status 0-关，1-开
    * @param  [in]  max_time  最大等待时间，单位ms
    * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    * @return  错误码
    */
    errno_t  WaitMultiDI(int mode, int id, uint8_t status, int max_time, int opt);

等待工具数字量输入
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief 等待工具数字量输入
    * @param  [in] id  io编号，范围[0~1]
    * @param  [in]  status 0-关，1-开
    * @param  [in]  max_time  最大等待时间，单位ms
    * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    * @return  错误码
    */
    errno_t  WaitToolDI(int id, uint8_t status, int max_time, int opt);

获取控制箱模拟量输入
+++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取控制箱模拟量输入
    * @param  [in] id  io编号，范围[0~1]
    * @param  [in] block  0-阻塞，1-非阻塞
    * @param  [out] result  输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
    * @return  错误码
    */   
    errno_t  GetAI(int id, uint8_t block, float *result); 

获取工具模拟量输入
+++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取工具模拟量输入
    * @param  [in] id  io编号，范围[0]
    * @param  [in] block  0-阻塞，1-非阻塞
    * @param  [out] result  输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
    * @return  错误码
    */   
    errno_t  GetToolAI(int id, uint8_t block, float *result);   

等待控制箱模拟量输入
+++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief 等待控制箱模拟量输入
    * @param  [in] id  io编号，范围[0~1]
    * @param  [in]  sign 0-大于，1-小于
    * @param  [in]  value 输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
    * @param  [in]  max_time  最大等待时间，单位ms
    * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    * @return  错误码
    */
    errno_t  WaitAI(int id, int sign, float value, int max_time, int opt);  

等待工具模拟量输入
+++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief 等待工具模拟量输入
    * @param  [in] id  io编号，范围[0]
    * @param  [in]  sign 0-大于，1-小于
    * @param  [in]  value 输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
    * @param  [in]  max_time  最大等待时间，单位ms
    * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    * @return  错误码
    */
    errno_t  WaitToolAI(int id, int sign, float value, int max_time, int opt); 

代码示例
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
        FRRobot robot;                 //实例化机器人对象
        robot.RPC("192.168.58.2");     //与机器人控制器建立通信连接

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
        robot.WaitDI(0,1,0,2);              //一直等待
        robot.WaitMultiDI(1,3,3,10000,2);   //一直等待
        tool_di = robot.GetToolDI(1, block, &tool_di);
        printf("tool_di1:%u\n", tool_di);
        robot.WaitToolDI(1,1,0,2);          //一直等待

        robot.GetAI(0,block, &ai);
        printf("ai0:%f\n", ai);
        robot.WaitAI(0,0,50,0,2);           //一直等待
        robot.WaitToolAI(0,0,50,0,2);       //一直等待
        tool_ai = robot.GetToolAI(0,block, &tool_ai);
        printf("tool_ai0:%f\n", tool_ai);

        return 0;
    }