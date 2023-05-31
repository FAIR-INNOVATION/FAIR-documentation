机器人WebAPP程序使用
======================

.. toctree:: 
    :maxdepth: 5

设置开机自动加载默认的作业程序
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  设置开机自动加载默认的作业程序
    * @param  [in] flag  0-开机不自动加载默认程序，1-开机自动加载默认程序
    * @param  [in] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
    * @return  错误码
    */
    errno_t  LoadDefaultProgConfig(uint8_t flag, char program_name[64]);

加载指定的作业程序
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  加载指定的作业程序
    * @param  [in] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
    * @return  错误码
    */
    errno_t  ProgramLoad(char program_name[64]);

获取已加载的作业程序名
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取已加载的作业程序名
    * @param  [out] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
    * @return  错误码
    */
    errno_t  GetLoadedProgram(char program_name[64]);  

获取当前机器人作业程序的执行行号
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取当前机器人作业程序执行的行号
    * @param  [out] line  行号
    * @return  错误码
    */   
    errno_t  GetCurrentLine(int *line);

运行当前加载的作业程序
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  运行当前加载的作业程序
    * @return  错误码
    */
    errno_t  ProgramRun();

暂停当前运行的作业程序
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  暂停当前运行的作业程序
    * @return  错误码
    */ 
    errno_t  ProgramPause();

恢复当前暂停的作业程序
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  恢复当前暂停的作业程序
    * @return  错误码
    */ 
    errno_t  ProgramResume();  

终止当前运行的作业程序
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  终止当前运行的作业程序
    * @return  错误码
    */ 
    errno_t  ProgramStop();    

获取机器人作业程序执行状态
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取机器人作业程序执行状态
    * @param  [out]  state 1-程序停止或无程序运行，2-程序运行中，3-程序暂停
    * @return  错误码
    */
    errno_t  GetProgramState(uint8_t *state);

代码示例
++++++++++++
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

        char program_name[64] = "/fruser/ptps.lua";
        char loaded_name[64] = "";
        uint8_t state;
        int line;

        robot.Mode(0);
        robot.ProgramLoad(program_name);
        robot.ProgramRun();
        sleep(5);
        robot.ProgramPause();
        robot.GetProgramState(&state);
        printf("program state:%u\n", state);
        robot.GetCurrentLine(&line);
        printf("current line:%d\n", line);
        robot.GetLoadedProgram(loaded_name);
        printf("program name:%s\n", loaded_name);
        sleep(5);
        robot.ProgramResume();
        sleep(5);
        robot.ProgramStop();
        sleep(2);

        return 0;
    }
