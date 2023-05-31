机器人基础
=============

.. toctree:: 
    :maxdepth: 5

实例化机器人
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  机器人接口类构造函数
    */
    FRRobot();

与控制器建立通信
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  与机器人控制器建立通信
    * @param  [in] ip  控制器IP地址，出场默认为192.168.58.2
    * @return 错误码
    */
    errno_t  RPC(const char *ip);

查询SDK版本号
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  查询SDK版本号
    * @param  [out] version   SDK版本号
    * @return  错误码
    */  
    errno_t  GetSDKVersion(char *version);

获取控制器IP
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取控制器IP
    * @param  [out] ip  控制器IP
    * @return  错误码
    */
    errno_t  GetControllerIP(char *ip);

控制机器人进入或退出拖动示教模式
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  控制机器人进入或退出拖动示教模式
    * @param  [in] state 0-退出拖动示教模式，1-进入拖动示教模式
    * @return  错误码
    */
    errno_t  DragTeachSwitch(uint8_t state);

查询机器人是否处于拖动模式
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  查询机器人是否处于拖动示教模式
    * @param  [out] state 0-非拖动示教模式，1-拖动示教模式
    * @return  错误码
    */
    errno_t  IsInDragTeach(uint8_t *state);

控制机器人上使能或下使能
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  控制机器人上使能或下使能，机器人上电后默认自动上使能
    * @param  [in] state  0-下使能，1-上使能
    * @return  错误码
    */
    errno_t  RobotEnable(uint8_t state);

控制机器人手自动模式切换
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief 控制机器人手自动模式切换
    * @param [in] mode 0-自动模式，1-手动模式
    * @return 错误码
    */
    errno_t  Mode(int mode);

代码示例
+++++++++++++
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

        char ip[64]="";
        char version[64] = "";
        uint8_t state;

        robot.GetSDKVersion(version);
        printf("SDK version:%s\n", version);
        robot.GetControllerIP(ip);
        printf("controller ip:%s\n", ip);

        robot.Mode(1);
        sleep(1);
        robot.DragTeachSwitch(1);
        robot.IsInDragTeach(&state);
        printf("drag state :%u\n", state);
        sleep(3);
        robot.DragTeachSwitch(0);
        sleep(1);
        robot.IsInDragTeach(&state);
        printf("drag state :%u\n", state);
        sleep(3);

        robot.RobotEnable(0);
        sleep(3);
        robot.RobotEnable(1);

        robot.Mode(0);
        sleep(1);
        robot.Mode(1);
        
        return 0;
    }
