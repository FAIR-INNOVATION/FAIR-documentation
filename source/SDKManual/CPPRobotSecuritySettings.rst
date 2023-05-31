机器人安全设置
=================

.. toctree:: 
    :maxdepth: 5

设置碰撞等级
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief 设置碰撞等级
    * @param  [in]  mode  0-等级，1-百分比
    * @param  [in]  level 碰撞阈值，等级对应范围[],百分比对应范围[0~1]
    * @param  [in]  config 0-不更新配置文件，1-更新配置文件
    * @return  错误码
    */
    errno_t  SetAnticollision(int mode, float level[6], int config);

设置碰撞后策略
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  设置碰撞后策略
    * @param  [in] strategy  0-报错停止，1-继续运行
    * @return  错误码  
    */
    errno_t  SetCollisionStrategy(int strategy);

设置正限位
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  设置正限位
    * @param  [in] limit 六个关节位置，单位deg
    * @return  错误码
    */
    errno_t  SetLimitPositive(float limit[6]);

设置负限位
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  设置负限位
    * @param  [in] limit 六个关节位置，单位deg
    * @return  错误码
    */
    errno_t  SetLimitNegative(float limit[6]);   

错误状态清除
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  错误状态清除
    * @return  错误码
    */
    errno_t  ResetAllError();

关节摩擦力补偿开关
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  关节摩擦力补偿开关
    * @param  [in]  state  0-关，1-开
    * @return  错误码
    */
    errno_t  FrictionCompensationOnOff(uint8_t state);

设置关节摩擦力补偿系数-正装
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  设置关节摩擦力补偿系数-正装
    * @param  [in]  coeff 六个关节补偿系数，范围[0~1]
    * @return  错误码
    */
    errno_t  SetFrictionValue_level(float coeff[6]);

设置关节摩擦力补偿系数-侧装
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  设置关节摩擦力补偿系数-侧装
    * @param  [in]  coeff 六个关节补偿系数，范围[0~1]
    * @return  错误码
    */
    errno_t  SetFrictionValue_wall(float coeff[6]);

设置关节摩擦力补偿系数-倒装
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  设置关节摩擦力补偿系数-倒装
    * @param  [in]  coeff 六个关节补偿系数，范围[0~1]
    * @return  错误码
    */
    errno_t  SetFrictionValue_ceiling(float coeff[6]);

设置关节摩擦力补偿系数-自由安装
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  设置关节摩擦力补偿系数-自由安装
    * @param  [in]  coeff 六个关节补偿系数，范围[0~1]
    * @return  错误码
    */
    errno_t  SetFrictionValue_freedom(float coeff[6]);

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

        int mode = 0;
        int config = 1;
        float level1[6] = {1.0,2.0,3.0,4.0,5.0,6.0};
        float level2[6] = {50.0,20.0,30.0,40.0,50.0,60.0};

        robot.SetAnticollision(mode, level1, config);
        mode = 1;
        robot.SetAnticollision(mode, level2, config);
        robot.SetCollisionStrategy(1);

        float plimit[6] = {170.0,80.0,150.0,80.0,170.0,160.0};
        robot.SetLimitPositive(plimit);
        float nlimit[6] = {-170.0,-260.0,-150.0,-260.0,-170.0,-160.0};
        robot.SetLimitNegative(nlimit);

        robot.ResetAllError();

        float lcoeff[6] = {0.9,0.9,0.9,0.9,0.9,0.9};
        float wcoeff[6] = {0.4,0.4,0.4,0.4,0.4,0.4};
        float ccoeff[6] = {0.6,0.6,0.6,0.6,0.6,0.6};
        float fcoeff[6] = {0.5,0.5,0.5,0.5,0.5,0.5};
        robot.FrictionCompensationOnOff(1);
        robot.SetFrictionValue_level(lcoeff);
        robot.SetFrictionValue_wall(wcoeff);
        robot.SetFrictionValue_ceiling(ccoeff);
        robot.SetFrictionValue_freedom(fcoeff);

        return 0;
    }
