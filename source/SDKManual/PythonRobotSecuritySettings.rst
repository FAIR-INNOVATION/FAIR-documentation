机器人安全设置
=================

.. toctree:: 
    :maxdepth: 5


设置碰撞等级
+++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetAnticollision (mode,level,config)``"
    "描述", "设置碰撞等级"
    "参数", "- ``mode``:0-等级，1-百分比；
    - ``level=[j1,j2,j3,j4,j5,j6]``:碰撞阈值；
    - ``config``:0-不更新配置文件，1-更新配置文件"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5,7

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    level = [1.0,2.0,3.0,4.0,5.0,6.0]
    robot.SetAnticollision(0,level,1)     #  设置碰撞等级
    level = [50.0,20.0,30.0,40.0,50.0,60.0]
    robot.SetAnticollision(1,level,1)     #  设置碰撞百分比

设置碰撞后策略
++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetCollisionStrategy (strategy)``"
    "描述", "设置碰撞后策略"
    "参数", "- ``strategy``：0-报错暂停，1-继续运行"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.SetCollisionStrategy(1)    # 设置碰撞后策略，1-继续运行

设置正限位
+++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetLimitPositive(p_limit)``"
    "描述", "设置正限位"
    "参数", "- ``p_limit=[j1,j2,j3,j4,j5,j6]``：六个关节位置"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    p_limit = [170.0,80.0,150.0,80.0,170.0,160.0]
    robot.SetLimitPositive(p_limit)   #  设置正限位

设置负限位
+++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetLimitNegative(n_limit)``"
    "描述", "设置负限位"
    "参数", "- ``n_limit=[j1,j2,j3,j4,j5,j6]``：六个关节位置"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    n_limit = [-170.0,-260.0,-150.0,-260.0,-170.0,-160.0]
    robot.SetLimitNegative(n_limit)   # 设置负限位

错误状态清除
++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``ResetAllError()``"
    "描述", "错误状态清除，只能清除可复位的错误"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.ResetAllError()    #  错误状态清除

关节摩擦力补偿开关
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``FrictionCompensationOnOff(state)``"
    "描述", "关节摩擦力补偿开关"
    "参数", "- ``state``：0-关，1-开"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.FrictionCompensationOnOff(1)   #  关节摩擦力补偿开

设置关节摩擦力补偿系数-正装
++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetFrictionValue_level(coeff)``"
    "描述", "设置关节摩擦力补偿系数-固定安装-正装"
    "参数", "- ``coeff=[j1,j2,j3,j4,j5,j6]``：六个关节补偿系数"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.FrictionCompensationOnOff(1)   #  关节摩擦力补偿开
    lcoeff = [0.9,0.9,0.9,0.9,0.9,0.9]
    robot.SetFrictionValue_level(lcoeff)   #  设置关节摩擦力补偿系数

设置关节摩擦力补偿系数-侧装
++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetFrictionValue_wall(coeff)``"
    "描述", "设置关节摩擦力补偿系数-固定安装-侧装"
    "参数", "- ``coeff=[j1,j2,j3,j4,j5,j6]``：六个关节补偿系数"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.FrictionCompensationOnOff(1)   #  关节摩擦力补偿开
    wcoeff = [0.4,0.4,0.4,0.4,0.4,0.4]
    robot.SetFrictionValue_wall(wcoeff)  #  设置关节摩擦力补偿系数

设置关节摩擦力补偿系数-倒装
++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetFrictionValue_ceiling(coeff)``"
    "描述", "设置关节摩擦力补偿系数-固定安装-倒装"
    "参数", "- ``coeff=[j1,j2,j3,j4,j5,j6]``：六个关节补偿系数"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.FrictionCompensationOnOff(1)   #  关节摩擦力补偿开
    ccoeff = [0.6,0.6,0.6,0.6,0.6,0.6]
    robot.SetFrictionValue_ceiling(ccoeff)  #  设置关节摩擦力补偿系数

设置关节摩擦力补偿系数-自由安装
+++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetFrictionValue_freedom(coeff)``"
    "描述", "设置关节摩擦力补偿系数-自由安装"
    "参数", "- ``coeff=[j1,j2,j3,j4,j5,j6]``：六个关节补偿系数"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.FrictionCompensationOnOff(1)   #  关节摩擦力补偿开
    fcoeff = [0.5,0.5,0.5,0.5,0.5,0.5]
    robot.SetFrictionValue_freedom(fcoeff)   #  设置关节摩擦力补偿系数
