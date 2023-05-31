机器人常用设置
=================

.. toctree:: 
    :maxdepth: 5

设置全局速度
+++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetSpeed(vel)``"
    "描述", "设置全局速度"
    "参数", "- ``vel``:速度百分比，范围[0~100]"
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
    robot.SetSpeed(20)   # 设置全局速度，手动模式与自动模式独立设置


设置系统变量值
+++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetSysVarValue(id,value)``"
    "描述", "设置系统变量"
    "参数", "- ``id``：变量编号，范围[1~20];
    - ``value``：变量值"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5,8

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    for i in range(1,21):
        robot.SetSysVarValue(i,i+0.5)    #  设置系统变量值
    robot.WaitMs(1000)
    for i in range(1,21):
        sys_var = robot.GetSysVarValue(i)  #  查询系统变量值
        print(sys_var)

设置工具坐标系
+++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetToolCoord(id,t_coord,type,install)``"
    "描述", "设置工具坐标系"
    "参数", "- ``id``:坐标系编号，范围[0~14]；
    - ``t_coord``:工具中心点相对末端法兰中心位姿，单位[mm][°]；
    - ``type``:0-工具坐标系，1-传感器坐标系；
    - ``install``:安装位置，0-机器人末端，1-机器人外部"
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
    t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
    robot.SetToolCoord(10,t_coord,0,0)  #  设置工具坐标系

设置工具坐标系列表
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetToolList(id,t_coord ,type,install)``"
    "描述", "设置工具坐标系列表"
    "参数", "- ``id``:坐标系编号，范围[0~14]；
    - ``t_coord``:工具中心点相对末端法兰中心位姿，单位[mm][°]；
    - ``type``:0-工具坐标系，1-传感器坐标系；
    - ``install``:安装位置，0-机器人末端，1-机器人外部"
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
    t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
    robot.SetToolList(10,t_coord,0,0)  #  设置工具坐标系

设置外部工具坐标系
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetExToolCoord(id,etcp ,etool)``"
    "描述", "设置外部工具坐标系"
    "参数", "- ``id``:坐标系编号，范围[0~14]；
    - ``etcp``:外部工具坐标系，单位[mm][°]；
    - ``etool``:末端工具坐标系，单位[mm][°]；"
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
    etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
    etool = [21.0,22.0,23.0,24.0,25.0,26.0]
    robot.SetExToolCoord(10,etcp,etool)

设置外部工具坐标系列表
++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetExToolList(id,etcp ,etool)``"
    "描述", "设置外部工具坐标系列表"
    "参数", "- ``id``:坐标系编号，范围[0~14]；
    - ``etcp``:外部工具坐标系，单位[mm][°]；
    - ``etool``:末端工具坐标系，单位[mm][°]；"
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
    etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
    etool = [21.0,22.0,23.0,24.0,25.0,26.0]
    robot.SetExToolList(10,etcp,etool)

设置工件坐标系
++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetWObjCoord(id,w_coord)``"
    "描述", "设置工件坐标系"
    "参数", "- ``id``:坐标系编号，范围[0~14]；
    - ``w_coord``:坐标系相对位姿，单位[mm][°]；"
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
    w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
    robot.SetWObjCoord(11,w_coord)

设置工件坐标系列表
++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetWObjList(id,w_coord)``"
    "描述", "设置工件坐标系列表"
    "参数", "- ``id``:坐标系编号，范围[0~14]；
    - ``w_coord``:坐标系相对位姿，单位[mm][°]；"
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
    w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
    robot.SetWObjList(11,w_coord)

设置末端负载重量
++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetLoadWeight(weight)``"
    "描述", "设置末端负载重量"
    "参数", "- ``weight``:单位[kg]"
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
    robot.SetLoadWeight(3.0)   # 设置负载重量

设置机器人安装方式-固定安装
++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetRobotInstallPos(method)``"
    "描述", "设置机器人安装方式-固定安装"
    "参数", "- ``method``:0-平装，1-侧装，2-挂装"
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
    robot.SetRobotInstallPos(0)    #   设置机器人安装方式

设置机器人安装角度-自由安装
++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetRobotInstallAngle(yangle,zangle)``"
    "描述", "设置机器人安装角度-自由安装"
    "参数", "- ``yangle``：倾斜角
    - ``zangle``：旋转角"
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
    robot.SetRobotInstallAngle(0.0,0.0)    #   设置机器人安装角度

设置末端负载质心坐标
++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetLoadCoord(x,y,z)``"
    "描述", "设置末端负载质心坐标"
    "参数", "- ``x``,``y``,``z``: 质心坐标，单位[mm]"
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
    robot.SetLoadCoord(3.0,4.0,5.0)    #   设置负载质心坐标

等待指定时间
+++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``WaitMs(t_ms)``"
    "描述", "等待指定时间"
    "参数", "- ``t_ms``:单位[ms]"
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
    robot.WaitMs(1000)    #  等待1000ms
