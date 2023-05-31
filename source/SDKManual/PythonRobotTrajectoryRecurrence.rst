机器人轨迹复现
=================

.. toctree:: 
    :maxdepth: 5

设置轨迹记录参数
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetTPDParam(type,name,period_ms,di_choose,do_choose)``"
    "描述", "设置轨迹记录参数"
    "参数", "- ``type``：数据类型，1-关节位置；
    - ``name``：轨迹名；
    - ``period_ms``：采样周期，固定值，2ms或4ms或8ms；
    - ``di_choose``：DI选择,bit0~bit7对应控制箱DI0~DI7，bit8~bit9对应末端DI0~DI1，0-不选择，1-选择
    - ``do_choose``：DO选择,bit0~bit7对应控制箱DO0~DO7，bit8~bit9对应末端DO0~DO1，0-不选择，1-选择"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 9

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    type = 1  # 数据类型，1-关节位置
    name = 'tpd2023'  # 轨迹名
    period = 4  #采样周期，2ms或4ms或8ms
    di_choose = 0 # di输入配置
    do_choose = 0 # do输出配置
    robot.SetTPDParam(type, name, period, di_choose, do_choose)    #配置TPD参数

开始轨迹记录
++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetTPDStart(type,name,period_ms,di_choose,do_choose)``"
    "描述", "开始轨迹记录"
    "参数", "- ``type``：数据类型，1-关节位置；
    - ``name``：轨迹名；
    - ``period_ms``：采样周期，固定值，2ms或4ms或8ms；
    - ``di_choose``：DI选择,bit0~bit7对应控制箱DI0~DI7，bit8~bit9对应末端DI0~DI1，0-不选择，1-选择
    - ``do_choose``：DO选择,bit0~bit7对应控制箱DO0~DO7，bit8~bit9对应末端DO0~DO1，0-不选择，1-选择"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

停止轨迹记录
++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetWebTPDStop()``"
    "描述", "停止轨迹记录"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 14, 16

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    type = 1  # 数据类型，1-关节位置
    name = 'tpd2023'  # 轨迹名
    period = 4  #采样周期，2ms或4ms或8ms
    di_choose = 0 # di输入配置
    do_choose = 0 # do输出配置
    robot.SetTPDParam(type, name, period, di_choose, do_choose)    #配置TPD参数
    robot.Mode(1)  # 机器人切入手动模式
    time.sleep(1)  
    robot.DragTeachSwitch(1)  #机器人切入拖动示教模式
    robot.SetTPDStart(type, name, period, di_choose, do_choose)   # 开始记录示教轨迹
    time.sleep(30)
    robot.SetWebTPDStop()  # 停止记录示教轨迹
    robot.DragTeachSwitch(0)  #机器人切入非拖动示教模式

删除轨迹记录
+++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetTPDDelete(name)``"
    "描述", "删除轨迹记录"
    "参数", "- ``name``:轨迹名"
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
    robot.SetTPDDelete('tpd2023')   # 删除TPD轨迹

轨迹预加载
+++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``LoadTPD(name)``"
    "描述", "轨迹预加载"
    "参数", "- ``name``:轨迹名"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

轨迹复现
++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``MoveTPD(name,blend,ovl)``"
    "描述", "轨迹复现"
    "参数", "- ``name``:轨迹名
    - ``blend``：是否平滑，0-不平滑，1-平滑
    - ``ovl``：速度缩放因子，范围[0~100]"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 8, 10

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    P1=[-378.9,-340.3,107.2,179.4,-1.3,125.0]
    name = 'tpd2023'   #轨迹名
    blend = 1   #是否平滑，1-平滑，0-不平滑
    ovl = 100.0   #速度缩放
    robot.LoadTPD(name)  #轨迹预加载
    robot.MoveCart(P1,1,0,100.0,100.0,100.0,-1.0,-1)       #运动到起始点
    robot.MoveTPD(name, blend, ovl)  #轨迹复现