机器人IO
============

.. toctree:: 
    :maxdepth: 5

设置控制箱数字量输出
++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetDO(id,status,smooth,block)``"
    "描述", "设置控制箱数字量输出"
    "参数", "-  ``id``:io编号，范围[0~15]；
    - ``status``:0-关，1-开；
    - ``smooth``:0-不平滑，1-平滑；
    - ``block``:0-阻塞，1-非阻塞。"
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
    for i in range(0,16):
        robot.SetDO(i,1,0,0)   #打开控制箱DO
    robot.WaitMs(1000)
    for i in range(0,16):
        robot.SetDO(i,0,0,0)   #关闭控制箱DO
    robot.WaitMs(1000)


设置工具数字量输出
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetToolDO(id,status,smooth,block)``"
    "描述", "设置工具数字量输出"
    "参数", "-  ``id``:io编号，范围[0~1]；
    - ``status``:0-关，1-开；
    - ``smooth``:0-不平滑，1-平滑；
    - ``block``:0-阻塞，1-非阻塞。"
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
    for i in range(0,2):
        robot.SetToolDO(i,1,0,0)    #打开工具DO
    robot.WaitMs(1000)
    for i in range(0,2):
        robot.SetToolDO(i,0,0,0)    #关闭工具DO


设置控制箱模拟量输出
++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetAO(id,value,block)``"
    "描述", "设置控制箱模拟量输出"
    "参数", "- ``id``:io编号，范围[0~1]；
    - ``value``:电流或电压值百分比，范围[0~100%]对应电流值[0~20mA]或电压[0~10V]；
    - ``block``:[0]-阻塞，[1]-非阻塞。"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4,6

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.SetAO(0,0.0,0)    # 设置控制箱模拟量输出
    robot.WaitMs(1000)
    robot.SetAO(1,100.0,0)

设置工具模拟量输出
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetToolAO(id,value,block)``"
    "描述", "设置工具模拟量输出"
    "参数", "- ``id``:io编号，范围[0]；
    - ``value``:电流或电压值百分比，范围[0~100%]对应电流值[0~20mA]或电压[0~10V]；
    - ``block``:[0]-阻塞，[1]-非阻塞。"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4,6

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.SetToolAO(0,100.0,0)   # 设置工具模拟量输出
    robot.WaitMs(1000)
    robot.SetToolAO(0,0.0,0)

获取控制箱数字量输入
+++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetDI(id,block)``"
    "描述", "获取控制箱数字量输入"
    "参数", "- ``id``:io编号，范围[0~15]；
    - ``block``:[0]-阻塞，[1]-非阻塞。"
    "返回值", "- 成功：[0,di],di: 0-低电平，1-高电平
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    di = robot.GetDI(0,0)   # 获取控制箱数字量输入
    print(di)

获取工具数字量输入
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetToolDI(id,block)``"
    "描述", "获取工具数字量输入"
    "参数", "- ``id``:io编号，范围[0~1]；
    - ``block``:[0]-阻塞，[1]-非阻塞。"
    "返回值", "- 成功：[0,di],di: 0-低电平，1-高电平
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    tool_di = robot.GetToolDI(1,0)   # 获取工具数字量输入
    print(tool_di)

等待控制箱数字量输入
+++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``WaitDI(id,status,maxtime,opt)``"
    "描述", "等待控制箱数字量输入"
    "参数", "- ``id``:io编号，范围[0~15]；
    - ``status``:0-关，1-开；
    - ``maxtime``:最大等待时间，单位[ms]；
    - ``opt``:超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待"
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
    robot.WaitDI(0,1,0,2)    # 一直等待控制箱数字量输入


等待控制箱多路数字量输入
++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``WaitMultiDI(mode,id,status,maxtime,opt)``"
    "描述", "等待控制箱多路数字量输入"
    "参数", "- ``mode``:[0]-多路与，[1]-多路或；
    - ``id``:io编号，bit0~bit7对应DI0~DI7，bit8~bit15对应CI0~CI7；
    - ``status(uint16_t)``:bit0~bit7对应DI0~DI7状态，bit8~bit15对应CI0~CI7状态位的状态[0]-关，[1]-开；
    - ``maxtime``:最大等待时间，单位[ms]；
    - ``opt``:超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待。"
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
    robot.WaitMultiDI(1,3,3,10000,2)   #  一直等待控制箱多路数字量输入

等待工具数字量输入
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``WaitToolDI(id,status,maxtime,opt)``"
    "描述", "等待末端数字量输入"
    "参数", "- ``id``:io编号，范围[0~1]；
    - ``status``:0-关，1-开；
    - ``maxtime``:最大等待时间，单位[ms]；
    - ``opt``:超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待"
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
    robot.WaitToolDI(1,1,0,2)    #  一直等待工具数字量输入

获取控制箱模拟量输入
++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetAI(id,block)``"
    "描述", "获取控制箱模拟量输入"
    "参数", "- ``id``:io编号，范围[0~1]；
    - ``block``:0-阻塞，1-非阻塞。"
    "返回值", "- 成功：[0,value], value:输入电流或电压值百分比，范围[0~100]对应电流值[0~20mA]或电压[0~10V]；
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ai = robot.GetAI(0,1)   #  获取控制箱模拟量输入
    print(ai)

获取工具模拟量输入
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetToolAI(id,block)``"
    "描述", "获取末端模拟量输入"
    "参数", "- ``id``:io编号，范围[0]；
    - ``block``:0-阻塞，1-非阻塞。"
    "返回值", "- 成功：[0,value], value:输入电流或电压值百分比，范围[0~100]对应电流值[0~20mA]或电压[0~10V]；
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    tool_ai = robot.GetToolAI(0,1)    #   获取工具模拟量输入
    print(tool_ai)

等待控制箱模拟量输入
++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``WaitAI(id,sign,value,maxtime,opt)``"
    "描述", "等待控制箱模拟量输入"
    "参数", "- ``id``:io编号，范围[0~1]；
    - ``sign``:0-大于，1-小于
    - ``value``:输入电流或电压值百分比，范围[0~100]对应电流值[0~20mA]或电压[0~10V]；
    - ``maxtime``:最大等待时间，单位[ms]；
    - ``opt``:超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待"
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
    robot.WaitAI(0,0,50,0,2)   #  一直等待控制箱模拟量输入

等待工具模拟量输入
++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``WaitToolAI(id,sign,value,maxtime,opt)``"
    "描述", "等待末端模拟量输入"
    "参数", "- ``id``:io编号，范围[0]；
    - ``sign``:0-大于，1-小于
    - ``value``:输入电流或电压值百分比，范围[0~100]对应电流值[0~20mA]或电压[0~10V]；
    - ``maxtime``:最大等待时间，单位[ms]；
    - ``opt``:超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待"
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
    robot.WaitToolAI(0,0,50,0,2)   #  一直等待工具模拟量输入
