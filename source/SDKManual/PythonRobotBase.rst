机器人基础
=============

.. toctree:: 
    :maxdepth: 5

实例化机器人
++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``RPC(ip)``"
    "描述", "实例化一个机器人对象"
    "参数", "- ``ip``：机器人的IP地址，默认出厂IP为“192.168.58.2”"
    "返回值", "- 成功：返回一个机器人对象
    - 失败：创建的对象会被销毁"
     
代码示例
--------

.. code-block:: python
    :linenos:
    :emphasize-lines: 3

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')

查询SDK版本号
++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetSDKVersion()``"
    "描述", "查询SDK版本号"
    "参数", "无"
    "返回值", "- 成功：[0,version]
    - 失败：[errcode,]"

代码示例
-----------

.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetSDKVersion()    # 查询SDK版本号
    if ret[0] == 0:  
        # 0-无故障，返回格式：[errcode,data],errcode-故障码，data-数据
        print("SDK version is:",ret[1])
    else:
        print("the errcode is: ", ret[0])

获取控制器IP
+++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetControllerIP()``"
    "描述", "查询控制器IP"
    "参数", "无"
    "返回值", "- 成功：[0,IP]
    - 失败：[errcode,]"

代码示例
----------

.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetControllerIP()    #查询控制器IP
    if ret[0] == 0:
        print("controller ip is:",ret[1])
    else:
        print("the errcode is: ", ret[0])


控制机器人手自动模式切换
++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``Mode(state)``"
    "描述", "控制机器人手自动模式切换"
    "参数", "- ``state``：1-手动模式，0-自动模式"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
----------

.. code-block:: python
    :linenos:
    :emphasize-lines: 5, 7

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.Mode(0)   #机器人切入自动运行模式
    time.sleep(1)  
    robot.Mode(1)   #机器人切入手动模式


机器人拖动模式
+++++++++++++++++

控制机器人进入或退出拖动示教模式
---------------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``DragTeachSwitch(state)``"
    "描述", "控制机器人进入或退出拖动示教模式"
    "参数", "- ``state``：1-进入拖动示教模式，0-退出拖动示教模式"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

查询机器人是否处于拖动模式
----------------------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``IsInDragTeach()``"
    "描述", "查询机器人是否处于拖动示教模式"
    "参数", "无"
    "返回值", "- 成功：[0,state]，state:0-非拖动示教模式，1-拖动示教模式
    - 失败：[errcode]"

代码示例
^^^^^^^^^^

.. code-block:: python
    :linenos:
    :emphasize-lines: 7, 9, 15, 17

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.Mode(1) #机器人切入手动模式
    time.sleep(1)
    robot.DragTeachSwitch(1)  #机器人切入拖动示教模式，必须在手动模式下才能切入拖动示教模式
    time.sleep(1)
    ret = robot.IsInDragTeach()    #查询是否处于拖动示教模式，1-拖动示教模式，0-非拖动示教模式
    if ret[0] == 0:
        print("drag state is:",ret[1])
    else:
        print("the errcode is: ", ret[0])
    time.sleep(3)
    robot.DragTeachSwitch(0)  #机器人切入非拖动示教模式，必须在手动模式下才能切入非拖动示教模式
    time.sleep(1)
    ret = robot.IsInDragTeach()    #查询是否处于拖动示教模式，1-拖动示教模式，0-非拖动示教模式
    if ret[0] == 0:
        print("drag state is:",ret[1])
    else:
        print("the errcode is: ", ret[0])


控制机器人上使能或下使能
++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``RobotEnable(state)``"
    "描述", "控制机器人上使能或下使能"
    "参数", "- ``state``：1-上使能，0-下使能"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
----------

.. code-block:: python
    :linenos:
    :emphasize-lines: 5, 7

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.RobotEnable(0)   #机器人下使能
    time.sleep(3)
    robot.RobotEnable(1)   #机器人上使能，机器人上电后默认自动上使能