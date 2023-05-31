机器人外设
============

.. toctree:: 
    :maxdepth: 5

获取夹爪配置
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetGripperConfig()``"
    "描述", "获取夹爪配置"
    "参数", "无"
    "返回值", "- 成功：[0, company,device,softversion,bus],company:夹爪厂商
    - 失败：[errcode]"

激活夹爪
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``ActGripper(index,action)``"
    "描述", "激活夹爪"
    "参数", "- ``index``:夹爪编号；
    - ``action``:0-复位，1-激活"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

控制夹爪
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``MoveGripper(index,pos,speed,force,maxtime,block)``"
    "描述", "控制夹爪"
    "参数", "- ``index``:夹爪编号；
    - ``pos``:位置百分比，范围[0~100]；
    - ``speed``:速度百分比，范围[0~100];
    - ``force``:力矩百分比，范围[0~100]；
    - ``maxtime``:最大等待时间，范围[0~30000]，单位[ms]；
    - ``block``:0-阻塞，1-非阻塞。"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

获取夹爪运动状态
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetGripperMotionDone()``"
    "描述", "获取夹爪运动状态"
    "参数", "无"
    "返回值", "- 成功：[0,status], status:0-运动未完成，1-运动完成
    - 失败：[errcode]"

配置夹爪
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SetGripperConfig(company,device,softversion,bus)``"
    "描述", "配置夹爪"
    "参数", "- ``company``：夹爪厂商，1-Robotiq，2-慧灵，3-天机，4-大寰，5-知行；
    - ``device``：设备号，Robotiq(0-2F-85系列)，慧灵(0-NK系列,1-Z-EFG-100)，天机(0-TEG-110)，大寰(0-PGI-140)，知行(0-CTPM2F20)
    - ``softversion``：软件版本号，暂不使用，默认为0；
    - ``bus``：设备挂载末端总线位置，暂不使用，默认为0；"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    robot.SetGripperConfig(4,0,0,1)  # 配置夹爪
    time.sleep(1)
    config = robot.GetGripperConfig()  # 获取夹爪配置
    print(config)
    robot.ActGripper(1,0)   # 夹爪复位
    time.sleep(1)
    robot.ActGripper(1,1)   # 夹爪激活
    time.sleep(2)
    robot.MoveGripper(1,100,48,46,30000,0)   # 夹爪运动
    time.sleep(3)
    robot.MoveGripper(1,0,50,0,30000,0)
    ret = robot.GetGripperMotionDone()    # 查询夹爪运动完成状态
    print(ret)
