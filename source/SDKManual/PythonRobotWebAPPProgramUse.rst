机器人WebAPP程序使用
======================

.. toctree:: 
    :maxdepth: 5

设置开机自动加载默认的作业程序
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``LoadDefaultProgConfig(flag,program_name)``"
    "描述", "设置开机自动加载默认的作业程序"
    "参数", "- ``flag``：1-开机自动加载默认程序，0-不自动加载默认程序
    - ``program_name``：作业程序名及路径，如“/fruser/movej.lua”，其中“/fruser/”为固定路径"
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
    robot.LoadDefaultProgConfig(1, "/fruser/splineptp.lua")  # 设置开机自动加载默认的作业程序

加载指定的作业程序
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``ProgramLoad(program_name)``"
    "描述", "加载指定的作业程序"
    "参数", "- ``program_name``：作业程序名及路径，如“/fruser/movej.lua”，其中“/fruser/”为固定路径"
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
    #机器人webapp程序使用接口
    robot.Mode(0)   #机器人切入自动运行模式
    robot.ProgramLoad('/fruser/testPTP.lua')   #加载要执行的机器人程序，testPTP.lua程序需要先在webapp上编写好

获取当前机器人作业程序的执行行号
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetCurrentLine()``"
    "描述", "获取当前机器人作业程序的执行行号"
    "参数", "无"
    "返回值", "- 成功：[0,line_num]
    - 失败：[errcode]"

运行当前加载的作业程序
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``ProgramRun()``"
    "描述", "运行当前加载的作业程序"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

暂停当前运行的作业程序
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``ProgramPause()``"
    "描述", "暂停当前运行的作业程序"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

恢复当前暂停的作业程序
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``ProgramResume()``"
    "描述", "恢复当前暂停的作业程序"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

终止当前运行的作业程序
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``ProgramStop()``"
    "描述", "终止当前运行的作业程序"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

获取机器人作业程序执行状态
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetProgramState()``"
    "描述", "获取机器人作业程序执行状态"
    "参数", "无"
    "返回值", "- 成功：[0,state],state:1-程序停止或无程序运行，2-程序运行中，3-程序暂停
    - 失败：[errcode]"

获取已加载的作业程序名
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetLoadedProgram()``"
    "描述", "获取已加载的作业程序名"
    "参数", "无"
    "返回值", "- 成功：[0,program_name]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6-8

    import frrpc
    import time
    import _thread
    def print_program_state(name,rb):
        while(1):
            pstate = robot.GetProgramState()    #查询程序运行状态,1-程序停止或无程序运行，2-程序运行中，3-程序暂停
            linenum = robot.GetCurrentLine()    #查询当前作业程序执行的行号
            name = robot.GetLoadedProgram()     #查询已加载的作业程序名
            print("the robot program state is:",pstate[1])
            print("the robot program line number is:",linenum[1])
            print("the robot program name is:",name[1])
            time.sleep(1)
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    #机器人webapp程序使用接口
    robot.Mode(0)   #机器人切入自动运行模式
    robot.ProgramLoad('/fruser/testPTP.lua')   #加载要执行的机器人程序，testPTP.lua程序需要先在webapp上编写好
    robot.ProgramRun()     #执行机器人程序
    _thread.start_new_thread(print_program_state,("print_state",robot))
    time.sleep(5)         #休息10s
    robot.ProgramPause()   #暂停正在执行的机器人程序
    time.sleep(5)
    robot.ProgramResume()  #恢复暂停执行的机器人程序
    time.sleep(5)
    robot.ProgramStop()    #停止正在执行的机器人程序
    time.sleep(2)