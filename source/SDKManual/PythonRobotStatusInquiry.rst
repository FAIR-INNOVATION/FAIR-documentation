机器人状态查询
===============

.. toctree:: 
    :maxdepth: 5

获取机器人安装角度
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetRobotInstallAngle()``"
    "描述", "获取机器人安装角度"
    "参数", "无"
    "返回值", "- 成功：[0,yangle,zangle],yangle-倾斜角，zangle-旋转角
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetRobotInstallAngle()  # 获取机器人安装角度
    print(ret)

获取系统变量值
+++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetSysVarValue(id)``"
    "描述", "获取系统变量值"
    "参数", "- ``id``：系统变量编号，范围[1~20]"
    "返回值", "- 成功：[0,var_value]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 8

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    for i in range(1,21):
        robot.SetSysVarValue(i,i+0.5)    #  设置系统变量值
    robot.WaitMs(1000)
    for i in range(1,21):
        sys_var = robot.GetSysVarValue(i)  #  查询系统变量值
        print(sys_var)

获取当前关节位置(角度)
++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetActualJointPosDegree(flag)``"
    "描述", "获取关节当前位置(角度)"
    "参数", "- ``flag``：0-阻塞，1-非阻塞"
    "返回值", "- 成功：[0,joint_pos],joint_pos=[j1,j2,j3,j4,j5,j6]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetActualJointPosDegree(0)  # 获取机器人当前关节位置
    print(ret)

获取当前关节位置(弧度)
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetActualJointPosRadian(flag)``"
    "描述", "获取关节当前位置(弧度)"
    "参数", "- ``flag``：0-阻塞，1-非阻塞"
    "返回值", "- 成功：[0,joint_pos],joint_pos=[j1,j2,j3,j4,j5,j6]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetActualJointPosRadian(0)  # 获取机器人当前关节位置
    print(ret)

获取当前工具位姿
++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetActualTCPPose(flag)``"
    "描述", "获取当前工具位姿"
    "参数", "- ``flag``：0-阻塞，1-非阻塞"
    "返回值", "- 成功：[0,tcp_pose],tcp_pose=[x,y,z,rx,ry,rz]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetActualTCPPose(0)  # 获取机器人当前工具位姿
    print(ret)

获取当前工具坐标系编号
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetActualTCPNum(flag)``"
    "描述", "获取当前工具坐标系编号"
    "参数", "- ``flag``：0-阻塞，1-非阻塞"
    "返回值", "- 成功：[0,tool_id]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetActualTCPNum(0)  # 获取机器人当前工具坐标系编号
    print(ret)

获取当前工件坐标系编号
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetActualWObjNum(flag)``"
    "描述", "获取当前工件坐标系编号"
    "参数", "- ``flag``：0-阻塞，1-非阻塞"
    "返回值", "- 成功：[0,wobj_id]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetActualWObjNum(0)  # 获取机器人当前工件坐标系编号
    print(ret)

获取当前末端法兰位姿
++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetActualToolFlangePose(flag)``"
    "描述", "获取当前末端法兰位姿"
    "参数", "- ``flag``：0-阻塞，1-非阻塞"
    "返回值", "- 成功：[0,flange_pose],flange_pose=[x,y,z,rx,ry,rz]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetActualToolFlangePose(0)  # 获取机器人当前末端法兰位姿
    print(ret)

逆运动学求解
++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetInverseKin(type,desc_pos,config)``"
    "描述", "逆运动学，笛卡尔位姿求解关节位置 "
    "参数", "- ``type``:0-绝对位姿(基坐标系)，1-相对位姿（基坐标系），2-相对位姿（工具坐标系）
    - ``desc_pose``:[x,y,z,rx,ry,rz],工具位姿，单位[mm][°]
    - ``config``:关节配置，[-1]-参考当前关节位置求解，[0~7]-依据关节配置求解"
    "返回值", "- 成功：[0,joint_pos],joint_pos=[j1,j2,j3,j4,j5,j6]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    ret = robot.GetInverseKin(0,P1,-1)
    print(ret)

逆运动学求解-指定参考位置
++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetInverseKinRef(type,desc_pos,joint_pos_ref)``"
    "描述", "逆运动学，工具位姿求解关节位置，参考指定关节位置求解"
    "参数", "- ``type``:0-绝对位姿(基坐标系)，1-相对位姿（基坐标系），2-相对位姿（工具坐标系）
    - ``desc_pos``：[x,y,z,rx,ry,rz]工具位姿，单位[mm][°]
    - ``joint_pos_ref``：[j1,j2,j3,j4,j5,j6]，关节参考位置，单位[°]"
    "返回值", "- 成功：[0,joint_pos],joint_pos=[j1,j2,j3,j4,j5,j6]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    ret = robot.GetInverseKinRef(0,P1,J1)
    print(ret)

逆运动学求解-是否有解
++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetInverseKinHasSolution(type,desc_pos,joint_pos_ref)``"
    "描述", "逆运动学，工具位姿求解关节位置 是否有解"
    "参数", "- ``type``:0-绝对位姿(基坐标系)，1-相对位姿（基坐标系），2-相对位姿（工具坐标系）
    - ``desc_pos``：[x,y,z,rx,ry,rz]工具位姿，单位[mm][°]
    - ``joint_pos_ref``：[j1,j2,j3,j4,j5,j6]，关节参考位置，单位[°]"
    "返回值", "- 成功：[0,result],“True”-有解，“False”-无解
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    ret = robot.GetInverseKinHasSolution(0,P1,J1)
    print(ret)

正运动学求解
+++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetForwardKin(joint_pos)``"
    "描述", "正运动学，关节位置求解工具位姿"
    "参数", "- ``joint_pos``:[j1,j2,j3,j4,j5,j6]:关节位置，单位[°]"
    "返回值", "- 成功：[0,desc_pos],desc_pos=[x,y,z,rx,ry,rz]:工具位姿，单位[mm][°]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    ret = robot.GetForwardKin(J1)
    print(ret)

获取当前关节转矩
+++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetJointTorques(flag)``"
    "描述", "获取当前关节转矩"
    "参数", "- ``flag``：0-阻塞，1-非阻塞"
    "返回值", "- 成功：[0,torques],torques=[j1,j2,j3,j4,j5,j6]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetJointTorques(0)  # 获取机器人当前关节转矩
    print(ret)

获取当前负载的重量
+++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetTargetPayload(flag)``"
    "描述", "获取当前负载的质量"
    "参数", "- ``flag``：0-阻塞，1-非阻塞"
    "返回值", "- 成功：[0,weight],单位[kg]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetTargetPayload(0)  # 获取机器人当前负载重量
    print(ret)

获取当前负载的质心
+++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetTargetPayloadCog(flag)``"
    "描述", "获取当前负载的质心"
    "参数", "- ``flag``：0-阻塞，1-非阻塞"
    "返回值", "- 成功：[0,cog], cog=[x,y,z]:质心坐标，单位[mm]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetTargetPayloadCog(0)  # 获取机器人当前负载质心
    print(ret)

获取当前工具坐标系
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetTCPOffset(flag)``"
    "描述", "获取当前工具坐标系"
    "参数", "- ``flag``：0-阻塞，1-非阻塞"
    "返回值", "- 成功：[0,tcp_offset], tcp_offset=[x,y,z,rx,ry,rz]:相对位姿，单位[mm][°]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetTCPOffset(0)  # 获取机器人当前工具坐标系
    print(ret)

获取当前工件坐标系
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetWObjOffset(flag)``"
    "描述", "获取当前工件坐标系"
    "参数", "- ``flag``：0-阻塞，1-非阻塞"
    "返回值", "- 成功：[0,wobj_offset], wobj _offset=[x,y,z,rx,ry,rz]:相对位姿，单位[mm][°]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetWObjOffset(0)  # 获取机器人当前工件坐标系
    print(ret)

获取关节软限位角度
+++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetJointSoftLimitDeg(flag)``"
    "描述", "获取关节软限位角度"
    "参数", "- ``flag``：0-阻塞，1-非阻塞"
    "返回值", "- 成功：[0, j1min,j1max,j2min,j2max,j3min,j3max,j4min,j4max,j5min,j5max,j6min,j6max] :轴1~轴6关节负限位与正限位，单位[mm]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetJointSoftLimitDeg(0)  # 获取机器人关节软限位角度
    print(ret)

获取系统时间
++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetSystemClock()``"
    "描述", "获取系统时间"
    "参数", "无"
    "返回值", "- 成功：[0,t_ms]:单位[ms]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetSystemClock()  # 获取控制器系统时间
    print(ret)

获取机器人当前关节配置
++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetRobotCurJointsConfig()``"
    "描述", "获取机器人当前关节配置"
    "参数", "无"
    "返回值", "- 成功：[0,config]:范围[0~7]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetRobotCurJointsConfig()  # 获取机器人当前关节配置
    print(ret)

获取默认速度
+++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetDefaultTransVel()``"
    "描述", "获取默认速度"
    "参数", "无"
    "返回值", "- 成功：[0,vel]:单位[mm/s]
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetDefaultTransVel()  # 获取机器人当前速度
    print(ret)

查询机器人运动是否完成
++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``GetRobotMotionDone()``"
    "描述", "查询机器人运动是否完成"
    "参数", "无"
    "返回值", "- 成功：[0,state],state:0-未完成，1-完成
    - 失败：[errcode,]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    ret = robot.GetRobotMotionDone()    #查询机器人运动完成状态
    if ret[0] == 0:
        print(ret[1])
    else:
        print("the errcode is: ", ret[0])
