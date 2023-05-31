机器人力控
============

.. toctree:: 
    :maxdepth: 5

获取力传感器配置
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_GetConfig()"
    "描述", "获取力传感器配置"
    "参数", "无"
    "返回值", "- 成功：[0, company,device,softversion,bus],company:传感器厂商
    - 失败：[errcode]"

力传感器配置
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_SetConfig(company,device,softversion,bus)"
    "描述", "力传感器配置"
    "参数", "- ``company``：传感器厂商，17-坤维科技，19-航天十一院，20-ATI传感器，21-中科米点，22-伟航敏芯；
    - ``device``：设备号，坤维(0-KWR75B)，航天十一院(0-MCS6A-200-4)，ATI(0-AXIA80-M8)，中科米点(0-MST2010)，伟航敏芯(0-WHC6L-YB-10A)；
    - ``softversion``：软件版本号，暂不使用，默认为0；
    - ``bus``：设备挂载末端总线位置，暂不使用，默认为0；"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 8, 9

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    company = 17    #传感器厂商，17-坤维科技
    device = 0      #传感器设备号
    softversion = 0 #软件版本号
    bus = 1         #末端总线位置
    robot.FT_SetConfig(company, device, softversion, bus)   #配置力传感器
    config = robot.FT_GetConfig() #获取力传感器配置信息，厂商编号下发比反馈大1
    print(config)

力传感器激活
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_Activate(state)"
    "描述", "力传感器激活"
    "参数", "- ``state``：0-复位，1-激活"
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
    robot.FT_Activate(0)  #传感器复位
    time.sleep(1)
    robot.FT_Activate(1)  #传感器激活
    time.sleep(1)

力传感器校零
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_SetZero(state)"
    "描述", "力传感器校零"
    "参数", "- ``state``：0-去除零点，1-零点矫正"
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
    robot.FT_SetZero(0)   #传感器去除零点
    time.sleep(1)
    robot.FT_SetZero(1)   #传感器零点矫正,注意此时末端不能安装工具，只有力传感器
    time.sleep(1)

设置力传感器参考坐标系
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_SetRCS(ref)"
    "描述", "设置力传感器参考坐标系"
    "参数", "- ``ref``：0-工具坐标系，1-基坐标系"
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
    robot.FT_SetRCS(0)    #设置参考坐标系为工具坐标系，0-工具坐标系，1-基坐标系
    time.sleep(1)

负载重量辨识计算
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_PdIdenCompute()"
    "描述", "负载重量辨识计算"
    "参数", "无"
    "返回值", "- 成功：[0,weight] ,weight-负载重量，单位kg
    - 失败：[errcode]"

负载重量辨识记录
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_PdIdenRecord(tool_id)"
    "描述", "负载重量辨识记录"
    "参数", "- ``tool_id``：传感器坐标系编号，范围[0~14]"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 13, 15

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    #负载辨识，此时末端安装要辨识的工具，工具安装在力传感器下方,末端竖直向下
    robot.FT_SetRCS(0)    #设置参考坐标系为工具坐标系，0-工具坐标系，1-基坐标系
    time.sleep(1)
    tool_id = 10  #传感器坐标系编号
    tool_coord = [0.0,0.0,35.0,0.0,0.0,0.0]   # 传感器相对末端法兰位姿
    tool_type = 1  # 0-工具，1-传感器
    tool_install = 0 # 0-安装末端，1-机器人外部
    robot.SetToolCoord(tool_id,tool_coord,tool_type,tool_install)     #设置传感器坐标系，传感器相对末端法兰位姿
    time.sleep(1)
    robot.FT_PdIdenRecord(tool_id)   #记录辨识数据
    time.sleep(1)
    weight = robot.FT_PdIdenCompute()  #计算负载重量，单位kg
    print(weight)


负载质心辨识计算
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_PdCogIdenCompute ()"
    "描述", "负载质心辨识计算"
    "参数", "无"
    "返回值", "- 成功：[0,cog],cog=[cogx,cogy,cogz] ，负载质心，单位mm
    - 失败：[errcode]"

负载质心辨识记录
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_PdCogIdenRecord(tool_id)"
    "描述", "负载质心辨识记录"
    "参数", "- ``tool_id``：传感器坐标系编号，范围[0~14]"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 9,14,19,21
    
    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    #负载质心辨识，机器人需要示教三个不同的姿态，然后记录辨识数据，最后计算负载质心
    P1=[-160.619,-586.138,384.988,-170.166,-44.782,169.295]
    robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)         #关节空间点到点运动
    time.sleep(1)
    robot.FT_PdCogIdenRecord(tool_id,1)                               #记录辨识数据
    time.sleep(1)
    P2=[-87.615,-606.209,556.119,-102.495,10.118,178.985]
    robot.MoveCart(P2,9,0,100.0,100.0,100.0,-1.0,-1)
    time.sleep(1)      
    robot.FT_PdCogIdenRecord(tool_id,2)
    time.sleep(1)
    P3=[41.479,-557.243,484.407,-125.174,46.995,-132.165]
    robot.MoveCart(P3,9,0,100.0,100.0,100.0,-1.0,-1)
    time.sleep(1)
    robot.FT_PdCogIdenRecord(tool_id,3)
    time.sleep(1)
    cog = robot.FT_PdCogIdenCompute()   # 计算辨识的负载质心
    print(cog)

获取参考坐标系下力/扭矩数据
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_GetForceTorqueRCS()"
    "描述", "获取参考坐标系下力/扭矩数据"
    "参数", "无"
    "返回值", "- 成功：[0,data] ,data=[fx,fy,fz,mx,my,mz]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    rcs = robot.FT_GetForceTorqueRCS()  #查询传感器坐标系下数据
    print(rcs)

获取力传感器原始力/扭矩数据
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_GetForceTorqueOrigin()"
    "描述", "获取力传感器原始力/扭矩数据"
    "参数", "无"
    "返回值", "- 成功：[0,data] ,data=[fx,fy,fz,mx,my,mz]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    origin = robot.FT_GetForceTorqueOrigin()   #查询传感器原始数据
    print(origin)

碰撞守护
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_Guard(flag,sensor_num,select,force_torque,max_threshold,min_threshold)"
    "描述", "碰撞守护"
    "参数", "- ``flag``：0-关闭碰撞守护，1-开启碰撞守护；
    - ``sensor_num``：力传感器编号；
    - ``select``：六个自由度是否检测碰撞[fx,fy,fz,mx,my,mz]，0-不生效，1-生效；
    - ``force_torque``：碰撞检测力/力矩，单位N或Nm；
    - ``max_threshold``：最大阈值；
    - ``min_threshold``：最小阈值；
    - 力/力矩检测范围:(force_torque-min_threshold,force_torque+max_threshold)"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    actFlag = 1   #开启标志，0-关闭碰撞守护，1-开启碰撞守护
    sensor_num = 1  #力传感器编号
    is_select = [1,1,1,1,1,1]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    force_torque = [0.0,0.0,0.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
    max_threshold = [10.0,10.0,10.0,10.0,10.0,10.0]  #最大阈值
    min_threshold = [5.0,5.0,5.0,5.0,5.0,5.0]   #最小阈值
    P1=[-160.619,-586.138,384.988,-170.166,-44.782,169.295]
    P2=[-87.615,-606.209,556.119,-102.495,10.118,178.985]
    P3=[41.479,-557.243,484.407,-125.174,46.995,-132.165]
    robot.FT_Guard(actFlag, sensor_num, is_select, force_torque, max_threshold, min_threshold)    #开启碰撞守护
    robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)         #关节空间点到点运动
    robot.MoveCart(P2,9,0,100.0,100.0,100.0,-1.0,-1)
    robot.MoveCart(P3,9,0,100.0,100.0,100.0,-1.0,-1)
    actFlag = 0  
    robot.FT_Guard(actFlag, sensor_num, is_select, force_torque, max_threshold, min_threshold)    #关闭碰撞守护

恒力控制
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_Control(flag,sensor_num,select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)"
    "描述", "恒力控制"
    "参数", "- ``flag``：恒力控制开启标志，0-关，1-开；
    - ``sensor_num``：力传感器编号；
    - ``select``：六个自由度是否检测 [fx,fy,fz,mx,my,mz]，0-不生效，1-生效；
    - ``force_torque``：检测力/力矩，单位N或Nm；
    - ``gain``：[f_p,f_i,f_d,m_p,m_i,m_d],力PID参数，力矩PID参数；
    - ``adj_sign``：自适应启停状态，0-关闭，1-开启；
    - ``ILC_sign``: ILC控制启停状态，0-停止，1-训练，2-实操；
    - ``max_dis``：最大调整距离；
    - ``max_ang``：最大调整角度；"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    status = 1  #恒力控制开启标志，0-关，1-开
    sensor_num = 1 #力传感器编号
    is_select = [0,0,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0005,0.0,0.0,0.0,0.0,0.0]  #最大阈值
    adj_sign = 0  #自适应启停状态，0-关闭，1-开启
    ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
    max_dis = 100.0  #最大调整距离
    max_ang = 0.0  #最大调整角度
    J1=[-68.987,-96.414,-111.45,-61.105,92.884,11.089]
    P1=[62.795,-511.979,291.697,-179.545,3.027,-170.039]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[0.000,0.000,0.000,0.000,0.000,0.000]
    J2=[-107.596,-109.154,-104.735,-56.176,90.739,11.091]
    P2=[-294.768,-503.708,233.158,179.799,0.713,151.309]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[0.000,0.000,0.000,0.000,0.000,0.000]
    robot.MoveJ(J1,P1,9,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)    #关节空间运动PTP,工具号9，实际测试根据现场数据及工具号使用
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)   #恒力控制
    robot.MoveL(J2,P2,9,0,100.0,180.0,20.0,-1.0,eP2,0,0,dP2)   #笛卡尔空间直线运动
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)

螺旋线探索
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_SpiralSearch(rcs,dr,fFinsih,t,vmax)"
    "描述", "螺旋线探索"
    "参数", "- ``rcs``：参考坐标系，0-工具坐标系，1-基坐标系
    - ``dr``：每圈半径进给量，单位mm；
    - ``fFinish``：力或力矩阈值(0~100)，单位N或Nm；
    - ``t``：最大探索时间，单位ms；
    - ``vmax``：线速度最大值，单位mm/s"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    #恒力参数
    status = 1  #恒力控制开启标志，0-关，1-开
    sensor_num = 1 #力传感器编号
    is_select = [0,0,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #最大阈值
    adj_sign = 0  #自适应启停状态，0-关闭，1-开启
    ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
    max_dis = 100.0  #最大调整距离
    max_ang = 5.0  #最大调整角度
    #螺旋线探索参数
    rcs = 0  #参考坐标系，0-工具坐标系，1-基坐标系
    dr = 0.7  #每圈半径进给量，单位mm
    fFinish = 1.0 #力或力矩阈值（0~100），单位N或Nm
    t = 60000.0 #最大探索时间，单位ms
    vmax = 3.0 #线速度最大值，单位mm/s
    is_select = [0,0,1,1,1,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    robot.FT_SpiralSearch(rcs,dr,fFinish,t,vmax)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)

旋转插入
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_RotInsertion(rcs,angVelRot,forceInsertion,angleMax,orn,angAccmax,rotorn)"
    "描述", "旋转插入"
    "参数", "- ``rcs``：参考坐标系，0-工具坐标系，1-基坐标系；
    - ``angVelRot``：旋转角速度，单位°/s；
    - ``forceInsertion``：力或力矩阈值(0~100)，单位N或Nm；
    - ``angleMax``：最大旋转角度，单位°；
    - ``orn``：力的方向，1-fz,2-mz；
    - ``angAccmax``：最大旋转加速度，单位°/s^2，暂不使用
    - ``rotorn``：旋转方向，1-顺时针，2-逆时针"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    #恒力参数
    status = 1  #恒力控制开启标志，0-关，1-开
    sensor_num = 1 #力传感器编号
    is_select = [0,0,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #最大阈值
    adj_sign = 0  #自适应启停状态，0-关闭，1-开启
    ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
    max_dis = 100.0  #最大调整距离
    max_ang = 5.0  #最大调整角度
    #旋转插入参数
    rcs = 0  #参考坐标系，0-工具坐标系，1-基坐标系
    angVelRot = 2.0  #旋转角速度，单位°/s
    forceInsertion = 1.0 #力或力矩阈值（0~100），单位N或Nm
    angleMax= 45 #最大旋转角度，单位°
    orn = 1 #力的方向，1-fz,2-mz
    angAccmax = 0.0 #最大旋转角加速度，单位°/s^2,暂不使用
    rotorn = 1 #旋转方向，1-顺时针，2-逆时针
    s_select = [0,0,1,1,1,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #最大阈值
    status = 1
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    robot.FT_RotInsertion(rcs,angVelRot,forceInsertion,angleMax,orn,angAccmax,rotorn)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)

直线插入
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn)"
    "描述", "直线插入"
    "参数", "- ``rcs``：参考坐标系，0-工具坐标系，1-基坐标系；
    - ``force_goal``：力或力矩阈值(0~100)，单位N或Nm；
    - ``lin_v``：直线速度，单位mm/s；
    - ``lin_a``：直线加速度，单位mm/s^2，暂不使用；
    - ``disMax``：最大插入距离，单位mm；
    - ``linorn``：插入方向，1-正方向，0-负方向；"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:
    
    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    #恒力参数
    status = 1  #恒力控制开启标志，0-关，1-开
    sensor_num = 1 #力传感器编号
    is_select = [0,0,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #最大阈值
    adj_sign = 0  #自适应启停状态，0-关闭，1-开启
    ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
    max_dis = 100.0  #最大调整距离
    max_ang = 5.0  #最大调整角度
    #直线插入参数
    rcs = 0  #参考坐标系，0-工具坐标系，1-基坐标系
    force_goal = 20.0  #力或力矩阈值（0~100），单位N或Nm
    lin_v = 0.0 #直线速度，单位mm/s
    lin_a = 0.0 #直线加速度，单位mm/s^2,暂不使用
    disMax = 100.0 #最大插入距离，单位mm
    linorn = 1 #插入方向，1-正方向，2-负方向
    is_select = [1,1,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    gain = [0.00005,0.0,0.0,0.0,0.0,0.0]  #最大阈值
    force_torque = [0.0,0.0,-30.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
    status = 1
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    robot.FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)

计算中间平面位置开始
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_CalCenterStart()"
    "描述", "计算中间平面位置开始"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

计算中间平面位置结束
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_CalCenterEnd()"
    "描述", "计算中间平面位置结束"
    "参数", "无"
    "返回值", "- 成功：[0,pos] ,pos=[x,y,z,rx,ry,rz]
    - 失败：[errcode]"

表面定位
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_FindSurface (rcs,dir,axis,lin_v,lin_a,disMax,force_goal)"
    "描述", "表面定位"
    "参数", "- ``rcs``： 参考坐标系，0-工具坐标系，1-基坐标系；
    - ``dir``：移动方向，1-正方向，2-负方向；
    - ``axis``：移动轴，1-x，2-y，3-z；
    - ``lin_v``：探索直线速度，单位mm/s；
    - ``lin_a``：探索直线加速度，单位mm/s^2；
    - ``disMax``：最大探索距离，单位mm
    - ``force_goal``：动作终止力阈值，单位N；"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    #恒力控制
    status = 1  #恒力控制开启标志，0-关，1-开
    sensor_num = 1 #力传感器编号
    is_select = [1,0,0,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    force_torque = [-2.0,0.0,0.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0002,0.0,0.0,0.0,0.0,0.0]  #最大阈值
    adj_sign = 0  #自适应启停状态，0-关闭，1-开启
    ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
    max_dis = 15.0  #最大调整距离
    max_ang = 0.0  #最大调整角度
    #表面定位参数
    rcs = 0 #参考坐标系，0-工具坐标系，1-基坐标系
    direction = 1 #移动方向，1-正方向，2-负方向
    axis = 1 #移动轴，1-X,2-Y,3-Z
    lin_v = 3.0  #探索直线速度，单位mm/s
    lin_a = 0.0  #探索直线加速度，单位mm/s^2
    disMax = 50.0 #最大探索距离，单位mm
    force_goal = 2.0 #动作终止力阈值，单位N
    P1=[-230.959,-364.017,226.179,-179.004,0.002,89.999]
    robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)       #关节空间点到点运动
    #x方向寻找中心
    #第1个表面
    robot.FT_CalCenterStart()
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    robot.FT_FindSurface(rcs,direction,axis,lin_v,lin_a,disMax,force_goal)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)       #关节空间点到点运动
    robot.WaitMs(1000)
    #第2个表面
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    direction = 2 #移动方向，1-正方向，2-负方向
    robot.FT_FindSurface(rcs,direction,axis,lin_v,lin_a,disMax,force_goal)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    #计算x方向中心位置
    xcenter= robot.FT_CalCenterEnd()
    print(xcenter)
    xcenter = [xcenter[1],xcenter[2],xcenter[3],xcenter[4],xcenter[5],xcenter[6]]
    robot.MoveCart(xcenter,9,0,60.0,50.0,50.0,0.0,-1)
    #y方向寻找中心
    #第1个表面
    robot.FT_CalCenterStart()
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    direction = 1 #移动方向，1-正方向，2-负方向
    axis = 2 #移动轴，1-X,2-Y,3-Z
    disMax = 150.0 #最大探索距离，单位mm
    lin_v = 6.0  #探索直线速度，单位mm/s
    robot.FT_FindSurface(rcs,direction,axis,lin_v,lin_a,disMax,force_goal)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)       #关节空间点到点运动
    robot.WaitMs(1000)
    #第2个表面
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    direction = 2 #移动方向，1-正方向，2-负方向
    robot.FT_FindSurface(rcs,direction,axis,lin_v,lin_a,disMax,force_goal)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    #计算y方向中心位置
    ycenter=robot.FT_CalCenterEnd()
    print(ycenter)
    ycenter = [ycenter[1],ycenter[2],ycenter[3],ycenter[4],ycenter[5],ycenter[6]]
    robot.MoveCart(ycenter,9,0,60.0,50.0,50.0,-1.0,-1)

柔顺控制关闭
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_ComplianceStop()"
    "描述", "柔顺控制关闭"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

柔顺控制开启
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "FT_ComplianceStart(p,force)"
    "描述", "柔顺控制开启"
    "参数", "- ``p``: 位置调节系数或柔顺系数
    - ``force``：柔顺开启力阈值，单位N"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------
.. code-block:: python
    :linenos:

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    J1=[-105.3,-68.0,-127.9,-75.5,90.8,77.8]
    P1=[-208.9,-274.5,334.6,178.8,-1.3,86.7]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[0.000,0.000,0.000,0.000,0.000,0.000]
    J2=[-105.3,-97.9,-101.5,-70.3,90.8,77.8]
    P2=[-264.8,-480.5,341.8,179.2,0.3,86.7]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[0.000,0.000,0.000,0.000,0.000,0.000]
    #恒力控制参数
    status = 1  #恒力控制开启标志，0-关，1-开
    sensor_num = 1 #力传感器编号
    is_select = [1,1,1,0,0,0]  #六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    force_torque = [-10.0,-10.0,-10.0,0.0,0.0,0.0]  #碰撞检测力和力矩，检测范围（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0005,0.0,0.0,0.0,0.0,0.0]  #最大阈值
    adj_sign = 0  #自适应启停状态，0-关闭，1-开启
    ILC_sign = 0  #ILC控制启停状态，0-停止，1-训练，2-实操
    max_dis = 1000.0  #最大调整距离
    max_ang = 0.0  #最大调整角度
    #柔顺控制
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    p = 0.00005  #位置调节系数或柔顺系数
    force = 30.0 #柔顺开启力阈值，单位N
    robot.FT_ComplianceStart(p,force)
    count = 15  #循环次数
    while(count):
        robot.MoveL(J1,P1,9,0,100.0,180.0,100.0,-1.0,eP1,0,1,dP1)   #笛卡尔空间直线运动
        robot.MoveL(J2,P2,9,0,100.0,180.0,100.0,-1.0,eP2,0,0,dP2)
        count = count - 1
    robot.FT_ComplianceStop()
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
