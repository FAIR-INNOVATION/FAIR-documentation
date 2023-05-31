机器人运动
============

.. toctree:: 
    :maxdepth: 5

机器人点动
+++++++++++++

jog点动
---------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``StartJOG(ref,nb,dir,vel,acc,max_dis)``"
    "描述", "jog点动"
    "参数", "- ``ref``：0-关节点动,2-基坐标系点动,4-工具坐标系点动,8-工件坐标系点动；
    - ``nb``：1-1关节(x轴),2-2关节(y轴),3-3关节(z轴),4-4关节(rx),5-5关节(ry),6-6关节(rz);
    - ``dir``：0-负方向，1-正方向;
    - ``vel``：速度百分比，[0~100];
    - ``acc``：加速度百分比，[0~100];
    - ``max_dis``：单次点动最大角度/距离，单位°或mm"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

jog点动减速停止
-----------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``StopJOG(ref)``"
    "描述", "jog点动减速停止"
    "参数", "- ``ref``：1-关节点动停止,3-基坐标系点动停止,5-工具坐标系点动停止,9-工件坐标系点动停止"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"


jog点动立即停止
-----------------

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``ImmStopJOG()``"
    "描述", "jog点动立即停止"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"
     
代码示例
^^^^^^^^^^^^

.. code-block:: python
    :linenos:
    :emphasize-lines: 6,9,11

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    # 机器人单轴点动
    robot.StartJOG(0,1,0,20.0,20.0,30.0)    # 单关节运动,StartJOG为非阻塞指令，运动状态下接收其他运动指令（包含StartJOG）会被丢弃
    time.sleep(1)
    #机器人单轴点动减速停止
    # robot.StopJOG(1)
    #机器人单轴点动立即停止
    robot.ImmStopJOG()
    robot.StartJOG(0,2,1,20.0,20.0,30.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(0,3,1,20.0,20.0,30.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(0,4,1,20.0,20.0,30.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(0,5,1,20.0,20.0,30.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(0,6,1,20.0,20.0,30.0)
    time.sleep(1)
    robot.ImmStopJOG()
    # 基坐标
    robot.StartJOG(2,1,0,20.0,20.0,100.0)  #基坐标系下点动
    time.sleep(1)
    #机器人单轴点动减速停止
    # robot.StopJOG(2)
    # #机器人单轴点动立即停止
    robot.ImmStopJOG()
    robot.StartJOG(2,1,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,2,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,3,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,4,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,5,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(2,6,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    # 工具坐标
    robot.StartJOG(4,1,0,20.0,20.0,100.0)  #工具坐标系下点动
    time.sleep(1)
    #机器人单轴点动减速停止
    # robot.StopJOG(5)
    # #机器人单轴点动立即停止
    robot.ImmStopJOG()
    robot.StartJOG(4,1,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,2,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,3,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,4,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,5,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(4,6,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    # 工件坐标
    robot.StartJOG(8,1,0,20.0,20.0,100.0)  #工件坐标系下点动
    time.sleep(1)
    #机器人单轴点动减速停止
    # robot.StopJOG(9)
    # #机器人单轴点动立即停止
    robot.ImmStopJOG()
    robot.StartJOG(8,1,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,2,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,3,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,4,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,5,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()
    robot.StartJOG(8,6,1,20.0,20.0,100.0)
    time.sleep(1)
    robot.ImmStopJOG()


关节空间运动
++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``MoveJ(joint_pos,desc_pos,tool,user,vel,acc,ovl,exaxis_pos,blendT,offset_flag,offset_pos)``"
    "描述", "关节空间运动"
    "参数", "- ``joint_pos``:目标关节位置，单位[°]；
    - ``desc_pos``:目标笛卡尔位姿，单位[mm][°]；
    - ``tool``:工具号，[0~14]；
    - ``user``:工件号，[0~14]；
    - ``vel``:速度百分比，[0~100]；
    - ``acc``:加速度百分比，[0~100]，暂不开放；
    - ``ovl``:速度缩放因子，[0~100]；
    - ``exaxis_pos``:外部轴1位置~外部轴4位置；
    - ``blendT``:[-1.0]-运动到位(阻塞)，[0~500]-平滑时间(非阻塞)，单位[ms]；
    - ``offset_flag``:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移；
    - ``offset_pos``:位姿偏移量，单位[mm][°]"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
-------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 13,14

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    J1=[-168.847,-93.977,-93.118,-80.262,88.985,11.831]
    P1=[-558.082,27.343,208.135,-177.205,-0.450,89.288]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[1.000,1.000,1.000,1.000,1.000,1.000]
    J2=[168.968,-93.977,-93.118,-80.262,88.986,11.831]
    P2=[-506.436,236.053,208.133,-177.206,-0.450,67.102]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[1.000,1.000,1.000,1.000,1.000,1.000]
    robot.MoveJ(J1,P1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)    #关节空间运动PTP,工具号1，实际测试根据现场数据及工具号使用
    robot.MoveJ(J2,P2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
    time.sleep(2)
    j1 = robot.GetInverseKin(0,P1,-1)       #只有笛卡尔空间坐标的情况下，可用逆运动学接口求解关节位置
    print(j1)
    j1 = [j1[1],j1[2],j1[3],j1[4],j1[5],j1[6]]
    robot.MoveJ(j1,P1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1) 
    j2 = robot.GetInverseKin(0,P2,-1)
    print(j2)
    j2 = [j2[1],j2[2],j2[3],j2[4],j2[5],j2[6]]
    robot.MoveJ(j2,P2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
    time.sleep(2)
    p1 = robot.GetForwardKin(J1)       #只有关节位置的情况下，可用正运动学接口求解笛卡尔空间坐标
    print(p1)
    p1 = [p1[1],p1[2],p1[3],p1[4],p1[5],p1[6]]
    robot.MoveJ(J1,p1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1) 
    p2 = robot.GetForwardKin(J2)
    print(p2)
    p2 = [p2[1],p2[2],p2[3],p2[4],p2[5],p2[6]]
    robot.MoveJ(J2,p2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)

笛卡尔空间直线运动
+++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``MoveL(joint_pos,desc_pos,tool,user,vel,acc,ovl,blendR,exaxis_pos,search,offset_flag,offset_pos)``"
    "描述", "笛卡尔空间直线运动"
    "参数", "- ``joint_pos``:目标关节位置，单位[°]；
    - ``desc_pos``:目标笛卡尔位姿，单位[mm][°]；
    - ``tool``:工具号，[0~14]；
    - ``user``:工件号，[0~14]；
    - ``vel``:速度百分比，[0~100]；
    - ``acc``:加速度百分比，[0~100]，暂不开放；
    - ``ovl``:速度缩放因子，[0~100]；
    - ``blendR``:[-1.0]-运动到位(阻塞)，[0~1000]-平滑半径(非阻塞)，单位[mm]；
    - ``exaxis_pos``:外部轴1位置~外部轴4位置；
    - ``search``:[0]-不焊丝寻位，[1]-焊丝寻位；
    - ``offset_flag``:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移；
    - ``offset_pos``:位姿偏移量，单位[mm][°]"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 16-18

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[10.000,10.000,10.000,0.000,0.000,0.000]
    J2=[123.709,-121.190,-82.838,-63.499,90.471,-47.174]
    P2=[-273.856,643.260,259.235,-177.972,-1.494,80.866]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[0.000,0.000,0.000,0.000,0.000,0.000]
    J3=[167.066,-95.700,-123.494,-42.493,90.466,-47.174]
    P3=[-423.044,229.703,241.080,-173.990,-5.772,123.971]
    eP3=[0.000,0.000,0.000,0.000]
    dP3=[0.000,0.000,0.000,0.000,0.000,0.000]
    robot.MoveL(J1,P1,0,0,100.0,180.0,100.0,-1.0,eP1,0,1 ,dP1)   #笛卡尔空间直线运动
    robot.MoveL(J2,P2,0,0,100.0,180.0,100.0,-1.0,eP2,0,0,dP2)
    robot.MoveL(J3,P3,0,0,100.0,180.0,100.0,-1.0,eP3,0,0,dP3)

笛卡尔空间圆弧运动
++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``MoveC(joint_pos_p,desc_pos_p,ptool,puser,pvel,pacc,exaxis_pos_p,poffset_flag,offset_pos_p,joint_pos_t,desc_pos_t,ttool,tuser,tvel,tacc,exaxis_pos_t ,toffset_flag,offset_pos_t,ovl,blendR)``"
    "描述", "笛卡尔空间圆弧运动"
    "参数", "- ``joint_pos_p``:路径点关节位置，单位[°]；
    - ``desc_pos_p``:路径点笛卡尔位姿，单位[mm][°]；
    - ``ptool``:工具号，[0~14]；
    - ``puser``:工件号，[0~14]；
    - ``pvel``:速度百分比，[0~100]；
    - ``pacc``:加速度百分比，[0~100]，暂不开放；
    - ``exaxis_pos_p``:外部轴1位置~外部轴4位置；
    - ``poffset_flag``:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移；
    - ``offset_pos_p``:偏移量，单位[mm][°]；
    - ``joint_pos_t``:目标点关节位置，单位[°]；
    - ``desc_pos_t``:目标点笛卡尔位姿，单位[mm][°]；
    - ``ttool``:工具号，[0~14]；
    - ``tuser``:工件号，[0~14]；
    - ``tvel``:速度百分比，[0~100]；
    - ``tacc``:加速度百分比，[0~100]，暂不开放；
    - ``exaxis_pos_t``:外部轴1位置~外部轴4位置；
    - ``toffset_flag``:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移；
    - ``offset_pos_t``:偏移量，单位[mm][°]。
    - ``ovl``:速度缩放因子，[0~100]；
    - ``blendR``:[-1.0]-运动到位(阻塞)，[0~1000]-平滑半径(非阻塞)，单位[mm]"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
-------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 19,20

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    J1=[121.381,-97.108,-123.768,-45.824,89.877,-47.296]
    P1=[-127.772,459.534,221.274,-177.850,-2.507,78.627]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[10.000,10.000,10.000,10.000,10.000,10.000]
    J2=[138.884,-114.522,-103.933,-49.694,90.688,-47.291]
    P2=[-360.468,485.600,196.363,-178.239,-0.893,96.172]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[10.000,10.000,10.000,10.000,10.000,10.000]
    pa2=[0.0,0.0,100.0,180.0]
    J3=[159.164,-96.105,-128.653,-41.170,90.704,-47.290]
    P3=[-360.303,274.911,203.968,-176.720,-2.514,116.407]
    eP3=[0.000,0.000,0.000,0.000]
    dP3=[10.000,10.000,10.000,10.000,10.000,10.000]
    pa3=[0.0,0.0,100.0,180.0]
    dP=[10.000,10.000,10.000,10.000,10.000,10.000]
    robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)       #关节空间运动PTP
    robot.MoveC(J2,P2,pa2,eP2,0,dP2,J3,P3,pa3,eP3,0,dP3,100.0,-1.0)    #笛卡尔空间圆弧运动

笛卡尔空间整圆运动
+++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``Circle(joint_pos_p,desc_pos_p,ptool,puser,pvel,pacc,exaxis_pos_p,joint_pos_t,desc_pos_t,ttool,tuser,tvel,tacc,exaxis_pos_t,ovl,offset_flag,offset_pos)``"
    "描述", "笛卡尔空间整圆运动"
    "参数", "- ``joint_pos_p``:路径点关节位置，单位[°]；
    - ``desc_pos_p``:路径点笛卡尔位姿，单位[mm][°]；
    - ``ptool``:工具号，[0~14]；
    - ``puser``:工件号，[0~14]；
    - ``pvel``:速度百分比，[0~100]；
    - ``pacc``:加速度百分比，[0~100]，暂不开放；
    - ``exaxis_pos_p``:外部轴1位置~外部轴4位置；
    - ``joint_pos_t``:目标点关节位置，单位[°]；
    - ``desc_pos_t``:目标点笛卡尔位姿，单位[mm][°]；
    - ``ttool``:工具号，[0~14]；
    - ``tuser``:工件号，[0~14]；
    - ``tvel``:速度百分比，[0~100]；
    - ``tacc``:加速度百分比，[0~100]，暂不开放；
    - ``exaxis_pos_t``:外部轴1位置~外部轴4位置；
    - ``ovl``:速度缩放因子，[0~100%]；
    - ``offset_flag``:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移；
    - ``offset_pos``:偏移量，单位[mm][°]"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
-------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 19,20

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    J1=[121.381,-97.108,-123.768,-45.824,89.877,-47.296]
    P1=[-127.772,459.534,221.274,-177.850,-2.507,78.627]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[10.000,10.000,10.000,10.000,10.000,10.000]
    J2=[138.884,-114.522,-103.933,-49.694,90.688,-47.291]
    P2=[-360.468,485.600,196.363,-178.239,-0.893,96.172]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[10.000,10.000,10.000,10.000,10.000,10.000]
    pa2=[0.0,0.0,100.0,180.0]
    J3=[159.164,-96.105,-128.653,-41.170,90.704,-47.290]
    P3=[-360.303,274.911,203.968,-176.720,-2.514,116.407]
    eP3=[0.000,0.000,0.000,0.000]
    dP3=[10.000,10.000,10.000,10.000,10.000,10.000]
    pa3=[0.0,0.0,100.0,180.0]
    dP=[10.000,10.000,10.000,10.000,10.000,10.000]
    robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)    #关节空间运动PTP
    robot.Circle(J2,P2,pa2,eP2,J3,P3,pa3,eP3,100.0,0,dP)    #笛卡尔空间整圆运动

笛卡尔空间螺旋线运动
++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``NewSpiral(joint_pos,desc_pos,tool,user,vel,acc,exaxis_pos,ovl,offset_flag,offset_pos,param)``"
    "描述", "笛卡尔空间螺旋线运动"
    "参数", "- ``joint_pos``:目标关节位置，单位[°]；
    - ``desc_pos``:目标笛卡尔位姿，单位[mm][°]；
    - ``tool``:工具号，[0~14]；
    - ``user``:工件号，[0~14]；
    - ``vel``:速度百分比，[0~100]；
    - ``acc``:加速度百分比，[0~100]，暂不开放；
    - ``exaxis_pos``:外部轴1位置~外部轴4位置；
    - ``ovl``:速度缩放因子，[0~100]；
    - ``offset_flag``:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移；
    - ``offset_pos``:位姿偏移量，单位[mm][°]
    - ``param``:[circle_num,circle_angle,rad_init,rad_add,rotaxis_add,rot_direction]，circle_num:螺旋圈数，circle_angle:螺旋倾角，rad_init:螺旋初始半径，rad_add:半径增量，rotaxis_add:转轴方向增量，rot_direction:旋转方向，0-顺时针，1-逆时针"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
---------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 18

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    J1=[127.888,-101.535,-94.860,17.836,96.931,-61.325]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[50.0,0.0,0.0,-30.0,0.0,0.0]
    J2=[127.888,-101.535,-94.860,17.836,96.931,-61.325]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[50.0,0.0,0.0,-5.0,0.0,0.0]
    Pa = [5.0,5.0,50.0,10.0,10.0,0.0]
    P1 = robot.GetForwardKin(J1)       #只有关节位置的情况下，可用正运动学接口求解笛卡尔空间坐标
    print(P1)
    P1 = [P1[1],P1[2],P1[3],P1[4],P1[5],P1[6]]
    robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,0.0,2,dP1)
    P2 = robot.GetForwardKin(J2)       #只有关节位置的情况下，可用正运动学接口求解笛卡尔空间坐标
    print(P2)
    P2 = [P2[1],P2[2],P2[3],P2[4],P2[5],P2[6]]
    robot.NewSpiral(J2,P2,0,0,100.0,180.0,eP2,100.0,2,dP2,Pa)   #螺旋线运动

关节空间伺服模式运动
+++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``ServoJ(joint_pos,acc,vel,cmdT,filterT,gain)``"
    "描述", "关节空间伺服模式运动"
    "参数", "- ``joint_pos``:目标关节位置，单位[°]；
    - ``acc``:加速度，范围[0~100]，暂不开放，默认为0；
    - ``vel``:速度，范围[0~100]，暂不开放，默认为0；
    - ``cmdT``:指令周期，单位[s]，[0.001~0.016]；
    - ``filterT``:滤波时间，单位[s]，暂不开放；
    - ``gain``:目标位置的比例放大器，暂不开放"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
--------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 15

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    joint_pos = robot.GetActualJointPosDegree(0)
    print(joint_pos)
    joint_pos = [joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5],joint_pos[6]]
    acc = 0.0
    vel = 0.0
    t = 0.008
    lookahead_time = 0.0
    P = 0.0
    count = 100
    while(count):
        robot.ServoJ(joint_pos, acc, vel, t, lookahead_time, P)
        joint_pos[0] = joint_pos[0] + 0.1
        count = count - 1
        time.sleep(0.008)

笛卡尔空间伺服模式运动
++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``ServoCart(mode,desc_pos,pos_gain,acc,vel,cmdT,filterT,gain)``"
    "描述", "笛卡尔空间伺服模式运动"
    "参数", "- ``mode``:[0]-绝对运动(基坐标系)，[1]-增量运动(基坐标系)，[2]-增量运动(工具坐标系)；
    - ``desc_pos``:目标笛卡尔位置/目标笛卡尔位置增量；
    - ``pos_gain``:位姿增量比例系数，仅在增量运动下生效，范围[0~1]；
    - ``acc``:加速度，范围[0~100]，暂不开放，默认为0；
    - ``vel``:速度，范围[0~100]，暂不开放，默认为0；
    - ``cmdT``:指令周期，单位[s]，[0.001~0.016]；
    - ``filterT``:滤波时间，单位[s]，暂不开放；
    - ``gain``:目标位置的比例放大器，暂不开放"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
--------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 15

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    mode = 2  #工具坐标系增量运动
    n_pos = [0.0,0.0,0.5,0.0,0.0,0.0]   #笛卡尔空间位姿增量
    gain = [0.0,0.0,1.0,0.0,0.0,0.0]
    acc = 0.0
    vel = 0.0
    t = 0.008
    lookahead_time = 0.0
    P = 0.0
    count = 100
    while(count):
        robot.ServoCart(mode, n_pos, gain, acc, vel, t, lookahead_time, P)
        count = count - 1
        time.sleep(0.008)

笛卡尔空间点到点运动
++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``MoveCart(desc_pos,tool,user,vel,acc,ovl,blendT,config)``"
    "描述", "笛卡尔空间点到点运动"
    "参数", "- ``desc_pos``:目标笛卡尔位置；
    - ``tool``:工具号，[0~14]；
    - ``user``:工件号，[0~14]；
    - ``vel``:速度，范围[0~100]，暂不开放，默认为0；
    - ``acc``:加速度，范围[0~100]，暂不开放，默认为0；
    - ``ovl``:速度缩放因子，[0~100]；
    - ``blendT``:[-1.0]-运动到位(阻塞)，[0~500]-平滑时间(非阻塞)，单位[ms]；
    - ``config``:关节配置，[-1]-参考当前关节位置求解，[0~7]-依据关节配置求解"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
-------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 8-10

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    P2=[-273.856,643.260,259.235,-177.972,-1.494,80.866]
    P3=[-423.044,229.703,241.080,-173.990,-5.772,123.971]
    robot.MoveCart(P1,0,0,100.0,100.0,100.0,-1.0,-1)       #笛卡尔空间点到点运动
    robot.MoveCart(P2,0,0,100.0,100.0,100.0,-1.0,-1)
    robot.MoveCart(P3,0,0,100.0,100.0,100.0,0.0,-1)
    time.sleep(1)
    robot.StopMotion()    #停止运动

机器人样条运动
++++++++++++++++

样条运动开始
-----------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SplineStart()``"
    "描述", "样条运动开始"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

样条运动PTP
---------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SplinePTP(joint_pos,desc_pos,tool,user,vel,acc,ovl)``"
    "描述", "样条运动PTP"
    "参数", "- ``joint_pos``:目标关节位置，单位[°]；
    - ``desc_pos``:目标笛卡尔位姿，单位[mm][°]；
    - ``tool``:工具号，[0~14]；
    - ``user``:工件号，[0~14]；
    - ``vel``:速度，范围[0~100]，暂不开放，默认为0；
    - ``acc``:加速度，范围[0~100]，暂不开放，默认为0；
    - ``ovl``:速度缩放因子，[0~100]；"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

样条运动结束
----------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``SplineEnd()``"
    "描述", "样条运动结束"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
^^^^^^^^^^^^

.. code-block:: python
    :linenos:
    :emphasize-lines: 15-20

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    J1 = [114.578,-117.798,-97.745,-54.436,90.053,-45.216]
    P1 = [-140.418,619.351,198.369,-179.948,0.023,69.793]
    eP1 = [0.000,0.000,0.000,0.000]
    dP1 = [0.000,0.000,0.000,0.000,0.000,0.000]
    J2 = [115.401,-105.206,-117.959,-49.727,90.054,-45.222]
    P2 = [-95.586,504.143,186.880,178.001,2.091,70.585]
    J3 = [135.609,-103.249,-120.211,-49.715,90.058,-45.219]
    P3 = [-252.429,428.903,188.492,177.804,2.294,90.782]
    J4 = [154.766,-87.036,-135.672,-49.045,90.739,-45.223]
    P4 = [-277.255,272.958,205.452,179.289,1.765,109.966]
    robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
    robot.SplineStart()    #样条运动开始
    robot.SplinePTP(J1,P1,0,0,100.0,180.0,100.0)    #样条PTP运动
    robot.SplinePTP(J2,P2,0,0,100.0,180.0,100.0)
    robot.SplinePTP(J3,P3,0,0,100.0,180.0,100.0)
    robot.SplinePTP(J4,P4,0,0,100.0,180.0,100.0)
    robot.SplineEnd()     #样条运动结束

机器人新样条运动
+++++++++++++++++++
新样条运动开始
------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``NewSplineStart(type)``"
    "描述", "新样条运动开始"
    "参数", "- ``type``:0-圆弧过渡，1-给定点位路径点"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"


新样条运动结束
-------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "原型", "``NewSplineEnd()``"
    "描述", "新样条运动结束"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"


新样条指令点
----------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "原型", "``NewSplinePoint(joint_pos,desc_pos,tool,user,vel,acc,ovl,blendR,lastFlag)``"
    "描述", "新样条指令点"
    "参数", "- ``joint_pos``:目标关节位置，单位[°]；
    - ``desc_pos``:目标笛卡尔位姿，单位[mm][°]；
    - ``tool``:工具号，[0~14]；
    - ``user``:工件号，[0~14]；
    - ``vel``:速度，范围[0~100]，暂不开放，默认为0；
    - ``acc``:加速度，范围[0~100]，暂不开放，默认为0；
    - ``ovl``:速度缩放因子，[0~100]；
    - ``blendR``: [0~1000]-平滑半径，单位[mm]；
    - ``lastFlag``:是否为最后一个点，0-否，1-是"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
^^^^^^^^^^^^

.. code-block:: python
    :linenos:
    :emphasize-lines: 15-20

    import frrpc
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    J1 = [114.578,-117.798,-97.745,-54.436,90.053,-45.216]
    P1 = [-140.418,619.351,198.369,-179.948,0.023,69.793]
    eP1 = [0.000,0.000,0.000,0.000]
    dP1 = [0.000,0.000,0.000,0.000,0.000,0.000]
    J2 = [115.401,-105.206,-117.959,-49.727,90.054,-45.222]
    P2 = [-95.586,504.143,186.880,178.001,2.091,70.585]
    J3 = [135.609,-103.249,-120.211,-49.715,90.058,-45.219]
    P3 = [-252.429,428.903,188.492,177.804,2.294,90.782]
    J4 = [154.766,-87.036,-135.672,-49.045,90.739,-45.223]
    P4 = [-277.255,272.958,205.452,179.289,1.765,109.966]
    robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
    robot.NewSplineStart(1)    #样条运动开始
    robot.NewSplinePoint(J1,P1,0,0,50.0,50.0,50.0,0.0,0)    #样条控制点
    robot.NewSplinePoint(J2,P2,0,0,50.0,50.0,50.0,0.0,0)
    robot.NewSplinePoint(J3,P3,0,0,50.0,50.0,50.0,0.0,0)
    robot.NewSplinePoint(J4,P4,0,0,50.0,50.0,50.0,0.0,1)
    robot.NewSplineEnd() 

机器人终止运动
++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "原型", "``StopMotion()``"
    "描述", "终止运动，使用终止运动需运动指令为非阻塞状态"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
-------------

.. code-block:: python
    :linenos:
    :emphasize-lines: 12

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
    P2=[-273.856,643.260,259.235,-177.972,-1.494,80.866]
    P3=[-423.044,229.703,241.080,-173.990,-5.772,123.971]
    robot.MoveCart(P1,0,0,100.0,100.0,100.0,-1.0,-1)       #关节空间点到点运动
    robot.MoveCart(P2,0,0,100.0,100.0,100.0,-1.0,-1)
    robot.MoveCart(P3,0,0,100.0,100.0,100.0,0.0,-1)   #此条运动指令为非阻塞状态
    time.sleep(1)
    robot.StopMotion()    #停止运动

机器人点位整体偏移
+++++++++++++++++++
点位整体偏移开始
-------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "原型", "``PointsOffsetEnable(flag,offset_pos)``"
    "描述", "点位整体偏移开始"
    "参数", "- ``flag``:0-基坐标或工件坐标系下偏移， 2-工具坐标系下偏移；
    - ``offset_pos``:偏移量，单位[mm][°]。"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

点位整体偏移结束
--------------------
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "原型", "``PointsOffsetDisable()``"
    "描述", "点位整体偏移结束"
    "参数", "无"
    "返回值", "- 成功：[0]
    - 失败：[errcode]"

代码示例
^^^^^^^^^^^^

.. code-block:: python
    :linenos:
    :emphasize-lines: 19,22

    import frrpc
    import time
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = frrpc.RPC('192.168.58.2')
    #机器人点位整体偏移
    J1=[-168.847,-93.977,-93.118,-80.262,88.985,11.831]
    P1=[-558.082,27.343,208.135,-177.205,-0.450,89.288]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[10.000,10.000,10.000,0.000,0.000,0.000]
    J2=[168.968,-93.977,-93.118,-80.262,88.986,11.831]
    P2=[-506.436,236.053,208.133,-177.206,-0.450,67.102]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[0.000,0.000,0.000,0.000,0.000,0.000]
    robot.MoveJ(J1,P1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
    robot.MoveJ(J2,P2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
    time.sleep(2)
    flag = 0
    offset = [100.0,5.0,6.0,0.0,0.0,0.0]   #位姿偏移量
    robot.PointsOffsetEnable(flag, offset)   #整体偏移开始
    robot.MoveJ(J1,P1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
    robot.MoveJ(J2,P2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
    robot.PointsOffsetDisable()  #整体偏移结束
