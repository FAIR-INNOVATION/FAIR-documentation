机器人运动
============

.. toctree:: 
    :maxdepth: 5


jog点动
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  jog点动
    * @param  [in]  ref 0-关节点动，2-基坐标系下点动，4-工具坐标系下点动，8-工件坐标系下点动
    * @param  [in]  nb 1-关节1(或x轴)，2-关节2(或y轴)，3-关节3(或z轴)，4-关节4(或绕x轴旋转)，5-关节5(或绕y轴旋转)，6-关节6(或绕z轴旋转)
    * @param  [in]  dir 0-负方向，1-正方向
    * @param  [in]  vel 速度百分比，[0~100]
    * @param  [in]  acc 加速度百分比， [0~100]
    * @param  [in]  max_dis 单次点动最大角度，单位[°]或距离，单位[mm]
    * @return  错误码
    */
    errno_t  StartJOG(uint8_t ref, uint8_t nb, uint8_t dir, float vel, float acc, float max_dis);

jog点动减速停止
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  jog点动减速停止
    * @param  [in]  ref  1-关节点动停止，3-基坐标系下点动停止，5-工具坐标系下点动停止，9-工件坐标系下点动停止
    * @return  错误码
    */
    errno_t  StopJOG(uint8_t ref);

jog点动立即停止
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief jog点动立即停止
    * @return  错误码
    */
    errno_t  ImmStopJOG(); 

代码示例
++++++++++++++
.. code-block:: c++
    :linenos:

    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    #include "FRRobot.h"
    #include "RobotTypes.h"

    using namespace std;

    int main(void)
    {
        FRRobot robot;                 //实例化机器人对象
        robot.RPC("192.168.58.2");     //与机器人控制器建立通信连接

        robot.StartJOG(0,1,0,20.0,20.0,30.0);   //单关节运动，StartJOG为非阻塞指令，运动状态下接收其他运动指令（包含StartJOG）会被丢弃
        sleep(1);
        //robot.StopJOG(1)  //机器人单轴点动减速停止
        robot.ImmStopJOG();  //机器人单轴点动立即停止
        robot.StartJOG(0,2,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG(); 
        robot.StartJOG(0,3,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG();
        robot.StartJOG(0,4,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG();  
        robot.StartJOG(0,5,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG(); 
        robot.StartJOG(0,6,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG(); 

        robot.StartJOG(2,1,0,20.0,20.0,30.0);   //基坐标系下点动
        sleep(1);
        //robot.StopJOG(3)  //机器人单轴点动减速停止
        robot.ImmStopJOG();  //机器人单轴点动立即停止
        robot.StartJOG(2,2,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG(); 
        robot.StartJOG(2,3,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG();
        robot.StartJOG(2,4,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG();  
        robot.StartJOG(2,5,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG(); 
        robot.StartJOG(2,6,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG(); 

        robot.StartJOG(4,1,0,20.0,20.0,30.0);   //工具坐标系下点动
        sleep(1);
        //robot.StopJOG(5)  //机器人单轴点动减速停止
        robot.ImmStopJOG();  //机器人单轴点动立即停止
        robot.StartJOG(4,2,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG(); 
        robot.StartJOG(4,3,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG();
        robot.StartJOG(4,4,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG();  
        robot.StartJOG(4,5,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG(); 
        robot.StartJOG(4,6,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG(); 

        robot.StartJOG(8,1,0,20.0,20.0,30.0);   //工件坐标系下点动
        sleep(1);
        //robot.StopJOG(9)  //机器人单轴点动减速停止
        robot.ImmStopJOG();  //机器人单轴点动立即停止
        robot.StartJOG(8,2,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG(); 
        robot.StartJOG(8,3,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG();
        robot.StartJOG(8,4,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG();  
        robot.StartJOG(8,5,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG(); 
        robot.StartJOG(8,6,1,20.0,20.0,30.0);
        sleep(1);
        robot.ImmStopJOG(); 

        return 0;
    }

关节空间运动
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  关节空间运动
    * @param  [in] joint_pos  目标关节位置,单位deg
    * @param  [in] desc_pos   目标笛卡尔位姿
    * @param  [in] tool  工具坐标号，范围[1~15]
    * @param  [in] user  工件坐标号，范围[1~15]
    * @param  [in] vel  速度百分比，范围[0~100]
    * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
    * @param  [in] ovl  速度缩放因子，范围[0~100]
    * @param  [in] epos  扩展轴位置，单位mm
    * @param  [in] blendT [-1.0]-运动到位(阻塞)，[0~500.0]-平滑时间(非阻塞)，单位ms
    * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
    * @param  [in] offset_pos  位姿偏移量
    * @return  错误码
    */
    errno_t  MoveJ(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos *epos, float blendT, uint8_t offset_flag, DescPose *offset_pos);

笛卡尔空间直线运动
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  笛卡尔空间直线运动
    * @param  [in] joint_pos  目标关节位置,单位deg
    * @param  [in] desc_pos   目标笛卡尔位姿
    * @param  [in] tool  工具坐标号，范围[1~15]
    * @param  [in] user  工件坐标号，范围[1~15]
    * @param  [in] vel  速度百分比，范围[0~100]
    * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
    * @param  [in] ovl  速度缩放因子，范围[0~100]
    * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm    
    * @param  [in] epos  扩展轴位置，单位mm
    * @param  [in] search  0-不焊丝寻位，1-焊丝寻位
    * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
    * @param  [in] offset_pos  位姿偏移量
    * @return  错误码
    */   
    errno_t  MoveL(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos);

笛卡尔空间圆弧运动
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  笛卡尔空间圆弧运动
    * @param  [in] joint_pos_p  路径点关节位置,单位deg
    * @param  [in] desc_pos_p   路径点笛卡尔位姿
    * @param  [in] ptool  工具坐标号，范围[1~15]
    * @param  [in] puser  工件坐标号，范围[1~15]
    * @param  [in] pvel  速度百分比，范围[0~100]
    * @param  [in] pacc  加速度百分比，范围[0~100],暂不开放
    * @param  [in] epos_p  扩展轴位置，单位mm
    * @param  [in] poffset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
    * @param  [in] offset_pos_p  位姿偏移量
    * @param  [in] joint_pos_t  目标点关节位置,单位deg
    * @param  [in] desc_pos_t   目标点笛卡尔位姿
    * @param  [in] ttool  工具坐标号，范围[1~15]
    * @param  [in] tuser  工件坐标号，范围[1~15]
    * @param  [in] tvel  速度百分比，范围[0~100]
    * @param  [in] tacc  加速度百分比，范围[0~100],暂不开放
    * @param  [in] epos_t  扩展轴位置，单位mm
    * @param  [in] toffset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
    * @param  [in] offset_pos_t  位姿偏移量   
    * @param  [in] ovl  速度缩放因子，范围[0~100]    
    * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm    
    * @return  错误码
    */      
    errno_t  MoveC(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, uint8_t poffset_flag, DescPose *offset_pos_p,JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, uint8_t toffset_flag, DescPose *offset_pos_t,float ovl, float blendR);

笛卡尔空间整圆运动
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  笛卡尔空间整圆运动
    * @param  [in] joint_pos_p  路径点1关节位置,单位deg
    * @param  [in] desc_pos_p   路径点1笛卡尔位姿
    * @param  [in] ptool  工具坐标号，范围[1~15]
    * @param  [in] puser  工件坐标号，范围[1~15]
    * @param  [in] pvel  速度百分比，范围[0~100]
    * @param  [in] pacc  加速度百分比，范围[0~100],暂不开放
    * @param  [in] epos_p  扩展轴位置，单位mm
    * @param  [in] joint_pos_t  路径点2关节位置,单位deg
    * @param  [in] desc_pos_t   路径点2笛卡尔位姿
    * @param  [in] ttool  工具坐标号，范围[1~15]
    * @param  [in] tuser  工件坐标号，范围[1~15]
    * @param  [in] tvel  速度百分比，范围[0~100]
    * @param  [in] tacc  加速度百分比，范围[0~100],暂不开放
    * @param  [in] epos_t  扩展轴位置，单位mm
    * @param  [in] ovl  速度缩放因子，范围[0~100]   
    * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
    * @param  [in] offset_pos  位姿偏移量     
    * @return  错误码
    */      
    errno_t  Circle(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, float ovl, uint8_t offset_flag, DescPose *offset_pos);

代码示例
++++++++++++++
.. code-block:: c++
    :linenos:

    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    #include "FRRobot.h"
    #include "RobotTypes.h"

    using namespace std;

    int main(void)
    {
        FRRobot robot;                 //实例化机器人对象
        robot.RPC("192.168.58.2");     //与机器人控制器建立通信连接

        JointPos j1,j2,j3,j4;
        DescPose desc_pos1,desc_pos2,desc_pos3,desc_pos4,offset_pos;
        ExaxisPos  epos;

        memset(&j1, 0, sizeof(JointPos));
        memset(&j2, 0, sizeof(JointPos));
        memset(&j3, 0, sizeof(JointPos));
        memset(&j4, 0, sizeof(JointPos));
        memset(&desc_pos1, 0, sizeof(DescPose));
        memset(&desc_pos2, 0, sizeof(DescPose));
        memset(&desc_pos3, 0, sizeof(DescPose));
        memset(&desc_pos4, 0, sizeof(DescPose));
        memset(&offset_pos, 0, sizeof(DescPose));
        memset(&epos, 0, sizeof(ExaxisPos));

        j1 = {114.578,-117.798,-97.745,-54.436,90.053,-45.216};
        desc_pos1.tran.x = -140.418;
        desc_pos1.tran.y = 619.351;
        desc_pos1.tran.z = 198.369;
        desc_pos1.rpy.rx = -179.948;
        desc_pos1.rpy.ry = 0.023;
        desc_pos1.rpy.rz = 69.793;

        j2 = {121.381,-97.108,-123.768,-45.824,89.877,-47.296};
        desc_pos2.tran.x = -127.772;
        desc_pos2.tran.y = 459.534;
        desc_pos2.tran.z = 221.274;
        desc_pos2.rpy.rx = -177.850;
        desc_pos2.rpy.ry = -2.507;
        desc_pos2.rpy.rz = 78.627;

        j3 = {138.884,-114.522,-103.933,-49.694,90.688,-47.291};
        desc_pos3.tran.x = -360.468;
        desc_pos3.tran.y = 485.600;
        desc_pos3.tran.z = 196.363;
        desc_pos3.rpy.rx = -178.239;
        desc_pos3.rpy.ry = -0.893;
        desc_pos3.rpy.rz = 96.172;

        j4 = {159.164,-96.105,-128.653,-41.170,90.704,-47.290};
        desc_pos4.tran.x = -360.303;
        desc_pos4.tran.y = 274.911;
        desc_pos4.tran.z = 203.968;
        desc_pos4.rpy.rx = -176.720;
        desc_pos4.rpy.ry = -2.514;
        desc_pos4.rpy.rz = 116.407;   

        int tool = 0;
        int user = 0;
        float vel = 100.0;
        float acc = 100.0;
        float ovl = 100.0;
        float blendT = 0.0;
        float blendR = 0.0;
        uint8_t flag = 0;
        uint8_t search = 0;

        robot.SetSpeed(20);
        
        int err1 = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT,flag, &offset_pos);
        printf("movej errcode:%d\n", err1);

        int err2 = robot.MoveL(&j2, &desc_pos2, tool, user, vel, acc, ovl, blendR, &epos,search,flag, &offset_pos);
        printf("movel errcode:%d\n", err2);   

        int err3 = robot.MoveC(&j3,&desc_pos3,tool,user,vel,acc,&epos,flag,&offset_pos,&j4,&desc_pos4,tool,user,vel,acc,&epos,flag,&offset_pos,ovl,blendR);
        printf("movec errcode:%d\n", err3); 

        int err4 = robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT,flag, &offset_pos);
        printf("movej errcode:%d\n", err4);

        int err5 = robot.Circle(&j3,&desc_pos3,tool,user,vel,acc,&epos,&j4,&desc_pos4,tool,user,vel,acc,&epos,ovl,flag,&offset_pos);
        printf("circle errcode:%d\n", err5);
        
        return 0;
    }

笛卡尔空间螺旋线运动
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  笛卡尔空间螺旋线运动
    * @param  [in] joint_pos  目标关节位置,单位deg
    * @param  [in] desc_pos   目标笛卡尔位姿
    * @param  [in] tool  工具坐标号，范围[1~15]
    * @param  [in] user  工件坐标号，范围[1~15]
    * @param  [in] vel  速度百分比，范围[0~100]
    * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
    * @param  [in] epos  扩展轴位置，单位mm
    * @param  [in] ovl  速度缩放因子，范围[0~100]    
    * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
    * @param  [in] offset_pos  位姿偏移量
    * @param  [in] spiral_param  螺旋参数
    * @return  错误码
    */
    errno_t  NewSpiral(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, ExaxisPos *epos, float ovl, uint8_t offset_flag, DescPose *offset_pos, SpiralParam spiral_param);  

代码示例
++++++++++++++
.. code-block:: c++
    :linenos:

    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    #include "FRRobot.h"
    #include "RobotTypes.h"

    using namespace std;

    int main(void)
    {
        FRRobot robot;                 //实例化机器人对象
        robot.RPC("192.168.58.2");     //与机器人控制器建立通信连接

        JointPos j;
        DescPose desc_pos, offset_pos1, offset_pos2;
        ExaxisPos  epos;
        SpiralParam sp;

        memset(&j, 0, sizeof(JointPos));
        memset(&desc_pos, 0, sizeof(DescPose));
        memset(&offset_pos1, 0, sizeof(DescPose));
        memset(&offset_pos2, 0, sizeof(DescPose));
        memset(&epos, 0, sizeof(ExaxisPos));
        memset(&sp, 0, sizeof(SpiralParam));

        j = {127.888,-101.535,-94.860,17.836,96.931,-61.325};
        offset_pos1.tran.x = 50.0;
        offset_pos1.rpy.rx = -30.0;
        offset_pos2.tran.x = 50.0;
        offset_pos2.rpy.rx = -5.0;

        sp.circle_num = 5;
        sp.circle_angle = 5.0;
        sp.rad_init = 50.0;
        sp.rad_add = 10.0;
        sp.rotaxis_add = 10.0;
        sp.rot_direction = 0;

        int tool = 0;
        int user = 0;
        float vel = 100.0;
        float acc = 100.0;
        float ovl = 100.0;
        float blendT = 0.0;
        uint8_t flag = 2;

        robot.SetSpeed(20);

        int ret = robot.GetForwardKin(&j, &desc_pos);  //只有关节位置的情况下，可用正运动学接口求解笛卡尔空间坐标

        if(ret == 0)
        {
            int err1 = robot.MoveJ(&j, &desc_pos, tool, user, vel, acc, ovl, &epos, blendT,flag, &offset_pos1);
            printf("movej errcode:%d\n", err1);

            int err2 = robot.NewSpiral(&j, &desc_pos, tool, user, vel, acc, &epos, ovl, flag, &offset_pos2, sp);
            printf("newspiral errcode:%d\n", err2);
        }
        else
        {
            printf("GetForwardKin errcode:%d\n", ret);
        }

        return 0;
    }

关节空间伺服模式运动
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  关节空间伺服模式运动
    * @param  [in] joint_pos  目标关节位置,单位deg
    * @param  [in] acc  加速度百分比，范围[0~100],暂不开放，默认为0
    * @param  [in] vel  速度百分比，范围[0~100]，暂不开放，默认为0
    * @param  [in] cmdT  指令下发周期，单位s，建议范围[0.001~0.0016]
    * @param  [in] filterT 滤波时间，单位s，暂不开放，默认为0
    * @param  [in] gain  目标位置的比例放大器，暂不开放，默认为0
    * @return  错误码
    */
    errno_t  ServoJ(JointPos *joint_pos, float acc, float vel, float cmdT, float filterT, float gain);

代码示例
++++++++++++++
.. code-block:: c++
    :linenos:

    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    #include "FRRobot.h"
    #include "RobotTypes.h"

    using namespace std;

    int main(void)
    {
        FRRobot robot;                 //实例化机器人对象
        robot.RPC("192.168.58.2");     //与机器人控制器建立通信连接

        JointPos j;

        memset(&j, 0, sizeof(JointPos));

        float vel = 0.0;
        float acc = 0.0;
        float cmdT = 0.008;
        float filterT = 0.0;
        float gain = 0.0;
        uint8_t flag = 0;
        int count = 500;
        float dt = 0.1;

        int ret = robot.GetActualJointPosDegree(flag, &j);
        if(ret == 0)
        {
            while (count)
            {
                robot.ServoJ(&j, acc, vel, cmdT, filterT, gain);
                j.jPos[0] += dt;
                count -= 1;
                robot.WaitMs(cmdT*1000);
            }
        }
        else
        {
            printf("GetActualJointPosDegree errcode:%d\n", ret);
        }

        return 0;
    }

笛卡尔空间伺服模式运动
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  笛卡尔空间伺服模式运动
    * @param  [in]  mode  0-绝对运动(基坐标系)，1-增量运动(基坐标系)，2-增量运动(工具坐标系)
    * @param  [in]  desc_pos  目标笛卡尔位姿或位姿增量
    * @param  [in]  pos_gain  位姿增量比例系数，仅在增量运动下生效，范围[0~1]
    * @param  [in] acc  加速度百分比，范围[0~100],暂不开放，默认为0
    * @param  [in] vel  速度百分比，范围[0~100]，暂不开放，默认为0
    * @param  [in] cmdT  指令下发周期，单位s，建议范围[0.001~0.0016]
    * @param  [in] filterT 滤波时间，单位s，暂不开放，默认为0
    * @param  [in] gain  目标位置的比例放大器，暂不开放，默认为0
    * @return  错误码
    */
    errno_t  ServoCart(int mode, DescPose *desc_pose, float pos_gain[6], float acc, float vel, float cmdT, float filterT, float gain);

代码示例
++++++++++++++
.. code-block:: c++
    :linenos:

    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    #include "FRRobot.h"
    #include "RobotTypes.h"

    using namespace std;

    int main(void)
    {
        FRRobot robot;                 //实例化机器人对象
        robot.RPC("192.168.58.2");     //与机器人控制器建立通信连接

        DescPose desc_pos_dt;
        memset(&desc_pos_dt, 0, sizeof(DescPose));

        desc_pos_dt.tran.z = -0.5;
        float pos_gain[6] = {0.0,0.0,1.0,0.0,0.0,0.0};
        int mode = 2;
        float vel = 0.0;
        float acc = 0.0;
        float cmdT = 0.008;
        float filterT = 0.0;
        float gain = 0.0;
        uint8_t flag = 0;
        int count = 100;

        robot.SetSpeed(20);

        while (count)
        {
            robot.ServoCart(mode, &desc_pos_dt, pos_gain, acc, vel, cmdT, filterT, gain);
            count -= 1;
            robot.WaitMs(cmdT*1000);
        }

        return 0;
    }

笛卡尔空间点到点运动
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  笛卡尔空间点到点运动
    * @param  [in]  desc_pos  目标笛卡尔位姿或位姿增量
    * @param  [in] tool  工具坐标号，范围[1~15]
    * @param  [in] user  工件坐标号，范围[1~15]
    * @param  [in] vel  速度百分比，范围[0~100]
    * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
    * @param  [in] ovl  速度缩放因子，范围[0~100]
    * @param  [in] blendT [-1.0]-运动到位(阻塞)，[0~500.0]-平滑时间(非阻塞)，单位ms 
    * @param  [in] config  关节空间配置，[-1]-参考当前关节位置解算，[0~7]-参考特定关节空间配置解算，默认为-1   
    * @return  错误码
    */
    errno_t  MoveCart(DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendT, int config);

代码示例
++++++++++++++
.. code-block:: c++
    :linenos:

    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    #include "FRRobot.h"
    #include "RobotTypes.h"

    using namespace std;

    int main(void)
    {
        FRRobot robot;                 //实例化机器人对象
        robot.RPC("192.168.58.2");     //与机器人控制器建立通信连接

        DescPose desc_pos1, desc_pos2, desc_pos3;
        memset(&desc_pos1, 0, sizeof(DescPose));
        memset(&desc_pos2, 0, sizeof(DescPose));
        memset(&desc_pos3, 0, sizeof(DescPose));

        desc_pos1.tran.x = 75.414;
        desc_pos1.tran.y = 568.526;
        desc_pos1.tran.z = 338.135;
        desc_pos1.rpy.rx = -178.348;
        desc_pos1.rpy.ry = -0.930;
        desc_pos1.rpy.rz = 52.611;

        desc_pos2.tran.x = -273.856;
        desc_pos2.tran.y = 643.260;
        desc_pos2.tran.z = 259.235;
        desc_pos2.rpy.rx = -177.972;
        desc_pos2.rpy.ry = -1.494;
        desc_pos2.rpy.rz = 80.866;

        desc_pos3.tran.x = -423.044;
        desc_pos3.tran.y = 229.703;
        desc_pos3.tran.z = 241.080;
        desc_pos3.rpy.rx = -173.990;
        desc_pos3.rpy.ry = -5.772;
        desc_pos3.rpy.rz = 123.971;

        int tool = 0;
        int user = 0;
        float vel = 100.0;
        float acc = 100.0;
        float ovl = 100.0;
        float blendT = -1.0;
        float blendT1 = 0.0;
        int config = -1;

        robot.SetSpeed(20);
        robot.MoveCart(&desc_pos1, tool, user, vel, acc, ovl, blendT, config);
        robot.MoveCart(&desc_pos2, tool, user, vel, acc, ovl, blendT, config);
        robot.MoveCart(&desc_pos3, tool, user, vel, acc, ovl, blendT1, config);

        return 0;
    }

样条运动开始
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  样条运动开始
    * @return  错误码
    */
    errno_t  SplineStart();

样条运动PTP
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  关节空间样条运动
    * @param  [in] joint_pos  目标关节位置,单位deg
    * @param  [in] desc_pos   目标笛卡尔位姿
    * @param  [in] tool  工具坐标号，范围[1~15]
    * @param  [in] user  工件坐标号，范围[1~15]
    * @param  [in] vel  速度百分比，范围[0~100]
    * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
    * @param  [in] ovl  速度缩放因子，范围[0~100]   
    * @return  错误码
    */
    errno_t  SplinePTP(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl);

样条运动结束
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  样条运动结束
    * @return  错误码
    */
    errno_t  SplineEnd();

代码示例
++++++++++++++
.. code-block:: c++
    :linenos:

    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    #include "FRRobot.h"
    #include "RobotTypes.h"

    using namespace std;

    int main(void)
    {
        FRRobot robot;                 //实例化机器人对象
        robot.RPC("192.168.58.2");     //与机器人控制器建立通信连接

        JointPos j1,j2,j3,j4;
        DescPose desc_pos1,desc_pos2,desc_pos3,desc_pos4,offset_pos;
        ExaxisPos  epos;

        memset(&j1, 0, sizeof(JointPos));
        memset(&j2, 0, sizeof(JointPos));
        memset(&j3, 0, sizeof(JointPos));
        memset(&j4, 0, sizeof(JointPos));
        memset(&desc_pos1, 0, sizeof(DescPose));
        memset(&desc_pos2, 0, sizeof(DescPose));
        memset(&desc_pos3, 0, sizeof(DescPose));
        memset(&desc_pos4, 0, sizeof(DescPose));
        memset(&offset_pos, 0, sizeof(DescPose));
        memset(&epos, 0, sizeof(ExaxisPos));

        j1 = {114.578,-117.798,-97.745,-54.436,90.053,-45.216};
        desc_pos1.tran.x = -140.418;
        desc_pos1.tran.y = 619.351;
        desc_pos1.tran.z = 198.369;
        desc_pos1.rpy.rx = -179.948;
        desc_pos1.rpy.ry = 0.023;
        desc_pos1.rpy.rz = 69.793;

        j2 = {115.401,-105.206,-117.959,-49.727,90.054,-45.222};
        desc_pos2.tran.x = -95.586;
        desc_pos2.tran.y = 504.143;
        desc_pos2.tran.z = 186.880;
        desc_pos2.rpy.rx = 178.001;
        desc_pos2.rpy.ry = 2.091;
        desc_pos2.rpy.rz = 70.585;

        j3 = {135.609,-103.249,-120.211,-49.715,90.058,-45.219};
        desc_pos3.tran.x = -252.429;
        desc_pos3.tran.y = 428.903;
        desc_pos3.tran.z = 188.492;
        desc_pos3.rpy.rx = 177.804;
        desc_pos3.rpy.ry = 2.294;
        desc_pos3.rpy.rz = 90.782;

        j4 = {154.766,-87.036,-135.672,-49.045,90.739,-45.223};
        desc_pos4.tran.x = -277.255;
        desc_pos4.tran.y = 272.958;
        desc_pos4.tran.z = 205.452;
        desc_pos4.rpy.rx = 179.289;
        desc_pos4.rpy.ry = 1.765;
        desc_pos4.rpy.rz = 109.966;   

        int tool = 0;
        int user = 0;
        float vel = 100.0;
        float acc = 100.0;
        float ovl = 100.0;
        float blendT = -1.0;
        uint8_t flag = 0;

        robot.SetSpeed(20);
        
        int err1 = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT,flag, &offset_pos);
        printf("movej errcode:%d\n", err1);
        robot.SplineStart();
        robot.SplinePTP(&j1, &desc_pos1, tool, user, vel, acc, ovl);
        robot.SplinePTP(&j2, &desc_pos2, tool, user, vel, acc, ovl);
        robot.SplinePTP(&j3, &desc_pos3, tool, user, vel, acc, ovl);
        robot.SplinePTP(&j4, &desc_pos4, tool, user, vel, acc, ovl);
        robot.SplineEnd();
        
        return 0;
    }

终止运动
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief 终止运动
    * @return  错误码
    */
    errno_t  StopMotion();

点位整体偏移开始
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  点位整体偏移开始
    * @param  [in]  flag  0-基坐标系下/工件坐标系下偏移，2-工具坐标系下偏移
    * @param  [in] offset_pos  位姿偏移量
    * @return  错误码
    */
    errno_t  PointsOffsetEnable(int flag, DescPose *offset_pos);

点位整体偏移结束
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  点位整体偏移结束
    * @return  错误码
    */
    errno_t  PointsOffsetDisable();

代码示例
++++++++++++++
.. code-block:: c++
    :linenos:

    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    #include "FRRobot.h"
    #include "RobotTypes.h"

    using namespace std;

    int main(void)
    {
        FRRobot robot;                 //实例化机器人对象
        robot.RPC("192.168.58.2");     //与机器人控制器建立通信连接

        JointPos j1,j2;
        DescPose desc_pos1,desc_pos2,offset_pos,offset_pos1;
        ExaxisPos  epos;

        memset(&j1, 0, sizeof(JointPos));
        memset(&j2, 0, sizeof(JointPos));
        memset(&desc_pos1, 0, sizeof(DescPose));
        memset(&desc_pos2, 0, sizeof(DescPose));
        memset(&offset_pos, 0, sizeof(DescPose));
        memset(&offset_pos1, 0, sizeof(DescPose));
        memset(&epos, 0, sizeof(ExaxisPos));

        j1 = {114.578,-117.798,-97.745,-54.436,90.053,-45.216};
        desc_pos1.tran.x = -140.418;
        desc_pos1.tran.y = 619.351;
        desc_pos1.tran.z = 198.369;
        desc_pos1.rpy.rx = -179.948;
        desc_pos1.rpy.ry = 0.023;
        desc_pos1.rpy.rz = 69.793;

        j2 = {115.401,-105.206,-117.959,-49.727,90.054,-45.222};
        desc_pos2.tran.x = -95.586;
        desc_pos2.tran.y = 504.143;
        desc_pos2.tran.z = 186.880;
        desc_pos2.rpy.rx = 178.001;
        desc_pos2.rpy.ry = 2.091;
        desc_pos2.rpy.rz = 70.585;

        offset_pos1.tran.x = 100.0;
        offset_pos1.tran.y = 100.0;
        offset_pos1.tran.z = 100.0;
        offset_pos1.rpy.rx = 5.0;
        offset_pos1.rpy.ry = 5.0;
        offset_pos1.rpy.rz = 5.0;    

        int tool = 0;
        int user = 0;
        float vel = 100.0;
        float acc = 100.0;
        float ovl = 100.0;
        float blendT = -1.0;
        float blendR = 0.0;
        uint8_t flag = 0;
        int type = 0;

        robot.SetSpeed(20);
        
        robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT,flag, &offset_pos);
        robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT,flag, &offset_pos);
        sleep(2);
        robot.PointsOffsetEnable(type, &offset_pos1);
        robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT,flag, &offset_pos);
        robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT,flag, &offset_pos);
        robot.PointsOffsetDisable();

        return 0;
    }
