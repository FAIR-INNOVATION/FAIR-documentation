机器人状态查询
===============

.. toctree:: 
    :maxdepth: 5

获取机器人安装角度
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取机器人安装角度
    * @param  [out] yangle 倾斜角
    * @param  [out] zangle 旋转角
    * @return  错误码
    */
    errno_t  GetRobotInstallAngle(float *yangle, float *zangle);

获取系统变量值
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取系统变量值
    * @param  [in] id 系统变量编号，范围[1~20]
    * @param  [out] value  系统变量值
    * @return  错误码
    */
    errno_t  GetSysVarValue(int id, float *value);

获取当前关节位置(角度)
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取当前关节位置(角度)
    * @param  [in] flag 0-阻塞，1-非阻塞
    * @param  [out] jPos 六个关节位置，单位deg
    * @return  错误码
    */
    errno_t  GetActualJointPosDegree(uint8_t flag, JointPos *jPos);

获取当前关节位置(弧度)
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取当前关节位置(弧度)
    * @param  [in] flag 0-阻塞，1-非阻塞
    * @param  [out] jPos 六个关节位置，单位rad
    * @return  错误码
    */   
    errno_t  GetActualJointPosRadian(uint8_t flag, JointPos *jPos);

获取当前工具位姿
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取当前工具位姿
    * @param  [in] flag  0-阻塞，1-非阻塞
    * @param  [out] desc_pos  工具位姿
    * @return  错误码
    */
    errno_t  GetActualTCPPose(uint8_t flag, DescPose *desc_pos);

获取当前工具坐标系编号
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取当前工具坐标系编号
    * @param  [in] flag  0-阻塞，1-非阻塞
    * @param  [out] id  工具坐标系编号
    * @return  错误码
    */
    errno_t  GetActualTCPNum(uint8_t flag, int *id);

获取当前工件坐标系编号
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取当前工件坐标系编号
    * @param  [in] flag  0-阻塞，1-非阻塞
    * @param  [out] id  工件坐标系编号
    * @return  错误码
    */
    errno_t  GetActualWObjNum(uint8_t flag, int *id);  

获取当前末端法兰位姿
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取当前末端法兰位姿
    * @param  [in] flag  0-阻塞，1-非阻塞
    * @param  [out] desc_pos  法兰位姿
    * @return  错误码
    */
    errno_t  GetActualToolFlangePose(uint8_t flag, DescPose *desc_pos);  

逆运动学求解
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  逆运动学求解
    * @param  [in] type 0-绝对位姿(基坐标系)，1-增量位姿(基坐标系)，2-增量位姿(工具坐标系)
    * @param  [in] desc_pos 笛卡尔位姿
    * @param  [in] config 关节空间配置，[-1]-参考当前关节位置解算，[0~7]-依据特定关节空间配置求解
    * @param  [out] joint_pos 关节位置
    * @return  错误码
    */
    errno_t  GetInverseKin(int type, DescPose *desc_pos, int config, JointPos *joint_pos);

逆运动学求解
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  逆运动学求解，参考指定关节位置求解
    * @param  [in] type 0-绝对位姿(基坐标系)，1-增量位姿(基坐标系)，2-增量位姿(工具坐标系)
    * @param  [in] desc_pos 笛卡尔位姿
    * @param  [in] joint_pos_ref 参考关节位置
    * @param  [out] joint_pos 关节位置
    * @return  错误码
    */   
    errno_t  GetInverseKinRef(int type, DescPose *desc_pos, JointPos *joint_pos_ref, JointPos *joint_pos);

逆运动学求解
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  逆运动学求解，参考指定关节位置判断是否有解
    * @param  [in] type 0-绝对位姿(基坐标系)，1-增量位姿(基坐标系)，2-增量位姿(工具坐标系)
    * @param  [in] desc_pos 笛卡尔位姿
    * @param  [in] joint_pos_ref 参考关节位置
    * @param  [out] result 0-无解，1-有解
    * @return  错误码
    */   
    errno_t  GetInverseKinHasSolution(int type, DescPose *desc_pos, JointPos *joint_pos_ref, uint8_t *result);

正运动学求解
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  正运动学求解
    * @param  [in] joint_pos 关节位置
    * @param  [out] desc_pos 笛卡尔位姿
    * @return  错误码
    */
    errno_t  GetForwardKin(JointPos *joint_pos, DescPose *desc_pos);

获取当前关节转矩
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief 获取当前关节转矩
    * @param  [in] flag 0-阻塞，1-非阻塞
    * @param  [out] torques 关节转矩
    * @return  错误码
    */
    errno_t  GetJointTorques(uint8_t flag, float torques[6]);

获取当前负载的重量
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取当前负载的重量
    * @param  [in] flag 0-阻塞，1-非阻塞
    * @param  [out] weight 负载重量，单位kg
    * @return  错误码
    */
    errno_t  GetTargetPayload(uint8_t flag, float *weight);

获取当前负载的质心
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取当前负载的质心
    * @param  [in] flag 0-阻塞，1-非阻塞
    * @param  [out] cog 负载质心，单位mm
    * @return  错误码
    */   
    errno_t  GetTargetPayloadCog(uint8_t flag, DescTran *cog);

获取当前工具坐标系
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取当前工具坐标系
    * @param  [in] flag 0-阻塞，1-非阻塞
    * @param  [out] desc_pos 工具坐标系位姿
    * @return  错误码
    */
    errno_t  GetTCPOffset(uint8_t flag, DescPose *desc_pos);

获取当前工件坐标系
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取当前工件坐标系
    * @param  [in] flag 0-阻塞，1-非阻塞
    * @param  [out] desc_pos 工件坐标系位姿
    * @return  错误码
    */   
    errno_t  GetWObjOffset(uint8_t flag, DescPose *desc_pos);

获取关节软限位角度
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取关节软限位角度
    * @param  [in] flag 0-阻塞，1-非阻塞    
    * @param  [out] negative  负限位角度，单位deg
    * @param  [out] positive  正限位角度，单位deg
    * @return  错误码
    */
    errno_t  GetJointSoftLimitDeg(uint8_t flag, float negative[6], float positive[6]);

获取系统时间
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取系统时间
    * @param  [out] t_ms 单位ms
    * @return  错误码
    */
    errno_t  GetSystemClock(float *t_ms);

获取机器人当前关节配置
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取机器人当前关节位置
    * @param  [out]  config  关节空间配置，范围[0~7]
    * @return  错误码
    */
    errno_t  GetRobotCurJointsConfig(int *config);

获取当前速度
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  获取机器人当前速度
    * @param  [out]  vel  速度，单位mm/s
    * @return  错误码
    */   
    errno_t  GetDefaultTransVel(float *vel);

查询机器人运动是否完成
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  查询机器人运动是否完成
    * @param  [out]  state  0-未完成，1-完成
    * @return  错误码
    */   
    errno_t  GetRobotMotionDone(uint8_t *state);

代码示例
+++++++++++
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

        float yangle, zangle;
        int flag = 0;
        JointPos j_deg, j_rad;
        DescPose tcp, flange, tcp_offset, wobj_offset;
        DescTran cog;
        int id;
        float torques[6] = {0.0};
        float weight;
        float neg_deg[6]={0.0},pos_deg[6]={0.0};
        float t_ms;
        int config;
        float vel;

        memset(&j_deg, 0, sizeof(JointPos));
        memset(&j_rad, 0, sizeof(JointPos));
        memset(&tcp, 0, sizeof(DescPose));
        memset(&flange, 0, sizeof(DescPose));
        memset(&tcp_offset, 0, sizeof(DescPose));
        memset(&wobj_offset, 0, sizeof(DescPose));
        memset(&cog, 0, sizeof(DescTran));

        robot.GetRobotInstallAngle(&yangle, &zangle);
        printf("yangle:%f,zangle:%f\n", yangle, zangle);

        robot.GetActualJointPosDegree(flag, &j_deg);
        printf("joint pos deg:%f,%f,%f,%f,%f,%f\n", j_deg.jPos[0],j_deg.jPos[1],j_deg.jPos[2],j_deg.jPos[3],j_deg.jPos[4],j_deg.jPos[5]);

        robot.GetActualJointPosRadian(flag, &j_rad);
        printf("joint pos rad:%f,%f,%f,%f,%f,%f\n", j_rad.jPos[0],j_rad.jPos[1],j_rad.jPos[2],j_rad.jPos[3],j_rad.jPos[4],j_rad.jPos[5]);   

        robot.GetActualTCPPose(flag, &tcp);
        printf("tcp pose:%f,%f,%f,%f,%f,%f\n", tcp.tran.x, tcp.tran.y, tcp.tran.z, tcp.rpy.rx, tcp.rpy.ry, tcp.rpy.rz); 

        robot.GetActualToolFlangePose(flag, &flange);
        printf("flange pose:%f,%f,%f,%f,%f,%f\n", flange.tran.x, flange.tran.y, flange.tran.z, flange.rpy.rx, flange.rpy.ry, flange.rpy.rz); 

        robot.GetActualTCPNum(flag, &id);
        printf("tcp num:%d\n", id);

        robot.GetActualWObjNum(flag, &id);
        printf("wobj num:%d\n", id);  

        robot.GetJointTorques(flag, torques);
        printf("torques:%f,%f,%f,%f,%f,%f\n", torques[0],torques[1],torques[2],torques[3],torques[4],torques[5]); 

        robot.GetTargetPayload(flag, &weight);
        printf("payload weight:%f\n", weight);

        robot.GetTargetPayloadCog(flag, &cog);
        printf("payload cog:%f,%f,%f\n",cog.x, cog.y, cog.z);

        robot.GetTCPOffset(flag, &tcp_offset);
        printf("tcp offset:%f,%f,%f,%f,%f,%f\n", tcp_offset.tran.x,tcp_offset.tran.y,tcp_offset.tran.z,tcp_offset.rpy.rx,tcp_offset.rpy.ry,tcp_offset.rpy.rz);

        robot.GetWObjOffset(flag, &wobj_offset);
        printf("wobj offset:%f,%f,%f,%f,%f,%f\n", wobj_offset.tran.x,wobj_offset.tran.y,wobj_offset.tran.z,wobj_offset.rpy.rx,wobj_offset.rpy.ry,wobj_offset.rpy.rz);

        robot.GetJointSoftLimitDeg(flag, neg_deg, pos_deg);
        printf("neg limit deg:%f,%f,%f,%f,%f,%f\n",neg_deg[0],neg_deg[1],neg_deg[2],neg_deg[3],neg_deg[4],neg_deg[5]);
        printf("pos limit deg:%f,%f,%f,%f,%f,%f\n",pos_deg[0],pos_deg[1],pos_deg[2],pos_deg[3],pos_deg[4],pos_deg[5]);

        robot.GetSystemClock(&t_ms);
        printf("system clock:%f\n", t_ms);

        robot.GetRobotCurJointsConfig(&config);
        printf("joint config:%d\n", config);

        robot.GetDefaultTransVel(&vel);
        printf("trans vel:%f\n", vel);

        return 0;
    }
