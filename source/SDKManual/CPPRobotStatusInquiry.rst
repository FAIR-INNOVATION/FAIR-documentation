Status query
=====================

.. toctree:: 
    :maxdepth: 5

Obtain robot mounting Angle
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Obtain robot mounting Angle
    * @param  [out] yangle Angle of inclination
    * @param  [out] zangle Angle of rotation
    * @return  Error code
    */
    errno_t  GetRobotInstallAngle(float *yangle, float *zangle);

Get the system variable value
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the system variable value
    * @param  [in] id System variable number, range[1~20]
    * @param  [out] value  System variable value
    * @return  Error code
    */
    errno_t  GetSysVarValue(int id, float *value);

Get the current joint position (Angle)
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the current joint position (Angle)
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] jPos Six joint positions, unit: deg
    * @return  Error code
    */
    errno_t  GetActualJointPosDegree(uint8_t flag, JointPos *jPos);

Get the current joint position (radians)
+++++++++++++++++++++++++++++++++++++++++++

.. deprecated:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the current joint position (radians)
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] jPos Six joint positions, unit: rad
    * @return  Error code
    */   
    errno_t  GetActualJointPosRadian(uint8_t flag, JointPos *jPos);

Get joint feedback speed
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get joint feedback speed-deg/s
     * @param  [in] flag 0-blocking, 1-non-blocking
     * @param  [out] speed Six joint speeds
     * @return  Error code 
     */ 
    errno_t  GetActualJointSpeedsDegree(uint8_t flag, float speed[6]);

Get joint feedback acceleration
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get joint feedback acceleration-deg/s^2
     * @param  [in] flag 0-blocking, 1-non-blocking
     * @param  [out] acc speed Six joint acceleration
     * @return  Error code 
     */ 
    errno_t  GetActualJointAccDegree(uint8_t flag, float acc[6]);   

Get TCP command speed
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get TCP command speed
     * @param  [in] flag 0-blocking, 1-non-blocking
     * @param  [out] tcp_speed Linear speed
     * @param  [out] ori_speed posture speed
     * @return  Error code 
     */
    errno_t  GetTargetTCPCompositeSpeed(uint8_t flag, float *tcp_speed, float *ori_speed);

Get TCP feedback speed
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get TCP feedback speed
     * @param  [in] flag 0-blocking, 1-non-blocking
     * @param  [out] tcp_speed Linear speed
     * @param  [out] ori_speed posture speed
     * @return  Error code  
     */ 
    errno_t  GetActualTCPCompositeSpeed(uint8_t flag, float *tcp_speed, float *ori_speed);

Get TCP command speed
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get TCP command speed
     * @param  [in] flag 0-blocking, 1-non-blocking
     * @param  [out] speed [x,y,z,rx,ry,rz]speed
     * @return  Error code  
     */ 
    errno_t  GetTargetTCPSpeed(uint8_t flag, float speed[6]);

Get TCP feedback speed
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get TCP feedback speed
     * @param  [in] flag 0-blocking, 1-non-blocking
     * @param  [out] speed [x,y,z,rx,ry,rz]speed
     * @return  Error code 
     */ 
    errno_t  GetActualTCPSpeed(uint8_t flag, float speed[6]);

Get the current tool pose
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the current tool pose
    * @param  [in] flag  0- blocking, 1- non-blocking
    * @param  [out] desc_pos  Tool position
    * @return  Error code
    */
    errno_t  GetActualTCPPose(uint8_t flag, DescPose *desc_pos);

Get the current tool coordinate system number
++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the current tool coordinate system number
    * @param  [in] flag  0- blocking, 1- non-blocking
    * @param  [out] id  Tool coordinate system number
    * @return  Error code
    */
    errno_t  GetActualTCPNum(uint8_t flag, int *id);

Get the current workpiece coordinate system number
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the current workpiece coordinate system number
    * @param  [in] flag  0- blocking, 1- non-blocking
    * @param  [out] id  Job coordinate system number
    * @return  Error code
    */
    errno_t  GetActualWObjNum(uint8_t flag, int *id);  

Get the current end flange pose
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the current end flange pose
    * @param  [in] flag  0- blocking, 1- non-blocking
    * @param  [out] desc_pos  Flange pose
    * @return  Error code
    */
    errno_t  GetActualToolFlangePose(uint8_t flag, DescPose *desc_pos);  

Inverse kinematics solution
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Inverse kinematics solution
    * @param  [in] type 0- absolute pose (base frame), 1- incremental pose (base frame), 2- incremental pose (tool frame)
    * @param  [in] desc_pos Cartesian pose
    * @param  [in] config Joint space configuration, [-1]- based on the current joint position, [0~7]- based on the specific joint space configuration
    * @param  [out] joint_pos Joint position
    * @return  Error code
    */
    errno_t  GetInverseKin(int type, DescPose *desc_pos, int config, JointPos *joint_pos);

Inverse kinematics solution
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Inverse kinematics is solved by referring to the specified joint position
    * @param  [in] type 0- absolute pose (base frame), 1- incremental pose (base frame), 2- incremental pose (tool frame)
    * @param  [in] desc_pos Cartesian pose
    * @param  [in] joint_pos_ref Reference joint position
    * @param  [out] joint_pos Joint position
    * @return  Error code
    */   
    errno_t  GetInverseKinRef(int type, DescPose *desc_pos, JointPos *joint_pos_ref, JointPos *joint_pos);

Inverse kinematics solution
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  To solve the inverse kinematics, refer to the specified joint position to determine whether there is a solution
    * @param  [in] type 0- absolute pose (base frame), 1- incremental pose (base frame), 2- incremental pose (tool frame)
    * @param  [in] desc_pos Cartesian pose
    * @param  [in] joint_pos_ref Reference joint position
    * @param  [out] result 0- no solution, 1-solution
    * @return  Error code
    */   
    errno_t  GetInverseKinHasSolution(int type, DescPose *desc_pos, JointPos *joint_pos_ref, uint8_t *result);

Forward kinematics solution
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Forward kinematics solution
    * @param  [in] joint_pos Joint position
    * @param  [out] desc_pos Cartesian pose
    * @return  Error code
    */
    errno_t  GetForwardKin(JointPos *joint_pos, DescPose *desc_pos);

Obtain the current joint torque
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Obtain the current joint torque
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] torques Joint torque
    * @return  Error code
    */
    errno_t  GetJointTorques(uint8_t flag, float torques[6]);

Get the weight of the current load
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Gets the weight of the current load
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] weight Load weight, unit: kg
    * @return  Error code
    */
    errno_t  GetTargetPayload(uint8_t flag, float *weight);

Get the center of mass of the current load
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the center of mass of the current load
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] cog Load center of mass, unit: mm
    * @return  Error code
    */   
    errno_t  GetTargetPayloadCog(uint8_t flag, DescTran *cog);

Get the current tool coordinate system
++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the current tool coordinate system
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] desc_pos Tool coordinate position
    * @return  Error code
    */
    errno_t  GetTCPOffset(uint8_t flag, DescPose *desc_pos);

Get the current work frame
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the current work frame
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] desc_pos Position of workpiece coordinate system
    * @return  Error code
    */   
    errno_t  GetWObjOffset(uint8_t flag, DescPose *desc_pos);

Obtain joint soft limit Angle
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Obtain joint soft limit Angle
    * @param  [in] flag 0- blocking, 1- non-blocking    
    * @param  [out] negative  Negative limit Angle, unit: deg
    * @param  [out] positive  Positive limit Angle, unit: deg
    * @return  Error code
    */
    errno_t  GetJointSoftLimitDeg(uint8_t flag, float negative[6], float positive[6]);

Get system time
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get system time
    * @param  [out] t_ms unit: ms
    * @return  Error code
    */
    errno_t  GetSystemClock(float *t_ms);

Get the current joint configuration of the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the current joint configuration of the robot
    * @param  [out]  config  Joint space configuration, range [0~7]
    * @return  Error code
    */
    errno_t  GetRobotCurJointsConfig(int *config);

Get current speed
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the robot's current speed
    * @param  [out]  vel  The unit is mm/s
    * @return  Error code
    */   
    errno_t  GetDefaultTransVel(float *vel);

Query whether the robot movement is complete
+++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Query whether the robot movement is complete
    * @param  [out]  state  0- Incomplete, 1- completed
    * @return  Error code
    */   
    errno_t  GetRobotMotionDone(uint8_t *state);

Query robot error code
+++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Query robot error code
     * @param  [out]  maincode  main error code 
     * @param  [out]  subcode   sub main code
     * @return  error code
     */ 
    errno_t  GetRobotErrorCode(int *maincode, int *subcode);

Query robot teaching and management point data
+++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Query robot teaching and management point data
     * @param  [in]  name  Point name
     * @param  [out]  data   point data
     * @return  error code
     */ 
    errno_t  GetRobotTeachingPoint(char name[64], float data[20]);

Query the robot motion queue cache length
+++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Query the robot motion queue cache length
     * @param  [out]  len  cache length
     * @return  error code
     */ 
    errno_t  GetMotionQueueLength(int *len);

Code example
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
        FRRobot robot;                 //Instantiate the robot object
        robot.RPC("192.168.58.2");     //Establish a communication connection with the robot controller

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
