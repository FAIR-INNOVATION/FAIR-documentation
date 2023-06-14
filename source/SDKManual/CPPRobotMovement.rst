Movement
=================

.. toctree:: 
    :maxdepth: 5


Jog point movement
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Jog point movement
    * @param  [in]  ref 0- node movement, 2- base coordinate system, 4- tool coordinate system, 8- workpiece coordinate system
    * @param  [in]  nb 1-joint 1(or axis x), 2-joint 2(or axis y), 3-joint 3(or axis z), 4-joint 4(or rotation about axis x), 5-joint 5(or rotation about axis y), 6-joint 6(or rotation about axis z)
    * @param  [in]  dir 0-negative correlation, 1-positive correlation
    * @param  [in]  vel The percentage of velocity,[0~100]
    * @param  [in]  acc The percentage of acceleration, [0~100]
    * @param  [in]  max_dis Maximum Angle of single click, unit: [°] or distance, unit: [mm]
    * @return  Error code
    */
    errno_t  StartJOG(uint8_t ref, uint8_t nb, uint8_t dir, float vel, float acc, float max_dis);

Jog point dynamic deceleration stop
++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Jog point dynamic deceleration stop
    * @param  [in]  ref  1- point stop, 3- point stop in base coordinate system, 5- point stop in tool coordinate system, 9- point stop in workpiece coordinate system
    * @return  Error code
    */
    errno_t  StopJOG(uint8_t ref);

The jog stops immediately
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief The jog stops immediately
    * @return  Error code
    */
    errno_t  ImmStopJOG(); 

Code example
+++++++++++++++
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

        robot.StartJOG(0,1,0,20.0,20.0,30.0);   //For single-joint motion, StartJOG is a non-blocking command. Receiving other motion commands (including StartJOG) while in motion is discarded
        sleep(1);
        //robot.StopJOG(1)  //Robot single axis point deceleration stop
        robot.ImmStopJOG();  //The single axis of the robot stops immediately
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

        robot.StartJOG(2,1,0,20.0,20.0,30.0);   //Point in the base coordinate system
        sleep(1);
        //robot.StopJOG(3)  //Robot single axis point deceleration stop
        robot.ImmStopJOG();  //The single axis of the robot stops immediately
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

        robot.StartJOG(4,1,0,20.0,20.0,30.0);   //Point in the tool coordinate system
        sleep(1);
        //robot.StopJOG(5)  //Robot single axis point deceleration stop
        robot.ImmStopJOG();  //The single axis of the robot stops immediately
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

        robot.StartJOG(8,1,0,20.0,20.0,30.0);   //Point in the workpiece coordinate system
        sleep(1);
        //robot.StopJOG(9)  //Robot single axis point deceleration stop
        robot.ImmStopJOG();  //The single axis of the robot stops immediately
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

Joint space motion
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Joint space motion
    * @param  [in] joint_pos  Target joint location, unit: deg
    * @param  [in] desc_pos   Target Cartesian position
    * @param  [in] tool  Tool coordinate number, range [1~15]
    * @param  [in] user  Workpiece coordinate number, range [1~15]
    * @param  [in] vel  Percentage of speed, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] ovl  Velocity scaling factor, range[0~100]
    * @param  [in] epos  Position of expansion shaft, unit: mm
    * @param  [in] blendT [-1.0]- movement in place (blocking), [0~500.0]- smoothing time (non-blocking), in ms
    * @param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos  The pose offset
    * @return  Error code
    */
    errno_t  MoveJ(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos *epos, float blendT, uint8_t offset_flag, DescPose *offset_pos);

Rectilinear motion in Cartesian space
++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Rectilinear motion in Cartesian space
    * @param  [in] joint_pos  Target joint location, unit: deg
    * @param  [in] desc_pos   Target Cartesian position
    * @param  [in] tool  Tool coordinate number, range [1~15]
    * @param  [in] user  Workpiece coordinate number, range [1~15]
    * @param  [in] vel  Percentage of speed, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] ovl  Velocity scaling factor, range[0~100]
    * @param  [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm    
    * @param  [in] epos  Position of expansion shaft, unit: mm
    * @param  [in] search  0- no wire seeking, 1- wire seeking
    * @param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos  The pose offset
    * @return  Error code
    */   
    errno_t  MoveL(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos);

Circular arc motion in Cartesian space
++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Circular arc motion in Cartesian space
    * @param  [in] joint_pos_p  Waypoint joint position, unit: deg
    * @param  [in] desc_pos_p   Waypoint Cartesian position
    * @param  [in] ptool  Tool coordinate number, range [1~15]
    * @param  [in] puser  Workpiece coordinate number, range [1~15]
    * @param  [in] pvel  Percentage of speed, range [0~100]
    * @param  [in] pacc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] epos_p  Position of expansion shaft, unit: mm
    * @param  [in] poffset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos_p  The pose offset
    * @param  [in] joint_pos_t  Target joint position, unit: deg
    * @param  [in] desc_pos_t   Target point Cartesian position
    * @param  [in] ttool  Tool coordinate number, range [1~15]
    * @param  [in] tuser  Workpiece coordinate number, range [1~15]
    * @param  [in] tvel  Percentage of speed, range [0~100]
    * @param  [in] tacc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] epos_t  Position of expansion shaft, unit: mm
    * @param  [in] toffset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos_t  The pose offset   
    * @param  [in] ovl  Velocity scaling factor, range[0~100]    
    * @param  [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm    
    * @return  Error code
    */      
    errno_t  MoveC(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, uint8_t poffset_flag, DescPose *offset_pos_p,JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, uint8_t toffset_flag, DescPose *offset_pos_t,float ovl, float blendR);

Circular motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Circular motion in Cartesian space
    * @param  [in] joint_pos_p  Path point 1 joint position, unit: deg
    * @param  [in] desc_pos_p   Waypoint 1 Cartesian position
    * @param  [in] ptool  Tool coordinate number, range [1~15]
    * @param  [in] puser  Workpiece coordinate number, range [1~15]
    * @param  [in] pvel  Percentage of speed, range [0~100]
    * @param  [in] pacc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] epos_p  Position of expansion shaft, unit: mm
    * @param  [in] joint_pos_t  Joint position at waypoint 2, unit: deg
    * @param  [in] desc_pos_t   Waypoint 2 Cartesian position
    * @param  [in] ttool  Tool coordinate number, range [1~15]
    * @param  [in] tuser  Workpiece coordinate number, range [1~15]
    * @param  [in] tvel  Percentage of speed, range [0~100]
    * @param  [in] tacc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] epos_t  Position of expansion shaft, unit: mm
    * @param  [in] ovl  Velocity scaling factor, range[0~100]   
    * @param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos  The pose offset     
    * @return  Error code
    */      
    errno_t  Circle(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, float ovl, uint8_t offset_flag, DescPose *offset_pos);

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

Spiral motion in Cartesian space
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Spiral motion in Cartesian space
    * @param  [in] joint_pos  Target joint location, unit: deg
    * @param  [in] desc_pos   Target Cartesian position
    * @param  [in] tool  Tool coordinate number, range [1~15]
    * @param  [in] user  Workpiece coordinate number, range [1~15]
    * @param  [in] vel  Percentage of speed, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] epos  Position of expansion shaft, unit: mm
    * @param  [in] ovl  Velocity scaling factor, range[0~100]    
    * @param  [in] offset_flag  0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos  The pose offset
    * @param  [in] spiral_param  Spiral parameter
    * @return  Error code
    */
    errno_t  NewSpiral(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, ExaxisPos *epos, float ovl, uint8_t offset_flag, DescPose *offset_pos, SpiralParam spiral_param);  

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

        int ret = robot.GetForwardKin(&j, &desc_pos);  //The forward kinematic interface can be used to solve Cartesian space coordinates with only joint positions

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

Joint space servo mode motion
+++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Joint space servo mode motion
    * @param  [in] joint_pos  Target joint location, unit: deg
    * @param  [in] acc  Acceleration percentage range[0~100], not open yet, default: 0
    * @param  [in] vel  The value ranges from 0 to 100. The value is not available. The default value is 0
    * @param  [in] cmdT Instruction delivery period, unit: s, recommended range [0.001~0.0016]
    * @param  [in] filterT Filtering time (unit: s), temporarily disabled. The default value is 0
    * @param  [in] gain  The proportional amplifier at the target position, not yet open, defaults to 0
    * @return  Error code
    */
    errno_t  ServoJ(JointPos *joint_pos, float acc, float vel, float cmdT, float filterT, float gain);

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

Cartesian space servo mode motion
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Cartesian space servo mode motion
    * @param  [in]  mode  0- absolute motion (base coordinates), 1- incremental motion (base coordinates), 2- incremental motion (tool coordinates)
    * @param  [in]  desc_pos  Target Cartesian pose or pose increment
    * @param  [in]  pos_gain  Proportional coefficient of pose increment, effective only for incremental motion, range [0~1]
    * @param  [in] acc  Acceleration percentage range[0~100], not open yet, default: 0
    * @param  [in] vel  The value ranges from 0 to 100. The value is not available. The default value is 0
    * @param  [in] cmdT Instruction delivery period, unit: s, recommended range [0.001~0.0016]
    * @param  [in] filterT Filtering time (unit: s), temporarily disabled. The default value is 0
    * @param  [in] gain  The proportional amplifier at the target position, not yet open, defaults to 0
    * @return  Error code
    */
    errno_t  ServoCart(int mode, DescPose *desc_pose, float pos_gain[6], float acc, float vel, float cmdT, float filterT, float gain);

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

Point to point motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Point to point motion in Cartesian space
    * @param  [in]  desc_pos  Target Cartesian pose or pose increment
    * @param  [in] tool  Tool coordinate number, range [1~15]
    * @param  [in] user  Workpiece coordinate number, range [1~15]
    * @param  [in] vel  Percentage of speed, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] ovl  Velocity scaling factor, range[0~100]
    * @param  [in] blendT [-1.0]- movement in place (blocking), [0~500.0]- smoothing time (non-blocking), in ms
    * @param  [in] config  Joint space configuration, [-1]- refer to the current joint position, [0~7]- refer to the specific joint space configuration, the default is -1 
    * @return  Error code
    */
    errno_t  MoveCart(DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendT, int config);

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

The spline motion begins
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  The spline motion begins
    * @return  Error code
    */
    errno_t  SplineStart();

Spline motion PTP
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Joint space spline movement
    * @param  [in] joint_pos  Target joint location, unit: deg
    * @param  [in] desc_pos   Target Cartesian position
    * @param  [in] tool  Tool coordinate number, range [1~15]
    * @param  [in] user  Workpiece coordinate number, range [1~15]
    * @param  [in] vel  Percentage of speed, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not open for now
    * @param  [in] ovl  Velocity scaling factor, range[0~100]   
    * @return  Error code
    */
    errno_t  SplinePTP(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl);

The spline movement ends
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  The spline movement is complete
    * @return  Error code
    */
    errno_t  SplineEnd();

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

Termination motion
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Termination motion
    * @return  Error code
    */
    errno_t  StopMotion();

The whole point shift begins
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  The whole point shift begins
    * @param  [in]  flag 0- offset in base coordinate system/workpiece coordinate system, 2- offset in tool coordinate system
    * @param  [in] offset_pos  The pose offset
    * @return  Error code
    */
    errno_t  PointsOffsetEnable(int flag, DescPose *offset_pos);

The whole point shift ends
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  The whole point shift ends
    * @return  Error code
    */
    errno_t  PointsOffsetDisable();

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
