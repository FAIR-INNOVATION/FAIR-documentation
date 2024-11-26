Force control
=======================

.. toctree:: 
    :maxdepth: 5

Force sensor configuration
+++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Configured force sensor
    * @param  [in] company  Manufacturer of force sensors, 17-Kunwei Technology
    * @param  [in] device  Device number, not used yet. The default value is 0
    * @param  [in] softvesion  Software version. The value is not used. The default value is 0
    * @param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    * @return  Error code
    */
    errno_t  FT_SetConfig(int company, int device, int softvesion, int bus);

Get the force sensor configuration
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the force sensor configuration
    * @param  [in] company  Force sensor manufacturer, to be determined
    * @param  [in] device  Device number, not used yet. The default value is 0
    * @param  [in] softvesion  Software version. The value is not used. The default value is 0
    * @param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    * @return  Error code
    */
    errno_t  FT_GetConfig(int *company, int *device, int *softvesion, int *bus);

Force sensor activation
+++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Force sensor activation
    * @param  [in] act  0- reset, 1- activate
    * @return  Error code
    */
    errno_t  FT_Activate(uint8_t act);

Force sensor calibration
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Force sensor calibration
    * @param  [in] act  0- zero removal, 1- zero correction
    * @return  Error code
    */
    errno_t  FT_SetZero(uint8_t act);   

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

        int company = 17;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int index = 1;
        int act = 0;

        robot.FT_SetConfig(company, device, softversion, bus);
        sleep(1);
        robot.FT_GetConfig(&company, &device, &softversion, &bus);
        printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
        sleep(1);

        robot.FT_Activate(act);
        sleep(1);
        act = 1;
        robot.FT_Activate(act);
        sleep(1);

        robot.SetLoadWeight(0.0);
        sleep(1);
        DescTran coord;
        memset(&coord, 0, sizeof(DescTran));
        robot.SetLoadCoord(&coord);
        sleep(1);
        robot.FT_SetZero(0);
        sleep(1);

        ForceTorque ft;
        memset(&ft, 0, sizeof(ForceTorque));
        robot.FT_GetForceTorqueOrigin(&ft);
        printf("ft origin:%f,%f,%f,%f,%f,%f\n", ft.fx,ft.fy,ft.fz,ft.tx,ft.ty,ft.tz);
        robot.FT_SetZero(1);
        sleep(1);
        memset(&ft, 0, sizeof(ForceTorque));
        printf("ft rcs:%f,%f,%f,%f,%f,%f\n",ft.fx,ft.fy,ft.fz,ft.tx,ft.ty,ft.tz);

        return 0;
    }

Set the reference coordinate system of the force sensor
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the reference coordinate system of the force sensor
    * @param  [in] ref  0- tool frame, 1- base frame
    * @return  Error code
    */
    errno_t  FT_SetRCS(uint8_t ref); 

Load weight identification record
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Load weight identification record
    * @param  [in] id  Sensor coordinate system number, range [1~14]
    * @return  Error code
    */
    errno_t  FT_PdIdenRecord(int id);   

Load weight identification calculation
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Load weight identification calculation
    * @param  [out] weight  Load weight, unit: kg
    * @return  Error code
    */   
    errno_t  FT_PdIdenCompute(float *weight);

Load centroid identification record
+++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Load centroid identification record
    * @param  [in] id  Sensor coordinate system number, range [1~14]
    * @param  [in] index Point number, range [1~3]
    * @return  Error code
    */
    errno_t  FT_PdCogIdenRecord(int id, int index);    

Load centroid identification calculation
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Load centroid identification calculation
    * @param  [out] cog  Load center of mass, unit: mm
    * @return  Error code
    */   
    errno_t  FT_PdCogIdenCompute(DescTran *cog); 

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

        float weight;

        DescPose tcoord, desc_p1, desc_p2, desc_p3;
        memset(&tcoord, 0, sizeof(DescPose));
        memset(&desc_p1, 0, sizeof(DescPose));
        memset(&desc_p2, 0, sizeof(DescPose));
        memset(&desc_p3, 0, sizeof(DescPose));

        robot.FT_SetRCS(0);
        sleep(1);

        tcoord.tran.z = 35.0;
        robot.SetToolCoord(10, &tcoord, 1, 0);
        sleep(1);
        robot.FT_PdIdenRecord(10);
        sleep(1);
        robot.FT_PdIdenCompute(&weight);
        printf("payload weight:%f\n", weight);

        desc_p1.tran.x = -160.619;
        desc_p1.tran.y = -586.138;
        desc_p1.tran.z = 384.988;
        desc_p1.rpy.rx = -170.166;
        desc_p1.rpy.ry = -44.782;
        desc_p1.rpy.rz = 169.295;

        desc_p2.tran.x = -87.615;
        desc_p2.tran.y = -606.209;
        desc_p2.tran.z = 556.119;
        desc_p2.rpy.rx = -102.495;
        desc_p2.rpy.ry = 10.118;
        desc_p2.rpy.rz = 178.985;

        desc_p3.tran.x = 41.479;
        desc_p3.tran.y = -557.243;
        desc_p3.tran.z = 484.407;
        desc_p3.rpy.rx = -125.174;
        desc_p3.rpy.ry = 46.995;
        desc_p3.rpy.rz = -132.165;

        robot.MoveCart(&desc_p1, 9, 0, 100.0, 100.0, 100.0, -1.0, -1);
        sleep(1);
        robot.FT_PdCogIdenRecord(10, 1);
        robot.MoveCart(&desc_p2, 9, 0, 100.0, 100.0, 100.0, -1.0, -1);
        sleep(1);
        robot.FT_PdCogIdenRecord(10, 2);
        robot.MoveCart(&desc_p3, 9, 0, 100.0, 100.0, 100.0, -1.0, -1);
        sleep(1);
        robot.FT_PdCogIdenRecord(10, 3);
        sleep(1);
        DescTran cog;
        memset(&cog, 0, sizeof(DescTran));
        robot.FT_PdCogIdenCompute(&cog);
        printf("cog:%f,%f,%f\n",cog.x, cog.y, cog.z);

        return 0;
    }

Obtain force/torque data in the reference coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Obtain force/torque data in the reference coordinate system
    * @param  [out] ft  Force/torque，fx,fy,fz,tx,ty,tz
    * @return  Error code
    */   
    errno_t  FT_GetForceTorqueRCS(ForceTorque *ft); 

Obtain the raw force/torque data of the force sensor
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Obtain the raw force/torque data of the force sensor
    * @param  [out] ft  Force/torque，fx,fy,fz,tx,ty,tz
    * @return  Error code
    */   
    errno_t  FT_GetForceTorqueOrigin(ForceTorque *ft); 

Collision guard
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Collision guard
    * @param  [in] flag 0- Disable collision guard. 1- Enable collision guard
    * @param  [in] sensor_id Force sensor number
    * @param  [in] select  Select the six degrees of freedom whether to detect collision, 0- no detection, 1- detection
    * @param  [in] ft  Impact force/torque，fx,fy,fz,tx,ty,tz
    * @param  [in] max_threshold Maximum threshold
    * @param  [in] min_threshold Minimum threshold
    * @note   Force/torque detection range:(ft-min_threshold, ft+max_threshold)
    * @return  Error code
    */   
    errno_t  FT_Guard(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float max_threshold[6], float min_threshold[6]); 

Code Example
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

        uint8_t flag = 1;
        uint8_t sensor_id = 1;
        uint8_t select[6] = {1,1,1,1,1,1};
        float max_threshold[6] = {10.0,10.0,10.0,10.0,10.0,10.0};
        float min_threshold[6] = {5.0,5.0,5.0,5.0,5.0,5.0};

        ForceTorque ft;
        DescPose desc_p1, desc_p2, desc_p3;
        memset(&ft, 0, sizeof(ForceTorque));
        memset(&desc_p1, 0, sizeof(DescPose));
        memset(&desc_p2, 0, sizeof(DescPose));
        memset(&desc_p3, 0, sizeof(DescPose));

        desc_p1.tran.x = -160.619;
        desc_p1.tran.y = -586.138;
        desc_p1.tran.z = 384.988;
        desc_p1.rpy.rx = -170.166;
        desc_p1.rpy.ry = -44.782;
        desc_p1.rpy.rz = 169.295;

        desc_p2.tran.x = -87.615;
        desc_p2.tran.y = -606.209;
        desc_p2.tran.z = 556.119;
        desc_p2.rpy.rx = -102.495;
        desc_p2.rpy.ry = 10.118;
        desc_p2.rpy.rz = 178.985;

        desc_p3.tran.x = 41.479;
        desc_p3.tran.y = -557.243;
        desc_p3.tran.z = 484.407;
        desc_p3.rpy.rx = -125.174;
        desc_p3.rpy.ry = 46.995;
        desc_p3.rpy.rz = -132.165;

        robot.FT_Guard(flag, sensor_id, select, &ft, max_threshold, min_threshold);
        robot.MoveCart(&desc_p1,9,0,100.0,100.0,100.0,-1.0,-1);
        robot.MoveCart(&desc_p2,9,0,100.0,100.0,100.0,-1.0,-1);
        robot.MoveCart(&desc_p3,9,0,100.0,100.0,100.0,-1.0,-1);
        flag = 0;
        robot.FT_Guard(flag, sensor_id, select, &ft, max_threshold, min_threshold);

        return 0;
    }

Constant force control
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Constant force control
    * @param  [in] flag 0- turn off constant force control, 1- turn on constant force control
    * @param  [in] sensor_id Force sensor number
    * @param  [in] select  Select the six degrees of freedom whether to detect collision, 0- no detection, 1- detection
    * @param  [in] ft  Impact force/torque，fx,fy,fz,tx,ty,tz
    * @param  [in] ft_pid Force pid parameter, torque pid parameter
    * @param  [in] adj_sign Adaptive start-stop control, 0- off, 1- on
    * @param  [in] ILC_sign ILC start stop control, 0- stop, 1- training, 2- operation
    * @param  [in] Maximum Adjustment distance, unit: mm
    * @param  [in] Maximum Adjustment Angle, unit: deg
    * @return  Error code
    */   
    errno_t  FT_Control(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float ft_pid[6], uint8_t adj_sign, uint8_t ILC_sign, float max_dis, float max_ang);   

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

        uint8_t flag = 1;
        uint8_t sensor_id = 1;
        uint8_t select[6] = {0,0,1,0,0,0};
        float ft_pid[6] = {0.0005,0.0,0.0,0.0,0.0,0.0};
        uint8_t adj_sign = 0;
        uint8_t ILC_sign = 0;
        float max_dis = 100.0;
        float max_ang = 0.0;

        ForceTorque ft;
        DescPose desc_p1, desc_p2, offset_pos;
        JointPos j1,j2;
        ExaxisPos epos;
        memset(&ft, 0, sizeof(ForceTorque));
        memset(&desc_p1, 0, sizeof(DescPose));
        memset(&desc_p2, 0, sizeof(DescPose));
        memset(&offset_pos, 0, sizeof(DescPose));
        memset(&epos, 0, sizeof(ExaxisPos));
        memset(&j1, 0, sizeof(JointPos));
        memset(&j2, 0, sizeof(JointPos));

        j1 = {-68.987,-96.414,-111.45,-61.105,92.884,11.089};
        j2 = {-107.596,-109.154,-104.735,-56.176,90.739,11.091};

        desc_p1.tran.x = 62.795;
        desc_p1.tran.y = -511.979;
        desc_p1.tran.z = 291.697;
        desc_p1.rpy.rx = -179.545;
        desc_p1.rpy.ry = 3.027;
        desc_p1.rpy.rz = -170.039;

        desc_p2.tran.x = -294.768;
        desc_p2.tran.y = -503.708;
        desc_p2.tran.z = 233.158;
        desc_p2.rpy.rx = 179.799;
        desc_p2.rpy.ry = 0.713;
        desc_p2.rpy.rz = 151.309;

        ft.fz = -10.0;

        robot.MoveJ(&j1,&desc_p1,9,0,100.0,180.0,100.0,&epos,-1.0,0,&offset_pos);
        robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);
        robot.MoveL(&j2,&desc_p2,9,0,100.0,180.0,20.0,-1.0,&epos,0,0,&offset_pos);
        flag = 0;
        robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);

        return 0;
    }

Spiral exploration
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Spiral exploration
    * @param  [in] rcs Reference frame, 0- tool frame, 1- base frame
    * @param  [in] dr Feed per circle radius
    * @param  [in] ft Force/torque threshold，fx,fy,fz,tx,ty,tz，range[0~100]
    * @param  [in] max_t_ms Maximum exploration time, unit: ms
    * @param  [in] max_vel Maximum linear velocity, unit: mm/s
    * @return  Error code
    */   
    errno_t  FT_SpiralSearch(int rcs, float dr, float ft, float max_t_ms, float max_vel);  

Rotary insertion
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Rotary insertion
    * @param  [in] rcs Reference frame, 0- tool frame, 1- base frame
    * @param  [in] angVelRot Angular velocity of rotation, unit: deg/s
    * @param  [in] ft  Force/torque threshold，fx,fy,fz,tx,ty,tz，range[0~100]
    * @param  [in] max_angle Maximum rotation Angle, unit: deg
    * @param  [in] orn Force/torque direction, 1- along the z axis, 2- around the z axis
    * @param  [in] max_angAcc Maximum rotational acceleration, in deg/s^2, not used yet, default is 0
    * @param  [in] rotorn  Rotation direction, 1- clockwise, 2- counterclockwise
    * @return  Error code
    */   
    errno_t  FT_RotInsertion(int rcs, float angVelRot, float ft, float max_angle, uint8_t orn, float max_angAcc, uint8_t rotorn);    

Linear insertion
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Linear insertion
    * @param  [in] rcs Reference frame, 0- tool frame, 1- base frame
    * @param  [in] ft  Force/torque threshold，fx,fy,fz,tx,ty,tz，range[0~100]
    * @param  [in] lin_v Linear velocity, unit: mm/s
    * @param  [in] lin_a Linear acceleration, unit: mm/s^2, not used yet
    * @param  [in] max_dis Maximum insertion distance, unit: mm
    * @param  [in] linorn  Insert direction, 0- negative, 1- positive
    * @return  Error code
    */   
    errno_t  FT_LinInsertion(int rcs, float ft, float lin_v, float lin_a, float max_dis, uint8_t linorn);    

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

        //Constant force parameter
        uint8_t status = 1;  //Constant force control open sign, 0- off, 1- on
        int sensor_num = 1; //Force sensor number
        float gain[6] = {0.0001,0.0,0.0,0.0,0.0,0.0};  //Maximum threshold
        uint8_t adj_sign = 0;  //Adaptive start-stop state, 0- off, 1- on
        uint8_t ILC_sign = 0;  //ILC control start stop state, 0- stop, 1- training, 2- real operation
        float max_dis = 100.0;  //Maximum adjustment distance
        float max_ang = 5.0;  //Maximum adjustment Angle

        ForceTorque ft;
        memset(&ft, 0, sizeof(ForceTorque));

        //Helix explore parameters
        int rcs = 0;  //Reference frame, 0- tool frame, 1- base frame
        float dr = 0.7;  //Radius feed per turn, unit: mm
        float fFinish = 1.0; //Force or torque threshold (0 to 100), unit: N or Nm
        float t = 60000.0; //Maximum exploration time, unit: ms
        float vmax = 3.0; //The maximum linear velocity, unit: mm/s

        //Linear insertion parameter
        float force_goal = 20.0;  //Force or torque threshold (0 to 100), unit: N or Nm
        float lin_v = 0.0; //Linear velocity, unit: mm/s
        float lin_a = 0.0; //Linear acceleration, unit: mm/s^2, not used yet
        float disMax = 100.0; //Maximum insertion distance, in mm
        uint8_t linorn = 1; //Insert direction, 1- positive, 2- negative

        //Rotational insertion parameter
        float angVelRot = 2.0;  //Angular velocity of rotation, in °/s
        float forceInsertion = 1.0; //Force or torque threshold (0 to 100), in N or Nm
        int angleMax= 45; //Maximum rotation Angle, unit: °
        uint8_t orn = 1; //Direction of force，1-fz,2-mz
        float angAccmax = 0.0; //Maximum angular acceleration of rotation, unit: °/s^2, not in use
        uint8_t rotorn = 1; //Rotation direction, 1- clockwise, 2- counterclockwise

        uint8_t select1[6] = {0,0,1,1,1,0}; //Six degrees of freedom options [fx,fy,fz,mx,my,mz], 0- does not work, 1- works
        ft.fz = -10.0;
        robot.FT_Control(status,sensor_num,select1,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
        robot.FT_SpiralSearch(rcs,dr,fFinish,t,vmax);
        status = 0;
        robot.FT_Control(status,sensor_num,select1,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

        uint8_t select2[6] = {1,1,1,0,0,0};  //Six degrees of freedom options [fx,fy,fz,mx,my,mz], 0- does not work, 1- works
        gain[0] = 0.00005;
        ft.fz = -30.0;
        status = 1;
        robot.FT_Control(status,sensor_num,select2,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
        robot.FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn);
        status = 0;
        robot.FT_Control(status,sensor_num,select2,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

        uint8_t select3[6] = {0,0,1,1,1,0};  //Six degrees of freedom options [fx,fy,fz,mx,my,mz], 0- does not work, 1- works
        ft.fz = -10.0;
        gain[0] = 0.0001;
        status = 1;
        robot.FT_Control(status,sensor_num,select3,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
        robot.FT_RotInsertion(rcs,angVelRot,forceInsertion,angleMax,orn,angAccmax,rotorn);
        status = 0;
        robot.FT_Control(status,sensor_num,select3,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

        uint8_t select4[6] = {1,1,1,0,0,0};  //Six degrees of freedom options [fx,fy,fz,mx,my,mz], 0- does not work, 1- works
        ft.fz = -30.0;
        status = 1;
        robot.FT_Control(status,sensor_num,select4,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
        robot.FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn);
        status = 0;
        robot.FT_Control(status,sensor_num,select4,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

        return 0;
    }

Surface positioning
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Surface positioning
    * @param  [in] rcs Reference frame, 0- tool frame, 1- base frame
    * @param  [in] dir  The direction of travel, 1- positive, 2- negative
    * @param  [in] axis Axis of movement, 1-x axis, 2-y axis, 3-z axis
    * @param  [in] lin_v Explore the linear velocity in mm/s
    * @param  [in] lin_a Explore linear acceleration, in mm/s^2, not used yet, default to 0
    * @param  [in] max_dis Maximum exploration distance, in mm
    * @param  [in] ft  Action termination force/torque threshold，fx,fy,fz,tx,ty,tz  
    * @return  Error code
    */   
    errno_t  FT_FindSurface(int rcs, uint8_t dir, uint8_t axis, float lin_v, float lin_a, float max_dis, float ft);   

Calculation of midplane position starts
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Calculation of midplane position starts
    * @return  Error code
    */   
    errno_t  FT_CalCenterStart();

Calculation of midplane position ends
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Calculation of midplane position ends
    * @param  [out] pos Intermediate plane position
    * @return  Error code
    */      
    errno_t  FT_CalCenterEnd(DescPose *pos);

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

        int rcs = 0;
        uint8_t dir = 1;
        uint8_t axis = 1;
        float lin_v = 3.0;
        float lin_a = 0.0;
        float maxdis = 50.0;
        float ft_goal = 2.0;

        DescPose desc_pos, xcenter, ycenter;
        ForceTorque ft;
        memset(&desc_pos, 0, sizeof(DescPose));
        memset(&xcenter, 0, sizeof(DescPose));
        memset(&ycenter, 0, sizeof(DescPose));
        memset(&ft, 0, sizeof(ForceTorque));

        desc_pos.tran.x = -230.959;
        desc_pos.tran.y = -364.017;
        desc_pos.tran.z = 217.5;
        desc_pos.rpy.rx = -179.004;
        desc_pos.rpy.ry = 0.002;
        desc_pos.rpy.rz = 89.999;

        ft.fx = -2.0;

        robot.MoveCart(&desc_pos, 9,0,100.0,100.0,100.0,-1.0,-1);

        robot.FT_CalCenterStart();
        robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
        robot.MoveCart(&desc_pos, 9,0,100.0,100.0,100.0,-1.0,-1);
        robot.WaitMs(1000);

        dir = 2;
        robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
        robot.FT_CalCenterEnd(&xcenter);
        printf("xcenter:%f,%f,%f,%f,%f,%f\n",xcenter.tran.x,xcenter.tran.y,xcenter.tran.z,xcenter.rpy.rx,xcenter.rpy.ry,xcenter.rpy.rz);
        robot.MoveCart(&xcenter, 9,0,60.0,50.0,50.0,-1.0,-1);

        robot.FT_CalCenterStart();
        dir = 1;
        axis = 2;
        lin_v = 6.0;
        maxdis = 150.0;
        robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
        robot.MoveCart(&desc_pos, 9,0,100.0,100.0,100.0,-1.0,-1);
        robot.WaitMs(1000);

        dir = 2;
        robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);  
        robot.FT_CalCenterEnd(&ycenter);
        printf("ycenter:%f,%f,%f,%f,%f,%f\n",ycenter.tran.x,ycenter.tran.y,ycenter.tran.z,ycenter.rpy.rx,ycenter.rpy.ry,ycenter.rpy.rz);
        robot.MoveCart(&ycenter, 9,0,60.0,50.0,50.0,0.0,-1);   

        return 0;
    }

Compliant control on
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Compliant control on
    * @param  [in] p Coefficient of position adjustment or compliance
    * @param  [in] force Compliant opening force threshold, unit: N
    * @return  Error code
    */   
    errno_t  FT_ComplianceStart(float p, float force); 

Compliant control off
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Compliant control off
    * @return  Error code
    */   
    errno_t  FT_ComplianceStop(); 

Load identification initialization
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Load identification initialization
     * @return Error code 
     */
    errno_t LoadIdentifyDynFilterInit();

Load identification initialization
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Load identification initialization
     * @return Error code 
     */
    errno_t LoadIdentifyDynVarInit();

Load identification main program
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief load identification main program
     * @param [in] joint_torque joint torque
     * @param [in] joint_pos joint position
     * @param [in] t sampling period
     * @return Error code 
     */
    errno_t LoadIdentifyMain(double joint_torque[6], double joint_pos[6], double t);

Get load identification results
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get load identification results
     * @param [in] gain  
     * @param [out] weight load weight
     * @param [out] cog cog load center of mass
     * @return Error code 
     */
    errno_t LoadIdentifyGetResult(double gain[12], double *weight, DescTran *cog);

The transmission belt starts and stops
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief The transmission belt starts and stops
     * @param [in] status status, 1-start, 0-stop
     * @return Error code 
     */
    errno_t ConveyorStartEnd(uint8_t status);

Records IO detection points
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief records IO detection points
     * @return Error code 
     */
    errno_t ConveyorPointIORecord();

Records point A
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Record point A
     * @return Error code 
     */
    errno_t ConveyorPointARecord();

Records reference point
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief record reference point
     * @return Error code 
     */
    errno_t ConveyorRefPointRecord();

Records point B
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Record point B
     * @return Error code 
     */
    errno_t ConveyorPointBRecord();

Conveyor belt workpiece IO detection
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Conveyor belt workpiece IO detection
     * @param [in] max_t maximum detection time, unit ms
     * @return Error code 
     */
    errno_t ConveyorIODetect(int max_t);

Get the current position of the object
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get the current position of the object
     * @param [in] mode 
     * @return Error code 
     */
    errno_t ConveyorGetTrackData(int mode);

Belt tracking starts
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Belt tracking starts
     * @param [in] status status , 1-start, 0-stop
     * @return Error code 
     */
    errno_t ConveyorTrackStart(uint8_t status);

Drive belt tracking stopped
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Drive belt tracking stopped
     * @return Error code 
     */
    errno_t ConveyorTrackEnd();

Transmission belt parameter configuration
+++++++++++++++++++++++++++++++++++++++++++++

.. versionchanged:: C++ SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    /**
     * @brief transmission belt parameter configuration
     * @param [in] para[0] Encoder channel 1~2
     * @param [in] para[1] Number of pulses for one revolution of the encoder
     * @param [in] para[2] The distance traveled by the conveyor belt in one revolution of the encoder
     * @param [in] para[3] Workpiece coordinate system number. Select the workpiece coordinate system number for the tracking motion function. Set tracking grab and TPD tracking to 0.
     * @param [in] para[4] Whether it is suitable for vision 0 Not suitable 1 Yes
     * @param [in] para[5] Speed ​​ratio for conveyor belt tracking capture options (1-100) Other options default to 1
     * @return error code
     */
    errno_t ConveyorSetParam(float param[5]);

Grab point compensation
+++++++++++++++++++++++++++++++++++++++++++++

.. versionchanged:: C++ SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    /**
     * @brief belt grab point compensation
     * @param [in] cmp compensation position double[3]{x, y, z}
     * @return error code
     */
    errno_t ConveyorCatchPointComp(double cmp[3]);

Linear motion
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief linear motion
     * @param [in] status status, 1-start, 0-stop
     * @return Error code 
     */
    errno_t TrackMoveL(char name[32], int tool, int wobj, float vel, float acc, float ovl, float blendR, uint8_t flag, uint8_t type);

Get SSH public key
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get SSH public key
     * @param [out] keygen public key
     * @return Error code 
     */
    errno_t GetSSHKeygen(char keygen[1024]);

Issues SCP instructions
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief issues SCP instructions
     * @param [in] mode 0-upload (host computer->controller), 1-download (controller->host computer)
     * @param [in] sshname host computer user name
     * @param [in] sship host computer ip address
     * @param [in] usr_file_url host computer file path
     * @param [in] robot_file_url robot controller file path
     * @return Error code 
     */
    errno_t SetSSHScpCmd(int mode, char sshname[32], char sship[32], char usr_file_url[128], char robot_file_url[128]);

Calculate the MD5 value of the file under the specified path
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Calculate the MD5 value of the file under the specified path
     * @param [in] file_path The file path contains the file name. The default Traj folder path is: "/fruser/traj/", such as "/fruser/traj/trajHelix_aima_1.txt"
     * @param [out] md5 file MD5 value
     * @return Error code 
     */
    errno_t ComputeFileMD5(char file_path[256], char md5[256]);

Get the emergency stop status of the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get the emergency stop status of the robot
     * @param [out] state emergency stop status, 0-non-emergency stop, 1-emergency stop
     * @return Error code   
     */
    errno_t GetRobotEmergencyStopState(uint8_t *state);

Get the communication status between SDK and robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get the communication status between SDK and robot
     * @param [out]  state communication status, 0-communication is normal, 1-communication is abnormal
     */
    errno_t GetSDKComState(int *state);

Get the safe stop signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get the safe stop signal
     * @param [out]  si0_state safety stop signal SI0, 0-invalid, 1-valid
     * @param [out]  si1_state safety stop signal SI1, 0-invalid, 1-valid
     */
    errno_t GetSafetyStopState(uint8_t *si0_state, uint8_t *si1_state);

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

        uint8_t flag = 1;
        int sensor_id = 1;
        uint8_t select[6] = {1,1,1,0,0,0};
        float ft_pid[6] = {0.0005,0.0,0.0,0.0,0.0,0.0};
        uint8_t adj_sign = 0;
        uint8_t ILC_sign = 0;
        float max_dis = 100.0;
        float max_ang = 0.0;

        ForceTorque ft;
        DescPose desc_p1, desc_p2, offset_pos;
        ExaxisPos epos;
        JointPos j1, j2;
        memset(&ft, 0, sizeof(ForceTorque));
        memset(&desc_p1, 0, sizeof(DescPose));
        memset(&desc_p2, 0, sizeof(DescPose));
        memset(&offset_pos, 0, sizeof(DescPose));
        memset(&j1, 0, sizeof(JointPos));
        memset(&j2, 0, sizeof(JointPos));
        memset(&epos, 0, sizeof(ExaxisPos));

        j1 = {-105.3,-68.0,-127.9,-75.5,90.8,77.8};
        j2 = {-105.3,-97.9,-101.5,-70.3,90.8,77.8};

        desc_p1.tran.x = -208.9;
        desc_p1.tran.y = -274.5;
        desc_p1.tran.z = 334.6;
        desc_p1.rpy.rx = 178.8;
        desc_p1.rpy.ry = -1.3;
        desc_p1.rpy.rz = 86.7;

        desc_p2.tran.x = -264.8;
        desc_p2.tran.y = -480.5;
        desc_p2.tran.z = 341.8;
        desc_p2.rpy.rx = 179.2;
        desc_p2.rpy.ry = 0.3;
        desc_p2.rpy.rz = 86.7;

        ft.fx = -10.0;
        ft.fy = -10.0;
        ft.fz = -10.0;
        robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);  
        float p = 0.00005;
        float force = 30.0; 
        robot.FT_ComplianceStart(p, force); 
        int count = 15;
        while (count)
        {
            robot.MoveL(&j1,&desc_p1,9,0,100.0,180.0,100.0,-1.0,&epos,0,1,&offset_pos);
            robot.MoveL(&j2,&desc_p2,9,0,100.0,180.0,100.0,-1.0,&epos,0,0,&offset_pos);
            count -= 1;
        }
        robot.FT_ComplianceStop();
        flag = 0;
        robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);

        return 0;
    }

Code example
+++++++++++++++

.. versionadded:: C++ SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    #include "libfairino/robot.h"

    //If using Windows, include the following header files
    #include <string.h>
    #include <windows.h>
    //If using Linux, include the following header files
    /*
    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    */
    #include <chrono>
    #include <thread>
    #include <string>

    using namespace std;


    int main(void)
    {
        FRRobot robot;         
        robot.RPC("192.168.58.2"); 

        int retval = 0;

        retval = robot.LoadIdentifyDynFilterInit();
        printf("LoadIdentifyDynFilterInit retval is: %d \n", retval);

        retval = robot.LoadIdentifyDynVarInit();
        printf("LoadIdentifyDynVarInit retval is: %d \n", retval);

        double joint_toq[6] = {0};
        double joint_pos[6] = {0};
        retval = robot.LoadIdentifyMain(joint_toq, joint_pos,1);
        printf("LoadIdentifyMain retval is: %d \n", retval);

        double gain[12] = {0};
        double weight = 0;
        DescTran load_pos;
        memset(&load_pos, 0, sizeof(DescTran));
        retval = robot.LoadIdentifyGetResult(gain, &weight, &load_pos);
        printf("LoadIdentifyGetResult retval is: %d \n", retval);
        printf("weight is: %f, load pose is: %f, %f, %f\n", weight, load_pos.x, load_pos.y, load_pos.z);

        retval = robot.WaitMs(10);
        printf("WaitMs retval is: %d \n", retval);
    }

Code example
+++++++++++++++

.. versionadded:: C++ SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    #include "libfairino/robot.h"

    //If using Windows, include the following header files
    #include <string.h>
    #include <windows.h>
    //If using Linux, include the following header files
    /*
    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    */
    #include <chrono>
    #include <thread>
    #include <string>

    using namespace std;

    int main(void)
    {
        FRRobot robot; 
        robot.RPC("192.168.58.2"); 

        int retval = 0;

        retval = robot.ConveyorStartEnd(1);
        printf("ConveyorStartEnd retval is: %d\n", retval);

        retval = robot.ConveyorPointIORecord();
        printf("ConveyorPointIORecord retval is: %d\n", retval);

        retval = robot.ConveyorPointARecord();
        printf("ConveyorPointARecord retval is: %d\n", retval);

        retval = robot.ConveyorRefPointRecord();
        printf("ConveyorRefPointRecord retval is: %d\n", retval);

        retval = robot.ConveyorPointBRecord();
        printf("ConveyorPointBRecord retval is: %d\n", retval);

        retval = robot.ConveyorStartEnd(0);
        printf("ConveyorStartEnd retval is: %d\n", retval);
        
        retval = 0;
        float param[6] ={1,10000,200,0,0,20};
        retval = robot.ConveyorSetParam(param);
        printf("ConveyorSetParam retval is: %d\n", retval);
        
        double cmp[3] = {0.0, 0.0, 0.0};
        retval = robot.ConveyorCatchPointComp(cmp);
        printf("ConveyorCatchPointComp retval is: %d\n", retval);

        int index = 1;
        int max_time = 30000;
        uint8_t block = 0;
        retval = 0;
        
        /* The following is a conveyor belt grabbing process */
        DescPose desc_p1;
        desc_p1.tran.x = -351.553;
        desc_p1.tran.y = 87.913;
        desc_p1.tran.z = 354.175;
        desc_p1.rpy.rx = -179.680;
        desc_p1.rpy.ry =  -0.133;
        desc_p1.rpy.rz = 2.472;

        DescPose desc_p2;
        desc_p2.tran.x = -351.535;
        desc_p2.tran.y = -247.222;
        desc_p2.tran.z = 354.173;
        desc_p2.rpy.rx = -179.680;
        desc_p2.rpy.ry =  -0.137;
        desc_p2.rpy.rz = 2.473;


        retval = robot.MoveCart(&desc_p1, 1, 0, 100.0, 100.0, 100.0, -1.0, -1);
        printf("MoveCart retval is: %d\n", retval);

        retval = robot.WaitMs(1);
        printf("WaitMs retval is: %d\n", retval);

        retval = robot.ConveyorIODetect(10000);
        printf("ConveyorIODetect retval is: %d\n", retval);

        retval = robot.ConveyorGetTrackData(1);
        printf("ConveyorGetTrackData retval is: %d\n", retval);

        retval = robot.ConveyorTrackStart(1);
        printf("ConveyorTrackStart retval is: %d\n", retval);

        retval = robot.TrackMoveL("cvrCatchPoint",  1, 0, 100, 100, 100, -1.0, 0, 0);
        printf("TrackMoveL retval is: %d\n", retval);

        retval = robot.MoveGripper(index, 51, 40, 30, max_time, block);
        printf("MoveGripper retval is: %d\n", retval);

        retval = robot.TrackMoveL("cvrRaisePoint", 1, 0, 100, 100, 100, -1.0, 0, 0);
        printf("TrackMoveL retval is: %d\n", retval);

        retval = robot.ConveyorTrackEnd();
        printf("ConveyorTrackEnd retval is: %d\n", retval);

        robot.MoveCart(&desc_p2, 1, 0, 100.0, 100.0, 100.0, -1.0, -1);

        retval = robot.MoveGripper(index, 100, 40, 10, max_time, block);
        printf("MoveGripper retval is: %d\n", retval);

        return 0;
    }

Code example
+++++++++++++++

.. versionadded:: C++ SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    #include "libfairino/robot.h"
    //If using Windows, include the following header files
    #include <string.h>
    #include <windows.h>
    //If using Linux, include the following header files
    /*
    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    */
    #include <chrono>
    #include <thread>
    #include <string>

    using namespace std;

    int main(void)
    {
        FRRobot robot;           
        robot.RPC("192.168.58.2");

        char file_path[256] = "/fruser/traj/test_computermd5.txt.txt";
        char md5[256] = {0};
        uint8_t emerg_state = 0;
        uint8_t si0_state = 0;
        uint8_t si1_state = 0;
        int sdk_com_state = 0;

        char ssh_keygen[1024] = {0};
        int retval = robot.GetSSHKeygen(ssh_keygen);
        printf("GetSSHKeygen retval is: %d\n", retval);
        printf("ssh key is: %s \n", ssh_keygen);

        char ssh_name[32] = "fr";
        char ssh_ip[32] = "192.168.58.44";
        char ssh_route[128] = "/home/fr";
        char ssh_robot_url[128] = "/root/robot/dhpara.config";
        retval = robot.SetSSHScpCmd(1, ssh_name, ssh_ip, ssh_route, ssh_robot_url);
        printf("SetSSHScpCmd retval is: %d\n", retval);
        printf("robot url is: %s\n", ssh_robot_url);

        robot.ComputeFileMD5(file_path, md5);
        printf("md5 is: %s \n", md5);

        robot.GetRobotEmergencyStopState(&emerg_state);
        printf("emergency state is: %u \n", emerg_state);

        robot.GetSafetyStopState(&si0_state, &si1_state);
        printf("safety stop state is: %u, %u \n", si0_state, si1_state);

        robot.GetSDKComState(&sdk_com_state);
        printf("sdk com state is: %d", sdk_com_state);
        return 0;
    }

Wire search begins
+++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief  Wire search begins
    * @param  [in] refPos  1- Reference point 2- contact point
    * @param  [in] searchVel   Search speed %
    * @param  [in] searchDis  Seeking distance mm
    * @param  [in] autoBackFlag Automatic return flag, 0- not automatic; - Auto
    * @param  [in] autoBackVel  Automatic return speed %
    * @param  [in] autoBackDis  Automatic return distance mm
    * @param  [in] offectFlag  1- Find with offset; 2- Find the teaching point
    * @return  error code
    */
     errno_t WireSearchStart(int refPos, float searchVel, int searchDis, int autoBackFlag, float autoBackVel, int autoBackDis, int offectFlag);

Wire locating is complete
++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
     * @brief  Wire locating is complete
     * @param  [in] refPos  1- Reference point 2- contact point
     * @param  [in] searchVel   Search speed %
     * @param  [in] searchDis  Seeking distance mm
     * @param  [in] autoBackFlag Automatic return flag, 0- not automatic; - Auto
     * @param  [in] autoBackVel  Automatic return speed %
     * @param  [in] autoBackDis  Automatic return distance mm
     * @param  [in] offectFlag  1- Find with offset; 2- Find the teaching point
     * @return  error code
     */
    errno_t WireSearchEnd(int refPos, float searchVel, int searchDis, int autoBackFlag, float autoBackVel, int autoBackDis, int offectFlag);

Calculate the seeking offset of the welding wire
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

     /**
      * @brief  Calculate the seeking offset of the welding wire
      * @param  [in] seamType  Weld type
      * @param  [in] method   Calculation method
      * @param  [in] varNameRef Reference points 1-6, "#" indicates no point variable
      * @param  [in] varNameRes Contact points 1-6, "#" indicates no point variable
      * @param  [out] offectFlag 0- offset is superimposed directly to the instruction point; 1- Offset requires a coordinate transformation of the instruction point
      * @param  [out] offect Offset pose[x, y, z, a, b, c]
      * @return  error code
      */
     errno_t GetWireSearchOffset(int seamType, int method, std::vector<std::string> varNameRef, std::vector<std::string> varNameRes, int& offectFlag, DescPose& offect);

Wait for wire locating to complete
+++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

     /**
      * @brief  Wait for wire locating to complete
      * @return error code
      */
     errno_t WireSearchWait(std::string varName);

Wire seeking contact is written to the database
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Wire seeking contact is written to the database
      * @param  [in] varName  Contact point name: RES0 ~ RES99
      * @param  [in] pos  Contact data[x, y, x, a, b, c]
      * @return  error code
      */
     errno_t SetPointToDatabase(std::string varName, DescPose pos);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    void Wiresearch(FRRobot* robot)
    {
    int rtn0, rtn1, rtn2 = 0;
    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };

    DescPose descStart = { 203.061, 56.768, 62.719, -177.249, 1.456, -83.597 };
    JointPos jointStart = { -127.012, -112.931, -94.078, -62.014, 87.186, 91.326 };

    DescPose descEnd = { 122.471, 55.718, 62.209, -177.207, 1.375, -76.310 };
    JointPos jointEnd = { -119.728, -113.017, -94.027, -62.061, 87.199, 91.326 };

    robot->MoveL(&jointStart, &descStart, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese );
    robot->MoveL(&jointEnd, &descEnd, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);

    DescPose descREF0A = { 147.139, -21.436, 60.717, -179.633, -3.051, -83.170 };
    JointPos jointREF0A = { -121.731, -106.193, -102.561, -64.734, 89.972, 96.171 };

    DescPose descREF0B = { 139.247, 43.721, 65.361, -179.634, -3.043, -83.170 };
    JointPos jointREF0B = { -122.364, -113.991, -90.860, -68.630, 89.933, 95.540 };

    DescPose descREF1A = { 289.747, 77.395, 58.390, -179.074, -2.901, -89.790 };
    JointPos jointREF1A = { -135.719, -119.588, -83.454, -70.245, 88.921, 88.819 };

    DescPose descREF1B = { 259.310, 79.998, 64.774, -179.073, -2.900, -89.790 };
    JointPos jointREF1B = { -133.133, -119.029, -83.326, -70.976, 89.069, 91.401 };

    rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot->MoveL(&jointREF0A, &descREF0A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //Start point
    robot->MoveL(&jointREF0B, &descREF0B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //Direction point
    rtn1 = robot->WireSearchWait("REF0");
    rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot->MoveL(&jointREF1A, &descREF1A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
    robot->MoveL(&jointREF1B, &descREF1B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
    rtn1 = robot->WireSearchWait("REF1");
    rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot->MoveL(&jointREF0A, &descREF0A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
    robot->MoveL(&jointREF0B, &descREF0B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
    rtn1 = robot->WireSearchWait("RES0");
    rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot->MoveL(&jointREF1A, &descREF1A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
    robot->MoveL(&jointREF1B, &descREF1B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
    rtn1 = robot->WireSearchWait("RES1");
    rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    vector <string> varNameRef = { "REF0", "REF1", "#", "#", "#", "#" };
    vector <string> varNameRes = { "RES0", "RES1", "#", "#", "#", "#" };
    int offectFlag = 0;
    DescPose offectPos = {0, 0, 0, 0, 0, 0};
    rtn0 = robot->GetWireSearchOffset(0, 0, varNameRef, varNameRes, offectFlag, offectPos);
    robot->PointsOffsetEnable(0, &offectPos);
    robot->MoveL(&jointStart, &descStart, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    robot->MoveL(&jointEnd, &descEnd, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);
    robot->PointsOffsetDisable();
    }

Arc tracking control
+++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
     * @brief  Arc tracking control
     * @param  [in] flag Switch, 0-off; 1-on
     * @param  [in] dalayTime Lag time, in ms
     * @param  [in] isLeftRight Left-right deviation compensation
     * @param  [in] klr Left-right adjustment coefficient (sensitivity);
     * @param  [in] tStartLr Left-right start compensation time around cyc
     * @param  [in] stepMaxLr Left-right the maximum compensation amount each time mm
     * @param  [in] sumMaxLr Left-right total maximum compensation mm
     * @param  [in] isUpLow Up-down compensation
     * @param  [in] kud Up-down adjustment factor;
     * @param  [in] tStartUd Start Up-down compensation time cyc
     * @param  [in] stepMaxUd Maximum compensation amount Up-down each time mm
     * @param  [in] sumMaxUd Total maximum compensation Up-down
     * @param  [in] axisSelect Up-down coordinate system selection, 0-swing; 1- Tools; 2- Base
     * @param  [in] referenceType Up-down reference current setting mode, 0-feedback; 1-constant
     * @param  [in] referSampleStartUd Up-down reference current sampling start count (feedback);cyc
     * @param  [in] referSampleCountUd Up-down reference current sampling cycle count;cyc
     * @param  [in] referenceCurrent Up-down reference current mA
     * @return  error code
      */
     errno_t ArcWeldTraceControl(int flag, double delaytime, int isLeftRight, double klr, double tStartLr, double stepMaxLr, double sumMaxLr, int isUpLow, double kud, double tStartUd, double stepMaxUd, double sumMaxUd, int axisSelect, int referenceType, double referSampleStartUd, double referSampleCountUd, double referenceCurrent);

Set the input signal port for arc tracking
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Set the input signal port for arc tracking
      * @param  [in] channel Arc tracking AI passband selection,[0-3]
      * @return  error code
      */
     errno_t ArcWeldTraceExtAIChannelConfig(int channel);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int WeldTraceControl(FRRobot* robot)
    {
    DescPose startdescPose = { -583.168, 325.637, 1.176, 75.262, 0.978, -3.571 };
    JointPos startjointPos = { -49.049, -77.203, 136.826, -189.074, -79.407, -11.811 };

    DescPose enddescPose = { -559.439, 420.491, 32.252, 77.745, 1.460, -10.130 };
    JointPos endjointPos = { -54.986, -77.639, 131.865, -185.707, -80.916, -12.218 };

    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };

    robot->WeldingSetCurrent(1, 230, 0, 0);
    robot->WeldingSetVoltage(1, 24, 0, 1);

    robot->MoveJ(&startjointPos, &startdescPose, 13, 0, 5, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot->ArcWeldTraceControl(1, 0, 0, 0.06, 5, 5, 300, 1, -0.06, 5, 5, 300, 1, 0, 4, 1, 10);
    robot->ARCStart(1, 0, 10000);
    robot->MoveL(&endjointPos, &enddescPose, 13, 0, 5, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    robot->ARCEnd(1, 0, 10000);

    robot->ArcWeldTraceControl(0, 0, 0, 0.06, 5, 5, 300, 1, -0.06, 5, 5, 300, 1, 0, 4, 1, 10);
    return 0;
    }

Force sensor assists drag
+++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
     * @brief  Force sensor assists drag
     * @param  [in] status Control status, 0- off; 1- On
     * @param  [in] asaptiveFlag Adaptive on flag, 0- off; 1- On
     * @param  [in] interfereDragFlag Interference drag flag, 0- off; 1- On
     * @param  [in] M Inertia coefficient
     * @param  [in] B Damping coefficient
     * @param  [in] K Stiffness coefficient
     * @param  [in] F Drag the six-dimensional force threshold
     * @param  [in] Fmax Maximum towing power limit
     * @param  [in] Vmax Maximum joint speed limit
     * @return  error code
     */
     errno_t EndForceDragControl(int status, int asaptiveFlag, int interfereDragFlag, std::vector<double> M, std::vector<double> B, std::vector<double> K, std::vector<double> F, double Fmax, double Vmax);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int DragControl(FRRobot* robot)
    {
    vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
    vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
    vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
    robot->EndForceDragControl(1, 0, 0, M, B, K, F, 50, 100);

    robot->Sleep(5000);

    robot->EndForceDragControl(0, 0, 0, M, B, K, F, 50, 100);
    }

The force sensor automatically On after the error is cleared
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  The force sensor automatically On after the error is cleared
      * @param  [in] status Control status, 0- off; 1- On
      * @return  error code
      */
     errno_t SetForceSensorDragAutoFlag(int status);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int FTAutoOn(FRRobot* robot)
    {
    robot->SetForceSensorDragAutoFlag(1);
    vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
    vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
    vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
    robot->EndForceDragControl(1, 0, 0, M, B, K, F, 50, 100);
    return 0;
    }

Sets the hybrid drag switch and parameters of six-dimensional force
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
     * @brief sets the hybrid drag switch and parameters of six-dimensional force and joint impedance
     * @param [in] status Control status, 0- off; 1- Open
     * @param [in] impedanceFlag Impedanceflag, 0-off; 1- Open
     * @param [in] lamdeDain Drag gain
     * @param [in] KGain Stiffness gain
     * @param [in] BGain Damping gain
     * @param [in] dragMaxTcpVel Drag the end maximum line speed limit
     * @param [in] dragMaxTcpOriVel Drag end maximum angular speed limit
     * @return  error code
     */
     errno_t ForceAndJointImpedanceStartStop(int status, int impedanceFlag, std::vector<double> lamdeDain, std::vector<double> KGain, std::vector<double> BGain, double dragMaxTcpVel, double dragMaxTcpOriVel);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int SixDiaDrag(FRRobot* robot)
    {
    robot->DragTeachSwitch(1);
    vector <double> lamdeDain = { 3.0, 2.0, 2.0, 2.0, 2.0, 3.0 };
    vector <double> KGain = { 0, 0, 0, 0, 0, 0 };
    vector <double> BGain = { 150, 150, 150, 5.0, 5.0, 1.0 };
    robot->ForceAndJointImpedanceStartStop(1, 0, lamdeDain, KGain, BGain, 1000, 180);

    robot->Sleep(5000);

    robot->DragTeachSwitch(0);
    robot->ForceAndJointImpedanceStartStop(0, 0, lamdeDain, KGain, BGain, 1000, 180);

    return 0;
    }

Get the drag switch status of the force sensor
+++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  get the drag switch status of the force sensor
      * @param  [out] dragState Force sensor auxiliary drag control status, 0-off; 1- Open
      * @param  [out] sixDimensionalDragState Control state, 0- off; 1- Open
      * @return  error code
      */
     errno_t GetForceAndTorqueDragState(int& dragState, int& sixDimensionalDragState);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int RobotGetFTDragState(FRRobot* robot)
    {
    int dragState = 0;
    int sixDimensionalDragState = 0;
    robot->GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
    printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);
    robot->Sleep(1000);
    vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
    vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
    vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
    robot->EndForceDragControl(1, 0, 0, M, B, K, F, 50, 100);
    robot->GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
    printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);

    robot->Sleep(1000);
    robot->EndForceDragControl(0, 0, 0, M, B, K, F, 50, 100);
    robot->GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
    printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);
    robot->Sleep(1000);

    robot->DragTeachSwitch(1);
    vector <double> lamdeDain = { 3.0, 2.0, 2.0, 2.0, 2.0, 3.0 };
    vector <double> KGain = { 0, 0, 0, 0, 0, 0 };
    vector <double> BGain = { 150, 150, 150, 5.0, 5.0, 1.0 };
    robot->ForceAndJointImpedanceStartStop(1, 0, lamdeDain, KGain, BGain, 1000, 180);
    robot->GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
    printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);
    robot->Sleep(1000);
    robot->DragTeachSwitch(0);
    robot->ForceAndJointImpedanceStartStop(0, 0, lamdeDain, KGain, BGain, 1000, 180);
    robot->GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
    printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);

    return 0;
    }

Sets the load weight under the force sensor
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Sets the load weight under the force sensor
      * @param  [in] weight Load weight kg
      * @return  error code
      */
     errno_t SetForceSensorPayload(double weight);

Sets the load center of mass under the force sensor
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Sets the load center of mass under the force sensor
      * @param  [in] x Load centroid x mm
      * @param  [in] y Load centroid y mm
      * @param  [in] z Load centroid z mm
      * @return  error code
      */
     errno_t SetForceSensorPayloadCog(double x, double y, double z);

Obtains the load weight under the force sensor
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:
    
     /**
      * @brief  Obtains the load weight under the force sensor
      * @param  [in] weight Load weight kg
      * @return  error code
      */
     errno_t GetForceSensorPayload(double& weight);

Obtains the load center of mass under the force sensor
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Obtains the load center of mass under the force sensor
      * @param  [out] x Load centroid x mm
      * @param  [out] y Load centroid y mm
      * @param  [out] z Load centroid z mm
      * @return  error code
      */
     errno_t GetForceSensorPayloadCog(double& x, double& y, double& z);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int FTLoadSetGet(FRRobot* robot)
    {
    robot->SetForceSensorPayload(0.824);
    robot->SetForceSensorPayloadCog(0.778, 2.554, 48.765);
    double weight = 0;
    double x = 0, y = 0, z = 0;
    robot->GetForceSensorPayload(weight);
    robot->GetForceSensorPayloadCog(x, y, z);
    printf("the FT load is   %lf,  %lf  %lf  %lf\n", weight, x, y, z);

    return 0;
    }

Force sensor automatically adjusts homing
+++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  force sensor automatically adjusts homing
      * @param  [out] weight Weight of the sensor kg
      * @param  [out] pos sensor centroid mm
      * @return  error code
      */
     errno_t ForceSensorAutoComputeLoad(double& weight, DescTran& pos);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int FTAutoComputeLoad(FRRobot* robot)
    {
    robot->SetForceSensorPayload(0);
    robot->SetForceSensorPayloadCog(0, 0, 0);
    double weight = 0;
    DescTran tran = {};
    robot->ForceSensorAutoComputeLoad(weight, tran);
    cout << "the result is weight " << weight << " pos is  " << tran.x << "  " << tran.y << "  " << tran.z << endl;
    return 0;
    }

Set welding process curve parameters
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Set welding process curve parameters
      * @param [in] id Welding process number (1-99)
      * @param [in] startCurrent Arcing current (A)
      * @param [in] startVoltage Arc voltage (V)
      * @param [in] startTime Arc starting time (ms)
      * @param [in] weldCurrent Welding current (A)
      * @param [in] weldVoltage Welding voltage (V)
      * @param [in] endCurrent (A)
      * @param [in] endVoltage (V)
      * @param [in] endTime Arc recovery time (ms)
      * @return  error code
      */
     errno_t WeldingSetProcessParam(int id, double startCurrent, double startVoltage, double startTime, double weldCurrent, double weldVoltage, double endCurrent, double endVoltage, double endTime);

Obtain welding process curve parameters
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Obtain welding process curve parameters
      * @param [in] id Welding process number (1-99)
      * @param [out] startCurrent Arcing current (A)
      * @param [out] startVoltage Arc voltage (V)
      * @param [out] startTime Arc starting time (ms)
      * @param [out] weldCurrent Welding current (A)
      * @param [out] weldVoltage Welding voltage (V)
      * @param [out] endCurrent (A)
      * @param [out] endVoltage Return voltage (V)
      * @param [out] endTime Arc recovery time (ms)
      * @return  error code
      */
     errno_t WeldingGetProcessParam(int id, double& startCurrent, double& startVoltage, double& startTime, double& weldCurrent, double& weldVoltage, double& endCurrent, double& endVoltage, double& endTime);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int WeldingProcessParamConfig(FRRobot* robot)
    {
    robot->WeldingSetProcessParam(1, 177, 27, 1000, 178, 28, 176, 26, 1000);
    robot->WeldingSetProcessParam(2, 188, 28, 555, 199, 29, 133, 23, 333);

    double startCurrent = 0;
    double startVoltage = 0;
    double startTime = 0;
    double weldCurrent = 0;
    double weldVoltage = 0;
    double endCurrent = 0;
    double endVoltage = 0;
    double endTime = 0;

    robot->WeldingGetProcessParam(1, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);
    cout << "the Num 1 process param is " << startCurrent << "  " << startVoltage<< "  " <<startTime<<"  " <<weldCurrent<< "  " <<weldVoltage<< "  " <<endCurrent<< "  " <<endVoltage<< "  " <<endTime << endl;

    robot->WeldingGetProcessParam(2, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);
    cout << "the Num 2 process param is " << startCurrent << "  " << startVoltage << "  " << startTime << "  " << weldCurrent << "  " << weldVoltage << "  " << endCurrent << "  " << endVoltage << "  " << endTime << endl;
    return 0;
    }

End sensor configuration
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief  End sensor configuration
    * @param [in] idCompany, 18-JUNKONG; 25-HUIDE
    * @param [in] idDevice type, 0-JUNKONG/RYR6T.V1.0
    * @param [in] idSoftware Software version, 0-J1.0/HuiDe1.0(not yet available)
    * @param [in] idBus mount position, 1- end port 1; 2- Terminal port 2... 8- Terminal Port 8 (not yet open)
    * @return  error code
    */
    errno_t AxleSensorConfig(int idCompany, int idDevice, int idSoftware, int idBus);

Gets the end sensor configuration
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  gets the end sensor configuration
      * @param  [out] idCompany Company，18-JUNKONG; 25-HUIDE
      * @param  [out] idDevice type, 0-JUNKONG/RYR6T.V1.0
      * @return  error code
      */
    errno_t AxleSensorConfigGet(int& idCompany, int& idDevice);

End sensor activation
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  end sensor activation
      * @param  [in] actFlag 0- Reset; 1- Activation
      * @return  error code
      */
    errno_t AxleSensorActivate(int actFlag);

End sensor register write
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  end sensor register write
      * @param [in] devAddr Indicates the device address number 0-255
      * @param [in] regHAddr register address high 8 bits
      * @param [in] regLAddr Lower 8 bits of the register address
      * @param [in] regNum Number of registers 0-255
      * @param [in] data1 writes the register value 1
      * @param [in] data2 writes the register value 2
      * @param [in] isNoBlock 0- Block; 1- Non-blocking
      * @return  error code
      */
    errno_t AxleSensorRegWrite(int devAddr, int regHAddr, int regLAddr, int regNum, int data1, int data2, int isNoBlock);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    void AxleSensorConfig(FRRobot* robot)
    {
    robot->AxleSensorConfig(18, 0, 0, 1);
    int company = -1;
    int type = -1;
    robot->AxleSensorConfigGet(company, type);
    printf("company is %d, type is %d\n", company, type);

    robot->AxleSensorActivate(1);

    robot->Sleep(5000);

    while (true)
    {
    robot->AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0);
    }

    }

Sets whether the output is reset after the DO stop/pause of the control
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief  Sets whether the output is reset after the DO stop/pause of the control
     * @param  [in] resetFlag  0- no more bits; 1- Reset
     * @return  error code
     */
    errno_t SetOutputResetCtlBoxDO(int resetFlag);

Sets whether the output is reset after the control box AO is 
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Sets whether the output is reset after the control box AO is 
      * @param  [in] resetFlag  0- no more bits; 1- Reset
      * @return  error code
      */
    errno_t SetOutputResetCtlBoxAO(int resetFlag);

Sets whether the output is reset after the end tool DO is stopped/paused
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Sets whether the output is reset after the end tool DO is stopped/paused
      * @param  [in] resetFlag  0- no more bits; 1- Reset
      * @return  error code
      */
    errno_t SetOutputResetAxleDO(int resetFlag);
 
Sets whether the output is reset after the end tool AO is stopped/paused
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Sets whether the output is reset after the end tool AO is stopped/paused
      * @param  [in] resetFlag  0- no more bits; 1- Reset
      * @return  error code
      */
    errno_t SetOutputResetAxleAO(int resetFlag);
 
Sets whether the output is reset after the extension DO is stopped/paused
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Sets whether the output is reset after the extension DO is stopped/paused
      * @param  [in] resetFlag  0- no more bits; 1- Reset
      * @return  error code
      */
    errno_t SetOutputResetExtDO(int resetFlag);
 
Sets whether the output is reset after the extended AO is stopped/paused
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Sets whether the output is reset after the extended AO is stopped/paused
      * @param  [in] resetFlag  0- no more bits; 1- Reset
      * @return  error code
      */
    errno_t SetOutputResetExtAO(int resetFlag);
 
sets whether the output is reset after SmartTool stops/pauses
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  sets whether the output is reset after SmartTool stops/pauses
      * @param  [in] resetFlag  0- no more bits; 1- Reset
      * @return  error code
      */
    errno_t SetOutputResetSmartToolDO(int resetFlag);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int IOReset(FRRobot* robot)
    {
    int resetFlag = 0;
    int rtn = robot->SetOutputResetCtlBoxDO(resetFlag);
    robot->SetOutputResetCtlBoxAO(resetFlag);
    robot->SetOutputResetAxleDO(resetFlag);
    robot->SetOutputResetAxleAO(resetFlag);
    robot->SetOutputResetExtDO(resetFlag);
    robot->SetOutputResetExtAO(resetFlag);
    robot->SetOutputResetSmartToolDO(resetFlag);
    return 0;
    }
 
Simulation swing starts
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Simulation swing starts
      * @param  [in] weaveNum  Swing parameter number
      * @return  error code
      */
    errno_t WeaveStartSim(int weaveNum);
 
Simulation swing is over
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief Simulation swing is over
      * @param  [in] weaveNum  Swing parameter number
      * @return  error code
      */
    errno_t WeaveEndSim(int weaveNum);
 
Start trajectory detection warning (no movement)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief  Start trajectory detection warning (no movement)
      * @param  [in] weaveNum   Swing parameter number
      * @return  error code
      */
    errno_t WeaveInspectStart(int weaveNum);
 
End track detection warning (no movement)
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief End track detection warning (no movement)
      * @param  [in] weaveNum   Swing parameter number
      * @return  error code
      */
    errno_t WeaveInspectEnd(int weaveNum);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     int WeaveSim(FRRobot* robot)
    {
    DescPose startdescPose = { 238.209, -403.633, 251.291, 177.222, -1.433, 133.675 };
    JointPos startjointPos = { -48.728, -86.235, -95.288, -90.025, 92.715, 87.595 };
    DescPose enddescPose = { 238.207, -596.305, 251.294, 177.223, -1.432, 133.675 };
    JointPos endjointPos = { -60.240, -110.743, -66.784, -94.531, 92.351, 76.078 };

    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };

    robot->MoveL(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    robot->WeaveStartSim(0);
    robot->MoveL(&endjointPos, &enddescPose, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    robot->WeaveEndSim(0);
    return 0;
    }

    int WeaveInspect(FRRobot* robot)
    {
    DescPose startdescPose = { 238.209, -403.633, 251.291, 177.222, -1.433, 133.675 };
    JointPos startjointPos = { -48.728, -86.235, -95.288, -90.025, 92.715, 87.595 };
    DescPose enddescPose = { 238.207, -596.305, 251.294, 177.223, -1.432, 133.675 };
    JointPos endjointPos = { -60.240, -110.743, -66.784, -94.531, 92.351, 76.078 };

    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };

    robot->MoveL(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    robot->WeaveInspectStart(0);
    robot->MoveL(&endjointPos, &enddescPose, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    robot->WeaveInspectEnd(0);
    return 0;
    }
 
Extension IO- Configure welder gas detection signa
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief Extension IO- Configure welder gas detection signa
      * @param  [in] DONum Gas detection signal extension DO number
      * @return  error code
      */
    errno_t SetAirControlExtDoNum(int DONum);
 
Extension IO- Configs the arc signal of the welder
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief Extension IO- Configs the arc signal of the welder
      * @param  [in] DONum Welding machine arc signal extension DO number
      * @return  error code
      */
    errno_t SetArcStartExtDoNum(int DONum);
 
Extension IO- Configure the welder reverse wire feed signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief Extension IO- Configure the welder reverse wire feed signal
      * @param  [in] DONum  Reverse feed signal extension DO number
      * @return  error code
      */
    errno_t SetWireReverseFeedExtDoNum(int DONum);
 
Extension IO- Configure the welder forward wire feed signal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief Extension IO- Configure the welder forward wire feed signal
      * @param  [in] DONum  Extends the DO number to the forward wire feed signal
      * @return  error code
      */
    errno_t SetWireForwardFeedExtDoNum(int DONum);
 
Extension IO- Configure the welder arc success signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief Extension IO- Configure the welder arc success signal
      * @param  [in] DINum  起弧成功信号扩展DI编号
      * @return  error code
      */
    errno_t SetArcDoneExtDiNum(int DINum);
 
Extension IO- Configure the welder ready signal
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief Extension IO- Configure the welder ready signal
      * @param  [in] DINum Welder ready signal extension DI number
      * @return  error code
      */
    errno_t SetWeldReadyExtDiNum(int DINum);
 
Extension IO- Configs the welding interrupt recovery signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief Extension IO- Configs the welding interrupt recovery signal
      * @param  [in] reWeldDINum  Resume welding signal extension DI number after welding interruption
      * @param  [in] abortWeldDINum  Indicates the DI number of a welding signal that exits after a welding failure
      * @return  error code
      */
    errno_t SetExtDIWeldBreakOffRecover(int reWeldDINum, int abortWeldDINum);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int SetExtDIOFuntion(FRRobot* robot)
    {
    robot->SetArcStartExtDoNum(10);
    robot->SetAirControlExtDoNum(20);
    robot->SetWireForwardFeedExtDoNum(30);
    robot->SetWireReverseFeedExtDoNum(40);

    robot->SetWeldReadyExtDiNum(50);
    robot->SetArcDoneExtDiNum(60);
    robot->SetExtDIWeldBreakOffRecover(70, 80);
    return 0;
    }
 
Sets the collision detection method of the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief Sets the collision detection method of the robot
      * @param  [in] method Collision detection method: 0- current mode; 1- Dual encoder; 2- Current and dual encoder turn on simultaneously
      * @return  error code
      */
    errno_t SetCollisionDetectionMethod(int method);
 
Indicates that collision detection is disabled in static mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief Indicates that collision detection is disabled in static mode
      * @param  [in] status 0- Off; 1- Open
      * @return  error code
      */
    errno_t SetStaticCollisionOnOff(int status);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int StaticCollision(FRRobot* robot)
    {
    robot->SetCollisionDetectionMethod(0);
    robot->SetStaticCollisionOnOff(1);
    robot->Sleep(5000);
    robot->SetStaticCollisionOnOff(0);
    return 0;
    }
 
joint torque power detection
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     /**
      * @brief joint torque power detection
      * @param  [in] status 0- Off; 1- Open
      * @param  [in] power Set maximum power (W)
      * @return  error code
      */
     errno_t SetPowerLimit(int status, double power);
 
Joint torque control starts
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint torque control starts
    * @return  error code
    */
    errno_t ServoJTStart();
 
Joint torque control
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint torque control
    * @param  [in] torque j1 to j6 Joint torque, unit: Nm
    * @param  [in] interval Instruction period, unit s, range [0.001~0.008]
    * @return  error code
    */
    errno_t ServoJT(float torque[], double interval);
 
Joint torque control end
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Joint torque control end
    * @return  error code
    */
    errno_t ServoJTEnd();

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    int PowerLimitOn(FRRobot* robot)
    {
    robot->DragTeachSwitch(1);
    robot->SetPowerLimit(1, 2);
    float torques[] = { 0, 0, 0, 0, 0, 0 };
    robot->GetJointTorques(1, torques);

    int count = 100;
    robot->ServoJTStart(); //   #Servo JT starts
    int error = 0;
    while (count > 0)
    {
    torques[0] = torques[0] + 0.1;//  #Increase 0.1NM per axis and exercise 100 times
    error = robot->ServoJT(torques, 0.001);  //# Joint space servo mode motion
    count = count - 1;
    robot->Sleep(1);
    }

    error = robot->ServoJTEnd();  //#Servo motion ends
    return 0;
    }
 
Set the robot 20004 port feedback cycle
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Set the robot 20004 port feedback cycle
     * @param [in] period Robot 20004 Port Feedback Period(ms)
     * @return  error code
     */
    errno_t SetRobotRealtimeStateSamplePeriod(int period);
 
Obtains feedback cycle of robot 20004 port
+++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief  Obtains feedback cycle of robot 20004 port
     * @param [out] period Robot 20004 Port Feedback Period(ms)
     * @return  error code
     */
    errno_t GetRobotRealtimeStateSamplePeriod(int& period);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     void TestRealTimePeriod(FRRobot* robot)
     {
     robot->SetRobotRealtimeStateSamplePeriod(10);
     int getPeriod = 0;
     robot->GetRobotRealtimeStateSamplePeriod(getPeriod);
     cout << "period is " << getPeriod << endl;
     robot->Sleep(1000);
     }
 
Get robot joint driver temperature(℃)
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Get robot joint driver temperature(℃)
    * @return error code
    */
    errno_t GetJointDriverTemperature(double temperature[]);
 
Get robot joint drive torque(Nm)
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Get robot joint drive torque(Nm)
     * @return error code
     */
    errno_t GetJointDriverTorque(double torque[]);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     void TestTorue(FRRobot* robot)
     {
     robot->ProgramLoad("/fruser/test2.lua");
     robot->ProgramRun();
     int rtn = 0;
     while (true)
     {
     double temperature[6] = {};
     rtn = robot->GetJointDriverTemperature(temperature);
     double torque[6] = {};
     rtn = robot->GetJointDriverTorque(torque);
     printf("test torque is %f %f %f %f %f %f  temperature is %f %f %f %f %f %f\n", torque[0], torque[1], torque[2], torque[3], torque[4], torque[5], temperature[0], temperature[1], temperature[2], temperature[3], temperature[4], temperature[5]);
     robot->Sleep(100);
     }
     }
 
Arc tracing + multi - layer compensation open
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Arc tracing + multi - layer compensation open
     * @return error code
     */
    errno_t ArcWeldTraceReplayStart();
 
Arc Tracking + multi - layer compensation off
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Arc Tracking + multi - layer compensation off
     * @return error code
     */
    errno_t ArcWeldTraceReplayEnd();
 
Offset coordinate change - multipass welding
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Offset coordinate change - multipass welding
     * @return error code
     */
    errno_t MultilayerOffsetTrsfToBase(DescTran pointO, DescTran pointX, DescTran pointZ, double dx, double dy, double db, DescPose& offset);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    void TestWeldTraceReply(FRRobot* robot)
     {
     JointPos mulitilineorigin1_joint;
     mulitilineorigin1_joint.jPos[0] = -24.090;
     mulitilineorigin1_joint.jPos[1] = -63.501;
     mulitilineorigin1_joint.jPos[2] = 84.288;
     mulitilineorigin1_joint.jPos[3] = -111.940;
     mulitilineorigin1_joint.jPos[4] = -93.426;
     mulitilineorigin1_joint.jPos[5] = 57.669;
     
     DescPose mulitilineorigin1_desc;
     mulitilineorigin1_desc.tran.x = -677.559;
     mulitilineorigin1_desc.tran.y = 190.951;
     mulitilineorigin1_desc.tran.z = -1.205;
     mulitilineorigin1_desc.rpy.rx = 1.144;
     mulitilineorigin1_desc.rpy.ry = -41.482;
     mulitilineorigin1_desc.rpy.rz = -82.577;

     DescTran mulitilineX1_desc;
     mulitilineX1_desc.x = -677.556;
     mulitilineX1_desc.y = 211.949;
     mulitilineX1_desc.z = -1.206;

     DescTran mulitilineZ1_desc;
     mulitilineZ1_desc.x = -677.564;
     mulitilineZ1_desc.y = 190.956;
     mulitilineZ1_desc.z = 19.817;

     JointPos mulitilinesafe_joint;
     mulitilinesafe_joint.jPos[0] = -25.734;
     mulitilinesafe_joint.jPos[1] = -63.778;
     mulitilinesafe_joint.jPos[2] = 81.502;
     mulitilinesafe_joint.jPos[3] = -108.975;
     mulitilinesafe_joint.jPos[4] = -93.392;
     mulitilinesafe_joint.jPos[5] = 56.021;

     DescPose mulitilinesafe_desc;
     mulitilinesafe_desc.tran.x = -677.561;
     mulitilinesafe_desc.tran.y = 211.950;
     mulitilinesafe_desc.tran.z = 19.812;
     mulitilinesafe_desc.rpy.rx = 1.144;
     mulitilinesafe_desc.rpy.ry = -41.482;
     mulitilinesafe_desc.rpy.rz = -82.577;

     JointPos mulitilineorigin2_joint;
     mulitilineorigin2_joint.jPos[0] = -29.743;
     mulitilineorigin2_joint.jPos[1] = -75.623;
     mulitilineorigin2_joint.jPos[2] = 101.241;
     mulitilineorigin2_joint.jPos[3] = -116.354;
     mulitilineorigin2_joint.jPos[4] = -94.928;
     mulitilineorigin2_joint.jPos[5] = 55.735;

     DescPose mulitilineorigin2_desc;
     mulitilineorigin2_desc.tran.x = -563.961;
     mulitilineorigin2_desc.tran.y = 215.359;
     mulitilineorigin2_desc.tran.z = -0.681;
     mulitilineorigin2_desc.rpy.rx = 2.845;
     mulitilineorigin2_desc.rpy.ry = -40.476;
     mulitilineorigin2_desc.rpy.rz = -87.443;
     
     DescTran mulitilineX2_desc;
     mulitilineX2_desc.x = -563.965;
     mulitilineX2_desc.y = 220.355;
     mulitilineX2_desc.z = -0.680;

     DescTran mulitilineZ2_desc;
     mulitilineZ2_desc.x = -563.968;
     mulitilineZ2_desc.y = 215.362;
     mulitilineZ2_desc.z = 4.331;

     ExaxisPos epos;
     epos.ePos[0] = 0;
     epos.ePos[1] = 0;
     epos.ePos[2] = 0;
     epos.ePos[3] = 0;
     DescPose offset;
     offset.tran.x = 0;
     offset.tran.y = 0;
     offset.tran.z = 0;
     offset.rpy.rx = 0;
     offset.rpy.ry = 0;
     offset.rpy.rz = 0;

     robot->Sleep(10);
     int error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ARCStart(1, 0, 3000);
     printf("ARCStart return:  %d\n", error);

     error = robot->WeaveStart(0);
     printf("WeaveStart return:  %d\n", error);

     error = robot->ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10);
     printf("ArcWeldTraceControl return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 1, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10);
     printf("ArcWeldTraceControl return:  %d\n", error);

     error = robot->WeaveEnd(0);
     printf("WeaveEnd return:  %d\n", error);

     error = robot->ARCEnd(1, 0, 10000);
     printf("ARCEnd return:  %d\n", error);

     error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);

     error = robot->MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 10.0, 0.0, 0.0, offset);
     printf("MultilayerOffsetTrsfToBase return:  %d   offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);

     error = robot->MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ARCStart(1, 0, 3000);
     printf("ARCStart return:  %d\n", error);

     error = robot->MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 10, 0, 0, offset);
     printf("MultilayerOffsetTrsfToBase return:  %d   offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);

     error = robot->ArcWeldTraceReplayStart();
     printf("ArcWeldTraceReplayStart return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 2, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ArcWeldTraceReplayEnd();
     printf("ArcWeldTraceReplayEnd return:  %d\n", error);

     error = robot->ARCEnd(1, 0, 10000);
     printf("ARCEnd return:  %d\n", error);

     error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);

     error = robot->MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 0, 10, 0, offset);
     printf("MultilayerOffsetTrsfToBase return:  %d   offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);

     error = robot->MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 13, 0, 10, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ARCStart(1, 0, 3000);
     printf("ARCStart return:  %d\n", error);

     error = robot->MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 0, 10, 0, offset);
     printf("MultilayerOffsetTrsfToBase return:  %d   offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);

     error = robot->ArcWeldTraceReplayStart();
     printf("MoveJ return:  %d\n", error);

     error = robot->MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 13, 0, 2, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
     printf("MoveL return:  %d\n", error);

     error = robot->ArcWeldTraceReplayEnd();
     printf("ArcWeldTraceReplayEnd return:  %d\n", error);

     error = robot->ARCEnd(1, 0, 3000);
     printf("ARCEnd return:  %d\n", error);

     error = robot->MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 13, 0, 10, 100, 100, &epos, -1, 0, &offset);
     printf("MoveJ return:  %d\n", error);
     }
 
Specifies attitude speed on
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Specifies attitude speed on
    * @param [in] ratio Attitude velocity percentage[0 - 300]
    * @return  error code
    */
    errno_t AngularSpeedStart(int ratio);
 
Specifies attitude speed off
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Specifies attitude speed off
     * @return  error code
     */
    errno_t AngularSpeedEnd();
 
Code example
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    void TestAngular(FRRobot* robot)
     {
     JointPos JP1(-68.030, -63.537, -105.223, -78.368, 72.828, 24.876);
     DescPose DP1(-60.984, -533.958, 279.089, -22.052, -4.777, 172.406);

     JointPos JP2(-80.916, -76.030, -108.901, -70.956, 99.026, -74.533);
     DescPose DP2(36.750, -488.721, 145.781, -37.539, -11.211, -96.491);

     JointPos JP3(-86.898, -95.200, -103.665, -70.570, 98.266, -93.321);
     DescPose DP3(-21.462, -509.234, 25.706, -41.780, -1.042, -83.611);

     JointPos JP4(-85.364, -102.697, -94.674, -70.557, 95.302, -93.116);
     DescPose DP4(-24.075, -580.525, 25.881, -44.818, -2.357, -82.259);

     JointPos JP5(-78.815, -94.279, -105.315, -65.348, 87.328, 3.220);
     DescPose DP5(-29.155, -580.477, 25.884, -44.795, -2.374, -172.261);

     JointPos JP6(-81.057, -94.494, -105.107, -65.241, 87.527, 0.987);
     DescPose DP6(-49.270, -580.460, 25.886, -44.796, -2.374, -172.263);

     JointPos JP7(-76.519, -101.428, -94.915, -76.521, 85.041, 95.758);
     DescPose DP7(-54.189, -580.362, 25.878, -44.779, -2.353, 97.740);

     JointPos JP8(-74.406, -90.991, -106.574, -75.480, 85.150, 97.875);
     DescPose DP8(-54.142, -503.358, 25.865, -44.780, -2.353, 97.740);

     ExaxisPos epos(0, 0, 0, 0);
     DescPose offset(0, 0, 0, 0, 0, 0);

     int tool = 7;
     int user = 0;
     double vel = 100.0;
     double acc = 100.0;
     double ovl = 50.0;
     int blend = -1;
     int offsetFlag = 0;

     int error = robot->MoveJ(&JP1, &DP1, tool, user, vel, acc, ovl, &epos, blend, offsetFlag, &offset);
     error = robot->MoveJ(&JP2, &DP2, tool, user, vel, acc, ovl, &epos, blend, offsetFlag, &offset);
     error = robot->MoveL(&JP3, &DP3, tool, user, vel, acc, ovl, blend, &epos, 0, offsetFlag, &offset, 0, 100);
     robot->SetOaccScale(100);
     error = robot->MoveL(&JP4, &DP4, tool, user, vel, acc, ovl * 0.1, blend, &epos, 0, offsetFlag, &offset, 0, 100);
     robot->AngularSpeedStart(50);
     error = robot->MoveL(&JP5, &DP5, tool, user, vel, acc, ovl * 0.1, blend, &epos, 0, offsetFlag, &offset, 0, 100);
     robot->AngularSpeedEnd();
     error = robot->MoveL(&JP6, &DP6, tool, user, vel, acc, ovl * 0.1, blend, &epos, 0, offsetFlag, &offset, 0, 100);
     robot->AngularSpeedStart(50);
     error = robot->MoveL(&JP7, &DP7, tool, user, vel, acc, ovl * 0.1, blend, &epos, 0, offsetFlag, &offset, 0, 100);
     robot->AngularSpeedEnd();
     error = robot->MoveL(&JP8, &DP8, tool, user, vel, acc, ovl * 0.1, blend, &epos, 0, offsetFlag, &offset, 0, 100);
     }
 
Robot software upgrade
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Robot software upgrade
     * @param [in] filePath Full path of the software upgrade package
     * @param [in] block Whether to block until the upgrade is complete true: blocks. false : not blocking
     * @return  error code
     */
    errno_t SoftwareUpgrade(std::string filePath, bool block);
 
Get the robot software upgrade status
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the robot software upgrade status
    * @param [out] state Upgrade status of the robot software package(0 - Idle or uploading the upgrade package; 1 to 100: percentage of upgrade completion. - 1 : The software upgrade fails. - 2 : The verification fails. - 3 : Version verification fails. - 4 : The decompression fails. - 5 : The user configuration upgrade fails. - 6 : The peripheral configuration fails to be upgraded. - 7 : The expansion axis configuration fails to be upgraded. - 8 : The robot configuration fails to be upgraded. - 9 : Failed to configure DH parameters.
    * @return  error code
    */
    errno_t GetSoftwareUpgradeState(int &state);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     void TestUpgrade(FRRobot* robot)
     {
     robot->SoftwareUpgrade("D://test/software.tar.gz", false);
     while (true)
     {
     int curState = -1;
     robot->GetSoftwareUpgradeState(curState);
     printf("upgrade state is %d\n", curState);
     robot->Sleep(300);
     }
     }
 
Sets the speed of the 485 expansion axis
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief sets the speed of the 485 expansion axis
     * @param [in] acc 485 expansion axis motion acceleration
     * @param [in] dec 485 Expansion axis motion deceleration
     * @return  error code
     */
    errno_t AuxServoSetAcc(double acc, double dec);
 
Sets the speed of 485 expansion shaft to stop
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Sets the speed of 485 expansion shaft to stop
     * @param [in] acc 485 expansion axis acceleration
     * @param [in] dec 485 Expansion shaft stopped and slowed down
     * @return  error code
     */
    errno_t AuxServoSetEmergencyStopAcc(double acc, double dec);
 
Gets the speed of the 485 expansion axis movement
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Gets the speed of the 485 expansion axis movement
     * @param [out] acc 485 expansion axis acceleration
     * @param [out] dec 485 expansion axis motion deceleration
     * @return  error code
     */
    errno_t AuxServoGetAcc(double& acc, double& dec);
 
Gets the speed of 485 expansion axis
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Gets the speed of 485 expansion axis
     * @param [out] acc 485 expansion axis acceleration
     * @param [out] dec 485 Expansion shaft stopped and slowed down
     * @return  error code
     */
    errno_t AuxServoGetEmergencyStopAcc(double& acc, double& dec);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:
        
    void TestAuxservo(FRRobot* robot)
     {
     robot->AuxServoSetParam(1, 1, 1, 1, 130172, 15.45);
     robot->AuxServoEnable(1, 0);
     robot->Sleep(1000);
     robot->AuxServoSetControlMode(1, 1);
     robot->Sleep(1000);
     robot->AuxServoEnable(1, 1);
     robot->Sleep(1000);
     robot->AuxServoHoming(1, 1, 10, 10, 100);
     robot->Sleep(4000);
     robot->AuxServoSetAcc(3000, 3000);
     robot->AuxServoSetEmergencyStopAcc(5000, 5000);
     robot->Sleep(1000);
     double emagacc = 0;
     double emagdec = 0;
     robot->AuxServoGetEmergencyStopAcc(emagacc, emagdec);
     printf("emergency acc is %f  dec is %f \n", emagacc ,emagdec);

     robot->AuxServoSetTargetSpeed(1, 500, 100);

     robot->ProgramLoad("/fruser/testPTP.lua");
     robot->ProgramRun();
     int i = 0;
     while (true)
     {
     i++;
     if (i > 400)
     {
     robot->ResetAllError();
     i = 0;

     robot->AuxServoSetTargetSpeed(1, 500, 100);
     }
     ROBOT_STATE_PKG pkg;
     robot->GetRobotRealTimeState(&pkg);
     printf("%d:%d  cur velocity is %f   cur 485 axis emergency state is %d   robot collision state is %d  robot emergency state is %d\n",
     pkg.robotTime.second,pkg.robotTime.millisecond,pkg.aux_state.servoVel, ((pkg.aux_state.servoState >> 7) & 0x01), pkg.collisionState, pkg.EmergencyStop);
     robot->Sleep(5);
     ROBOT_STATE_PKG pkg;
     robot->GetRobotRealTimeState(&pkg);
     printf("cur velocity is %f   cur emergency state is %d \n", pkg.aux_state.servoVel, ((pkg.aux_state.servoState >> 7) & 0x01));
     robot->Sleep(20);
    robot->AuxServoSetAcc(5000, 5000);
    robot->AuxServoSetTargetPos(1, 1000, 500, 100);
    robot->Sleep(2000);
    robot->AuxServoSetTargetPos(1, 0, 500, 100);
    robot->Sleep(3000);
    robot->AuxServoSetAcc(500, 500);
    robot->AuxServoSetTargetPos(1, 1000, 500, 100);
    robot->Sleep(2000);
    robot->AuxServoSetTargetPos(1, 0, 500, 100);
    robot->Sleep(3000);
    robot->AuxServoSetTargetPos(1, 1000, 500, 10);
    robot->Sleep(5000);
    robot->AuxServoSetTargetPos(1, 0, 500, 10);
    robot->Sleep(5000);

    robot->AuxServoSetTargetSpeed(1, 500, 100);
    robot->Sleep(2000);
    robot->AuxServoSetTargetSpeed(1, 0, 100);
    robot->Sleep(2000);
    robot->AuxServoSetTargetSpeed(1, 500, 10);
    robot->Sleep(2000);
    robot->AuxServoSetTargetSpeed(1, 0, 10);
    robot->Sleep(2000);
     }
     }
 
Movable device enable
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Movable device enable
     * @param enable false-disable；true-enable
     * @return error code
     */
    errno_t TractorEnable(bool enable);
 
Movable device homing
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Movable device homing
     * @return error code
     */
    errno_t TractorHoming();
 
Movable device linear motion
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:
    
    /**
     * @brief Movable device linear motion
     * @param distance Linear motion distance (mm)
     * @param vel Linear speed percentage (0-100)
     * @return error code
     */
    errno_t TractorMoveL(double distance, double vel);
 
Circular motion of movable device
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Circular motion of movable device
     * @param radio Circular Motion radius (mm)
     * @param angle Angle of arc motion (°)
     * @param vel Linear speed percentage (0-100)
     * @return error code
     */
    errno_t TractorMoveC(double radio, double angle, double vel);
 
The movable device stops moving
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief The movable device stops moving
     * @return error code
     */
    errno_t TractorStop();

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    void TestTractorMove(FRRobot* robot)
     {
     robot->ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10);
     robot->ExtDevLoadUDPDriver();
     robot->ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
     robot->ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
     robot->SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0);

     robot->TractorEnable(false);
     robot->Sleep(2000);
     robot->TractorEnable(true);
     robot->Sleep(2000);
     robot->TractorHoming();
     robot->Sleep(2000);
     robot->TractorMoveL(100, 2);
     robot->Sleep(5000);
     robot->TractorStop();
     robot->TractorMoveL(-100, 20);
     robot->Sleep(5000);
     robot->TractorMoveC(300, 90, 20);
     robot->Sleep(10000);
     robot->TractorMoveC(300, -90, 20);
     robot->Sleep(1);
     }
 
Sets the solder wire seeking expansion IO port
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Sets the solder wire seeking expansion IO port
     * @param searchDoneDINum DO port (0-127)
     * @param searchStartDONum DO port for wire search start stop control (0-127)
     * @return error code
     */
    errno_t SetWireSearchExtDIONum(int searchDoneDINum, int searchStartDONum);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     void TestUDPWireSearch(FRRobot* robot)
     {
     robot->ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10);
     robot->ExtDevLoadUDPDriver();

     robot->SetWireSearchExtDIONum(0, 0);

     int rtn0, rtn1, rtn2 = 0;
     ExaxisPos exaxisPos = { 0.0, 0.0, 0.0, 0.0 };
     DescPose offdese = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
     
     DescPose descStart = { -158.767, -510.596, 271.709, -179.427, -0.745, -137.349 };
     JointPos jointStart = { 61.667, -79.848, 108.639, -119.682, -89.700, -70.985 };
     
     DescPose descEnd = { 0.332, -516.427, 270.688, 178.165, 0.017, -119.989 };
     JointPos jointEnd = { 79.021, -81.839, 110.752, -118.298, -91.729, -70.981 };

     robot->MoveL(&jointStart, &descStart, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     robot->MoveL(&jointEnd, &descEnd, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     
     DescPose descREF0A = { -66.106, -560.746, 270.381, 176.479, -0.126, -126.745 };
     JointPos jointREF0A = { 73.531, -75.588, 102.941, -116.250, -93.347, -69.689 };
     
     DescPose descREF0B = { -66.109, -528.440, 270.407, 176.479, -0.129, -126.744 };
     JointPos jointREF0B = { 72.534, -79.625, 108.046, -117.379, -93.366, -70.687 };
     
     DescPose descREF1A = { 72.975, -473.242, 270.399, 176.479, -0.129, -126.744 };
     JointPos jointREF1A = { 87.169, -86.509, 115.710, -117.341, -92.993, -56.034 };
     
     DescPose descREF1B = { 31.355, -473.238, 270.405, 176.480, -0.130, -126.745 };
     JointPos jointREF1B = { 82.117, -87.146, 116.470, -117.737, -93.145, -61.090 };

     rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot->MoveL(&jointREF0A, &descREF0A, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
     robot->MoveL(&jointREF0B, &descREF0B, 1, 0, 10, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
     rtn1 = robot->WireSearchWait("REF0");
     rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

     rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot->MoveL(&jointREF1A, &descREF1A, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
     robot->MoveL(&jointREF1B, &descREF1B, 1, 0, 10, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
     rtn1 = robot->WireSearchWait("REF1");
     rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

     rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot->MoveL(&jointREF0A, &descREF0A, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
     robot->MoveL(&jointREF0B, &descREF0B, 1, 0, 10, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
     rtn1 = robot->WireSearchWait("RES0");
     rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

     rtn0 = robot->WireSearchStart(0, 10, 100, 0, 10, 100, 0);
     robot->MoveL(&jointREF1A, &descREF1A, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);  //起点
     robot->MoveL(&jointREF1B, &descREF1B, 1, 0, 10, 100, 100, -1, &exaxisPos, 1, 0, &offdese);  //方向点
     rtn1 = robot->WireSearchWait("RES1");
     rtn2 = robot->WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

     vector <string> varNameRef = { "REF0", "REF1", "#", "#", "#", "#" };
     vector <string> varNameRes = { "RES0", "RES1", "#", "#", "#", "#" };
     int offectFlag = 0;
     DescPose offectPos = { 0, 0, 0, 0, 0, 0 };
     rtn0 = robot->GetWireSearchOffset(0, 0, varNameRef, varNameRes, offectFlag, offectPos);
     robot->PointsOffsetEnable(0, &offectPos);
     robot->MoveL(&jointStart, &descStart, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     robot->MoveL(&jointEnd, &descEnd, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
     robot->PointsOffsetDisable();
     }
 
Set the welding machine control mode expansion DO port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Set the welding machine control mode expansion DO port
     * @param DONum Welding machine control mode DO port (0-127)
     * @return error code
     */
    errno_t SetWeldMachineCtrlModeExtDoNum(int DONum);
 
Sets the welder control mode
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Sets the welder control mode
     * @param mode welding machine control mode; 0-Unified mode； 1-separate mode
     * @return error code
     */
    errno_t SetWeldMachineCtrlMode(int mode);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    void TestWeldmechineMode(FRRobot* robot)
     {
     robot->ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10);
     robot->ExtDevLoadUDPDriver();

     robot->SetWeldMachineCtrlModeExtDoNum(17);
     for (int i = 0; i < 5; i++)
     {
     robot->SetWeldMachineCtrlMode(0);
     robot->Sleep(1000);
     robot->SetWeldMachineCtrlMode(1);
     robot->Sleep(1000);
     }

     robot->SetWeldMachineCtrlModeExtDoNum(18);
     for (int i = 0; i < 5; i++)
     {
     robot->SetWeldMachineCtrlMode(0);
     robot->Sleep(1000);
     robot->SetWeldMachineCtrlMode(1);
     robot->Sleep(1000);
     }

     robot->SetWeldMachineCtrlModeExtDoNum(19);
     for (int i = 0; i < 5; i++)
     {
     robot->SetWeldMachineCtrlMode(0);
     robot->Sleep(1000);
     robot->SetWeldMachineCtrlMode(1);
     robot->Sleep(1000);
     }
     }
 
Set communication reconnection parameters with the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set communication reconnection parameters with the robot
    * @param  [in] enable   Enable reconnection when the network is faulty true- enabled false- disabled
    * @param  [in] reconnectTime Reconnection time, unit: ms
    * @param  [in] period Reconnection period, expressed in ms
    * @return  error code
    */
    errno_t SetReConnectParam(bool enable, int reconnectTime = 30000, int period = 50);

Code example
+++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

     int main(void)
     {
     ROBOT_STATE_PKG pkg = {};
     FRRobot robot;

     robot.LoggerInit();
     robot.SetLoggerLevel(1);
     int rtn = robot.RPC("192.168.58.2");
     robot.SetReConnectParam(true, 30000, 500);

     while (true)
     {
     int rtn = robot.GetRobotRealTimeState(&pkg);
     printf("the robot motiondone state is %d\n", pkg.jt_cur_pos[0]);
     robot.Sleep(200);
     }
     return 0;
     }
 
Start singular pose protection
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

	/**
	* @brief Start singular pose protection
	* @param [in] protectMode Singular protection mode, 0: joint mode; 1- Cartesian model
	* @param [in] minShoulderPos Shoulder Singular adjustment range (mm), default 100
	* @param [in] minElbowPos Elbow singular adjustment range (mm), default 50
	* @param [in] minWristPos Wrist singular adjustment range (°), default 10
	* @return error code
	*/
	errno_t SingularAvoidStart(int protectMode, double minShoulderPos, double minElbowPos, double minWristPos);
 
Stop singular pose protection
++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

	/**
	* @brief Stop singular pose protection
	* @return error code
	*/
	errno_t SingularAvoidEnd();

Code example
+++++++++++++++
.. code-block:: c++
    :linenos:

    void TestSingularAvoidWLin(FRRobot* robot)
    {
        DescPose startdescPose(-402.473, -185.876, 103.985, -175.367, 59.682, 94.221);
        JointPos startjointPos(-0.095, -50.828, 109.737, -150.708, -30.225, -0.623);

        DescPose enddescPose(-399.264, -184.434, 296.022, -4.402, 58.061, -94.161);
        JointPos endjointPos(-0.095, -65.547, 105.145, -131.397, 31.851, -0.622);

        ExaxisPos exaxisPos(0, 0, 0, 0);
        DescPose offdese(0, 0, 0, 0, 0, 0);

        robot->MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
        robot->SingularAvoidStart(0, 150, 50, 20);
        robot->MoveL(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
        robot->SingularAvoidEnd();
    }