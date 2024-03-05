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
    * @note   Force/torque detection range：(ft-min_threshold, ft+max_threshold)
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

.. versionchanged:: C++SDK-v2.1.2.0

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

.. versionchanged:: C++SDK-v2.1.2.0

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

.. versionadded:: C++SDK-v2.1.2.0

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

.. versionadded:: C++SDK-v2.1.2.0

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

.. versionadded:: C++SDK-v2.1.2.0

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
