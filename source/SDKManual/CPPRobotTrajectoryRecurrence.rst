Trajectory recurrence
==============================

.. toctree:: 
    :maxdepth: 5

Set track recording parameters
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set track recording parameters
    * @param  [in] type  Record data type, 1- joint position
    * @param  [in] name  Track file name
    * @param  [in] period_ms  Data sampling period, fixed value 2ms or 4ms or 8ms
    * @param  [in] di_choose  DI Select,bit0 to bit7 corresponds to control box DI0 to DI7, bit8 to bit9 corresponds to end DI0 to DI1, 0- do not select, 1- select
    * @param  [in] do_choose  DO select,bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0- do not select, 1- select
    * @return  Error code
    */
    errno_t  SetTPDParam(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose);

Start track recording
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Start track recording
    * @param  [in] type  Record data type, 1- joint position
    * @param  [in] name  Track file name
    * @param  [in] period_ms  Data sampling period, fixed value 2ms or 4ms or 8ms
    * @param  [in] di_choose  DI Select,bit0 to bit7 corresponds to control box DI0 to DI7, bit8 to bit9 corresponds to end DI0 to DI1, 0- do not select, 1- select
    * @param  [in] do_choose  DO select,bit0~bit7 corresponds to control box DO0~DO7, bit8~bit9 corresponds to end DO0~DO1, 0- do not select, 1- select
    * @return  Error code
    */
    errno_t  SetTPDStart(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose); 

Stop track recording
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Stop track recording
    * @return  Error code
    */
    errno_t  SetWebTPDStop();

Delete track record
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Delete track record
    * @param  [in] name  Track file name
    * @return  Error code
    */   
    errno_t  SetTPDDelete(char name[30]);

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

        int type = 1;
        char name[30] = "tpd2023";
        int period_ms = 4;
        uint16_t di_choose = 0;
        uint16_t do_choose = 0;

        robot.SetTPDParam(type, name, period_ms, di_choose, do_choose);

        robot.Mode(1);
        sleep(1);
        robot.DragTeachSwitch(1);
        robot.SetTPDStart(type, name, period_ms, di_choose, do_choose);
        sleep(30);
        robot.SetWebTPDStop();
        robot.DragTeachSwitch(0);

        //robot.SetTPDDelete(name);

        return 0;
    }

Trajectory preloading
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Trajectory preloading
    * @param  [in] name  Track file name
    * @return  Error code
    */      
    errno_t  LoadTPD(char name[30]);

Trajectory recurrence
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Trajectory recurrence
    * @param  [in] name  Track file name
    * @param  [in] blend 0- not smooth, 1- smooth
    * @param  [in] ovl  Speed scaling percentage, range [0~100]
    * @return  Error code
    */
    errno_t  MoveTPD(char name[30], uint8_t blend, float ovl);

Code example
++++++++++++++++++
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

        char name[30] = "tpd2023";
        int tool = 1;
        int user = 0;
        float vel = 100.0;
        float acc = 100.0;
        float ovl = 100.0;
        float blendT = -1.0;
        int config = -1;
        uint8_t blend = 1;

        DescPose desc_pose;
        memset(&desc_pose, 0, sizeof(DescPose));   

        desc_pose.tran.x = -378.9;
        desc_pose.tran.y = -340.3;
        desc_pose.tran.z = 107.2;
        desc_pose.rpy.rx = 179.4;
        desc_pose.rpy.ry = -1.3;
        desc_pose.rpy.rz = 125.0;

        robot.LoadTPD(name);
        robot.MoveCart(&desc_pose, tool, user, vel, acc, ovl, blendT, config);
        robot.MoveTPD(name, blend, ovl);

        return 0;
    }

Get the starting pose of the trajectory
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get the starting pose of the trajectory
     * @param  [in] name trajectory file name, file suffix is not required
     * @return  error code
     */     
    errno_t  GetTPDStartPose(char name[30], DescPose *desc_pose);

Trajectory preprocessing
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  trajectory preprocessing
     * @param  [in] name  trajectory file name
     * @param  [in] ovl speed scaling percentage, range [0~100]
     * @param  [in] opt 1-control point, default is 1
     * @return  error code 
     */     
    errno_t  LoadTrajectoryJ(char name[30], float ovl, int opt);

Trajectory recurrence
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  trajectory recurrence
     * @return  error code 
     */     
    errno_t  MoveTrajectoryJ();

Get the starting pose of the trajectory
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get the starting pose of the trajectory
     * @param  [in] name trajectory file name
     * @return  error code
     */     
    errno_t  GetTrajectoryStartPose(char name[30], DescPose *desc_pose);

Get the track point number
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get the track point number
     * @return  error code
     */     
    errno_t  GetTrajectoryPointNum(int *pnum);

Set the speed during trajectory running
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set the speed during trajectory running
     * @param  [in] ovl speed percentage
     * @return  error code 
     */     
    errno_t  SetTrajectoryJSpeed(float ovl);

Set the force and torque during trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set the force and torque during trajectory operation
     * @param  [in] ft Force and torque in three directions, units N and Nm
     * @return  error code
     */     
    errno_t  SetTrajectoryJForceTorque(ForceTorque *ft);

Set the force along the x direction during trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set the force along the x direction during trajectory operation
     * @param  [in] fx Force along the x direction, unit N
     * @return  error code  
     */     
    errno_t  SetTrajectoryJForceFx(double fx);

Set the force along the y direction during trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set the force along the y direction during trajectory operation
     * @param  [in] fy Force along the y direction, unit N
     * @return  error code  
     */     
    errno_t  SetTrajectoryJForceFy(double fy);

Set the force along the z direction during trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set the force along the z direction during trajectory operation
     * @param  [in] fz Force along the z direction, unit N
     * @return  error code 
     */     
    errno_t  SetTrajectoryJForceFz(double fz);

Set the torque around the x-axis during trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set the torque around the x-axis during trajectory operation
     * @param  [in] tx Torque around the x-axis, unit Nm
     * @return  error code
     */     
    errno_t  SetTrajectoryJTorqueTx(double tx);

Set the torque around the y-axis during trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set the torque around the y-axis during trajectory operation
     * @param  [in] ty Torque around the y-axis, unit Nm
     * @return  error code
     */     
    errno_t  SetTrajectoryJTorqueTy(double ty);

Set the torque around the z-axis during trajectory operation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set the torque around the z-axis during trajectory operation
     * @param  [in] tz Torque around the z-axis, unit Nm
     * @return  error code
     */     
    errno_t  SetTrajectoryJTorqueTz(double tz);
