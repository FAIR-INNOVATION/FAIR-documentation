Common Settings
==========================

.. toctree:: 
    :maxdepth: 5

Set global speed
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set global speed
    * @param  [in]  vel  Percentage of velocity, range[0~100]
    * @return  Error code
    */
    errno_t  SetSpeed(int vel);

Set the value of a system variable
+++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the value of a system variable
    * @param  [in]  id  Variable number, range[1~20]
    * @param  [in]  value Variable value
    * @return  Error code
    */
    errno_t  SetSysVarValue(int id, float value);

Set tool coordinate system
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set tool coordinate system
    * @param  [in] id Frame number, range[1~15]
    * @param  [in] coord  Tool center position relative to end flange center position
    * @param  [in] type  0- tool coordinates, 1- sensor coordinates
    * @param  [in] install Installation position, 0- robot end, 1- robot outside
    * @return  Error code
    */
    errno_t  SetToolCoord(int id, DescPose *coord, int type, int install);

Set the tool coordinate list
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the tool coordinate list
    * @param  [in] id Frame number, range[1~15]
    * @param  [in] coord  Tool center position relative to end flange center position
    * @param  [in] type  0- tool coordinates, 1- sensor coordinates
    * @param  [in] install Installation position, 0- robot end, 1- robot outside
    * @return  Error code
    */
    errno_t  SetToolList(int id, DescPose *coord, int type, int install);   

Set the external tool coordinate system
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the external tool coordinate system
    * @param  [in] id Frame number, range[1~15]
    * @param  [in] etcp  Tool center position relative to end flange center position
    * @param  [in] etool  To be determined
    * @return  Error code
    */
    errno_t  SetExToolCoord(int id, DescPose *etcp, DescPose *etool);

Set the list of external tool coordinate systems
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the list of external tool coordinate systems
    * @param  [in] id Frame number, range[1~15]
    * @param  [in] etcp  Tool center position relative to end flange center position
    * @param  [in] etool  To be determined
    * @return  Error code
    */
    errno_t  SetExToolList(int id, DescPose *etcp, DescPose *etool);  

Set the workpiece coordinate system
++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the workpiece coordinate system
    * @param  [in] id Frame number, range[1~15]
    * @param  [in] coord  Tool center position relative to end flange center position
    * @return  Error code
    */    
    errno_t  SetWObjCoord(int id, DescPose *coord);

Set the list of work coordinate systems
++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the list of work coordinate systems
    * @param  [in] id Frame number, range[1~15]
    * @param  [in] coord  Tool center position relative to end flange center position
    * @return  Error code
    */    
    errno_t  SetWObjList(int id, DescPose *coord);  

Set the end load weight
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the end load weight
    * @param  [in] weight  Load weight, unit: kg
    * @return  Error code
    */
    errno_t  SetLoadWeight(float weight);

Set the end-load centroid coordinates
+++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the end-load centroid coordinates
    * @param  [in] coord Centroid coordinates, unit: mm
    * @return  Error code
    */
    errno_t  SetLoadCoord(DescTran *coord);

Set the robot installation mode
+++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the robot installation mode
    * @param  [in] install  Installation mode: 0- formal installation, 1- side installation, 2- inverted installation
    * @return  Error code
    */
    errno_t  SetRobotInstallPos(uint8_t install);   

Set the robot installation Angle
+++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the robot installation Angle, free installation
    * @param  [in] yangle  Angle of inclination
    * @param  [in] zangle  Angle of rotation
    * @return  Error code
    */
    errno_t  SetRobotInstallAngle(double yangle, double zangle);


Wait for the specified time
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Wait for the specified time
    * @param  [in]  t_ms  unit: ms
    * @return  Error code
    */
    errno_t  WaitMs(int t_ms);

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

        int i;
        float value;
        int id;
        int type;
        int install;

        DescTran coord;
        DescPose t_coord, etcp, etool, w_coord;
        memset(&coord, 0, sizeof(DescTran));
        memset(&t_coord, 0, sizeof(DescPose));
        memset(&etcp, 0, sizeof(DescPose));
        memset(&etool, 0, sizeof(DescPose));
        memset(&w_coord, 0, sizeof(DescPose));

        robot.SetSpeed(20);

        for(i = 1; i < 21; i++)
        {
            robot.SetSysVarValue(i, i+0.5);
            robot.WaitMs(1000);
        }

        for(i = 1; i < 21; i++)
        {
            robot.GetSysVarValue(i, &value);
            printf("sys value:%f\n", value);
        }

        robot.SetLoadWeight(2.5);

        coord.x = 3.0;
        coord.y = 4.0;
        coord.z = 5.0;

        robot.SetLoadCoord(&coord);

        id = 10;
        t_coord.tran.x = 1.0;
        t_coord.tran.y = 2.0;
        t_coord.tran.z = 3.0;
        t_coord.rpy.rx = 4.0;
        t_coord.rpy.ry = 5.0;
        t_coord.rpy.rz = 6.0;
        type = 0;
        install = 0;
        robot.SetToolCoord(id, &t_coord, type, install);
        robot.SetToolList(id, &t_coord, type, install);

        etcp.tran.x = 1.0;
        etcp.tran.y = 2.0;
        etcp.tran.z = 3.0;
        etcp.rpy.rx = 4.0;
        etcp.rpy.ry = 5.0;
        etcp.rpy.rz = 6.0;
        etool.tran.x = 11.0;
        etool.tran.y = 22.0;
        etool.tran.z = 33.0;
        etool.rpy.rx = 44.0;
        etool.rpy.ry = 55.0;
        etool.rpy.rz = 66.0;
        id = 11;
        robot.SetExToolCoord(id, &etcp, &etool);
        robot.SetExToolList(id, &etcp, &etool);

        w_coord.tran.x = 11.0;
        w_coord.tran.y = 12.0;
        w_coord.tran.z = 13.0;
        w_coord.rpy.rx = 14.0;
        w_coord.rpy.ry = 15.0;
        w_coord.rpy.rz = 16.0;   
        id = 12;
        robot.SetWObjCoord(id, &w_coord);
        robot.SetWObjList(id, &w_coord);

        robot.SetRobotInstallPos(0);
        robot.SetRobotInstallAngle(15.0,25.0);

        return 0;
    }