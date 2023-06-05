Basics
=================

.. toctree:: 
    :maxdepth: 5

Instantiate the robot
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Robot interface class constructor
    */
    FRRobot();

Establishes communication with the controller
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Establish communication with the robot controller
    * @param  [in] ip  Controller IP address. The default value is 192.168.58.2
    * @return Error code
    */
    errno_t  RPC(const char *ip);

Query the SDK version number
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Query the SDK version number
    * @param  [out] version  SDK version
    * @return  Error code
    */  
    errno_t  GetSDKVersion(char *version);

Obtain Controller IP address
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Obtain Controller IP address
    * @param  [out] ip  Controller IP
    * @return  Error code
    */
    errno_t  GetControllerIP(char *ip);

Control the robot to enter or exit the drag teaching mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Control the robot to enter or exit the drag teaching mode
    * @param  [in] state 0-exit drag mode，1-enter the drag mode
    * @return  Error code
    */
    errno_t  DragTeachSwitch(uint8_t state);

Queries whether the robot is in drag mode
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Check whether the robot is in drag mode
    * @param  [out] state 0-non-drag teaching mode，1-drag the teaching mode
    * @return  Error code
    */
    errno_t  IsInDragTeach(uint8_t *state);

Control up enable and down enable
+++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Enable or disable the function on or off the robot. By default, the function is enabled automatically after the robot is powered on
    * @param  [in] state  0-down-enable，1-upper enable
    * @return  Error code
    */
    errno_t  RobotEnable(uint8_t state);

Control robot hand/automatic mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Control robot hand/automatic mode
    * @param [in] mode 0-automatic mode，1-manual mode
    * @return Error code
    */
    errno_t  Mode(int mode);

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

        char ip[64]="";
        char version[64] = "";
        uint8_t state;

        robot.GetSDKVersion(version);
        printf("SDK version:%s\n", version);
        robot.GetControllerIP(ip);
        printf("controller ip:%s\n", ip);

        robot.Mode(1);
        sleep(1);
        robot.DragTeachSwitch(1);
        robot.IsInDragTeach(&state);
        printf("drag state :%u\n", state);
        sleep(3);
        robot.DragTeachSwitch(0);
        sleep(1);
        robot.IsInDragTeach(&state);
        printf("drag state :%u\n", state);
        sleep(3);

        robot.RobotEnable(0);
        sleep(3);
        robot.RobotEnable(1);

        robot.Mode(0);
        sleep(1);
        robot.Mode(1);
        
        return 0;
    }
