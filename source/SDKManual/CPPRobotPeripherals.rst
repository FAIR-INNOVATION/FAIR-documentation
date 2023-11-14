Peripheral
====================

.. toctree:: 
    :maxdepth: 5

Configure the gripper
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Configure the gripper
    * @param  [in] company  Claw manufacturer, to be determined
    * @param  [in] device  Device number, not used yet. The default value is 0
    * @param  [in] softvesion  Software version. The value is not used. The default value is 0
    * @param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    * @return  Error code
    */
    errno_t  SetGripperConfig(int company, int device, int softvesion, int bus);

Obtain the gripper configuration
+++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Obtain the gripper configuration
    * @param  [in] company  Claw manufacturer, to be determined
    * @param  [in] device  Device number, not used yet. The default value is 0
    * @param  [in] softvesion  Software version. The value is not used. The default value is 0
    * @param  [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    * @return  Error code
    */
    errno_t  GetGripperConfig(int *company, int *device, int *softvesion, int *bus);

Activate gripper
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Activate Activate gripper
    * @param  [in] index  gripper gripper
    * @param  [in] act  0- reset, 1- activate
    * @return  Error code
    */
    errno_t  ActGripper(int index, uint8_t act);

Control gripper
++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Control gripper
    * @param  [in] index  gripper number
    * @param  [in] pos  Percentage of position, range[0~100]
    * @param  [in] vel  Percentage of velocity, range[0~100]
    * @param  [in] force  Percentage of torque, range[0~100]
    * @param  [in] max_time  Maximum wait time, range[0~30000], unit: ms
    * @param  [in] block  0- blocking, 1- non-blocking
    * @return  Error code
    */
    errno_t  MoveGripper(int index, int pos, int vel, int force, int max_time, uint8_t block);

Obtain the gripper motion state
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get the gripper motion status
     * @param  [out] fault 0-no error, 1-error
     * @param  [out] staus 0-the movement is not completed, 1-the movement is completed
     * @return  Error code 
     */
    errno_t  GetGripperMotionDone(uint16_t *fault, uint8_t *status);

Get the gripper activation status
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get the gripper activation status
     * @param  [out] fault 0-no error, 1-error
     * @param  [out] status bit0~bit15 corresponds to the gripper number 0~15, bit=0 is inactive, bit=1 is activated
     * @return  Error code
     */
    errno_t  GetGripperActivateStatus(uint16_t *fault, uint16_t *status);

Get the gripper position
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get the gripper position
     * @param  [out] fault 0-no error, 1-error
     * @param  [out] position position percentage, range 0~100%
     * @return  Error code 
     */
    errno_t  GetGripperCurPosition(uint16_t *fault, uint8_t *position);

Get the gripper speed
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get the gripper speed
     * @param  [out] fault 0-no error, 1-error
     * @param  [out] speed speed percentage, range 0~100%
     * @return  Error code 
     */
    errno_t  GetGripperCurSpeed(uint16_t *fault, int8_t *speed);

Get the gripper current
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get the gripper current
     * @param  [out] fault 0-no error, 1-error
     * @param  [out] current current percentage, range 0~100%
     * @return  Error code 
     */
    errno_t  GetGripperCurCurrent(uint16_t *fault, int8_t *current);

Get the gripper voltage
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get the gripper voltage
     * @param  [out] fault 0-no error, 1-error
     * @param  [out] voltage voltage, unit 0.1V
     * @return  Error code 
     */
    errno_t  GetGripperVoltage(uint16_t *fault, int *voltage);

Get the gripper temperature
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get the gripper temperature
     * @param  [out] fault 0-no error, 1-error
     * @param  [out] temp temperature, unit °C
     * @return  Error code 
     */
    errno_t  GetGripperTemp(uint16_t *fault, int *temp);

Calculate pre-fetch points - vision
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Calculate pre-fetch points - vision
     * @param  [in] desc_pos grab point Cartesian pose
     * @param  [in] zlength z-axis offset
     * @param  [in] zangle rotation offset around z-axis
     * @return  Error code  
     */
    errno_t  ComputePrePick(DescPose *desc_pos, double zlength, double zangle, DescPose *pre_pos);

Calculate retreat point-visual
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Calculate retreat point-visual
     * @param  [in] desc_pos grab point Cartesian pose
     * @param  [in] zlength z-axis offset
     * @param  [in] zangle rotation offset around z-axis
     * @return  Error code  
     */
    errno_t  ComputePostPick(DescPose *desc_pos, double zlength, double zangle, DescPose *post_pos);

Code example
++++++++++++++++
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

        int company = 4;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int index = 1;
        int act = 0;
        int max_time = 30000;
        uint8_t block = 0;
        uint8_t status, fault;

        robot.SetGripperConfig(company, device, softversion, bus);
        sleep(1);
        robot.GetGripperConfig(&company, &device, &softversion, &bus);
        printf("gripper config:%d,%d,%d,%d\n", company, device, softversion, bus);

        robot.ActGripper(index, act);
        sleep(1);
        act = 1;
        robot.ActGripper(index, act);
        sleep(2);

        robot.MoveGripper(index, 100, 50, 50, max_time, block);
        sleep(3);
        robot.MoveGripper(index, 0, 50, 0, max_time, block);

        robot.GetGripperMotionDone(&fault, &status);
        printf("motion status:%u,%u\n", fault, status);

        return 0;
    }
