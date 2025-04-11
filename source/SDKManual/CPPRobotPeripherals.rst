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
.. versionchanged:: C++SDK-v2.1.5.0
    
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
	* @param  [in] type grippr type, 0-parallel clamp; 1- Rotate the claw
	* @param  [in] rotNum Number of turns[0-100]
	* @param  [in] rotVel Percent rotation speed[100]
	* @param  [in] rotTorque Percentage of rotating torque [0-100]
    * @return  Error code
	 */
	errno_t  MoveGripper(int index, int pos, int vel, int force, int max_time, uint8_t block, int type, double rotNum, int rotVel, int rotTorque);

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

.. versionchanged:: C++ SDK-v2.1.2.0

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

    using namespace std;

    int main(void)
    {
        FRRobot robot; 
        robot.RPC("192.168.58.2"); 

        int company = 4;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int index = 1;
        int act = 0;
        int max_time = 30000;
        uint8_t block = 0;
        uint8_t status;
        uint16_t fault;
        uint16_t active_status = 0;
        uint8_t current_pos = 0;
        int8_t current = 0;
        int voltage = 0;
        int temp = 0;
        int8_t speed = 0;

        robot.SetGripperConfig(company, device, softversion, bus);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        robot.GetGripperConfig(&company, &device, &softversion, &bus);
        printf("gripper config:%d,%d,%d,%d\n", company, device, softversion, bus);

        robot.ActGripper(index, act);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        act = 1;
        robot.ActGripper(index, act);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        robot.MoveGripper(index, 100, 50, 50, max_time, block);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        robot.MoveGripper(index, 0, 50, 0, max_time, block);

        robot.GetGripperMotionDone(&fault, &status);
        printf("motion status:%u,%u\n", fault, status);

        robot.GetGripperActivateStatus(&fault, &active_status);
        printf("gripper active fault is: %u, status is: %u\n", fault, active_status);

        robot.GetGripperCurPosition(&fault, &current_pos);
        printf("fault is:%u, current position is: %u\n", fault, current_pos);

        robot.GetGripperCurCurrent(&fault, &current);
        printf("fault is:%u, current current is: %d\n", fault, current);

        robot.GetGripperVoltage(&fault, &voltage);
        printf("fault is:%u, current voltage is: %d \n", fault, voltage);

        robot.GetGripperTemp(&fault, &temp);
        printf("fault is:%u, current temperature is: %d\n", fault, temp);

        robot.GetGripperCurSpeed(&fault, &speed);
        printf("fault is:%u, current speed is: %d\n", fault, speed);

        int retval = 0;
        DescPose prepick_pose;
        DescPose postpick_pose;
        memset(&prepick_pose, 0, sizeof(DescPose));
        memset(&postpick_pose, 0, sizeof(DescPose));

        DescPose desc_p1;
        desc_p1.tran.x = -351.553;
        desc_p1.tran.y = 87.913;
        desc_p1.tran.z = 354.175;
        desc_p1.rpy.rx = -179.680;
        desc_p1.rpy.ry = -0.133;
        desc_p1.rpy.rz = 2.472;

        DescPose desc_p2;
        desc_p2.tran.x = -351.535;
        desc_p2.tran.y = -247.222;
        desc_p2.tran.z = 354.173;
        desc_p2.rpy.rx = -179.680;
        desc_p2.rpy.ry = -0.137;
        desc_p2.rpy.rz = 2.473;

        retval = robot.ComputePrePick(&desc_p1, 10, 0, &prepick_pose);
        printf("ComputePrePick retval is: %d\n", retval);
        printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", prepick_pose.tran.x, prepick_pose.tran.y, prepick_pose.tran.z, prepick_pose.rpy.rx, prepick_pose.rpy.ry, prepick_pose.rpy.rz);

        retval = robot.ComputePostPick(&desc_p2, -10, 0, &postpick_pose);
        printf("ComputePostPick retval is: %d\n", retval);
        printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", postpick_pose.tran.x, postpick_pose.tran.y, postpick_pose.tran.z, postpick_pose.rpy.rx, postpick_pose.rpy.ry, postpick_pose.rpy.rz);

        return 0;
    }

Gets the number of turns of the rotary gripper
++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: V3.7.6

.. code-block:: c++
    :linenos:

    /**
	 * @brief  Gets the number of turns of the rotary gripper
	 * @param  [out] fault  0-no error, 1-error
	 * @param  [out] num  Number of turns
	 * @return  Error code
	 */
	errno_t GetGripperRotNum(uint16_t* fault, double* num);

Gets the rotation speed of the rotating gripper
++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: V3.7.6

.. code-block:: c++
    :linenos:

    /**
	 * @brief  Gets the rotation speed of the rotating gripper
	 * @param  [out] fault  0-no error, 1-error
	 * @param  [out] speed  Percent rotation speed
	 * @return  Error code
	 */
	errno_t GetGripperRotSpeed(uint16_t* fault, int* speed);

Gets the rotating torque of the rotating gripper
+++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: V3.7.6

.. code-block:: c++
    :linenos:

    /**
	 * @brief  Gets the rotating torque of the rotating gripper
	 * @param  [out] fault  0-no error, 1-error
	 * @param  [out] torque  Percent torque of rotation
	 * @return  Error code
	 */
	errno_t GetGripperRotTorque(uint16_t* fault, int* torque);

Code example
********************

.. versionadded:: V3.7.6

.. code-block:: c++
    :linenos:

    int MoveRotGripper(FRRobot* robot, int pos, double rotPos)
    {
        robot->ResetAllError();
        robot->ActGripper(1, 1);
        robot->Sleep(1000);
        int rtn = robot->MoveGripper(1, pos, 50, 50, 5000, 1, 1, rotPos, 50, 100);
        printf("move gripper rtn is %d\n", rtn);
        uint16_t fault = 0;
        double rotNum = 0.0;
        int rotSpeed = 0;
        int rotTorque = 0;
        robot->GetGripperRotNum(&fault, &rotNum);
        robot->GetGripperRotSpeed (&fault, &rotSpeed);
        robot->GetGripperRotTorque(&fault, &rotTorque);
        printf("gripper rot num : %lf, gripper rotSpeed : %d, gripper rotTorque : %d\n", rotNum, rotSpeed, rotTorque);

        return 0;
    }