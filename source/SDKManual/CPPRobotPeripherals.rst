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

.. versionchanged:: C++SDK-v2.1.2.0

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

Welding starts
++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Welding starts
	* @param [in] ioType 0 - Controller IO; 1 - Extended IO
	* @param [in] arcNum welder profile number
	* @param [in] timeout time limit
	* @return Error code
	*/
	errno_t ARCStart(int ioType, int arcNum, int timeout);

Welding ended
++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Welding ended
	* @param [in] ioType IO Type 0 - Controller IO; 1 - Extended IO
	* @param [in] arcNum welder profile number
	* @param [in] timeout arc extinguishing timeout
	* @return Error code
	*/
	errno_t ARCEnd(int ioType, int arcNum, int timeout);

Set the relationship between welding current and output analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Set the corresponding relationship between welding current and output analog quantity
	* @param [in] currentMin current value at the left point of the linear relationship between welding current and analog output (A)
	* @param [in] currentMax current value at the right point of the linear relationship between welding current and analog output (A)
	* @param [in] outputVoltageMin Analog output voltage value (V) of the left point of the linear relationship between welding current and analog output
	* @param [in] outputVoltageMax The analog output voltage value (V) of the right point of the linear relationship between welding current and analog output
	* @return Error code
	*/
	errno_t WeldingSetCurrentRelation(double currentMin, double currentMax, double outputVoltageMin, double outputVoltageMax);

Set the relationship between welding voltage and output analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Set the corresponding relationship between welding voltage and output analog quantity
	* @param [in] weldVoltageMin eldVoltageMin Welding voltage value (A) at the left point of the linear relationship between welding voltage and analog output
	* @param [in] weldVoltageMax Welding voltage-analog output linear relationship right point welding voltage value (A)
	* @param [in] outputVoltageMin Analog output voltage value (V) of the left point of the linear relationship between welding voltage and analog output
	* @param [in] outputVoltageMax The analog output voltage value (V) of the right point of the linear relationship between welding voltage and analog output
	* @return Error code
	*/
	errno_t WeldingSetVoltageRelation(double weldVoltageMin, double weldVoltageMax, double outputVoltageMin, double outputVoltageMax);

Obtain the corresponding relationship between welding current and output analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Get the corresponding relationship between welding current and output analog quantity
	* @param [out] currentMin current value at the left point of the linear relationship between welding current and analog output (A)
	* @param [out] currentMax welding current and analog output (A)
	* @param [out] outputVoltageMin Analog output voltage value (V) of the left point of the linear relationship between welding current and analog output
	* @param [out] outputVoltageMax The analog output voltage value (V) of the right point of the linear relationship between welding current and analog output
	* @return Error code
	*/
	errno_t WeldingGetCurrentRelation(double *currentMin, double *currentMax, double *outputVoltageMin, double *outputVoltageMax);

Obtain the corresponding relationship between welding voltage and output analog
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Get the corresponding relationship between welding voltage and output analog quantity
	* @param [out] weldVoltageMin Welding voltage value (A) at the left point of the linear relationship between welding voltage and analog output
	* @param [out] weldVoltageMax Welding voltage-analog output linear relationship right point welding voltage value (A)
	* @param [out] outputVoltageMin Analog output voltage value (V) of the left point of the linear relationship between welding voltage and analog output
	* @param [out] outputVoltageMax The analog output voltage value (V) of the right point of the linear relationship between welding voltage and analog output
	* @return Error code
	*/
	errno_t WeldingGetVoltageRelation(double *weldVoltageMin, double *weldVoltageMax, double *outputVoltageMin, double *outputVoltageMax);

Set welding current
++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Set welding current
	* @param [in] ioType 0-control box IO； 1-extend IO
	* @param [in] current welding current(A)
	* @param [in] AOIndexWelding current control box analog output port(0-1)
	* @return Error code
	*/
	errno_t WeldingSetCurrent(int ioType, double current, int AOIndex);

Set welding voltage
++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Set welding voltage
	* @param [in] ioType 0-control box IO； 1-extend IO
	* @param [in] voltage welding voltage(V)
	* @param [in] AOIndex Welding voltage control box analog output port(0-1)
	* @return Error code
	*/
	errno_t WeldingSetVoltage(int ioType, double voltage, int AOIndex);

Set weave parameters
++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Set weave parameters
	* @param [in] weaveNum parameters number
	* @param [in] weaveType weave type：0- plane triangular weave ; 1- vertical L-shaped triangular weave; 2- clockwise circular weave; 3-counterclockwise circular weave; 4-plane sine weave; 5-vertical L-shaped sine weave; 6- vertical triangular weave; 7- Vertical sine weave
	* @param [in] weaveFrequency weave frequency(Hz)
	* @param [in] weaveIncStayTime Wait mode 0- period does not contain wait time; 1- Period contains the wait time
	* @param [in] weaveRange weave amplitude(mm)
	* @param [in] weaveLeftStayTime weave left residence time(ms)
	* @param [in] weaveRightStayTime weave right residence time(ms)
	* @param [in] weaveCircleRadio Circular wiggle-pullback ratio(0-100%)
	* @param [in] weaveStationary weave position wait, 0- position continue to move within the waiting time; 1- The position is stationary during the waiting time
	* @return Error code
	*/
	errno_t WeaveSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary);

Set weave parameters in real time
++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Set weave parameters in real time
	* @param [in] weaveNum parameters number
	* @param [in] weaveType weave type：0- plane triangular weave ; 1- vertical L-shaped triangular weave; 2- clockwise circular weave; 3-counterclockwise circular weave; 4-plane sine weave; 5-vertical L-shaped sine weave; 6- vertical triangular weave; 7- Vertical sine weave
	* @param [in] weaveFrequency weave frequency(Hz)
	* @param [in] weaveIncStayTime Wait mode 0- period does not contain wait time; 1- Period contains the wait time
	* @param [in] weaveRange weave amplitude(mm)
	* @param [in] weaveLeftStayTime weave left residence time(ms)
	* @param [in] weaveRightStayTime weave right residence time(ms)
	* @param [in] weaveCircleRadio Circular wiggle-pullback ratio(0-100%)
	* @param [in] weaveStationary weave position wait, 0- position continue to move within the waiting time; 1- The position is stationary during the waiting time
	* @return Error code
	*/
	errno_t WeaveOnlineSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary);

Weave start
++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Weave start
	* @param [in] weaveNum Weave welding parameter configuration number
	* @return Error code
	*/
	errno_t WeaveStart(int weaveNum);

Weave end
++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Weave end
	* @param [in] weaveNum Weave welding parameter configuration number
	* @return Error code
	*/
	errno_t WeaveEnd(int weaveNum);

Forward wire feed
++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Forward Wire Feed
	* @param [in] ioType 0-control box IO； 1-extend IO
	* @param [in] wireFeed wire control: 0-stop wire feed ；1-wire feed
	* @return Error code
	*/
	errno_t SetForwardWireFeed(int ioType, int wireFeed);

Reverse wire feed
++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Reverse wire feed
	* @param [in] ioType 0-control box IO； 1-extend IO
	* @param [in] wireFeed wire control: 0-stop wire feed ；1-wire feed
	* @return Error code
	*/
	errno_t SetReverseWireFeed(int ioType, int wireFeed);

Aspirated
++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief aspirated
	* @param [in] ioType  0-control box IO； 1-extend IO
	* @param [in] airControl aspirated control: 0-stop aspirated；1-aspirated
	* @return Error code
	*/
	errno_t SetAspirated(int ioType, int airControl);

Segment weld start
++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
	* @brief Segment weld start
	* @param [in] startDesePos Starting point Cartesian position
	* @param [in] endDesePos Ending point Cartesian position
	* @param [in] startJPos Starting point joint position
	* @param [in] endJPos Ending point joint position
	* @param [in] weldLength Weld length(mm)
	* @param [in] noWeldLength Length of unwelded section(mm)
	* @param [in] weldIOType 0-control box IO； 1-extend IO
	* @param [in] arcNum Welder configuration file number
	* @param [in] weldTimeout Arcing timeout time
	* @param [in] isWeave Weave or not
	* @param [in] weaveNum Weave welding parameter configuration number
	* @param [in] tool tool number
	* @param [in] user Workpiece coordinate number, range [1~15]
	* @param [in] vel Percentage of speed [0~100]
	* @param [in] acc Acceleration percentage, range[0~100]
	* @param [in] ovl Velocity scaling factor, range[0~100]
	* @param [in] blendR [-1.0]- movement in place (blocking), [0~1000.0]- Smoothing radius (non-blocking), unit: mm
	* @param [in] epos Position of expansion shaft, unit: mm
 	* @param [in] search 0- no wire seeking, 1- wire seeking
	* @param [in] offset_flag 0- no offset, 1- offset in base/job coordinate system, 2- offset in tool coordinate system
	* @param [in] offset_pos The pose offset
	* @return Error code
	*/
	errno_t SegmentWeldStart(DescPose *startDesePos, DescPose *endDesePos, JointPos *startJPos, JointPos *endJPos, double weldLength, double noWeldLength, int weldIOType, int arcNum, int weldTimeout, bool isWeave, int weaveNum, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos);

Code example
++++++++++++++++++++++++++++++++++++++

.. versionchanged:: C++SDK-v2.1.2.0

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

        double current_min = 0;
        double current_max = 0;
        double vol_min = 0;
        double vol_max = 0;
        double output_vmin = 0;
        double output_vmax = 0;

        DescPose start_descpose;
        start_descpose.rpy.rx = 2.243;
        start_descpose.rpy.ry = 0.828;
        start_descpose.rpy.rz = -148.894;
        start_descpose.tran.x = -208.064;
        start_descpose.tran.y = 412.155;
        start_descpose.tran.z = 1.926;

        JointPos start_jointpose;
        start_jointpose.jPos[0] = -51.489;
        start_jointpose.jPos[1] = -105.721;
        start_jointpose.jPos[2] = 130.695;
        start_jointpose.jPos[3] = -108.338;
        start_jointpose.jPos[4] = -91.356;
        start_jointpose.jPos[5] = 62.014;

        DescPose end_descpose;
        end_descpose.rpy.rx = 2.346;
        end_descpose.rpy.ry = -3.633;
        end_descpose.rpy.rz = -106.313;
        end_descpose.tran.x = -425.087;
        end_descpose.tran.y = 389.637;
        end_descpose.tran.z = -9.249;

        JointPos end_jointpose;
        end_jointpose.jPos[0] = -47.137;
        end_jointpose.jPos[1] = -102.345;
        end_jointpose.jPos[2] = 127.607;
        end_jointpose.jPos[3] = -108.526;
        end_jointpose.jPos[4] = -91.407;
        end_jointpose.jPos[5] = 23.537;

        ExaxisPos ex_axis_pose;
        memset(&ex_axis_pose, 0, sizeof(ExaxisPos));
        DescPose offset_pose;
        memset(&offset_pose, 0, sizeof(DescPose));
        int retval = 0;

        retval = robot.WeldingSetCurrentRelation(0, 400, 0, 10);
        cout << "WeldingSetCurrentRelation retval is: " << retval << endl;

        retval = robot.WeldingSetVoltageRelation(0, 40, 0, 10);
        cout << "WeldingSetVoltageRelation retval is: " << retval << endl;

        retval = robot.WeldingGetCurrentRelation(&current_min, &current_max, &output_vmin, &output_vmax);
        cout << "WeldingGetCurrentRelation retval is: " << retval << endl;
        cout << "current min " << current_min << " current max " << current_max << " output vol min " << output_vmin << " output vol max "<< output_vmax<<endl;

        retval = robot.WeldingGetVoltageRelation(&vol_min, &vol_max, &output_vmin, &output_vmax);
        cout << "WeldingGetVoltageRelation retval is: " << retval << endl;
        cout << "vol min " << vol_min << " vol max " << vol_max << " output vol min " << output_vmin << " output vol max "<< output_vmax<<endl;

        retval = robot.WeldingSetCurrent(1, 100, 0);
        cout << "WeldingSetCurrent retval is: " << retval << endl;

        this_thread::sleep_for(chrono::seconds(3));

        retval = robot.WeldingSetVoltage(1, 10, 0);
        cout << "WeldingSetVoltage retval is: " << retval << endl;

        retval = robot.WeaveSetPara(0, 0, 2.0, 0, 10, 0, 0, 0, 0);
        cout << "retval is: " << retval << endl;

        retval = robot.MoveJ(&start_jointpose, &start_descpose, 1, 0, 50, 50, 50, &ex_axis_pose, 0, 0, &offset_pose);
        if (retval != 0)
        {
            cout << "movej fail " << retval << endl;
            return 0;
        }

        retval = robot.WeaveStart(0);
        cout << "retval is: " << retval << endl;

        retval = robot.MoveL(&end_jointpose, &end_descpose, 1, 0, 50, 50, 50, 0, &ex_axis_pose, 0, 0, &offset_pose);
        if (retval != 0)
        {
            cout << "MoveL fail " << retval << endl;
            robot.WeaveEnd(0);
            return 0;
        }

        retval = robot.WeaveEnd(0);
        cout << "retval is: " << retval << endl;

        retval = 0;
        retval = robot.SetForwardWireFeed(1, 1);
        cout << "SetForwardWireFeed retval is: " << retval << endl;

        this_thread::sleep_for(chrono::seconds(3));

        retval = robot.SetForwardWireFeed(1, 0);
        cout << "SetForwardWireFeed retval is: " << retval << endl;

        retval = robot.SetReverseWireFeed(1, 1);
        cout << "SetReverseWireFeed retval is: " << retval << endl;

        this_thread::sleep_for(chrono::seconds(3));

        retval = robot.SetReverseWireFeed(1, 0);
        cout << "SetReverseWireFeed retval is: " << retval << endl;

        retval = robot.SetAspirated(1, 1);
        cout << "SetAspirated retval " << retval << endl;

        this_thread::sleep_for(chrono::seconds(2));

        retval = robot.SetAspirated(1, 0);
        cout << "SetAspirated retval " << retval << endl;

        /* All coordinate points are subject to actual working conditions. */
        start_descpose.rpy.rx = 7.178;
        start_descpose.rpy.ry = -0.809;
        start_descpose.rpy.rz = -133.134;
        start_descpose.tran.x = -135.56;
        start_descpose.tran.y = 373.448;
        start_descpose.tran.z = 36.767;

        start_jointpose.jPos[0] = -70.228;
        start_jointpose.jPos[1] = -130.911;
        start_jointpose.jPos[2] = 134.147;
        start_jointpose.jPos[3] = -83.379;
        start_jointpose.jPos[4] = -95.656;
        start_jointpose.jPos[5] = 27.74;

        end_descpose.rpy.rx = -4.586;
        end_descpose.rpy.ry = -10.926;
        end_descpose.rpy.rz = -124.298;
        end_descpose.tran.x = -380.207;
        end_descpose.tran.y = 371.358;
        end_descpose.tran.z = 55.898;

        end_jointpose.jPos[0] = -50.247;
        end_jointpose.jPos[1] = -113.273;
        end_jointpose.jPos[2] = 125.856;
        end_jointpose.jPos[3] = -100.351;
        end_jointpose.jPos[4] = -80.702;
        end_jointpose.jPos[5] = 38.478;

        memset(&ex_axis_pose, 0, sizeof(ExaxisPos));
        memset(&offset_pose, 0, sizeof(DescPose));
        retval = 0;

        retval = robot.SegmentWeldStart(&start_descpose, &end_descpose, &start_jointpose, &end_jointpose, 20, 20, 1, 0, 5000, 1, 0, 1, 0, 20, 50, 50, 0, &ex_axis_pose, 0, 0, &offset_pose);
        if(0 != retval)
        {
            cout << "SegmentWeldStart end " << retval << endl;
        }

        return 0;
    }
