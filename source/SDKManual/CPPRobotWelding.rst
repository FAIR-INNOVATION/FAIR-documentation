Robot welding
======================

.. toctree:: 
    :maxdepth: 5

Welding starts
++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Set weave parameters
	* @param [in] weaveNum parameters number
	* @param [in] weaveType weave type:0- plane triangular weave ; 1- vertical L-shaped triangular weave; 2- clockwise circular weave; 3-counterclockwise circular weave; 4-plane sine weave; 5-vertical L-shaped sine weave; 6- vertical triangular weave; 7- Vertical sine weave
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

.. versionadded:: C++ SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Set weave parameters in real time
	* @param [in] weaveNum parameters number
	* @param [in] weaveType weave type:0- plane triangular weave ; 1- vertical L-shaped triangular weave; 2- clockwise circular weave; 3-counterclockwise circular weave; 4-plane sine weave; 5-vertical L-shaped sine weave; 6- vertical triangular weave; 7- Vertical sine weave
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

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

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

.. versionadded:: C++ SDK-v2.1.1.0

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
	* @param [in] user Workpiece coordinate number, range [0~14]
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


Sets the detection parameters of unexpected interruption of robot welding arc
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
	 * @brief Sets the detection parameters of unexpected interruption of robot welding arc
	 * @param [in] checkEnable Whether the check is enabled. 0: Indicates that the function is disabled. 1- Enable
	 * @param [in] arcInterruptTimeLength Duration for confirming arc interruption (ms)
	 * @return Error code
    */
	errno_t WeldingSetCheckArcInterruptionParam(int checkEnable, int arcInterruptTimeLength);

Get the detection parameters of unexpected interruption of robot welding arc
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
	 * @brief Get the detection parameters of unexpected interruption of robot welding arc
	 * @param [out] checkEnable Whether the check is enabled. 0: Indicates that the function is disabled. 1- Enable
	 * @param [out] arcInterruptTimeLength Duration for confirming arc interruption (ms)
	 * @return Error code
    */
	errno_t WeldingGetCheckArcInterruptionParam(int* checkEnable, int* arcInterruptTimeLength);

Set the parameters of robot welding interruption recovery
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
	 * @brief Set the parameters of robot welding interruption recovery
	 * @param [in] enable Whether to enable welding interrupt recovery
	 * @param [in] length Weld overlap distance (mm)
	 * @param [in] velocity Percentage of velocity at which the robot returns to the rearcing point (0-100)
	 * @param [in] moveType Indicates how the robot moves to the rearcing point. 0-LIN; 1-PTP
	 * @return Error code
    */
	errno_t WeldingSetReWeldAfterBreakOffParam(int enable, double length, double velocity, int moveType);
    
Get robot welding interrupt recovery parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
	 * @brief Get robot welding interrupt recovery parameters
	 * @param [out] enable Whether to enable welding interrupt recovery
	 * @param [out] length Weld overlap distance (mm)
	 * @param [out] velocity Percentage of robot return to rearcing point (0-100)
	 * @param [out] moveType Indicates how the robot moves to the rearcing point. 0-LIN; 1-PTP
	 * @return Error code
    */
	errno_t WeldingGetReWeldAfterBreakOffParam(int* enable, double* length, double* velocity, int* moveType);

Sets the robot to resume welding after welding interruption
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
	 * @brief Sets the robot to resume welding after welding interruption
	 * @return Error code
    */
	errno_t WeldingStartReWeldAfterBreakOff();

Sets the robot to exit welding after welding interruption
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
	 * @brief Sets the robot to exit welding after welding interruption
	 * @return Error code
	 */
	errno_t WeldingAbortWeldAfterBreakOff();

Code example
********************
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    void TestReWeld(FRRobot* robot)
    {
        int rtn = -1;
        rtn = robot->WeldingSetCheckArcInterruptionParam(1, 200);
        printf("WeldingSetCheckArcInterruptionParam    %d\n", rtn);
        rtn = robot->WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0);
        printf("WeldingSetReWeldAfterBreakOffParam    %d\n", rtn);
        int enable = 0;
        double length = 0;
        double velocity = 0;
        int moveType = 0;
        int checkEnable = 0;
        int arcInterruptTimeLength = 0;
        rtn = robot->WeldingGetCheckArcInterruptionParam(&checkEnable, &arcInterruptTimeLength);
        printf("WeldingGetCheckArcInterruptionParam  checkEnable  %d   arcInterruptTimeLength  %d\n", checkEnable, arcInterruptTimeLength);
        rtn = robot->WeldingGetReWeldAfterBreakOffParam(&enable, &length, &velocity, &moveType);
        printf("WeldingGetReWeldAfterBreakOffParam  enable = %d, length = %lf, velocity = %lf, moveType = %d\n", enable, length, velocity, moveType);

        robot->ProgramLoad("/fruser/test.lua");
        robot->ProgramRun();

        robot->Sleep(5000);

        while (true)
        {
            ROBOT_STATE_PKG pkg = {};
            robot->GetRobotRealTimeState(&pkg);
            printf("welding breakoff state is     %d\n", pkg.weldingBreakOffState.breakOffState);
            if (pkg.weldingBreakOffState.breakOffState == 1)
            {
                printf("welding breakoff ! \n");
                robot->Sleep(2000);
                rtn = robot->WeldingStartReWeldAfterBreakOff();
                printf("WeldingStartReWeldAfterBreakOff    %d\n", rtn);
                break;
            }
            robot->Sleep(100);
        }
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

Weave Transition Start
+++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.0-3.8.0

.. code-block:: c++
    :linenos:

    /**
     * @brief  Weave transition start
     * @param  [in] weaveNum Weave number
     * @return  Error code
     */
    errno_t WeaveChangeStart(int weaveNum);

Weave Transition End
+++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.2.0-3.8.0

.. code-block:: c++
    :linenos:

    /**
     * @brief  Weave transition end
     * @return  Error code
     */
    errno_t WeaveChangeEnd();

Code Example
********************

.. code-block:: c++
    :linenos:
    
    void TestWeaveChange(FRRobot* robot)
    {
        DescPose p1Desc(-72.912, -587.664, 31.849, 43.283, -6.731, 15.068);
        JointPos p1Joint(74.620, -80.903, 94.608, -109.882, -90.436, -13.432);

        DescPose p2Desc(-104.915, -483.712, -25.231, 42.228, -6.572, 18.433);
        JointPos p2Joint(66.431, -92.875, 116.362, -120.516, -88.627, -24.731);

        DescPose p3Desc(-240.651, -483.840, -7.161, 46.577, -5.286, 8.318);
        JointPos p3Joint(56.457, -84.796, 104.618, -114.497, -92.422, -25.430);

        ExaxisPos exaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offdese(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        robot->WeldingSetVoltage(1, 19, 0, 0);
        robot->WeldingSetCurrent(1, 190, 0, 0);
        robot->MoveJ(&p1Joint, &p1Desc, 1, 1, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot->MoveL(&p2Joint, &p2Desc, 1, 1, 100, 100, 50, -1, &exaxisPos, 0, 0, &offdese);
        robot->ARCStart(1, 0, 10000);
        robot->ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
        robot->WeaveStart(0);
        robot->WeaveChangeStart(1);
        robot->MoveL(&p3Joint, &p3Desc, 1, 1, 100, 100, 1, -1, &exaxisPos, 0, 0, &offdese);
        robot->WeaveChangeEnd();
        robot->WeaveEnd(0);
        robot->ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
        robot->ARCEnd(1, 0, 10000);
    }