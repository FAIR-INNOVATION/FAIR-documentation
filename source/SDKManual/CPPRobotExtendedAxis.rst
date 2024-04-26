Extended Axis
=================

.. toctree:: 
    :maxdepth: 5

Set 485 extended axis parameters
++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:
     
    /**
      * @brief Set 485 extended axis parameters
      * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID
      * @param [in] servoCompany Servo drive manufacturer, 1-Dynatec
      * @param [in] servoModel servo drive model, 1-FD100-750C
      * @param [in] servoSoftVersion servo driver software version, 1-V1.0
      * @param [in] servoResolution encoder resolution
      * @param [in] axisMechTransRatio mechanical transmission ratio
      * @return error code
      */
    errno_t AuxServoSetParam(int servoId, int servoCompany, int servoModel, int servoSoftVersion, int servoResolution, double axisMechTransRatio);

Get 485 extended axis configuration parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
      * @brief Get 485 extended axis configuration parameters
      * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID
      * @param [out] servoCompany Servo drive manufacturer, 1-Dynatec
      * @param [out] servoModel Servo drive model, 1-FD100-750C
      * @param [out] servoSoftVersion servo driver software version, 1-V1.0
      * @param [out] servoResolution encoder resolution
      * @param [out] axisMechTransRatio mechanical transmission ratio
      * @return error code
      */
    errno_t AuxServoGetParam(int servoId, int* servoCompany, int* servoModel, int* servoSoftVersion, int* servoResolution, double* axisMechTransRatio);

Code example
+++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    #include "libfairino/robot.h"

    //  If using Windows, include the following header files
    #include <string.h>
    #include <windows.h>
    //  If using linux, include the following header files
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
        robot.LoggerInit();
        robot.SetLoggerLevel();
        robot.RPC("192.168.58.2"); 

        int retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 15.45);
        std::cout << "AuxServoSetParam is: " << retval << std::endl;

        int servoCompany;
        int servoModel;
        int servoSoftVersion;
        int servoResolution;
        double axisMechTransRatio;
        retval = robot.AuxServoGetParam(1, &servoCompany, &servoModel, &servoSoftVersion, &servoResolution, &axisMechTransRatio);
        std::cout << "servoCompany " << servoCompany<< "\n"
                << "servoModel " << servoModel << "\n"
                << "servoSoftVersion " << servoSoftVersion<< "\n"
                << "servoResolution " << servoResolution<< "\n"
                << "axisMechTransRatio "<<axisMechTransRatio<< "\n"
                << std::endl;

        retval = robot.AuxServoSetParam(1, 10, 11, 12, 13, 14);
        std::cout << "AuxServoSetParam is: " << retval << std::endl;

        retval = robot.AuxServoGetParam(1, &servoCompany, &servoModel, &servoSoftVersion, &servoResolution, &axisMechTransRatio);
        std::cout << "servoCompany " << servoCompany<< "\n"
            << "servoModel " << servoModel << "\n"
            << "servoSoftVersion " << servoSoftVersion<< "\n"
            << "servoResolution " << servoResolution<< "\n"
            << "axisMechTransRatio "<<axisMechTransRatio<< "\n"
            << std::endl;

        return 0;
    }

Set 485 expansion axis enable/disable
+++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
      * @brief Set 485 expansion axis enable/disable
      * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID
      * @param [in] status enable status, 0-disabled, 1-enabled
      * @return error code
      */
    errno_t AuxServoEnable(int servoId, int status);

Set 485 extended axis control mode
+++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
      * @brief Set 485 extended axis control mode
      * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID
      * @param [in] mode control mode, 0-position mode, 1-speed mode
      * @return error code
      */
    errno_t AuxServoSetControlMode(int servoId, int mode);

Set the 485 extended axis target position (position mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
      * @brief Set the 485 extended axis target position (position mode)
      * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID
      * @param [in] pos target position, mm or °
      * @param [in] speed target speed, mm/s or °/s
      * @return error code
      */
    errno_t AuxServoSetTargetPos(int servoId, double pos, double speed);

Set 485 extended axis target torque (torque mode)-- Not open yet
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
      * @brief Set 485 extended axis target torque (torque mode)
      * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID
      * @param [in] torque target torque, Nm
      * @return error code
      */
    errno_t AuxServoSetTargetTorque(int servoId, double torque);

Set 485 extended axis homing
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
      * @brief Set 485 extended axis homing
      * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID
      * @param [in] mode zero return mode, 0-current position return to zero; 1-limit return to zero
      * @param [in] searchVel zero return speed, mm/s or °/s
      * @param [in] latchVel hoop speed, mm/s or °/s
      * @return error code
      */
    errno_t AuxServoHoming(int servoId, int mode, double searchVel, double latchVel);

Clear 485 extended axis error message
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
      * @brief Clear 485 extended axis error message
      * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID
      * @return error code
      */
    errno_t AuxServoClearError(int servoId);

Get 485 extended axis servo status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
      * @brief Get 485 extended axis servo status
      * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID
      * @param [out] servoErrCode servo drive fault code
      * @param [out] servo drive status [decimal number converted to binary, bit0: 0-disable,1-enable;bit1:0-not running,1-running; bit2: 0-attch postive limit, 1-not touch negative limit; bit3:0-attach negative limit,1-not attach negative limit; bit4:0-not locate,1-loacated; bit5:0-not zero,1-have zero
      * @param [out] servoPos servo current position mm or °
      * @param [out] servoSpeed Servo current speed mm/s or °/s
      * @param [out] servoTorque Servo current torque Nm
      * @return error code
      */
    errno_t AuxServoGetStatus(int servoId, int* servoErrCode, int* servoState, double* servoPos, double* servoSpeed, double* servoTorque);

Code example
+++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    #include "libfairino/robot.h"

    //  If using Windows, include the following header files
    #include <string.h>
    #include <windows.h>
    //  If using linux, include the following header files
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
        robot.LoggerInit();
        robot.SetLoggerLevel();
        robot.RPC("192.168.58.2");
        int retval = 0;

        retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 36);
        std::cout << "AuxServoSetParam is: " << retval << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));

        retval = robot.AuxServoEnable(1, 0);
        std::cout << "AuxServoEnable disenable " << retval << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        int servoerrcode = 0;
        int servoErrCode;
        int servoState;
        double servoPos;
        double servoSpeed;
        double servoTorque;
        retval = robot.AuxServoGetStatus(1, &servoErrCode, &servoState, &servoPos, &servoSpeed, &servoTorque);
        std::cout << "AuxServoGetStatus servoState "<< servoState << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        retval = robot.AuxServoEnable(1, 1);
        std::cout << "AuxServoEnable enable " << retval << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        retval = robot.AuxServoGetStatus(1, &servoErrCode, &servoState, &servoPos, &servoSpeed, &servoTorque);
        std::cout << "AuxServoGetStatus servoState "<< servoState << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        retval = robot.AuxServoHoming(1, 1, 5, 1);
        std::cout << "AuxServoHoming " << retval << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));

        retval = robot.AuxServoSetTargetPos(1, 200, 30);
        std::cout << "AuxServoSetTargetPos " << retval << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        retval = robot.AuxServoGetStatus(1, &servoErrCode, &servoState, &servoPos, &servoSpeed, &servoTorque);
        std::cout << "AuxServoGetStatus servoSpeed "<< servoSpeed << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        return 0;
    }

Set the 485 extended axis target speed (speed mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
      * @brief Set the 485 extended axis target speed (speed mode)
      * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID
      * @param [in] speed target speed, mm/s or °/s
      * @return error code
      */
    errno_t AuxServoSetTargetSpeed(int servoId, double speed);

Set the 485 extended axis data axis number in status feedback
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:
    
    /**
      * @brief Set the 485 extended axis data axis number in status feedback
      * @param [in] servoId servo drive ID, range [1-15], corresponding slave ID
      * @return error code
      */
    errno_t AuxServosetStatusID(int servoId);

Get the real-time status structure of the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
      * @brief Get the real-time status structure of the robot
      * @param [out] pkg robot real-time status structure
      * @return error code
      */
    errno_t GetRobotRealTimeState(ROBOT_STATE_PKG *pkg);

Get the robot peripheral protocol
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:
    
    /**
      * @brief Get the robot peripheral protocol
      * @param [out] protocol Robot peripheral protocol number 4096-Extended axis control card; 4097-ModbusSlave; 4098-ModbusMaster
      * @return error code
      */
    errno_t GetExDevProtocol(int *protocol);

Set the robot peripheral protocol
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    /**
      * @brief Set the robot peripheral protocol
      * @param [in] protocol Robot peripheral protocol number 4096-Extended axis control card; 4097-ModbusSlave; 4098-ModbusMaster
      * @return error code
      */
    errno_t SetExDevProtocol(int protocol);

Code example
+++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.3.0

.. code-block:: c++
    :linenos:

    #include "libfairino/robot.h"

    //  If using Windows, include the following header files
    #include <string.h>
    #include <windows.h>
    //  If using linux, include the following header files
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
        robot.LoggerInit();
        robot.SetLoggerLevel();
        robot.RPC("192.168.58.2");
        int retval = 0;

        ROBOT_STATE_PKG robot_pkg;
        int i = 0;
        while (i < 5)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            memset(&robot_pkg, 0, sizeof(ROBOT_STATE_PKG));
            retval = robot.GetRobotRealTimeState(&robot_pkg);
            std::cout << "program_state " << (int)robot_pkg.program_state<< "\n"
                << "data_len " << (int)robot_pkg.data_len << "\n"
                << "robot_state " << (int)robot_pkg.robot_state << "\n"
                << "robot_mode " << (int)robot_pkg.robot_mode << std::endl;
            i++;
        }

        int protocol = 4096;
        retval = robot.SetExDevProtocol(protocol);
        std::cout << "SetExDevProtocol retval " << retval << std::endl;
        retval = robot.GetExDevProtocol(&protocol);
        std::cout << "GetExDevProtocol retval " << retval <<" protocol is: " << protocol << std::endl;

        return 0;
    }