Extended Axis
=================

.. toctree:: 
    :maxdepth: 5

Set 485 Extended Axis Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set 485 extended axis parameters
    * @param [in] servoId Servo driver ID, range [1-16], corresponding to slave ID
    * @param [in] param 485 extended axis parameters
    * @return Error code 
    */
    int AuxServoSetParam(int servoId, Axis485Param param)
    
Get 485 Extended Axis Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get 485 extended axis configuration parameters
    * @param [in] servoId Servo driver ID, range [1-16], corresponding to slave ID
    * @param [out] param 485 extended axis parameters
    * @return Error code 
    */
    int AuxServoGetParam(int servoId, Axis485Param param);

Set 485 Extended Axis Enable/Disable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set 485 extended axis enable/disable
    * @param [in] servoId Servo driver ID, range [1-16], corresponding to slave ID
    * @param [in] status Enable status, 0-disable, 1-enable
    * @return Error code 
    */
    int AuxServoEnable(int servoId, int status);
        
Set 485 Extended Axis Control Mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set 485 extended axis control mode
    * @param [in] servoId Servo driver ID, range [1-16], corresponding to slave ID
    * @param [in] mode Control mode, 0-position mode, 1-speed mode
    * @return Error code 
    */
    int AuxServoSetControlMode(int servoId, int mode);

Set 485 Extended Axis Target Position (Position Mode)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set 485 extended axis target position (position mode)
    * @param [in] servoId Servo driver ID, range [1-16], corresponding to slave ID
    * @param [in] pos Target position, mm or °
    * @param [in] speed Target speed, mm/s or °/s
    * @param [in] acc Acceleration percentage [0-100]
    * @return Error code 
    */
    int AuxServoSetTargetPos(int servoId, double pos, double speed, double acc);
    
Set 485 Extended Axis Target Torque (Torque Mode) - Not Yet Available
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set 485 extended axis target torque (torque mode) - Not yet available
    * @param [in] servoId Servo driver ID, range [1-16], corresponding to slave ID
    * @param [in] torque Target torque, Nm
    * @return Error code 
    */
    int AuxServoSetTargetTorque(int servoId, double torque);
        
Set 485 Extended Axis 
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set 485 extended axis homing
    * @param [in] servoId Servo driver ID, range [1-16], corresponding to slave ID
    * @param [in] mode Homing mode, 1-current position homing; 2-negative limit homing; 3-positive limit homing
    * @param [in] searchVel Homing speed, mm/s or °/s
    * @param [in] latchVel Latch speed, mm/s or °/s
    * @param [in] acc Acceleration percentage [0-100]
    * @return Error code 
    */
    int AuxServoHoming(int servoId, int mode, double searchVel, double latchVel);

Clear 485 Extended Axis Error Information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Clear 485 extended axis error information
    * @param [in] servoId Servo driver ID, range [1-16], corresponding to slave ID
    * @return Error code 
    */
    int AuxServoClearError(int servoId);

Get 485 Extended Axis Servo Status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get 485 extended axis servo status
    * @param [in] servoId Servo driver ID, range [1-16], corresponding to slave ID
    * @param [in] servoErrCode Servo driver error code
    * @param [in] servoState Servo driver status bit0:0-disabled; 1-enabled; bit1:0-not moving; 1-moving; bit4 0-positioning not completed; 1-positioning completed; bit5:0-homing not completed; 1-homing completed
    * @param [in] servoPos Servo current position mm or °
    * @param [in] servoSpeed Servo current speed mm/s or °/s
    * @param [in] servoTorque Servo current torque Nm
    * @return Error code 
    */
    int AuxServoGetStatus(int servoId, int[] servoErrCode, int[] servoState, double[] servoPos, double[] servoSpeed, double[] servoTorque)

Set 485 Extended Axis Target Speed (Speed Mode)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set 485 extended axis target speed (speed mode)
    * @param [in] servoId Servo driver ID, range [1-16], corresponding to slave ID
    * @param [in] speed Target speed, mm/s or °/s
    * @param [in] acc Acceleration percentage [0-100]
    * @return Error code 
    */
    int AuxServoSetTargetSpeed(int servoId, double speed, double acc);

Set Axis Number for Status Feedback of 485 Extended Axis
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set axis number for status feedback of 485 extended axis
    * @param [in] servoId Servo driver ID, range [1-16], corresponding to slave ID
    * @return Error code 
    */
    int AuxServosetStatusID(int servoId);

Set 485 Extended Axis Motion Acceleration/Deceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set 485 extended axis motion acceleration/deceleration
    * @param [in] acc 485 extended axis motion acceleration
    * @param [in] dec 485 extended axis motion deceleration
    * @return Error code 
    */
    int AuxServoSetAcc(double acc, double dec)

Set 485 Extended Axis Emergency Stop Acceleration/Deceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set 485 extended axis emergency stop acceleration/deceleration
    * @param [in] acc 485 extended axis emergency stop acceleration
    * @param [in] dec 485 extended axis emergency stop deceleration
    * @return Error code 
    */
    int AuxServoSetEmergencyStopAcc(double acc, double dec)

Get 485 Extended Axis Motion Acceleration/Deceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get 485 extended axis motion acceleration/deceleration
    * @return List[0]: Error code; List[1]: 485 extended axis motion acceleration; List[2]: 485 extended axis motion deceleration 
    */
    List<Number> AuxServoGetAcc()

Get 485 Extended Axis Emergency Stop Acceleration/Deceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get 485 extended axis emergency stop acceleration/deceleration
    * @return List[0]: Error code; List[1]: 485 extended axis emergency stop acceleration; List[2]: 485 extended axis emergency stop deceleration
    */
    List<Number> AuxServoGetEmergencyStopAcc()

Extended Axis Control Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int Test485Auxservo(Robot robot)
    {
        Axis485Param ax=new Axis485Param(1, 1, 1, 131072, 15.45);
        int retval = robot.AuxServoSetParam(1, ax);

        Axis485Param ax2=new Axis485Param();
        retval = robot.AuxServoGetParam(1, ax2);

        ax.servoCompany=10;
        ax.servoModel=11;
        ax.servoSoftVersion=12;
        ax.servoResolution=13;
        ax.axisMechTransRatio=14;

        retval = robot.AuxServoSetParam(1, ax);

        retval = robot.AuxServoGetParam(1,ax2);

        ax.servoCompany=1;
        ax.servoModel=1;
        ax.servoSoftVersion=1;
        ax.servoResolution=131072;
        ax.axisMechTransRatio=36;

        retval = robot.AuxServoSetParam(1, ax);
        robot.Sleep(3000);

        robot.AuxServoSetAcc(3000, 3000);
        robot.AuxServoSetEmergencyStopAcc(5000, 5000);
        robot.Sleep(1000);
        double emagacc = 0, acc = 0;
        double emagdec = 0, dec = 0;

        List<Number> aux=new ArrayList<>();

        aux=robot.AuxServoGetEmergencyStopAcc();
        aux=robot.AuxServoGetAcc();

        robot.AuxServoSetControlMode(1, 0);
        robot.Sleep(2000);

        retval = robot.AuxServoEnable(1, 0);
        robot.Sleep(1000);
        int[] servoerrcode =new int[]{0};
        int[] servoErrCode=new int[]{0};
        int[] servoState=new int[]{0};
        double[] servoPos=new double[]{0};
        double[] servoSpeed=new double[]{0};
        double[] servoTorque=new double[]{0};
        retval = robot.AuxServoGetStatus(1, servoErrCode, servoState, servoPos, servoSpeed, servoTorque);
        robot.Sleep(1000);;

        retval = robot.AuxServoEnable(1, 1);
        robot.Sleep(1000);
        retval = robot.AuxServoGetStatus(1, servoErrCode, servoState, servoPos, servoSpeed, servoTorque);
        robot.Sleep(1000);

        retval = robot.AuxServoHoming(1, 1, 5, 1,100);
        robot.Sleep(3000);

        retval = robot.AuxServoSetTargetPos(1, 200, 30,100);
        robot.Sleep(1000);
        retval = robot.AuxServoGetStatus(1, servoErrCode, servoState, servoPos, servoSpeed, servoTorque);
        robot.Sleep(8000);


        robot.AuxServoSetControlMode(1, 1);
        robot.Sleep(2000);

        robot.AuxServoEnable(1, 0);
        robot.Sleep(1000);
        robot.AuxServoEnable(1, 1);
        robot.Sleep(1000);
        robot.AuxServoSetTargetSpeed(1, 100, 80);

        robot.Sleep(5000);
        robot.AuxServoSetTargetSpeed(1, 0, 80);

        robot.CloseRPC();
        return 0;
    }

UDP Extended Axis Communication Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Java SDK-v1.0.5-3.8.2

.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extended axis communication parameter configuration
    * @param [in] param Communication parameters
    * @return Error code
    */
    int ExtDevSetUDPComParam(UDPComParam param);     

Get UDP Extended Axis Communication Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get UDP extended axis communication parameters
    * @param [out] param Communication parameters
    * @return Error code
    */
    int ExtDevGetUDPComParam(UDPComParam param);       

Load UDP Communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Load UDP communication
    * @return Error code
    */
    int ExtDevLoadUDPDriver();

Unload UDP Communication
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Unload UDP communication
    * @return Error code
    */
    int ExtDevUnloadUDPDriver();

Reconnect After UDP Extended Axis Communication Exception Disconnection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Reconnect after UDP extended axis communication exception disconnection
    * @return Error code
    */
    int ExtDevUDPClientComReset();

Close Communication After UDP Extended Axis Communication Exception Disconnection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Close communication after UDP extended axis communication exception disconnection
    * @return Error code
    */
    int ExtDevUDPClientComClose();

UDP Extended Axis Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extended axis parameter configuration
    * @param [in] axisID Axis number
    * @param [in] axisType Extended axis type 0-translation; 1-rotation
    * @param [in] axisDirection Extended axis direction 0-forward; 1-reverse
    * @param [in] axisMax Extended axis maximum position mm
    * @param [in] axisMin Extended axis minimum position mm
    * @param [in] axisVel Speed mm/s
    * @param [in] axisAcc Acceleration mm/s2
    * @param [in] axisLead Lead mm
    * @param [in] encResolution Encoder resolution
    * @param [in] axisOffect Weld starting point extended axis offset
    * @param [in] axisCompany Driver manufacturer 1-Hechuan; 2-Huichuan; 3-Panasonic
    * @param [in] axisModel Driver model 1-Hechuan-SV-XD3EA040L-E, 2-Hechuan-SV-X2EA150A-A, 1-Huichuan-SV620PT5R4I, 1-Panasonic-MADLN15SG, 2-Panasonic-MSDLN25SG, 3-Panasonic-MCDLN35SG
    * @param [in] axisEncType Encoder type 0-incremental; 1-absolute
    * @return Error code
    */
    int ExtAxisParamConfig(int axisID, int axisType, int axisDirection, double axisMax, double axisMin, double axisVel, double axisAcc, double axisLead, int encResolution, double axisOffect, int axisCompany, int axisModel, int axisEncType);

Set Extended Axis Installation Position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set extended axis installation position
    * @param [in] installType 0-robot installed on external axis, 1-robot installed outside external axis
    * @return Error code
    */
    int SetRobotPosToAxis(int installType);

Set Extended Axis System DH Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set extended axis system DH parameter configuration
    * @param [in]  axisConfig External axis configuration, 0-single degree of freedom linear slide, 1-two degree of freedom L-type positioner, 2-three degree of freedom, 3-four degree of freedom, 4-single degree of freedom positioner
    * @param [in]  axisDHd1 External axis DH parameter d1 mm
    * @param [in]  axisDHd2 External axis DH parameter d2 mm
    * @param [in]  axisDHd3 External axis DH parameter d3 mm
    * @param [in]  axisDHd4 External axis DH parameter d4 mm
    * @param [in]  axisDHa1 External axis DH parameter a1 mm
    * @param [in]  axisDHa2 External axis DH parameter a2 mm
    * @param [in]  axisDHa3 External axis DH parameter a3 mm
    * @param [in]  axisDHa4 External axis DH parameter a4 mm
    * @return Error code
    */
    int SetAxisDHParaConfig(int axisConfig, double axisDHd1, double axisDHd2, double axisDHd3, double axisDHd4, double axisDHa1, double axisDHa2, double axisDHa3, double axisDHa4);

UDP Extended Axis Enable
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extended axis enable
    * @param [in] axisID Axis number [1-4]
    * @param [in] status 0-disable; 1-enable
    * @return Error code
    */
    int ExtAxisServoOn(int axisID, int status);

UDP Extended Axis Homing
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extended axis homing
    * @param [in] axisID Axis number [1-4]
    * @param [in] mode Homing method 0-current position homing, 1-negative limit homing, 2-positive limit homing
    * @param [in] searchVel Homing speed (mm/s)
    * @param [in] latchVel Homing latch speed (mm/s)
    * @return Error code
    */
    int ExtAxisSetHoming(int axisID, int mode, double searchVel, double latchVel);

UDP Extended Axis Jog Start
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extended axis jog start
    * @param [in] axisID Axis number [1-4]
    * @param [in] direction Rotation direction 0-reverse; 1-forward
    * @param [in] vel Speed (mm/s)
    * @param [in] acc Acceleration (mm/s2)
    * @param [in] maxDistance Maximum jog distance
    * @return Error code
    */
    int ExtAxisStartJog(int axisID, int direction, double vel, double acc, double maxDistance);
    
UDP Extended Axis Jog Stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extended axis jog stop
    * @param [in] axisID Axis number [1-4]
    * @return Error code
    */
    int ExtAxisStopJog(int axisID);

UDP Extended Axis Configuration and Jog Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: Java
    :linenos:

    public static int TestUDPAxis(Robot robot)//UDP
    {
        UDPComParam para1=new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1);
        int rtn = robot.ExtDevSetUDPComParam(para1);
        String ip = ""; int port = 0; int period = 0; int lossPkgTime = 0; int lossPkgNum = 0; int disconnectTime = 0; int reconnectEnable = 0; int reconnectPeriod = 0; int reconnectNum = 0;
        UDPComParam para2=new UDPComParam(ip, port, period, lossPkgTime, lossPkgNum, disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum,0);
        rtn = robot.ExtDevGetUDPComParam(para2);

        robot.ExtDevLoadUDPDriver();

        rtn = robot.ExtAxisServoOn(1, 1);
        rtn = robot.ExtAxisServoOn(2, 1);
        robot.Sleep(3000);

        robot.ExtAxisSetHoming(1, 0, 10, 2);
        robot.Sleep(3000);
        rtn = robot.ExtAxisSetHoming(2, 0, 10, 2);

        robot.Sleep(4000);

        rtn = robot.SetRobotPosToAxis(1);
        rtn = robot.SetAxisDHParaConfig(10, 20, 0, 0, 0, 0, 0, 0, 0);
        rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0);
        rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0);

        robot.Sleep(4000);
        robot.ExtAxisStartJog(1, 0, 10, 10, 30);
        robot.Sleep(4000);
        robot.ExtAxisStopJog(1);
        robot.Sleep(4000);
        robot.ExtAxisServoOn(1, 0);

        robot.Sleep(4000);
        robot.ExtAxisStartJog(2, 0, 10, 10, 30);
        robot.Sleep(4000);
        robot.ExtAxisStopJog(2);
        robot.Sleep(4000);
        robot.ExtAxisServoOn(2, 0);
        robot.Sleep(4000);
        robot.ExtDevUnloadUDPDriver();

        return 0;
    }

Set Extended Axis Coordinate System Reference Point - Four-Point Method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set extended axis coordinate system reference point - four-point method
    * @param [in]  pointNum Point number [1-4]
    * @return Error code
    */
    int ExtAxisSetRefPoint(int pointNum);

Calculate Extended Axis Coordinate System - Four-Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Calculate extended axis coordinate system - four-point method
    * @param [out]  coord Coordinate system value
    * @return Error code
    */
    int ExtAxisComputeECoordSys(DescPose coord);

Positioner Coordinate System Reference Point Setting
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Positioner coordinate system reference point setting
    * @param [in]  pointNum Point number [1-4]
    * @return Error code
    */
    int PositionorSetRefPoint(int pointNum);

Positioner Coordinate System Calculation - Four-Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Positioner coordinate system calculation - four-point method
    * @param [out] coord Coordinate system value
    * @return Error code
    */
    int PositionorComputeECoordSys(DescPose coord);

Set Calibration Reference Point Pose in Extended Axis End Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set calibration reference point pose in extended axis end coordinate system
    * @param [in] pos Pose value
    * @return Error code
    */
    int SetRefPointInExAxisEnd(DescPose pos);

Apply Extended Axis Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Apply extended axis coordinate system
    * @param [in]  applyAxisId Extended axis number bit0-bit3 corresponds to extended axis number 1-4, e.g., to apply extended axis 1 and 3, it is 0b 0000 0101; which is 5
    * @param [in]  axisCoordNum Extended axis coordinate system number
    * @param [in]  coord Coordinate system value
    * @param [in]  calibFlag Calibration flag 0-no, 1-yes
    * @return Error code
    */
    int ExtAxisActiveECoordSys(int applyAxisId, int axisCoordNum, DescPose coord, int calibFlag);

Get Extended Axis Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.5-3.8.2

.. code-block:: Java
    :linenos:

    /** 
    * @brief Get extended axis coordinate system
    * @param [out] coord Extended axis coordinate system
    * @return Error code
    */
    int ExtAxisGetCoord(DescPose coord);

Extended Axis Coordinate System Calibration Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestUDPAxisCalib(Robot robot)
    {
        UDPComParam para1=new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1);

        int rtn = robot.ExtDevSetUDPComParam(para1);
        String ip = ""; int port = 0; int period = 0; int lossPkgTime = 0; int lossPkgNum = 0; int disconnectTime = 0; int reconnectEnable = 0; int reconnectPeriod = 0; int reconnectNum = 0;
        UDPComParam para2=new UDPComParam(ip, port, period, lossPkgTime, lossPkgNum, disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum,0);

        rtn = robot.ExtDevGetUDPComParam(para2);

        robot.ExtDevLoadUDPDriver();

        rtn = robot.ExtAxisServoOn(1, 1);
        rtn = robot.ExtAxisServoOn(2, 1);

        robot.Sleep(4000);

        rtn = robot.SetRobotPosToAxis(1);
        rtn = robot.SetAxisDHParaConfig(1, 128.5, 206.4,  0, 0, 0, 0, 0, 0);
        rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0);
        rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0);

        DescPose toolCoord=new DescPose(0, 0, 210, 0, 0, 0);
        robot.SetToolCoord(1, toolCoord, 0, 0, 1, 0);

        JointPos jSafe=new JointPos(115.193, -96.149, 92.489, -87.068, -89.15, -83.488);
        JointPos j1=new JointPos(117.559, -92.624, 100.329, -96.909, -94.057, -83.488);
        JointPos j2=new JointPos(112.239, -90.096, 99.282, -95.909, -89.824, -83.488);
        JointPos j3=new JointPos(110.839, -83.473, 93.166, -89.22, -90.499, -83.487);
        JointPos j4=new JointPos(107.935, -83.572, 95.424, -92.873, -87.933, -83.488);

        DescPose descSafe =new DescPose(0,0,0,0,0,0);
        DescPose desc1 = new DescPose(0,0,0,0,0,0);
        DescPose desc2 = new DescPose(0,0,0,0,0,0);
        DescPose desc3 = new DescPose(0,0,0,0,0,0);
        DescPose desc4 = new DescPose(0,0,0,0,0,0);
        ExaxisPos exaxisPos =new ExaxisPos(0,0,0,0);
        DescPose offdese =new DescPose(0, 0, 0, 0, 0, 0);

        robot.GetForwardKin(jSafe, descSafe);
        robot.MoveJ(jSafe, descSafe, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.Sleep(2000);

        robot.GetForwardKin(j1, desc1);
        robot.MoveJ(j1, desc1, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.Sleep(2000);

        DescPose actualTCPPos =new DescPose(0,0,0,0,0,0);

        robot.GetActualTCPPose(actualTCPPos);
        robot.SetRefPointInExAxisEnd(actualTCPPos);
        rtn = robot.PositionorSetRefPoint(1);
        robot.Sleep(2000);

        robot.MoveJ(jSafe, descSafe, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.ExtAxisStartJog(1, 0, 50, 50, 10);
        robot.Sleep(1000);
        robot.ExtAxisStartJog(2, 0, 50, 50, 10);
        robot.Sleep(1000);
        robot.GetForwardKin(j2, desc2);
        rtn = robot.MoveJ(j2, desc2, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        rtn = robot.PositionorSetRefPoint(2);
        robot.Sleep(2000);

        robot.MoveJ(jSafe, descSafe, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.ExtAxisStartJog(1, 0, 50, 50, 10);
        robot.Sleep(1000);
        robot.ExtAxisStartJog(2, 0, 50, 50, 10);
        robot.Sleep(1000);
        robot.GetForwardKin(j3, desc3);
        robot.MoveJ(j3, desc3, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        rtn = robot.PositionorSetRefPoint(3);
        robot.Sleep(2000);

        robot.MoveJ(jSafe, descSafe, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.ExtAxisStartJog(1, 0, 50, 50, 10);
        robot.Sleep(1000);
        robot.ExtAxisStartJog(2, 0, 50, 50, 10);
        robot.Sleep(1000);
        robot.GetForwardKin(j4, desc4);
        robot.MoveJ(j4, desc4, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        rtn = robot.PositionorSetRefPoint(4);
        robot.Sleep(2000);

        DescPose axisCoord = new DescPose();
        robot.PositionorComputeECoordSys(axisCoord);
        robot.MoveJ(jSafe, descSafe, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        rtn = robot.ExtAxisActiveECoordSys(3, 1, axisCoord, 1);

        robot.CloseRPC();
        return 0;
    }

UDP Extended Axis Motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: Java SDK-v1.0.7-3.8.4

.. code-block:: Java
    :linenos:

    /**
    *@ brief UDP Extended Axis Motion
    *@ param [in] pos target location
    *@ param [in] ovl speed percentage
    *@ param [in] blend smoothing parameter (mm or ms)
    *@ return error code
    */
    int ExtAxisMove(ExaxisPos pos, double ovl, double blend);

UDP Extended Axis Motion Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestUDPAxisCalib(Robot robot)
    {
        ExaxisPos exaxisPos = new ExaxisPos( 20, 0, 0, 0 );
        robot.ExtAxisMove(exaxisPos,40);
        robot.CloseRPC();
        return 0;
    }

UDP Extended Axis and Robot Joint Motion Synchronous Motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extended axis and robot joint motion synchronous motion
    * @param [in] joint_pos  Target joint position, unit deg
    * @param [in] desc_pos   Target Cartesian pose
    * @param [in] tool  Tool coordinate number, range [0~14]
    * @param [in] user  Workpiece coordinate number, range [0~14]
    * @param [in] vel  Speed percentage, range [0~100]
    * @param [in] acc  Acceleration percentage, range [0~100], not yet available
    * @param [in] ovl  Speed scaling factor, range [0~100]
    * @param [in] epos  Extended axis position, unit mm
    * @param [in] blendT [-1.0]-motion到位(blocking), [0~500.0]-smoothing time (non-blocking), unit ms
    * @param [in] offset_flag  0-no offset, 1-offset in base coordinate system/workpiece coordinate system, 2-offset in tool coordinate system
    * @param [in] ffset_pos  Pose offset
    * @return Error code
    */
    int ExtAxisSyncMoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, ExaxisPos epos, double blendT, int offset_flag, DescPose offset_pos);

UDP Extended Axis and Robot Joint Motion Synchronous Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public int testSyncMoveJ(Robot robot)
    {
        //1. Calibrate and apply robot tool coordinate system. You can use the four-point or six-point method for tool coordinate system calibration and application. The interfaces involved are as follows:
        //  int SetToolPoint(int point_num); //Set tool reference point - six-point method
        //  int ComputeTool(ref DescPose tcp_pose); //Calculate tool coordinate system
        //  int SetTcp4RefPoint(int point_num);  //Set tool reference point - four-point method
        //  int ComputeTcp4(ref DescPose tcp_pose);  //Calculate tool coordinate system - four-point method
        //  int SetToolCoord(int id, DescPose coord, int type, int install); //Set and apply tool coordinate system
        //  int SetToolList(int id, DescPose coord, int type, int install);  //Set and apply tool coordinate system list
        //2. Set UDP communication parameters and load UDP communication
        UDPComParam param=new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10,0);
        robot.ExtDevSetUDPComParam(param);
        robot.ExtDevLoadUDPDriver();
        //3. Set extended axis parameters, including extended axis type, extended axis driver parameters, and extended axis DH parameters
        robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //Single-axis positioner and DH parameters
        robot.SetRobotPosToAxis(1); //Extended axis installation position
        robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //Servo driver parameters. This example is for a single-axis positioner, so only one driver parameter needs to be set. If you choose an extended axis type with multiple axes, each axis needs to be configured.
        //4. Enable the selected axis and perform homing
        robot.ExtAxisServoOn(1, 0);
        robot.ExtAxisSetHoming(1, 0, 20, 3);
        //5. Perform extended axis coordinate system calibration and application
        DescPose pos = new DescPose(/* Enter your calibration point coordinates */ );
        robot.SetRefPointInExAxisEnd(pos);
        robot.PositionorSetRefPoint(1); /*You need to call this interface 4 times with different points to complete the calibration */
        DescPose coord = new DescPose();
        robot.PositionorComputeECoordSys(coord); //Calculate extended axis calibration result
        robot.ExtAxisActiveECoordSys(1, 1, coord, 1); //Apply the calibration result to the extended axis coordinate system
        //6. Calibrate the workpiece coordinate system on the extended axis. You will need the following interfaces:
        //int SetWObjCoordPoint(int point_num);
        //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
        //int SetWObjCoord(int id, DescPose coord);
        //int SetWObjList(int id, DescPose coord);
        //7. Record your synchronous joint motion starting point
        DescPose startdescPose = new DescPose(/*Enter your coordinates*/ );
        JointPos startjointPos = new JointPos(/*Enter your coordinates*/ );
        ExaxisPos startexaxisPos = new ExaxisPos(/* Enter your extended axis starting point coordinates */ );
        //8. Record your synchronous joint motion end point coordinates
        DescPose enddescPose = new DescPose(/*Enter your coordinates*/ );
        JointPos endjointPos = new JointPos(/*Enter your coordinates*/ );
        ExaxisPos endexaxisPos =new ExaxisPos(/* Enter your extended axis end point coordinates */);
        //9. Write the synchronous motion program
        //Move to the starting point, assuming the applied tool coordinate system and workpiece coordinate system are both 1
        robot.ExtAxisMove(startexaxisPos, 20);
        DescPose offdese = new DescPose( 0, 0, 0, 0, 0, 0 );
        robot.MoveJ(startjointPos, startdescPose, 1, 1, 100, 100, 100, startexaxisPos, 0, 0, offdese);
        //Start synchronous motion
        robot.ExtAxisSyncMoveJ(endjointPos, enddescPose, 1, 1, 100, 100, 100, endexaxisPos, -1, 0, offdese);
        robot.CloseRPC();
        return 0;
    }

UDP Extended Axis and Robot Linear Motion Synchronous Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extended axis and robot linear motion synchronous motion
    * @param [in] joint_pos  Target joint position, unit deg
    * @param [in] desc_pos   Target Cartesian pose
    * @param [in] tool  Tool coordinate number, range [0~14]
    * @param [in] user  Workpiece coordinate number, range [0~14]
    * @param [in] vel  Speed percentage, range [0~100]
    * @param [in] acc  Acceleration percentage, range [0~100], not yet available
    * @param [in] ovl  Speed scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-motion到位(blocking), [0~1000.0]-smoothing radius (non-blocking), unit mm
    * @param [in] epos  Extended axis position, unit mm
    * @param [in] offset_flag  0-no offset, 1-offset in base coordinate system/workpiece coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos  Pose offset
    * @return Error code
    */
    int ExtAxisSyncMoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendR, ExaxisPos epos, int offset_flag, DescPose offset_pos);

UDP Extended Axis and Robot Linear Motion Synchronous Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public int testSyncMoveL(Robot robot)
    {
        //1. Calibrate and apply robot tool coordinate system. You can use the four-point or six-point method for tool coordinate system calibration and application. The interfaces involved are as follows:
        //  int SetToolPoint(int point_num); //Set tool reference point - six-point method
        //  int ComputeTool(ref DescPose tcp_pose); //Calculate tool coordinate system
        //  int SetTcp4RefPoint(int point_num);  //Set tool reference point - four-point method
        //  int ComputeTcp4(ref DescPose tcp_pose);  //Calculate tool coordinate system - four-point method
        //  int SetToolCoord(int id, DescPose coord, int type, int install); //Set and apply tool coordinate system
        //  int SetToolList(int id, DescPose coord, int type, int install);  //Set and apply tool coordinate system list
        //2. Set UDP communication parameters and load UDP communication
        UDPComParam param=new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10,0);
        robot.ExtDevSetUDPComParam(param);
        robot.ExtDevLoadUDPDriver();
        //3. Set extended axis parameters, including extended axis type, extended axis driver parameters, and extended axis DH parameters
        robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //Single-axis positioner and DH parameters
        robot.SetRobotPosToAxis(1); //Extended axis installation position
        robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //Servo driver parameters. This example is for a single-axis positioner, so only one driver parameter needs to be set. If you choose an extended axis type with multiple axes, each axis needs to be configured.
        //4. Enable the selected axis and perform homing
        robot.ExtAxisServoOn(1, 0);
        robot.ExtAxisSetHoming(1, 0, 20, 3);
        //5. Perform extended axis coordinate system calibration and application
        DescPose pos = new DescPose(/* Enter your calibration point coordinates */ );
        robot.SetRefPointInExAxisEnd(pos);
        robot.PositionorSetRefPoint(1); /*You need to call this interface 4 times with different points to complete the calibration */
        DescPose coord = new DescPose();
        robot.PositionorComputeECoordSys(coord); //Calculate extended axis calibration result
        robot.ExtAxisActiveECoordSys(1, 1, coord, 1); //Apply the calibration result to the extended axis coordinate system
        //6. Calibrate the workpiece coordinate system on the extended axis. You will need the following interfaces:
        //int SetWObjCoordPoint(int point_num);
        //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
        //int SetWObjCoord(int id, DescPose coord);
        //int SetWObjList(int id, DescPose coord);
        //7. Record your synchronous joint motion starting point
        DescPose startdescPose = new DescPose(/*Enter your coordinates*/ );
        JointPos startjointPos = new JointPos(/*Enter your coordinates*/ );
        ExaxisPos startexaxisPos = new ExaxisPos(/* Enter your extended axis starting point coordinates */ );
        //8. Record your synchronous joint motion end point coordinates
        DescPose enddescPose = new DescPose(/*Enter your coordinates*/ );
        JointPos endjointPos = new JointPos(/*Enter your coordinates*/ );
        ExaxisPos endexaxisPos =new ExaxisPos(/* Enter your extended axis end point coordinates */);
        //9. Write the synchronous motion program
        //Move to the starting point, assuming the applied tool coordinate system and workpiece coordinate system are both 1
        robot.ExtAxisMove(startexaxisPos, 20);
        DescPose offdese = new DescPose( 0, 0, 0, 0, 0, 0 );
        robot.MoveJ(startjointPos, startdescPose, 1, 1, 100, 100, 100, startexaxisPos, 0, 0, offdese);
        //Start synchronous motion
        robot.ExtAxisSyncMoveL(endjointPos, enddescPose, 1, 1, 100, 100, 100, 0, endexaxisPos, 0, offdese);
        robot.CloseRPC();
        return 0;
    }

UDP Extended Axis and Robot Arc Motion Synchronous Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extended axis and robot arc motion synchronous motion
    * @param [in] joint_pos_p  Path point joint position, unit deg
    * @param [in] desc_pos_p   Path point Cartesian pose
    * @param [in] ptool  Tool coordinate number, range [0~14]
    * @param [in] puser  Workpiece coordinate number, range [0~14]
    * @param [in] pvel  Speed percentage, range [0~100]
    * @param [in] pacc  Acceleration percentage, range [0~100], not yet available
    * @param [in] epos_p  Intermediate point extended axis position, unit mm
    * @param [in] poffset_flag  0-no offset, 1-offset in base coordinate system/workpiece coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos_p  Pose offset
    * @param [in] joint_pos_t  Target point joint position, unit deg
    * @param [in] desc_pos_t   Target point Cartesian pose
    * @param [in] ttool  Tool coordinate number, range [0~14]
    * @param [in] tuser  Workpiece coordinate number, range [0~14]
    * @param [in] tvel  Speed percentage, range [0~100]
    * @param [in] tacc  Acceleration percentage, range [0~100], not yet available
    * @param [in] epos_t  Extended axis position, unit mm
    * @param [in] toffset_flag  0-no offset, 1-offset in base coordinate system/workpiece coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos_t  Pose offset
    * @param [in] ovl  Speed scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-motion到位(blocking), [0~1000.0]-smoothing radius (non-blocking), unit mm
    * @return Error code
    */
    int ExtAxisSyncMoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, double pvel, double pacc, ExaxisPos epos_p, int poffset_flag, DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, double tvel, double tacc, ExaxisPos epos_t, int toffset_flag, DescPose offset_pos_t, double ovl, double blendR);

UDP Extended Axis and Robot Arc Motion Synchronous Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public int testSyncMoveC(Robot robot)
    {
        //1. Calibrate and apply robot tool coordinate system. You can use the four-point or six-point method for tool coordinate system calibration and application. The interfaces involved are as follows:
        //  int SetToolPoint(int point_num); //Set tool reference point - six-point method
        //  int ComputeTool(ref DescPose tcp_pose); //Calculate tool coordinate system
        //  int SetTcp4RefPoint(int point_num);  //Set tool reference point - four-point method
        //  int ComputeTcp4(ref DescPose tcp_pose);  //Calculate tool coordinate system - four-point method
        //  int SetToolCoord(int id, DescPose coord, int type, int install); //Set and apply tool coordinate system
        //  int SetToolList(int id, DescPose coord, int type, int install);  //Set and apply tool coordinate system list
        //2. Set UDP communication parameters and load UDP communication
        UDPComParam param=new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10,0);
        robot.ExtDevSetUDPComParam(param);
        robot.ExtDevLoadUDPDriver();
        //3. Set extended axis parameters, including extended axis type, extended axis driver parameters, and extended axis DH parameters
        robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //Single-axis positioner and DH parameters
        robot.SetRobotPosToAxis(1); //Extended axis installation position
        robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //Servo driver parameters. This example is for a single-axis positioner, so only one driver parameter needs to be set. If you choose an extended axis type with multiple axes, each axis needs to be configured.
        //4. Enable the selected axis and perform homing
        robot.ExtAxisServoOn(1, 0);
        robot.ExtAxisSetHoming(1, 0, 20, 3);
        //5. Perform extended axis coordinate system calibration and application
        DescPose pos = new DescPose(/* Enter your calibration point coordinates */ );
        robot.SetRefPointInExAxisEnd(pos);
        robot.PositionorSetRefPoint(1); /*You need to call this interface 4 times with different points to complete the calibration */
        DescPose coord = new DescPose();
        robot.PositionorComputeECoordSys(coord); //Calculate extended axis calibration result
        robot.ExtAxisActiveECoordSys(1, 1, coord, 1); //Apply the calibration result to the extended axis coordinate system
        //6. Calibrate the workpiece coordinate system on the extended axis. You will need the following interfaces:
        //int SetWObjCoordPoint(int point_num);
        //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
        //int SetWObjCoord(int id, DescPose coord);
        //int SetWObjList(int id, DescPose coord);
        //7. Record your synchronous arc motion starting point
        DescPose startdescPose = new DescPose(/*Enter your coordinates*/ );
        JointPos startjointPos = new JointPos(/*Enter your coordinates*/ );
        ExaxisPos startexaxisPos = new ExaxisPos(/* Enter your extended axis starting point coordinates */ );
        //8. Record your synchronous arc motion end point coordinates
        DescPose enddescPose = new DescPose(/*Enter your coordinates*/ );
        JointPos endjointPos = new JointPos(/*Enter your coordinates*/ );
        ExaxisPos endexaxisPos = new ExaxisPos(/* Enter your extended axis end point coordinates */ );
        //9. Record your synchronous arc motion intermediate point coordinates
        DescPose middescPose = new DescPose(/*Enter your coordinates*/ );
        JointPos midjointPos =new JointPos(/*Enter your coordinates*/ );
        ExaxisPos midexaxisPos = new ExaxisPos(/* Enter the extended axis coordinates when the robot is at the arc intermediate point */ );
        //10. Write the synchronous motion program
        //Move to the starting point, assuming the applied tool coordinate system and workpiece coordinate system are both 1
        robot.ExtAxisMove(startexaxisPos, 20);
        DescPose offdese = new DescPose( 0, 0, 0, 0, 0, 0 );
        robot.MoveJ(startjointPos, startdescPose, 1, 1, 100, 100, 100, startexaxisPos, 0, 0, offdese);
        //Start synchronous motion
        robot.ExtAxisSyncMoveC(midjointPos, middescPose, 1, 1, 100, 100, midexaxisPos, 0, offdese, endjointPos, enddescPose, 1, 1, 100, 100, endexaxisPos, 0, offdese, 100, 0);
        robot.CloseRPC();
        return 0;
    }

Set Extended DO
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set extended DO
    * @param [in] DONum DO number
    * @param [in] bOpen Switch true-on; false-off
    * @param [in] smooth Whether to smooth
    * @param [in] block Whether to block
    * @return Error code
    */
    int SetAuxDO(int DONum, boolean bOpen, boolean smooth, boolean block);

Set Extended AO
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set extended AO
    * @param [in] AONum AO number
    * @param [in] value Analog value [0-4095]
    * @param [in] block Whether to block
    * @return Error code
    */
    int SetAuxAO(int AONum, double value, boolean block);

Set Extended DI Input Filter Time
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set extended DI input filter time
    * @param [in] filterTime Filter time (ms)
    * @return  Error code
    */
    int SetAuxDIFilterTime(int filterTime);

Set Extended AI Input Filter Time
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set extended AI input filter time
    * @param [in] AONum AO number
    * @param [in] filterTime Filter time (ms)
    * @return Error code
    */
    int SetAuxAIFilterTime(int AONum, int filterTime);

Wait for Extended DI Input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Wait for extended DI input
    * @param [in] DINum DI number
    * @param [in] bOpen Switch 0-off; 1-on
    * @param [in] time Maximum wait time (ms)
    * @param [in] errorAlarm Whether to continue motion
    * @return Error code
    */
    int WaitAuxDI(int DINum, boolean bOpen, int time, boolean errorAlarm);

Wait for Extended AI Input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Wait for extended AI input
    * @param [in] AINum AI number
    * @param [in] sign 0-greater than; 1-less than
    * @param [in] value AI value
    * @param [in] time Maximum wait time (ms)
    * @param [in] errorAlarm Whether to continue motion
    * @return Error code
    */
    int WaitAuxAI(int AINum, int sign, int value, int time, boolean errorAlarm);
    
Get Extended DI Value
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get extended DI value
    * @param [in] DINum DI number
    * @param [in] isNoBlock Whether to block
    * @return List[0]: Error code; List[1] : isOpen 0-off; 1-on
    */
    List<Integer> GetAuxDI(int DINum, boolean isNoBlock)

Get Extended AI Value
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get extended AI value
    * @param [in] AINum AI number
    * @param [in] isNoBlock Whether to block
    * @return List[0]: Error code; List[1] : value Input value
    */
    List<Integer> GetAuxAI(int AINum, boolean isNoBlock);

Extended IO Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestAuxDOAO(Robot robot)
    {
        for (int i = 0; i < 128; i++)
        {
            robot.SetAuxDO(i, true, false, true);
            robot.Sleep(100);
        }
        for (int i = 0; i < 128; i++)
        {
            robot.SetAuxDO(i, false, false, true);
            robot.Sleep(100);
        }

        for (int i = 0; i < 409; i++)
        {
            robot.SetAuxAO(0, i * 10, true);
            robot.SetAuxAO(1, 4095 - i * 10, true);
            robot.SetAuxAO(2, i * 10, true);
            robot.SetAuxAO(3, 4095 - i * 10, true);
            robot.Sleep(10);
        }

        robot.SetAuxDIFilterTime(10);
        robot.SetAuxAIFilterTime(0, 10);


        int curValue = -1;
        List<Integer> liter=new ArrayList<>();
        for (int i = 0; i < 4; i++)
        {
            liter = robot.GetAuxAI(i, true);
        }

        robot.WaitAuxDI(1, false, 1000, false);
        robot.WaitAuxAI(1, 1, 132, 1000, false);

        robot.CloseRPC();
        return 0;
    }

Movable Device Enable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Movable device enable
    * @param [in] enable false-disable; true-enable
    * @return Error code
    */
    int TractorEnable(Boolean enable);

Movable Device Homing
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Movable device homing
    * @return Error code
    */
    int TractorHoming();

Movable Device Linear Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Movable device linear motion
    * @param [in] distance Linear motion distance (mm)
    * @param [in] vel Linear motion speed percentage (0-100)
    * @return Error code
    */
    int TractorMoveL(double distance, double vel);

Movable Device Arc Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Movable device arc motion
    * @param [in] radio Arc motion radius (mm)
    * @param [in] angle Arc motion angle (°)
    * @param [in] vel Linear motion speed percentage (0-100)
    * @return Error code
    */
    int TractorMoveC(double radio, double angle, double vel);

Movable Device Stop Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Movable device stop motion
    * @return Error code
    */
    int TractorStop();

Movable Device Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set reconnection count and interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("RPC connection success");
        }
        else
        {
            System.out.println("RPC connection fail");
            return ;
        }
        UDPComParam param = new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevSetUDPComParam(param);//UDP extended axis communication
        robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
        robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
        robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0);

        robot.TractorEnable(false);
        robot.Sleep(2000);
        robot.TractorEnable(true);
        robot.Sleep(2000);
        robot.TractorHoming();

        robot.Sleep(2000);
        robot.TractorMoveL(100, 20);
        robot.Sleep(5000);
        robot.TractorMoveL(-100, 20);
        robot.Sleep(5000);
        robot.TractorMoveC(300, 90, 20);
        robot.Sleep(2000);
        robot.TractorStop();//Stop the device
        robot.TractorMoveC(300, -90, 20);
    }