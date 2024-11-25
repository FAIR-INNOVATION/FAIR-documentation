extended axis
=====================================

.. toctree:: 
    :maxdepth: 5

Setting the 485 Extended Axis Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting 485 extended axis parameters
    * @param [in] servoId Servo drive ID, range [1-16], corresponds to slave ID.
    * @param [in] param 485 extended axis parameter
    * @return error code 
    */
    int AuxServoSetParam(int servoId, Axis485Param param)
    
Getting 485 Extended Axis Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Getting 485 Extended Axis Configuration Parameters
    * @param [in] servoId Servo drive ID, range [1-16], corresponds to slave ID.
    * @param [out] param 485 extended axis parameter
    * @return error code 
    */
    int AuxServoGetParam(int servoId, Axis485Param param);

Setting the 485 expansion axis enable/disable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting 485 extension axis enable/disable
    * @param [in] servoId Servo drive ID, range [1-16], corresponds to slave ID.
    * @param [in] status enable status, 0-de-enable, 1-enable
    * @return error code 
    */
    int AuxServoEnable(int servoId, int status);
        
Setting the 485 Extended Axis Control Mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting the 485 Extended Axis Control Mode
    * @param [in] servoId Servo drive ID, range [1-16], corresponds to slave ID.
    * @param [in] mode Control mode, 0-position mode, 1-velocity mode
    * @return error code 
    */
    int AuxServoSetControlMode(int servoId, int mode).

Setting the 485 extended axis back to zero
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting the 485 extension axis back to zero.
    * @param [in] servoId Servo drive ID, range [1-16], corresponds to slave ID.
    * @param [in] mode Return to zero mode, 1-current position return to zero; 2-negative limit return to zero; 3-positive limit return to zero
    * @param [in] searchVel Return-to-zero velocity, mm/s or °/s
    * @param [in] latchVel Hoop speed, mm/s or °/s
    * @param [in] acc Acceleration percentage [0-100]
    * @return error code 
    */
    int AuxServoHoming(int servoId, int mode, double searchVel, double latchVel);

Setting the 485 extended axis target position (position mode)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting the 485 extended axis target position (position mode)
    * @param [in] servoId Servo drive ID, range [1-16], corresponds to slave ID.
    * @param [in] pos target position, mm or °
    * @param [in] speed target speed, mm/s or °/s
    * @param [in] acc Acceleration percentage [0-100]
    * @return error code 
    */
    int AuxServoSetTargetPos(int servoId, double pos, double speed, double acc);

Setting the 485 extended axis target speed (velocity mode)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting the 485 extended axis target speed (velocity mode)
    * @param [in] servoId Servo drive ID, range [1-16], corresponds to slave ID.
    * @param [in] speed target speed, mm/s or °/s
    * @param [in] acc Acceleration percentage [0-100]
    * @return error code 
    */
    int AuxServoSetTargetSpeed(int servoId, double speed, double acc);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        Axis485Param param = new Axis485Param();
        param.servoCompany = 1; // Servo drive manufacturer, 1 - Dynatec
        param.servoModel = 1; // Servo drive model, 1-FD100-750C
        param.servoSoftVersion = 1; // Servo drive software version, 1-V1.0
        param.servoResolution = 131072; // encoder resolution
        param.axisMechTransRatio = 13.45; // mechanical transmission ratio
        robot.AuxServoSetParam(1, param);//set 485 extended axis parameters

        robot.AuxServoGetParam(1, param);
        System.out.println("auxservo param servoCompany: " + param.servoCompany + " servoModel: " + param.servoModel + " param.servoSoftVersion: " + param. servoSoftVersion + " servoResolution: " + param.servoResolution + " axisMechTransRatio: " + param.axisMechTransRatio);
        
        robot.AuxServoSetControlMode(1, 1);
        robot.Sleep(2000);
        robot.AuxServoEnable(1, 0);
        robot.Sleep(3000);
        robot.AuxServoEnable(1, 1);
        robot.Sleep(2000);
        robot.AuxServoHoming(1, 1, 10, 10,100);
        robot.Sleep(5000);

        robot.AuxServoSetTargetSpeed(1, 100,100);
        robot.Sleep(3000);
        robot.AuxServoSetTargetSpeed(1, -200,100);
        robot.Sleep(3000);
        robot.AuxServoSetTargetSpeed(1, 0,100);
    }
    
Setting the 485 extended axis target torque (torque mode)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting 485 extended axis target torque (torque mode) - not yet available
    * @param [in] servoId Servo drive ID, range [1-16], corresponds to slave ID.
    * @param [in] torque Target torque, Nm
    * @return error code 
    */
    int AuxServoSetTargetTorque(int servoId, double torque).
        
Clearing 485 Expansion Axis Error Messages
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Clear 485 extended axis error message
    * @param [in] servoId Servo drive ID, range [1-16], corresponds to slave ID.
    * @return error code 
    */
    int AuxServoClearError(int servoId).

Setting the 485 Extended Axis Data Axis Number in Status Feedback
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
.. code-block:: Java
    :linenos:

    /** 
    * @brief Sets the 485 extended axis data axis number in the status feedback.
    * @param [in] servoId Servo drive ID, range [1-16], corresponds to slave ID.
    * @return error code 
    */
    int AuxServosetStatusID(int servoId).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.AuxServoSetControlMode(1, 1);
        robot.Sleep(2000);
        robot.AuxServoEnable(1, 0);
        robot.Sleep(3000);
        robot.AuxServoEnable(1, 1);
        robot.Sleep(2000);
        robot.AuxServoHoming(1, 1, 10, 10,100);
        robot.Sleep(5000);

        robot.AuxServoSetTargetSpeed(1, 40,100);
        robot.Sleep(3000);
        robot.AuxServoSetTargetSpeed(1, 40,100);

        robot.AuxServosetStatusID(1);

        while (true)
        {
            ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
            System.out.println("aux servo cur Pos : " + pkg.auxState.servoPos + " cur vel: " + pkg.auxState.servoVel);
            robot.Sleep(100);
        }
    }

Parameter configuration for UDP extended axis communication
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extended axis communication parameterization
    * @param [in] param communication parameters
    * @return error code
    */
    int ExtDevSetUDPComParam(UDPComParam param);     

Get UDP extended axis communication parameter configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get UDP extended axis communication parameters.
    * @param [out] param Communication parameters
    * @return error code
    */
    int ExtDevGetUDPComParam(UDPComParam param);       

Load UDP communication
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Load UDP communications
    * @return error code
    */
    int ExtDevLoadUDPDriver(); int ExtDevLoadUDPDriver(); int ExtDevLoadUDPDriver()

Offloading UDP communication
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Uninstalling UDP communication
    * @return error code
    */
    int ExtDevUnloadUDPDriver(); int ExtDevUnloadUDPDriver().

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        UDPComParam param = new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevSetUDPComParam(param);//udp extended axis communication

        UDPComParam getParam = new UDPComParam();
        robot.ExtDevGetUDPComParam(getParam);
        System.out.println(" " + getParam.ip + " , " + getParam.port + " , " + getParam.period + " , " + getParam.lossPkgTime + " , " + getParam.lossPkgNum + " , " + getParam.disconnectTime + " , " + getParam.reconnectEnable + " , " + getParam.reconnectPeriod + " , " + getParam.reconnectNum);
        robot.ExtDevUnloadUDPDriver();//unload UDP communication
        robot.Sleep(1000);
        robot.ExtDevLoadUDPDriver();//load UDP communication
        robot.Sleep(1000);
    }

UDP Extended Axis Communication Recovery after Abnormal Disconnection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP Extended Axis communication restored after abnormal disconnection
    * @return error code
    */
    int ExtDevUDPClientComReset();

UDP extension axis communication is closed after abnormal disconnection.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extended axis communication closed after abnormal disconnection of communication
    * @return error code
    */
    int ExtDevUDPClientComClose();

UDP Extended Axis Parameter Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP Extended Axis Parameter Configuration
    * @param [in] axisID axis number
    * @param [in] axisType Extended axis type 0-translation; 1-rotation
    * @param [in] axisDirection Extended axis direction 0-positive; 1-direction
    * @param [in] axisMax Extended axis maximum position mm
    * @param [in] axisMin Extended axis minimum position mm
    * @param [in] axisVel velocity mm/s
    * @param [in] axisAcc Acceleration mm/s2
    * @param [in] axisLead guide mm
    * @param [in] encResolution encoder resolution
    * @param [in] axisOffect Extended axis offset of weld start point
    * @param [in] axisCompany Drive Manufacturer 1-Hochuan; 2-Huichuan; 3-Panasonic
    * @param [in] axisModel Drive Model 1-Hochuan-SV-XD3EA040L-E, 2-Hochuan-SV-X2EA150A-A, 1-HuiChuan-SV620PT5R4I, 1-Panasonic-MADLN15SG, 2-Panasonic-MSDLN25SG, 3-Panasonic-MCDLN35SG
    * @param [in] axisEncType Encoder type 0-incremental; 1-absolute
    * @return error code
    */
    int ExtAxisParamConfig(int axisID, int axisType, int axisDirection, double axisMax, double axisMin, double axisVel, double axisAcc, double axisLead, int encResolution, double axisOffect, int axisCompany, int axisModel, int axisEncType);

Getting Extended Axis Drive Configuration Information
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get extended axis drive configuration information
    * @param [in] axisId axis number [1-4]
    * @return List[0]: error code List[1]: axisCompany drive manufacturers 1-Hochuan; 2-Huichuan; 3-Matsushita; ...
    * List[2]: axisModel Drive Model 1-Hochuan-SV-XD3EA040L-E, 2-Hochuan-SV-X2EA150A-A, 1-HuiChuan-SV620PT5R4I, 1-Panasonic-MADLN15SG, 2-Panasonic-MSDLN25SG, 3-Panasonic-MCDLN35SG
    * List[3]: axisEncType Encoder type 0-incremental; 1-absolute
    */
    List<Integer> GetExAxisDriverConfig(int axisId);

Setting the Extension Shaft Mounting Position
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the Extension Shaft Mounting Position
    * @param [in] installType 0 - robot mounted on external axis, 1 - robot mounted outside external axis
    * @return error code
    */
    int SetRobotPosToAxis(int installType).

Setting the extended axis system DH parameter configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting up the DH parameter configuration for the extended axis system
    * @param [in] axisConfig External axis configuration, 0-single degree of freedom linear slide, 1-two degree of freedom L-type translator, 2-three degree of freedom, 3-four degree of freedom, 4-single degree of freedom translator
    * @param [in] axisDHd1 external axisDH parameter d1 mm
    * @param [in] axisDHd2 external axisDH parameter d2 mm
    * @param [in] axisDHd3 external axisDH parameter d3 mm
    * @param [in] axisDHd4 external axisDH parameter d4 mm
    * @param [in] axisDHa1 external axisDH parameter11 mm
    * @param [in] axisDHa2 external axisDH parameter a2 mm
    * @param [in] axisDHa3 external axisDH parameter a3 mm
    * @param [in] axisDHa4 external axisDH parameter a4 mm
    * @return error code
    */
    int SetAxisDHParaConfig(int axisConfig, double axisDHd1, double axisDHd2, double axisDHd3, double axisDHd4, double axisDHa1, double axisDHa2, double axisDHa3, double axisDHa4);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.ExtAxisServoOn(1, 1);//extended axis 1 enable
        robot.ExtAxisServoOn(2, 1);//extended axis 2 enable
        robot.Sleep(1000);
        robot.ExtAxisSetHoming(1, 0, 10, 3);//1,2 extended axis are back to zero
        robot.ExtAxisSetHoming(2, 0, 10, 3);
        robot.Sleep(1000);

        int rtn = 0;
        rtn = robot.SetAxisDHParaConfig(1, 128.5, 206.4, 0, 0, 0, 0, 0, 0, 0);
        System.out.println("SetAxisDHParaConfig rtn is " + rtn);
        rtn = robot.SetRobotPosToAxis(1);
        System.out.println("SetRobotPosToAxis rtn is " + rtn);
        rtn = robot.ExtAxisParamConfig(1,1, 0, 50, -50, 1000, 1000, 1.905, 262144, 200, 0, 0, 0);
        System.out.println("ExtAxisParamConfig rtn is " + rtn);
        rtn = robot.ExtAxisParamConfig(2,2, 0, 1000, -1000, 1000, 1000, 1000, 4.444, 262144, 200, 0, 0, 0);
        System.out.println("ExtAxisParamConfig rtn is " + rtn);
    }

Setting the reference point of the extended axis coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the reference point of the extended axis coordinate system -- the four-point method
    * @param [in] pointNum pointNum [1-4]
    * @return error code
    */
    int ExtAxisSetRefPoint(int pointNum).

Calculating the Extended Axis Coordinate System - Four Point Method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Computing Extended Axis Coordinate Systems -- Four-Point Approach
    * @param [out] coord coordinate system value
    * @return error code
    */
    int ExtAxisComputeECoordSys(DescPose coord);

Applying the Extended Axis Coordinate System
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Applying the extended axis coordinate system
    * @param [in] applyAxisId extension axis number bit0-bit3 corresponds to extension axis numbers 1-4, e.g., 0b 0000 0101 if extension axes 1 and 3 are applied; that is, 5
    * @param [in] axisCoordNum extended axis coordinate system number
    * @param [in] coord coordinate system value
    * @param [in] calibFlag calibration flag 0-No, 1-Yes
    * @return error code
    */
    int ExtAxisActiveECoordSys(int applyAxisId, int axisCoordNum, DescPose coord, int calibFlag);;

Setting of the calibration reference point in the position in the coordinate system of the end of the translator
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the calibration reference point in the position in the coordinate system of the end of the translator
    * @param [in] pos position value
    * @return error code
    */
    int SetRefPointInExAxisEnd(DescPose pos);

Shifter coordinate system reference point setting
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting of reference points for the coordinate system of the translocator
    * @param [in] pointNum pointNum [1-4]
    * @return error code
    */
    int PositionorSetRefPoint(int pointNum).

Shifter Coordinate System Calculation - Four Point Method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Translator Coordinate System Calculation - Four Point Method
    * @param [out] coord coordinate system value
    * @return error code
    */
    int PositionorComputeECoordSys(DescPose coord).

End Sensor Register Write
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief End Sensor Register Writes
    * @param [in] devAddr device address number 0-255
    * @param [in] regHAddr register address high 8 bits
    * @param [in] regLAddr register address lower 8 bits
    * @param [in] regNum number of registers 0-255
    * @param [in] data1 Write register value 1
    * @param [in] data2 Write register value 2
    * @param [in] isNoBlock 0-blocking; 1-non-blocking
    * @return error code
    */
    int AxleSensorRegWrite(int devAddr, int regHAddr, int regLAddr, int regNum, int data1, int data2, int isNoBlock);

UDP Extended Axis Enable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP Extended Axis Enablement
    * @param [in] axisID axis number [1-4]
    * @param [in] status 0-de-enable; 1-enable
    * @return error code
    */
    int ExtAxisServoOn(int axisID, int status).

UDP Extended Axis Zero Return
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extension axis back to zero
    * @param [in] axisID axis number [1-4]
    * @param [in] mode Zero return mode 0 - current position zero return, 1 - negative limit zero return, 2 - positive limit zero return
    * @param [in] searchVel Zero search velocity (mm/s)
    * @param [in] latchVel Zeroing hoop speed (mm/s)
    * @return error code
    */
    int ExtAxisSetHoming(int axisID, int mode, double searchVel, double latchVel);

UDP Extended Axis Tap Start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP Extended Axis Tap Starts
    * @param [in] axisID axis number [1-4]
    * @param [in] direction direction of rotation 0-reverse; 1-forward
    * @param [in] vel velocity (mm/s)
    * @param [in] acc (acceleration mm/s2)
    * @param [in] maxDistance Maximum spotting distance
    * @return error code
    */
    int ExtAxisStartJog(int axisID, int direction, double vel, double acc, double maxDistance);
    
UDP Extended Axis Tap Stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP Extended Axis Tap Stop
    * @param [in] axisID axis number [1-4]
    * @return error code
    */
    int ExtAxisStopJog(int axisID).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }

        robot.ExtAxisServoOn(1, 1);
        robot.ExtAxisSetHoming(1, 0, 10, 3);
        robot.ExtAxisStartJog(1, 1, 100, 100, 100, 20);
        robot.Sleep(1000);
        robot.ExtAxisStopJog(1);
        robot.ExtAxisServoOn(1, 0);
    }

Setting the Extended DO
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief set up extended DO
    * @param [in] DONum DO No.
    * @param [in] bOpen switch true-on; false-off
    * @param [in] smooth Whether smooth or not
    * @param [in] block whether to block or not
    * @return error code
    */
    int SetAuxDO(int DONum, boolean bOpen, boolean smooth, boolean block);

Setting up Extended AO
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief set up extended AO
    * @param [in] AONum AO number
    * @param [in] value Analog value [0-4095]
    * @param [in] block whether to block or not
    * @return error code
    */
    int SetAuxAO(int AONum, double value, boolean block);

Setting the Extended DI Input Filter Time
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the extended DI input filter time
    * @param [in] filterTime filterTime (ms)
    * @return error code
    */
    int SetAuxDIFilterTime(int filterTime).

Setting the Extended AI Input Filter Time
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the extended AI input filter time
    * @param [in] AONum AO number
    * @param [in] filterTime filterTime (ms)
    * @return error code
    */
    int SetAuxAIFilterTime(int AONum, int filterTime);

Waiting for extended DI input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Waiting for extended DI input
    * @param [in] DINum DI number
    * @param [in] bOpen switch 0-off; 1-on
    * @param [in] time Maximum wait time (ms)
    * @param [in] errorAlarm whether or not to continue the movement
    * @return error code
    */
    int WaitAuxDI(int DINum, boolean bOpen, int time, boolean errorAlarm);

Waiting for extended AI input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Waiting for extended AI input
    * @param [in] AINum AI number
    * @param [in] sign 0-more than; 1-less than
    * @param [in] value AI value
    * @param [in] time Maximum wait time (ms)
    * @param [in] errorAlarm whether or not to continue the movement
    * @return error code
    */
    int WaitAuxAI(int AINum, int sign, int value, int time, boolean errorAlarm);
    
Get Extended AI Value
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get extended AI value
    * @param [in] AINum AI number
    * @param [in] isNoBlock whether to block or not
    * @return List[0]:error code; List[1] : value input value
    */
    List<Integer> GetAuxAI(int AINum, boolean isNoBlock);

UDP Extended Axis Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP Extended Axis Motion
    * @param [in] pos Target position
    * @param [in] ovl speed percentage
    * @return error code
    */
    int ExtAxisMove(ExaxisPos pos, double ovl);

UDP extension axes synchronized with robot joint motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extension axes synchronize motion with robot joint motion
    * @param [in] joint_pos Target joint position, in deg.
    * @param [in] desc_pos target Cartesian position
    * @param [in] tool tool coordinate number, range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] epos Extended axis position in mm
    * @param [in] blendT [-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in ms
    * @param [in] offset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] ffset_pos Bit position offset
    * @return error code
    */
    int ExtAxisSyncMoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, ExaxisPos epos, double blendT, int offset_flag, DescPose offset_pos);
    
UDP extension axes synchronized with robot linear motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extension axes synchronized with robot linear motion
    * @param [in] joint_pos Target joint position, in deg.
    * @param [in] desc_pos target Cartesian position
    * @param [in] tool tool coordinate number, range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], not open yet.
    * @param [in] ovl velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm
    * @param [in] epos Extended axis position in mm
    * @param [in] offset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos Bit position offset
    * @return error code
    */
    int ExtAxisSyncMoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendR, ExaxisPos epos, int offset_flag, DescPose offset_pos);

UDP extension axes synchronized with robot circular motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief UDP extension axes synchronized with robot arc motion
    * @param [in] joint_pos_p Path point joint position in deg.
    * @param [in] desc_pos_p Path point Cartesian pose
    * @param [in] ptool tool coordinate number, in the range [0~14].
    * @param [in] puser Workpiece coordinate number, range [0~14].
    * @param [in] pvel speed percentage, range [0~100]
    * @param [in] pacc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos_p Extended axis position of the midpoint in mm
    * @param [in] poffset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos_p bit position offset
    * @param [in] joint_pos_t Joint position of target point, in deg.
    * @param [in] desc_pos_t Destination point Cartesian pose
    * @param [in] ttool tool coordinate number, in the range [0~14].
    * @param [in] tuser Workpiece coordinate number, range [0~14].
    * @param [in] tvel speed percentage, range [0~100]
    * @param [in] tacc Acceleration percentage, range [0~100], not open yet.
    * @param [in] epos_t Extended axis position in mm
    * @param [in] toffset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos_t bit position offset
    * @param [in] ovl velocity scaling factor, range [0 to 100].
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm
    * @return error code
    */
    int ExtAxisSyncMoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, double pvel, double pacc, ExaxisPos epos_p, int poffset_flag , DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, double tvel, double tacc, ExaxisPos epos_t, int toffset_ flag, DescPose offset_pos_t, double ovl, double blendR).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.Mode(0);
        int tool = 1;
        int user = 0;
        double vel = 20.0;
        double acc = 100.0;
        double ovl = 100.0;
        ExaxisPos exaxisPos = new ExaxisPos( 0, 0, 0, 0, 0 );

        DescPose d0 = new DescPose(311.189, -309.688, 401.836, -174.375, -1.409, -82.354);
        JointPos j0 =new JointPos(118.217, -99.669, 79.928, -73.559, -85.229, -69.359);

        JointPos joint_pos0 =new JointPos(111.549,-99.821,108.707,-99.308,-85.305,-41.515);
        DescPose desc_pos0 = new DescPose(273.499,-345.746,201.573,-176.566 ,3.235,-116.819);
        ExaxisPos e_pos0=new ExaxisPos(0,0,0,0);

        JointPos joint_pos1 = new JointPos(112.395,-65.118,67.815,-61.449,-88.669,-41.517);
        DescPose desc_pos1 = new DescPose(291.393,-420.519,201.089,156.297,21.019,-120.919);
        ExaxisPos e_pos1 = new ExaxisPos(-30, -30, 0, 0);


        JointPos j2 = new JointPos(111.549,-98.369,108.036,-103.789,-95.203,-69.358);
        DescPose desc_pos2 = new DescPose(234.544,-392.777,205.566,176.584,-5.694,-89.109);
        ExaxisPos epos2 = new ExaxisPos(0.000,0.000,0.000,0.000,0.000);

        JointPos j3 = new JointPos(113.908,-61.947,63.829,-64.478,-85.406,-69.256);
        DescPose desc_pos3 = new DescPose(336.049,-444.969,192.799,173.776 ,27.104,-89.455);
        ExaxisPos epos3 = new ExaxisPos(-30.000,-30.000, 0.000, 0.000);

        // Starting point of the arc
        JointPos j4 = new JointPos(137.204,-98.475,106.624,-97.769,-90.634,-69.24);
        DescPose desc_pos4 = new DescPose(381.269,-218.688,205.735,179.274,0.128,-63.556);

        JointPos j5 = new JointPos(115.069,-92.709,97.285,-82.809,-90.455,-77.146);
        DescPose desc_pos5 = new DescPose(264.049,-329.478 ,220.747,176.906,11.359,-78.044);
        ExaxisPos epos5 = new ExaxisPos(-15, 0, 0, 0);


        JointPos j6 = new JointPos(102.409,-63.115,70.559,-70.156,-86.529,-77.148);
        DescPose desc_pos6 = new DescPose(232.407,-494.228 ,158.115,176.803,27.319,-92.056);
        ExaxisPos epos6 = new ExaxisPos(-30, 0, 0, 0);

        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);

        //Synchronized Joint Movement
        robot.MoveJ(j0, d0, 1, 0, vel, acc, ovl, exaxisPos, -1, 0, offset_pos);
        robot.ExtAxisMove(exaxisPos,40);
        robot.ExtAxisSyncMoveJ(joint_pos0, desc_pos0, 1, 0,20,100,100,e_pos0,-1,0,offset_pos);
        robot.ExtAxisSyncMoveJ(joint_pos1, desc_pos1, 1, 0,20,100,100,e_pos1,-1,0,offset_pos);


        //Synchronized linear motion
        robot.MoveJ(j0, d0, 1, 0, vel, acc, ovl, exaxisPos, -1, 0, offset_pos);
        robot.ExtAxisMove(exaxisPos,40);
        robot.ExtAxisSyncMoveL(j2, desc_pos2, tool, user, 40, 100, 100, -1, epos2, 0, offset_pos);
        robot.ExtAxisSyncMoveL(j3, desc_pos3, tool, user, 40, 100, 100, -1, epos3, 0, offset_pos);
        //Synchronized circular motion
        robot.MoveJ(j0, d0, 1, 0, vel, acc, ovl, exaxisPos, -1, 0, offset_pos);
        robot.ExtAxisMove(exaxisPos,20);
        robot.MoveJ(j4, desc_pos4, 1, 0, vel, acc, ovl, exaxisPos, -1, 0, offset_pos);

        robot.ExtAxisSyncMoveC(j5, desc_pos5, tool, user, 40, 100, epos5, 0, offset_pos, j6, desc_pos6, tool, user, 40, 100, epos6, 0, offset_pos, 100, 0);

        robot.Sleep(3000);
        robot.MoveJ(j0, d0, 1, 0, vel, acc, ovl, exaxisPos, -1, 0, offset_pos);
        robot.ExtAxisMove(exaxisPos,40);
        robot.Mode(1);
    }

Removable Device Enable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Removable device enable
    * @param [in] enable false-de-enable; true-enable
    * @return error code
    */
    int TractorEnable(Boolean enable).

Zeroing of removable devices
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Zeroing of removable devices
    * @return error code
    */
    int TractorHoming().

Movable unit linear motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Movable unit linear motion
    * @param [in] distance Linear motion distance (mm)
    * @param [in] vel Percentage of linear motion speed (0-100)
    * @return error code
    */
    int TractorMoveL(double distance, double vel).

Movable unit circular motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Movable unit circular motion
    * @param [in] radio radius of circular motion (mm)
    * @param [in] angle Angle of circular motion (°)
    * @param [in] vel Percentage of linear motion speed (0-100)
    * @return error code
    */
    int TractorMoveC(double radio, double angle, double vel).

Stopping motion of movable devices
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief stopping motion of movable devices
    * @return error code
    */
    int TractorStop().

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        UDPComParam param = new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevSetUDPComParam(param);//udp extended axis communication
        robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
        robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
        robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0, 0);

        robot.TractorEnable(false);
        robot.Sleep(2000);
        robot.TractorEnable(true);
        robot.Sleep(2000);
        robot.TractorHoming().

        robot.Sleep(2000);
        robot.TractorMoveL(100, 20);
        robot.Sleep(5000);
        robot.TractorMoveL(-100, 20);
        robot.Sleep(5000);
        robot.TractorMoveC(300, 90, 20).
        robot.Sleep(2000);
        robot.TractorStop();//trolley stops
        robot.TractorMoveC(300, -90, 20).
    }