Others
=================

.. toctree:: 
    :maxdepth: 5

Obtain the compensation value of robot DH parameter
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

    /**
	* @brief Obtain the compensation value of the DH parameter of the robot
	* @param [out] dhCompensation Robot DH parameter compensation value (mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]
	* @return Error code 
	*/
	errno_t GetDHCompensation(double dhCompensation[6]);

Download the point table database
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.1.0

.. code-block:: c++
    :linenos:
    
	/**
	* @brief Download the point table database
	* @param [in] pointTableName The name of the point table to be downloaded   pointTable1.db
	* @param [in] saveFilePath the storage path of the point table  C://test/
	* @return Error code 
	*/
	errno_t PointTableDownLoad(const std::string &pointTableName, const std::string &saveFilePath);

Upload the point table database
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Upload the point table database
	* @param [in] pointTableFilePath the full pathname of the point table    C://test/pointTable1.db
	* @return Error code
	*/
	errno_t PointTableUpLoad(const std::string &pointTableFilePath);

Update the LUA file for the point table
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.1.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Update the LUA file for the point table
	* @param [in] pointTableName The name of the point table to be switched  
	* @param [in] luaFileName name of lua file to be updated   "testPointTable.lua"
	* @return Error code
	*/
	errno_t PointTableUpdateLua(const std::string &pointTableName, const std::string &luaFileName);

Initialize log parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

	/**
	 * @brief Initialize log parameters;
	 * @param output_model:Output mode, 0-direct output; 1-buffered output; 2 - asynchronous output;
	 * @param file_path: File save path + name, the maximum length is 256, and the name must be in the form of xxx.log, such as /home/fr/linux/fairino.log;
	 * @param file_num:Scroll the number of files stored, 1~20. The maximum size of a single file is 50M;
	 * @return errno_t Error code;
	 */
	errno_t LoggerInit(int output_model = 0, std::string file_path = "", int file_num = 5);

Set the log filtering level
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    /**
     * @brief Set the log filtering level;
     * @param lvl: Filter the level value, the smaller the value, the less the output log, the default value is 1. 1-error, 2-warnning, 3-inform, 4-debug;
    */
	void SetLoggerLevel(int lvl = 1);

Code example
+++++++++++++++

.. versionadded:: C++ SDK-v2.1.2.0

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
	#include <string>

	using namespace std;

	int main(void)
	{
		FRRobot robot;
		robot.LoggerInit(2, "C:/Users/fr/Desktop/C++ SDK//sdk_with_log/abcd.log", 2);
		// robot.LoggerInit();
		robot.SetLoggerLevel(3);
		// robot.SetLoggerLevel();
		robot.RPC("192.168.58.2");

		double dh[6] = {0};
		int retval = 0;
		retval = robot.GetDHCompensation(dh);
		cout << "retval is: " << retval << endl;
		cout << "dh is: " << dh[0] << " " << dh[1] << " " << dh[2] << " " << dh[3] << " " << dh[4] << " " << dh[5] << endl;

		string save_path = "D://sharkLog/";
		string point_table_name = "point_table_a.db";
		retval = robot.PointTableDownLoad(point_table_name, save_path);
		cout<<"download : "<<point_table_name<<" fail: "<<retval<< endl;

		string upload_path = "D://sharkLog/0.db";
		retval = robot.PointTableUpLoad(upload_path);
		cout << "retval is: "<<retval<<endl;

		string point_tablename = "point_table_test.db";
		string lua_name = "testPoint.lua";
		retval = robot.PointTableUpdateLua(point_tablename, lua_name);
		cout << "retval is: " << retval << endl;
	}

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

Gets the terminal communication parameters
+++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief gets the terminal communication parameters
    * @param param axle communication parameters
    * @return  error code
    */
    errno_t GetAxleCommunicationParam(AxleComParam* param);

Sets the terminal communication parameters
+++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief sets the terminal communication parameters
    * @param param  axle communication parameters
    * @return  error code
    */
    errno_t SetAxleCommunicationParam(AxleComParam param);

Set the end file transfer type
+++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set the end file transfer type
    * @param type 1-MCU；2-LUA
    * @return  error code
    */
    errno_t SetAxleFileType(int type);

Set to enable axle LUA execution
+++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set to enable axle LUA execution
    * @param enable 0- Disable; 1- Enable
    * @return  error code
    */
    errno_t SetAxleLuaEnable(int enable);

Axle LUA file error reset
+++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Axle LUA file error reset
    * @param status 0- No recovery; 1- Recovery
    * @return  error code
    */
    errno_t SetRecoverAxleLuaErr(int status);

Gets the axle LUA execution enable status
+++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Gets the axle LUA execution enable status
    * @param status status[0]: 0- Is not enabled. 1- Enabled
    * @return  error code
    */
    errno_t GetAxleLuaEnableStatus(int status[]);

Set the axle LUA end device enablement type
+++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Set the axle LUA end device enablement type
    * @param forceSensorEnable Force sensor Enabled status, 0- Disable. 1- Enable
    * @param gripperEnable Specifies whether the gripper is enabled. 0- Disables the gripper. 1- Enable
    * @param IOEnable IO Indicates whether the device is enabled. 0- Indicates that the device is disabled. 1- Enable
    * @return  error code
    */
    errno_t SetAxleLuaEnableDeviceType(int forceSensorEnable, int gripperEnable, int IOEnable);

Gets the axle LUA end device enabled type
+++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief Gets the axle LUA end device enabled type
    * @param enable enable[0]: indicates whether the forceSensorEnable force sensor is enabled. 0- Indicates whether the forcesenSOrenable force sensor is disabled. 1- Enable
    * @param enable enable[1]:gripperEnable Indicates whether the gripper is enabled. 0- Disables the gripper. 1- Enable
    * @param enable enable[2]:IOEnable I/o Indicates whether the device is enabled. 0- Indicates whether the device is disabled. 1- Enable
    * @return  error code
    */
    errno_t GetAxleLuaEnableDeviceType(int* forceSensorEnable, int* gripperEnable, int* IOEnable);

Gets the currently configured end device
+++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief gets the currently configured end device
    *  @param forceSensorEnable Force sensor Enabled Device number 0- Not enabled; 1- Enable
    * @param gripperEnable Number of the gripperenable device, 0- Disable; 1- Enable
    * @param IODeviceEnable IODevice enable Device ID. 0- Disable. 1- Enable
    * @return  error code
    */
    errno_t GetAxleLuaEnableDevice(int forceSensorEnable[], int gripperEnable[], int IODeviceEnable[]);

Setting enables the gripper action control function
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief setting enables the gripper action control function
    * @param id ID of the gripper device
    * @param func func[0]- gripper enabled; func[1]- gripper initialization; 2- Position setting; 3- Speed setting; 4- Torque setting; 6- Read the gripper status; 7- Read the initialization state; 8- Read the fault code; 9- Read position; 10- Read speed; 11- Read torque
    * @return  error code
    */
    errno_t SetAxleLuaGripperFunc(int id, int func[]);

Obtains the enable gripper action control function
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief obtains the enable gripper action control function
    * @param id ID of the gripper device
    * @param func func[0]- gripper enabled; func[1]- gripper initialization; 2- Position setting; 3- Speed setting; 4- Torque setting; 6- Read the gripper status; 7- Read the initialization state; 8- Read the fault code; 9- Read position; 10- Read speed; 11- Read torque
    * @return  error code
    */
    errno_t GetAxleLuaGripperFunc(int id, int func[]);

Robot Ethercat writes files from the slave station
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * The @brief robot Ethercat writes files from the slave station
    * @param type Slave file type, 1- Upgrade slave file; 2- Upgrade the slave configuration file
    * @param slaveID Secondary station ID
    * @param fileName File name of the upload file
    * @return  error code
    */
    errno_t SlaveFileWrite(int type, int slaveID, std::string fileName);

Uploals the axle Lua open protocol file
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * @brief uploals the axle Lua open protocol file
    * @param filePath Local lua file pathname "... /AXLE_LUA_End_DaHuan.lua"
    * @return error code
    */
    errno_t AxleLuaUpload(std::string filePath);

Robot Ethercat entered boot mode from the station
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
    * The @brief robot Ethercat entered boot mode from the station
    * @return  error code
    */
    errno_t SetSysServoBootMode();

Example 1
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    void TestAxleLuaGripper(FRRobot* robot)
    {
        robot->AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan.lua"); 

        //Restart robot     

        ROBOT_STATE_PKG pkg;
        memset(&pkg, 0, sizeof(pkg));
        AxleComParam param(7, 8, 1, 0, 5, 3, 1);
        //AxleComParam param = new AxleComParam(8,7,2,1,6,4,2);
        robot->SetAxleCommunicationParam(param);

        AxleComParam getParam;
        robot->GetAxleCommunicationParam(&getParam);
        printf("GetAxleCommunicationParam param is %d %d %d %d %d %d %d\n", getParam.baudRate, getParam.dataBit, getParam.stopBit, getParam.verify, getParam.timeout, getParam.timeoutTimes, getParam.period);

        robot->SetAxleLuaEnable(1);
        int luaEnableStatus = 0;
        robot->GetAxleLuaEnableStatus(&luaEnableStatus);
        robot->SetAxleLuaEnableDeviceType(0, 1, 0);
        
        int forceEnable = 0;
        int gripperEnable = 0;
        int ioEnable = 0;
        robot->GetAxleLuaEnableDeviceType(&forceEnable, &gripperEnable, &ioEnable);
        printf("GetAxleLuaEnableDeviceType param is %d %d %d\n", forceEnable, gripperEnable, ioEnable);

        //int func[16] = {0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0};
        int func[16] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
        robot->SetAxleLuaGripperFunc(1, func);
        int getFunc[16] = {0};
        robot->GetAxleLuaGripperFunc(1, getFunc);
        int getforceEnable[16] = {0};
        int getgripperEnable[16] = {0};
        int getioEnable[16] = {0};
        robot->GetAxleLuaEnableDevice(getforceEnable, getgripperEnable, getioEnable);
        printf("\ngetforceEnable status : ");
        for (int i = 0; i < 16; i++)
        {
            printf("%d,", getforceEnable[i]);
        }
        printf("\ngetgripperEnable status : ");
        for (int i = 0; i < 16; i++)
        {
            printf("%d,", getgripperEnable[i]);
        }
        printf("\ngetioEnable status : ");
        for (int i = 0; i < 16; i++)
        {
            printf("%d,", getioEnable[i]);
        }
        printf("\n");
        robot->ActGripper(1, 0);
        robot->Sleep(2000);
        robot->ActGripper(1, 1);
        robot->Sleep(2000);
        robot->MoveGripper(1, 90, 10, 100, 50000, 0);
        int pos = 0;
        while (true)
        {
            robot->GetRobotRealTimeState(&pkg);
            printf("gripper pos is %u\n", pkg.gripper_position);
            robot->Sleep(100);
        }
    }

Example 2
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++ SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    void TestAxleLuaForceSensor(FRRobot* robot)
    {
        robot->AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan.lua");

        //Restart robot  

        ROBOT_STATE_PKG pkg;
        memset(&pkg, 0, sizeof(pkg));
        AxleComParam param(7, 8, 1, 0, 5, 3, 1);
        robot->SetAxleCommunicationParam(param);

        AxleComParam getParam;
        robot->GetAxleCommunicationParam(&getParam);
        printf("GetAxleCommunicationParam param is %d %d %d %d %d %d %d\n", getParam.baudRate, getParam.dataBit, getParam.stopBit, getParam.verify, getParam.timeout, getParam.timeoutTimes, getParam.period);

        robot->SetAxleLuaEnable(1);
        int luaEnableStatus = 0;
        robot->GetAxleLuaEnableStatus(&luaEnableStatus);
        robot->SetAxleLuaEnableDeviceType(1, 0, 0);

        int forceEnable = 0;
        int gripperEnable = 0;
        int ioEnable = 0;
        robot->GetAxleLuaEnableDeviceType(&forceEnable, &gripperEnable, &ioEnable);
        printf("GetAxleLuaEnableDeviceType param is %d %d %d\n", forceEnable, gripperEnable, ioEnable);

        
        int getforceEnable[16] = { 0 };
        int getgripperEnable[16] = { 0 };
        int getioEnable[16] = { 0 };
        robot->GetAxleLuaEnableDevice(getforceEnable, getgripperEnable, getioEnable);
        printf("\ngetforceEnable status : ");
        for (int i = 0; i < 16; i++)
        {
            printf("%d,", getforceEnable[i]);
        }
        printf("\ngetgripperEnable status : ");
        for (int i = 0; i < 16; i++)
        {
            printf("%d,", getgripperEnable[i]);
        }
        printf("\ngetioEnable status : ");
        for (int i = 0; i < 16; i++)
        {
            printf("%d,", getioEnable[i]);
        }
        printf("\n");
        
        vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
        vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
        vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
        robot->EndForceDragControl(1, 0, 0, M, B, K, F, 50, 100);

        robot->Sleep(10 * 1000);

        robot->EndForceDragControl(0, 0, 0, M, B, K, F, 50, 100);
    }
