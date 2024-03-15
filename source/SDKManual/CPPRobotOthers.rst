Others
=================

.. toctree:: 
    :maxdepth: 5

Obtain the compensation value of robot DH parameter
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.1.0

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

.. versionadded:: C++SDK-v2.1.1.0

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

.. versionadded:: C++SDK-v2.1.1.0

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

.. versionadded:: C++SDK-v2.1.1.0

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

.. versionadded:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

	/**
	 * @brief Initialize log parameters;
	 * @param output_model：Output mode, 0-direct output; 1-buffered output; 2 - asynchronous output;
	 * @param file_path: File save path + name, the maximum length is 256, and the name must be in the form of xxx.log, such as /home/fr/linux/fairino.log;
	 * @param file_num：Scroll the number of files stored, 1~20. The maximum size of a single file is 50M;
	 * @return errno_t Error code;
	 */
	errno_t LoggerInit(int output_model = 0, std::string file_path = "", int file_num = 5);

Set the log filtering level
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    /**
     * @brief Set the log filtering level;
     * @param lvl: Filter the level value, the smaller the value, the less the output log, the default value is 1. 1-error, 2-warnning, 3-inform, 4-debug;
    */
	void SetLoggerLevel(int lvl = 1);

Code example
+++++++++++++++

.. versionadded:: C++SDK-v2.1.2.0

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
		robot.LoggerInit(2, "C:/Users/fr/Desktop/c++sdk//sdk_with_log/abcd.log", 2);
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

Download Lua file
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Download Lua file
	* @param [in] fileName The name of the lua file to be downloaded, for example: "test.lua"
	* @param [in] savePath Save the file local path, for example: "D://Down/"
	* @return error code
	*/
	errno_t LuaDownLoad(std::string fileName, std::string savePath);

Upload Lua file
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Upload Lua file
	* @param [in] filePath local lua file path name
	* @return error code
	*/
	errno_t LuaUpload(std::string filePath);

Delete Lua files
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Delete Lua files
	* @param [in] fileName The name of the lua file to be deleted, for example: "test.lua"
	* @return error code
	*/
	errno_t LuaDelete(std::string fileName);

Get the names of all current lua files
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

	/**
	* @brief Get the names of all current lua files
	* @param [out] luaNames lua file name list
	* @return error code
	*/
	errno_t GetLuaList(std::list<std::string>* luaNames);
