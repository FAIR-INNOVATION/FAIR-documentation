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