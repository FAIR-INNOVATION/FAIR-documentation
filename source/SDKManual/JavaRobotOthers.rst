Other interfaces
=====================================

.. toctree:: 
    :maxdepth: 5

Download Point Table Database
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Download the point table database 
    * @param [in] pointTableName Name of the point table to be downloaded pointTable1.db
    * @param [in] saveFilePath The storage path to download the point table C://test/
    * @return error code 
    */
    int PointTableDownLoad(String pointTableName, String saveFilePath);

Upload point table database
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief uploading the point table database 
    * @param [in] pointTableFilePath Full pathname of the uploaded point table C://test/pointTable1.db
    * @return error code 
    */
    int PointTableUpLoad(String pointTableFilePath);;

Toggle point table and apply
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Toggle point table and apply 
    * @param [in] pointTableName Name of point table to be switched "pointTable1.db"
    * @param [in] errorStr Toggle point table error message
    * @return error code 
    */
    int PointTableSwitch(String pointTableName, String errorStr);

Point table update lua file
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief point table update lua file
    * @param [in] pointTableName The name of the point table to be switched, "pointTable1.db", when the point table is empty, i.e. "", it means updating the lua program to the initial program without applying the point table.
    * @param [in] luaFileName Name of the lua file to be updated "testPointTable.lua"
    * @param [out] errorStr Toggle point table error message
    * @return error code 
    */
    int PointTableUpdateLua(String pointTableName, String luaFileName, String errorStr);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
            return;
        }

        robot.PointTableUpLoad("D://zUP/point_table_test1.db");//Point Table Upload
        robot.PointTableDownLoad("point_table_test1.db", "D:///zUP/");//point table download
        String errStr = "";
        robot.PointTableSwitch("point_table_test1.db", errStr);//switch the point table
        // Point table update LUA program
        robot.PointTableUpdateLua("point_table_test2.db", "1010Test.lua", errStr);
    }

Initialize Logging Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Initialize log parameters
    * @param [in] logType: output mode, DIRECT-direct output; BUFFER-buffered output; ASYNC-asynchronous output
    * @param [in] logLevel: log filter level, ERROR-Error; WARNING-Warning; INFO-Information; DEBUG-Debugging
    * @param [in] filePath: file save path, e.g. "D:///Log/".
    * @param [in] saveFileNum: the number of files to be saved; files that exceed both the number of files to be saved and the number of days to be saved will be deleted.
    * @param [in] saveDays: the number of days to save the file, the file that exceeds both the number of files to save and the number of days to save the file will be deleted
    * @return error code
    */
    int LoggerInit(FrLogType logType, FrLogLevel logLevel, String filePath, int saveFileNum, int saveDays);;

Setting the log filter level
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief sets the level of log filtering.
    * @param [in] logLevel: log filter level, ERROR-Error; WARNING-Warning; INFO-Information; DEBUG-Debugging
    * @return error code
    */
    int SetLoggerLevel(FrLogLevel logLevel);


code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        robot.SetLoggerLevel(FrLogLevel.DEBUG);
    }

Setting up robot peripheral protocols
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Set up robot peripheral protocols
    * @param [in] protocol Robot peripheral protocol number 4096-Extended Axis Control Card; 4097-ModbusSlave; 4098-ModbusMaster
    * @return error code 
    */
    int SetExDevProtocol(int protocol).

Obtaining robot peripheral protocols
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get robot peripheral protocols
    * @return List[0]:Error code; List[1] : int protocol Robot peripheral protocol number 4096-Expanded axis control card; 4097-ModbusSlave; 4098-ModbusMaster 
    */
    List<Integer> GetExDevProtocol();

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

        robot.SetExDevProtocol(4096);
        List<Number> rtnArr = robot.GetTargetPayload(1);
        rtnArr=GetExDevProtocol();
    }

End Sensor Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief End sensor configuration
    * @param [in] config idCompany vendor, 18-JUNKONG; 25-HUIDE
    * @param [in] config idDevice type, 0-JUNKONG/RYR6T.
    * @param [in] config idSoftware software version, 0-J1.0/HuiDe1.0 (not yet open)
    * @param [in] config idBus mount location, 1-end port 1; 2-end port 2.... .8-end port 8 (not open yet)
    * @return error code
    */
    int AxleSensorConfig(DeviceConfig config).

Get End Sensor Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get end sensor configuration
    * @param [out] config idCompany vendor, 18-JUNKONG; 25-HUIDE
    * @param [out] config idDevice type, 0-JUNKONG/RYR6T.
    * @return error code
    */
    int AxleSensorConfigGet(DeviceConfig config);

End sensor activation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief End sensor activation
    * @param [in] actFlag 0-reset; 1-activate
    * @return error code
    */
    int AxleSensorActivate(int actFlag).

End Sensor Register Write
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
        DeviceConfig axleSensorConfig = new DeviceConfig(18, 0, 0, 1);
        robot.AxleSensorConfig(axleSensorConfig);

        DeviceConfig getConfig = new DeviceConfig();
        robot.AxleSensorConfigGet(getConfig);
        System.out.println("company is " + getConfig.company + ", type is " + getConfig.device);

        robot.AxleSensorActivate(1);

        robot.AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0);
    }