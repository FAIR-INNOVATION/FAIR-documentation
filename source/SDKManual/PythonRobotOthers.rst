Others
=================

.. toctree::
    :maxdepth: 5

Download Point Table Database
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``PointTableDownLoad(point_table_name, save_file_path)``"
    "Description", "Download Points Table Database"
    "Required Parameters", "- ``point_table_name``: name of the point table to be downloaded pointTable1.db;
    - ``save_file_path``: storage path to download the point table C://test/;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot

    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.PointTableDownLoad("point_table_a.db", "D://Desktop/testPoint/download/")
    print("PointTableDownLoad error code:",error)
 
Upload point table database
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``PointTableUpLoad(point_table_file_path)``"
    "Description", "Upload point table database"
    "Required Parameters", "- ``point_table_file_path``: full pathname of uploaded point table C://test/pointTable1.db"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------
.. code-block:: python
    :linenos:   

    from fairino import Robot

    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.PointTableUpLoad("D://Desktop/testPoint/point_table_a.db")
    print("PointTableUpLoad error code:",error)

Point table switching
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``PointTableSwitch(point_table_name)``"
    "Description", "Point table switching"
    "mandatory parameter","- ``point_table_name``: name of the point table to be switched pointTable1.db, when the point table is empty, i.e. "", it means updating the lua program to the initial program with no point table applied"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------
.. code-block:: python
    :linenos: 

    from fairino import Robot

    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.PointTableSwitch("point_table_a.db")
    print("PointTableSwitch:",error)

Point table update lua file
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``PointTableUpdateLua(point_table_name, lua_file_name)``"
    "Description", "Point table update lua file"
    "Required Parameters","- ``point_table_name``: the name of the point table to be switched, pointTable1.db, when the point table is empty, i.e. "", it means updating the lua program to the initial program with no point table applied
    - ``lua_file_name``: name of the lua file to update testPointTable.lua"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.PointTableUpdateLua("point_table_a.db", "testpoint.lua")
    print("PointTableUpdateLua:",error)

Initialize Logging Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoggerInit(output_model=1, file_path="", file_num=5)``"
    "Description", "Initialize logging parameters (logging is not enabled without initialization)"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``output_model``: output mode, 0 - direct output; 1 - buffered output; 2 - asynchronous output, default 1;
    - ``file_path``: file save path + name, the name must be in the form of xxx.log, e.g. D:///Desktop /fairino.log. Default path where the executable program is located, default name fairino_year+month+ data.log (e.g.: fairino_2024_03_13. log).
    - ``file_num``: number of files for rolling storage, 1~20, default value is 5. Individual files are capped at 50M."
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.LoggerInit(output_model=0,file_path="D://Desktop/fairino.log",file_num=3)
    robot.SetLoggerLevel(3)

Setting the log filter level
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLoggerLevel(lvl=1)``"
    "Description", "Set log filter level"
    "Mandatory parameters", "NULL"
    "Default Parameters","- ``lvl``: filter level value, the smaller the value the less output logs, 1-error, 2-warnning, 3-inform, 4-debug, default value is 1"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.LoggerInit(output_model=0,file_path="D://Desktop/fairino.log",file_num=3)
    robot.SetLoggerLevel(3)

Setting up robot peripheral protocols
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExDevProtocol(protocol)``"
    "Description", "Setting the robot peripheral protocol"
    "Mandatory parameters", "- ``protocol``: robot peripheral protocol number 4096 - Extended Axis Control Card; 4097 - ModbusSlave; 4098 - ModbusMaster"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    ret =robot.SetExDevProtocol(4098)
    print("SetExDevProtocol",ret)
    ret =robot.GetExDevProtocol()
    print("GetExDevProtocol",ret)

Obtaining Robot Peripheral Protocols
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetExDevProtocol()``"
    "Description", "Get robot peripheral protocol"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode. 
    - ``protocol``: Robot peripheral protocol number 4096-Extended Axis Control Card; 4097-ModbusSlave; 4098-ModbusMaster"

End Sensor Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AxleSensorConfig(idCompany, idDevice, idSoftware, idBus)``"
    "Description", "End Sensor Configuration"
    "Mandatory parameters", "
    - ``idCompany``: manufacturer, 18-Junkong; 25-Huide
    - ``idDevice``: type, 0-JUNKONG/RYR6T.V1.0
    - ``idSoftware``: software version, 0-J1.0/HuiDe1.0 (not yet available)
    - ``idBus``: mount location, 1-end port 1; 2-end port 2... ..8-end port 8 (not open yet)
    "
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
------------------------------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')
    error = robot.AxleSensorConfig(18,0,0,0)
    print("AxleSensorConfig return:", error)

    error = robot.AxleSensorConfigGet()
    print("AxleSensorConfigGet return:", error)

    error = robot.AxleSensorActivate(0)
    print("AxleSensorActivate return:", error)
    time.sleep(1)
    error = robot.AxleSensorActivate(1)
    print("AxleSensorActivate return:", error)

    while(1).
        error = robot.AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0)
        print("AxleSensorRegWrite return:", error)
        
Get End Sensor Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AxleSensorConfigGet()``"
    "Description", "Get end sensor configuration"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``idCompany``: manufacturer, 18-Junkong; 25-Huide
    - ``idDevice``: type, 0-JUNKONG/RYR6T.V1.0"
        
End sensor activation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AxleSensorActivate(actFlag)``"
    "Description", "End sensor activation"
    "Mandatory parameters", "``actFlag``: 0-reset; 1-activate"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``coord``: coordinate system values [x,y,z,rx,ry,rz]"

End Sensor Register Write
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AxleSensorRegWrite(devAddr, regHAddr, regLAddr, regNum, data1, data2, isNoBlock)``"
    "Description", "End Sensor Register Write"
    "Mandatory parameters", "- ``devAddr``: device address number 0-255
    - ``regHAddr``: register address high 8 bits
    - ``regLAddr``: register address lower 8 bits
    - ``regNum``: number of registers 0-255
    - ``data1``: write to register value 1
    - ``data2``: write register value 2
    - ``isNoBlock``: whether blocking 0 - blocking; 1 - non-blocking"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Sets whether the output is reset after the SmartTool is stopped/paused.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOutputResetSmartToolDO(resetFlag)``"
    "Description", "Sets whether the output is reset after a SmartTool stop/pause"
    "Mandatory parameter", "- ``resetFlag``: to reset or not to reset, 0 - no reset, 1 - reset"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Getting end communication parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAxleCommunicationParam()``"
    "Description", "Get end communication parameters"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``baudRate``: baud rate: support 1-9600, 2-14400, 3-19200, 4-38400, 5-56000, 6-67600, 7-115200, 8-128000
    - ``dataBit``: data bit: data bit support (8,9), currently commonly used 8
    - ``stopBit``: stop bit: 1-1, 2-0.5, 3-2, 4-1.5, currently 1 is commonly used.
    - ``verify``: check digit: 0-None, 1-Odd, 2-Even, currently 0.
    - ``timeout``: timeout time: 1~1000ms, this value needs to be combined with the peripheral with the setting of reasonable time parameters
    - ``timeoutTimes``: timeout times: 1~10, mainly for timeout retransmission, reduce occasional exceptions to improve user experience
    - ``period``: periodic instruction time interval:1~1000ms, mainly used for the time interval between each issuance of periodic instructions"

Setting the end communication parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAxleCommunicationParam(baudRate, dataBit, stopBit, verify, timeout, timeoutTimes, period)``"
    "description", "set end communication parameters"
    "Required Parameters","- ``baudRate``: baud rate: supports 1-9600, 2-14400, 3-19200, 4-38400, 5-56000, 6-67600, 7-115200, 8-128000
    - ``dataBit``: data bit: data bit support (8,9), currently commonly used 8
    - ``stopBit``: stop bit: 1-1, 2-0.5, 3-2, 4-1.5, currently 1 is commonly used.
    - ``verify``: check digit: 0-None, 1-Odd, 2-Even, currently 0.
    - ``timeout``: timeout time: 1~1000ms, this value needs to be combined with the peripheral with the setting of reasonable time parameters
    - ``timeoutTimes``: timeout times: 1~10, mainly for timeout retransmission, reduce occasional exceptions to improve user experience
    - ``period``: periodic instruction time interval:1~1000ms, mainly used for the time interval between each issuance of periodic instructions"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Setting the end file transfer type
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAxleFileType(type)``"
    "Description", "Set the end file transfer type"
    "Mandatory parameters", "- ``type``: 1-MCU upgrade file, 2-LUA file"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Setting Enable End LUA Execution
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAxleLuaEnable(enable)``"
    "Description", "Set Enable End LUA Enforcement"
    "Mandatory parameters", "- ``enable``: 0 - not enabled; 1 - enabled"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

End LUA file exception error recovery
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRecoverAxleLuaErr(enable)``"
    "Description", "End LUA File Exception Error Recovery"
    "Mandatory parameters", "- ``status``: 0 - no recovery; 1 - recovery"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Get end LUA execution enable status
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAxleLuaEnableStatus()``"
    "description", "Get end LUA execution enable state"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``enable``: 0-don't enable; 1-enable"

Setting the end LUA end device enable type
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetAxleLuaEnableDeviceType(forceSensorEnable, gripperEnable, IOEnable)``"
    "Description", "Set end LUA end device enable type"
    "Mandatory parameters","- ``forceSensorEnable``: force sensor enable status, 0 - not enabled; 1 - enabled
    - ``gripperEnable``: gripper enable status, 0 - not enabled; 1 - enabled
    - ``IOEnable``: IO device enable status, 0-not enabled; 1-enabled"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Get End LUA End Device Enablement Type
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAxleLuaEnableDeviceType()``"
    "Description", "Get End LUA End Device Enablement Type"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``forceSensorEnable``: force sensor enable status, 0 - not enabled; 1 - enabled
    - ``gripperEnable``: gripper enable status, 0 - not enabled; 1 - enabled
    - ``IOEnable``: IO device enable status, 0-not enabled; 1-enabled"

Get the currently configured end device
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAxleLuaEnableDevice()``"
    "Description", "Get the currently configured end device"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``forceSensorEnable[8]``: force sensor enable state, 0 - not enabled; 1 - enabled
    - ``gripperEnable[8]``: gripper enable status, 0 - not enabled; 1 - enabled
    - ``IOEnable[8]``: IO device enable status, 0-not enabled; 1-enabled"