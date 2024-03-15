Others
=========================

.. toctree:: 
    :maxdepth: 5

Download the point table database
+++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``PointTableDownLoad(point_table_name, save_file_path)``"
    "Description", "Download the point table database"
    "Required parameter", "- ``point_table_name``：pointTable1.db;
    - ``save_file_path``: C://test/;"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot

    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.PointTableDownLoad("point_table_a.db","D://Desktop/testPoint/download/")
    print("PointTableDownLoad:",error)
 
Upload the point table database
+++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``PointTableUpLoad(point_table_file_path)``"
    "Description", "Download the point table database"
    "Required parameter", "- ``point_table_file_path``：Full path name of the upload point table   C://test/pointTable1.db"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos:   

    from fairino import Robot

    # A connection is established with the robot controller. A successful connection returns a robot object 
    robot = Robot.RPC('192.168.58.2')
    error = robot.PointTableUpLoad("D://Desktop/testPoint/point_table_a.db")
    print("PointTableUpLoad:",error)

Point table switch
+++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``PointTableSwitch(point_table_name)``"
    "Description", "Point table switch"
    "Required parameter", "- ``point_table_name``: When the point table is empty, that is, an empty string, the lua program is updated to the original program that did not apply the point table"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return errorStr"

Code example
------------
.. code-block:: python
    :linenos: 

    from fairino import Robot

    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.PointTableSwitch("point_table_a.db")
    print("PointTableSwitch:",error)

Point table update Lua
+++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``PointTableUpdateLua(point_table_name, lua_file_name)``"
    "Description", "Point table update Lua"
    "Required parameter", "- ``point_table_name``: When the point table is empty, that is, an empty string, the lua program is updated to the original program that did not apply the point table
    - ``lua_file_name``: Name of the lua file to update"
    "Optional parameter", "Nothing" 
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return errorStr"

Code example
------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.PointTableUpdateLua("point_table_a.db","testpoint.lua")
    print("PointTableUpdateLua:",error)

Log init
+++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoggerInit(output_model=1, file_path="", file_num=5)``"
    "Description", "Initialize log parameters (The log function is not enabled without init)"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``output_model``: 0- direct output; 1- Buffer output; 2- Asynchronous output, default 1;
    - ``file_path``: ile save path + file name. The file name must be in the format of xxx.log, for example, D://Desktop /fairino.log. The default path of the execution program is fairino_ year+month+ data.log, for example, fairino_2024_03_13.log.
    - ``file_num``: The number of files to be stored. The value ranges from 1 to 20. The default value is 5. The single file must less than 50M."
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    robot.LoggerInit(output_model=0,file_path="D://Desktop/fairino.log",file_num=3)
    robot.SetLoggerLevel(3)

Set Log Level
+++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLoggerLevel(lvl=1)``"
    "Description", "Set Log Level"
    "Required parameter", "Nothing"
    "Optional parameter", "- ``lvl``：The filtering level. The smaller the value, the smaller the log output. 1-error, 2-warnning, 3-inform, 4-debug. The default value is 1."
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    robot.LoggerInit(output_model=0,file_path="D://Desktop/fairino.log",file_num=3)
    robot.SetLoggerLevel(3)

Download the Lua file
+++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30
    
    "Prototype", "``LuaDownLoad(fileName, savePath)``"
    "Description", "Download the Lua file"
    "Required parameter", "- ``fileName``: The name of the lua file to be downloaded, for example, “test.lua”
    - ``savePath``：The local path for saving the file, for example, “D://Down/”"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    robot.LuaDownLoad("test", "D://Desktop/test_download/")

Upload the Lua file
+++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LuaUpload(filePath)``"
    "Description", "Upload the Lua file"
    "Required parameter", "- ``filePath``：The full path name of the uploaded file, for example, D://test/test.lua"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - errorStr(The lua file has an error return)"

Code example
------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    robot.LuaUpload("D://test/test.lua")

Delete the Lua file
+++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LuaDelete(fileName)``"
    "Description", "Delete the Lua file"
    "Required parameter", "- ``fileName``: The name of the lua file to be deleted, for example, “test.lua”"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret = robot.GetSoftwareVersion()
    robot.LuaDelete("test2")

Obtain Name of all current lua files
+++++++++++++++++++++++++++++++++++++++

.. versionadded:: python SDK-v2.0.2

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetLuaList()``"
    "Description", "Obtain Name of all current lua files"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ret,num,name  = robot.GetLuaList()
    print(num)
    print(name)
