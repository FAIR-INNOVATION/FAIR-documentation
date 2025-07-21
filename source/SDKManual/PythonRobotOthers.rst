Others
=================

.. toctree::
    :maxdepth: 5

Get SSH public key
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSSHKeygen()``"
    "description", "Get SSH public key"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``keygen``: public key"

Issue the SCP command
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetSSHScpCmd(mode, sshname, sship, usr_file_url, robot_file_url)``"
    "Description", "Issue the SCP command"
    "Mandatory parameters", "- ``mode``: 0- Upload (Host Computer -> Controller), 1- Download (Controller -> Host Computer
    - ``sshname``: User name of the upper computer
    - ``sship``: The ip address of the upper computer
    - ``usr_file_url``: File path of the upper computer
    - ``robot_file_url``: Robot controller file path"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Calculate the MD5 value of a file in a specified path
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeFileMD5(file_path)``"
    "Description", "Calculates the MD5 value of a file in the specified path."
    "Mandatory parameter","- ``file_path``: file path including filename, default Traj folder path is :/fruser/traj/, e.g. /fruser/traj/trajHelix_aima_1.txt"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``md5``: the MD5 value of the file."

Robot SSH, MD5 instruction code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    file_path = "/fruser/airlab.lua"
    md5 = ""
    emerg_state = 0
    si0_state = 0
    si1_state = 0
    sdk_com_state = 0
    ssh_keygen = ""
    retval,ssh_keygen = robot.GetSSHKeygen()
    print(f"GetSSHKeygen retval is: {retval}")
    print(f"ssh key is: {ssh_keygen}")
    ssh_name = "fr"
    ssh_ip = "192.168.58.45"
    ssh_route = "/home/fr"
    ssh_robot_url = "/root/robot/dhpara.config"
    retval = robot.SetSSHScpCmd(1, ssh_name, ssh_ip, ssh_route, ssh_robot_url)
    print(f"SetSSHScpCmd retval is: {retval}")
    print(f"robot url is: {ssh_robot_url}")
    error, md5 = robot.ComputeFileMD5(file_path)
    print(f"md5 is: {md5}")
    robot.CloseRPC()

Setting the Robot 20004 Port Feedback Cycle
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotRealtimeStateSamplePeriod(period)``"
    "Description", "Setting the robot 20004 port feedback cycle"
    "Mandatory parameter", "- ``period``: robot 20004 port feedback period (ms)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Get robot 20004 port feedback cycle
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotRealtimeStateSamplePeriod()``"
    "Description", "Get robot 20004 port feedback cycle"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``period``: robot 20004 port feedback period (ms)"

Robot 20004 port state feedback cycle configuration code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.SetRobotRealtimeStateSamplePeriod(10)
    error,getPeriod = robot.GetRobotRealtimeStateSamplePeriod()
    print(f"period is {getPeriod}")
    time.sleep(1)
    robot.CloseRPC()
    
Robot software upgrade
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SoftwareUpgrade(filePath, block)``"
    "Description", "Robot Software Upgrade"
    "Required parameters","- ``filePath``: full path of the software upgrade package
    - ``block``: whether to block until the upgrade is complete true:blocking; false:non-blocking"
    "Default parameters", "NULL"
    "Return Value", "- errcode Success-0 Failure- errcode "

Get robot software upgrade status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSoftwareUpgradeState()``"
    "Description", "Get robot software upgrade status"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``state``: robot package upgrade status, 0: idle in progress or uploading upgrade package in progress, 1~100: percentage of upgrade completed, -1: upgrade software failure, -2: checksum failure, -3: version checksum failure, -4: unpacking failure, -5: user configuration upgrade failure, -6: peripheral configuration upgrade failure, -7: extended axis configuration upgrade failure, -8: robot configuration upgrade failure, -9: DH parameter configuration upgrade failure"

Robot software upgrade code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    error = robot.SoftwareUpgrade("D://zUP/QNX382/software.tar.gz", False)
    print(f"SoftwareUpgrade error is {error}")
    while True:
        curState = robot.GetSoftwareUpgradeState()
        print(f"upgrade state is {curState}")
        time.sleep(3)
    robot.CloseRPC()

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

Robot point table operation code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    save_path = "D://zDOWN/"
    point_table_name = "point_table_FR5.db"
    rtn = robot.PointTableDownLoad(point_table_name, save_path)
    print(f"download : {point_table_name} fail: {rtn}")
    upload_path = "D://zDOWN/point_table_FR5.db"
    rtn = robot.PointTableUpLoad(upload_path)
    print(f"retval is: {rtn}")
    point_tablename = "point_table_FR5.db"
    lua_name = "test0610.lua"
    rtn,error = robot.PointTableUpdateLua(point_tablename, lua_name)
    print(f"retval is: {rtn}")
    robot.CloseRPC()

Controller log download
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``RbLogDownload(savePath)``"
    "Description", "Controller log download"
    "Mandatory parameters", "- ``savePath``: Save the file path D://zDown/"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Download all data sources
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AllDataSourceDownload(savePath)``"
    "Description", "Download all data sources"
    "Mandatory parameters", "- ``savePath``: Save the file path D://zDown/"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Data backup package download
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``DataPackageDownload(savePath)``"
    "Description", "Data backup package download"
    "Mandatory parameters", "- ``savePath``: Save the file path D://zDown/"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Download the controller data code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    rtn = robot.RbLogDownload("D://zDOWN/")
    print(f"RbLogDownload rtn is {rtn}")
    rtn = robot.AllDataSourceDownload("D://zDOWN/")
    print(f"AllDataSourceDownload rtn is {rtn}")
    rtn = robot.DataPackageDownload("D://zDOWN/")
    print(f"DataPackageDownload rtn is {rtn}")
    robot.CloseRPC()

Set up encoder upgrade
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetEncoderUpgrade(path)``"
    "Description", "Set up encoder upgrade"
    "Mandatory parameters", "- ``path``：Local upgrade package full path(D://zUP/XXXXX.bin)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Set up joint firmware upgrade
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetJointFirmwareUpgrade(type, path)``"
    "Description", "Set up joint firmware upgrade"
    "Mandatory parameters", "- ``type``：Updating file types 1- Upgrade firmware; 2- Upgrade the slave profile
    - ``path``：Local upgrade package full path(D://zUP/XXXXX.bin)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Set up control box firmware upgrade
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetCtrlFirmwareUpgrade(type, path)``"
    "Description", "Set up control box firmware upgrade"
    "Mandatory parameters", "- ``type``：Updating file types 1- Upgrade firmware; 2- Upgrade the slave profile
    - ``path``：Local upgrade package full path(D://zUP/XXXXX.bin)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Set up the end firmware upgrade
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetEndFirmwareUpgrade(type, path)``"
    "Description", "Set up the end firmware upgrade"
    "Mandatory parameters", "- ``type``：Updating file types 1- Upgrade firmware; 2- Upgrade the slave profile
    - ``path``：Local upgrade package full path(D://zUP/XXXXX.bin)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Joint full parameter profile upgrade
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``JointAllParamUpgrade(path)``"
    "Description", "Joint full parameter profile upgrade"
    "Mandatory parameters", "- ``path``：Local upgrade package full path(D://zUP/XXXXX.bin)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Example of robot slave firmware upgrade code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.RobotEnable(0)
    time.sleep(0.2)
    rtn = robot.JointAllParamUpgrade("D://zUP/MT/joint0603/jointallparameters.db")
    print(f"robot JointAllParamUpgrade rtn is {rtn}")
    rtn = robot.SetCtrlFirmwareUpgrade(2, "D://zUP/MT/FAIR_Cobot_Cbd_Asix_V2.0.bin")
    print(f"robot SetCtrlFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetEndFirmwareUpgrade(2, "D://zUP/MT/FAIR_Cobot_Axle_Asix_V2.4.bin")
    print(f"robot SetEndFirmwareUpgrade rtn is {rtn}")
    robot.SetSysServoBootMode()
    time.sleep(0.2)
    rtn = robot.SetCtrlFirmwareUpgrade(1, "D://zUP/MT/FR_CTRL_PRIMCU_FV201412_MAIN_U4_T01_20250630(MT).bin")
    print(f"robot SetCtrlFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetEndFirmwareUpgrade(1, "D://zUP/MT/FR_END_FV2010010_MAIN_U1_T01_20250603.bin")
    print(f"robot SetEndFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetJointFirmwareUpgrade(1, "D://zUP/MT/FR_SERVO_FV504215_MAIN_U7_T07_20250603.bin")
    print(f"robot SetJointFirmwareUpgrade rtn is {rtn}")
    robot.CloseRPC()