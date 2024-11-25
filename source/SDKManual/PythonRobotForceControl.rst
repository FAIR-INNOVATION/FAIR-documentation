Force Control
====================================

.. toctree::
    :maxdepth: 5

Get Force Sensor Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_GetConfig()``"
    "Description", "Get Force Sensor Configuration"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``[number,company,device,softversion,bus]``：number 传感器编号;company  力传感器厂商，17-坤维科技，19-航天十一院，20-ATI 传感器，21-中科米点，22-伟航敏芯;device  设备号，坤维 (0- KWR75B), Aisino Eleven (0-MCS6A-200-4), ATI (0-AXIA80-M8), Zhongke MiDot (0-MST2010), Weihang Minxin (0-WHC6L-YB10A); softvesion software version number, not used for the time being, the default is 0." 

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    company = 17 #Sensor Manufacturer, 17-Kunwit Technology
    device = 0 # sensor device number
    error = robot.FT_SetConfig(company, device) #Configure force sensors
    print("Configuring force sensor error code",error)
    config = robot.FT_GetConfig() #Get force sensor configuration information
    print('Get force sensor configuration information',config)
    time.sleep(1)
    error = robot.FT_Activate(0) # sensor reset
    print("Sensor reset error code",error)
    time.sleep(1)
    error = robot.FT_Activate(1) #sensor activation
    print("Sensor activation error code",error)
    time.sleep(1)
    error = robot.SetLoadWeight(0.0) # end load set to zero
    print("End load set to zero error code",error)
    time.sleep(1)
    error = robot.SetLoadCoord(0.0,0.0,0.0) # end load center of mass set to zero
    print("End center of mass set to zero error code",error)
    time.sleep(1)
    error = robot.FT_SetZero(0) #sensor de-zeroing
    print("Sensor Remove Zero Error Code",error)
    time.sleep(1)
    error = robot.FT_GetForceTorqueOrigin() #Query sensor raw data
    print("Querying sensor raw data",error)
    error = robot.FT_SetZero(1) #Sensor zero correction, note that at this time the end can not be installed tools, only the force sensor
    print("Sensor zero correction",error)
    time.sleep(1)
    error = robot.FT_GetForceTorqueRCS() #Query data in sensor coordinate system
    print("Querying data in sensor coordinate system",error)

Force Sensor Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_SetConfig(company,device,softversion=0,bus=0)``"
    "Description", "Force Sensor Configuration"
    "Required Parameters","- ``company``: Sensor Manufacturer, 17 - Kunwei Technology, 19 - Aerospace 11th Academy, 20 - ATI Sensors, 21 - Zhongke MiDot, 22 - Weihang Minxin;
    - ``device``: device number, Kunwei (0-KWR75B), Aisino Eleventh Academy (0-MCS6A-200-4), ATI (0-AXIA80-M8), Zhongke MiDot (0-MST2010), Weihang Minxin (0-WHC6L-YB-10A);"
    "Default parameters", "- ``softversion``: software version number, not used for now, default is 0;
    - ``bus``: device mount end bus location, not used yet, default is 0;"
    "Return Value", "Error Code Success-0 Failure- errcode"

Force sensor activation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_Activate(state)``"
    "Description", "Force sensor activation"
    "Mandatory parameters", "- ``state``: 0-reset, 1-activate"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Force Sensor Zeroing
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_SetZero(state)``"
    "Description", "Force Transducer Zeroing"
    "Mandatory parameters", "- ``state``: 0-removal of zeros, 1-zero correction"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the force transducer reference coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``FT_SetRCS(ref,coord=[0,0,0,0,0,0,0])``"
    "Description", "Setting the force transducer reference coordinate system"
    "Mandatory parameters", "- ``ref``: 0 - tool coordinate system, 1 - base coordinate system"
    "Default parameters", "- ``coord``: [x,y,z,rx,ry,rz] customized coordinate system values, default [0,0,0,0,0,0,0]"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Load recognition, where the tool to be recognized is mounted at the end, underneath the force transducer, with the end pointing vertically downwards.
    error = robot.FT_SetRCS(0) #Set the reference coordinate system to the tool coordinate system, 0 - tool coordinate system, 1 - base coordinate system
    print('Setting reference coordinate system error code',error)
    time.sleep(1)
    tool_id = 10 #Sensor coordinate system number
    tool_coord = [0.0,0.0,35.0,0.0,0.0,0.0,0.0] # Sensor position relative to end flange
    tool_type = 1 # 0-tool, 1-sensor
    tool_install = 0 # 0 - end of installation, 1 - robot external
    error = robot.SetToolCoord(tool_id,tool_coord,tool_type,tool_install) #Set the sensor coordinate system, the sensor relative to the end flange position
    print('Setting sensor coordinate system error code',error)
    time.sleep(1)
    error = robot.FT_PdIdenRecord(tool_id) #record identification data
    print('Record load weight error code',error)
    time.sleep(1)
    error = robot.FT_PdIdenRecord() #Calculate the load weight in kg
    print('Calculating load weight error code',error)
    # Load center of mass recognition, the robot needs to be taught three different poses, then the recognition data is recorded and finally the load center of mass is calculated.
    robot.Mode(1)
    ret = robot.DragTeachSwitch(1) #Robot cuts to drag teach mode, must be in manual mode to cut to drag teach mode
    time.sleep(5)
    ret = robot.DragTeachSwitch(0)
    time.sleep(1)
    error = robot.FT_PdCogIdenRecord(tool_id,1)
    print('Load center of mass 1 error code',error)#record identification data
    ret = robot.DragTeachSwitch(1) #Robot cuts to drag teach mode, must be in manual mode to cut to drag teach mode
    time.sleep(5)
    ret = robot.DragTeachSwitch(0)
    time.sleep(1)
    error = robot.FT_PdCogIdenRecord(tool_id,2)
    print('Load center of mass 2 error code',error)
    ret = robot.DragTeachSwitch(1) #Robot cuts to drag teach mode, must be in manual mode to cut to drag teach mode
    time.sleep(5)
    ret = robot.DragTeachSwitch(0)
    time.sleep(1)
    error = robot.FT_PdCogIdenRecord(tool_id,3)
    print('Load center of mass 3 error code',error)
    time.sleep(1)
    error = robot.FT_PdCogIdenCompute()
    print('Load center of mass calculation error code',error)

Load weight identification calculation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_PdIdenCompute()``"
    "Description", "Load weight identification calculation"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode   
    - ``weight``: weight of the load in kg "

Load weight identification records
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_PdIdenRecord(tool_id)``"
    "Description", "Load weight identification record"
    "Mandatory parameters", "- ``tool_id``: sensor coordinate system number, range [0~14]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Load center of mass identification calculation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_PdCogIdenCompute()``"
    "Description", "Load center of mass identification calculation"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode  
    - ``cog=[cogx,cogy,cogz]``: center of mass of load in mm "

Load center of mass identification records
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``FT_PdCogIdenRecord(tool_id,index)``"
    "Description", "Load center of mass identification record"
    "Mandatory parameter", "- ``tool_id``: sensor coordinate system number, range [0~14].
    - ``index``: point number [1 to 3]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Obtaining force/torque data in the reference coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_GetForceTorqueRCS()``"
    "Description", "Get force/torque data in reference coordinate system"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``data=[fx,fy,fz,tx,ty,tz]``: force/torque data in the reference coordinate system."

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    import frrpc
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = frrpc.RPC('192.168.58.2')
    rcs = robot.FT_GetForceTorqueRCS() #Query data in sensor coordinate system
    print(rcs)

Obtaining Force Sensor Raw Force/Torque Data
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_GetForceTorqueOrigin()``"
    "Description", "Get force sensor raw force/torque data"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode  
    - ``data=[fx,fy,fz,tx,ty,tz]``: Force sensor raw force/torque data "

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    import frrpc
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = frrpc.RPC('192.168.58.2')
    origin = robot.FT_GetForceTorqueOrigin() #Query sensor raw data
    print(origin)

Collision Guard
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_Guard(flag,sensor_num,select,force_torque,max_threshold,min_threshold)``"
    "description", "collision guarding"
    "Required Parameters", "- ``flag``: 0 - turn off collision guarding, 1 - turn on collision guarding;
    - ``sensor_num``: force sensor number;
    - ``select``: whether or not the six degrees of freedom detect collisions [fx,fy,fz,mx,my,mz], 0 - not valid, 1 - valid;
    - ``force_torque``: collision detection force/torque in N or Nm;
    - ``max_threshold``: maximum threshold;
    - ``min_threshold``: minimum threshold;
    - Force/torque detection range: (force_torque-min_threshold,force_torque+max_threshold)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    #Collision Guard
    actFlag = 1 # turn on flag, 0 - turn off collision guarding, 1 - turn on collision guarding
    sensor_num = 1 # force sensor number
    is_select = [1,1,1,1,1,1] #Six degrees of freedom to select [fx,fy,fz,mx,my,mz], 0-not in effect, 1-in effect
    force_torque = [0.0,0.0,0.0,0.0,0.0,0.0,0.0] #Collision detection force and torque, detection range (force_torque-min_threshold,force_torque+max_threshold)
    max_threshold = [10.0,10.0,10.0,10.0,10.0,10.0] #max_threshold
    min_threshold = [5.0,5.0,5.0,5.0,5.0,5.0,5.0] #min_threshold
    p1=[-160.619,-586.138,384.988,-170.166,-44.782,169.295]
    p2=[-87.615,-606.209,556.119,-102.495,10.118,178.985]
    p3=[41.479,-557.243,484.407,-125.174,46.995,-132.165]
    error = robot.FT_Guard(actFlag, sensor_num, is_select, force_torque, max_threshold, min_threshold) #Enable collision guarding
    print("Collision guarding error code turned on",error)
    error = robot.MoveL(P1,1,0) # Cartesian space linear motion
    print("Cartesian space linear motion error code",error)
    error = robot.MoveL(P2,1,0)
    print("Cartesian space linear motion error code",error)
    error = robot.MoveL(P3,1,0)
    print("Cartesian space linear motion error code",error)
    actFlag = 0 
    error = robot.FT_Guard(actFlag, sensor_num, is_select, force_torque, max_threshold, min_threshold) #Turn off collision guarding
    print("Close collision guard error code",error)

constant force control
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype","``FT_Control(flag,sensor_num,select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)``"
    "Description", "Constant force control"
    "Mandatory parameter", "- ``flag``: constant force control on flag, 0-off, 1-on;
    - ``sensor_num``: force sensor number;
    - ``select``: whether the six degrees of freedom are detected [fx,fy,fz,mx,my,mz], 0 - not valid, 1 - valid;
    - ``force_torque``: detection of force/torque in N or Nm;
    - ``gain``: [f_p,f_i,f_d,m_p,m_i,m_d], force PID parameter, moment PID parameter;
    - ``adj_sign``: adaptive start/stop state, 0-off, 1-on;
    - ``ILC_sign``: ILC control start/stop status, 0-stop, 1-training, 2-practical;
    - ``max_dis``: maximum adjustment distance;
    - ``max_ang``: maximum angle of adjustment;"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Constant force control
    status = 1 # Constant force control on flag, 0 - off, 1 - on
    sensor_num = 1 # force sensor number
    is_select = [0,0,1,0,0,0,0] #Six degrees of freedom to select [fx,fy,fz,mx,my,mz], 0-not in effect, 1-in effect
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  
    gain = [0.0005,0.0,0.0,0.0,0.0,0.0,0.0] # force PID parameter, moment PID parameter
    adj_sign = 0 #Adaptive start/stop state, 0-off, 1-on
    ILC_sign = 0 #ILC control start/stop status, 0-stop, 1-training, 2-practice
    max_dis = 100.0 #Maximum adjustment distance
    max_ang = 0.0 #Maximum adjustment angle
    j1=[70.395, -46.976, 90.712, -133.442, -87.076, -27.138]
    p2=[-123.978, -674.129, 44.308, -178.921, 2.734, -172.449]
    p3=[123.978, -674.129, 42.308, -178.921, 2.734, -172.449]
    error = robot.MoveJ(J1,1,0)    
    print("Joint space motion instruction error code",error)
    error = robot.MoveL(P2,1,0)
    print("Cartesian space linear motion instruction error code",error)
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control on error code",error)
    error = robot.MoveL(P3,1,0) # Cartesian space linear motion
    print("Cartesian space linear motion instruction error code",error)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control end error code",error)

Helix Exploration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_SpiralSearch(rcs, ft, dr=0.7, max_t_ms=60000, max_vel=5)``"
    "Description", "Helix Explorations"
    "Mandatory parameters", "- ``rcs``: reference coordinate system, 0 - tool coordinate system, 1 - base coordinate system
    - ``ft``: force or moment threshold (0 to 100) in N or Nm;"
    "Default parameters","- ``dr``: radius feed per revolution in mm Default 0.7.
    - ``max_t_ms``: maximum time to explore, in ms default 60000.
    - ``max_vel``: maximum value of linear velocity in mm/s Default 5"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    p = [36.794, -675.119, 65.379, -176.938, 2.535, -179.829]
    # HENRY Parameters
    status = 1 # Constant force control on flag, 0 - off, 1 - on
    sensor_num = 1 # force sensor number
    is_select = [0,0,1,0,0,0,0] #Six degrees of freedom to select [fx,fy,fz,mx,my,mz], 0-not in effect, 1-in effect
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  
    gain = [0.0001,0.0,0.0,0.0,0.0,0.0] # force PID parameter, moment PID parameter
    adj_sign = 0 #Adaptive start/stop state, 0-off, 1-on
    ILC_sign = 0 #ILC control start/stop status, 0-stop, 1-training, 2-practice
    max_dis = 100.0 #Maximum adjustment distance
    max_ang = 5.0 #Maximum adjustment angle
    # Helix Exploration Parameters
    rcs = 0 # reference coordinate system, 0-tool coordinate system, 1-base coordinate system
    fFinish = 10 # Force or moment threshold (0~100) in N or Nm
    error = robot.MoveL(P,1,0) #Cartesian space linear motion to initial point
    print("Cartesian space linear motion to initial point",error)
    is_select = [0,0,1,1,1,0] #Six degrees of freedom to select [fx,fy,fz,mx,my,mz], 0-not in effect, 1-in effect
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign, max_dis,max_ang)
    print("Constant force control on error code",error)
    error = robot.FT_SpiralSearch(rcs,fFinish,max_vel=3)
    print("Helix Exploration Error Code",error)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign, max_dis,max_ang)
    print("Constant force control off error code",error)

Rotary insertion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_RotInsertion(rcs, ft, orn, angVelRot=3, angleMax=45, angAccmax=0, rotorn=1)``"
    "Description", "Rotary Insertion"
    "Mandatory parameters", "- ``rcs``: reference coordinate system, 0 - tool coordinate system, 1 - base coordinate system;
    - ``ft``: force or moment threshold (0 to 100) in N or Nm.
    - ``orn``: direction of force/torque, 1 - along the z-axis, 2 - around the z-axis;"
    "Default parameter", "- ``angVelRot``: angular speed of rotation in °/s, default 3;
    - ``angleMax``: maximum angle of rotation, in ° default 45.
    - ``angAccmax``: maximum rotational acceleration in °/s^2, not used yet Default 0.
    - ``rotorn``: direction of rotation, 1 - clockwise, 2 - counterclockwise Default 1"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    p = [36.794, -675.119, 65.379, -176.938, 2.535, -179.829]
    # HENRY Parameters
    status = 1 # Constant force control on flag, 0 - off, 1 - on
    sensor_num = 1 # force sensor number
    is_select = [0,0,1,0,0,0,0] #Six degrees of freedom to select [fx,fy,fz,mx,my,mz], 0-not in effect, 1-in effect
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0] 
    gain = [0.0001,0.0,0.0,0.0,0.0,0.0] # force PID parameter, moment PID parameter   
    adj_sign = 0 #Adaptive start/stop state, 0-off, 1-on
    ILC_sign = 0 #ILC control start/stop status, 0-stop, 1-training, 2-practice
    max_dis = 100.0 #Maximum adjustment distance
    max_ang = 5.0 #Maximum adjustment angle
    # Rotary insertion parameters
    rcs = 0 # reference coordinate system, 0-tool coordinate system, 1-base coordinate system
    forceInsertion = 2.0 # force or moment threshold (0~100) in N or Nm
    orn = 1 # direction of force, 1-fz,2-mz
    # Default parameter angVelRot: angular velocity of rotation in °/s Default 3
    #Default parameter angleMax: Maximum rotation angle in ° Default 5
    #Default parameter angAccmax: maximum rotational acceleration in °/s^2, not used yet Default 0
    #Default parameter rotorn: direction of rotation, 1-clockwise, 2-counterclockwise Default 1
    error = robot.MoveL(P,1,0) #Cartesian space linear motion to initial point
    print("Cartesian space linear motion to initial point",error)
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control on error code",error)
    error = robot.FT_RotInsertion(rcs,1,orn)
    print("Rotary insertion error code",error)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control off error code",error)

Linear insertion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``FT_LinInsertion(rcs, ft, disMax, linorn, lin_v=1.0, lin_a=1.0)``"
    "Description", "Linear Insertion"
    "Mandatory parameters", "- ``rcs``: reference coordinate system, 0 - tool coordinate system, 1 - base coordinate system;
    - ``ft``: force or moment threshold (0 to 100) in N or Nm.
    - ``disMax``: maximum insertion distance in mm.
    - ``linorn``: insertion direction: 0-negative direction, 1-positive direction"
    "Default parameters","- ``lin_v``: linear velocity in mm/s Default 1.
    - ``lin_a``: linear acceleration in mm/s^2, not used yet Default 1"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:
    
    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    p = [36.794, -675.119, 65.379, -176.938, 2.535, -179.829]
    # HENRY Parameters
    status = 1 # Constant force control on flag, 0 - off, 1 - on
    sensor_num = 1 # force sensor number
    is_select = [0,0,1,0,0,0,0] #Six degrees of freedom to select [fx,fy,fz,mx,my,mz], 0-not in effect, 1-in effect
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  
    gain = [0.0001,0.0,0.0,0.0,0.0,0.0] # force PID parameter, moment PID parameter
    adj_sign = 0 #Adaptive start/stop state, 0-off, 1-on
    ILC_sign = 0 #ILC control start/stop status, 0-stop, 1-training, 2-practice
    max_dis = 100.0 #Maximum adjustment distance
    max_ang = 5.0 #Maximum adjustment angle
    # Linear insertion parameters
    rcs = 0 # reference coordinate system, 0-tool coordinate system, 1-base coordinate system
    force_goal = 10.0 # Force or moment threshold (0~100) in N or Nm
    disMax = 100.0 # Maximum insertion distance in mm
    linorn = 1 # insertion direction, 1-positive direction, 2-negative direction
    #Default parameter lin_v: linear velocity in mm/s Default 1
    #Default parameter lin_a: linear acceleration in mm/s^2, not used Default 0
    error = robot.MoveL(P,1,0) #Cartesian space linear motion to initial point
    print("Cartesian space linear motion to initial point",error)
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control on error code",error)
    error = robot.FT_LinInsertion(rcs,force_goal,disMax,linorn)
    print("Linear insertion error code",error)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control off error code",error)

Calculation of the center plane position begins
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_CalCenterStart()``"
    "Description", "Calculation of mid-plane position begins"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Calculate end of mid-plane position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_CalCenterEnd()``"
    "Description", "End of calculation of mid-plane position"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode
    - ``pos=[x,y,z,rx,ry,rz]``: mid-plane position"

Surface positioning
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``FT_FindSurface (rcs, dir, axis, disMax, ft, lin_v=3.0, lin_a=0.0)``"
    "Description", "Surface Positioning"
    "Mandatory parameters", "- ``rcs``: reference coordinate system, 0 - tool coordinate system, 1 - base coordinate system;
    - ``dir``: direction of movement, 1-positive, 2-negative;
    - ``axis``: moving axes, 1-x, 2-y, 3-z;
    - ``disMax``: large exploration distance in mm.
    - ``ft``: action termination force threshold in N;"
    "Default Parameters", "- ``lin_v``: explore linear velocity in mm/s default 3.
    - ``lin_a``: explore linear acceleration in mm/s^2 default 0;"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    # Constant force control
    status = 1 # Constant force control on flag, 0 - off, 1 - on
    sensor_num = 1 # force sensor number
    is_select = [1,0,0,0,0,0,0] #Six degrees of freedom to select [fx,fy,fz,mx,my,mz], 0-not in effect, 1-in effect
    force_torque = [-2.0,0.0,0.0,0.0,0.0,0.0]  
    gain = [0.0002,0.0,0.0,0.0,0.0,0.0,0.0] # force PID parameter, moment PID parameter
    adj_sign = 0 #Adaptive start/stop state, 0-off, 1-on
    ILC_sign = 0 #ILC control start/stop status, 0-stop, 1-training, 2-practice
    max_dis = 15.0 #Maximum adjustment distance
    max_ang = 0.0 #Maximum adjustment angle
    # Surface positioning parameters
    rcs = 0 # reference coordinate system, 0-tool coordinate system, 1-base coordinate system
    direction = 1 # direction of movement, 1-positive direction, 2-negative direction
    axis = 1 # move axis, 1-X,2-Y,3-Z
    lin_v = 3.0 #Explore linear velocity in mm/s
    lin_a = 0.0 # Explore linear acceleration in mm/s^2
    disMax = 50.0 # Maximum exploration distance in mm
    force_goal = 2.0 # Action termination force threshold in N
    p1=[-77.24,-679.599,58.328,179.373,-0.028,-167.849]
    Robot.MoveCart(P1,1,0) #joint space point-to-point motion
    #x direction to find the center
    #1 surface
    error = robot.FT_CalCenterStart()
    print("Calculating the start of the midplane error code",error)
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control start error code",error)
    error = robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
    print("Looking for X+ surface error code",error)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control end error code",error)
    time.sleep(2)
    error = robot.MoveCart(P1,1,0) #joint space point-to-point motion
    print("Joint space point-to-point motion error code",error)
    time.sleep(5)
    #2nd surface
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control start error code",error)
    direction = 2 # direction of movement, 1-positive direction, 2-negative direction
    error = robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
    print("Looking for X-surface error code",error)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control end error code",error)
    # Calculate the x-direction center position
    error,xcenter = robot.FT_CalCenterEnd()
    print("Calculation of end-of-middle-plane error code in x-direction",xcenter) 
    error = robot.MoveCart(xcenter,1,0)
    print("Joint space point-to-point motion error code",error)
    time.sleep(1)
    #y direction to find the center
    #1 surface
    error =robot.FT_CalCenterStart()
    print("Calculating the start of the midplane error code",error)
    error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control start error code",error)
    direction = 1 # direction of movement, 1-positive direction, 2-negative direction
    axis = 2 # move axis, 1-X,2-Y,3-Z
    disMax = 150.0 # Maximum exploration distance in mm
    lin_v = 6.0 #Explore linear velocity in mm/s
    error =robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
    print("Looking for surface Y + error code",error)
    status = 0
    error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control end error code",error)
    error =robot.MoveCart(P1,1,0) #joint space point-to-point motion
    print("Joint space point-to-point motion error code",error)
    Robot.WaitMs(1000)
    #2nd surface
    error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control start error code",error)
    direction = 2 # direction of movement, 1-positive direction, 2-negative direction
    error =robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
    print("Looking for surface Y-error code",error)
    status = 0
    error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control end error code",error)
    # Calculate the y-direction center position
    error,ycenter=robot.FT_CalCenterEnd()
    print("Calculating end of mid-plane Y-direction error code",ycenter)
    error =robot.MoveCart(ycenter,1,0)
    print("Joint space point-to-point motion error code",error)

Soft control off
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_ComplianceStop()``"
    "Description", "Softening control off"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Soft control on
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_ComplianceStart(p, force)``"
    "Description", "Softening control on"
    "Mandatory parameters", "- ``p``: Position adjustment factor or softness factor
    - ``force``: soft opening force threshold in N"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j1=[75.005,-46.434,90.687,-133.708,-90.315,-27.139]
    p2=[-77.24,-679.599,38.328,179.373,-0.028,-167.849]
    p3=[77.24,-679.599,38.328,179.373,-0.028,-167.849]
    # Constant force control parameters
    status = 1 # Constant force control on flag, 0 - off, 1 - on
    sensor_num = 1 # force sensor number
    is_select = [1,1,1,0,0,0] #Six degrees of freedom to select [fx,fy,fz,mx,my,mz], 0-not in effect, 1-in effect
    force_torque = [-10.0,-10.0,-10.0,0.0,0.0,0.0] 
    gain = [0.0005,0.0,0.0,0.0,0.0,0.0,0.0] # force PID parameter, moment PID parameter
    adj_sign = 0 #Adaptive start/stop state, 0-off, 1-on
    ILC_sign = 0 #ILC control start/stop status, 0-stop, 1-training, 2-practice
    max_dis = 1000.0 #Maximum adjustment distance
    max_ang = 0.0 #Maximum adjustment angle
    error = robot.MoveJ(J1,1,0)
    print("Joint space movement to point 1 error code",error)
    #Smooth control
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control start error code",error)
    p = 0.00005 # Position adjustment factor or softness factor
    force = 30.0 # soft opening force threshold in N
    error = robot.FT_ComplianceStart(p,force)
    print("SoftControl start error code",error)
    error = robot.MoveL(P2,1,0,vel =10) # Cartesian space linear motion
    print("Cartesian space linear motion to point 2 error code", error)
    error = robot.MoveL(P3,1,0,vel =10)
    print("Cartesian space linear motion to point 3 error code", error)
    time.sleep(1)
    error = robot.FT_ComplianceStop()
    print("soft control end error code",error)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Constant force control off error code",error)

Load recognition filter initialization
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoadIdentifyDynFilterInit()``"
    "Description", "Load Recognition Filter Initialization"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot

    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    # Load recognition filter initialization
    error = robot.LoadIdentifyDynFilterInit()
    print("LoadIdentifyDynFilterInit:",error)
    # Load recognition variable initialization
    error = robot.LoadIdentifyDynVarInit()
    print("LoadIdentifyDynVarInit:",error)

    joint_torque= [0,0,0,0,0,0,0]
    joint_pos= [0,0,0,0,0,0,0]
    gain=[0,0.05,0,0,0,0,0,0,0.02,0,0,0,0]
    t =10
    error,joint_pos=robot.GetActualJointPosDegree(1)
    joint_pos[1] = joint_pos[1]+10
    error,joint_torque=robot.GetJointTorques(1)
    joint_torque[1] = joint_torque[1]+2
    # Load Recognition Main Program
    error = robot.LoadIdentifyMain(joint_torque, joint_pos, t)
    print("LoadIdentifyMain:",error)
    # Get load recognition results
    error = robot.LoadIdentifyGetResult(gain)
    print("LoadIdentifyGetResult:",error)

Initialization of load recognition variables
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoadIdentifyDynVarInit()``"
    "Description", "Load Identification Variable Initialization"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Load Recognition Main Program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoadIdentifyMain(joint_torque, joint_pos, t)``"
    "Description", "Load Recognition Master Program"
    "Mandatory parameters", "- ``joint_torque``: Joint torque j1-j6;
    - ``joint_pos``: joint position j1-j6
    - ``t``: sampling period"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Getting Load Recognition Results
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoadIdentifyGetResult(gain)``"
    "Description", "Get load identification results"
    "Mandatory parameters", "- ``gain``: gravity term coefficient double[6], centrifugal term coefficient double[6]"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``weight``: weight of the load
    - ``cog=[x,y,z]``: load center of mass coordinates"

Force Sensor Assisted Drag
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ForceAndJointImpedanceStartStop(status, impedanceFlag, lamdeDain, KGain, BGain, dragMaxTcpVel, dragMaxTcpOriVel)``"
    "Description", "Force sensor assisted drag"
    "Mandatory parameters", "- ``status``: control status, 0 - off; 1 - on
    - ``impedanceFlag``: impedance on flag, 0 - off; 1 - on
    - ``lamdeDain``: [D1, D2, D3, D4, D5, D6] drag gain
    - ``KGain``: [K1,K2,K3,K4,K5,K6] Stiffness Gain
    - ``BGain``: [B1,B2,B3,B4,B5,B] damping gain
    - ``dragMaxTcpVel``: maximum line speed limit at the end of a drag
    - ``dragMaxTcpOriVel``: drag end maximum angular velocity limit"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    status = 1 # control status, 0-off; 1-on
    asaptiveFlag = 1 #Adaptive on flag, 0-off; 1-on
    interfereDragFlag = 1 # interference zone drag flag, 0-off; 1-on
    M = [15, 15, 15, 0.5, 0.5, 0.1] #Inertia factor
    B = [150, 150, 150, 5, 5, 1] # Damping factor
    K = [0, 0, 0, 0, 0, 0, 0] #Stiffness factor
    F = [5, 5, 5, 1, 1, 1] #drag six-dimensional force thresholds
    Fmax = 50 # Maximum towing force limit
    Vmax = 1810 # Maximum joint speed limit

    error = robot.EndForceDragControl(status, asaptiveFlag, interfereDragFlag, M, B, K, F, Fmax, Vmax)
    print("EndForceDragControl return:",error)

    time.sleep(10)
    status=0
    error = robot.EndForceDragControl(status, asaptiveFlag, interfereDragFlag, M, B, K, F, Fmax, Vmax)
    print("EndForceDragControl return:",error)

The force sensor turns on automatically after the error is cleared.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetForceSensorDragAutoFlag(status)``"
    "Description", "The force sensor turns on automatically after an error is cleared."
    "Mandatory parameters", "- ``status``: control status, 0 - off; 1 - on"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
    
code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')

    error = robot. SetForceSensorDragAutoFlag (1)
    print("SetForceSensorDragAutoFlag return:",error)
    
Setting up hybrid drag switches and parameters for six-dimensional force and joint impedance
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``EndForceDragControl(status, asaptiveFlag, interfereDragFlag, M, B, K, F, Fmax, Vmax)``"
    "Description", "Setting up the six-dimensional force and joint impedance hybrid drag switch and parameters"
    "Mandatory parameters", "- ``status``: control status, 0 - off; 1 - on
    - ``asaptiveFlag``: adaptive on flag, 0 - off; 1 - on
    - ``interfereDragFlag``: interference area drag flag, 0 - off; 1 - on
    - ``M=[m1,m2,m3,m4,m5,m6]``: inertia factor
    - ``B=[b1,b2,b3,b4,b5,b6]``: damping factor
    - ``K=[k1,k2,k3,k4,k5,k6]``: stiffness factor
    - ``F=[f1,f2,f3,f4,f5,f6]``: drag six-dimensional force thresholds
    - ``Fmax``: Maximum towing force limitation
    - ``Vmax``: maximum joint speed limit"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
    
code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    status = 1 # control status, 0-off; 1-on
    impedanceFlag = 1 # impedance on flag, 0-off; 1-on
    lamdeDain = [ 3.0, 2.0, 2.0, 2.0, 2.0, 2.0, 3.0] # drag gain
    KGain = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00] # Stiffness gain
    BGain = [150, 150, 150, 5.0, 5.0, 1.0] # Damping gain
    dragMaxTcpVel = 1000 # Maximum line speed limit at the end of the drag
    dragMaxTcpOriVel = 180 # Maximum angular velocity limit at end of drag

    error = robot.DragTeachSwitch(1)
    print("DragTeachSwitch 1 return:",error)

    error = robot.ForceAndJointImpedanceStartStop(status, impedanceFlag, lamdeDain, KGain, BGain,dragMaxTcpVel,dragMaxTcpOriVel)
    print("ForceAndJointImpedanceStartStop return:",error)

    error = robot.GetForceAndTorqueDragState()
    print("GetForceAndTorqueDragState return:",error)

    time.sleep(10)

    status = 0 # control status, 0-off; 1-on
    impedanceFlag = 0 # impedance on flag, 0-off; 1-on
    error = robot.ForceAndJointImpedanceStartStop(status, impedanceFlag, lamdeDain, KGain, BGain,dragMaxTcpVel,dragMaxTcpOriVel)
    print("ForceAndJointImpedanceStartStop return:",error)

    error = robot.GetForceAndTorqueDragState()
    print("GetForceAndTorqueDragState return:",error)

    error = robot.DragTeachSwitch(0)
    print("DragTeachSwitch 0 return:",error)
        
Get force sensor drag switch status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetForceAndTorqueDragState()``"
    "Description", "Get force sensor drag switch status"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``dragState``: force sensor assisted drag control state, 0 - off; 1 - on
    - ``sixDimensionalDragState``: six-dimensional force-assisted drag control state, 0-off; 1-on"
        
Setting the load weight under the force transducer
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetForceSensorPayload(weight)``"
    "Description", "Set the load weight under the force transducer"
    "Mandatory parameters", " - ``weight``: load weight kg"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
        
code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    error = robot.SetForceSensorPayload(0.8)
    print("SetForceSensorPayload return:",error)

    error = robot.SetForceSensorPayloadCog(0.5,0.6,12.5)
    print("SetForceSensorPayLoadCog return:",error)

    error = robot.GetForceSensorPayload()
    print("GetForceSensorPayLoad return:",error)

    error = robot.GetForceSensorPayloadCog()
    print("GetForceSensorPayLoadCog return:",error)
            
Setting the load center of mass under the force transducer
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetForceSensorPayloadCog(x,y,z)``"
    "Description", "Setting the center of mass of the load under the force transducer"
    "Mandatory parameters", "
    - ``x``: load center of mass x mm
    - ``y``: load center of mass y mm
    - ``z``: load center of mass z mm
    "
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
            
Getting the load weight under the force transducer
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetForceSensorPayload()``"
    "Description", "Get the weight of the load under the force transducer."
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``weight``: loaded weight kg"
            
Obtaining the center of mass of the load under the force transducer
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetForceSensorPayloadCog()``"
    "Description", "Getting the center of mass of the load under the force transducer"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``x``: load center of mass x mm 
    - ``y``: load center of mass y mm 
    - ``z``: load center of mass z mm"
            
Automatic zeroing of force sensors
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ForceSensorAutoComputeLoad()``"
    "Description", "Automatic zeroing of force sensors"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``weight``: sensor mass kg
    - ``pos=[x,y,z]``: sensor center of mass mm"
        
code example
-------------------------------------------------------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful

    robot = Robot.RPC('192.168.58.2')

    error = robot.SetForceSensorPayload(0)
    print("SetForceSensorPayload return:",error)

    error = robot.SetForceSensorPayloadCog(0,0,0)
    print("SetForceSensorPayLoadCog return:",error)

    error = robot.ForceSensorAutoComputeLoad()
    print("ForceSensorAutoComputeLoad return:",error)

Sensor auto-zero data logging
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ForceSensorSetSaveDataFlag(recordCount)``"
    "Description", "Sensor auto-zero data logging"
    "Mandatory parameters", "- ``recordCount``: number of record data 1-3"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Fail-errcode"

Automatic sensor zeroing calculation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ForceSensorComputeLoad()``"
    "Description", "Sensor auto-zero data logging"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Fail-errcode
    - ``weight``: sensor mass kg 
    - ``pos=[x,y,z]``: sensor center of mass mm"