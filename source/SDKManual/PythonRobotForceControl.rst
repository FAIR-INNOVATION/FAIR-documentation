Force Control
====================================

.. toctree::
    :maxdepth: 5

Force Sensor Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_SetConfig(company,device,softversion=0,bus=0)``"
    "Description", "Force Sensor Configuration"
    "Required Parameters","- ``company``: Sensor Manufacturer, 17 - Kunwei Technology, 19 - Aerospace 11th Academy, 20 - ATI Sensors, 21 - Zhongke MiDot, 22 - Weihang Minxin，23-NBIT，24-XJC，26-NSR;
    - ``device``: device number, Kunwei (0-KWR75B), Aisino Eleventh Academy (0-MCS6A-200-4), ATI (0-AXIA80-M8), Zhongke MiDot (0-MST2010), Weihang Minxin (0-WHC6L-YB-10A)，NBIT(0-XLH93003ACS)，XJC(0-XJC-6F-D82)，NSR(0-NSR-FTSensorA);"
    "Default parameters", "- ``softversion``: software version number, not used for now, default is 0;
    - ``bus``: device mount end bus location, not used yet, default is 0;"
    "Return Value", "Error Code Success-0 Failure- errcode"

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

Force sensor configuration and automatic zero correction code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1
    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)
    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)
    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)
    time.sleep(1)
    robot.FT_SetZero(0)
    time.sleep(1)
    error,ft = robot.FT_GetForceTorqueOrigin()
    print(f"ft origin:{ft[0]},{ft[1]},{ft[2]},{ft[3]},{ft[4]},{ft[5]}")
    robot.FT_SetZero(1)
    time.sleep(1)
    ftCoord = [0, 0, 0, 0, 0, 0]
    robot.FT_SetRCS(0, ftCoord)
    robot.SetForceSensorPayload(0.824)
    robot.SetForceSensorPayloadCog(0.778, 2.554, 48.765)
    error,weight = robot.GetForceSensorPayload()
    error,x, y, z = robot.GetForceSensorPayloadCog()
    print(f"the FT load is  {weight}, {x} {y} {z}")
    robot.SetForceSensorPayload(0)
    robot.SetForceSensorPayloadCog(0, 0, 0)
    error,computeWeight, tran = robot.ForceSensorAutoComputeLoad()
    print(f"the result is weight {computeWeight} pos is {tran[0]} {tran[1]} {tran[2]}")
    robot.CloseRPC()

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

Force sensor load identification code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1
    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)
    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)
    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)
    robot.FT_SetZero(0)
    time.sleep(1)
    error,ft = robot.FT_GetForceTorqueOrigin()
    print(f"ft origin:{ft[0]},{ft[1]},{ft[2]},{ft[3]},{ft[4]},{ft[5]}")
    robot.FT_SetZero(1)
    time.sleep(1)
    tcoord = [0, 0, 35.0, 0, 0, 0]
    robot.SetToolCoord(10, tcoord, 1, 0, 0, 0)
    robot.FT_PdIdenRecord(10)
    time.sleep(1)
    error,weight = robot.FT_PdIdenCompute()
    print(f"payload weight:{weight}")
    desc_p1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_p2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    desc_p3 = [-327.622, 402.230, 320.402, -178.067, 2.127, -46.207]
    robot.MoveCart(desc_p1, 0, 0, 100.0)
    time.sleep(1)
    robot.FT_PdCogIdenRecord(10, 1)
    robot.MoveCart(desc_p2, 0, 0, 100.0)
    time.sleep(1)
    robot.FT_PdCogIdenRecord(10, 2)
    robot.MoveCart(desc_p3, 0, 0, 100.0)
    time.sleep(1)
    robot.FT_PdCogIdenRecord(10, 3)
    time.sleep(1)
    error,cog = robot.FT_PdCogIdenCompute()
    print(f"FT_PdCogIdenCompute return {error}")
    print(f"cog:{cog[0]},{cog[1]},{cog[2]}")
    robot.CloseRPC()

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

Collision guard code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1
    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)
    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)
    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)
    robot.FT_SetZero(0)
    time.sleep(1)
    sensor_id = 1
    select = [1, 1, 1, 1, 1, 1]
    max_threshold = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    min_threshold = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
    ft = None 
    desc_p1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_p2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    desc_p3 = [-327.622, 402.230, 320.402, -178.067, 2.127, -46.207]
    error = robot.FT_Guard(1, sensor_id, select,[0.0,0.0,0.0,0.0,0.0,0.0], max_threshold, min_threshold)
    robot.MoveCart(desc_p1, 0, 0, 100.0)
    robot.MoveCart(desc_p2, 0, 0, 100.0)
    robot.MoveCart(desc_p3, 0, 0, 100.0)
    robot.FT_Guard(0, sensor_id, select,[0.0,0.0,0.0,0.0,0.0,0.0], max_threshold, min_threshold)
    robot.CloseRPC()

constant force control
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype","``FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M=None, B=None, threshold=[0.2,0.2], adjustCoeff=[1.0,1.0], polishRadio=0, filter_Sign=0, posAdapt_sign=0, isNoBlock=0)``"
    "Description", "Constant force control"
    "Mandatory parameter", "- ``flag``: constant force control on flag, 0-off, 1-on;
    - ``sensor_id``: force sensor number;
    - ``select``: whether the six degrees of freedom are detected [fx,fy,fz,mx,my,mz], 0 - not valid, 1 - valid;
    - ``ft``: detection of force/torque in N or Nm;
    - ``ft_pid``: [f_p,f_i,f_d,m_p,m_i,m_d], force PID parameter, moment PID parameter;
    - ``adj_sign``: adaptive start/stop state, 0-off, 1-on;
    - ``ILC_sign``: ILC control start/stop status, 0-stop, 1-training, 2-practical;
    - ``max_dis``: maximum adjustment distance;
    - ``max_ang``: maximum angle of adjustment;"
    "Default parameters", "- ``M``：Quality parameters；
    - ``B``：Damping parameter；
    - ``threshold``：Activation threshold for rx, ry [0-10], default 0.2；
    - ``adjustCoeff``：Torque adjustment coefficient for rx, ry [0-1], default 1；
    - ``polishRadio``：Grinding disc radius, unit: mm；
    - ``filter_Sign``：Filter on indicator 0- off; 1- On, default 0- off；
    - ``posAdapt_sign``：The posture conforms to the opening mark 0-off. 1- On, default 0- off；
    - ``isNoBlock``：Block flag, 0- block; 1- Non-blocking default 0- blocking；"
    "Return Value", "Error Code Success-0 Failure- errcode "

A code example of constant force control with damping
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    robot = Robot.RPC('192.168.58.2')
    sensor_id = 10
    select = [0, 0, 1, 0, 0, 0]
    ft_pid = [0.0008, 0.0, 0.0, 0.0, 0.0, 0.0]
    adj_sign = 0
    ILC_sign = 0
    max_dis = 1000.0
    max_ang = 20.0
    ft = [0.0] * 6 
    epos = [0.0] * 4
    j1 = [80.765, -98.795, 106.548, -97.734, -89.999, 94.842]
    j2 = [43.067, -84.429, 92.620, -98.175, -90.011, 57.144]
    desc_p1 = [5.009, -547.463, 262.053, -179.999, -0.019, 75.923]
    desc_p2 = [-347.966, -547.463, 262.048, -180.000, -0.019, 75.923]
    offset_pos = [0.0] * 6
    M = [2.0, 2.0]
    B = [15.0, 15.0]
    threshold = [1.0, 1.0]
    adjustCoeff = [1.0, 0.8]
    polishRadio = 0.0
    filter_Sign = 0
    posAdapt_sign = 1
    isNoBlock = 0
    ft[2] = -10.0 
    while True:
        rtn = robot.FT_Control(1, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, threshold,adjustCoeff, 0, 0, 1, 0)
        print(f"FT_Control start rtn is {rtn}")
        rtn = robot.MoveL(desc_pos=desc_p1, tool=1, user=0, vel=100, acc=100, ovl=100, blendR=-1, blendMode = 0, exaxis_pos=epos, search=0, offset_flag=0, offset_pos=offset_pos)
        rtn = robot.MoveL(desc_pos=desc_p2, tool=1, user=0, vel=100, acc=100, ovl=100, blendR=-1, blendMode = 0, exaxis_pos=epos, search=0, offset_flag=0, offset_pos=offset_pos)
        rtn = robot.FT_Control(0, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, threshold,adjustCoeff, 0, 0, 1, 0)
        print(f"FT_Control end rtn is {rtn}")
    robot.CloseRPC()
    return 0

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

Rotational Insertion
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_RotInsertion(rcs, ft, orn, angVelRot=3, angleMax=45, angAccmax=0, rotorn=1, strategy=0)``"
    "Description", "Rotational Insertion"
    "Required Parameters", "- ``rcs``: Reference coordinate system, 0 - Tool coordinate system, 1 - Base coordinate system;
    - ``ft``: Force or torque threshold (0~100), unit N or Nm;
    - ``orn``: Force/torque direction, 1 - Along Z-axis direction, 2 - Around Z-axis direction;"
    "Default Parameters", "- ``angVelRot``: Rotational angular velocity, unit °/s, default 3;
    - ``angleMax``: Maximum rotation angle, unit °, default 45;
    - ``angAccmax``: Maximum rotational acceleration, unit °/s^2, currently unused, default 0;
    - ``rotorn``: Rotation direction, 1 - Clockwise, 2 - Counterclockwise, default 1;
    - ``strategy``: Processing strategy for undetected force/torque, 0 - Error; 1 - Warning, continue motion"
    "Return Value", "Error code. Success - 0, Failure - errcode"

Code Example for Spiral Search, Linear Insertion, and Other Commands
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    robot = Robot.RPC('192.168.58.2')
    j1=[-11.904,-99.669,117.473,-108.616,-91.726,74.256]
    j2=[-45.615,-106.172,124.296,-107.151,-91.282,74.255]
    j3=[-29.777,-84.536,109.275,-114.075,-86.655,74.257]
    j4=[-31.154,-95.317,94.276,-88.079,-89.740,74.256]
    desc_pos1=[-419.524,-13.000,351.569,-178.118,0.314,3.833]
    desc_pos2=[-321.222,185.189,335.520,-179.030,-1.284,-29.869]
    desc_pos3=[-487.434,154.362,308.576,176.600,0.268,-14.061]
    desc_pos4=[-443.165,147.881,480.951,179.511,-0.775,-15.409]
    offset_pos=[0.0]*6
    epos=[0.0]*4
    tool=0
    user=0
    vel=100.0
    acc=100.0
    ovl=100.0
    oacc=100.0
    blendT=0.0
    blendR=0.0
    flag=0
    search=0
    blendMode=0
    velAccMode=0
    robot.SetSpeed(20)
    rtn=robot.MoveJ(joint_pos=j1,tool=tool,user=user,vel=vel,acc=acc,ovl=ovl,exaxis_pos=epos,blendT=blendT,offset_flag=flag,offset_pos=offset_pos)
    print(f"movejerrcode:{rtn}")
    rtn=robot.MoveL(desc_pos=desc_pos2,tool=tool,user=user,vel=vel,acc=acc,ovl=ovl,blendR=blendR,blendMode=blendMode,exaxis_pos=epos,search=search,offset_flag=flag,offset_pos=offset_pos,oacc=oacc,velAccParamMode=velAccMode)
    print(f"movelerrcode:{rtn}")
    rtn=robot.MoveC(desc_pos_p=desc_pos3,tool_p=tool,user_p=user,vel_p=vel,acc_p=acc,exaxis_pos_p=epos,offset_flag_p=flag,offset_pos_p=offset_pos,desc_pos_t=desc_pos4,tool_t=tool,user_t=user,vel_t=vel,acc_t=acc,exaxis_pos_t=epos,offset_flag_t=flag,offset_pos_t=offset_pos,ovl=ovl,blendR=blendR,oacc=oacc,velAccParamMode=velAccMode)
    print(f"movecerrcode:{rtn}")
    rtn=robot.MoveJ(joint_pos=j2,tool=tool,user=user,vel=vel,acc=acc,ovl=ovl,exaxis_pos=epos,blendT=blendT,offset_flag=flag,offset_pos=offset_pos)
    print(f"movejerrcode:{rtn}")
    rtn=robot.Circle(desc_pos_p=desc_pos3,tool_p=tool,user_p=user,vel_p=vel,acc_p=acc,exaxis_pos_p=epos,desc_pos_t=desc_pos1,tool_t=tool,user_t=user,vel_t=vel,acc_t=acc,exaxis_pos_t=epos,ovl=ovl,offset_flag=flag,offset_pos=offset_pos,oacc=oacc,blendR=-1,velAccParamMode=velAccMode)
    print(f"circleerrcode:{rtn}")
    rtn=robot.MoveCart(desc_pos=desc_pos4,tool=tool,user=user,vel=vel,acc=acc,ovl=ovl,blendT=blendT,config=-1)
    print(f"MoveCarterrcode:{rtn}")
    rtn=robot.MoveJ(joint_pos=j1,tool=tool,user=user,vel=vel,acc=acc,ovl=ovl,exaxis_pos=epos,blendT=blendT,offset_flag=flag,offset_pos=offset_pos)
    print(f"movejerrcode:{rtn}")
    rtn=robot.MoveL(desc_pos=desc_pos2,tool=tool,user=user,vel=vel,acc=acc,ovl=ovl,blendR=blendR,blendMode=blendMode,exaxis_pos=epos,search=search,offset_flag=flag,offset_pos=offset_pos,config=-1,velAccParamMode=velAccMode)
    print(f"movelerrcode:{rtn}")
    rtn=robot.MoveC(desc_pos_p=desc_pos3,tool_p=tool,user_p=user,vel_p=vel,acc_p=acc,exaxis_pos_p=epos,offset_flag_p=flag,offset_pos_p=offset_pos,desc_pos_t=desc_pos4,tool_t=tool,user_t=user,vel_t=vel,acc_t=acc,exaxis_pos_t=epos,offset_flag_t=flag,offset_pos_t=offset_pos,ovl=ovl,blendR=blendR,config=-1,velAccParamMode=velAccMode)
    print(f"movecerrcode:{rtn}")
    rtn=robot.MoveJ(joint_pos=j2,tool=tool,user=user,vel=vel,acc=acc,ovl=ovl,exaxis_pos=epos,blendT=blendT,offset_flag=flag,offset_pos=offset_pos)
    print(f"movejerrcode:{rtn}")
    rtn=robot.Circle(desc_pos_p=desc_pos3,tool_p=tool,user_p=user,vel_p=vel,acc_p=acc,exaxis_pos_p=epos,desc_pos_t=desc_pos1,tool_t=tool,user_t=user,vel_t=vel,acc_t=acc,exaxis_pos_t=epos,ovl=ovl,offset_flag=flag,offset_pos=offset_pos,oacc=oacc,blendR=-1,velAccParamMode=velAccMode)
    print(f"circleerrcode:{rtn}")
    robot.CloseRPC()
    return 0

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

Examples of instruction code for spiral exploration, straight line insertion, etc
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:
    
    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1
    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)
    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)
    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)
    robot.FT_SetZero(0)
    time.sleep(1)
    status = 1
    sensor_num = 1
    gain = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0]
    adj_sign = 0
    ILC_sign = 0
    max_dis = 100.0
    max_ang = 5.0
    ft = [0.0,0.0,-10.0,0.0,0.0,0.0]
    rcs = 0
    dr = 0.7
    fFinish = 1.0
    t = 60000.0
    vmax = 3.0
    force_goal = 20.0
    lin_v = 0.0
    lin_a = 0.0
    disMax = 100.0
    linorn = 1
    angVelRot = 2.0
    forceInsertion = 1.0
    angleMax = 45
    orn = 1
    angAccmax = 0.0
    rotorn = 1
    select1 = [0, 0, 1, 1, 1, 0]
    robot.FT_Control(status, sensor_num, select1, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    rtn = robot.FT_SpiralSearch(rcs, dr, fFinish, t, vmax)
    print(f"FT_SpiralSearch rtn is {rtn}")
    robot.FT_Control(0, sensor_num, select1, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    select2 = [1, 1, 1, 0, 0, 0]
    gain[0] = 0.00005
    ft[2] = -30.0
    robot.FT_Control(1, sensor_num, select2, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    rtn = robot.FT_LinInsertion(rcs, force_goal, lin_v, lin_a, disMax, linorn)
    print(f"FT_LinInsertion rtn is {rtn}")
    robot.FT_Control(0, sensor_num, select2, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    select3 = [0, 0, 1, 1, 1, 0]
    ft[2] = -10.0
    gain[0] = 0.0001
    robot.FT_Control(1, sensor_num, select3, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    rtn = robot.FT_RotInsertion(rcs, angVelRot, forceInsertion, angleMax, orn, angAccmax, rotorn)
    print(f"FT_RotInsertion rtn is {rtn}")
    robot.FT_Control(0, sensor_num, select3, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    select4 = [1, 1, 1, 0, 0, 0]
    ft[2] = -30.0
    robot.FT_Control(1, sensor_num, select4, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    rtn = robot.FT_LinInsertion(rcs, force_goal, lin_v, lin_a, disMax, linorn)
    print(f"FT_LinInsertion rtn is {rtn}")
    robot.FT_Control(0, sensor_num, select4, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    robot.CloseRPC()

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

Sample code for surface localization
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1
    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)
    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)
    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)
    robot.FT_SetZero(0)
    time.sleep(1)
    rcs = 0
    dir = 1
    axis = 1
    lin_v = 3.0
    lin_a = 0.0
    maxdis = 50.0
    ft_goal = 2.0
    desc_pos = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    xcenter = [0, 0, 0, 0, 0, 0]
    ycenter = [0, 0, 0, 0, 0, 0]
    ft = [-2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot.MoveCart(desc_pos, 9, 0, 100.0)
    robot.FT_CalCenterStart()
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal)
    robot.MoveCart(desc_pos, 9, 0)
    robot.WaitMs(1000)
    dir = 2
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal)
    error,xcenter = robot.FT_CalCenterEnd()
    print(f"xcenter:{xcenter[0]},{xcenter[1]},{xcenter[2]},{xcenter[3]},{xcenter[4]},{xcenter[5]}")
    robot.MoveCart(xcenter, 9, 0, 60.0)
    robot.FT_CalCenterStart()
    dir = 1
    axis = 2
    lin_v = 6.0
    maxdis = 150.0
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal)
    robot.MoveCart(desc_pos, 9, 0, 100.0)
    robot.WaitMs(1000)
    dir = 2
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal)
    error,ycenter = robot.FT_CalCenterEnd()
    print(f"ycenter:{ycenter[0]},{ycenter[1]},{ycenter[2]},{ycenter[3]},{ycenter[4]},{ycenter[5]}")
    robot.MoveCart(ycenter, 9, 0, 60.0)
    robot.CloseRPC()

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

Soft control code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1
    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)
    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)
    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)
    robot.FT_SetZero(0)
    time.sleep(1)
    flag = 1
    sensor_id = 1
    select = [1, 1, 1, 0, 0, 0]
    ft_pid = [0.0005, 0.0, 0.0, 0.0, 0.0, 0.0]
    adj_sign = 0
    ILC_sign = 0
    max_dis = 100.0
    max_ang = 0.0
    ft = [-10.0, -10.0, -10.0, 0.0, 0.0, 0.0]
    offset_pos = [0.0,0.0,0.0,0.0,0.0,0.0]  # 替代 DescPose(0, 0, 0, 0, 0, 0)
    epos = [0.0,0.0,0.0,0.0]  # 替代 ExaxisPos(0, 0, 0, 0)
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]  # JointPos
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    desc_p1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]  # DescPose
    desc_p2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    p = 0.00005
    force = 30.0
    rtn = robot.FT_ComplianceStart(p, force)
    print(f"FT_ComplianceStart rtn is {rtn}")
    count = 3
    while count > 0:
        robot.MoveL(desc_pos=desc_p1,tool= 0,user= 0,vel= 100.0)
        robot.MoveL(desc_pos=desc_p2,tool= 0,user= 0,vel= 100.0)
        count -= 1
    robot.FT_ComplianceStop()
    print(f"FT_ComplianceStop rtn is {rtn}")
    flag = 0
    robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    robot.CloseRPC()

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

Robot load identification code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    retval = robot.LoadIdentifyDynFilterInit()
    print(f"LoadIdentifyDynFilterInit retval is: {retval}")
    retval = robot.LoadIdentifyDynVarInit()
    print(f"LoadIdentifyDynVarInit retval is: {retval}")
    error, posJ = robot.GetActualJointPosDegree(0)
    posJ[1] += 10  # Modify joint 2
    error, joint_toq = robot.GetJointTorques(0)
    joint_toq[1] += 2  # Modify torque on joint 2
    tmpTorque = joint_toq.copy()
    retval = robot.LoadIdentifyMain(tmpTorque, posJ, 1)
    print(f"LoadIdentifyMain retval is: {retval}")
    gain = [0, 0.05, 0, 0, 0, 0, 0, 0.02, 0, 0, 0, 0]
    weight = [0.0]
    load_pos = [0.0, 0.0, 0.0]
    retval, weight, load_pos = robot.LoadIdentifyGetResult(gain)
    print(f"LoadIdentifyGetResult retval is: {retval} ; weight is {weight}  cog is {load_pos[0]} {load_pos[1]} {load_pos[2]}")
    robot.CloseRPC()

Force Sensor Assisted Drag
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``EndForceDragControl(status, asaptiveFlag, interfereDragFlag, ingularityConstraintsFlag, M, B, K, F, Fmax, Vmax, forceCollisionFlag=0)``"
    "Description", "Force Sensor Assisted Drag"
    "Mandatory parameters", "- ``status``: control status, 0 - off; 1 - on
    - ``asaptiveFlag``: adaptive on flag, 0 - off; 1 - on
    - ``interfereDragFlag``: interference area drag flag, 0 - off; 1 - on
    - ``ingularityConstraintsFlag``: singularity strategy, 0- evade; 1- Crossing
    - ``forceCollisionFlag``: Robot collision detection mark during assisted dragging 0- Close 1- Enable
    - ``M=[m1,m2,m3,m4,m5,m6]``: inertia factor
    - ``B=[b1,b2,b3,b4,b5,b6]``: damping factor
    - ``K=[k1,k2,k3,k4,k5,k6]``: stiffness factor
    - ``F=[f1,f2,f3,f4,f5,f6]``: drag six-dimensional force thresholds
    - ``Fmax``: Maximum towing force limitation
    - ``Vmax``: maximum joint speed limit"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

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
    
Force sensor assisted drag code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.SetForceSensorDragAutoFlag(1)
    M = [15.0, 15.0, 15.0, 0.5, 0.5, 0.1]
    B = [150.0, 150.0, 150.0, 5.0, 5.0, 1.0]
    K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    F = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
    robot.EndForceDragControl(1, 0, 0, 0, M, B, K, F, 50, 100)
    time.sleep(5)
    drag_state = 0
    six_dimensional_drag_state = 0
    error,drag_state, six_dimensional_drag_state = robot.GetForceAndTorqueDragState()
    print(f"the drag state is {drag_state} {six_dimensional_drag_state}")
    robot.EndForceDragControl(0, 0, 0, 0, M, B, K, F, 50, 100)
    robot.CloseRPC()
    
Setting up hybrid drag switches and parameters for six-dimensional force and joint impedance
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.5

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ForceAndJointImpedanceStartStop(status, impedanceFlag, lamdeDain, KGain, BGain, dragMaxTcpVel, dragMaxTcpOriVel)``"
    "Description", "Setting up the six-dimensional force and joint impedance hybrid drag switch and parameters"
    "Mandatory parameters", "- ``status``: control status, 0 - off; 1 - on
    - ``impedanceFlag``: impedance on flag, 0 - off; 1 - on
    - ``lamdeDain``: [D1, D2, D3, D4, D5, D6] drag gain
    - ``KGain``: [K1,K2,K3,K4,K5,K6] Stiffness Gain
    - ``BGain``: [B1,B2,B3,B4,B5,B] damping gain
    - ``dragMaxTcpVel``: maximum line speed limit at the end of a drag
    - ``dragMaxTcpOriVel``: drag end maximum angular velocity limit"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"
    
Six dimensional force and joint impedance mixed drag code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.DragTeachSwitch(1)
    lamde_dain = [3.0, 2.0, 2.0, 2.0, 2.0, 3.0]
    k_gain = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    b_gain = [150.0, 150.0, 150.0, 5.0, 5.0, 1.0]
    rtn = robot.ForceAndJointImpedanceStartStop(1, 0, lamde_dain, k_gain, b_gain, 1000, 180)
    print(f"ForceAndJointImpedanceStartStop rtn is {rtn}")
    time.sleep(5)
    robot.DragTeachSwitch(0)
    rtn = robot.ForceAndJointImpedanceStartStop(0, 0, lamde_dain, k_gain, b_gain, 1000, 180)
    print(f"ForceAndJointImpedanceStartStop rtn is {rtn}")
    robot.CloseRPC()

Impedance start and stop control
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.6

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ImpedanceControlStartStop(status, workSpace, forceThreshold, m, b, k, maxV, maxVA, maxW, maxWA)``"
    "Description", "Impedance start and stop control"
    "Mandatory parameters", "- ``status``：0- closed; 1- On
    - ``workSpace``：0-joint space; 1-Dekal space
    - ``forceThreshold``：Trigger force threshold (N)
    - ``m``：Quality parameter
    - ``b``：Parameter of damping
    - ``k``：Parameter of stiffness
    - ``maxV``：Maximum linear velocity (mm/s)
    - ``maxVA``：Maximum linear acceleration (mm/s2)
    - ``maxW``：Maximum angular velocity (°/s)
    - ``maxWA``：Maximum angular acceleration (°/s2)"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Example code for impedance start and stop control
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    j1 = [102.622, -135.990, 120.769, -73.950, -90.848, 35.507]
    j2 = [93.674, -80.062, 82.947, -92.199, -90.967, 26.559]
    desc_pos1 = [136.552, -149.799, 449.532, 179.817, -1.172, 157.123]
    desc_pos2 = [136.540, -561.048, 449.542, 179.819, -1.172, 157.122]
    offset_pos = [0.0] * 6
    epos = [0.0] * 4
    tool = 0
    user = 0
    vel = 100.0
    acc = 200.0
    ovl = 100.0
    blendT = -1.0
    blendR = -1.0
    flag = 0
    search = 0
    robot.SetSpeed(20)
    company = 17
    device = 0
    softversion = 0
    bus = 1
    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)
    rnt,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)
    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)
    time.sleep(1)
    robot.FT_SetZero(0)
    time.sleep(1)
    robot.FT_SetZero(1)
    time.sleep(1)
    forceThreshold = [30.0, 30.0, 30.0, 5.0, 5.0, 5.0]
    m = [0.1, 0.1, 0.1, 0.02, 0.02, 0.02]
    b = [1.0, 1.0, 1.0, 0.08, 0.08, 0.08]
    k = [0.0] * 6
    rtn = robot.ImpedanceControlStartStop(1, 1, forceThreshold, m, b, k, 1000, 500, 100, 100)
    print(f"ImpedanceControlStartStop errcode:{rtn}")
    rtn = robot.MoveL(desc_pos=desc_pos1,tool= tool,user= user,vel= vel,speedPercent=1)
    rtn = robot.MoveL(desc_pos=desc_pos2,tool= tool,user= user,vel= vel,speedPercent=1)
    rtn = robot.MoveL(desc_pos=desc_pos1,tool= tool,user= user,vel= vel,speedPercent=1)
    rtn = robot.MoveL(desc_pos=desc_pos2,tool= tool,user= user,vel= vel,speedPercent=1)
    rtn = robot.MoveL(desc_pos=desc_pos1,tool= tool,user= user,vel= vel,speedPercent=1)
    rtn = robot.MoveL(desc_pos=desc_pos2,tool= tool,user= user,vel= vel,speedPercent=1)
    print(f"movel errcode:{rtn}")
    robot.ImpedanceControlStartStop(0, 1, forceThreshold, m, b, k, 1000, 500, 100, 100)
    robot.CloseRPC()

Enable Torque Compensation Function and Compensation Coefficients
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SerCoderCompenParams(status, torqueCoeff)``"
    "Description", "Enable Torque Compensation Function and Compensation Coefficients"
    "Required Parameters", "- ``status``：Switch, 0-Disable; 1-Enable
    - ``torqueCoeff``：J1-J6 Torque compensation coefficients [0-1]"
    "Default Parameters", "None"
    "Return Value", "Error code 0-Success  Error- errcode"