Force control
======================

.. toctree:: 
    :maxdepth: 5

Obtain force sensor configuration
++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_GetConfig()``"
    "Description", "Obtain force sensor configuration"
    "Required parameter", "Nothing"
    "Optional Parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
        - Return(if success): [number,company,device,softversion]
        - number: sensor number, range[1]
        - company: Sensor manufacturer,17-Kunwei Technology,19-Aerospace 11th Institute, 20-ATI sensors, 21-Zhongke Mi Dian, 22-Weihang Sensitive Core
        - device:equipment number: Kunwei (0-KWR75B), Aerospace 11th Institute (0-MCS6A-200-4), ATI (0-AXIA80-M8), Zhongkomi Point (0-MST2010), Weihang Minxin (0-WHC6L-YB-10A)
        - softversion:Software version number, temporarily not used, defaults to 0"

Force sensor configuration
++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_SetConfig(company,device,softversion=0,bus=0)``"
    "Description", "Force sensor configuration"
    "Required parameter", "- ``company``:Sensor manufacturer,17-Kunwei Technology,19-Aerospace 11th Institute,20-ATI sensors, 21-Zhongke Mi Dian, 22-Weihang Sensitive Core;
    - ``device``:equipment number: Kunwei (0-KWR75B), Aerospace 11th Institute (0-MCS6A-200-4), ATI (0-AXIA80-M8), Zhongkomi Point (0-MST2010), Weihang Minxin (0-WHC6L-YB-10A);
    - ``softversion``:software version number, temporarily not used, defaults to 0;;
    - ``bus``:device mounted terminal bus position, temporarily not used, defaults to 0;"
    "Optional parameter",	"- softversion:software version number, temporarily not used, defaults to 0;
        - bus:device mounted terminal bus position, temporarily not used, defaults to 0;"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    company = 17    
    device = 0     
    error = robot.FT_SetConfig(company, device)   # Force sensor configuration
    print("Force sensor configuration",error)
    config = robot.FT_GetConfig() # Obtain force sensor configuration
    print(' Obtain force sensor configuration ',config)

Force sensor activation
+++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_Activate(state)``"
    "Description", "Force sensor activation"
    "Required parameter", "- ``state``:0-Reset,1-Activate"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.FT_Activate(0)  # Reset force sensor
    print("Reset force sensor ",error)
    time.sleep(1)
    error = robot.FT_Activate(1)  # Activate force sensor 
    print("Activate force sensor ",error)

Zero calibration of force sensor
++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_SetZero(state)``"
    "Description", "Zero calibration of force sensor"
    "Required parameter", "- ``state``:0-Remove zero,1-Zero correction"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.FT_SetZero(0)   # Sensor zero removal
    print("Sensor zero removal ",error)
    error = robot.FT_SetZero(1)   # The zero point of the sensor should be corrected. Please note that no tool can be installed at the end of the sensor.
    print("Sensor zero corrected ",error)

Set the force sensor reference coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_SetRCS(ref)``"
    "Description", "Set the force sensor reference coordinate system"
    "Required parameter", "- ``ref``:0-Tool coordinate system,1-Base coordinate system"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.FT_SetRCS(0)    # Set the force sensor reference coordinate system
    print(' Set the force sensor reference coordinate system ',error)
    time.sleep(1)

Load weight identification calculation
++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_PdIdenCompute()``"
    "Description", "Load weight identification calculation"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success):weight Load weight,unit[kg] "

Load weight identification record
+++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_PdIdenRecord(tool_id)``"
    "Description", "Load weight identification record"
    "Required parameter", "- ``tool_id``:Sensor coordinate number,range[0~14]"
    Return(if success):weight Load weight,unit[kg] 
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    tool_id = 10  
    error = robot.FT_PdIdenRecord(tool_id)   # Load weight identification record
    print('Load weight identification record ',error)
    time.sleep(1)
    error = robot.FT_PdIdenRecord()  # Load weight identification calculation
    print('Load weight identification calculation ',error)

Load centroid identification calculation
++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_PdCogIdenCompute()``"
    "Description", "Load centroid identification calculation"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success): cog=[cogx,cogy,cogz],Load centroid ,unit[mm] "

Load centroid identification record
++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_PdCogIdenRecord(tool_id)``"
    "Description", "Load centroid identification record"
    "Required parameter", "- ``tool_id``:Sensor coordinate number,range[0~14]"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:
    
    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    #For load centroid identification, the robot needs to teach three different poses, then record the identification data, and finally calculate the load centroid
    robot.Mode(1)
    ret = robot.DragTeachSwitch(1)  
    time.sleep(5)
    ret = robot.DragTeachSwitch(0)
    time.sleep(1)
    error = robot.FT_PdCogIdenRecord(tool_id,1)
    print(' Load centroid identification record ',error) 
    ret = robot.DragTeachSwitch(1)   
    time.sleep(5)
    ret = robot.DragTeachSwitch(0)
    time.sleep(1)
    error = robot.FT_PdCogIdenRecord(tool_id,2)
    print('Load centroid identification record ',error)
    ret = robot.DragTeachSwitch(1)  
    time.sleep(5)
    ret = robot.DragTeachSwitch(0)
    time.sleep(1)
    error = robot.FT_PdCogIdenRecord(tool_id,3)
    print(' Load centroid identification record ',error)
    time.sleep(1)
    error = robot.FT_PdCogIdenCompute()
    print('Load centroid identification calculation ',error)

Obtain force/torque data in the reference coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_GetForceTorqueRCS()``"
    "Description", "Obtain force/torque data in the reference coordinate system"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success):data=[fx,fy,fz,mx,my,mz] "

Code example
----------------
.. code-block:: python
    :linenos:

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    rcs = robot.FT_GetForceTorqueRCS()  #Query data in the sensor coordinate system
    print(rcs)

Obtain raw force/torque data from the force sensor
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_GetForceTorqueOrigin()``"
    "Description", "Obtain raw force/torque data from the force sensor"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0 , Failed -errcode
    - Return(if success):data=[fx,fy,fz,mx,my,mz] "

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.FT_GetForceTorqueOrigin()   # Obtain raw force/torque data from the force sensor
    print("Obtain raw force/torque data from the force sensor ",error)

Collision protection
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_Guard(flag,sensor_num,select,force_torque,max_threshold,min_threshold)``"
    "Description", "Collision protection"
    "Required parameter", "- ``flag``:0-Turn off collision protection, 1-Turn on collision protection;
    - ``sensor_num``:Force sensor number;
    - ``select``:Whether the six degrees of freedom detect the collision[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective;
    - ``force_torque``:Collision detection force/moment,unit[N or Nm];
    - ``max_threshold``:Maximum threshold;
    - ``min_threshold``:Minimum Threshold;"
    "Optional parameter", "Nothing"
    - Force/torque detection range:(force_torque-min_threshold,force_torque+max_threshold)"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    actFlag = 1   #Enable flag, 0-Disable collision guard, 1-Enable collision guard
    sensor_num = 1  #Force sensor number
    is_select = [1,1,1,1,1,1]  #Whether the six degrees of freedom detect the collision[fx,fy,fz,mx,my,mz],0-Ineffective, 1-Effective
    force_torque = [0.0,0.0,0.0,0.0,0.0,0.0]  #Collision detection force/moment,detection range（force_torque-min_threshold,force_torque+max_threshold）
    max_threshold = [10.0,10.0,10.0,10.0,10.0,10.0]  #Maximum threshold
    min_threshold = [5.0,5.0,5.0,5.0,5.0,5.0]   #Minimum Threshold
    P1=[-160.619,-586.138,384.988,-170.166,-44.782,169.295]
    P2=[-87.615,-606.209,556.119,-102.495,10.118,178.985]
    P3=[41.479,-557.243,484.407,-125.174,46.995,-132.165]
    error = robot.FT_Guard(actFlag, sensor_num, is_select, force_torque, max_threshold, min_threshold)    # Turn on collision protection
    print("Turn on collision protection ",error)
    error = robot.MoveL(P1,1,0)         # linear motion in Cartesian space
    print("Linear motion in Cartesian space ",error)
    error = robot.MoveL(P2,1,0)
    print("Linear motion in Cartesian space ",error)
    error = robot.MoveL(P3,1,0)
    print("Linear motion in Cartesian space ",error)
    actFlag = 0
    error = robot.FT_Guard(actFlag, sensor_num, is_select, force_torque, max_threshold, min_threshold)    # Turn off collision protection
    print("Turn off collision protection ",error)

Constant force control
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_Control(flag,sensor_num,select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)``"
    "Description", "Constant force control"
    "Required parameter", "- ``flag``:Constant force control open flag, 0-off, 1-on;
    - ``sensor_num``:Force sensor number;
    - ``select``:Are the six degrees of freedom detected [fx,fy,fz,mx,my,mz],0-ineffective, 1-effective;
    - ``force_torque``:Detection force/torque, unit[N or Nm];
    - ``gain``:[f_p,f_i,f_d,m_p,m_i,m_d],Force PID parameters, Torque PID parameters;
    - ``adj_sign``:Adaptive start stop status, 0-off, 1-on;
    - ``ILC_sign``: ILC control start stop status, 0-stop, 1-training, 2-practical operation;
    - ``max_dis``:Maximum adjustment distance;
    - ``max_ang``:Maximum adjustment angle;"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #Constant force control
    status = 1  #Constant force control open flag, 0-off, 1-on
    sensor_num = 1 #Force sensor number
    is_select = [0,0,1,0,0,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0005,0.0,0.0,0.0,0.0,0.0]  #Maximum threshold
    adj_sign = 0  #Adaptive start stop status, 0-off, 1-on
    ILC_sign = 0  #ILC control start stop status, 0-stop, 1-training, 2-practical operation
    max_dis = 100.0  #Maximum adjustment distance
    max_ang = 0.0  #Maximum adjustment angle
    J1=[70.395, -46.976, 90.712, -133.442, -87.076, -27.138]
    P2=[-123.978, -674.129, 44.308, -178.921, 2.734, -172.449]
    P3=[123.978, -674.129, 42.308, -178.921, 2.734, -172.449]
    error = robot.MoveJ(J1,1,0)    
    print("Joint space motion PTP ",error)
    error = robot.MoveL(P2,1,0)
    print("Linear motion in Cartesian space ",error)
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Turn on constant force control ",error)
    error = robot.MoveL(P3,1,0)    
    print("Linear motion in Cartesian space ",error)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Turn off constant force control ",error)

Spiral line exploration
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_SpiralSearch(rcs,ft, dr = 0.7,max_t_ms = 60000, max_vel = 5)``"
    "Description", "Spiral line exploration"
    "Required parameter", "- ``rcs``:Reference coordinate system, 0-tool coordinate system, 1-base coordinate system
    - ``ft``:Force or torque threshold (0-100), unit[N/Nm];"
    "Optional parameter", "- ``dr``:Feed rate per circle radius, unit[mm], default to 0.7;
    - ``max_t_ms``:Maximum exploration time,unit[ms] , default to 6000;
    - ``max_vel``:Maximum linear speed,unit[mm/s] , default to 5"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    P = [36.794,-675.119, 65.379, -176.938, 2.535, -179.829]
    #Constant force parameter
    status = 1  #Constant force control open flag, 0-off, 1-on
    sensor_num = 1 #Force sensor number
    is_select = [0,0,1,0,0,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #Maximum threshold
    adj_sign = 0  #Adaptive start stop status, 0-off, 1-on
    ILC_sign = 0  #ILC control start stop status, 0-stop, 1-training, 2-practical operation
    max_dis = 100.0  #Maximum adjustment distance
    max_ang = 5.0  #Maximum adjustment angle
    #Helix explore parameters
    rcs = 0  #Reference frame, 0-Tool frame, 1-Base frame
    fFinish = 1.0 #Force or moment threshold（0~100）,unit[N or Nm]

    error = robot.MoveL(P,1,0)
    print("Linear motion in Cartesian space ",error)
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign, max_dis,max_ang)
    print("Turn on constant force control ",error)
    error = robot.FT_SpiralSearch(rcs,fFinish,max_vel=3)
    print("Spiral line exploration ",error)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign, max_dis,max_ang)
    print("Turn off constant force control ",error)

Rotate Insert
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_RotInsertion(rcs,ft, orn, angVelRot = 3, angleMax = 45, angAccmax = 0, rotorn =1)``"
    "Description", "Rotate Insert"
    "Required parameter", "- ``rcs``:Reference coordinate system, 0-tool coordinate system, 1-base coordinate system;
    - ``ft``:Force or torque threshold (0-100), unit[N/Nm];
    - ``orn``:Direction of force, 1-fz,2-mz;"
    "Optional parameter", "- ``angVelRot``:Rotational angular velocity: uni[t°/s],default to 3;
    - ``angleMax``:maximum rotation angle, unit[°] ,default to 45;
    - ``angAccmax``:Maximum rotational acceleration, unit[°/s^2],not used temporarily, default to 0;
    - ``rotorn``:Rotation direction, 1-clockwise, 2-counterclockwise, default to 1"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    P = [36.794,-675.119, 65.379, -176.938, 2.535, -179.829]
    #Constant force parameter
    status = 1  #Constant force control open flag, 0-off, 1-on
    sensor_num = 1 #Force sensor number
    is_select = [0,0,1,0,0,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #Maximum threshold
    adj_sign = 0  #Adaptive start stop status, 0-off, 1-on
    ILC_sign = 0  #ILC control start stop status, 0-stop, 1-training, 2-practical operation
    max_dis = 100.0  #Maximum adjustment distance
    max_ang = 5.0  #Maximum adjustment angle
    #Rotational insertion parameter
    rcs = 0  #Reference frame, 0-Tool frame, 1-Base frame
    angVelRot = 2.0  #Rotational angular velocity,unit[°/s]
    forceInsertion = 1.0 #Force or moment threshold（0~100）,unit[N or Nm]
    angleMax= 45 #Maximum rotation Angle,unit[°]
    orn = 1 #Direction of force,1-fz,2-mz
    angAccmax = 0.0 #Maximum rotational acceleration, unit[°/s^2],not used temporarily
    rotorn = 1 #Rotation direction, 1-clockwise, 2-counterclockwise
    error = robot.MoveL(P,1,0) # Linear motion in Cartesian space
    print("Linear motion in Cartesian space ",error)
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Turn on constant force control ",error)
    error = robot.FT_RotInsertion(rcs,1,orn)
    print("Rotate Insert ",error)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Turn off constant force control ",error)

Linear insertion
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_LinInsertion(rcs, ft, disMax, linorn, lin_v = 1.0, lin_a = 1.0)``"
    "Description", "Linear insertion"
    "Required parameter", "- ``rcs``:Reference frame, 0-Tool frame, 1-Base frame;
    - ``ft``:Force or torque threshold (0-100), unit[N/Nm];
    - ``disMax``:Maximum insertion distance,unit[mm];
    - ``linorn``:Insertion direction, 1-positive direction, 2-negative direction;"
    "Optional parameter", "- ``lin_v``:Linear velocity, unit[mm/s] , default to 1.0;
    - ``lin_a``:Linear acceleration, unit[mm/s^2],not used temporarily;rotorn:Rotation direction, 1-clockwise, 2-counterclockwise, default to 1.0"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:
    
    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    P = [36.794,-675.119, 65.379, -176.938, 2.535, -179.829]
    #Constant force parameter
    status = 1  #Constant force control open flag, 0-off, 1-on
    sensor_num = 1 #Force sensor number
    is_select = [0,0,1,0,0,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #Maximum threshold
    adj_sign = 0  #Adaptive start stop status, 0-off, 1-on
    ILC_sign = 0  #ILC control start stop status, 0-stop, 1-training, 2-practical operation
    max_dis = 100.0  #Maximum adjustment distance
    max_ang = 5.0  #Maximum adjustment angle
    #Linear insertion parameter
    rcs = 0  #Reference frame, 0-Tool frame, 1-Base frame
    force_goal = 20.0  #Force or moment threshold（0~100）,unit[N or Nm]
    lin_v = 0.0 #Linear velocity,unit[mm/s]
    lin_a = 0.0 #Linear acceleration, unit[mm/s^2],not used temporarily
    disMax = 100.0 #Maximum insertion distance,unit[mm]
    linorn = 1 #Insertion direction, 1-positive direction, 2-negative direction
    error = robot.MoveL(P,1,0) # Linear motion in Cartesian space
    print("Linear motion in Cartesian space ",error)
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Turn on constant force control ",error)
    error = robot.FT_LinInsertion(rcs,force_goal,disMax,linorn)
    print("Linear insertion",error)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Turn off constant force control ",error)

Calculate the middle plane position to start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_CalCenterStart()``"
    "Description", "Calculate the middle plane position to start"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Calculate the middle plane position to end
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_CalCenterEnd()``"
    "Description", "Calculate the middle plane position to end"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode
    - Return(if success):pos=[x,y,z,rx,ry,rz] "

Surface positioning
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_FindSurface (rcs, dir, axis, disMax, ft, lin_v = 3.0, lin_a = 0.0)``"
    "Description", "Surface positioning"
    "Required parameter", "- ``rcs``: Reference frame, 0-Tool frame, 1-Base frame;
    - ``dir``:Direction of movement, 1-positive, 2-negative;
    - ``axis``:Move Axis,1-x,2-y,3-z;
    - ``disMax``:Maximum exploration distance,unit[mm]"
    "Optional parameter", "- ``lin_v``:Exploring Linear Speed,unit[mm/s] , default to 3.0;
    - ``lin_a``:Exploring Linear Acceleration,unit[mm/s^2] , default to 0.0"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    #Constant force parameter
    status = 1  #Constant force control open flag, 0-off, 1-on
    sensor_num = 1 #Force sensor number
    is_select = [1,0,0,0,0,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
    force_torque = [-2.0,0.0,0.0,0.0,0.0,0.0]  #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0002,0.0,0.0,0.0,0.0,0.0]  #Maximum threshold
    adj_sign = 0  #Adaptive start stop status, 0-off, 1-on
    ILC_sign = 0  #ILC control start stop status, 0-stop, 1-training, 2-practical operation
    max_dis = 100.0  #Maximum adjustment distance
    max_ang = 5.0  #Maximum adjustment angle
    #Surface positioning parameter
    rcs = 0 #Reference frame, 0-Tool frame, 1-Base frame
    direction = 1 #Direction of movement,1-positive direction, 2-negative direction
    axis = 1 #Axis of movement,1-X,2-Y,3-Z
    lin_v = 3.0  #Exploring straight-line velocity,unit[mm/s]
    lin_a = 0.0  #Exploration linear acceleration,unit[mm/s^2]
    disMax = 50.0 #Maximum exploration distance,unit[mm]
    force_goal = 2.0 #Action termination force threshold,unit[N]

    P1=[-77.24,-679.599,58.328,179.373,-0.028,-167.849]
    Robot.MoveCart(P1,1,0)       #Point to point motion in joint space
    #Look for the center in the x direction
    #The first surface
    error = robot.FT_CalCenterStart()
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    error = robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    time.sleep(2)
    error = robot.MoveCart(P1,1,0)       #Point to point motion in joint space
    time.sleep(5)
    #The second surface
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    direction = 2 #Direction of movement,1-positive direction, 2-negative direction
    error = robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    #Calculate the x-direction center position
    error,xcenter = robot.FT_CalCenterEnd()
    print("xcenter",xcenter)
    error = robot.MoveCart(xcenter,1,0)
    time.sleep(1)
    #Look for the center in the y direction
    #The first surface
    error =robot.FT_CalCenterStart()
    error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    direction = 1 #Direction of movement,1-positive direction, 2-negative direction
    axis = 2 #Axis of movement,1-X,2-Y,3-Z
    disMax = 150.0 #Maximum exploration distance,unit[mm]
    lin_v = 6.0  #Exploring straight-line velocity,unit[mm/s]
    error =robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
    status = 0
    error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    error =robot.MoveCart(P1,1,0)       #Point to point motion in joint space
    Robot.WaitMs(1000)
    #The second surface
    error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    direction = 2 #Direction of movement,1-positive direction, 2-negative direction
    error =robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
    status = 0
    error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    #Calculate the y center position
    error,ycenter=robot.FT_CalCenterEnd()
    print("y center position",ycenter)
    error =robot.MoveCart(ycenter,1,0)

Flexibility control off
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_ComplianceStop()``"
    "Description", "Flexibility control off"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Flexibility control on
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_ComplianceStart(p,force)``"
    "Description", "Flexibility control on"
    "Required parameter", "- ``p``: Position adjustment coefficient or compliance coefficient
    - ``force``:flexibility opening force threshold, unit[N]"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    J1=[75.005,-46.434,90.687,-133.708,-90.315,-27.139]
    P2=[-77.24,-679.599,38.328,179.373,-0.028,-167.849]
    P3=[77.24,-679.599,38.328,179.373,-0.028,-167.849]
    #Constant force parameter
    status = 1  #Constant force control open flag, 0-off, 1-on
    sensor_num = 1 #Force sensor number
    is_select = [1,1,1,0,0,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
    force_torque = [-10.0,-10.0,-10.0,0.0,0.0,0.0] #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0005,0.0,0.0,0.0,0.0,0.0]  # 
    adj_sign = 0  #Adaptive start stop status, 0-off, 1-on
    ILC_sign = 0  #ILC control start stop status, 0-stop, 1-training, 2-practical operation
    max_dis = 1000.0  #Maximum adjustment distanc
    max_ang = 0.0  #Maximum adjustment angle
    error = robot.MoveJ(J1,1,0)
    #Compliance control
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Turn on constant force control ",error)
    p = 0.00005  # Coefficient of position adjustment or compliance
    force = 30.0 # Compliant opening force threshold,unit[N]
    error = robot.FT_ComplianceStart(p,force)
    print("Turn on flexibility control ",error)
    error = robot.MoveL(P2,1,0,vel =10)   # Rectilinear motion in Cartesian space
    print("Rectilinear motion in Cartesian space ", error)
    error = robot.MoveL(P3,1,0,vel =10)
    print("Rectilinear motion in Cartesian space ", error)
    time.sleep(1)
    error = robot.FT_ComplianceStop()
    print("Turn off flexibility control ",error)
    status = 0
    error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
    print("Turn off constant force control ",error)

Load identification initialization
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "LoadIdentifyDynFilterInit()"
    "Description", "Load identification initialization"
    "Required parameter", "Null"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')

    # Load identification initialization
    error = robot.LoadIdentifyDynFilterInit()
    print("LoadIdentifyDynFilterInit:",error)
    # Load identification variable initialization
    error = robot.LoadIdentifyDynVarInit()
    print("LoadIdentifyDynVarInit:",error)

    joint_torque= [0,0,0,0,0,0]
    joint_pos= [0,0,0,0,0,0]
    gain=[0,0.05,0,0,0,0,0,0.02,0,0,0,0]
    t =10
    error,joint_pos=robot.GetActualJointPosDegree(1)
    joint_pos[1] = joint_pos[1]+10
    error,joint_torque=robot.GetJointTorques(1)
    joint_torque[1] = joint_torque[1]+2
    # Load identification main program
    error = robot.LoadIdentifyMain(joint_torque, joint_pos, t)
    print("LoadIdentifyMain:",error)
    #Obtain the load identification result
    error = robot.LoadIdentifyGetResult(gain)
    print("LoadIdentifyGetResult:",error)

Load identification variable initialization
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "LoadIdentifyDynVarInit()"
    "Description", "Load identification variable initialization"
    "Required parameter", "Null"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Load identification main program
++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "LoadIdentifyMain(joint_torque, joint_pos, t)"
    "Description", "Load identification main program"
    "Required parameter", "- ``joint_torque``: j1-j6；
    - ``joint_pos``:j1-j6"
    "Return value", "Errcode: Success -0 , Failed -errcode"

Obtain the load identification result
+++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.1

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "LoadIdentifyGetResult(gain)"
    "Description", "Obtain the load identification result"
    "Required parameter", "- ``gain``"
    "Optional parameter", "Nothing"
    "Return value", "- Errcode: Success -0  Failed -errcod
    - Return:(if success) weight Load weight，cog Load centroid [x,y,z]"