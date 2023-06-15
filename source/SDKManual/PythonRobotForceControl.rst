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
    "Parameter", "Nothing"
    "Return value", "- Success:[0, company,device,softversion,bus],company:Sensor manufacturer
    - Failed:[errcode]"

Force sensor configuration
++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_SetConfig(company,device,softversion,bus)``"
    "Description", "Force sensor configuration"
    "Parameter", "- ``company``:Sensor manufacturer,17-Kunwei Technology,19-Aerospace 11th Institute,20-ATI sensors, 21-Zhongke Mi Dian, 22-Weihang Sensitive Core;
    - ``device``:equipment number: Kunwei (0-KWR75B), Aerospace 11th Institute (0-MCS6A-200-4), ATI (0-AXIA80-M8), Zhongkomi Point (0-MST2010), Weihang Minxin (0-WHC6L-YB-10A);
    - ``softversion``:software version number, temporarily not used, defaults to 0;;
    - ``bus``:device mounted terminal bus position, temporarily not used, defaults to 0;"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 8, 9

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    company = 17    #Sensor manufacturer,17-Kunwei Technology,
    device = 0      #Sensor equipment number
    softversion = 0 #software version number
    bus = 1         #End bus position
    robot.FT_SetConfig(company, device, softversion, bus)   #Configured force sensor
    config = robot.FT_GetConfig() #Obtain the configuration information of the force sensor. The manufacturer number is one larger than the feedback
    print(config)

Force sensor activation
+++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_Activate(state)``"
    "Description", "Force sensor activation"
    "Parameter", "- ``state``:0-Reset,1-Activate"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4,6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.FT_Activate(0)  #Sensor reset
    time.sleep(1)
    robot.FT_Activate(1)  #Sensor activation
    time.sleep(1)

Zero calibration of force sensor
++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_SetZero(state)``"
    "Description", "Zero calibration of force sensor"
    "Parameter", "- ``state``:0-Remove zero,1-Zero correction"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4,6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.FT_SetZero(0)   #Sensor zero removal
    time.sleep(1)
    robot.FT_SetZero(1)   #The zero point of the sensor should be corrected. Please note that no tool can be installed at the end of the sensor.
    time.sleep(1)

Set the force sensor reference coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_SetRCS(ref)``"
    "Description", "Set the force sensor reference coordinate system"
    "Parameter", "- ``ref``:0-Tool coordinate system,1-Base coordinate system"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.FT_SetRCS(0)    #Set reference coordinate system to tool coordinate system, 0- tool coordinate system, 1- base coordinate system
    time.sleep(1)

Load weight identification calculation
++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_PdIdenCompute()``"
    "Description", "Load weight identification calculation"
    "Parameter", "Nothing"
    "Return value", "- Success:[0,weight] ,weight-Load weight,unit[kg]
    - Failed:[errcode]"

Load weight identification record
+++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_PdIdenRecord(tool_id)``"
    "Description", "Load weight identification record"
    "Parameter", "- ``tool_id``:Sensor coordinate number,range[0~14]"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 13, 15

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    #Load identification. At this time, the tool to be identified is installed at the end. The tool is installed under the force sensor, and the end is vertical down
    robot.FT_SetRCS(0)    #Set reference coordinate system to tool coordinate system, 0- tool coordinate system, 1- base coordinate system
    time.sleep(1)
    tool_id = 10  #Sensor coordinate number
    tool_coord = [0.0,0.0,35.0,0.0,0.0,0.0]   # Position of sensor relative to end flange
    tool_type = 1  # 0-Tool, 1-Sensor
    tool_install = 0 # 0-Mount end, 1-Outside of robot
    robot.SetToolCoord(tool_id,tool_coord,tool_type,tool_install)     #Set sensor coordinate system, sensor relative end flange position
    time.sleep(1)
    robot.FT_PdIdenRecord(tool_id)   #Record identification data
    time.sleep(1)
    weight = robot.FT_PdIdenCompute()  #Calculated load weight,unit[kg]
    print(weight)


Load centroid identification calculation
++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_PdCogIdenCompute()``"
    "Description", "Load centroid identification calculation"
    "Parameter", "Nothing"
    "Return value", "- Success:[0,cog],cog=[cogx,cogy,cogz] ,Load centroid,unit[mm]
    - Failed:[errcode]"

Load centroid identification record
++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_PdCogIdenRecord(tool_id)``"
    "Description", "Load centroid identification record"
    "Parameter", "- ``tool_id``:Sensor coordinate number,range[0~14]"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 9,14,19,21
    
    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    #For load centroid identification, the robot needs to teach three different poses, then record the identification data, and finally calculate the load centroid
    P1=[-160.619,-586.138,384.988,-170.166,-44.782,169.295]
    robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)         #Point to point motion in joint space
    time.sleep(1)
    robot.FT_PdCogIdenRecord(tool_id,1)                               #Record identification data
    time.sleep(1)
    P2=[-87.615,-606.209,556.119,-102.495,10.118,178.985]
    robot.MoveCart(P2,9,0,100.0,100.0,100.0,-1.0,-1)
    time.sleep(1)      
    robot.FT_PdCogIdenRecord(tool_id,2)
    time.sleep(1)
    P3=[41.479,-557.243,484.407,-125.174,46.995,-132.165]
    robot.MoveCart(P3,9,0,100.0,100.0,100.0,-1.0,-1)
    time.sleep(1)
    robot.FT_PdCogIdenRecord(tool_id,3)
    time.sleep(1)
    cog = robot.FT_PdCogIdenCompute()   # Calculated and identified load centroid
    print(cog)

Obtain force/torque data in the reference coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_GetForceTorqueRCS()``"
    "Description", "Obtain force/torque data in the reference coordinate system"
    "Parameter", "Nothing"
    "Return value", "- Success:[0,data] ,data=[fx,fy,fz,mx,my,mz]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

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
    "Parameter", "Nothing"
    "Return value", "- Success:[0,data] ,data=[fx,fy,fz,mx,my,mz]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    origin = robot.FT_GetForceTorqueOrigin()   #Example Query the original sensor data
    print(origin)

Collision protection
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_Guard(flag,sensor_num,select,force_torque,max_threshold,min_threshold)``"
    "Description", "Collision protection"
    "Parameter", "- ``flag``:0-Turn off collision protection, 1-Turn on collision protection;
    - ``sensor_num``:Force sensor number;
    - ``select``:Whether the six degrees of freedom detect the collision[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective;
    - ``force_torque``:Collision detection force/moment,unit[N or Nm];
    - ``max_threshold``:Maximum threshold;
    - ``min_threshold``:Minimum Threshold;
    - Force/torque detection range:(force_torque-min_threshold,force_torque+max_threshold)"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    actFlag = 1   #Enable flag, 0-Disable collision guard, 1-Enable collision guard
    sensor_num = 1  #Force sensor number
    is_select = [1,1,1,1,1,1]  #Whether the six degrees of freedom detect the collision[fx,fy,fz,mx,my,mz],0-Ineffective, 1-Effective
    force_torque = [0.0,0.0,0.0,0.0,0.0,0.0]  #Collision detection force/moment,detection range（force_torque-min_threshold,force_torque+max_threshold）
    max_threshold = [10.0,10.0,10.0,10.0,10.0,10.0]  #Maximum threshold
    min_threshold = [5.0,5.0,5.0,5.0,5.0,5.0]   #Minimum Threshold
    P1=[-160.619,-586.138,384.988,-170.166,-44.782,169.295]
    P2=[-87.615,-606.209,556.119,-102.495,10.118,178.985]
    P3=[41.479,-557.243,484.407,-125.174,46.995,-132.165]
    robot.FT_Guard(actFlag, sensor_num, is_select, force_torque, max_threshold, min_threshold)    #Enable collision guard
    robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)         #Point to point motion in joint space
    robot.MoveCart(P2,9,0,100.0,100.0,100.0,-1.0,-1)
    robot.MoveCart(P3,9,0,100.0,100.0,100.0,-1.0,-1)
    actFlag = 0  
    robot.FT_Guard(actFlag, sensor_num, is_select, force_torque, max_threshold, min_threshold)    #Disable collision guard

Constant force control
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_Control(flag,sensor_num,select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)``"
    "Description", "Constant force control"
    "Parameter", "- ``flag``:Constant force control open flag, 0-off, 1-on;
    - ``sensor_num``:Force sensor number;
    - ``select``:Are the six degrees of freedom detected [fx,fy,fz,mx,my,mz],0-ineffective, 1-effective;
    - ``force_torque``:Detection force/torque, unit[N or Nm];
    - ``gain``:[f_p,f_i,f_d,m_p,m_i,m_d],Force PID parameters, Torque PID parameters;
    - ``adj_sign``:Adaptive start stop status, 0-off, 1-on;
    - ``ILC_sign``: ILC control start stop status, 0-stop, 1-training, 2-practical operation;
    - ``max_dis``:Maximum adjustment distance;
    - ``max_ang``:Maximum adjustment angle;"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    status = 1  #Constant force control open flag, 0-off, 1-on
    sensor_num = 1 #Force sensor number
    is_select = [0,0,1,0,0,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0005,0.0,0.0,0.0,0.0,0.0]  #Maximum threshold
    adj_sign = 0  #Adaptive start stop status, 0-off, 1-on
    ILC_sign = 0  #ILC control start stop status, 0-stop, 1-training, 2-practical operation
    max_dis = 100.0  #Maximum adjustment distance
    max_ang = 0.0  #Maximum adjustment angle
    J1=[-68.987,-96.414,-111.45,-61.105,92.884,11.089]
    P1=[62.795,-511.979,291.697,-179.545,3.027,-170.039]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[0.000,0.000,0.000,0.000,0.000,0.000]
    J2=[-107.596,-109.154,-104.735,-56.176,90.739,11.091]
    P2=[-294.768,-503.708,233.158,179.799,0.713,151.309]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[0.000,0.000,0.000,0.000,0.000,0.000]
    robot.MoveJ(J1,P1,9,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)    #Joint space movement PTP, tool number 9, actual test was used according to field data and tool number
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)   #Constant force control
    robot.MoveL(J2,P2,9,0,100.0,180.0,20.0,-1.0,eP2,0,0,dP2)   #Rectilinear motion in Cartesian space
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)

Spiral line exploration
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_SpiralSearch(rcs,dr,fFinsih,t,vmax)``"
    "Description", "Spiral line exploration"
    "Parameter", "- ``rcs``:Reference coordinate system, 0-tool coordinate system, 1-base coordinate system
    - ``dr``:Feed rate per circle radius, unit[mm];
    - ``fFinish``:Force or torque threshold (0-100), unit[N/Nm];
    - ``t``:Maximum exploration time,unit[ms];
    - ``vmax``:Maximum linear speed,unit[mm/s]"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
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
    dr = 0.7  #Feed per circle radius,unit[mm]
    fFinish = 1.0 #Force or moment threshold（0~100）,unit[N or Nm]
    t = 60000.0 #Maximum exploration time,unit[ms]
    vmax = 3.0 #The maximum linear velocity, unit[mm/s]
    is_select = [0,0,1,1,1,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    robot.FT_SpiralSearch(rcs,dr,fFinish,t,vmax)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)

Rotate Insert
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_RotInsertion(rcs,angVelRot,forceInsertion,angleMax,orn,angAccmax,rotorn)``"
    "Description", "Rotate Insert"
    "Parameter", "- ``rcs``:Reference coordinate system, 0-tool coordinate system, 1-base coordinate system;
    - ``angVelRot``:Rotational angular velocity: uni[t°/s];
    - ``forceInsertion``:Force or torque threshold(0~100),unit[N or Nm];
    - ``angleMax``:maximum rotation angle, unit[°];
    - ``orn``:Direction of force, 1-fz,2-mz;
    - ``angAccmax``:Maximum rotational acceleration, unit[°/s^2],not used temporarily
    - ``rotorn``:Rotation direction, 1-clockwise, 2-counterclockwise"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
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
    s_select = [0,0,1,1,1,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
    force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
    gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #Maximum threshold
    status = 1
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    robot.FT_RotInsertion(rcs,angVelRot,forceInsertion,angleMax,orn,angAccmax,rotorn)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)

Linear insertion
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn)``"
    "Description", "Linear insertion"
    "Parameter", "- ``rcs``:Reference frame, 0-Tool frame, 1-Base frame;
    - ``force_goal``:Force or torque threshold, unit[N or Nm];
    - ``lin_v``:Linear velocity, unit[mm/s];
    - ``lin_a``:Linear acceleration, unit[mm/s^2],not used temporarily;
    - ``disMax``:Maximum insertion distance,unit[mm];
    - ``linorn``:Insertion direction, 1-positive direction, 2-negative direction;"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:
    
    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
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
    is_select = [1,1,1,0,0,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
    gain = [0.00005,0.0,0.0,0.0,0.0,0.0]  #Maximum threshold
    force_torque = [0.0,0.0,-30.0,0.0,0.0,0.0]  #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
    status = 1
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    robot.FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)

Calculate the middle plane position to start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_CalCenterStart()``"
    "Description", "Calculate the middle plane position to start"
    "Parameter", "Nothing"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Calculate the middle plane position to end
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_CalCenterEnd()``"
    "Description", "Calculate the middle plane position to end"
    "Parameter", "Nothing"
    "Return value", "- Success:[0,pos] ,pos=[x,y,z,rx,ry,rz]
    - Failed:[errcode]"

Surface positioning
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_FindSurface (rcs,dir,axis,lin_v,lin_a,disMax,force_goal)``"
    "Description", "Surface positioning"
    "Parameter", "- ``rcs``: Reference frame, 0-Tool frame, 1-Base frame;
    - ``dir``:Direction of movement, 1-positive, 2-negative;
    - ``axis``:Move Axis,1-x,2-y,3-z;
    - ``lin_v``:Exploring Linear Speed,unit[mm/s];
    - ``lin_a``:Exploring Linear Acceleration,unit[mm/s^2];
    - ``disMax``:Maximum exploration distance,unit[mm]
    - ``force_goal``:Action termination force threshold,unit[N];"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
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
    P1=[-230.959,-364.017,226.179,-179.004,0.002,89.999]
    robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)       #Point to point motion in joint space
    #Look for the center in the x direction
    #The first surface
    robot.FT_CalCenterStart()
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    robot.FT_FindSurface(rcs,direction,axis,lin_v,lin_a,disMax,force_goal)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)       #Point to point motion in joint space
    robot.WaitMs(1000)
    #The second surface
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    direction = 2 #Direction of movement,1-positive direction, 2-negative direction
    robot.FT_FindSurface(rcs,direction,axis,lin_v,lin_a,disMax,force_goal)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    #Calculate the x-direction center position
    xcenter= robot.FT_CalCenterEnd()
    print(xcenter)
    xcenter = [xcenter[1],xcenter[2],xcenter[3],xcenter[4],xcenter[5],xcenter[6]]
    robot.MoveCart(xcenter,9,0,60.0,50.0,50.0,0.0,-1)
    #Look for the center in the y direction
    #The first surface
    robot.FT_CalCenterStart()
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    direction = 1 #Direction of movement,1-positive direction, 2-negative direction
    axis = 2 #Axis of movement,1-X,2-Y,3-Z
    disMax = 150.0 #Maximum exploration distance,unit[mm]
    lin_v = 6.0  #Exploring straight-line velocity,unit[mm/s]
    robot.FT_FindSurface(rcs,direction,axis,lin_v,lin_a,disMax,force_goal)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    robot.MoveCart(P1,9,0,100.0,100.0,100.0,-1.0,-1)       #Point to point motion in joint space
    robot.WaitMs(1000)
    #The second surface
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    direction = 2 #Direction of movement,1-positive direction, 2-negative direction
    robot.FT_FindSurface(rcs,direction,axis,lin_v,lin_a,disMax,force_goal)
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    #Calculate the y center position
    ycenter=robot.FT_CalCenterEnd()
    print(ycenter)
    ycenter = [ycenter[1],ycenter[2],ycenter[3],ycenter[4],ycenter[5],ycenter[6]]
    robot.MoveCart(ycenter,9,0,60.0,50.0,50.0,-1.0,-1)

Flexibility control off
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_ComplianceStop()``"
    "Description", "Flexibility control off"
    "Parameter", "Nothing"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Flexibility control on
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FT_ComplianceStart(p,force)``"
    "Description", "Flexibility control on"
    "Parameter", "- ``p``: Position adjustment coefficient or compliance coefficient
    - ``force``:flexibility opening force threshold, unit[N]"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Code example
----------------
.. code-block:: python
    :linenos:

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    J1=[-105.3,-68.0,-127.9,-75.5,90.8,77.8]
    P1=[-208.9,-274.5,334.6,178.8,-1.3,86.7]
    eP1=[0.000,0.000,0.000,0.000]
    dP1=[0.000,0.000,0.000,0.000,0.000,0.000]
    J2=[-105.3,-97.9,-101.5,-70.3,90.8,77.8]
    P2=[-264.8,-480.5,341.8,179.2,0.3,86.7]
    eP2=[0.000,0.000,0.000,0.000]
    dP2=[0.000,0.000,0.000,0.000,0.000,0.000]
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
    #Compliance control
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
    p = 0.00005  #Coefficient of position adjustment or compliance
    force = 30.0 #Compliant opening force threshold,unit[N]
    robot.FT_ComplianceStart(p,force)
    count = 15  #Number of cycles
    while(count):
        robot.MoveL(J1,P1,9,0,100.0,180.0,100.0,-1.0,eP1,0,1,dP1)   #Rectilinear motion in Cartesian space
        robot.MoveL(J2,P2,9,0,100.0,180.0,100.0,-1.0,eP2,0,0,dP2)
        count = count - 1
    robot.FT_ComplianceStop()
    status = 0
    robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
