Trajectory recurrence
================================ 

.. toctree:: 
    :maxdepth: 5

Set trajectory recording parameters
++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTPDParam(name, period_ms, type = 1,di_choose = 0, do_choose = 0)``"
    "Description", "Set trajectory recording parameters"
    "Required parameter", "- ``name``:Track name;
    - ``period_ms``:Sampling period, fixed value, 2ms or 4ms or 8ms;"
    "Optional parameter", "- ``type``:Data type, 1-joint position, default to 1;
    - ``di_choose``:DI selection, bit0~bit7 corresponds to control boxes DI0~DI7, bit8~bit9 corresponds to terminal DI0~DI1, 0-not selected, 1-selected, default to 0;
    - ``do_choose``:DO selection, bit0~bit7 corresponds to control boxes DO0~DO7, bit8~bit9 corresponds to terminal DO0~DO1, 0-not selected, 1-selected, default to 0"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
--------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 9

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    type = 1  # Data type, 1-joint position
    name = 'tpd2023'  # Track name
    period = 4  #Sampling period, fixed value, 2ms or 4ms or 8ms
    di_choose = 0 # di input configuration
    do_choose = 0 # do output configuration
    robot.SetTPDParam(type, name, period, di_choose, do_choose)    #Configure TPD Parameter

Start trajectory recording
++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTPDStart(name, period_ms, type = 1,di_choose = 0, do_choose = 0)``"
    "Description", "Start trajectory recording"
    "Required parameter", "-  ``name``:Track name;
    - ``period_ms``:Sampling period, fixed value, 2ms or 4ms or 8ms;"
    "Optional parameter", "- ``type``:Data type, 1-joint position;
    - ``di_choose``:DI selection, bit0~bit7 corresponds to control boxes DI0~DI7, bit8~bit9 corresponds to terminal DI0~DI1, 0-not selected, 1-selected, default to 0;
    - ``do_choose``:DO selection, bit0~bit7 corresponds to control boxes DO0~DO7, bit8~bit9 corresponds to terminal DO0~DO1, 0-not selected, 1-selected, default to 0"
    "Return value", "Errcode: Success -0  Failed -errcode"

Stop trajectory recording
++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWebTPDStop()``"
    "Description", "Stop trajectory recording"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 14, 16

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    type = 1  # Data type, 1-joint position
    name = 'tpd2023'  # Track name
    period = 4  #Sampling period, fixed value, 2ms or 4ms or 8ms
    di_choose = 0 # di input configuration
    do_choose = 0 # do output configuration
    robot.SetTPDParam(type, name, period, di_choose, do_choose)    #Configure TPD Parameter
    robot.Mode(1)  # The robot goes into manual mode
    time.sleep(1)  
    robot.DragTeachSwitch(1)  #The robot goes into drag teaching mode
    robot.SetTPDStart(type, name, period, di_choose, do_choose)   # Start recording the teaching track
    time.sleep(30)
    robot.SetWebTPDStop()  # Stop recording instructional tracks
    robot.DragTeachSwitch(0)  #The robot enters the non-drag teaching mode

Delete trajectory record
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTPDDelete(name)``"
    "Description", "Delete trajectory record"
    "Required parameter", "- ``name``:Track name"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.SetTPDDelete('tpd2023')   # Delete the TPD trace

Trajectory preloading
+++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoadTPD(name)``"
    "Description", "Trajectory preloading"
    "Required parameter", "- ``name``:Track name"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

ObtainTPD start pose
+++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTPDStartPose(name)``"
    "Description", "ObtainTPD start pose"
    "Required parameter", "- ``name``:Track name"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode
    - Return(if success): desc_pose [x,y,z,rx,ry,rz]"

Trajectory reproduction
++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveTPD(name,blend,ovl)``"
    "Description", "Trajectory reproduction"
    "Required parameter", "- ``name``:Track name
    - ``blend``:Is it smooth, 0-not smooth, 1-smooth
    - ``ovl``:Speed scaling factor, range[0~100]"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 8, 10

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    P1=[-378.9,-340.3,107.2,179.4,-1.3,125.0]
    name = 'tpd2023'   #Track name
    blend = 1   #Is it smooth, 0-not smooth, 1-smooth
    ovl = 100.0   #Speed scaling
    robot.LoadTPD(name)  #Trajectory preloading
    robot.MoveCart(P1,1,0,100.0,100.0,100.0,-1.0,-1)       #Let's go to the starting point
    robot.MoveTPD(name, blend, ovl)  #Trajectory reproduction

Track preprocessing - Import track files
++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``LoadTrajectoryJ(name,ovl,opt = 1)``"
    "Description", "Track preprocessing - Import track files"
    "Required parameter", "- ``name``:Track name
    - ``ovl``:Speed scaling factor, range[0~100]
    - ``opt``:1- Control point, default to 1"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Trajectory reproduction - Import track files
++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveTrajectoryJ()``"
    "Description", "Trajectory reproduction - Import track files"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Obtain trajectory start pose - Import track files
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTrajectoryStartPose(name)``"
    "Description", "Obtain trajectory start pose - Import track files"
    "Required parameter", "``name``:Track name"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode
    - Return(if success): desc_pose [x,y,z,rx,ry,rz]"

Obtain trajectory point number - Import track files
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTrajectoryPointNum()``"
    "Description", "Obtain trajectory point number - Import track files"
    "Required parameter", "``name``:Track name"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode
    - Return(if success):  pnum"

Set trajectory speed - Import track files
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJSpeed(ovl)``"
    "Description", "Set trajectory speed - Import track files"
    "Required parameter", "``ovl``:Speed scaling percentage, range [0~100]"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Set trajectory force and torque- Import track files
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJForceTorque(ft)``"
    "Description", "Set trajectory force and torque- Import track files"
    "Required parameter", "``ft``:[fx,fy,fz,tx,ty,tz], unit[N or Nm];"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Set trajectory force Fx- Import track files
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJForceFx(fx)``"
    "Description", "Set trajectory force Fx- Import track files"
    "Required parameter", "``fx``:Force in the x direction, unit N"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Set trajectory force Fy- Import track files
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJForceFx(fy)``"
    "Description", "Set trajectory force Fy- Import track files"
    "Required parameter", "``fy``:Force in the y direction, unit N"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Set trajectory force Fz- Import track files
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJForceFx(fz)``"
    "Description", "Set trajectory force Fz- Import track files"
    "Required parameter", "``fz``:Force in the z direction, unit N"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Set trajectory torqueTx- Import track files
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJTorqueTx(tx)``"
    "Description", "Set trajectory torqueTx- Import track files"
    "Required parameter", "``tx``:Torque around the x axis, unit Nm"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Set trajectory torqueTy- Import track files
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJTorqueTx(ty)``"
    "Description", "Set trajectory torqueTy- Import track files"
    "Required parameter", "``ty``:Torque around the y axis, unit Nm"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Set trajectory torqueTz- Import track files
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTrajectoryJTorqueTx(tz)``"
    "Description", "Set trajectory torqueTy- Import track files"
    "Required parameter", "``tz``:Torque around the z axis, unit Nm"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"
