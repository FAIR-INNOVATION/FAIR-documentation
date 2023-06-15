Trajectory recurrence
================================ 

.. toctree:: 
    :maxdepth: 5

Set trajectory recording parameters
++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTPDParam(type,name,period_ms,di_choose,do_choose)``"
    "Description", "Set trajectory recording parameters"
    "Parameter", "- ``type``:Data type, 1-joint position;
    - ``name``:Track name;
    - ``period_ms``:Sampling period, fixed value, 2ms or 4ms or 8ms;
    - ``di_choose``:DI selection, bit0~bit7 corresponds to control boxes DI0~DI7, bit8~bit9 corresponds to terminal DI0~DI1, 0-not selected, 1-selected
    - ``do_choose``:DO selection, bit0~bit7 corresponds to control boxes DO0~DO7, bit8~bit9 corresponds to terminal DO0~DO1, 0-not selected, 1-selected"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

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

    "Prototype", "``SetTPDStart(type,name,period_ms,di_choose,do_choose)``"
    "Description", "Start trajectory recording"
    "Parameter", "- ``type``:Data type, 1-joint position;
    - ``name``:Track name;
    - ``period_ms``:Sampling period, fixed value, 2ms or 4ms or 8ms;
    - ``di_choose``:DI selection, bit0~bit7 corresponds to control boxes DI0~DI7, bit8~bit9 corresponds to terminal DI0~DI1, 0-not selected, 1-selected
    - ``do_choose``:DO selection, bit0~bit7 corresponds to control boxes DO0~DO7, bit8~bit9 corresponds to terminal DO0~DO1, 0-not selected, 1-selected"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Stop trajectory recording
++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWebTPDStop()``"
    "Description", "Stop trajectory recording"
    "Parameter", "Nothing"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

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
    "Parameter", "- ``name``:Track name"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

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
    "Parameter", "- ``name``:Track name"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

Trajectory reproduction
++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveTPD(name,blend,ovl)``"
    "Description", "Trajectory reproduction"
    "Parameter", "- ``name``:Track name
    - ``blend``:Is it smooth, 0-not smooth, 1-smooth
    - ``ovl``:Speed scaling factor, range[0~100]"
    "Return value", "- Success:[0]
    - Failed:[errcode]"

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