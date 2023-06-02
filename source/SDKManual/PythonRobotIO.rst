Robot IO
============

.. toctree:: 
    :maxdepth: 5

Set the digital output of the control box
++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetDO(id,status,smooth,block)``"
    "Description", "Set the digital output of the control box"
    "Parameter", "-  ``id``:io number，range[0~15]；
    - ``status``:0-off, 1-on；
    - ``smooth``:0-unsmooth, 1-smooth；
    - ``block``:0-blocking, 1-non blocking."
    "Return value", "- success:[0]
    - Failed：[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5,8

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    for i in range(0,16):
        robot.SetDO(i,1,0,0)   #Open the control box DO
    robot.WaitMs(1000)
    for i in range(0,16):
        robot.SetDO(i,0,0,0)   #Close the control box DO
    robot.WaitMs(1000)


Set tool digital output
++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolDO(id,status,smooth,block)``"
    "Description", "Set tool digital output"
    "Parameter", "-  ``id``:io number，range[0~15]；
    - ``status``:0-off, 1-on；
    - ``smooth``:0-unsmooth, 1-smooth；
    - ``block``:0-blocking, 1-non blocking."
    "Return value", "- Success:[0]
    - Failed：[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5,8

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    for i in range(0,2):
        robot.SetToolDO(i,1,0,0)    #Open the control box DO
    robot.WaitMs(1000)
    robot.WaitMs(1000)
    for i in range(0,2):
        robot.SetToolDO(i,0,0,0)    #Close the control box DO


Set the analog output of the control box
+++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAO(id,value,block)``"
    "Description", "Set the analog output of the control box"
    "Parameter", "- ``id``:io number，range[0~1]；
    - ``value``:electricity or voltage value percentage, range [0-100%] corresponds to electricity value [0-20mA] or voltage [0-10V]；
    - ``block``:[0]-blocking, [1]-non blocking"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4,6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.SetAO(0,0.0,0)    # Set control box analog output
    robot.WaitMs(1000)
    robot.SetAO(1,100.0,0)

Set tool analog output
+++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolAO(id,value,block)``"
    "Description", "Set tool analog output"
    "Parameter", "- ``id``:io number，range[0]；
    - ``value``:electricity or voltage value percentage, range [0-100%] corresponds to electricity value [0-20mA] or voltage [0-10V]；
    - ``block``:[0]-blocking, [1]-non blocking"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4,6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.SetToolAO(0,100.0,0)   # Set tool analog output
    robot.WaitMs(1000)
    robot.SetToolAO(0,0.0,0)

Obtain the digital input of the control box
+++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetDI(id,block)``"
    "Description", "Obtain the digital input of the control box"
    "Parameter", "- ``id``:io number，range[0~15]；
    - ``block``:[0]-blocking, [1]-non blocking"
    "Return value", "- Success：[0,di],di: 0-Low level，1-High level
    - Failed：[errcode,]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    di = robot.GetDI(0,0)   # Obtain the digital input of the control box
    print(di)

Obtain tool digital input
+++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetToolDI(id,block)``"
    "Description", "Obtain tool digital input"
    "Parameter", "- ``id``:io number，range[0~1]；
    - ``block``:[0]-blocking, [1]-non blocking"
    "Return value", "- Success：[0,di],di: 0-Low level，1-High level
    - Failed：[errcode,]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    tool_di = robot.GetToolDI(1,0)   # Obtain tool digital input
    print(tool_di)

Waiting for digital input from the control box
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitDI(id,status,maxtime,opt)``"
    "Description", "Waiting for digital input from the control box"
    "Parameter", "- ``id``:io number，range[0~15]；
    - ``status``:0-off，1-on；
    - ``maxtime``:Maximum waiting time, unit[ms]；
    - ``opt``:After timeout strategy, 0-program stops and prompts for timeout, 1-ignore timeout prompt to continue executing the program, 2-keep waiting"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.WaitDI(0,1,0,2)    # Waiting for the control box digital input


Waiting for multiple digital inputs from the control box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitMultiDI(mode,id,status,maxtime,opt)``"
    "Description", "Waiting for multiple digital inputs from the control box"
    "Parameter", "- ``mode``:[0]-Multiplex AND, [1]-Multiplex OR；
    - ``id``:IO number, bit0~bit7 corresponds to DI0~DI7, bit8~bit15 corresponds to CI0~CI7；
    - ``status(uint16_t)``:bit0~bit7 corresponds to DI0~DI7 status, bit8~bit15 corresponds to the states of the CI0~CI7 status bits [0]-off, [1]-on；
    - ``maxtime``:Maximum waiting time, unit[ms]；
    - ``opt``:After timeout strategy, 0-program stops and prompts for timeout, 1-ignore timeout prompt to continue executing the program, 2-keep waiting。"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.WaitMultiDI(1,3,3,10000,2)   #  Waiting for control box multiplex digital input

Waiting for tool digital input
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitToolDI(id,status,maxtime,opt)``"
    "Description", "Waiting for tool digital input"
    "Parameter", "- ``id``:io number，range[0~1]；
    - ``status``:0-off，1-on；
    - ``maxtime``:Maximum waiting time, unit[ms]；
    - ``opt``:after timeout strategy, 0-program stops and prompts for timeout, 1-ignore timeout prompt to continue executing the program, 2-keep waiting"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.WaitToolDI(1,1,0,2)    #  Wait for the tool number to enter

Waiting for terminal digital input
+++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetAI(id,block)``"
    "Description", "Waiting for terminal digital input"
    "Parameter", "- ``id``:io number，range[0~1]；
    - ``block``:[0]-blocking, [1]-non blocking。"
    "Return value", "- Success：[0,value], value:Input current or voltage value percentage, range[0-100] corresponds to current value[0-20mA] or voltage[0-10V]；
    - Failed：[errcode,]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    ai = robot.GetAI(0,1)   #  Obtain control box analog input
    print(ai)

Obtain tool analog input
++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetToolAI(id,block)``"
    "Description", "Obtain terminal analog input"
    "Parameter", "- ``id``:io number，range[0]；
    - ``block``:[0]-blocking, [1]-non blocking"
    "Return value", "- Success：[0,value], value:Input current or voltage value percentage, range[0-100] corresponds to current value[0-20mA] or voltage[0-10V]；
    - Failed：[errcode,]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    tool_ai = robot.GetToolAI(0,1)    #   Obtain tool analog input
    print(tool_ai)

Waiting for tool analog input
+++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitAI(id,sign,value,maxtime,opt)``"
    "Description", "Waiting for tool analog input"
    "Parameter", "- ``id``:io number，range[0~1]；
    - ``sign``:0-Greater than，1-Less than
    - ``value``:Input current or voltage value percentage, range[0-100] corresponds to current value[0-20mA] or voltage[0-10V]；
    - ``maxtime``:Maximum waiting time, unit[ms]；
    - ``opt``:After timeout strategy, 0-program stops and prompts for timeout, 1-ignore timeout prompt to continue executing the program, 2-keep waiting"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.WaitAI(0,0,50,0,2)   # Always waiting for tool analog input

Waiting for tool analog input
++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``WaitToolAI(id,sign,value,maxtime,opt)``"
    "Description", "Waiting for terminal analog input"
    "Parameter", "- ``id``:io number，range[0]；
    - ``sign``:0-Greater than，1-Less than
    - ``value``: Input current or voltage value percentage, range[0-100] corresponds to current value[0-20mA] or voltage[0-10V]；
    - ``maxtime``:Maximum waiting time, unit[ms]；
    - ``opt``:After timeout strategy, 0-program stops and prompts for timeout, 1-ignore timeout prompt to continue executing the program, 2-keep waiting"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
---------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.WaitToolAI(0,0,50,0,2)   #  Always waiting for tool analog input