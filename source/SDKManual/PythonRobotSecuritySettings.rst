Security settings
=========================

.. toctree:: 
    :maxdepth: 5


Set collision level
++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAnticollision (mode,level,config)``"
    "Description", "Set collision level"
    "Required parameter", "- ``mode``:0-level, 1-percentage;;
    - ``level=[j1,j2,j3,j4,j5,j6]``:collision threshold;
    - ``config``:0-do not update configuration file, 1-update configuration file"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5,7

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    level = [1.0,2.0,3.0,4.0,5.0,6.0]
    robot.SetAnticollision(0,level,1)     #  Set collision level
    level = [50.0,20.0,30.0,40.0,50.0,60.0]
    robot.SetAnticollision(1,level,1)     #  Set collision percentage

Set the strategy after collision
+++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetCollisionStrategy (strategy)``"
    "Description", "Set the strategy after collision"
    "Required parameter", "- ``strategy``:0-Error Pause, 1-Continue Running"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.SetCollisionStrategy(1)    # Set post collision strategy,1-Continue Running

Set positive limit
+++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLimitPositive(p_limit)``"
    "Description", "Set positive limit"
    "Required parameter", "- ``p_limit=[j1,j2,j3,j4,j5,j6]``:six joint positions"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    p_limit = [170.0,80.0,150.0,80.0,170.0,160.0]
    robot.SetLimitPositive(p_limit)   #  Set positive limit

Set negative limit
++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLimitNegative(n_limit)``"
    "Description", "Set negative limit"
    "Required parameter", "- ``n_limit=[j1,j2,j3,j4,j5,j6]``:six joint positions"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
-------------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 5

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    n_limit = [-170.0,-260.0,-150.0,-260.0,-170.0,-160.0]
    robot.SetLimitNegative(n_limit)   # Set negative limit

Error status cleared
+++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ResetAllError()``"
    "Description", "Error status cleared,only resettable errors can be cleared"
    "Required parameter", "Nothing"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
--------------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.ResetAllError()    #  Error status cleared

Joint friction compensation switch
++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FrictionCompensationOnOff(state)``"
    "Description", "Joint friction compensation switch"
    "Required parameter", "- ``state``:0-off,1-on"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
-------------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 4

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.FrictionCompensationOnOff(1)   #  Joint friction compensation open

Set joint friction compensation coefficient formal installation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetFrictionValue_level(coeff)``"
    "Description", "Set joint friction compensation coefficient - formal installation"
    "Required parameter", "- ``coeff=[j1,j2,j3,j4,j5,j6]``:six joint compensation coefficients"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
---------------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.FrictionCompensationOnOff(1)   #  Joint friction compensation open
    lcoeff = [0.9,0.9,0.9,0.9,0.9,0.9]
    robot.SetFrictionValue_level(lcoeff)   #  Set joint friction compensation coefficient

Set joint friction compensation coefficient - Side Mount
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetFrictionValue_wall(coeff)``"
    "Description", "Set joint friction compensation coefficient - Side Mount"
    "Required parameter", "- ``coeff=[j1,j2,j3,j4,j5,j6]``:six joint compensation coefficients"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
--------------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.FrictionCompensationOnOff(1)   #  Joint friction compensation open
    wcoeff = [0.4,0.4,0.4,0.4,0.4,0.4]
    robot.SetFrictionValue_wall(wcoeff)  #  Set joint friction compensation coefficient

Set joint friction compensation coefficient-Inverted
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetFrictionValue_ceiling(coeff)``"
    "Description", "Set joint friction compensation coefficient-Inverted"
    "Required parameter", "- ``coeff=[j1,j2,j3,j4,j5,j6]``:six joint compensation coefficients"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
----------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.FrictionCompensationOnOff(1)   #  Joint friction compensation open
    ccoeff = [0.6,0.6,0.6,0.6,0.6,0.6]
    robot.SetFrictionValue_ceiling(ccoeff)  #  Set joint friction compensation coefficient

Set joint friction compensation coefficient-free installation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetFrictionValue_freedom(coeff)``"
    "Description", "Set joint friction compensation coefficient-free installation"
    "Required parameter", "- ``coeff=[j1,j2,j3,j4,j5,j6]``:six joint compensation coefficients"
    "Optional parameter", "Nothing"
    "Return value", "Errcode: Success -0  Failed -errcode"

Code example
------------------
.. code-block:: python
    :linenos:
    :emphasize-lines: 6

    import frrpc
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.FrictionCompensationOnOff(1)   #  Joint friction compensation open
    fcoeff = [0.5,0.5,0.5,0.5,0.5,0.5]
    robot.SetFrictionValue_freedom(fcoeff)   #  Set joint friction compensation coefficient
