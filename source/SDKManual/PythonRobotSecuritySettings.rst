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

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    level = [1.0,2.0,3.0,4.0,5.0,6.0]
    error = robot.SetAnticollision(0,level,1)
    print("Set collision level:",error)
    level = [50.0,20.0,30.0,40.0,50.0,60.0]
    error = robot.SetAnticollision(1,level,1)
    print("Set collision level:",error)

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

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.SetCollisionStrategy(1)
    print("Set the strategy after collision:",error)

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

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    p_limit = [170.0,80.0,150.0,80.0,170.0,160.0]
    error = robot.SetLimitPositive(p_limit)
    print("Set positive limit:",error)

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

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    n_limit = [-170.0,-260.0,-150.0,-260.0,-170.0,-160.0]
    error = robot.SetLimitNegative(n_limit)
    print("Set negative limit:",error)

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

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.ResetAllError()
    print("Error status cleared:",error)

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

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    error = robot.FrictionCompensationOnOff(1)
    print("Joint friction compensation switch:",error)

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

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    lcoeff = [0.9,0.9,0.9,0.9,0.9,0.9]
    error = robot.SetFrictionValue_level(lcoeff)
    print("Set joint friction compensation coefficient formal installation:",error)

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

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    wcoeff = [0.4,0.4,0.4,0.4,0.4,0.4]
    error = robot.SetFrictionValue_wall(wcoeff)
    print("Set joint friction compensation coefficient - Side Mount:",error)

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

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    ccoeff = [0.6,0.6,0.6,0.6,0.6,0.6]
    error =robot.SetFrictionValue_ceiling(ccoeff)
    print("Set joint friction compensation coefficient-Inverted:",error)

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

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = Robot.RPC('192.168.58.2')
    fcoeff = [0.5,0.5,0.5,0.5,0.5,0.5]
    error =robot.SetFrictionValue_freedom(fcoeff)
    print("Set joint friction compensation coefficient-free installation:",error)
