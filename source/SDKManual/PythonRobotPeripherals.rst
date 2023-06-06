Peripheral
=========================

.. toctree:: 
    :maxdepth: 5

Obtain gripper configuration
+++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperConfig()``"
    "Description", "Obtain gripper configuration"
    "Parameter", "Nothing"
    "Return value", "- Success：[0, company,device,softversion,bus],company:夹爪厂商
    - Failed：[errcode]"

Activate gripper
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ActGripper(index,action)``"
    "Description", "Activate gripper"
    "Parameter", "- ``index``:Claw number；
    - ``action``: 0-reset, 1-activate"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Control gripper
++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``MoveGripper(index,pos,speed,force,maxtime,block)``"
    "Description", "Control gripper"
    "Parameter", "- ``index``:Claw number；
    - ``pos``:Position percentage, range[0~100]；
    - ``speed``:Speed percentage, range[0~100];
    - ``force``:Moment percentage, range[0~100]；
    - ``maxtime``:Maximum waiting time, range[0~30000]，unit[ms]；
    - ``block``:0-blocking, 1-non blocking"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Obtain gripper movement status
++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetGripperMotionDone()``"
    "Description", "Obtain gripper movement status"
    "Parameter", "Nothing"
    "Return value", "- Success：[0,status], status:0-incomplete movement，1-exercise completion
    - Failed：[errcode]"

Configure gripper
+++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetGripperConfig(company,device,softversion,bus)``"
    "Description", "Configure gripper"
    "Parameter", "- ``company``：Claw manufacturers, 1-Robotiq, 2-Huiling, 3-Tianji, 4-Dahuan, 5-Zhixing；
    - ``device``：Equipment number: Robotiq(0-2F-85 series), Huiling(0-NK series, 1-Z-EFG-100), Tianji(0-TEG-110), Dahuan(0-PGI-140), Zhixing(0-CTPM2F20)
    - ``softversion``：Software version number, temporarily not used, defaults to 0；
    - ``bus``：Device mounted terminal bus position, temporarily not used, defaults to 0；"
    "Return value", "- Success：[0]
    - Failed：[errcode]"

Code example
--------------
.. code-block:: python
    :linenos:

    import frrpc
    import time
    # A connection is established with the robot controller. A successful connection returns a robot object
    robot = frrpc.RPC('192.168.58.2')
    robot.SetGripperConfig(4,0,0,1)  # Configuring Clamping Claws
    time.sleep(1)
    config = robot.GetGripperConfig()  # obtain gripper configuration
    print(config)
    robot.ActGripper(1,0)   # Claw reset
    time.sleep(1)
    robot.ActGripper(1,1)   # Claw activation
    time.sleep(2)
    robot.MoveGripper(1,100,48,46,30000,0)   # Claw movement
    time.sleep(3)
    robot.MoveGripper(1,0,50,0,30000,0)
    ret = robot.GetGripperMotionDone()    # Example Query the status of the claw movement
    print(ret)
