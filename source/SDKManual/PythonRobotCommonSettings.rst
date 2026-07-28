Common Robot Settings
====================================================================

.. toctree:. 
    :maxdepth: 5

Setting Tool Reference Points - Six-Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolPoint(point_num)``"
    "Description", "Setting Tool Reference Points - Six Point Method"
    "Mandatory parameters", "- ``point_num``: point number, range [1~6]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Calculation tool coordinate system - six-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeTool()``"
    "Description", "Calculate the tool coordinate system - six-point method (after setting the six tool reference points)"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``tcp_pose=[x,y,z,rx,ry,rz]``: tool coordinate system"

Setting Tool Reference Points - Four Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetTcp4RefPoint(point_num)``"
    "Description", "Setting Tool Reference Points - Four Point Method"
    "Mandatory parameter", "``point_num``: point number, range [1~4]"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``tcp_pose=[x,y,z,rx,ry,rz]``: tool coordinate system"

Calculation Tool Coordinate System - Four Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``ComputeTcp4()``"
    "Description", "Calculate tool coordinate system - four-point method (after setting the four tool reference points)"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``tcp_pose=[x,y,z,rx,ry,rz]``: tool coordinate system"

Calculate the tool coordinate system based on the point information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeToolCoordWithPoints(method, pos)``"
    "Description", "Calculate the tool coordinate system based on the point information"
    "Mandatory parameters", "- ``method``:Calculation method; 0-four point method; One - six point method
    - ``pos``:The array length of the joint position group is 4 in the four-point method and 6 in the six-point method"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode  
    - ``tcp_offset=[x,y,z,rx,ry,rz]``:Tool coordinate system calculated from point information, unit [mm][°]"

Setting the tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolCoord(id,t_coord,type,install,toolID,loadNum)``"
    "Description", "Setting the tool coordinate system"
    "Mandatory parameters", "- ``id``: coordinate system number, range [1~15];
    - ``t_coord``: Position of the tool center point relative to the center of the end flange in [mm][°];
    - ``type``: 0 - tool coordinate system, 1 - sensor coordinate system;
    - ``install``: installation position, 0 - robot end, 1 - robot exterior
    - ``toolID``: tool ID
    - ``loadNum``: load number"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Setting the tool coordinate system list
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetToolList(id,t_coord ,type, install, loadNum)``"
    "Description", "Set up a list of tool coordinate systems"
    "Mandatory parameters", "- ``id``: coordinate system number, range [1~15];
    - ``t_coord``: [x,y,z,rx,ry,rz] Tool center point relative to end flange center position in [mm][°];
    - ``type``: 0 - tool coordinate system, 1 - sensor coordinate system;
    - ``install``: installation position, 0 - robot end, 1 - robot exterior
    - ``loadNum``: load number"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Get the current tool coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTCPOffset(flag=1)``"
    "Description", "Get current tool coordinate system"
    "Mandatory parameters", "NULL"
    "Default parameters", "``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``tcp_offset=[x,y,z,rx,ry,rz]``: Relative position of the current tool coordinate system in [mm][°]"

Robot tool coordinate system manipulation code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    p1Desc = [186.331, 487.913, 209.850, 149.030, 0.688, -114.347]
    p2Desc = [69.721, 535.073, 202.882, -144.406, -14.775, -89.012]
    p3Desc = [146.861, 578.426, 205.598, 175.997, -36.178, -93.437]
    p4Desc = [136.284, 509.876, 225.613, 178.987, 1.372, -100.696]
    p5Desc = [138.395, 505.972, 298.016, 179.134, 2.147, -101.110]
    p6Desc = [105.553, 454.325, 232.017, -179.426, 0.444, -99.952]
    p1Joint = [-127.876, -75.341, 115.417, -122.741, -59.820, 74.300]
    p2Joint = [-101.780, -69.828, 110.917, -125.740, -127.841, 74.300]
    p3Joint = [-112.851, -60.191, 86.566, -80.676, -97.463, 74.300]
    p4Joint = [-116.397, -76.281, 113.845, -128.611, -88.654, 74.299]
    p5Joint = [-116.814, -82.333, 109.162, -118.662, -88.585, 74.302]
    p6Joint = [-115.649, -84.367, 122.447, -128.663, -90.432, 74.303]
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    posJ = [p1Joint, p2Joint, p3Joint, p4Joint, p5Joint, p6Joint]
    rtn,coordRtn = robot.ComputeToolCoordWithPoints(1, posJ)
    print(f"ComputeToolCoordWithPoints    {rtn}  coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} {coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")
    robot.MoveJ(joint_pos=p1Joint,tool=0, user=0, vel=100)
    robot.SetToolPoint(1)
    robot.MoveJ(joint_pos=p2Joint,tool=0, user=0, vel=100)
    robot.SetToolPoint(2)
    robot.MoveJ(joint_pos=p3Joint,tool=0, user=0, vel=100)
    robot.SetToolPoint(3)
    robot.MoveJ(joint_pos=p4Joint,tool=0, user=0, vel=100)
    robot.SetToolPoint(4)
    robot.MoveJ(joint_pos=p5Joint,tool=0, user=0, vel=100)
    robot.SetToolPoint(5)
    robot.MoveJ(joint_pos=p6Joint,tool=0, user=0, vel=100)
    robot.SetToolPoint(6)
    rtn,coordRtn = robot.ComputeTool()
    print(f"6 Point ComputeTool        {rtn}  coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} {coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")
    robot.SetToolList(1, coordRtn, 0, 0, 0)
    robot.MoveJ(joint_pos=p1Joint,tool=0, user=0, vel=100)
    robot.SetTcp4RefPoint(1)
    robot.MoveJ(joint_pos=p2Joint,tool=0, user=0, vel=100)
    robot.SetTcp4RefPoint(2)
    robot.MoveJ(joint_pos=p3Joint,tool=0, user=0, vel=100)
    robot.SetTcp4RefPoint(3)
    robot.MoveJ(joint_pos=p4Joint,tool=0, user=0, vel=100)
    robot.SetTcp4RefPoint(4)
    rtn,coordRtn = robot.ComputeTcp4()
    print(f"4 Point ComputeTool        {rtn}  coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} {coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")
    robot.SetToolCoord(2, coordRtn, 0, 0, 1, 0)
    rtn,getCoord = robot.GetTCPOffset(0)
    print(f"GetTCPOffset    {rtn}  coord is {getCoord[0]} {getCoord[1]} {getCoord[2]} {getCoord[3]} {getCoord[4]} {getCoord[5]}")
    robot.CloseRPC()

Setting External Tool Reference Points-Six-Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExTCPPoint(point_num)``"
    "Description", "Setting the external tool reference point - three-point method"
    "Mandatory parameters", "- ``point_num``: point number, range [1~3]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Calculation of the external tool coordinate system - Six-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeExTCF(point_num)``"
    "Description", "Calculate external tool coordinate system - three-point method (after setting three reference points)"
    "Mandatory parameter", "``point_num``: point number, range [1~3]"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``etcp=[x,y,z,rx,ry,rz]``: external tool coordinate system"

Setting the external tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExToolCoord(id,etcp,etool)``"
    "Description", "Setting the external tool coordinate system"
    "Mandatory parameters", "- ``id``: coordinate system number, 20-39 correspond to external tool coordinate systems 0-19;
    - ``etcp``: [x,y,z,rx,ry,rz] External tool coordinate system in [mm][°];
    - ``etool``: [x,y,z,rx,ry,rz] end-tool coordinate system in [mm] [°];"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting up a list of external tool coordinate systems
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetExToolList(id,etcp ,etool)``"
    "Description", "Set the list of external tool coordinate systems"
    "Mandatory parameters", "- ``id``: coordinate system number, 20-39 correspond to external tool coordinate systems 0-19;
    - ``etcp``: External tool coordinate system in [mm][°];
    - ``etool``: end-tool coordinate system in [mm] [°];"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Example code for robot external tool coordinate system operation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from time import sleep
    import time
    from fairino import Robot

    robot = Robot.RPC('192.168.58.2')

    def TestExtCoord(self):
        p1Desc = [-89.606, 779.517, 193.516, 178.000, 0.476, -92.484]
        p1Joint = [-108.145, -50.137, 85.818, -125.599, -87.946, 74.329]

        p2Desc = [-24.656, 850.384, 191.361, 177.079, -2.058, -95.355]
        p2Joint = [-111.024, -41.538, 69.222, -114.913, -87.743, 74.329]

        p3Desc = [-99.813, 766.661, 241.878, -176.817, 1.917, -91.604]
        p3Joint = [-107.266, -56.116, 85.971, -122.560, -92.548, 74.331]

        exaxisPos = [0, 0, 0, 0]
        offdese = [0, 0, 0, 0, 0, 0]

        posTCP = [p1Desc, p2Desc, p3Desc]
        # coordRtn = DescPose()

        robot.MoveJ(joint_pos=p1Joint,tool=1, user=0, vel=50)
        robot.SetExTCPPoint(1)
        robot.MoveJ(joint_pos=p2Joint,tool=1, user=0, vel=50)
        robot.SetExTCPPoint(2)
        robot.MoveJ(joint_pos=p3Joint,tool=1, user=0, vel=50)
        robot.SetExTCPPoint(3)
        rtn,coordRtn = robot.ComputeExTCF()
        print(f"ComputeExTCF {rtn}  coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} {coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")

        robot.SetExToolCoord(21, coordRtn, offdese)
        robot.SetExToolList(21, coordRtn, offdese)

        robot.CloseRPC()
    TestExtCoord(robot)

Setting the workpiece reference point - three-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWObjCoordPoint(point_num)``"
    "Description", "Setting the workpiece reference point - 3-point method"
    "Mandatory parameter", "``point_num``: point number, range [1~3]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Calculation of the workpiece coordinate system - three-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeWObjCoord(method, refFrame)``"
    "Description", "Calculate the workpiece coordinate system - three-point method (three reference points are set and then calculated);"
    "Mandatory parameters","- ``method``: calculation method 0: origin-x-axis-z-axis, 1: origin-x-axis-xy-plane
    - ``refFrame``: reference coordinate system"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode 
    - ``wobj_pose=[x,y,z,rx,ry,rz]``: workpiece coordinate system"

Setting the workpiece coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetWObjCoord(id, coord, refFrame)``"
    "Description", "Setting the workpiece coordinate system"
    "Mandatory parameters", "- ``id``: coordinate system number, range [0~14];
    - ``COORD``: Position of the workpiece in the coordinate system relative to the center of the end flange in [mm][°].
    - ``refFrame``: reference coordinate system"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the list of workpiece coordinate systems
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``SetWObjList(id, coord, refFrame)``"
    "Description", "Set the list of workpiece coordinate systems"
    "Mandatory parameters", "- ``id``: coordinate system number, range [0~14];
    - ``COORD``: Position of the workpiece in the coordinate system relative to the center of the end flange in [mm][°].
    - ``refFrame``: reference coordinate system"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Calculate the workpiece coordinate system based on the point information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.8

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeWObjCoordWithPoints(method, pos, refFrame)``"
    "Description", "Calculate the workpiece coordinate system based on the point information"
    "Mandatory parameters", "- ``method``:Calculation method; 0: origin - X-axis - Z-axis 1: origin - X-axis -xy plane
    - ``pos``:Three TCP location groups
    - ``refFrame``: reference coordinate system"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode 
    - ``wobj_offset=[x,y,z,rx,ry,rz]``:Workpiece coordinate system calculated from point information, unit [mm][°]"

Get the current workpiece coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "prototype", "``GetWObjOffset(flag=1)``"
    "Description", "Get current workpiece coordinate system"
    "Mandatory parameters", "NULL"
    "Default parameters", "``flag``: 0-blocking, 1-non-blocking, default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``wobj_offset=[x,y,z,rx,ry,rz]``: Relative position of the current workpiece coordinate system in [mm][°]"

Example of robot workpiece coordinate system manipulation code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    p1Desc = [-89.606, 779.517, 193.516, 178.000, 0.476, -92.484]
    p2Desc = [-24.656, 850.384, 191.361, 177.079, -2.058, -95.355]
    p3Desc = [-99.813, 766.661, 241.878, -176.817, 1.917, -91.604]
    p1Joint = [-108.145, -50.137, 85.818, -125.599, -87.946, 74.329]
    p2Joint = [-111.024, -41.538, 69.222, -114.913, -87.743, 74.329]
    p3Joint = [-107.266, -56.116, 85.971, -122.560, -92.548, 74.331]
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    posTCP = [p1Desc, p2Desc, p3Desc]
    rtn,coordRtn = robot.ComputeWObjCoordWithPoints(1, posTCP, 0)
    print(f"ComputeWObjCoordWithPoints    {rtn}  coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} {coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")
    robot.MoveJ(joint_pos=p1Joint,tool=1, user=0, vel=100)
    robot.SetWObjCoordPoint(1)
    robot.MoveJ(joint_pos=p2Joint,tool=1, user=0, vel=100)
    robot.SetWObjCoordPoint(2)
    robot.MoveJ(joint_pos=p3Joint,tool=1, user=0, vel=100)
    robot.SetWObjCoordPoint(3)
    rtn,coordRtn = robot.ComputeWObjCoord(1, 0)
    print(f"ComputeWObjCoord   {rtn}  coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} {coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")
    robot.SetWObjCoord(1, coordRtn, 0)
    robot.SetWObjList(1, coordRtn, 0)
    rtn,getWobjDesc = robot.GetWObjOffset(0)
    print(f"GetWObjOffset    {rtn}  coord is {getWobjDesc[0]} {getWobjDesc[1]} {getWobjDesc[2]} {getWobjDesc[3]} {getWobjDesc[4]} {getWobjDesc[5]}")
    robot.CloseRPC()

Setting the global speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetSpeed(vel)``"
    "Description", "Set global speed"
    "Mandatory parameter", "- ``vel``: percentage of speed, range [0~100]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting robot acceleration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.0.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetOaccScale(acc)``"
    "Description", "Setting the robot acceleration"
    "Mandatory parameter", "- ``acc``: percentage of robot acceleration"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Getting the default speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetDefaultTransVel()``"
    "Description", "Get Default Speed"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``vel``: default speed in [mm/s]"

Setting the end load weight
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLoadWeight(loadNum, weight)``"
    "Description", "Set the end load weight, incorrect load weight setting may cause the robot to go out of control in drag mode"
    "Mandatory parameters", "- ``loadNum``:load number
    - ``weight``: unit [kg]"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Setting the end load center of mass coordinates
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetLoadCoord(x,y,z,loadNum = 0)``"
    "Description", "Set end-load center of mass coordinates, incorrect load center of mass setting may cause robot to go out of control in drag mode"
    "Mandatory parameter", "- ``x``: center of mass coordinates in [mm].
    - ``y``: coordinates of the center of mass in [mm].
    - ``z``: coordinates of the center of mass in [mm]"
    "Default parameters", "- ``loadNum``: Payload number, 0 by default"
    "Return Value", "Error Code Success-0 Failure- errcode "

Get the weight of the current load
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetPayload(flag=1)``"
    "Description", "Get the quality of the current load"
    "Mandatory parameters", "NULL"
    "Default parameters", "``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``weight``: current load weight in [kg]"

Get the center of mass of the current load
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetTargetPayloadCog(flag=1)``"
    "Description", "Get the center of mass of the current load"
    "Mandatory parameters", "NULL"
    "Default parameters", "``flag``: 0-blocking, 1-non-blocking Default 1"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``cog=[x,y,z]``: coordinates of the current center of mass in [mm]"

Setting the robot mounting method - fixed mounting
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotInstallPos(method)``"
    "description", "set robot mounting method - fixed mounting, wrong mounting method setting can cause robot to lose control in drag mode"
    "Mandatory parameters", "- ``method``: 0-flat loading, 1-side loading, 2-hanging loading"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode "

Setting the robot mounting angle - free mounting
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotInstallAngle(yangle,zangle)``"
    "Description", "Setting the robot mounting angle - free mounting, wrong mounting angle setting can cause the robot to go out of control in drag mode"
    "Mandatory parameters", "- ``yangle``: angle of inclination;
    - ``zangle``: angle of rotation"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Getting the robot mounting angle
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotInstallAngle()``"
    "Description", "Get robot mounting angle"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``[yangle,zangle]``: yangle - angle of inclination, zangle - angle of rotation."

Setting system variable values
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetSysVarValue(id,value)``"
    "Description", "Setting System Variables"
    "Mandatory parameters", "- ``id``: variable number, in the range [1~20].
    - ``value``: variable value"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Getting system variable values
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSysVarValue(id)``"
    "Description", "Get system variable values"
    "Mandatory parameters", "- ``id``: system variable number, range [1~20]"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``var_value``: system variable value"

Robot common setup code examples
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    for i in range(1, 100):
        robot.SetSpeed(i)
        robot.SetOaccScale(i)
        time.sleep(0.03)
    error,defaultVel = robot.GetDefaultTransVel()
    print(f"GetDefaultTransVel is {defaultVel}")
    for i in range(1, 21):
        robot.SetSysVarValue(i, i + 0.5)
        time.sleep(0.1)
    for i in range(1, 21):
        value = robot.GetSysVarValue(i)
        print(f"sys value {i} is: {value}")
        time.sleep(0.1)
    robot.SetLoadWeight(0, 2.5)
    robot.SetLoadCoord(3.0,4.0,5.0)
    time.sleep(1)
    error,getLoad = robot.GetTargetPayload(0)
    error,getLoadTran = robot.GetTargetPayloadCog(0)
    print(f"get load is {getLoad}; get load cog is {getLoadTran[0]} {getLoadTran[1]} {getLoadTran[2]}")
    robot.SetRobotInstallPos(0)
    robot.SetRobotInstallAngle(15.0, 25.0)
    error,[anglex, angley] = robot.GetRobotInstallAngle()
    print(f"GetRobotInstallAngle x: {anglex}; y: {angley}")
    robot.CloseRPC()

Joint Friction Compensation Switch
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FrictionCompensationOnOff(state)``"
    "Description", "Joint Friction Compensation Switch"
    "Mandatory parameters", "- ``state``: 0-off, 1-on"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the joint friction compensation coefficients - positive loading
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetFrictionValue_level(coeff)``"
    "Description", "Setting the joint friction compensation coefficient - Fixed mounting - Positive mounting"
    "Mandatory parameters", "- ``coeff=[j1,j2,j3,j4,j5,j6]``: six joint compensation coefficients"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the joint friction compensation coefficient - side mounting
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetFrictionValue_wall(coeff)``"
    "Description", "Setting the joint friction compensation coefficient - fixed mounting - side mounting"
    "Mandatory parameters", "- ``coeff=[j1,j2,j3,j4,j5,j6]``: six joint compensation coefficients"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the Joint Friction Compensation Factor - Inverted
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetFrictionValue_ceiling(coeff)``"
    "Description", "Setting the joint friction compensation coefficient - fixed mounting - inverted mounting"
    "Mandatory parameters", "- ``coeff=[j1,j2,j3,j4,j5,j6]``: six joint compensation coefficients"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Setting the joint friction compensation factor - free mounting
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetFrictionValue_freedom(coeff)``"
    "Description", "Setting the joint friction compensation factor - free mounting"
    "Mandatory parameters", "- ``coeff=[j1,j2,j3,j4,j5,j6]``: six joint compensation coefficients"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Robot setup joint friction compensation code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    lcoeff = [0.9, 0.9, 0.9, 0.9, 0.9, 0.9]
    wcoeff = [0.4, 0.4, 0.4, 0.4, 0.4, 0.4]
    ccoeff = [0.6, 0.6, 0.6, 0.6, 0.6, 0.6]
    fcoeff = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
    rtn = robot.FrictionCompensationOnOff(1)
    print(f"FrictionCompensationOnOff rtn is {rtn}")
    rtn = robot.SetFrictionValue_level(lcoeff)
    print(f"SetFrictionValue_level rtn is {rtn}")
    rtn = robot.SetFrictionValue_wall(wcoeff)
    print(f"SetFrictionValue_wall rtn is {rtn}")
    rtn = robot.SetFrictionValue_ceiling(ccoeff)
    print(f"SetFrictionValue_ceiling rtn is {rtn}")
    rtn = robot.SetFrictionValue_freedom(fcoeff)
    print(f"SetFrictionValue_freedom rtn is {rtn}")
    robot.CloseRPC()

Query Robot Error Code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetRobotErrorCode()``"
    "Description", "Query Robot Error Code"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- errorcode Success-0 Failure- errcode
    - ``[maincode subcode]``: robot error code, maincode - main error code, subcode - suberror code"

Error state clearing
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ResetAllError()``"
    "Description", "Error state clearing, only resettable errors can be cleared"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Robot fault state acquisition and clearing error code examples
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    p1Joint = [-108.145, -50.137, 85.818, -125.599, -87.946, 74.329]
    robot.MoveJ(joint_pos=p1Joint, tool=5, user=2, vel=50)
    time.sleep(1)
    error,[maincode, subcode] = robot.GetRobotErrorCode()
    print(f"robot maincode is {maincode}; subcode is {subcode}")
    time.sleep(1)
    robot.ResetAllError()
    time.sleep(1)
    error,[maincode, subcode] = robot.GetRobotErrorCode()
    print(f"robot maincode is {maincode}; subcode is {subcode}")
    robot.CloseRPC()

Set the monitoring parameters for the temperature and fan speed of the wide-voltage control box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWideBoxTempFanMonitorParam(enable, period)``"
    "Description", "Set the monitoring parameters for the temperature and fan speed of the wide-voltage control box"
    "Mandatory parameters", "- ``enable``: 0- Monitoring is not enabled; 1- Enable monitoring
    - ``period``: Monitoring period (s), ranging from 1 to 100"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Obtain the monitoring parameters of the temperature and fan speed of the wide-voltage control box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.3

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetWideBoxTempFanMonitorParam()``"
    "Description", "Obtain the monitoring parameters of the temperature and fan speed of the wide-voltage control box"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``enable``: 0- Monitoring is not enabled; 1- Enable monitoring
    - ``period``: Monitoring period (s), ranging from 1 to 100"

Sample code for obtaining wide voltage control box temperature and fan current status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    robot.SetWideBoxTempFanMonitorParam(1, 2)
    error, enable, period = robot.GetWideBoxTempFanMonitorParam()
    print(f"GetWideBoxTempFanMonitorParam enable is:{enable},period is:{period}")
    for i in range(100):
        error, pkg = robot.GetRobotRealTimeState()
        print(f"robot ctrl box temp is:{pkg.wideVoltageCtrlBoxTemp},fan current is:{pkg.wideVoltageCtrlBoxFanCurrent}")
        time.sleep(0.1)
    rtn = robot.SetWideBoxTempFanMonitorParam(0, 2)
    print(f"SetWideBoxTempFanMonitorParam rtn is:{rtn}")
    error, enable, period = robot.GetWideBoxTempFanMonitorParam()
    print(f"GetWideBoxTempFanMonitorParam enable is:{enable},period is:{period}")
    for i in range(100):
        error, pkg = robot.GetRobotRealTimeState()
        print(f"robot ctrl box temp is:{pkg.wideVoltageCtrlBoxTemp},fan current is:{pkg.wideVoltageCtrlBoxFanCurrent}")
        time.sleep(0.1)
    robot.CloseRPC()

Sets the focus point
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetFocusCalibPoint(pointNum, point)``"
    "Description", "Sets the focus point"
    "Mandatory parameters", "- ``pointNum``：The number of the focus calibration point 1-8
    - ``point``：Coordinate the points"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Calculate the focus calibration result
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``ComputeFocusCalib(pointNum)``"
    "Description", "Calculate the focus calibration result"
    "Mandatory parameters", "- ``pointNum``：Number of calibration points"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``resultPos``：Calibration result XYZ
    - ``accuracy``：Calibration accuracy error"

Enable focus following
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FocusStart(kp=50.0, kpredic=19.0, aMax=1440, vMax=180, type=0)``"
    "Description", "Enable focus following"
    "Mandatory parameters", "NULL"
    "Default parameters", "- ``kp``：Scale parameter. Default is 50.0
    - ``kpredic``：Feedforward parameter, default 19.0
    - ``aMax``：Maximum angular acceleration limit, default 1440°/s^2
    - ``vMax``：Maximum angular speed limit, default 180°/s
    - ``type``：Lock X-axis pointing (0-reference input vector; 1-level; 2- vertical), 0 by default"
    "Return Value", "Error Code Success-0 Failure- errcode"

Stop focusing following
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``FocusEnd()``"
    "Description", "Stop focusing following"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Set the focus coordinates
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.4

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetFocusPosition(pos)``"
    "Description", "Set the focus coordinates"
    "Mandatory parameters", "- ``pos``：Focus coordinate XYZ"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Robot focus following code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    p1Desc = [186.331, 487.913, 209.850, 149.030, 0.688, -114.347]
    p1Joint = [-127.876, -75.341, 115.417, -122.741, -59.820, 74.300]
    p2Desc = [69.721, 535.073, 202.882, -144.406, -14.775, -89.012]
    p2Joint = [-101.780, -69.828, 110.917, -125.740, -127.841, 74.300]
    p3Desc = [146.861, 578.426, 205.598, 175.997, -36.178, -93.437]
    p3Joint = [-112.851, -60.191, 86.566, -80.676, -97.463, 74.300]
    p4Desc = [136.284, 509.876, 225.613, 178.987, 1.372, -100.696]
    p4Joint = [-116.397, -76.281, 113.845, -128.611, -88.654, 74.299]
    p5Desc = [138.395, 505.972, 298.016, 179.134, 2.147, -101.110]
    p5Joint = [-116.814, -82.333, 109.162, -118.662, -88.585, 74.302]
    p6Desc = [105.553, 454.325, 232.017, -179.426, 0.444, -99.952]
    p6Joint = [-115.649, -84.367, 122.447, -128.663, -90.432, 74.303]
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 100, 0, 0, 0]
    robot.MoveJ(joint_pos=p1Joint,tool=0,user=0,vel=100,acc=100,ovl=100,exaxis_pos=exaxisPos,blendT=-1,offset_flag=0,offset_pos=offdese)
    robot.SetTcp4RefPoint(1)
    robot.MoveJ(joint_pos=p2Joint,tool=0,user=0,vel=100,acc=100,ovl=100,exaxis_pos=exaxisPos,blendT=-1,offset_flag=0,offset_pos=offdese)
    robot.SetTcp4RefPoint(2)
    robot.MoveJ(joint_pos=p3Joint,tool=0,user=0,vel=100,acc=100,ovl=100,exaxis_pos=exaxisPos,blendT=-1,offset_flag=0,offset_pos=offdese)
    robot.SetTcp4RefPoint(3)
    robot.MoveJ(joint_pos=p4Joint,tool=0,user=0,vel=100,acc=100,ovl=100,exaxis_pos=exaxisPos,blendT=-1,offset_flag=0,offset_pos=offdese)
    robot.SetTcp4RefPoint(4)
    rtn,coordRtn = robot.ComputeTcp4()
    print(f"4 Point ComputeTool {rtn} coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} "
          f"{coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")
    robot.SetToolCoord(1, coordRtn, 0, 0, 1, 0)
    error, p1Desc = robot.GetForwardKin(p1Joint)
    error, p2Desc = robot.GetForwardKin(p2Joint)
    error, p3Desc = robot.GetForwardKin(p3Joint)
    robot.SetFocusCalibPoint(1, p1Desc)
    robot.SetFocusCalibPoint(2, p2Desc)
    robot.SetFocusCalibPoint(3, p3Desc)
    rtn, resultPos, accuracy = robot.ComputeFocusCalib(pointNum=3)
    print(f"ComputeFocusCalib coord is {rtn} {resultPos[0]} {resultPos[1]} {resultPos[2]} accuracy is {accuracy}")
    rtn = robot.SetFocusPosition(resultPos)
    error, p5Desc = robot.GetForwardKin(p5Joint)
    error, p6Desc = robot.GetForwardKin(p6Joint)
    robot.MoveL(desc_pos=p5Desc,tool=1,user=0,vel=10,acc=100,ovl=100,blendR=-1,blendMode=0,exaxis_pos=exaxisPos,search=0,offset_flag=1,offset_pos=offdese)
    robot.MoveL(desc_pos=p6Desc,tool=1,user=0,vel=10,acc=100,ovl=100,blendR=-1,blendMode=0,exaxis_pos=exaxisPos,search=0,offset_flag=1,offset_pos=offdese)
    robot.FocusStart(50, 19, 710, 90, 0)
    robot.MoveL(desc_pos=p5Desc,tool=1,user=0,vel=10,acc=100,ovl=100,blendR=-1,blendMode=0,exaxis_pos=exaxisPos,search=0,offset_flag=1,offset_pos=offdese)
    robot.MoveL(desc_pos=p6Desc,tool=1,user=0,vel=10,acc=100,ovl=100,blendR=-1,blendMode=0,exaxis_pos=exaxisPos,search=0,offset_flag=1,offset_pos=offdese)
    robot.FocusEnd()
    robot.CloseRPC()

Joint torque sensor sensitivity calibration function is enabled
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``JointSensitivityEnable(status)``"
    "Description", "Joint torque sensor sensitivity calibration function is enabled"
    "Mandatory parameters", "- ``status``：0- closed; 1- On"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Joint torque sensor sensitivity data acquisition
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``JointSensitivityCollect()``"
    "Description", "Joint torque sensor sensitivity data acquisition"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

The sensitivity calibration results of the joint torque sensor were obtained
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``JointSensitivityCalibration()``"
    "Description", "The sensitivity calibration results of the joint torque sensor were obtained"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``calibResult``：j1~j6 Joint sensitivity [0-1]
    - ``linearityn``：j1~j6 joint linearity[0-1]"

Get Joint Torque Sensor Hysteresis Error
+++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``JointHysteresisError()``"
    "Description", "Get joint torque sensor hysteresis error"
    "Required Parameters", "None"
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure- errcode
    - ``hysteresisError``：j1~j6 joint hysteresis error"

Get Joint Torque Sensor Repeatability
+++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``JointRepeatability()``"
    "Description", "Get joint torque sensor repeatability"
    "Required Parameters", "None"
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure- errcode
    - ``repeatability``：j1~j6 joint repeatability"

Set Joint Force Sensor Parameters
+++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetAdmittanceParams(M, B, K, threshold, sensitivity, setZeroFlag)``"
    "Description", "Set joint force sensor parameters"
    "Required Parameters", "
    - ``M``：J1-J6 mass coefficient [0.001 ~ 10]
    - ``B``：J1-J6 damping coefficient [0.001 ~ 10]
    - ``K``：J1-J6 stiffness coefficient [0.001 ~ 10]
    - ``threshold``：Force control threshold, Nm
    - ``sensitivity``：Sensitivity, Nm/V, [0 ~ 10]
    - ``setZeroFlag``：Function enable flag; 0-Off; 1-On; 2-Record zero point at position 1; 3-Record zero point at position 2"
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure- errcode"

Sample code for automatic calibration of joint torque sensor sensitivity
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    robot = Robot.RPC('192.168.58.2')
    rtn = robot.JointSensitivityEnable(0)
    rtn = robot.JointSensitivityEnable(1)
    print(f"JointSensitivityEnable rtn is {rtn}")
    curJPos = [0.0] * 6
    rtn, curJPos = robot.GetActualJointPosDegree(0)
    epos = [0.0] * 4
    offset_pos = [0.0] * 6
    jointPos1 = [curJPos[0], 0.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos1 = [0.0] * 6
    rtn, descPos1 = robot.GetForwardKin(jointPos1)
    robot.MoveJ(joint_pos=jointPos1, desc_pos=descPos1, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.2)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 1 rtn is {rtn}")
    time.sleep(0.1)
    jointPos2 = [curJPos[0], -30.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos2 = [0.0] * 6
    rtn, descPos2 = robot.GetForwardKin(jointPos2)
    robot.MoveJ(joint_pos=jointPos2, desc_pos=descPos2, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 2 rtn is {rtn}")
    time.sleep(0.1)
    jointPos3 = [curJPos[0], -60.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos3 = [0.0] * 6
    rtn, descPos3 = robot.GetForwardKin(jointPos3)
    robot.MoveJ(joint_pos=jointPos3, desc_pos=descPos3, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 3 rtn is {rtn}")
    time.sleep(0.1)
    jointPos4 = [curJPos[0], -90.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos4 = [0.0] * 6
    rtn, descPos4 = robot.GetForwardKin(jointPos4)
    robot.MoveJ(joint_pos=jointPos4, desc_pos=descPos4, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 4 rtn is {rtn}")
    time.sleep(0.1)
    jointPos5 = [curJPos[0], -120.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos5 = [0.0] * 6
    rtn, descPos5 = robot.GetForwardKin(jointPos5)
    robot.MoveJ(joint_pos=jointPos5, desc_pos=descPos5, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 5 rtn is {rtn}")
    time.sleep(0.1)
    jointPos6 = [curJPos[0], -150.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos6 = [0.0] * 6
    rtn, descPos6 = robot.GetForwardKin(jointPos6)
    robot.MoveJ(joint_pos=jointPos6, desc_pos=descPos6, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 6 rtn is {rtn}")
    time.sleep(0.1)
    jointPos7 = [curJPos[0], -180.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos7 = [0.0] * 6
    rtn, descPos7 = robot.GetForwardKin(jointPos7)
    robot.MoveJ(joint_pos=jointPos7, desc_pos=descPos7, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 7 rtn is {rtn}")
    time.sleep(0.1)
    robot.MoveJ(joint_pos=jointPos6, desc_pos=descPos6, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 8 rtn is {rtn}")
    time.sleep(0.1)
    robot.MoveJ(joint_pos=jointPos5, desc_pos=descPos5, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 9 rtn is {rtn}")
    time.sleep(0.1)
    robot.MoveJ(joint_pos=jointPos4, desc_pos=descPos4, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 10 rtn is {rtn}")
    time.sleep(0.1)
    robot.MoveJ(joint_pos=jointPos3, desc_pos=descPos3, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 11 rtn is {rtn}")
    time.sleep(0.1)
    robot.MoveJ(joint_pos=jointPos2, desc_pos=descPos2, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 12 rtn is {rtn}")
    time.sleep(0.1)
    robot.MoveJ(joint_pos=jointPos1, desc_pos=descPos1, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    time.sleep(0.2)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 13 rtn is {rtn}")
    time.sleep(0.1)
    calibResult = [0.0] * 6
    linearity = [0.0] * 6
    rtn,calibResult, linearity = robot.JointSensitivityCalibration()
    print(f"JointSensitivityCalibration rtn is {rtn}")
    rtn = robot.JointSensitivityEnable(0)
    print(f"JointSensitivityEnable rtn is {rtn}")
    print(f"jointSensor Calib result is {calibResult[0]},{calibResult[1]},{calibResult[2]},{calibResult[3]},{calibResult[4]},{calibResult[5]}")
    print( f"jointSensor linearity is {linearity[0]},{linearity[1]},{linearity[2]},{linearity[3]},{linearity[4]},{linearity[5]}")
    hysteresisError = [0.0] * 6
    rtn,hysteresisError = robot.JointHysteresisError()
    print(f"JointHysteresisError result is {hysteresisError[0]},{hysteresisError[1]},{hysteresisError[2]},{hysteresisError[3]},{hysteresisError[4]},{hysteresisError[5]}")
    repeatability = [0.0] * 6
    rtn,repeatability = robot.JointRepeatability()
    print(f"JointRepeatability result is {repeatability[0]},{repeatability[1]},{repeatability[2]},{repeatability[3]},{repeatability[4]},{repeatability[5]}")
    M = [1.0] * 6
    B = [1.0] * 6
    K = [0.0] * 6
    threshold = [1.0] * 6
    setZeroFlag = 1
    rtn = robot.SetAdmittanceParams(M, B, K, threshold, calibResult, setZeroFlag)
    print(f"SetAdmittanceParams rtn is {rtn}")
    robot.CloseRPC()

The number of error frames at eight slave ports of the robot is obtained
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetSlavePortErrCounter()``"
    "Description", "The number of error frames at eight slave ports of the robot is obtained"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``inRecvErr``：Input the number of error frames received
    - ``inCRCErr``：Input the number of CRC error frames
    - ``inTransmitErr``：Input the number of forwarding error frames
    - ``inLinkErr``：Enter the number of link error frames
    - ``outRecvErr``：Outputs the number of frames received incorrectly
    - ``outCRCErr``：The number of CRC error frames is output
    - ``outTransmitErr``：Outputs the number of forwarding error frames
    - ``outLinkErr``：Output link error frames"

Slave port error frame reset
+++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``JointSensitivityEnable(slaveID)``"
    "Description", "Slave port error frame reset"
    "Mandatory parameters", "- ``slaveID``：Slave station numbers 0 to 7"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

Get sample slave port error frame code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    import threading
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    inRecvErr = [0] * 8
    inCRCErr = [0] * 8
    inTransmitErr = [0] * 8
    inLinkErr = [0] * 8
    outRecvErr = [0] * 8
    outCRCErr = [0] * 8
    outTransmitErr = [0] * 8
    outLinkErr = [0] * 8
    rtn,inRecvErr, inCRCErr, inTransmitErr, inLinkErr, outRecvErr, outCRCErr, outTransmitErr, outLinkErr = robot.GetSlavePortErrCounter()
    for i in range(8):
        if inRecvErr[i] != 0:
            print(f"inRecvErr {i} is {inRecvErr[i]}")
        if inCRCErr[i] != 0:
            print(f"inCRCErr {i} is {inCRCErr[i]}")
        if inTransmitErr[i] != 0:
            print(f"inTransmitErr {i} is {inTransmitErr[i]}")
        if inLinkErr[i] != 0:
            print(f"inLinkErr {i} is {inLinkErr[i]}")
        if outRecvErr[i] != 0:
            print(f"outRecvErr {i} is {outRecvErr[i]}")
        if outCRCErr[i] != 0:
            print(f"outCRCErr {i} is {outCRCErr[i]}")
        if outTransmitErr[i] != 0:
            print(f"outTransmitErr {i} is {outTransmitErr[i]}")
        if outLinkErr[i] != 0:
            print(f"outLinkErr {i} is {outLinkErr[i]}")
    print("others has no err!")
    for i in range(8):
        robot.SlavePortErrCounterClear(i)
    robot.CloseRPC()

Set the feed-forward coefficient of each axis speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetVelFeedForwardRatio(radio)``"
    "Description", "Set the feed-forward coefficient of each axis speed"
    "Mandatory parameters", "- ``radio``：Each axis velocity feedforward coefficient"
    "Default parameters", "NULL"
    "Return Value", "Error Code Success-0 Failure- errcode"

The velocity feedforward coefficients of each axis are obtained
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: python SDK-v2.1.7

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``GetVelFeedForwardRatio()``"
    "Description", "The velocity feedforward coefficients of each axis are obtained"
    "Mandatory parameters", "NULL"
    "Default parameters", "NULL"
    "Return Value", "- Error Code Success-0 Failure- errcode
    - ``radio``：Each axis velocity feedforward coefficient"

Robot velocity feedforward coefficient code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos:

    from fairino import Robot
    import time
    import threading
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    setRadio = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    robot.SetVelFeedForwardRatio(setRadio)
    getRadio = [0.0] * 6
    rtn,getRadio = robot.GetVelFeedForwardRatio()
    print(f"{getRadio[0]},{getRadio[1]},{getRadio[2]},{getRadio[3]},{getRadio[4]},{getRadio[5]}")
    robot.CloseRPC()

Photoelectric Sensor TCP Calibration - Compute Tool RPY
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``TCPComputeRPY(Btool, Etool, sensor, radius, dz)``"
    "Description", "Photoelectric Sensor TCP Calibration - Compute Tool RPY"
    "Required Parameters", "
    - ``Btool``: Robot Cartesian position
    - ``Etool``: Current tool coordinate values
    - ``sensor``: Current sensor coordinate values (not yet available)
    - ``radius``: Circular motion radius in mm (not yet available)
    - ``dz``: Movement distance along the negative Z-axis of the base coordinate system; when dz = 10000, the function directly returns tool RPY"
    "Default Parameters", "None"
    "Return Value", "- Error code. Success - 0, Failure - errcode"

Photoelectric Sensor TCP Calibration - Compute Tool XYZ
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``TCPComputeXYZ(select, originDirection, pos1, pos2, pos3, pos4)``"
    "Description", "Photoelectric Sensor TCP Calibration - Compute Tool XYZ"
    "Required Parameters", "
    - ``select``: 0-Compute tool TCP; 1-Compute sensor origin; 2-Compute sensor orientation; 3-Directly return tool TCP; 4-Record current workpiece coordinate system and tool coordinate system
    - ``originDirection``: 0-X direction; 1-Y direction; 2-Z direction
    - ``pos1``: Robot Cartesian position 1
    - ``pos2``: Robot Cartesian position 2
    - ``pos3``: Robot Cartesian position 3
    - ``pos4``: Robot Cartesian position 4"
    "Default Parameters", "None"
    "Return Value", "
    - Error code. Success - 0, Failure - errcode
    - Return value (returned on successful call) TCP Tool XYZ values"

Photoelectric Sensor TCP Calibration - Start Recording Flange Center Position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``TCPRecordFlangePosStart()``"
    "Description", "Photoelectric Sensor TCP Calibration - Start Recording Flange Center Position"
    "Required Parameters", "None"
    "Default Parameters", "None"
    "Return Value", "- Error code. Success - 0, Failure - errcode"

Photoelectric Sensor TCP Calibration - Stop Recording Flange Center Position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``TCPRecordFlangePosEnd()``"
    "Description", "Photoelectric Sensor TCP Calibration - Stop Recording Flange Center Position"
    "Required Parameters", "None"
    "Default Parameters", "None"
    "Return Value", "- Error code. Success - 0, Failure - errcode"

Photoelectric Sensor TCP Calibration - Get Tool Center Point Position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``TCPGetRecordFlangePos()``"
    "Description", "Photoelectric Sensor TCP Calibration - Get Tool Center Point Position"
    "Required Parameters", "None"
    "Default Parameters", "None"
    "Return Value", "
    - Error code. Success - 0, Failure - errcode
    - Return value (returned on successful call) TCP Tool center point position (x, y, z)"

Photoelectric Sensor TCP Calibration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``PhotoelectricSensorTCPCalibration(self, luaFile, offsetX)``"
    "Description", "Photoelectric Sensor TCP Calibration"
    "Required Parameters", "
    - ``luaFile``: Automatic calibration Lua program path: For  - 'FR_CalibrateTheToolTcp.lua';
    - ``offsetX``: Teaching point offset (x, y, z) in mm"
    "Default Parameters", "None"
    "Return Value", "
    - Error code. Success - 0, Failure - errcode
    - Return value (returned on successful call) TCP Tool XYZ values"

Photoelectric Sensor TCP Calibration Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos: 

    from fairino import Robot
    import time
    robot = Robot.RPC('192.168.58.2')
    offset = [10.0, 10.0, 3.0]
    TCP = [0.0] * 6
    rtn, TCP = robot.PhotoelectricSensorTCPCalibration("FR_CalibrateTheToolTcp.lua", offset)
    print(f"PhotoelectricSensorTCPCalibration rtn is {rtn},{TCP[0]},{TCP[1]},{TCP[2]},{TCP[3]},{TCP[4]},{TCP[5]}")
    robot.CloseRPC()
    return 0

Real-time Speed Setting (Command Frame, Low Latency)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetSpeedInstant(self, vel)``"
    "Description", "Real-time speed setting (command frame, low latency)"
    "Required Parameters", "
    - ``vel``: Speed percentage, range [0~100]"
    "Default Parameters", "None"
    "Return Value", "
    - Error code, 0-success; non-zero-error"

Set Real-Time Weave Offset
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetWeaveOffsetRT(self, offset)``"
    "Description", "Set real-time weave offset"
    "Required Parameters", "
    - ``offset``: Real-time offset [x,y,z,rx,ry,rz], unit [mm, °]"
    "Default Parameters", "None"
    "Return Value", "
    - Error code, 0-success; non-zero-error"

Weave Speed and Offset Test Code Example    
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: python
    :linenos: 

    from fairino import Robot
    import time
    import threading


    def main():
        robot = Robot.RPC('192.168.58.2')
        time.sleep(0.5)  

        epos = [0.0, 0.0, 0.0, 0.0]
        offset_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        j1 = [5.027, -84.331, -75.139, -103.690, 86.379, 20.794]
        d1 = [324.752, -83.339, 366.314, -172.321, -0.936, -106.047]

        j2 = [-35.335, -117.598, -57.174, -95.234, 90.001, -19.560]
        d2 = [324.999, -355.439, 260.000, 179.995, 0.003, -105.775]

        j3 = [59.787, -117.594, -57.183, -95.222, 90.006, 75.562]
        d3 = [324.998, 355.441, 260.002, 179.995, 0.003, -105.775]

        rtn = 0

        print("\nStep 1: MoveJ to start point")
        rtn = robot.MoveJ(joint_pos=j1, desc_pos=d1, tool=1, user=0, vel=100, acc=100, ovl=50,
                        exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
        print(f"  MoveJ(j1) rtn={rtn}")
        time.sleep(0.5)

        print("\nStep 2: MoveJ to weave entry point")
        rtn = robot.MoveJ(joint_pos=j2, desc_pos=d2, tool=1, user=0, vel=100, acc=100, ovl=50,
                        exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
        print(f"  MoveJ(j2) rtn={rtn}")
        time.sleep(0.5)

        print("\nStep 3: WeaveStart + MoveL in background thread")
        robot.WeaveStart(0)

        weave_running = True
        move_rtn = [0]  

        def weave_move_thread():
            rtn_val = robot.MoveL(joint_pos=j3, desc_pos=d3, tool=1, user=0, vel=100, acc=100, ovl=5,
                                blendR=-1, offset_flag=0, exaxis_pos=epos, search=0,
                                offset_pos=offset_pos, config=5, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
            print(f"  MoveL(weave) thread finished, rtn={rtn_val}")
            move_rtn[0] = rtn_val
            nonlocal weave_running
            weave_running = False

        weave_thread = threading.Thread(target=weave_move_thread)
        weave_thread.daemon = True
        weave_thread.start()
        time.sleep(0.5)  

        print("\nStep 4: SetSpeed test during weaving")
        speed_values = [20, 50, 80, 30, 60, 10]
        for speed in speed_values:
            if not weave_running:
                break
            rtn = robot.SetSpeedInstant(speed)
            print(f"  SetSpeed({speed}) -> rtn={rtn}")
            # rtn, pkg = robot.GetRobotRealTimeState()
            # print(f"target_TCP_CmpSpeed: [{', '.join([f'{x:.3f}' for x in pkg.target_TCP_CmpSpeed])}]")
            time.sleep(5)

        time.sleep(5)

        print("\nStep 5: SetWeaveOffsetRT test (50 iterations, delta=0.1)")
        accum_offset = 0.0
        for i in range(50):
            if not weave_running:
                break
            accum_offset += 1
            weave_offset = [0.0, 0.0, accum_offset, 0.0, 0.0, 0.0]
            rtn = robot.SetWeaveOffsetRT(weave_offset)
            rtn, pkg = robot.GetRobotRealTimeState()
            print(f"  [{i+1}/50] SetWeaveOffsetRT(x={accum_offset:.1f}) -> rtn={rtn}, "
                f"TCP_pos=({pkg.tl_cur_pos[0]:.2f},{pkg.tl_cur_pos[1]:.2f},{pkg.tl_cur_pos[2]:.2f})")
            time.sleep(0.1)

        print("\nStep 6: Wait for weave MoveL, then WeaveEnd")
        weave_thread.join()
        robot.WeaveEnd(0)
        time.sleep(0.5)

        print("\nStep 7: MoveL back to start")
        rtn = robot.MoveL(joint_pos=j1, desc_pos=d1, tool=1, user=0, vel=100, acc=100, ovl=50,
                        blendR=-1, offset_flag=0, exaxis_pos=epos, search=0,
                        offset_pos=offset_pos, config=50, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
        print(f"  MoveL(back) rtn={rtn}")

        rtn, pkg = robot.GetRobotRealTimeState()
        print(f"\n  Final robot state: main_code={pkg.main_code}, sub_code={pkg.sub_code}")
        
        robot.CloseRPC()


    if __name__ == "__main__":
        main()    