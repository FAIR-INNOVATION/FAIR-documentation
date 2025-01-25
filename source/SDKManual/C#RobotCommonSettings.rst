Common Robot Settings
====================================================

.. toctree:: 
    :maxdepth: 5

Setting the global speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the global speed
    * @param [in] vel velocity percentage, range [0~100]
    * @return error code
    */
    int SetSpeed(int vel). 

Setting system variable values
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief Setting system variable values
    * @param [in] id Variable number in the range [1~20].
    * @param [in] value Variable value
    * @return error code
    */
    int SetSysVarValue(int id, double value). 

Setting Tool Reference Points - Six Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Setting Tool Reference Points -- Six-Point Approach 
    * @param [in] point_num point_num, range [1~6] 
    * @return error code 
    */ 
    int SetToolPoint(int point_num). 

Calculation Tool Coordinate System - Six Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Calculate the tool coordinate system
    * @param [out] tcp_pose tool coordinate system
    * @return error code 
    */ 
    int ComputeTool(ref DescPose tcp_pose). 

Setting Tool Reference Points - Four Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Setting Tool Reference Points -- Four-Point Approach 
    * @param [in] point_num point_number, range [1~4] 
    * @return error code 
    */ 
    int SetTcp4RefPoint(int point_num);

Calculation Tool Coordinate System - Four Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Calculate the tool coordinate system
    * @param [out] tcp_pose tool coordinate system
    * @return error code 
    */ 
    int ComputeTcp4(ref DescPose tcp_pose).

Setting the tool coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the tool coordinate system
    * @param [in] id Coordinate system number, range [0~14]
    * @param [in] coord Tool centre point relative to end flange centre attitude
    * @param [in] type 0-tool coordinate system, 1-sensor coordinate system
    * @param [in] install install position, 0 - robot end, 1 - robot exterior
    * param [in] toolID toolID
    * @param [in] loadNum loadNumber
    * @return error code
    */
    int SetToolCoord(int id, DescPose coord, int type, int install,int toolID, int loadNum);  

Setting up a list of tool coordinate systems
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief sets up a list of tool coordinate systems
    * @param [in] id Coordinate system number, range [0~14]
    * @param [in] coord Tool centre point relative to end flange centre attitude
    * @param [in] type 0-tool coordinate system, 1-sensor coordinate system
    * @param [in] install install position, 0 - robot end, 1 - robot exterior
    * @param [in] loadNum loadNumber
    * @return error code
    */
    int SetToolList(int id, DescPose coord, int type, int install, int loadNum);  

Setting the external tool coordinate reference point - three-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Setting external tool reference points -- three-point method 
    * @param [in] point_num point_num, range [1~3] 
    * @return error code 
    */ 
    int SetExTCPPoint(int point_num). 

Calculation of the external tool coordinate system - three-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
    
    /** 
    * :: @brief Calculate external tool coordinate system -- three-point method
    * @param [out] tcp_pose external tool coordinate system
    * @return error code 
    */ 
    int ComputeExTCF(ref DescPose tcp_pose). 

Setting the external tool coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the external tool coordinate system 
    * @param [in] id Coordinate system number, range [0~14] 
    * @param [in] etcp Tool centre point relative to end flange centre attitude 
    * @param [in] etool TBD 
    * @return error code 
    */
    int SetExToolCoord(int id, DescPose etcp, DescPose etool);; 

Setting up a list of external tool coordinate systems
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting up a list of external tool coordinate systems
    * @param [in] id Coordinate system number, range [0~14] 
    * @param [in] etcp Tool centre point relative to end flange centre attitude
    * @param [in] etool TBD
    * @return error code
    */
    int SetExToolList(int id, DescPose etcp, DescPose etool); 

Setting the reference point of the workpiece coordinate system - three-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Setting the workpiece reference point -- three-point method 
    * @param [in] point_num point_num, range [1~3]  
    * @return error code 
    */ 
    int SetWObjCoordPoint(int point_num). 

Calculation of the workpiece coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Calculate workpiece coordinate system
    * @param [in] method Calculation method 0: origin-x-axis-z-axis 1: origin-x-axis-xy plane
    * @param [in] refFrame reference coordinate system
    * @param [out] wobj_pose Workpiece coordinate system
    * @return error code
    */
    int ComputeWObjCoord(int method, int refFrame, ref DescPose wobj_pose); 

Setting the workpiece coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the workpiece coordinate system
    * @param [in] id coordinate system number, range [1-15]
    * @param [in] coord Workpiece coordinate system relative to end flange centre attitude
    * @param [in] refFrame reference coordinate system
    * @return error code
    */
    int SetWObjCoord(int id, DescPose coord, int refFrame);

Setting the list of workpiece coordinate systems
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting up a list of workpiece coordinate systems
    * @param [in] id Coordinate system number, range [0~14] 
    * @param [in] coord Workpiece coordinate system relative to end flange centre attitude
    * @param [in] refFrame reference coordinate system
    * @return error code
    */  
    int SetWObjList(int id, DescPose coord, int refFrame);

Setting the end load weight
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the end load weight
    * @param [in] loadNum loadNumber
    * @param [in] weight load weight in kg
    * @return Error code
    */
    int SetLoadWeight(int loadNum, float weight)

Setting the end-load centre-of-mass coordinates
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting end-load centre-of-mass coordinates
    * @param [in] coord coordinates of centre of mass in mm
    * @return error code
    */
    int SetLoadCoord(DescTran coord). 

Setting the robot installation method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting up the robot installation method
    * @param [in] install Installation method, 0-front, 1-side, 2-inverted
    * @return error code
    */
    int SetRobotInstallPos(byte install). 

Setting the robot mounting angle
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting the robot mounting angle, free mounting
    * @param [in] yangle Tilt angle
    * @param [in] zangle rotation angle
    * @return error code
    */
    int SetRobotInstallAngle(double yangle, double zangle); 

Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnCommonSets_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        int i;
        double value = 0;
        int id;
        int type;
        int install.

        DescTran coord = new DescTran();
        DescPose t_coord, etcp, etool, w_coord.
        t_coord = new DescPose();
        etcp = new DescPose();
        w_coord = new DescPose();

        robot.SetSpeed(20);

        for (i = 1; i < 21; i++)
        {
            robot.SetSysVarValue(i, (float)(i + 0.5));
            robot.WaitMs(100);
        }

        for (i = 1; i < 21; i++)
        {
            robot.GetSysVarValue(i, ref value);
            Console.WriteLine($"sys value : {value}");
        }

        robot.SetLoadWeight((float)2.5);
        coord.x = 3.0;
        coord.y = 4.0;
        coord.z = 5.0;
        robot.SetLoadCoord(coord);
                
        id = 3;
        t_coord.tran.x = 1.0;
        t_coord.tran.y = 2.0;
        t_coord.tran.z = 300.0;
        t_coord.rpy.rx = 4.0;
        t_coord.rpy.ry = 5.0;
        t_coord.rpy.rz = 6.0;
        type = 0;
        install = 0;

        int rtn1 = -1;
        int rtn2 = -1;
        rtn1 = robot.SetToolCoord(id, t_coord, type, install);
        rtn2 = robot.SetToolList(id, t_coord, type, install);
        Console.WriteLine($"set tool coord result {rtn1}, set tool list rtn{rtn2}");
            
        etcp.tran.x = 1.0;
        etcp.tran.y = 2.0;
        etcp.tran.z = 3.0;
        etcp.rpy.rx = 4.0;
        etcp.rpy.ry = 5.0;
        etcp.rpy.rz = 6.0;
        etool.tran.x = 11.0;
        etool.tran.y = 22.0;
        etool.tran.z = 330.0;
        etool.rpy.rx = 44.0;
        etool.rpy.ry = 55.0;
        etool.rpy.rz = 66.0;
        id = 5;
        robot.SetExToolCoord(id, etcp, etool);
        robot.SetExToolList(id, etcp, etool);

        w_coord.tran.x = 110.0;
        w_coord.tran.y = 12.0;
        w_coord.tran.z = 13.0;
        w_coord.rpy.rx = 14.0;
        w_coord.rpy.ry = 15.0;
        w_coord.rpy.rz = 16.0;
        id = 12;
        robot.SetWObjCoord(id, w_coord);
        //robot.SetWObjList(id, w_coord);

        double yangle = 0, zangle = 0;
        robot.SetRobotInstallPos(1);//Side-loading
        robot.SetRobotInstallAngle(15.0, 25.0);
        Thread.Sleep(1000);
        robot.GetRobotInstallAngle(ref yangle, ref zangle);
        Console.WriteLine($"yangle {yangle} zangle {zangle}");
        robot.SetRobotInstallAngle(10.0, 10.0);
        Thread.Sleep(1000);
        robot.GetRobotInstallAngle(ref yangle, ref zangle);
        Console.WriteLine($"yangle {yangle} zangle {zangle}");
    }

Waiting for a specified time
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Waiting for specified time
    * @param [in] t_ms Unit ms
    * @return error code
    */
    int WaitMs(int t_ms).

Setting robot acceleration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Setting robot acceleration
    * @param [in] acc Robot acceleration percentage
    * @return error code
    */
    int SetOaccScale(double acc)

Calculate the tool coordinate system from the point information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Calculate tool coordinate system from point information
    * @param [in] method Calculation method; 0 - four-point method; 1 - six-point method
    * @param [in] pos Joint position group, array length is 4 for 4-point method, 6 for 6-point method.
    * @return Error code
    */

    int ComputeToolCoordWithPoints(int method, JointPos[] pos, ref DescPose coordRtn)

Calculate the workpiece coordinate system from the point information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Calculate the workpiece coordinate system from the point information
    * @param [in] method Calculation method; 0: origin-x-axis-z-axis 1: origin-x-axis-xy plane
    * @param [in] pos three TCP position groups
    * @param [in] refFrame reference coordinate system
    * @return Error code
    */
    int ComputeWObjCoordWithPoints(int method, DescPose[] pos, int refFrame, ref DescPose coordRtn)

code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void TestTCP_Click(object sender, EventArgs e)
    {
      DescPose p1Desc = new DescPose(-394.073, -276.405, 399.451, -133.692, 7.657, -139.047);
      JointPos p1Joint = new JointPos(15.234, -88.178, 96.583, -68.314, -52.303, -122.926);

      DescPose p2Desc = new DescPose( -187.141, -444.908, 432.425, 148.662, 15.483, -90.637);
      JointPos p2Joint = new JointPos(61.796, -91.959, 101.693, -102.417, -124.511, -122.767);

      DescPose p3Desc = new DescPose(-368.695, -485.023, 426.640, -162.588, 31.433, -97.036);
      JointPos p3Joint = new JointPos(43.896, -64.590, 60.087, -50.269, -94.663, -122.652);

      DescPose p4Desc = new DescPose(-291.069, -376.976, 467.560, -179.272, -2.326, -107.757);
      JointPos p4Joint = new JointPos(39.559, -94.731, 96.307, -93.141, -88.131, -122.673);

      DescPose p5Desc = new DescPose(-284.140, -488.041, 478.579, 179.785, -1.396, -98.030);
      JointPos p5Joint = new JointPos(49.283, -82.423, 81.993, -90.861, -89.427, -122.678);

      DescPose p6Desc = new DescPose(-296.307, -385.991, 484.492, -178.637, -0.057, -107.059);
      JointPos p6Joint = new JointPos(40.141, -92.742, 91.410, -87.978, -88.824, -122.808);

      ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
      DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

      JointPos[] posJ = new JointPos[6]{ p1Joint, p2Joint, p3Joint, p4Joint, p5Joint, p6Joint };
      DescPose coordRtn = new DescPose(0, 0, 0, 0, 0, 0); 
      int rtn = robot.ComputeToolCoordWithPoints(0, posJ,ref coordRtn);
      Console.WriteLine("ComputeToolCoordWithPoints {0}  coord is {1} {2} {3} {4} {5} {6}", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);


      robot.MoveJ(p1Joint, p1Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
      robot.SetTcp4RefPoint(1);
      robot.MoveJ(p2Joint, p2Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
      robot.SetTcp4RefPoint(2);
      robot.MoveJ(p3Joint, p3Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
      robot.SetTcp4RefPoint(3);
      robot.MoveJ(p4Joint, p4Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
      robot.SetTcp4RefPoint(4);
      robot.ComputeTcp4(ref coordRtn);
      Console.WriteLine("ComputeTcp4 {0}  coord is {1} {2} {3} {4} {5} {6}", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
      //robot.MoveJ(p5Joint, p5Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
      //robot.MoveJ(p6Joint, p6Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
    }
