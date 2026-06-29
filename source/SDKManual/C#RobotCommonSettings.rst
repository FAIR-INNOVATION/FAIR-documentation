Common Robot Settings
===================================================

.. toctree:: 
    :maxdepth: 5
 
Setting the tool reference point - six-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
    
     /** 
     * @brief Setting the tool reference point - the six-point method. 
     * @param [in] point_num point_num, range [1~6]. 
     * @return Error code 
     */ 
     int SetToolPoint(int point_num). 
 
Calculating the Tool Coordinate System - Six Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /** 
     * @brief Calculate the tool coordinate system
     * @param [out] tcp_pose tool coordinate system
     * @return Error code. 
     */ 
     int ComputeTool(ref DescPose tcp_pose). 
  
Setting the tool reference point - four-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /** 
     * @brief Setting the tool reference point - four-point method. 
     * @param [in] point_num point_num, range [1~4]. 
     * @return Error code 
     */ 
     int SetTcp4RefPoint(int point_num).
 
Calculate the tool coordinate system - four-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
      /** 
     * @brief Calculate the tool coordinate system
     * @param [out] tcp_pose tool coordinate system
     * @return Error code. 
     */ 
     int ComputeTcp4(ref DescPose tcp_pose).
 
Set the tool coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set the tool coordinate system.
     * @param [in] id Coordinate system number, range [0~14].
     * @param [in] coord tool center with respect to end-flange center.
     * @param [in] type 0-tool coordinate system, 1-sensor coordinate system
     * @param [in] install mounting position, 0-robot end, 1-robot exterior
     * param [in] toolID toolID
     * @param [in] loadNum loadNumber
     * @return ErrorCode
     */
     int SetToolCoord(int id, DescPose coord, int type, int install,int toolID, int loadNum);
 
Calculate the tool coordinate system from the point information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Calculate tool coordinate system from point information.
     * @param [in] method Calculation method; 0 - four-point method; 1 - six-point method
     * @param [in] pos Joint position group, array length is 4 for 4-point method, 6 for 6-point method.
     * @return Error code
     */
     int ComputeToolCoordWithPoints(int method, JointPos[] pos, ref DescPose coordRtn)  
 
Set the list of tool coordinate systems
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Setting up a list of tool coordinate systems.
     * @param [in] id Coordinate system number, range [0~14].
     * @param [in] coord Tool center point relative to end flange center position.
     * @param [in] type 0-tool coordinate system, 1-sensor coordinate system
     * @param [in] install mounting position, 0-robot end, 1-robot exterior
     * @param [in] loadNum loadNumber
     * @return Error code
     */
     int SetToolList(int id, DescPose coord, int type, int install, int loadNum);  
 
Get the current tool coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     /**
     * @brief Get the current tool coordinate system.
     * @param [in] flag 0-blocking, 1-non-blocking
     * @param [out] desc_pos Tool coordinate system position.
     * @return Error code.
     */
     int GetTCPOffset(byte flag, ref DescPose desc_pos); 
 
Robot tool coordinate system operation code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     private void button18_Click(object sender, EventArgs e)
    {
        DescPose p1Desc = new DescPose(186.331f, 487.913f, 209.850f, 149.030f, 0.688f, -114.347f);
        JointPos p1Joint = new JointPos(-127.876f, -75.341f, 115.417f, -122.741f, -59.820f, 74.300f);

        DescPose p2Desc = new DescPose(69.721f, 535.073f, 202.882f, -144.406f, -14.775f, -89.012f);
        JointPos p2Joint = new JointPos(-101.780f, -69.828f, 110.917f, -125.740f, -127.841f, 74.300f);

        DescPose p3Desc = new DescPose(146.861f, 578.426f, 205.598f, 175.997f, -36.178f, -93.437f);
        JointPos p3Joint = new JointPos(-112.851f, -60.191f, 86.566f, -80.676f, -97.463f, 74.300f);

        DescPose p4Desc = new DescPose(136.284f, 509.876f, 225.613f, 178.987f, 1.372f, -100.696f);
        JointPos p4Joint = new JointPos(-116.397f, -76.281f, 113.845f, -128.611f, -88.654f, 74.299f);

        DescPose p5Desc = new DescPose(138.395f, 505.972f, 298.016f, 179.134f, 2.147f, -101.110f);
        JointPos p5Joint = new JointPos(-116.814f, -82.333f, 109.162f, -118.662f, -88.585f, 74.302f);

        DescPose p6Desc = new DescPose(105.553f, 454.325f, 232.017f, -179.426f, 0.444f, -99.952f);
        JointPos p6Joint = new JointPos(-115.649f, -84.367f, 122.447f, -128.663f, -90.432f, 74.303f);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

        JointPos[] posJ = new JointPos[] { p1Joint, p2Joint, p3Joint, p4Joint, p5Joint, p6Joint };
        DescPose coordRtn = new DescPose();
        int rtn = robot.ComputeToolCoordWithPoints(1, posJ, ref coordRtn);
        Console.WriteLine($"ComputeToolCoordWithPoints    {rtn}  coord is {coordRtn.tran.x} {coordRtn.tran.y} {coordRtn.tran.z} {coordRtn.rpy.rx} {coordRtn.rpy.ry} {coordRtn.rpy.rz}");

        robot.MoveJ( p1Joint,  p1Desc, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetToolPoint(1);
        robot.MoveJ( p2Joint,  p2Desc, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetToolPoint(2);
        robot.MoveJ( p3Joint,  p3Desc, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetToolPoint(3);
        robot.MoveJ( p4Joint,  p4Desc, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetToolPoint(4);
        robot.MoveJ( p5Joint,  p5Desc, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetToolPoint(5);
        robot.MoveJ( p6Joint,  p6Desc, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetToolPoint(6);
        rtn = robot.ComputeTool(ref coordRtn);
        Console.WriteLine($"6 Point ComputeTool        {rtn}  coord is {coordRtn.tran.x} {coordRtn.tran.y} {coordRtn.tran.z} {coordRtn.rpy.rx} {coordRtn.rpy.ry} {coordRtn.rpy.rz}");
        robot.SetToolList(1,  coordRtn, 0, 0, 0);

        robot.MoveJ( p1Joint,  p1Desc, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetTcp4RefPoint(1);
        robot.MoveJ( p2Joint,  p2Desc, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetTcp4RefPoint(2);
        robot.MoveJ( p3Joint,  p3Desc, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetTcp4RefPoint(3);
        robot.MoveJ( p4Joint,  p4Desc, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetTcp4RefPoint(4);
        rtn = robot.ComputeTcp4(ref coordRtn);
        Console.WriteLine($"4 Point ComputeTool        {rtn}  coord is {coordRtn.tran.x} {coordRtn.tran.y} {coordRtn.tran.z} {coordRtn.rpy.rx} {coordRtn.rpy.ry} {coordRtn.rpy.rz}");

        robot.SetToolCoord(2, coordRtn, 0, 0, 1, 0);

        DescPose getCoord = new DescPose();
        rtn = robot.GetTCPOffset(0, ref getCoord);
        Console.WriteLine($"GetTCPOffset    {rtn}  coord is {coordRtn.tran.x} {coordRtn.tran.y} {coordRtn.tran.z} {coordRtn.rpy.rx} {coordRtn.rpy.ry} {coordRtn.rpy.rz}");
    }
 
Setting the external tool coordinate reference point - three-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /** 
     * @brief Setting the external tool reference point - the three-point method. 
     * @param [in] point_num point_num, range [1~3]. 
     * @return Error code 
     */ 
     int SetExTCPPoint(int point_num). 
 
Calculating an external tool coordinate system - three-point method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
     
     /** 
     * @brief Calculate external tool coordinate system - three-point method.
     * @param [out] tcp_pose external tool coordinate system
     * @return Error code. 
     */ 
     int ComputeExTCF(ref DescPose tcp_pose). 
 
Set the external tool coordinate system
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set the external tool coordinate system. 
     * @param [in] id Coordinate system number, range [0~14]. 
     * @param [in] etcp Tool center point relative to end-flange center position. 
     * @param [in] etool To be determined. 
     * @return Error code 
     */
     int SetExToolCoord(int id, DescPose etcp, DescPose etool); 
 
Setting up a list of external tool coordinate systems
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set the list of external tool coordinate systems.
     * @param [in] id Coordinate system number, range [0~14]. 
     * @param [in] etcp tool center point relative to end-flange center position
     * @param [in] etool pending
     * @return Error code
     */
     int SetExToolList(int id, DescPose etcp, DescPose etool); 
 
Calculate the workpiece coordinate system from the point information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Calculate the workpiece coordinate system from point information.
     * @param [in] method Calculation method; 0: origin-x-axis-z-axis 1: origin-x-axis-xy plane
     * @param [in] pos Three TCP position groups
     * @param [in] refFrame reference coordinate system
     * @return Error code
     */
     int ComputeWObjCoordWithPoints(int method, DescPose[] pos, int refFrame, ref DescPose coordRtn)
 
Sample code for manipulating the robot's external tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     private void button20_Click(object sender, EventArgs e)
    {
        DescPose p1Desc = new DescPose(-89.606f, 779.517f, 193.516f, 178.000f, 0.476f, -92.484f);
        JointPos p1Joint = new JointPos(-108.145f, -50.137f, 85.818f, -125.599f, -87.946f, 74.329f);

        DescPose p2Desc = new DescPose(-24.656f, 850.384f, 191.361f, 177.079f, -2.058f, -95.355f);
        JointPos p2Joint = new JointPos(-111.024f, -41.538f, 69.222f, -114.913f, -87.743f, 74.329f);

        DescPose p3Desc = new DescPose(-99.813f, 766.661f, 241.878f, -176.817f, 1.917f, -91.604f);
        JointPos p3Joint = new JointPos(-107.266f, -56.116f, 85.971f, -122.560f, -92.548f, 74.331f);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

        DescPose[] posTCP = new DescPose[] { p1Desc, p2Desc, p3Desc };
        DescPose coordRtn = new DescPose();

        robot.MoveJ( p1Joint,  p1Desc, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetExTCPPoint(1);
        robot.MoveJ( p2Joint,  p2Desc, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetExTCPPoint(2);
        robot.MoveJ( p3Joint,  p3Desc, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetExTCPPoint(3);
        int rtn = robot.ComputeExTCF(ref coordRtn);
        Console.WriteLine($"ComputeExTCF                   {rtn}  coord is {coordRtn.tran.x} {coordRtn.tran.y} {coordRtn.tran.z} {coordRtn.rpy.rx} {coordRtn.rpy.ry} {coordRtn.rpy.rz}");

        robot.SetExToolCoord(1,  coordRtn,  offdese);
        robot.SetExToolList(1,  coordRtn,  offdese);
    }
 
Setting the reference point of the workpiece coordinate system - three-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /** 
     * @brief Setting the reference point of a workpiece - the three-point method. 
     * @param [in] point_num point_num, range [1~3].  
     * @return Error code 
     */ 
     int SetWObjCoordPoint(int point_num). 
 
Calculate the work coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Calculate the workpiece coordinate system.
     * @param [in] method Calculation method 0: origin-x-axis-z-axis 1: origin-x-axis-xy-plane
     * @param [in] refFrame reference coordinate system
     * @param [out] wobj_pose Workpiece coordinate system
     * @return Error code
     */
     int ComputeWObjCoord(int method, int refFrame, ref DescPose wobj_pose); 
 
Set the workpiece coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set the workpiece coordinate system.
     * @param [in] id Coordinate system number, range [1~15].
     * @param [in] coord Workpiece coordinate system relative to end flange center.
     * @param [in] refFrame The reference coordinate system.
     * @return Error code
     */
     int SetWObjCoord(int id, DescPose coord, int refFrame);
 
Set the list of workpiece coordinate systems
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set the list of workpiece coordinate systems.
     * @param [in] id coordinate system number, range [0~14] 
     * @param [in] coord Workpiece coordinate system relative to the center of the end flange.
     * @param [in] refFrame The reference coordinate system.
     * @return Error code
     */    
     int SetWObjList(int id, DescPose coord, int refFrame);
 
Get the current workpiece coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Get the current workpiece coordinate system.
     * @param [in] flag 0-blocking, 1-non-blocking
     * @param [out] desc_pos Workpiece coordinate system position.
     * @return Error code.
     */   
     int GetWObjOffset(byte flag, ref DescPose desc_pos); 
 
Robot workpiece coordinate system operation code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    private void button19_Click(object sender, EventArgs e)
    {
        DescPose p1Desc = new DescPose(-89.606, 779.517, 193.516, 178.000, 0.476, -92.484);
        JointPos p1Joint = new JointPos(-108.145, -50.137, 85.818, -125.599, -87.946, 74.329);

        DescPose p2Desc = new DescPose(-24.656, 850.384, 191.361, 177.079, -2.058, -95.355);
        JointPos p2Joint = new JointPos(-111.024, -41.538, 69.222, -114.913, -87.743, 74.329);

        DescPose p3Desc = new DescPose(-99.813, 766.661, 241.878, -176.817, 1.917, -91.604);
        JointPos p3Joint = new JointPos(-107.266, -56.116, 85.971, -122.560, -92.548, 74.331);

        robot.GetForwardKin(p1Joint,ref p1Desc);
        robot.GetForwardKin(p2Joint,ref p2Desc);
        robot.GetForwardKin(p3Joint, ref p3Desc);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

        DescPose[] posTCP = new DescPose[] { p1Desc, p2Desc, p3Desc };
        DescPose coordRtn = new DescPose();
        int rtn = robot.ComputeWObjCoordWithPoints(1, posTCP, 0, ref coordRtn);
        Console.WriteLine($"ComputeWObjCoordWithPoints    {rtn}  coord is {coordRtn.tran.x} {coordRtn.tran.y} {coordRtn.tran.z} {coordRtn.rpy.rx} {coordRtn.rpy.ry} {coordRtn.rpy.rz}");

        robot.MoveJ( p1Joint,  p1Desc, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetWObjCoordPoint(1);
        robot.MoveJ( p2Joint,  p2Desc, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetWObjCoordPoint(2);
        robot.MoveJ( p3Joint,  p3Desc, 1, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.SetWObjCoordPoint(3);
        rtn = robot.ComputeWObjCoord(1, 0, ref coordRtn);
        Console.WriteLine($"ComputeWObjCoord                   {rtn}  coord is {coordRtn.tran.x} {coordRtn.tran.y} {coordRtn.tran.z} {coordRtn.rpy.rx} {coordRtn.rpy.ry} {coordRtn.rpy.rz}");

        robot.SetWObjCoord(1,  coordRtn, 0);
        robot.SetWObjList(1,  coordRtn, 0);

        DescPose getWobjDesc = new DescPose();
        rtn = robot.GetWObjOffset(0, ref getWobjDesc);
        Console.WriteLine($"GetWObjOffset                   {rtn}  coord is {coordRtn.tran.x} {coordRtn.tran.y} {coordRtn.tran.z} {coordRtn.rpy.rx} {coordRtn.rpy.ry} {coordRtn.rpy.rz}");   
    } 
 
Setting the global speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set the global speed.
     * @param [in] vel Speed percentage, range [0~100].
     * @return Error code.
     */
     int SetSpeed(int vel). 
 
Set the robot acceleration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set robot acceleration
     * @param [in] acc robot acceleration percentage
     * @return Error code.
     */
     int SetOaccScale(double acc)
 
Get the default robot speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Get the default speed of the robot.
     * @param [out] vel velocity in mm/s
     * @return Error code.
     */   
     int GetDefaultTransVel(ref double vel). 
 
Set the end load weight
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set the end load weight.
     * @param [in] loadNum loadNumber
     * @param [in] weight load weight in kg
     * @return Error code.
     */
     int SetLoadWeight(int loadNum, float weight)
 
Set the center of mass of the end load
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set end-load center-of-mass coordinates.
     * @param [in] coord center of mass in mm.
     * @return Error code.
     */
     int SetLoadCoord(DescTran coord). 
 
Get the weight of the current load
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Get the weight of the current load
     * @param [in] flag 0-blocking, 1-non-blocking
     * @param [out] weight weight of the load in kg
     * @return Error code.
     */
     int GetTargetPayload(byte flag, ref double weight); 
 
Get the center of mass of the current load
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Get the center of mass of the current load.
     * @param [in] flag 0-blocking, 1-non-blocking
     * @param [out] cog load center of mass in mm
     * @return Error code.
     */   
     int GetTargetPayloadCog(byte flag, ref DescTran cog);
 
Set the robot mounting method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set the robot installation method.
     * @param [in] install Installation method, 0-front, 1-side, 2-backwards.
     * @return Error code.
     */
     int SetRobotInstallPos(byte install). 
 
Set the robot installation angle
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Setting the robot mounting angle for free mounting.
     * @param [in] yangle tilt angle
     * @param [in] zangle angle of rotation.
     * @return Error code.
     */
     int SetRobotInstallAngle(double yangle, double zangle); 
 
Get the robot installation angle
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Get the robot mounting angle.
     * @param [out] yangle tilt angle
     * @param [out] zangle angle of rotation.
     * @return Error code.
     */
     int GetRobotInstallAngle(ref double yangle, ref double zangle); 
 
Sets the value of the system variable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Setting system variable values
     * @param [in] id Variable number, range [1~20].
     * @param [in] value Variable number, range [1~20].
     * @return Error code.
     */
     int SetSysVarValue(int id, double value); 
 
Get the value of a system variable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Get the value of a system variable.
     * @param [in] id System variable number, in the range [1~20].
     * @param [out] value System variable value.
     * @return Error code.
     */
     int GetSysVarValue(int id, ref double value); 
 
Sample code for common robot settings
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    private void button21_Click(object sender, EventArgs e)
    {
        for (int i = 1; i < 100; i++)
        {
            robot.SetSpeed(i);
            robot.SetOaccScale(i);
            Thread.Sleep(30);
        }

        double defaultVel = 0.0f;
        robot.GetDefaultTransVel(ref defaultVel);
        Console.WriteLine($"GetDefaultTransVel is {defaultVel}");

        for (int i = 1; i < 21; i++)
        {
            robot.SetSysVarValue(i, i + 0.5f);
            Thread.Sleep(100);
        }

        for (int i = 1; i < 21; i++)
        {
            double value = 0;
            robot.GetSysVarValue(i, ref value);
            Console.WriteLine($"sys value  {i} is :{value}");
            Thread.Sleep(100);
        }

        robot.SetLoadWeight(0, 2.5f);

        DescTran loadCoord = new DescTran();
        loadCoord.x = 3.0f;
        loadCoord.y = 4.0f;
        loadCoord.z = 5.0f;
        robot.SetLoadCoord( loadCoord);

        Thread.Sleep(1000);

        double getLoad = 0.0f;
        robot.GetTargetPayload(0, ref getLoad);

        DescTran getLoadTran = new DescTran();
        robot.GetTargetPayloadCog(0, ref getLoadTran);
        Console.WriteLine($"get load is {getLoad}; get load cog is {getLoadTran.x} {getLoadTran.y} {getLoadTran.z}");

        robot.SetRobotInstallPos(0);
        robot.SetRobotInstallAngle(15.0f, 25.0f);

        double anglex = 0.0f;
        double angley = 0.0f;
        robot.GetRobotInstallAngle(ref anglex, ref angley);
        Console.WriteLine($"GetRobotInstallAngle x:  {anglex};  y:  {angley}");
    }
 
Joint friction compensation switch
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /** 
     * @brief Joint friction compensation switch. 
     * @param [in] state 0-off, 1-on 
     * @return Error code. 
     */ 
     int FrictionCompensationOnOff(byte state). 
 
Sets the joint friction compensation factor - proper
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set the joint friction compensation coefficients-front-loading
     * @param [in] coeff Six joint compensation coefficients, range [0~1].
     * @return error code
     */
     int SetFrictionValue_level(double[] coeff).
 
Set joint friction compensation coefficients - side mount
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set joint friction compensation coefficients-side-loading
     * @param [in] coeff Six joint compensation coefficients, range [0~1].
     * @return error code
     */
     int SetFrictionValue_wall(double[] coeff). 
 
Set joint friction compensation coefficients - inverted
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set joint friction compensation coefficients-flip-flop
     * @param [in] coeff Six joint compensation coefficients, range [0~1].
     * @return Error code
     */
     int SetFrictionValue_ceiling(double[] coeff).
 
Set joint friction compensation coefficients - free mounting
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Set joint friction compensation coefficients - free-mounting
     * @param [in] coeff Six joint compensation coefficients, range [0 to 1].
     * @return Error code.
     */
     int SetFrictionValue_freedom(double[] coeff);
        
Robot set joint friction compensation code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     private void btnRobotSafetySet_Click(object sender, EventArgs e)
     {
         double[] lcoeff = { 0.9f, 0.9f, 0.9f, 0.9f, 0.9f, 0.9f };
         double[] wcoeff = { 0.4f, 0.4f, 0.4f, 0.4f, 0.4f, 0.4f };
         double[] ccoeff = { 0.6f, 0.6f, 0.6f, 0.6f, 0.6f, 0.6f }; 
         double[] fcoeff = { 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f }; 
 
         int rtn = robot.FrictionCompensationOnOff(1);
         Console.WriteLine($"FrictionCompensationOnOff rtn is{rtn}");
 
         Console.WriteLine($"FrictionCompensationOnOff rtn is{rtn}");
         Console.WriteLine($"SetFrictionValue_level rtn is {rtn}");
 
         Console.WriteLine($"SetFrictionValue_level rtn is {rtn}");
         Console.WriteLine($"SetFrictionValue_wall rtn is {rtn}");
 
         rtn = robot.SetFrictionValue_ceiling(ccoeff);
         Console.WriteLine($"SetFrictionValue_ceiling rtn is {rtn}");
 
         rtn = robot.SetFrictionValue_freedom(fcoeff);
         Console.WriteLine($"SetFrictionValue_freedom rtn is {rtn}");
     }
 
Querying the robot error code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /** 
     * @brief Query robot error code 
     * @param [out] maincode mainerrorcode
     * @param [out] subcode suberrorcode
     * @return errorcode 
     */ 
     int GetRobotErrorCode(ref int maincode, ref int subcode);
 
Error status clearing
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Error state clearing
     * @return Error code
     */
     int ResetAllError(); 
 
Example of getting robot fault status and clearing error code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
     private void btnRobotSafetySet_Click(object sender, EventArgs e)
     {
         int maincode=0, subcode=0;
         robot.GetRobotErrorCode(ref maincode, ref subcode);
         Console.WriteLine($"robot maincode is{maincode}; subcode is {subcode}" ); 
 
         robot.ResetAllError();
 
         Thread.Sleep(1000);
 
         robot.GetRobotErrorCode(ref maincode, ref subcode);
         Console.WriteLine($"robot maincode is{maincode}; subcode is{subcode}");
     }
 
Setting the parameters for monitoring the temperature and fan speed of the wide voltage control box
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.4  Web-3.8.3
    
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Setting the parameters for monitoring the temperature and fan speed of the wide voltage control box.
     * @param [in] enable 0-does not enable monitoring; 1-enables monitoring
     * @param [in] period Monitoring period (s), range 1-100.
     * @return Error code
     */ 
     int SetWideBoxTempFanMonitorParam(int enable, int period);
 
Get the parameters for monitoring the temperature and fan speed of the wide-voltage control box.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.4  Web-3.8.3
    
.. code-block:: c#
    :linenos:
 
     /**
     * @brief Get the parameters for monitoring the temperature and fan speed of the wide voltage control box.
     * @param [out] enable 0-not enable monitoring; 1-enable monitoring
     * @param [out] period Monitoring period (s), range 1-100.
     * @return Error code
     */ 
     int GetWideBoxTempFanMonitorParam(ref int enable, ref int period);
 
Code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.4  Web-3.8.3
    
.. code-block:: c#
    :linenos:
 
    private void button46_Click(object sender, EventArgs e)
    {
        var pkg = new ROBOT_STATE_PKG(); 
        robot.SetWideBoxTempFanMonitorParam(1, 2);    
        int enable = 0;
        int period = 0;
        robot.GetWideBoxTempFanMonitorParam(ref enable, ref period);
        Console.WriteLine($"GetWideBoxTempFanMonitorParam enable is {enable}   period is {period}");  
        for (int i = 0; i < 100; i++)
        {
            robot.GetRobotRealTimeState(ref pkg);
            Console.WriteLine($"robot ctrl box temp is {pkg.wideVoltageCtrlBoxTemp}, fan current is {pkg.wideVoltageCtrlBoxFanVel}");
            Thread.Sleep(100);
        }       
        int rtn = robot.SetWideBoxTempFanMonitorParam(0, 2);
        Console.WriteLine($"SetWideBoxTempFanMonitorParam rtn is {rtn}");       
        enable = 0;
        period = 0;
        robot.GetWideBoxTempFanMonitorParam(ref enable, ref period);
        Console.WriteLine($"GetWideBoxTempFanMonitorParam enable is {enable}   period is {period}");  
        for (int i = 0; i < 100; i++)
        {
            robot.GetRobotRealTimeState(ref pkg);
            Console.WriteLine($" robot ctrl box temp is {pkg.wideVoltageCtrlBoxTemp}, fan current is {pkg.wideVoltageCtrlBoxFanVel}");
            Thread.Sleep(100);
        }
    }

Set the focus calibration point
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.5  Web-3.8.4
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the focus calibration point
    * @param [in] pointNum Calibration point number of the focus 1-8
    * @param [in] point Coordinates of the calibration point
    * @return Error code 
    */
    int SetFocusCalibPoint(int pointNum, DescPose point);

Set the focus coordinates
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.5  Web-3.8.4
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the focus coordinates
    * @param [in] pos Focus coordinates XYZ
    * @return Error code 
    */
    int SetFocusPosition(DescTran pos);

Enable focus following
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.5  Web-3.8.4
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Enable focus following
    * @param [in] kp Proportional parameter, default 50.0
    * @param [in] kpredict Feedforward parameter, default 19.0
    * @param [in] aMax Maximum angular acceleration limit, default 1440°/s^2
    * @param [in] vMax Maximum angular velocity limit, default 180°/s
    * @param [in] type Lock X-axis direction (0 - reference input vector; 1 - horizontal; 2 - vertical)
    * @return Error code 
    */
    int FocusStart(double kp, double kpredict, double aMax, double vMax, int type);

Stop focus following
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.5  Web-3.8.4
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Stop focus following
    * @return Error code 
    */
    int FocusEnd();

Focus Follow Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.5  Web-3.8.4
    
.. code-block:: c#
    :linenos:

    private void button81_Click(object sender, EventArgs e)
    {
        DescPose p1Desc=new DescPose(186.331, 487.913, 209.850, 149.030, 0.688, -114.347);
        JointPos p1Joint = new JointPos(-127.876, -75.341, 115.417, -122.741, -59.820, 74.300);
        DescPose p2Desc = new DescPose(69.721, 535.073, 202.882, -144.406, -14.775, -89.012);
        JointPos p2Joint = new JointPos(-101.780, -69.828, 110.917, -125.740, -127.841, 74.300);
        DescPose p3Desc = new DescPose(146.861, 578.426, 205.598, 175.997, -36.178, -93.437);
        JointPos p3Joint = new JointPos(-112.851, -60.191, 86.566, -80.676, -97.463, 74.300);
        DescPose p4Desc = new DescPose(136.284, 509.876, 225.613, 178.987, 1.372, -100.696);
        JointPos p4Joint = new JointPos(-116.397, -76.281, 113.845, -128.611, -88.654, 74.299);
        DescPose p5Desc = new DescPose(138.395, 505.972, 298.016, 179.134, 2.147, -101.110);
        JointPos p5Joint = new JointPos(-116.814, -82.333, 109.162, -118.662, -88.585, 74.302);
        DescPose p6Desc = new DescPose(105.553, 454.325, 232.017, -179.426, 0.444, -99.952);
        JointPos p6Joint = new JointPos(-115.649, -84.367, 122.447, -128.663, -90.432, 74.303);
        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 100, 0, 0, 0);
        robot.GetForwardKin(p1Joint,ref p1Desc);
        robot.GetForwardKin(p2Joint, ref p2Desc);
        robot.GetForwardKin(p3Joint, ref p3Desc);
        robot.GetForwardKin(p4Joint, ref p4Desc);
        robot.GetForwardKin(p5Joint, ref p5Desc);
        robot.GetForwardKin(p6Joint, ref p6Desc);
        robot.MoveJ(p1Joint, p1Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(1);
        robot.MoveJ(p2Joint, p2Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(2);
        robot.MoveJ(p3Joint, p3Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(3);
        robot.MoveJ(p4Joint, p4Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(4);
        DescPose coordRtn = new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        int rtn = robot.ComputeTcp4(ref coordRtn);
        Console.WriteLine($"4 Point ComputeTool      {rtn} coord is {coordRtn.tran.x} ,{coordRtn.tran.y} ,{coordRtn.tran.z} ,{coordRtn.rpy.rx} ,{coordRtn.rpy.ry} ,{coordRtn.rpy.rz} ");
        robot.SetToolCoord(1, coordRtn, 0, 0, 1, 0);
        robot.GetForwardKin(p1Joint, ref p1Desc);
        robot.GetForwardKin(p2Joint, ref p2Desc);
        robot.GetForwardKin(p3Joint, ref p3Desc);
        robot.SetFocusCalibPoint(1, p1Desc);
        robot.SetFocusCalibPoint(2, p2Desc);
        robot.SetFocusCalibPoint(3, p3Desc);
        DescTran resultPos = new DescTran(0.0, 0.0, 0.0);
        double accuracy = 0.0;
        rtn = robot.ComputeFocusCalib(3, ref resultPos, ref accuracy);
        Console.WriteLine($"ComputeFocusCalib coord is  {rtn},{ resultPos.x} ,{ resultPos.y}, { resultPos.z}, accuracy is {accuracy} ");
        rtn = robot.SetFocusPosition(resultPos);
        robot.GetForwardKin(p5Joint, ref p5Desc);
        robot.GetForwardKin(p6Joint, ref p6Desc);
        robot.MoveL(p5Joint, p5Desc, 1, 0, 10, 100, 100, -1, 0, exaxisPos, 0, 1, offdese);
        robot.MoveL(p6Joint, p6Desc, 1, 0, 10, 100, 100, -1, 0, exaxisPos, 0, 1, offdese);
        robot.FocusStart(50, 19, 710, 90, 0);
        robot.MoveL(p5Joint, p5Desc, 1, 0, 10, 100, 100, -1, 0, exaxisPos, 0, 1, offdese);
        robot.MoveL(p6Joint, p6Desc, 1, 0, 10, 100, 100, -1, 0, exaxisPos, 0, 1, offdese);
        robot.FocusEnd();
    }

Enable joint torque sensor sensitivity calibration function
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Enable joint torque sensor sensitivity calibration function
    * @param [in] status 0-Disable；1-Enable
    * @return Error code
    */
    public int JointSensitivityEnable(int status);

Sensitivity data acquisition of joint torque sensors
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Sensitivity data acquisition of joint torque sensors
    * @return Error code
    */
    public int JointSensitivityCollect();

Get the sensitivity calibration results of the joint torque sensor
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get joint torque sensor sensitivity calibration results
    * @param [out] calibResult j1~j6 joint sensitivity [0-1]
    * @param [out] linearity j1~j6 joint linearity [0-1]
    * @return Error code
    */
    public int JointSensitivityCalibration(double calibResult[6], double linearity[6]);

Get Joint Torque Sensor Hysteresis Error
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Get joint torque sensor hysteresis error
    * @param [out] hysteresisError j1~j6 joint hysteresis error
    * @return Error code
    */
    public int JointHysteresisError(ref double[] hysteresisError);
    
Get Joint Torque Sensor Repeatability
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:
    
    /**
    * @brief Get joint torque sensor repeatability
    * @param [out] repeatability j1~j6 joint torque sensor repeatability
    * @return Error code
    */
    public int JointRepeatability(ref double[] repeatability);
    
Set Joint Force Sensor Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Set joint force sensor parameters
    * @param [in] M J1-J6 mass coefficient [0.001 ~ 10]
    * @param [in] B J1-J6 damping coefficient [0.001 ~ 10]
    * @param [in] K J1-J6 stiffness coefficient [0.001 ~ 10]
    * @param [in] threshold Force control threshold, Nm
    * @param [in] sensitivity Sensitivity, Nm/V, [0 ~ 10]
    * @param [in] setZeroFlag Function enable flag; 0-Off; 1-On; 2-Record zero point at position 1; 3-Record zero point at position 2
    * @return Error code
    */
    public int SetAdmittanceParams(double[] M, double[] B, double[] K, double[] threshold, double[] sensitivity, int setZeroFlag);

Joint torque sensor sensitivity automatic calibration Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    public int TestSensitivityCalib()
    {
        int rtn; 
        rtn = robot.JointSensitivityEnable(0);
        rtn = robot.JointSensitivityEnable(1);
        Console.WriteLine($"JointSensitivityEnable rtn is {rtn}");

        JointPos curJPos = new JointPos(0, 0, 0, 0, 0, 0);
        robot.GetActualJointPosDegree(0, ref curJPos);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        JointPos[] jointPoses = new JointPos[]
        {
            new JointPos(curJPos.jPos[0], 0, 0, -90, 0.02, curJPos.jPos[5]),
            new JointPos(curJPos.jPos[0], -30, 0, -90, 0.02, curJPos.jPos[5]),
            new JointPos(curJPos.jPos[0], -60, 0, -90, 0.02, curJPos.jPos[5]),
            new JointPos(curJPos.jPos[0], -90, 0, -90, 0.02, curJPos.jPos[5]),
            new JointPos(curJPos.jPos[0], -120, 0, -90, 0.02, curJPos.jPos[5]),
            new JointPos(curJPos.jPos[0], -150, 0, -90, 0.02, curJPos.jPos[5]),
            new JointPos(curJPos.jPos[0], -180, 0, -90, 0.02, curJPos.jPos[5])
        };
        for (int i = 0; i < jointPoses.Length; i++)
        {
            DescPose descPos = new DescPose(0, 0, 0, 0, 0, 0);
            robot.GetForwardKin(jointPoses[i], ref descPos);
            robot.MoveJ(jointPoses[i], descPos, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);

            Thread.Sleep(i == 0 ? 200 : 100);
            rtn = robot.JointSensitivityCollect();
            Console.WriteLine($"JointSensitivityCollect {i + 1} rtn is {rtn}");
            Thread.Sleep(100);
        }

        for (int i = jointPoses.Length - 2; i >= 0; i--)
        {
            DescPose descPos = new DescPose();
            robot.GetForwardKin(jointPoses[i], ref descPos);
            robot.MoveJ(jointPoses[i], descPos, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
            Thread.Sleep(100);
            rtn = robot.JointSensitivityCollect();
            Console.WriteLine($"JointSensitivityCollect {jointPoses.Length + (jointPoses.Length - 1 - i)} rtn is {rtn}");
            Thread.Sleep(100);
        }
        double[] calibResult = new double[6];
        double[] linearity = new double[6];
        rtn = robot.JointSensitivityCalibration(ref calibResult, ref linearity);
        Console.WriteLine($"JointSensitivityCalibration rtn is {rtn}");
        rtn = robot.JointSensitivityEnable(0);
        Console.WriteLine($"JointSensitivityEnable rtn is {rtn}");
        Console.WriteLine($"jointSensor Calib result is {calibResult[0]:F6} {calibResult[1]:F6} {calibResult[2]:F6} {calibResult[3]:F6} {calibResult[4]:F6} {calibResult[5]:F6}");
        Console.WriteLine($"jointSensor linearity is {linearity[0]:F6} {linearity[1]:F6} {linearity[2]:F6} {linearity[3]:F6} {linearity[4]:F6} {linearity[5]:F6}"); 
        double[] hysteresisError = new double[6];
        rtn = robot.JointHysteresisError(ref hysteresisError);
        Console.WriteLine($"JointHysteresisError result is {hysteresisError[0]:F6} {hysteresisError[1]:F6} {hysteresisError[2]:F6} {hysteresisError[3]:F6} {hysteresisError[4]:F6} {hysteresisError[5]:F6}");   
        double[] repeatability = new double[6];
        rtn = robot.JointRepeatability(ref repeatability);
        Console.WriteLine($"JointRepeatability result is {repeatability[0]:F6} {repeatability[1]:F6} {repeatability[2]:F6} {repeatability[3]:F6} {repeatability[4]:F6} {repeatability[5]:F6}");
        double[] M = new double[6] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        double[] B = new double[6] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        double[] K = new double[6] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        double[] threshold = new double[6] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        int setZeroFlag = 1;
        rtn = robot.SetAdmittanceParams(M, B, K, threshold, calibResult, setZeroFlag);
        Console.WriteLine($"SetAdmittanceParams rtn is {rtn}");
        robot.CloseRPC();
        return 0;
    }
 
Get the number of 8 slave port error frames of the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the number of 8 slave port error frames of the robot
    * @param [out] inRecvErr Input receiving error frames
    * @param [out] inCRCErr Input CRC error frames
    * @param [out] inTransmitErr Input transmit error frames
    * @param [out] inLinkErr Input link error frames
    * @param [out] outRecvErr Output receiving error frames
    * @param [out] outCRCErr Output CRC error frames
    * @param [out] outTransmitErr Output transmit error frames
    * @param [out] outLinkErr Output link error frames
    * @return Error code
    */
    public int GetSlavePortErrCounter(ref int[] inRecvErr,ref int[] inCRCErr,ref int[] inTransmitErr,ref int[] inLinkErr,ref int[] outRecvErr,ref int[] outCRCErr,ref int[] outTransmitErr,ref int[] outLinkErr);

Clear the slave port error num
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Clear the slave port error num
    * @param [in] slaveID slave id 0~7
    * @return Error code
    */
    public int SlavePortErrCounterClear(int slaveID);

Gets the slave port error frame code example
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    public void TestSlavePortErr()
    {
        int[] inRecvErr = new int[8];
        int[] inCRCErr = new int[8];
        int[] inTransmitErr = new int[8];
        int[] inLinkErr = new int[8];
        int[] outRecvErr = new int[8];
        int[] outCRCErr = new int[8];
        int[] outTransmitErr = new int[8];
        int[] outLinkErr = new int[8];

        robot.GetSlavePortErrCounter(ref inRecvErr, ref inCRCErr, ref inTransmitErr, ref inLinkErr,
            ref outRecvErr, ref outCRCErr, ref outTransmitErr, ref outLinkErr);

        for (int i = 0; i < 8; i++)
        {
            if (inRecvErr[i] != 0)
            {
                Console.WriteLine($"inRecvErr {i} is {inRecvErr[i]}");
            }

            if (inCRCErr[i] != 0)
            {
                Console.WriteLine($"inCRCErr {i} is {inCRCErr[i]}");
            }

            if (inTransmitErr[i] != 0)
            {
                Console.WriteLine($"inTransmitErr {i} is {inTransmitErr[i]}");
            }

            if (inLinkErr[i] != 0)
            {
                Console.WriteLine($"inLinkErr {i} is {inLinkErr[i]}");
            }

            if (outRecvErr[i] != 0)
            {
                Console.WriteLine($"outRecvErr {i} is {outRecvErr[i]}");
            }

            if (outCRCErr[i] != 0)
            {
                Console.WriteLine($"outCRCErr {i} is {outCRCErr[i]}");
            }

            if (outTransmitErr[i] != 0)
            {
                Console.WriteLine($"outTransmitErr {i} is {outTransmitErr[i]}");
            }

            if (outLinkErr[i] != 0)
            {
                Console.WriteLine($"outLinkErr {i} is {outLinkErr[i]}");
            }
        }
        Console.WriteLine("others has no err!");

        for (int i = 0; i < 8; i++)
        {
            robot.SlavePortErrCounterClear(i);
        }

        robot.CloseRPC();
    }

Set the feedforward coefficients of the velocities of each axis
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the feedforward coefficients of the velocities of each axis
    * @param [in] radio feedforward coefficients of the velocities of each axis
    * @return Error code
    */
    public int SetVelFeedForwardRatio(double radio[6]);

Get the feedforward coefficients of the velocities of each axis
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the feedforward coefficients of the velocities of each axis
    * @param [out] radio feedforward coefficients of the velocities of each axis
    * @return Error code
    */
    public int GetVelFeedForwardRatio(ref double radio[6]);

Code example for setting robot velocity feedforward
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    public void TestVelFeedForwardRatio()
    {

        double[] setRadio = new double[6] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        robot.SetVelFeedForwardRatio(setRadio);
        double[] getRadio = new double[6] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        robot.GetVelFeedForwardRatio(ref getRadio);
        Console.WriteLine($" {getRadio[0]:F6} {getRadio[1]:F6} {getRadio[2]:F6} {getRadio[3]:F6} {getRadio[4]:F6} {getRadio[5]:F6}");
    }
 
Photoelectric Sensor TCP Calibration - Compute Tool RPY
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Photoelectric Sensor TCP Calibration - Compute Tool RPY
    * @param [in] Btool Robot Cartesian position
    * @param [in] Etool Current tool coordinate values
    * @param [in] sensor Current sensor coordinate values (not yet available)
    * @param [in] radius Circular motion radius in mm (not yet available)
    * @param [in] dz Movement distance along the negative Z-axis of the base coordinate system; when dz = 10000, the function directly returns tool RPY
    * @param [out] TCPRPY Tool RPY values
    * @return Error code
    */
    public int TCPComputeRPY(DescPose Btool, DescPose Etool, DescPose sensor, double radius, double dz, out Rpy TCPRPY);

Photoelectric Sensor TCP Calibration - Compute Tool XYZ
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Photoelectric Sensor TCP Calibration - Compute Tool XYZ
    * @param [in] select 0-Compute tool TCP; 1-Compute sensor origin; 2-Compute sensor orientation; 3-Directly return tool TCP; 4-Record current workpiece coordinate system and tool coordinate system
    * @param [in] originDirection 0-X direction; 1-Y direction; 2-Z direction
    * @param [in] pos1 Robot Cartesian position 1
    * @param [in] pos2 Robot Cartesian position 2
    * @param [in] pos3 Robot Cartesian position 3
    * @param [in] pos4 Robot Cartesian position 4
    * @param [out] TCP Tool XYZ values
    * @return Error code
    */
    public int TCPComputeXYZ(int select, double originDirection, DescTran pos1, DescTran pos2,DescTran pos3, DescTran pos4, out DescTran TCP);

Photoelectric Sensor TCP Calibration - Start Recording Flange Center Position
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Photoelectric Sensor TCP Calibration - Start Recording Flange Center Position
    * @return Error code
    */
    public int TCPRecordFlangePosStart();

Photoelectric Sensor TCP Calibration - Stop Recording Flange Center Position
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Photoelectric Sensor TCP Calibration - Stop Recording Flange Center Position
    * @return Error code
    */
    public int TCPRecordFlangePosEnd();

Photoelectric Sensor TCP Calibration - Get Tool Center Point Position
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Photoelectric Sensor TCP Calibration - Get Tool Center Point Position
    * @param [out] TCP Tool center point position (x, y, z)
    * @return Error code
    */
    public int TCPGetRecordFlangePos(out DescTran TCP);

Photoelectric Sensor TCP Calibration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Photoelectric Sensor TCP Calibration
    * @param [in] luaPath Automatic calibration Lua program path: "FR_CalibrateTheToolTcp.lua"; 
    * @param [in] offsetX Teaching point offset (x, y, z) in mm
    * @param [out] TCP Calibrated tool coordinate system (x, y, z, rx, ry, rz)
    * @return Error code
    */
    public int PhotoelectricSensorTCPCalibration(string luaPath, DescTran offset, out DescPose TCP);

Photoelectric Sensor TCP Calibration Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    public void TestPhotoelectricSensorTCPCalib()
    {
        ROBOT_STATE_PKG pkg =new ROBOT_STATE_PKG();
        DescTran offset = new DescTran( 10.0, 10.0, 3.0 );
        DescPose TCP = new DescPose();
        int rtn = robot.PhotoelectricSensorTCPCalibration("FR_CalibrateTheToolTcp.lua", offset, out TCP);
        Console.WriteLine($"PhotoelectricSensorTCPCalibration : {rtn}");
        Console.WriteLine($"Tool TCP Coordinate: X={TCP.tran.x:F3}, Y={TCP.tran.y:F3}, Z={TCP.tran.z:F3}");
        Console.WriteLine($"Tool RPY Orientation: RX={TCP.rpy.rx:F3}, RY={TCP.rpy.ry:F3}, RZ={TCP.rpy.rz:F3}");
    }

Set Global Speed Instantly
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Set global speed instantly
    * @param [in] vel Speed percentage, range [0~100]
    * @return Error code
    */
    public int SetWeaveOffsetRT(DescPose offset)