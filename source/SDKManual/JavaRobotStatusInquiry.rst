Robot status inquiry  
=========================  

.. toctree::  
    :maxdepth: 5  

Get current joint positions (degrees)  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get current joint positions (degrees)  
    * @param  [out] jPos Six joint positions in deg  
    * @return  Error code  
    */  
    int GetActualJointPosDegree(JointPos jPos);  

Get joint feedback speed (deg/s)  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get joint feedback speed (deg/s)  
    * @param [out] speed Six joint speeds  
    * @return Error code  
    */  
    int GetActualJointSpeedsDegree(Object[] speed);  

Get joint feedback acceleration  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get joint feedback acceleration (deg/s^2)  
    * @param  [in] flag 0-Blocking, 1-Non-blocking  
    * @param  [out] acc Six joint accelerations  
    * @return  Error code  
    */  
    public int GetActualJointAccDegree(int flag, Object[] acc)  

Get TCP command composite speed  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get TCP command speed  
    * @param  [in] flag 0-Blocking, 1-Non-blocking  
    * @param  [out] tcp_speed Linear speed  
    * @param  [out] ori_speed Orientation speed  
    * @return  Error code  
    */  
    public int GetTargetTCPCompositeSpeed(int flag, double tcp_speed, double ori_speed)  

Get TCP feedback composite speed  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get TCP feedback composite speed  
    * @param  [in] flag 0-Blocking, 1-Non-blocking  
    * @param  [out] tcp_speed Linear speed  
    * @param  [out] ori_speed Orientation speed  
    * @return  Error code  
    */  
    public int GetActualTCPCompositeSpeed(int flag, double tcp_speed, double ori_speed)  

Get TCP command speed  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get TCP command speed  
    * @param  [in] flag 0-Blocking, 1-Non-blocking  
    * @param  [out] speed [x,y,z,rx,ry,rz] speed  
    * @return  Error code  
    */  
    public int GetTargetTCPSpeed(int flag, Object[] speed)  

Get TCP feedback speed  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get TCP feedback speed  
    * @param  [in] flag 0-Blocking, 1-Non-blocking  
    * @param  [out] speed [x,y,z,rx,ry,rz] speed  
    * @return  Error code  
    */  
    public int GetActualTCPSpeed(int flag, Object[] speed)  

Get current tool pose  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get current tool pose  
    * @param  [out] desc_pos  Tool pose  
    * @return  Error code  
    */  
    int GetActualTCPPose(DescPose desc_pos);  

Get current tool coordinate system number  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get current tool coordinate system number  
    * @param  [in] flag  0-Blocking, 1-Non-blocking  
    * @param  [out] id  Tool coordinate system number  
    * @return  Error code  
    */  
    int GetActualTCPNum(int flag, int[] id)  

Get current workpiece coordinate system number  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get current workpiece coordinate system number  
    * @param  [in] flag  0-Blocking, 1-Non-blocking  
    * @param  [out] id  Workpiece coordinate system number  
    * @return  Error code  
    */  
    public int GetActualWObjNum(int flag, int[] id)  

Get current end flange pose  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get current end flange pose  
    * @param  [in] flag  0-Blocking, 1-Non-blocking  
    * @param  [out] desc_pos  Flange pose  
    * @return  Error code  
    */  
    public int GetActualToolFlangePose(int flag, DescPose desc_pos)  

Get current joint torque  
+++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get current joint torque  
    * @param  [in]  flag 0-Blocking, 1-Non-blocking  
    * @param  [out]  torques Joint torques  
    * @return  Error code  
    */  
    int GetJointTorques(int flag, Object[] torques);  

Get system time  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get system time  
    * @return  List[0]:int Error code; List[1]:double t_ms in ms  
    */  
    List<Number> GetSystemClock();  

Check if robot motion is completed  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Check if robot motion is completed  
    * @param   [out] state  0-Not completed, 1-Completed  
    * @return  Error code  
    */  
    public int GetRobotMotionDone(int[] state)  

Query robot motion queue buffer length  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Query robot motion queue buffer length  
    * @param   [out] len  Buffer length  
    * @return  Error code  
    */  
    public int GetMotionQueueLength(int[] len)  

Get robot emergency stop state  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get robot emergency stop state  
    * @param [out] state Emergency stop state, 0-Normal, 1-Emergency stop  
    * @return Error code  
    */  
    public int GetRobotEmergencyStopState(int[] state)  

Get SDK-robot communication state  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get SDK-robot communication state  
    * @return state Communication state, 0-Normal, 1-Abnormal  
    */  
    public int GetSDKComState()  

Get safety stop signal  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get safety stop signal  
    * @param  [out] si0_state Safety stop signal SI0, 0-Invalid, 1-Valid  
    * @param  [out] si1_state Safety stop signal SI1, 0-Invalid, 1-Valid  
    * @return Error code  
    */  
    public int GetSafetyStopState(int[] si0_state, int[] si1_state)  

Get robot joint driver temperature (°C)  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get robot joint driver temperature (°C)  
    * @param  [out] temperature Temperature  
    * @return Error code  
    */  
    public int GetJointDriverTemperature(double[] temperature)  

Get robot joint driver torque (Nm)  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get robot joint driver torque (Nm)  
    * @param  [out] torque Torque  
    * @return Error code  
    */  
    public int GetJointDriverTorque(double[] torque)  

Get robot real-time state structure  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get robot real-time state structure  
    * @return Real-time state structure  
    */  
    public ROBOT_STATE_PKG GetRobotRealTimeState()  

Robot status inquiry code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    public static int TestGetStatus(Robot robot)  
    {  
        List<Number> angle=new ArrayList<>();  
        angle=robot.GetRobotInstallAngle();  
        System.out.println("yangle:"+angle.get(1)+",zangle:"+angle.get(2));  

        JointPos j_deg =new JointPos(){};  
        robot.GetActualJointPosDegree( j_deg);  

        Object[] jointSpeed =new Object[] { 0,0,0,0,0,0 };  
        robot.GetActualJointSpeedsDegree(jointSpeed);  

        Object[] jointAcc = new Object[]{0,0,0,0,0,0 };  
        robot.GetActualJointAccDegree(0, jointAcc);  

        double tcp_speed = 0.0;  
        double ori_speed = 0.0;  
        robot.GetTargetTCPCompositeSpeed(0, tcp_speed, ori_speed);  

        robot.GetActualTCPCompositeSpeed(0, tcp_speed, ori_speed);  

        Object[] targetSpeed =new Object[] { 0,0,0,0,0,0 };  
        robot.GetTargetTCPSpeed(0, targetSpeed);  

        Object[] actualSpeed =new Object[] {0,0,0,0,0,0 };  
        robot.GetActualTCPSpeed(0, actualSpeed);  

        DescPose tcp = new DescPose(){};  
        robot.GetActualTCPPose(tcp);  

        DescPose flange = new DescPose(){};  
        robot.GetActualToolFlangePose(0, flange);  

        int[] id = {};  
        robot.GetActualTCPNum(0, id);  

        robot.GetActualWObjNum(0, id);  

        List<Number> jtorque=new ArrayList<>();  
        jtorque=robot.GetJointTorques(0);  

        List<Number> t_ms = new ArrayList<>();  
        t_ms=robot.GetSystemClock();  

        List<Integer> config = new ArrayList<>();  
        config=robot.GetRobotCurJointsConfig();  

        int motionDone = 0;  
        robot.GetRobotMotionDone(motionDone);  

        int[] len ={0 };  
        robot.GetMotionQueueLength(len);  

        int[] emergState = {0};  
        robot.GetRobotEmergencyStopState(emergState);  

        int comstate = 0;  
        comstate=robot.GetSDKComState();  

        int[] si0_state=new int[]{0}, si1_state=new int[]{0};  
        robot.GetSafetyStopState(si0_state, si1_state);  

        double[] temp =new double[] { 0,0,0,0,0,0 };  
        robot.GetJointDriverTemperature(temp);  

        double[] torque = new double[]{ 0,0,0,0,0,0 };  
        robot.GetJointDriverTorque(torque);  

        ROBOT_STATE_PKG pkg=new ROBOT_STATE_PKG();  
        pkg=robot.GetRobotRealTimeState();  

        return 0;  
    }  

Inverse kinematics calculation  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Inverse kinematics calculation  
    * @param  [in] type 0-Absolute pose (base frame), 1-Incremental pose (base frame), 2-Incremental pose (tool frame)  
    * @param  [in] desc_pos Cartesian pose  
    * @param  [in] config Joint space configuration, [-1]-Solve with reference to current joint position, [0~7]-Solve with specific joint space configuration  
    * @param  [out] joint_pos Joint position  
    * @return  Error code  
    */  
    int GetInverseKin(int type, DescPose desc_pos, int config, JointPos joint_pos);  

Inverse kinematics calculation (reference position)  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Inverse kinematics calculation with reference joint position  
    * @param  [in] posMode 0-Absolute pose, 1-Relative pose-base frame, 2-Relative pose-tool frame  
    * @param  [in] desc_pos Cartesian pose  
    * @param  [in] joint_pos_ref Reference joint position  
    * @param  [out] joint_pos Joint position  
    * @return  Error code  
    */  
    int GetInverseKinRef(int posMode, DescPose desc_pos, JointPos joint_pos_ref, JointPos joint_pos);  

Check if inverse kinematics has solution  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Check if inverse kinematics has solution with reference joint position  
    * @param  [in] posMode 0-Absolute pose, 1-Relative pose-base frame, 2-Relative pose-tool frame  
    * @param  [in] desc_pos Cartesian pose  
    * @param  [in] joint_pos_ref Reference joint position  
    * @return  Error code  List[0]:Error code; List[1]: int hasResult 0-No solution, 1-Solution exists  
    */  
    List<Integer> GetInverseKinHasSolution(int posMode, DescPose desc_pos, JointPos joint_pos_ref);  

Forward kinematics calculation  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Forward kinematics calculation  
    * @param  [in] joint_pos Joint position  
    * @param  [out] desc_pos Cartesian pose  
    * @return  Error code  
    */  
    int GetForwardKin(JointPos joint_pos, DescPose desc_pos);  

Robot forward/inverse kinematics calculation code example  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestInverseKin(Robot robot)  
    {  
        JointPos j1=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);  
        DescPose desc_pos1=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);  

        JointPos inverseRtn = new JointPos(){};  

        robot.GetInverseKin(0, desc_pos1, -1, inverseRtn);  
        robot.GetInverseKinRef(0, desc_pos1, j1, inverseRtn);  

        int hasResut = 0;  
        robot.GetInverseKinHasSolution(0, desc_pos1, j1);  

        DescPose forwordResult = new DescPose(){};  
        robot.GetForwardKin(j1, forwordResult);  

        return 0;  
    }  

Query robot teaching management point data  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:   

    /**  
    * @brief Query robot teaching management point data  
    * @param [in]  name  Point name  
    * @return  List[0]:Error code; List[1] - List[20] : Point data double[20]{x,y,z,rx,ry,rz,j1,j2,j3,j4,j5,j6,tool,wobj,speed,acc,e1,e2,e3,e4}  
    */  
    List<Number> GetRobotTeachingPoint(String name);  

Get robot DH parameter compensation values  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Get robot DH parameter compensation values  
    * @param dhCompensation Robot DH parameter compensation values (mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]  
    * @return Error code  
    */  
    public int GetDHCompensation(Object[] dhCompensation)  

Get controller SN code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.4-3.8.1

.. code-block:: Java  
    :linenos:   

    /**  
    * @brief Get controller SN code  
    * @param [out] SNCode Controller SN code  
    * @return Error code  
    */  
    int GetRobotSN(String[] SNCode);  

Query robot teaching management point data code example  
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java  
    :linenos:  

    public static int TestGetTeachPoint(Robot robot)  
    {  
        String name = "P1";  
        List<Number> data=new ArrayList<>();  
        data = robot.GetRobotTeachingPoint(name);  
        System.out.println(name+" name is: "+data.get(0));  
        for (int i = 0; i < 20; i++)  
        {  
            System.out.println("data is: "+ data.get(i+1));  
        }  

        int[] que_len = {0};  
        int rtn = robot.GetMotionQueueLength(que_len);  
        System.out.println("GetMotionQueueLength rtn is:"+rtn+", queue length is:"+ que_len[0]);  

        Object[] dh = new Object[]{ 0,0,0,0,0,0 };  
        int retval = 0;  
        retval = robot.GetDHCompensation(dh);  
        System.out.println("retval is: "+retval);  

        String[] SN = new String[]{""};  
        robot.GetRobotSN(SN);  
        System.out.println("robot SN is "+SN[0]);  
        return 0;  
    }  

Get Tool Coordinate System by ID
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: Java
    :linenos:

    /**
    * @brief Get Tool Coordinate System by ID
    * @param [in] id Tool coordinate system ID
    * @param [out] coord Coordinate system value
    * @return Error code
    */
    int GetToolCoordWithID(int id, DescPose coord)

Get Work Object Coordinate System by ID
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: Java
    :linenos:

    /**
    * @brief Get Work Object Coordinate System by ID
    * @param [in]  id Work object coordinate system ID
    * @param [out] coord Coordinate system value
    * @return Error code
    */
    public int GetWObjCoordWithID(int id, DescPose coord)

Get External Tool Coordinate System by ID
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: Java
    :linenos:

    /**
    * @brief Get External Tool Coordinate System by ID
    * @param [in]  id External tool coordinate system ID
    * @param [out] coord Coordinate system value
    * @return Error code
    */
    public int GetExToolCoordWithID(int id, DescPose coord)

Get Extended Axis Coordinate System by ID
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: Java
    :linenos:

    /**
    * @brief Get Extended Axis Coordinate System by ID
    * @param [in]  id Extended axis coordinate system ID
    * @param [out] coord Coordinate system value
    * @return Error code
    */
    public int GetExAxisCoordWithID(int id, DescPose coord)

Get Current Tool Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: Java
    :linenos:

    /**
     * @brief Get Current Tool Coordinate System
     * @param [out] coord Coordinate system value
     * @return Error code
     */
    public int GetCurToolCoord(DescPose coord)

Get Current Work Object Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: Java
    :linenos:

    /**
     * @brief Get Current Work Object Coordinate System
     * @param [out] coord Coordinate system value
     * @return Error code
     */
    public int GetCurWObjCoord(DescPose coord)

Get Current External Tool Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: Java
    :linenos:

    /**
     * @brief Get Current External Tool Coordinate System
     * @param  [out] coord Coordinate system value
     * @return Error code
     */
    public int GetCurExToolCoord(DescPose coord)

Get Current Extended Axis Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.9-3.8.6

.. code-block:: Java
    :linenos:

    /**
     * @brief Get Current Extended Axis Coordinate System
     * @param [out] coord Coordinate system value
     * @return Error code
     */
    public int GetCurExAxisCoord(DescPose coord)

Get Robot Coordinate System and Payload Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void TestCoord(Robot robot)
    {
        int id = 1;
        int rtn = 0;
        DescPose toolCoord = new DescPose();
        DescPose extoolCoord = new DescPose();
        DescPose wobjCoord = new DescPose();
        DescPose exAxisCoord = new DescPose();


        robot.GetCurToolCoord(toolCoord);// Tool
        System.out.println("GetToolCoord:"+id+","+
                toolCoord.tran.x+","+ toolCoord.tran.y+","+ toolCoord.tran.z+","+
                toolCoord.rpy.rx+","+ toolCoord.rpy.ry+","+ toolCoord.rpy.rz);


        robot.GetCurWObjCoord(toolCoord);// Work object
        System.out.println("GetCurWObjCoord:"+id+","+
                toolCoord.tran.x+","+ toolCoord.tran.y+","+ toolCoord.tran.z+","+
                toolCoord.rpy.rx+","+ toolCoord.rpy.ry+","+ toolCoord.rpy.rz);

        robot.GetCurExToolCoord(toolCoord);// External tool
        System.out.println("GetCurExToolCoord:"+id+","+
                toolCoord.tran.x+","+ toolCoord.tran.y+","+ toolCoord.tran.z+","+
                toolCoord.rpy.rx+","+ toolCoord.rpy.ry+","+ toolCoord.rpy.rz);


        robot.GetCurExAxisCoord(toolCoord);// Extended axis
        System.out.println("GetCurExAxisCoord:"+id+","+ // Corrected print string for consistency
                toolCoord.tran.x+","+ toolCoord.tran.y+","+ toolCoord.tran.z+","+
                toolCoord.rpy.rx+","+ toolCoord.rpy.ry+","+ toolCoord.rpy.rz);


        List<Number> weightT = new ArrayList<>();// Center of gravity
        DescTran cogT=new DescTran();
        weightT=robot.GetTargetPayload(0);
        robot.GetTargetPayloadCog(0,cogT);
        System.out.println("GetTargetPayload :"+weightT.get(1).doubleValue()+", "+
                cogT.x+", "+cogT.y+", "+cogT.z);


        robot.GetToolCoordWithID(id, toolCoord);
        System.out.println("GetToolCoordWithID:"+id+","+
                toolCoord.tran.x+","+ toolCoord.tran.y+","+ toolCoord.tran.z+","+
                toolCoord.rpy.rx+","+ toolCoord.rpy.ry+","+ toolCoord.rpy.rz);

        robot.GetWObjCoordWithID(id, wobjCoord);
        System.out.println("GetWObjCoordWithID "+id+", "+
                wobjCoord.tran.x+","+ wobjCoord.tran.y+","+ wobjCoord.tran.z+","+
                wobjCoord.rpy.rx+","+ wobjCoord.rpy.ry+","+ wobjCoord.rpy.rz);


        robot.GetExToolCoordWithID(id, extoolCoord);// External tool
        System.out.println("GetExToolCoordWithID :"+ id+","+
                extoolCoord.tran.x+","+ extoolCoord.tran.y+","+ extoolCoord.tran.z+","+
                extoolCoord.rpy.rx+","+ extoolCoord.rpy.ry+","+ extoolCoord.rpy.rz);

        robot.GetExAxisCoordWithID(id, exAxisCoord);// Extended axis
        System.out.println("GetExAxisCoordWithID "+id+","+
                exAxisCoord.tran.x+","+ exAxisCoord.tran.y+","+ exAxisCoord.tran.z+","+
                exAxisCoord.rpy.rx+","+ exAxisCoord.rpy.ry+","+ exAxisCoord.rpy.rz);


        double[] weight = new double[1];// Payload center of gravity
        DescTran getCog = new DescTran();
        robot.GetTargetPayloadWithID(id, weight, getCog);
        System.out.println("GetTargetPayloadWithID :"+ id+","+ weight[0]+","+
                getCog.x+","+ getCog.y+","+ getCog.z);

        DescPose coordSet0 = new DescPose(0, 0, 0, 0, 0, 0);
        DescPose coordSet = new DescPose(1, 2, 3, 4, 5, 6);
        DescPose etcp = new DescPose(10, 20, 30, 40, 50, 60);
        DescPose etool = new DescPose(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
        DescTran cog = new DescTran(1, 2, 3);

        robot.SetToolCoord(id, coordSet, 0, 0, 1, 0);
        robot.Sleep(100);
        robot.SetWObjCoord(id, coordSet, 0);
        robot.Sleep(100);
        robot.ExtAxisActiveECoordSys(id, 1, coordSet, 1); // Apply calibration result to extended axis coordinate system
        robot.Sleep(100);
        rtn = robot.SetExToolCoord(id, etcp, etool);
        robot.Sleep(100);
        rtn = robot.SetLoadWeight(id, 1.5);
        robot.Sleep(500);
        rtn = robot.SetLoadCoord(id, cog);
        robot.Sleep(100);
    }
