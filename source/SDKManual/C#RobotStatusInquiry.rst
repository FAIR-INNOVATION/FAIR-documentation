Robot Status Check
=========================

.. toctree:: 
    :maxdepth: 5

Get current joint position (angle).
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get current joint position (angle).
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] jPos Six joint positions in deg.
    * @return Error code.
    */
    int GetActualJointPosDegree(byte flag, ref JointPos jPos); 

Get current joint position in degrees of arc.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get current joint position in radians.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] jPos six joint positions in rad
    * @return Error code
    */    
    int GetActualJointPosRadian(byte flag, ref JointPos jPos);

Get the joint feedback velocity
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get joint feedback speed -deg/s 
    * @param [in] flag 0-blocking, 1-non-blocking 
    * @param [out] speed Six joint speeds. 
    * @return Error code. 
    */
    int GetActualJointSpeedsDegree(byte flag, ref double[] speed);

Get joint feedback acceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the joint feedback acceleration -deg/s^2 
    * @param [in] flag 0-blocking, 1-non-blocking 
    * @param [out] acc Six joint accelerations. 
    * @return Error code. 
    */
    int GetActualJointAccDegree(byte flag, ref double[] acc); 

Get TCP command velocity - joint velocity
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get TCP command speed-combining speed. 
    * @param [in] flag 0-blocking, 1-non-blocking 
    * @param [out] tcp_speed linear_speed 
    * @param [out] ori_speed Attitude Speed 
    * @return error code 
    */
    int GetTargetTCPCompositeSpeed(byte flag, ref double tcp_speed, ref double ori_speed); 

Get TCP feedback speed-composite speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
    
    /** 
    * @brief Get TCP feedback speed-combination speed.
    * @param [in] flag 0-blocking, 1-non-blocking 
    * @param [out] tcp_speed linear_speed 
    * @param [out] ori_speed Attitude Speed 
    * @return error code 
    */
    int GetActualTCPCompositeSpeed(byte flag, ref double tcp_speed, ref double ori_speed);

Get TCP command speed-composite speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get TCP command speed-split.
    * @param [in] flag 0-blocking, 1-non-blocking 
    * @param [out] speed [x,y,z,rx,ry,rz] speed 
    * @return error code 
    */
    int GetTargetTCPSpeed(byte flag, ref double[] speed);

Get TCP feedback speed-split speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get TCP feedback speed-cents.
    * @param [in] flag 0-blocking, 1-non-blocking 
    * @param [out] speed [x,y,z,rx,ry,rz] speed 
    * @return error code 
    */
    int GetActualTCPSpeed(byte flag, ref double[] speed);

Get current tool position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get current tool position
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] desc_pos Tool position.
    * @return Error code.
    */
    int GetActualTCPPose(byte flag, ref DescPose desc_pos); 

Get the current tool coordinate system number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the current tool coordinate system number.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] id tool coordinate system number
    * @return Error code.
    */
    int GetActualTCPNum(byte flag, ref int id).  
 
Get the current tool coordinate system number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the current workpiece coordinate system number.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] id The workpiece coordinate system number.
    * @return Error code.
    */
    int GetActualWObjNum(byte flag, ref int id);

Get the current end flange position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the current end flange position.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] desc_pos Flange position.
    * @return Error code.
    */
    int GetActualToolFlangePose(byte flag, ref DescPose desc_pos);  

Get the current joint torque
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get current joint torque.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] torques Joint torques.
    * @return Error code.
    */
    int GetJointTorques(byte flag, float[] torques). 

Get system time
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the system time.
    * @param [out] t_ms unit ms
    * @return Error code.
    */
    int GetSystemClock(ref double t_ms).

Queries if the robot movement is complete
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Query if robot motion is complete.
    * @param [out] state 0-not completed, 1-completed
    * @return Error code.
    */    
    int GetRobotMotionDone(ref byte state).

Queries the length of the robot motion queue cache
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Query the robot motion queue cache length. 
    * @param [out] len Cache length.
    * @return Error code. 
    */
    int GetMotionQueueLength(ref int len).

Get the robot emergency stop state
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the state of the robot's emergency stop.
    * @param [out] state Emergency stop state, 0-non-emergency stop, 1-emergency stop.
    * @return Error code.  
    */
    int GetRobotEmergencyStopState(ref byte state).

Get the state of the communication between the SDK and the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the communication state of the SDK and the robot.
    * @param [out] state communication state, 0-normal communication, 1-abnormal communication.
    */
    int GetSDKComState(ref int state).

Get safety stop signal
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the safe stop signal.
    * @param [out] si0_state Safe stop signal SI0, 0-invalid, 1-valid
    * @param [out] si1_state safe stop signal SI1, 0-invalid, 1-valid
    */
    int GetSafetyStopState(ref byte si0_state, ref byte si1_state);

Get robot joint actuator temperature (℃)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the robot's articulated actuator temperature (℃).
    * @return Error code
    */
    int GetJointDriverTemperature(double[] temperature).

Get robot joint driver torque(Nm).
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get robot joint actuator torque (Nm).
    * @return Error code
    */
    int GetJointDriverTorque(double torque[]).

Get robot real-time status structure
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the robot realtime state structure.
    * @param [out] pkg Robot real-time status structure. 
    * @return Error code 
    */
    int GetRobotRealTimeState(ref ROBOT_STATE_PKG pkg);

Sample Robot Status Query Code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button29_Click(object sender, EventArgs e)
    {
        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
        double yangle = 0, zangle = 0;
        robot.GetRobotInstallAngle(ref yangle, ref zangle);
        Console.WriteLine($"yangle:{yangle},zangle:{zangle}");

        JointPos j_deg = new JointPos(0,0,0,0,0,0);
        robot.GetActualJointPosDegree(0, ref j_deg);
        Console.WriteLine($"joint pos deg:{j_deg.jPos[0]},{j_deg.jPos[1]},{j_deg.jPos[2]},{j_deg.jPos[3]},{j_deg.jPos[4]},{j_deg.jPos[5]}");

        double[] jointSpeed = new double[6];
        robot.GetActualJointSpeedsDegree(0, ref jointSpeed);
        Console.WriteLine($"joint speeds deg:{jointSpeed[0]},{jointSpeed[1]},{jointSpeed[2]},{jointSpeed[3]},{jointSpeed[4]},{jointSpeed[5]}");

        double[] jointAcc = new double[6];
        robot.GetActualJointAccDegree(0, ref jointAcc);
        Console.WriteLine($"joint acc deg:{jointAcc[0]},{jointAcc[1]},{jointAcc[2]},{jointAcc[3]},{jointAcc[4]},{jointAcc[5]}");

        double tcp_speed = 0, ori_speed = 0;
        robot.GetTargetTCPCompositeSpeed(0, ref tcp_speed, ref ori_speed);
        Console.WriteLine($"GetTargetTCPCompositeSpeed tcp {tcp_speed}; ori {ori_speed}");

        robot.GetActualTCPCompositeSpeed(0, ref tcp_speed, ref ori_speed);
        Console.WriteLine($"GetActualTCPCompositeSpeed tcp {tcp_speed}; ori {ori_speed}");

        double[] targetSpeed = new double[6];
        robot.GetTargetTCPSpeed(0,ref targetSpeed);
        Console.WriteLine($"GetTargetTCPSpeed {targetSpeed[0]},{targetSpeed[1]},{targetSpeed[2]},{targetSpeed[3]},{targetSpeed[4]},{targetSpeed[5]}");

        double[] actualSpeed = new double[6];
        robot.GetActualTCPSpeed(0, ref actualSpeed);
        Console.WriteLine($"GetTargetTCPSpeed {actualSpeed[0]},{actualSpeed[1]},{actualSpeed[2]},{actualSpeed[3]},{actualSpeed[4]},{actualSpeed[5]}");

        DescPose tcp = new DescPose(0, 0, 0, 0, 0, 0);
        robot.GetActualTCPPose(0, ref tcp);
        Console.WriteLine($"tcp pose:{tcp.tran.x},{tcp.tran.y},{tcp.tran.z},{tcp.rpy.rx},{tcp.rpy.ry},{tcp.rpy.rz}");

        DescPose flange = new DescPose(0, 0, 0, 0, 0, 0);
        robot.GetActualToolFlangePose(0, ref flange);
        Console.WriteLine($"flange pose:{flange.tran.x},{flange.tran.y},{flange.tran.z},{flange.rpy.rx},{flange.rpy.ry},{flange.rpy.rz}");

        int id = 0;
        robot.GetActualTCPNum(0, ref id);
        Console.WriteLine($"tcp num:{id}");

        robot.GetActualWObjNum(0, ref id);
        Console.WriteLine($"wobj num:{id}");

        double[] jtorque = new double[6];
        robot.GetJointTorques(0, jtorque);
        Console.WriteLine($"torques:{jtorque[0]},{jtorque[1]},{jtorque[2]},{jtorque[3]},{jtorque[4]},{jtorque[5]}");

        double t_ms = 0;
        robot.GetSystemClock(ref t_ms);
        Console.WriteLine($"system clock:{t_ms}");

        int config = 0;
        robot.GetRobotCurJointsConfig(ref config);
        Console.WriteLine($"joint config:{config}");

        byte motionDone = 0;
        robot.GetRobotMotionDone(ref motionDone);
        Console.WriteLine($"GetRobotMotionDone :{motionDone}");

        int len = 0;
        robot.GetMotionQueueLength(ref len);
        Console.WriteLine($"GetMotionQueueLength :{len}");

        byte emergState = 0;
        robot.GetRobotEmergencyStopState(ref emergState);
        Console.WriteLine($"GetRobotEmergencyStopState :{emergState}");

        int comstate = 0;
        robot.GetSDKComState(ref comstate);
        Console.WriteLine($"GetSDKComState :{comstate}");

        byte si0_state = 0, si1_state = 0;
        robot.GetSafetyStopState(ref si0_state, ref si1_state);
        Console.WriteLine($"GetSafetyStopState :{si0_state} {si1_state}");

        double[] temp = new double[6];
        robot.GetJointDriverTemperature(temp);
        Console.WriteLine($"Temperature:{temp[0]},{temp[1]},{temp[2]},{temp[3]},{temp[4]},{temp[5]}");

        double[] torque = new double[6];
        robot.GetJointDriverTorque(torque);
        Console.WriteLine($"torque:{torque[0]},{torque[1]},{torque[2]},{torque[3]},{torque[4]},{torque[5]}");

        robot.GetRobotRealTimeState(ref pkg);
    }

Inverse kinematics solution
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Inverse kinematics solution
    * @param [in] type 0-absolute position (base coordinate system), 1-incremental position (base coordinate system), 2-incremental position (instrumental coordinate system)
    * @param [in] desc_pos Cartesian Position
    * @param [in] config joint_space_configuration, [-1] - solve with reference to the current joint position, [0~7] - solve according to a specific joint space configuration
    * @param [out] joint_pos Joint position.
    * @return Error code
    */ 
    int GetInverseKin(int type, DescPose desc_pos, int config, ref JointPos joint_pos);

Inverse kinematics solution (reference position)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Inverse kinematics solving, referencing a specified joint position to determine if there is a solution.
    * @param [in] type 0-absolute position (base coordinate system), 1-incremental position (base coordinate system), 2-incremental position (instrumental coordinate system)
    * @param [in] desc_pos Cartesian Position
    * @param [in] joint_pos_ref reference joint position
    * @param [out] result 0-no solution, 1-with solution
    * @return Error code
    */    
    int GetInverseKinRef(int posMode, DescPose desc_pos, JointPos joint_pos_ref, ref JointPos joint_pos); 

Get whether the inverse kinematics has a solution
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Inverse kinematics solution, determine if there is a solution for the specified reference joint position.
    * @param [in] posMode 0 absolute position, 1 relative position-base coordinate system, 2 relative position-tool coordinate system. 
    * @param [in] desc_pos Cartesian position. 
    * @param [in] joint_pos_ref Reference joint position. 
    * @param [out] hasResult 0-no solution, 1-has solution 
    * @return Error code 
    */ 
    int GetInverseKinHasSolution(int posMode, DescPose desc_pos, JointPos joint_pos_ref, ref bool hasResult);  

Positive kinematics solution
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Positive kinematics solution.
    * @param [in] joint_pos joint position
    * @param [out] desc_pos Cartesian position
    * @return Error code.
    */
    int GetForwardKin(JointPos joint_pos, ref DescPose desc_pos); 

Robot Forward and Reverse Kinematics Calculation Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button30_Click(object sender, EventArgs e)
    {
         JointPos j1 = new JointPos(-11.904f, -99.669f, 117.473f, -108.616f, -91.726f, 74.256f);
         DescPose desc_pos1 = new DescPose(-419.524f, -13.000f, 351.569f, -178.118f, 0.314f, 3.833f);
         JointPos inverseRtn = new JointPos(0, 0, 0, 0, 0, 0, 0, 0);
         robot.GetInverseKin(0, desc_pos1, -1, ref inverseRtn);
         Console.WriteLine($"dcs1 GetInverseKin rtn is {inverseRtn.jPos[0]} {inverseRtn.jPos[1]} {inverseRtn.jPos[2]} {inverseRtn.jPos[3]} { inverseRtn.jPos[4]} {inverseRtn.jPos[5]}");
         robot.GetInverseKinRef(0, desc_pos1, j1, ref inverseRtn);
         Console.WriteLine($"dcs1 GetInverseKinRef rtn is {inverseRtn.jPos[0]} {inverseRtn.jPos[1]} {inverseRtn.jPos[2]} {inverseRtn.jPos[3]} { inverseRtn.jPos[4]} {inverseRtn.jPos[5]}");
         bool hasResut = false;
         robot.GetInverseKinHasSolution(0, desc_pos1, j1, ref hasResut);
         Console.WriteLine($"dcs1 GetInverseKinRef result {hasResut}");
         DescPose forwordResult = new DescPose(0, 0, 0, 0, 0, 0, 0);
         robot.GetForwardKin(j1, ref forwordResult);
         Console.WriteLine($"jpos1 forwordResult rtn is {forwordResult.tran.x} {forwordResult.tran.y} {forwordResult.tran.z} {forwordResult.rpy.rx } {forwordResult.rpy.ry} {forwordResult.rpy.rz}");
    }

Querying Robot Teaching Management Point Data
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Query robot instruction management point data. 
    * @param [in] name Point name.
    * @param [out] data point data double[20]{x,y,z,rx,ry,rz,j1,j2,j3,j4,j5,j6,tool, wobj,speed,acc,e1,e2,e3,e4}
    * @return Error code 
    */ 
    int GetRobotTeachingPoint(string name, ref double[] data); 

Get the robot DH parameter compensation value 
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get the robot DH parameter compensation value. 
    * @param [out] dhCompensation Robot DH parameter compensation value (mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]
    * @return Error code 
    */
    int GetDHCompensation(ref double[] dhCompensation).

Get control box SN code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get control box SN code.
    * @param [out] SNCode Control Box SN Code
    * @return Error code.
    */
    int GetRobotSN(ref string SNCode);

Query the robot teaching management point data code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button31_Click(object sender, EventArgs e)
    {
        string name = "A0";
        double[] data = new double[20];
        int rtn = robot.GetRobotTeachingPoint(name, ref data);
        Console.WriteLine(" {0} name is: {1} \n", rtn, name);
        for (int i = 0; i < 20; i++)
        {
            Console.WriteLine("data is: {0} \n", data[i]);
        }

        int que_len = 0;
        rtn = robot.GetMotionQueueLength(ref que_len);
        Console.WriteLine("GetMotionQueueLength rtn is: {0}, queue length is: {1} \n", rtn, que_len);

        double[] dh = { 0, 0, 0, 0, 0, 0 };
        int retval = 0;
        retval = robot.GetDHCompensation(ref dh);
        Console.WriteLine($"retval is  {retval}");
        Console.WriteLine($"dh is {dh[0]}, {dh[1]}, {dh[2]}, {dh[3]}, {dh[4]}, {dh[5]}");
        string SN = "";
        robot.GetRobotSN(ref SN);
        Console.WriteLine($"robot SN is  {SN}");
    }

Get Tool Coordinate System by ID
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Get Tool Coordinate System by ID
    * @param [in] id Tool coordinate system ID
    * @param [out] coord Coordinate system value
    * @return Error code
    */
    int GetToolCoordWithID(int id,ref DescPose coord)

Get Work Object Coordinate System by ID
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Get Work Object Coordinate System by ID
    * @param [in]  id Work object coordinate system ID
    * @param [out] coord Coordinate system value
    * @return Error code
    */
    public int GetWObjCoordWithID(int id, ref DescPose coord)

Get External Tool Coordinate System by ID
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Get External Tool Coordinate System by ID
    * @param [in]  id External tool coordinate system ID
    * @param [out] coord Coordinate system value
    * @return Error code
    */
    public int GetExToolCoordWithID(int id, ref DescPose coord)

Get Extended Axis Coordinate System by ID
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
    * @brief Get Extended Axis Coordinate System by ID
    * @param [in]  id Extended axis coordinate system ID
    * @param [out] coord Coordinate system value
    * @return Error code
    */
    public int GetExAxisCoordWithID(int id, ref DescPose coord)

Get Current Tool Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
     * @brief Get Current Tool Coordinate System
     * @param [out] coord Coordinate system value
     * @return Error code
     */
    public int GetCurToolCoord(ref DescPose coord)

Get Current Work Object Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
     * @brief Get Current Work Object Coordinate System
     * @param [out] coord Coordinate system value
     * @return Error code
     */
    public int GetCurWObjCoord(ref DescPose coord)

Get Current External Tool Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
     * @brief Get Current External Tool Coordinate System
     * @param  [out] coord Coordinate system value
     * @return Error code
     */
    public int GetCurExToolCoord(ref DescPose coord)

 Get Current Extended Axis Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    /**
     * @brief Get Current Extended Axis Coordinate System
     * @param [out] coord Coordinate system value
     * @return Error code
     */
    public int GetCurExAxisCoord(ref DescPose coord)

Get Current Extended Axis Coordinate System
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    public void TestCoordMain()
    {  
        DescPose t_coord = new DescPose(0, 0, 0, 0, 0, 0);
        t_coord.tran.x = 1.0;
        t_coord.tran.y = 2.0;
        t_coord.tran.z = 300.0;
        t_coord.rpy.rx = 4.0;
        t_coord.rpy.ry = 5.0;
        t_coord.rpy.rz = 6.0;
        int id = 1;
        DescPose toolCoord = new DescPose();
        robot.GetToolCoordWithID(id, ref toolCoord);
        Console.WriteLine($"GetToolCoordWithID {id}, {toolCoord.tran.x} {toolCoord.tran.y} {toolCoord.tran.z} {toolCoord.rpy.rx} {toolCoord.rpy.ry} {toolCoord.rpy.rz}");
        DescPose wobjCoord = new DescPose();
        robot.GetWObjCoordWithID(id, ref wobjCoord);
        Console.WriteLine($"GetWObjCoordWithID {id}, {wobjCoord.tran.x} {wobjCoord.tran.y} {wobjCoord.tran.z} {wobjCoord.rpy.rx} {wobjCoord.rpy.ry} {wobjCoord.rpy.rz}");
        DescPose extoolCoord = new DescPose();
        robot.GetExToolCoordWithID(id, ref extoolCoord);
        Console.WriteLine($"GetExToolCoordWithID {id}, {extoolCoord.tran.x} {extoolCoord.tran.y} {extoolCoord.tran.z} {extoolCoord.rpy.rx} {extoolCoord.rpy.ry} {extoolCoord.rpy.rz}");
        DescPose exAxisCoord = new DescPose();
        robot.GetExAxisCoordWithID(id, ref exAxisCoord);
        Console.WriteLine($"GetExAxisCoordWithID {id}, {exAxisCoord.tran.x} {exAxisCoord.tran.y} {exAxisCoord.tran.z} {exAxisCoord.rpy.rx} {exAxisCoord.rpy.ry} {exAxisCoord.rpy.rz}");
        double weight = 0.0;
        DescTran cog = new DescTran();
        robot.GetTargetPayloadWithID(id, ref weight, ref cog);
        Console.WriteLine($"GetTargetPayloadWithID {id}, {weight} {cog.x} {cog.y} {cog.z}");
        robot.GetCurToolCoord(ref toolCoord);
        Console.WriteLine($"GetCurToolCoord {toolCoord.tran.x} {toolCoord.tran.y} {toolCoord.tran.z} {toolCoord.rpy.rx} {toolCoord.rpy.ry} {toolCoord.rpy.rz}");

        robot.GetCurWObjCoord(ref wobjCoord);
        Console.WriteLine($"GetCurWObjCoord {wobjCoord.tran.x} {wobjCoord.tran.y} {wobjCoord.tran.z} {wobjCoord.rpy.rx} {wobjCoord.rpy.ry} {wobjCoord.rpy.rz}");
        robot.GetCurExToolCoord(ref extoolCoord);
        Console.WriteLine($"GetExToolCoordWithID {extoolCoord.tran.x} {extoolCoord.tran.y} {extoolCoord.tran.z} {extoolCoord.rpy.rx} {extoolCoord.rpy.ry} {extoolCoord.rpy.rz}");
        robot.GetCurExAxisCoord(ref exAxisCoord);
        Console.WriteLine($"GetCurExAxisCoord {exAxisCoord.tran.x} {exAxisCoord.tran.y} {exAxisCoord.tran.z} {exAxisCoord.rpy.rx} {exAxisCoord.rpy.ry} {exAxisCoord.rpy.rz}");
        double weightT = 0.0f;
        DescTran cogT = new DescTran();
        robot.GetTargetPayload(0, ref weightT);
        robot.GetTargetPayloadCog(0, ref cogT);
        Console.WriteLine($"GetTargetPayload {weightT} {cogT.x} {cogT.y} {cogT.z}");
        DescPose coordSet = new DescPose(0, 10, 2, 3, 4, 5);
        robot.SetToolCoord(2, coordSet, 0, 0, 1, 0);
        DescPose Coordset0 = new DescPose(0, 0, 0, 0, 0, 0);
        DescPose Coordset = new DescPose(1, 2, 3, 4, 5, 6);
        DescPose etcp = new DescPose(10, 20, 30, 40, 50, 60);
        DescPose etctool = new DescPose(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
        robot.SetToolCoord(id, Coordset, 0, 0, 1, 0);
        Thread.Sleep(100);
        robot.SetWObjCoord(id, Coordset, 0);
        Thread.Sleep(100);
        robot.ExtAxisActiveECoordSys(id, 1, Coordset, 0);
        Thread.Sleep(100);
        robot.SetExToolCoord(id, etcp, etctool);
        Thread.Sleep(100);
        robot.SetLoadWeight(id, (float)1.5);
        //Thread.Sleep(500);
        robot.SetLoadCoord(id, cog);
        Thread.Sleep(100);
    }





