Robot Status Enquiry
============================
.. toctree:: 
    :maxdepth: 5


Getting the robot mounting angle
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get robot mounting angle
    * @param [out] yangle Tilt angle
    * @param [out] zangle rotation angle
    * @return error code
    */
    int GetRobotInstallAngle(ref double yangle, ref double zangle); 

Getting system variable values
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief Getting system variable values
    * @param [in] id System variable number, range [1~20].
    * @param [out] value System variable value
    * @return error code
    */
    int GetSysVarValue(int id, ref double value). 

Get current joint position (angle)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get current joint position (angle).
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] jPos six joint positions in deg
    * @return error code
    */
    int GetActualJointPosDegree(byte flag, ref JointPos jPos); 

Get the current joint position (in radians)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get current joint position in radians.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] jPos six joint positions in rad
    * @return error code
    */  
    int GetActualJointPosRadian(byte flag, ref JointPos jPos);

Getting joint feedback speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get joint feedback speed -deg/s 
    * @param [in] flag 0-blocking, 1-non-blocking 
    * @param [out] speed Six joint speeds. 
    * @return error code 
    */
    int GetActualJointSpeedsDegree(byte flag, ref double[] speed);

Getting joint feedback acceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get joint feedback acceleration -deg/s^2 
    * @param [in] flag 0-blocking, 1-non-blocking 
    * @param [out] acc six joint acceleration 
    * @return error code 
    */
    int GetActualJointAccDegree(byte flag, ref double[] acc); 

Get TCP command speed-combination speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get TCP Command Speed-Combination Speed 
    * @param [in] flag 0-blocking, 1-non-blocking 
    * @param [out] tcp_speed Linear Speed 
    * @param [out] ori_speed Attitude Speed 
    * @return error code 
    */
    int GetTargetTCPCompositeSpeed(byte flag, ref double tcp_speed, ref double ori_speed); 

Get TCP feedback speed-combination speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
    
    /** 
    * @brief Getting TCP feedback speed-combination speeds
    * @param [in] flag 0-blocking, 1-non-blocking 
    * @param [out] tcp_speed Linear Speed 
    * @param [out] ori_speed Attitude Speed 
    * @return error code 
    */
    int GetActualTCPCompositeSpeed(byte flag, ref double tcp_speed, ref double ori_speed);

Get TCP command speed-split speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get TCP Command Speed - Minute Speed
    * @param [in] flag 0-blocking, 1-non-blocking 
    * @param [out] speed [x,y,z,rx,ry,rz] Speed 
    * @return error code 
    */
    int GetTargetTCPSpeed(byte flag, ref double[] speed);

Get TCP feedback speed-split speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get TCP feedback speed-split speed
    * @param [in] flag 0-blocking, 1-non-blocking 
    * @param [out] speed [x,y,z,rx,ry,rz] Speed 
    * @return error code 
    */
    int GetActualTCPSpeed(byte flag, ref double[] speed);

Get current tool position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get current tool position
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] desc_pos tool position
    * @return error code
    */
    int GetActualTCPPose(byte flag, ref DescPose desc_pos); 

Get the current tool coordinate system number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the current tool coordinate system number.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] id Tool coordinate system number
    * @return error code
    */
    int GetActualTCPNum(byte flag, ref int id);  

Get the current workpiece coordinate system number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the current workpiece coordinate system number.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] id Workpiece coordinate system number
    * @return error code
    */
    int GetActualWObjNum(byte flag, ref int id);

Get the current end flange position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get current end-flange position.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] desc_pos Flange position
    * @return error code
    */
    int GetActualToolFlangePose(byte flag, ref DescPose desc_pos);   

Inverse kinematics solution
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Inverse kinematics solution
    * @param [in] type 0-absolute pose (base coordinate system), 1-incremental pose (base coordinate system), 2-incremental pose (tool coordinate system)
    * @param [in] desc_pos Cartesian Positions
    * @param [in] config Joint space configuration, [-1] - solve with reference to current joint position, [0~7] - solve based on specific joint space configuration.
    * @param [out] joint_pos Joint position
    * @return error code
    */ 
    int GetInverseKin(int type, DescPose desc_pos, int config, ref JointPos joint_pos);

Inverse kinematics solution
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Inverse kinematics solving with reference to specified joint position solving
    * @param [in] type 0-absolute pose (base coordinate system), 1-incremental pose (base coordinate system), 2-incremental pose (tool coordinate system)
    * @param [in] desc_pos Cartesian Positions
    * @param [in] joint_pos_ref Reference joint position
    * @param [out] joint_pos Joint position
    * @return error code
    */  
    int GetInverseKin(int type, DescPose desc_pos, int config, ref JointPos joint_pos); 

Inverse kinematics solution (with reference to specified joint positions)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Inverse kinematics solving, refer to specified joint positions to determine if there is a solution
    * @param [in] type 0-absolute pose (base coordinate system), 1-incremental pose (base coordinate system), 2-incremental pose (tool coordinate system)
    * @param [in] desc_pos Cartesian Positions
    * @param [in] joint_pos_ref Reference joint position
    * @param [out] result 0 - no solution, 1 - solution
    * @return error code
    */  
    int GetInverseKinRef(int posMode, DescPose desc_pos, JointPos joint_pos_ref, ref JointPos joint_pos); 

Determining whether inverse kinematics has a solution 
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Inverse kinematics solving to determine if a solution exists for a specified reference joint position
    * @param [in] posMode 0 absolute pose, 1 relative pose - base coordinate system, 2 relative pose - tool coordinate system 
    * @param [in] desc_pos Cartesian Positions 
    * @param [in] joint_pos_ref Reference joint position 
    * @param [out] hasResult 0 - no solution, 1 - yes solution 
    * @return error code 
    */ 
    int GetInverseKinHasSolution(int posMode, DescPose desc_pos, JointPos joint_pos_ref, ref bool hasResult);  

Positive kinematics solution
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Positive kinematics solving
    * @param [in] joint_pos Joint position
    * @param [out] desc_pos Cartesian Positions
    * @return error code
    */
    int GetForwardKin(JointPos joint_pos, ref DescPose desc_pos); 

Get current joint torque
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get current joint torque
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] torques Joint torques
    * @return error code
    */
    int GetJointTorques(byte flag, float[] torques); 

Get the weight of the current load
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the weight of the current load
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] weight Load weight in kg
    * @return error code
    */
    int GetTargetPayload(byte flag, ref double weight). 

Get the centre of mass of the current load
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the centre of mass of the current load.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] cog load centre of mass in mm
    * @return error code
    */  
    int GetTargetPayloadCog(byte flag, ref DescTran cog);

Get the current tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the current tool coordinate system.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] desc_pos Tool coordinate system position
    * @return error code
    */
    int GetTCPOffset(byte flag, ref DescPose desc_pos); 

Get the current workpiece coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the current workpiece coordinate system.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] desc_pos Workpiece coordinate system position
    * @return error code
    */  
    int GetWObjOffset(byte flag, ref DescPose desc_pos); 

Obtaining the soft limiting angle of a joint
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Getting the soft limiting angle of the joints
    * @param [in] flag 0-blocking, 1-non-blocking    
    * @param [out] negative negative limit angle in deg
    * @param [out] positive positive limit angle in deg
    * @return error code
    */
    int GetJointSoftLimitDeg(byte flag, ref double[] negative, ref double[] positive); 

Get system time
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get system time
    * @param [out] t_ms Unit ms
    * @return error code
    */
    int GetSystemClock(ref double t_ms).

Get the current joint configuration of the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the current joint position of the robot
    * @param [out] config Joint space configuration, range [0-7].
    * @return error code
    */
    int GetRobotCurJointsConfig(ref int config).

Get current speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the current speed of the robot
    * @param [out] vel velocity in mm/s
    * @return error code
    */  
    int GetDefaultTransVel(ref double vel). 

Queries whether robot motion is complete
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Query whether robot motion is complete
    * @param [out] state 0-not completed, 1-completed
    * @return error code
    */  
    int GetRobotMotionDone(ref byte state).

code example
++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnRobotState_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        double yangle = 0, zangle = 0;
        byte flag = 0;
        JointPos j_deg = new JointPos(0, 0, 0, 0, 0, 0, 0, 0);
        JointPos j_rad = new JointPos(0, 0, 0, 0, 0, 0, 0);
        DescPose tcp, flange, tcp_offset, wobj_offset;
        DescTran cog.
        tcp = new DescPose();
        flange = new DescPose();
        tcp_offset = new DescPose();
        wobj_offset = new DescPose();
        cog = new DescTran();

        int id = 0;
        float[] torques = new float[6] { 0, 0, 0, 0, 0, 0, 0};
        double weight = 0.0;
        double[] neg_deg = new double[6] { 0, 0, 0, 0, 0, 0, 0 };
        double[] pos_deg = new double[6] { 0, 0, 0, 0, 0, 0, 0 };
        double t_ms = 0;
        int config = 0;
        double vel = 0;

        robot.GetRobotInstallAngle(ref yangle, ref zangle);
        Console.WriteLine($"yangle:{yangle},zangle:{zangle}");

        robot.GetActualJointPosDegree(flag, ref j_deg);
        Console.WriteLine($"joint pos deg:{j_deg.jPos[0]}, {j_deg.jPos[1]}, {j_deg.jPos[2]}, {j_deg.jPos[3]},{j_deg.jPos[4]},{j_deg.jPos[5]}");
        robot.GetActualJointPosRadian(flag, ref j_rad);
        Console.WriteLine($"joint pos rad:{j_rad.jPos[0]}, {j_rad.jPos[1]}, {j_rad.jPos[2]},{j_rad.jPos[3]},{j_rad.jPos[4]},{j_rad.jPos[5]}");

        robot.GetActualTCPPose(flag, ref tcp);
        Console.WriteLine($"tcp pose:{tcp.tran.x}, {tcp.tran.y}, {tcp.tran.z}, {tcp.rpy.rx}, {tcp.rpy.ry},{tcp.rpy.rz}");

        robot.GetActualToolFlangePose(flag, ref flange);
        Console.WriteLine($"flange pose:{flange.tran.x}, {flange.tran.y}, {flange.tran.z}, {flange.rpy.rx},{flange.rpy.ry},{flange.rpy.rz}");

        robot.GetActualTCPNum(flag, ref id);
        Console.WriteLine($"tcp num : {id}");

        robot.GetActualWObjNum(flag, ref id);
        Console.WriteLine($"wobj num : {id}");

        robot.GetJointTorques(flag, torques);
        Console.WriteLine($"torques:{torques[0]},{torques[1]},{torques[2]},{torques[3]},{torques[4]},{torques[5]}");

        robot.GetTargetPayload(flag, ref weight);
        Console.WriteLine($"payload weight : {weight}");

        robot.GetTargetPayloadCog(flag, ref cog);
        Console.WriteLine($"payload cog:{cog.x},{cog.y},{cog.z}");

        robot.GetTCPOffset(flag, ref tcp_offset);
        Console.WriteLine($"tcp offset:{tcp_offset.tran.x}, {tcp_offset.tran.y}, {tcp_offset.tran.z},{tcp_offset.rpy.rx}, {tcp_offset.rpy.ry}, {tcp_offset.rpy.rz}").

        robot.GetWObjOffset(flag, ref wobj_offset);
        Console.WriteLine($"wobj offset:{wobj_offset.tran.x}, {wobj_offset.tran.y},{wobj_offset.tran.z},{wobj_offset.rpy.rx},{wobj_offset. rpy.ry},{wobj_offset.rpy.rz}").

        robot.GetJointSoftLimitDeg(flag, ref neg_deg, ref pos_deg);
        Console.WriteLine($"neg limit deg:{neg_deg[0]}, {neg_deg[1]}, {neg_deg[2]}, {neg_deg[3]},{neg_deg[4]},{neg_deg[5]}");
        Console.WriteLine($"limit deg:{pos_deg[0]}, {pos_deg[1]}, {pos_deg[2]}, {pos_deg[3]},{pos_deg[4]},{pos_deg[5]}");

        robot.GetSystemClock(ref t_ms);
        Console.WriteLine($"system clock : {t_ms}");

        robot.GetRobotCurJointsConfig(ref config);
        Console.WriteLine($"joint config : {config}");

        robot.GetDefaultTransVel(ref vel);
        Console.WriteLine($"trans vel : {vel}");
    }

Query Robot Error Code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief query robot error code 
    * @param [out] maincode mainerrorcode
    * @param [out] subcode suberror code
    * @return error code 
    */ 
    int GetRobotErrorCode(ref int maincode, ref int subcode); 

Query Robot Teaching Management Point Data
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Query robotics instructional management point data 
    * @param [in] name Point name
    * @param [out] data point data double[20]{x,y,z,rx,ry,rz,j1,j2,j3,j4,j5,j6,tool, wobj,speed,acc,e1,e2,e3,e4}
    * @return error code 
    */ 
    int GetRobotTeachingPoint(string name, ref double[] data); 

Querying the robot motion queue cache length
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Query robot motion queue cache lengths 
    * @param [out] len Cache length
    * @return error code 
    */
    int GetMotionQueueLength(ref int len).

code example
+++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnRobotState2_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        byte robotMotionState = 255;
        robot.GetRobotMotionDone(ref robotMotionState);
        Console.WriteLine($"robotMotionState {robotMotionState}");

        int mainErrCode = -1;
        int subErrCode = -1;
        robot.GetRobotErrorCode(ref mainErrCode, ref subErrCode);
        Console.WriteLine($"mainErrCode {mainErrCode} subErrCode {subErrCode} ");

        string name = "a1";
        double[] point = new double[6] {0, 0, 0, 0, 0, 0, 0};
        robot.GetRobotTeachingPoint(name, ref point);
        Console.WriteLine($"GetRobotTeachingPoint:{point[0]},{point[1]},{point[2]},{point[3]},{point[4]},{point[5]}");

        int length = -1;
        robot.GetMotionQueueLength(ref length);
        Console.WriteLine($"GetMotionQueueLength {length}");
    }

Get robot real-time status structure
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
... versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Get robot real-time status structure
    * @param [out] pkg robot real-time status structure 
    * @return error code 
    */
    int GetRobotRealTimeState(ref ROBOT_STATE_PKG pkg);

code example
+++++++++++++++++++++++++++++++++
... versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    private void btnGetState_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        ROBOT_STATE_PKG pKG = new ROBOT_STATE_PKG();
        robot.GetRobotRealTimeState(ref pKG);
        Console.WriteLine($"the state is {pKG.main_code}");
    }