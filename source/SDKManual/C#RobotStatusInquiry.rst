Status query
=====================

.. toctree:: 
    :maxdepth: 5

Obtain robot mounting Angle
+++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Obtain robot mounting Angle
    * @param  [out] yangle Angle of inclination
    * @param  [out] zangle Angle of rotation
    * @return  Error code
    */
    int GetRobotInstallAngle(ref double yangle, ref double zangle); 

Get the system variable value
+++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the system variable value
    * @param  [in] id System variable number, range[1~20]
    * @param  [out] value  System variable value
    * @return  Error code
    */
    int GetSysVarValue(int id, ref double value); 

Get the current joint position (Angle)
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the current joint position (Angle)
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] jPos Six joint positions, unit: deg
    * @return  Error code
    */
    int GetActualJointPosDegree(byte flag, ref JointPos jPos); 

Get the current joint position (radians)
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the current joint position (radians)
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] jPos Six joint positions, unit: rad
    * @return  Error code
    */   
    int GetActualJointPosRadian(byte flag, ref JointPos jPos);

Get joint feedback velocity
+++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get joint feedback velocity-deg/s 
    * @param [in] flag blocking state 0-blocking, 1-non-blocking
    * @param [out] speed Joint velocity
    * @return Error code
    */
    int GetActualJointSpeedsDegree(byte flag, ref double[] speed);

Get joint feedback acceleration
+++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get joint feedback acceleration-deg/s^2 
    * @param [in] flag blocking state 0-blocking, 1-non-blocking
    * @param [out] acc Joint acceleration
    * @return Error code 
    */
    int GetActualJointAccDegree(byte flag, ref double[] acc); 

Gets TCP target velocity- close velocity
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Gets TCP target velocity- close velocity
    * @param [in] flag blocking state 0-blocking, 1-non-blocking
    * @param [out] tcp_speed Linear speed
    * @param [out] ori_speed Attitude velocity
    * @return Error code 
    */
    int GetTargetTCPCompositeSpeed(byte flag, ref double tcp_speed, ref double ori_speed); 

Gets TCP actual velocity- close velocity
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Gets TCP actual velocity- close velocity
    * @param [in] flag blocking state 0-blocking, 1-non-blocking 
    * @param [out] tcp_speed Linear speed 
    * @param [out] ori_speed Attitude velocity
    * @return Error code
    */
    int GetActualTCPCompositeSpeed(byte flag, ref double tcp_speed, ref double ori_speed);

Gets TCP target velocity- component of velocity
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Gets TCP target velocity- component of velocity
    * @param [in] flag blocking state 0-blocking, 1-non-blocking
    * @param [out] speed [x,y,z,rx,ry,rz] velocity
    * @return Error code 
    */
    int GetTargetTCPSpeed(byte flag, ref double[] speed);

Gets TCP actual velocity- component of velocity
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Gets TCP actual velocity- component of velocity
    * @param [in] flag blocking state 0-blocking, 1-non-blocking
    * @param [out] speed [x,y,z,rx,ry,rz]velocity 
    * @return Error code
    */
    int GetActualTCPSpeed(byte flag, ref double[] speed);

Get the current tool pose
+++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the current tool pose
    * @param  [in] flag  0- blocking, 1- non-blocking
    * @param  [out] desc_pos  Tool position
    * @return  Error code
    */
    int GetActualTCPPose(byte flag, ref DescPose desc_pos); 

Get the current tool coordinate system number
++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the current tool coordinate system number
    * @param  [in] flag  0- blocking, 1- non-blocking
    * @param  [out] id  Tool coordinate system number
    * @return  Error code
    */
    int GetActualTCPNum(byte flag, ref int id); 

Get the current workpiece coordinate system number
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the current workpiece coordinate system number
    * @param  [in] flag  0- blocking, 1- non-blocking
    * @param  [out] id  Job coordinate system number
    * @return  Error code
    */
    int GetActualWObjNum(byte flag, ref int id); 

Get the current end flange pose
+++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the current end flange pose
    * @param  [in] flag  0- blocking, 1- non-blocking
    * @param  [out] desc_pos  Flange pose
    * @return  Error code
    */
    int GetActualToolFlangePose(byte flag, ref DescPose desc_pos);

Inverse kinematics solution
+++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Inverse kinematics solution
    * @param  [in] type 0- absolute pose (base frame), 1- incremental pose (base frame), 2- incremental pose (tool frame)
    * @param  [in] desc_pos Cartesian pose
    * @param  [in] config Joint space configuration, [-1]- based on the current joint position, [0~7]- based on the specific joint space configuration
    * @param  [out] joint_pos Joint position
    * @return  Error code
    */
    int GetInverseKin(int type, DescPose desc_pos, int config, ref JointPos joint_pos); 

Inverse kinematics solution
++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Inverse kinematics is solved by referring to the specified joint position
    * @param  [in] type 0- absolute pose (base frame), 1- incremental pose (base frame), 2- incremental pose (tool frame)
    * @param  [in] desc_pos Cartesian pose
    * @param  [in] joint_pos_ref Reference joint position
    * @param  [out] joint_pos Joint position
    * @return  Error code
    */   
    int GetInverseKinRef(int posMode, DescPose desc_pos, JointPos joint_pos_ref, ref JointPos joint_pos); 

Inverse kinematics solution (Refer to the specified joint position)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  To solve the inverse kinematics, refer to the specified joint position to determine whether there is a solution
    * @param  [in] posMode  0- absolute pose (base frame), 1- incremental pose (base frame), 2- incremental pose (tool frame)
    * @param  [in] desc_pos Cartesian pose
    * @param  [in] joint_pos_ref Reference joint position
    * @param  [out] hasResult 0- no solution, 1-solution
    * @return  Error code
    */   
    int GetInverseKinHasSolution(int posMode, DescPose desc_pos, JointPos joint_pos_ref, ref bool hasResult); 

Determine whether inverse kinematics has a solution
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Inverse kinematics solution to determine whether there is a solution for the specified reference joint position
    * @param [in] posMode 0 absolute pose, 1 relative pose - base coordinate system, 2 relative pose - tool coordinate system 
    * @param [in] desc_pos Descartes pose
    * @param [in] joint_pos_ref Reference joint position 
    * @param [out] hasResult 0-No solution, 1-There is a solution
    * @return Error code
    */ 
    int GetInverseKinHasSolution(int posMode, DescPose desc_pos, JointPos joint_pos_ref, ref bool hasResult); 

Forward kinematics solution
++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Forward kinematics solution
    * @param  [in] joint_pos Joint position
    * @param  [out] desc_pos Cartesian pose
    * @return  Error code
    */
    int GetForwardKin(JointPos joint_pos, ref DescPose desc_pos); 

Obtain the current joint torque
++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Obtain the current joint torque
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] torques Joint torque
    * @return  Error code
    */
    int GetJointTorques(byte flag, float[] torques);

Get the weight of the current load
++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Gets the weight of the current load
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] weight Load weight, unit: kg
    * @return  Error code
    */
    int GetTargetPayload(byte flag, ref double weight);

Get the center of mass of the current load
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the center of mass of the current load
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] cog Load center of mass, unit: mm
    * @return  Error code
    */   
    int GetTargetPayloadCog(byte flag, ref DescTran cog);

Get the current tool coordinate system
++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the current tool coordinate system
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] desc_pos Tool coordinate position
    * @return  Error code
    */
    int GetTCPOffset(byte flag, ref DescPose desc_pos);

Get the current work frame
++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the current work frame
    * @param  [in] flag 0- blocking, 1- non-blocking
    * @param  [out] desc_pos Position of workpiece coordinate system
    * @return  Error code
    */   
    int GetWObjOffset(byte flag, ref DescPose desc_pos);

Obtain joint soft limit Angle
++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Obtain joint soft limit Angle
    * @param  [in] flag 0- blocking, 1- non-blocking    
    * @param  [out] negative  Negative limit Angle, unit: deg
    * @param  [out] positive  Positive limit Angle, unit: deg
    * @return  Error code
    */
    int GetJointSoftLimitDeg(byte flag, ref double[] negative, ref double[] positive);

Get system time
++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get system time
    * @param  [out] t_ms unit: ms
    * @return  Error code
    */
    int GetSystemClock(ref double t_ms);

Get the current joint configuration of the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the current joint configuration of the robot
    * @param  [out]  config  Joint space configuration, range [0~7]
    * @return  Error code
    */
    int GetRobotCurJointsConfig(ref int config);

Get current speed
++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the robot's default speed
    * @param  [out]  vel  The unit is mm/s
    * @return  Error code
    */   
    errno_t  GetDefaultTransVel(float *vel);

Query whether the robot movement is complete
+++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Query whether the robot movement is complete
    * @param  [out]  state  0- Incomplete, 1- completed
    * @return  Error code
    */   
    int GetDefaultTransVel(ref double vel);

Code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnRobotState_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        double yangle = 0, zangle = 0;
        byte flag = 0;
        JointPos j_deg = new JointPos(0, 0, 0, 0, 0, 0);
        JointPos j_rad = new JointPos(0, 0, 0, 0, 0, 0);
        DescPose tcp, flange, tcp_offset, wobj_offset;
        DescTran cog;
        tcp = new DescPose();
        flange = new DescPose();
        tcp_offset = new DescPose();
        wobj_offset = new DescPose();
        cog = new DescTran();

        int id = 0;
        float[] torques = new float[6] { 0, 0, 0, 0, 0, 0};
        double weight = 0.0;
        double[] neg_deg = new double[6] { 0, 0, 0, 0, 0, 0 };
        double[] pos_deg = new double[6] { 0, 0, 0, 0, 0, 0 };
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
        Console.WriteLine($"tcp offset:{tcp_offset.tran.x}, {tcp_offset.tran.y}, {tcp_offset.tran.z},{tcp_offset.rpy.rx},{tcp_offset.rpy.ry},{tcp_offset.rpy.rz}");

        robot.GetWObjOffset(flag, ref wobj_offset);
        Console.WriteLine($"wobj offset:{wobj_offset.tran.x}, {wobj_offset.tran.y},{wobj_offset.tran.z},{wobj_offset.rpy.rx},{wobj_offset.rpy.ry},{wobj_offset.rpy.rz}");

        robot.GetJointSoftLimitDeg(flag, ref neg_deg, ref pos_deg);
        Console.WriteLine($"neg limit deg:{neg_deg[0]}, {neg_deg[1]}, {neg_deg[2]}, {neg_deg[3]},{neg_deg[4]},{neg_deg[5]}");
        Console.WriteLine($"pos limit deg:{pos_deg[0]}, {pos_deg[1]}, {pos_deg[2]}, {pos_deg[3]},{pos_deg[4]},{pos_deg[5]}");

        robot.GetSystemClock(ref t_ms);
        Console.WriteLine($"system clock : {t_ms}");

        robot.GetRobotCurJointsConfig(ref config);
        Console.WriteLine($"joint config : {config}");

        robot.GetDefaultTransVel(ref vel);
        Console.WriteLine($"trans vel : {vel}");
    }

Query the robot error code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Query the robot error code 
    * @param [out] maincode Master error code
    * @param [out] subcode  Sub error code
    * @return Error code 
    */ 
    int GetRobotErrorCode(ref int maincode, ref int subcode); 

Query the data of robot teaching management point
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Query the data of robot teaching management point
    * @param [in] name point name
    * @param [out] data point data double[20]{x,y,z,rx,ry,rz,j1,j2,j3,j4,j5,j6,tool, wobj,speed,acc,e1,e2,e3,e4}
    * @return Error code 
    */ 
    int GetRobotTeachingPoint(string name, ref double[] data); 

Query the robot motion queue cache length
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Query the robot motion queue cache length 
    * @param [out] len  robot motion queue cache length
    * @return Error code
    */
    int GetMotionQueueLength(ref int len);

Code example
++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnRobotState2_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        byte robotMotionState = 255;
        robot.GetRobotMotionDone(ref robotMotionState);
        Console.WriteLine($"robotMotionState  {robotMotionState}");

        int mainErrCode = -1;
        int subErrCode = -1;
        robot.GetRobotErrorCode(ref mainErrCode, ref subErrCode);
        Console.WriteLine($"mainErrCode  {mainErrCode}  subErrCode  {subErrCode} ");

        string name = "a1";
        double[] point = new double[6] {0, 0, 0, 0, 0, 0};
        robot.GetRobotTeachingPoint(name, ref point);
        Console.WriteLine($"GetRobotTeachingPoint:{point[0]},{point[1]},{point[2]},{point[3]},{point[4]},{point[5]}");

        int length = -1;
        robot.GetMotionQueueLength(ref length);
        Console.WriteLine($"GetMotionQueueLength  {length}");
    }

Get the real-time status structure of the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. versionadded:: C# SDK-v1.0.6

.. code-block:: c#
    :linenos:
    
    /** 
    * @brief Get the real-time status structure of the robot
    * @param [out] pkg pkg robot real-time status structure
    * @return error code
    */
    int GetRobotRealTimeState(ref ROBOT_STATE_PKG pkg);

Code example
++++++++++++++

.. versionadded:: C# SDK-v1.0.6

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
