Robot Status Inquiry
=====================================

.. toctree:: 
    :maxdepth: 5

Getting the robot mounting angle
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get robot mounting angle
    * @return List[0]:error code; List[1]:double yangle tilt angle; List[2]:double zangle rotation angle
    */
    List<Number> GetRobotInstallAngle(); 

Getting system variable values
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Getting the value of a system variable
    * @param [in] id System variable number, range [1~20].
    * @return List[0]:error code; List[1]:double value system variable value
    */
    List<Number> GetSysVarValue(int id). 

Get the current joint position (angle).
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get current joint position (angle).
    * @param [out] jPos The six joint positions obtained, in degrees.
    * @return error code
    */
    int GetActualJointPosDegree(JointPos jPos); 

Get joint feedback speed -deg/s
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get joint feedback speed -deg/s 
    * @param [out] speed six joint speeds
    * @return error code 
    */
    int GetActualJointSpeedsDegree(Object[] speed).

Get current tool position
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get current tool position
    * @param [out] desc_pos Tool position
    * @return error code
    */
    int GetActualTCPPose(DescPose desc_pos); 

Inverse kinematics solution
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Inverse kinematics solving
    * @param [in] type 0-absolute pose (base coordinate system), 1-incremental pose (base coordinate system), 2-incremental pose (tool coordinate system)
    * @param [in] desc_pos Cartesian Positions
    * @param [in] config Joint space configuration, [-1] - solve with reference to the current joint position, [0~7] - solve based on specific joint space configuration.
    * @param [out] joint_pos Joint position
    * @return error code
    */ 
    int GetInverseKin(int type, DescPose desc_pos, int config, JointPos joint_pos);

Inverse kinematics solution
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Inverse kinematics solving with reference to specified joint position solving
    * @param [in] posMode 0 absolute pose, 1 relative pose - base coordinate system 2 relative pose - tool coordinate system
    * @param [in] desc_pos Cartesian Positions
    * @param [in] joint_pos_ref Reference joint position
    * @param [out] joint_pos Joint position
    * @return error code
    */  
    int GetInverseKinRef(int posMode, DescPose desc_pos, JointPos joint_pos_ref, JointPos joint_pos); 

Inverse kinematics solution
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Inverse kinematics solving, refer to specified joint positions to determine if there is a solution
    * @param [in] posMode 0 absolute pose, 1 relative pose - base coordinate system 2 relative pose - tool coordinate system
    * @param [in] desc_pos Cartesian Positions
    * @param [in] joint_pos_ref Reference joint position
    * @return ErrorCode List[0]:ErrorCode; List[1]: int hasResult 0-no solution, 1-with solution
    */  
    List<Integer> GetInverseKinHasSolution(int posMode, DescPose desc_pos, JointPos joint_pos_ref);  

Positive kinematics solving
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Positive kinematics solving
    * @param [in] joint_pos Joint position
    * @param [out] desc_pos Cartesian Positions
    * @return error code
    */
    int GetForwardKin(JointPos joint_pos, DescPose desc_pos); 

Get current joint torque
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get current joint torque
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] torques Joint torque
    * @return error code
    */
    int GetJointTorques(int flag, Object[] torques); 

Get the weight of the current load
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get the weight of the current load
    * @param [in] flag 0-blocking, 1-non-blocking
    * @return List[0]: int error code; List[1]: double weight load weight in kg
    */
    List<Number> GetTargetPayload(int flag). 

Get the center of mass of the current load
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get the center of mass of the current load.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] cog load center of mass in mm
    * @return error code
    */  
    int GetTargetPayloadCog(int flag, DescTran cog);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        robot.SetRobotInstallAngle(23.4, 56.7);
        List<Number> rtnArr = robot.GetRobotInstallAngle();
        System.out.println("Mounting Angle: " + rtnArr.get(1) + " " + rtnArr.get(2));
        robot.SetRobotInstallAngle(0, 0);

        robot.SetLoadWeight(0);
        robot.SetLoadCoord(new DescTran(0.0, 0.0, 0.0));

        DescTran cog = new DescTran();
        robot.GetTargetPayloadCog(1, cog);

        System.out.println("weight is " + rtnArr.get(1) + " cog is " + cog.x + " " + cog.y + " " + cog.z);

        List<Integer> Arr = robot.GetRobotCurJointsConfig();
        System.out.println("config is " + Arr.get(1));

        DescPose desc_p1=new DescPose();

        JointPos JP1=new JointPos(117.408,-86.777,81.499,-87.788,-92.964,92.959);
        JointPos JP_test=new JointPos();
        DescPose DP1 =new DescPose(327.359,-420.973,518.377,-177.199,3.209,114.449);
        robot.GetInverseKin(0, DP1, -1, JP_test);
        List<Integer> rtnArrInt = robot.GetInverseKinHasSolution(0, DP1, JP1);//whether there is a solution in the reverse direction
        System.out.println("has Solution ? " + rtnArrInt.get(1));
        robot.GetForwardKin(JP1, desc_p1);// forward kinematics
        JointPos j2 = new JointPos();
        robot.GetInverseKinRef(0, DP1, JP1, JP_test);//inverse kinematics
    }

Get the current tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get the current tool coordinate system.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] desc_pos Tool coordinate system position
    * @return error code
    */
    int GetTCPOffset(int flag, DescPose desc_pos); 

Get the current workpiece coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get the current coordinate system of the workpiece.
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] desc_pos Workpiece coordinate system position
    * @return error code
    */  
    int GetWObjOffset(int flag, DescPose desc_pos); 

Obtaining the soft limiting angle of a joint
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Getting the soft limiting angle of the joints
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] negative negative limit angle in deg
    * @param [out] positive positive limit angle in deg
    * @return error code
    */
    int GetJointSoftLimitDeg(int flag, Object[] negative, Object[] positive); 

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        DescPose offset = new DescPose();
        robot.GetTCPOffset(1, offset);//tools
        System.out.println("offset is " + offset);
        robot.GetWObjOffset(1, offset);//workpiece
        System.out.println("offset is " + offset);

        Object[] neg_deg = new Object[]{0, 0 , 0, 0, 0, 0, 0};
        Object[] pos_deg = new Object[]{0, 0 , 0, 0, 0, 0, 0, 0};
        robot.GetJointSoftLimitDeg(1, neg_deg, pos_deg);
        System.out.println("neg is " + Arrays.toString(neg_deg) + " pos is " + Arrays.toString(pos_deg));
    }

Get system time
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get system time
    * @return List[0]:int error code; List[1]:double t_ms unit ms
    */
    List<Number> GetSystemClock();

Get the current joint configuration of the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get the current joint configuration of the robot
    * @return List[0]:int error code; List[1]:int config joint space configuration, range [0~7]
    */
    List<Integer> GetRobotCurJointsConfig();

Get the default speed of the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get the default speed of the robot
    * @return List[0]: int error code; List[1]: double vel velocity in mm/s
    */  
    List<Number> GetDefaultTransVel(); 

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        List<Integer> rtnArr = robot.GetRobotCurJointsConfig();
        System.out.println("config is " + rtnArr.get(1));

        List<Number> rtnArrN = robot.GetSystemClock();
        System.out.println("systom clock is " + rtnArrN.get(1));

        rtnArrN = robot.GetDefaultTransVel();
        System.out.println("Robot's current speed is: " + rtnArrN.get(1));
    }

Query Robot Teaching Management Point Data
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Query robotics instructional management point data 
    * @param [in] name point name
    * @return List[0]: error code; List[1] - List[20] : point data double[20]{x,y,z,rx,ry,rz,j1,j2,j3,j4,j5,j6,tool,wobj,speed,acc,e1,e2,e3,e4} 
    */ 
    List<Number> GetRobotTeachingPoint(String name). 

Get SSH public key
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Getting SSH public key 
    * @param [in] keygen public key 
    * @return error code    
    */ 
    List<Number> GetRobotTeachingPoint(String name). 

Calculate the MD5 value of a file in a specified path
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Calculates the MD5 value of a file in a specified path. 
    * @param [in] file_path file path contains the file name, the default Traj folder path is: "/fruser/traj/", such as "/fruser/traj/trajHelix_aima_1.txt". 
    * @return error code   
    */ 
    int ComputeFileMD5(String file_path, String md5); 

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection successful");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }

        List<Number> rtnArr = robot.GetRobotTeachingPoint("P1");
        System.out.println("point data " + rtnArr);

        String[] key = {""};
        robot.GetSSHKeygen(key);
        System.out.println("ssh key " + key[0]);
    }

Getting the robot software version
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get robot software version
    * @param [out] robotModel robotModel
    * @param [out] webVersion web version
    * @param [out] controllerVersion controllerVersion
    * @return error code 
    */
    int GetSoftwareVersion(String robotModel, String webVersion, String controllerVersion);

Getting the robot hardware version
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get robot hardware version
    * @param [out] ctrlBoxBoardVersion Control Box Board Hardware Version
    * @param [out] driver1Version driver1 hardware version
    * @param [out] driver1Version driver2HardwareVersion
    * @param [out] driver1Version driver3 hardware version
    * @param [out] driver1Version driver4 hardware version
    * @param [out] driver1Version driver5 hardware version
    * @param [out] driver1Version driver6 hardware version
    * @param [out] endBoardVersion endBoardHardwareVersion
    * @return error code 
    */
    int GetHardwareVersion(String ctrlBoxBoardVersion, String driver1Version, String driver2Version, String driver3Version,
                                          String driver4Version, String driver5Version, String driver6Version, String endBoardVersion);

Getting the robot firmware version
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Get robot firmware version
    * @param [out] ctrlBoxBoardVersion Control Box Board Firmware Version
    * @param [out] driver1Version driver1 firmware version
    * @param [out] driver1Version driver2FirmwareVersion
    * @param [out] driver1Version driver3 firmware version
    * @param [out] driver1Version driver4 firmware version
    * @param [out] driver1Version driver5 firmware version
    * @param [out] driver1Version driver6 firmware version
    * @param [out] endBoardVersion endBoardFirmwareVersion
    * @return error code 
    */
    int GetFirmwareVersion(String ctrlBoxBoardVersion, String driver1Version, String driver2Version, String driver3Version,
                                          String driver4Version, String driver5Version, String driver6Version, String endBoardVersion);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        String ctrlBoxBoardVersion = "";
        String driver1Version = "";
        String driver2Version = "";
        String driver3Version = "";
        String driver4Version = "";
        String driver5Version = "";
        String driver6Version = "";
        String endBoardVersion = "";
        robot.GetHardwareVersion(ctrlBoxBoardVersion ,driver1Version, driver2Version, driver3Version,)
                 driver4Version, driver5Version, driver6Version, endBoardVersion).

        robot.GetFirmwareVersion(ctrlBoxBoardVersion, driver1Version, driver2Version, driver3Version,
                driver4Version, driver5Version, driver6Version, endBoardVersion).
    }