Common Robot Settings
=====================================

.. toctree:: 
    :maxdepth: 5

Setting the global speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the global speed
    * @param [in] vel velocity percentage, range [0~100]
    * @return error code
    */
    int SetSpeed(int vel). 

Setting system variable values
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting system variable values
    * @param [in] id Variable number in the range [1~20].
    * @param [in] value Variable value
    * @return error code
    */
    int SetSysVarValue(int id, double value); 

Setting Tool Reference Points - Six-Point Method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting Tool Reference Points -- Six-Point Approach
    * @param [in] point_num point_num, range [1~6].
    * @return error code 
    */ 
    int SetToolPoint(int point_num). 

Calculation tool coordinate system - six-point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Calculate the tool coordinate system
    * @param [out] tcp_pose tool coordinate system
    * @return error code 
    */ 
    int ComputeTool(DescPose tcp_pose). 

Setting Tool Reference Points - Four Point Method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting Tool Reference Points -- Four-Point Approach
    * @param [in] point_num point_num, range [1~4].
    * @return error code 
    */ 
    int SetTcp4RefPoint(int point_num).

Calculation Tool Coordinate System - Four Point Method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Calculate the tool coordinate system
    * @param [out] tcp_pose tool coordinate system
    * @return error code 
    */ 
    int ComputeTcp4(DescPose tcp_pose).

Setting the tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting the tool coordinate system 
    * @param [in] id coordinate system number in the range [0 to 14].
    * @param [in] coord Tool center point relative to end flange center position
    * @param [in] type 0-tool coordinate system, 1-sensor coordinate system
    * @param [in] install install position, 0 - end of robot, 1 - outside of robot
    * @param [in] toolID toolID
    * @param [in] loadNum loadNumber
    * @return error code 
    */ 
    int SetToolCoord(int id, DescPose coord, int type, int install, int toolID, int loadNum);;  

Setting the tool coordinate system list
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief sets up a list of tool coordinate systems
    * @param [in] id coordinate system number in the range [0 to 14].
    * @param [in] coord Tool center point relative to end flange center orientation
    * @param [in] type 0-tool coordinate system, 1-sensor coordinate system
    * @param [in] install install position, 0 - end of robot, 1 - outside of robot
    * @param [in] loadNum loadNumber
    * @return error code
    */
    int SetToolList(int id, DescPose coord, int type, int install, int loadNum);;  

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
        robot.Mode(1);
        robot.SetSpeed(20);
        robot.Mode(0);

        for(int i = 1; i < 10; i++)
        {
            robot.SetSysVarValue(i, i * 10);
        }
        for(int i = 1; i < 10; i++)
        {
            List<Number> rtnArr = robot.GetSysVarValue(i);//get the system variable
            System.out.println("SysVarValue " + i + " is " + rtnArr.get(1));
        }

        JointPos jp1=new JointPos(-89.407,-148.279,-83.169,-45.689,133.689,41.705);
        JointPos jp2=new JointPos(-67.595,-143.7,-88.006,-48.514,57.073,56.189);
        JointPos jp3=new JointPos(-88.229,-152.355,-67.815,-78.07,129.029,58.739);
        JointPos jp4=new JointPos(-77.528,-141.519,-89.826,-37.184,90.274,41.769);
        JointPos jp5=new JointPos(-76.744,-138.219,-97.714,-32.595,90.255,42.558);
        JointPos jp6=new JointPos(-77.595,-138.454,-90.065,-40.014,90.275,41.709);
        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();

        DescPose desc_p1 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        DescPose desc_p2 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        DescPose desc_p3 = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);
        DescPose desc_p4 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        DescPose desc_p5 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        DescPose desc_p6 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        robot.GetForwardKin(jp1, desc_p1);
        robot.GetForwardKin(jp2, desc_p2);
        robot.GetForwardKin(jp3, desc_p3);
        robot.GetForwardKin(jp4, desc_p4);
        robot.GetForwardKin(jp5, desc_p5);
        robot.GetForwardKin(jp6, desc_p6);
        robot.MoveJ(jp1, desc_p1,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
        robot.SetToolPoint(1);

        robot.MoveJ(jp2, desc_p2,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
        robot.SetToolPoint(2);

        robot.MoveJ(jp3, desc_p3,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
        robot.SetToolPoint(3);

        robot.MoveJ(jp4, desc_p4,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
        robot.SetToolPoint(4);

        robot.MoveJ(jp5, desc_p5,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
        robot.SetToolPoint(5);

        robot.MoveJ(jp6, desc_p6,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
        robot.SetToolPoint(6);

        DescPose coord = new DescPose();
        robot.ComputeTool(coord);
        System.out.println("result is " + coord.tran.x + " " + coord.tran.y + " " + coord.tran.z + " " + coord.rpy.rx + " " + coord.rpy.ry + " " + coord.rpy.rz);

        robot.SetToolCoord(5, coord, 0, 0,0,0);//set tool coordinate system
        robot.SetToolList(5, coord, 0, 0, 0);
    }

Setting External Tool Coordinate Reference Points
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting external tool reference points -- the three-point method 
    * @param [in] point_num point_number, range [1~3].
    * @return error code 
    */ 
    int SetExTCPPoint(int point_num). 

Calculation of the external tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:
    
    /** 
    * @brief Calculating the external tool coordinate system -- three-point method
    * @param [out] tcp_pose external tool coordinate system
    * @return error code 
    */ 
    int ComputeExTCF(DescPose tcp_pose). 

Setting the external tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the external tool coordinate system 
    * @param [in] id coordinate system number in the range [0 to 14].
    * @param [in] etcp Tool center point relative to end flange center position
    * @param [in] etool TBD
    * @return error code 
    */
    int SetExToolCoord(int id, DescPose etcp, DescPose etool);; 

Setting up a list of external tool coordinate systems
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting up a list of external tool coordinate systems
    * @param [in] id coordinate system number in the range [0 to 14].
    * @param [in] etcp Tool center point relative to end flange center position
    * @param [in] etool TBD
    * @return error code
    */
    int SetExToolList(int id, DescPose etcp, DescPose etool);; 

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
        robot.Mode(1);
        robot.SetSpeed(20);
        robot.Mode(0);

        for(int i = 1; i < 10; i++)
        {
            robot.SetSysVarValue(i, i * 10);
        }
        for(int i = 1; i < 10; i++)
        {
            List<Number> rtnArr = robot.GetSysVarValue(i);//get the system variable
            System.out.println("SysVarValue " + i + " is " + rtnArr.get(1));
        }

        JointPos j1 = new JointPos(-84.787, -152.056,-75.689 , -37.899, 94.486,41.709);
        JointPos j2 = new JointPos(-79.438,-152.139,-75.634,-37.469,94.065,47.058);
        JointPos j3 = new JointPos(-84.788,-145.179,-77.119,-43.345,94.487,41.709);


        DescPose desc_p1 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        DescPose desc_p2 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        DescPose desc_p3 = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);

        robot.GetForwardKin(j1, desc_p1);
        robot.GetForwardKin(j2, desc_p2);
        robot.GetForwardKin(j3, desc_p3);

        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();

        robot.MoveJ(j1, desc_p1,0, 0, 20, 100, 100, epos, -1, 0, offset_pos);
        robot.SetExTCPPoint(1);

        robot.MoveJ(j2, desc_p2,0, 0, 20, 100, 100, epos, -1, 0, offset_pos);
        robot.SetExTCPPoint(2);

        robot.MoveJ(j3, desc_p3,0, 0, 20, 100, 100, epos, -1, 0, offset_pos);
        robot.SetExTCPPoint(3);

        DescPose coordE = new DescPose();
        robot.ComputeExTCF(coordE).
        System.out.println("result is " + coordE.tran.x + " " + coordE.tran.y + " " + coordE.tran.z + " " + coordE.rpy.rx + " " + coordE.rpy.ry + " " + coordE.rpy.rz);

        robot.SetExToolCoord(5, coordE, coordE); robot.
        robot.SetExToolList(5,coordE, coordE); robot.
    }

Setting the reference point of the workpiece coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Setting the workpiece reference point - three-point method 
    * @param [in] point_num point_number, range [1~3].
    * @return error code 
    */ 
    int SetWObjCoordPoint(int point_num). 

Calculation of the workpiece coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Calculate workpiece coordinate system
    * @param [in] method Calculation method 0: origin-x-axis-z-axis 1: origin-x-axis-xy plane
    * @param [in] refFrame reference coordinate system
    * @param [out] wobj_pose Workpiece coordinate system
    * @return error code 
    */ 
    int ComputeWObjCoord(int method, int refFrame, DescPose wobj_pose); 

Setting the workpiece coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting the workpiece coordinate system
    * @param [in] id coordinate system number, range [1-15]
    * @param [in] coord Workpiece coordinate system with respect to the center of the end flange.
    * @param [in] refFrame reference coordinate system
    * @return error code
    */  
    int SetWObjCoord(int id, DescPose coord, int refFrame);

Setting the list of workpiece coordinate systems
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting up a list of workpiece coordinate systems
    * @param [in] id coordinate system number, range [1-15]
    * @param [in] coord Workpiece coordinate system with respect to the center of the end flange.
    * @param [in] refFrame reference coordinate system
    * @return error code
    */  
    int SetWObjList(int id, DescPose coord, int refFrame);

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

        JointPos j1 = new JointPos(-84.787, -152.056,-75.689,-37.899,94.486,41.709);
        JointPos j2 = new JointPos(-79.438,-152.139,-75.634,-37.469,94.065,47.058);
        JointPos j3 = new JointPos(-84.788,-145.179,-77.119,-43.345,94.487,41.709);
        DescPose desc_p1 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        DescPose desc_p2 = new DescPose(0, 0, 0, 0, 0, 0, 0);
        DescPose desc_p3 = new DescPose(0, 0, 0, 0, 0, 0, 0, 0);

        robot.GetForwardKin(j1, desc_p1);
        robot.GetForwardKin(j2, desc_p2);
        robot.GetForwardKin(j3, desc_p3);

        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();

        robot.MoveJ(j1, desc_p1,0, 0, 20, 100, 100, epos, -1, 0, offset_pos);
        robot.SetWObjCoordPoint(1);

        robot.MoveJ(j2, desc_p2,0, 0, 20, 100, 100, epos, -1, 0, offset_pos);
        robot.SetWObjCoordPoint(2);

        robot.MoveJ(j3, desc_p3,0, 0, 20, 100, 100, epos, -1, 0, offset_pos);
        robot.SetWObjCoordPoint(3);

        DescPose coordE = new DescPose();
        robot.ComputeWObjCoord(0, coordE);
        System.out.println("result is " + coordE.tran.x + " " + coordE.tran.y + " " + coordE.tran.z + " " + coordE.rpy.rx + " " + coordE.rpy.ry + " " + coordE.rpy.rz);

        robot.SetWObjCoord(5, coordE,0);
        robot.SetWObjList(5,coordE,0);
    }

Setting the end load weight
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting of end load weights
    * @param [in] weight Load weight in kg
    * @return error code
    */
    int SetLoadWeight(double weight).

Setting the end load center of mass coordinates
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting end-load center of mass coordinates
    * @param [in] coord coordinates of center of mass in mm
    * @return error code
    */
    int SetLoadCoord(DescTran coord). 

Setting the robot installation method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting up the robot installation method
    * @param [in] install Installation method, 0-front, 1-side, 2-inverted
    * @return error code
    */
    int SetRobotInstallPos(int install). 

Setting the robot mounting angle
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting robot mounting angle, free mounting
    * @param [in] yangle Tilt angle
    * @param [in] zangle rotation angle
    * @return error code
    */
    int SetRobotInstallAngle(double yangle, double zangle); 

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
        robot.SetLoadWeight(2);
        robot.SetLoadCoord(new DescTran(1.0, 2.0, 3.0));
        robot.SetRobotInstallPos(0);
        robot.SetRobotInstallAngle(0, 0);
    }

Waiting for a specified time
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Waiting for specified time
    * @param [in] t_ms Unit ms
    * @return error code
    */
    int WaitMs(int t_ms).

Setting robot acceleration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Setting robot acceleration
    * @param [in] acc Robot acceleration percentage
    * @return error code
    */
    int SetOaccScale(double acc).