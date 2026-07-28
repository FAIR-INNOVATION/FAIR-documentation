Robot common settings
==================================

.. toctree::
    :maxdepth: 5

Set tool reference point - six point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set tool reference point - six point method
    * @param [in] point_num Point number, range [1~6]
    * @return Error code
    */
    int SetToolPoint(int point_num);

Calculate tool coordinate system - six point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Calculate tool coordinate system
    * @param [out] tcp_pose Tool coordinate system
    * @return Error code
    */
    int ComputeTool(DescPose tcp_pose);

Set tool reference point - four point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set tool reference point - four point method
    * @param [in] point_num Point number, range [1~4]
    * @return Error code
    */
    int SetTcp4RefPoint(int point_num);

Calculate tool coordinate system - four point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Calculate tool coordinate system
    * @param [out] tcp_pose Tool coordinate system
    * @return Error code
    */
    int ComputeTcp4(DescPose tcp_pose);

Calculate tool coordinate system based on point information
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Calculate tool coordinate system based on point information
    * @param [in] method Calculation method; 0-four point method; 1-six point method
    * @param [in] pos Joint position group, array length is 4 for four point method, 6 for six point method
    * @param [out] tool_pose Output tool coordinate system
    * @return Error code
    */
    int ComputeToolCoordWithPoints(int method, JointPos[] pos, DescPose tool_pose);

Set tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set tool coordinate system
    * @param [in] id Coordinate system number, range [0~14]
    * @param [in] coord Tool center point pose relative to end flange center
    * @param [in] type 0-tool coordinate system, 1-sensor coordinate system
    * @param [in] install Installation position, 0-robot end, 1-external to robot
    * @param [in] toolID Tool ID
    * @param [in] loadNum Load number
    * @return Error code
    */
    int SetToolCoord(int id, DescPose coord, int type, int install, int toolID, int loadNum);

Set tool coordinate system list
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set tool coordinate system list
    * @param [in] id Coordinate system number, range [0~14]
    * @param [in] coord Tool center point pose relative to end flange center
    * @param [in] type 0-tool coordinate system, 1-sensor coordinate system
    * @param [in] install Installation position, 0-robot end, 1-external to robot
    * @param [in] loadNum Load number
    * @return Error code
    */
    int SetToolList(int id, DescPose coord, int type, int install, int loadNum);

Get current tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get current tool coordinate system
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] desc_pos Tool coordinate system pose
    * @return Error code
    */
    int GetTCPOffset(int flag, DescPose desc_pos);

Robot tool coordinate system operation code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestTCPCompute(Robot robot)
    {
        DescPose p1Desc=new DescPose(186.331, 487.913, 209.850, 149.030, 0.688, -114.347);
        JointPos p1Joint=new JointPos(-127.876, -75.341, 115.417, -122.741, -59.820, 74.300);

        DescPose p2Desc=new DescPose(69.721, 535.073, 202.882, -144.406, -14.775, -89.012);
        JointPos p2Joint=new JointPos(-101.780, -69.828, 110.917, -125.740, -127.841, 74.300);

        DescPose p3Desc=new DescPose(146.861, 578.426, 205.598, 175.997, -36.178, -93.437);
        JointPos p3Joint=new JointPos(-112.851, -60.191, 86.566, -80.676, -97.463, 74.300);

        DescPose p4Desc=new DescPose(136.284, 509.876, 225.613, 178.987, 1.372, -100.696);
        JointPos p4Joint=new JointPos(-116.397, -76.281, 113.845, -128.611, -88.654, 74.299);

        DescPose p5Desc=new DescPose(138.395, 505.972, 298.016, 179.134, 2.147, -101.110);
        JointPos p5Joint=new JointPos(-116.814, -82.333, 109.162, -118.662, -88.585, 74.302);

        DescPose p6Desc=new DescPose(105.553, 454.325, 232.017, -179.426, 0.444, -99.952);
        JointPos p6Joint=new JointPos(-115.649, -84.367, 122.447, -128.663, -90.432, 74.303);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        JointPos[] posJ = { p1Joint , p2Joint , p3Joint , p4Joint , p5Joint , p6Joint };
        DescPose coordRtn =new DescPose() {};
        int rtn = robot.ComputeToolCoordWithPoints(1, posJ, coordRtn);

        robot.MoveJ(p1Joint, p1Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetToolPoint(1);
        robot.MoveJ(p2Joint, p2Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetToolPoint(2);
        robot.MoveJ(p3Joint, p3Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetToolPoint(3);
        robot.MoveJ(p4Joint, p4Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetToolPoint(4);
        robot.MoveJ(p5Joint, p5Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetToolPoint(5);
        robot.MoveJ(p6Joint, p6Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetToolPoint(6);
        rtn = robot.ComputeTool(coordRtn);
        robot.SetToolList(3, coordRtn, 0, 0, 0);

        robot.MoveJ(p1Joint, p1Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(1);
        robot.MoveJ(p2Joint, p2Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(2);
        robot.MoveJ(p3Joint, p3Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(3);
        robot.MoveJ(p4Joint, p4Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(4);
        rtn = robot.ComputeTcp4(coordRtn);

        robot.SetToolCoord(4, coordRtn, 0, 0, 1, 0);

        DescPose getCoord = new DescPose(){};
        rtn = robot.GetTCPOffset(0, getCoord);
        return 0;
    }

Set external tool reference point - six point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set external tool reference point - three point method
    * @param [in] point_num Point number, range [1~3]
    * @return Error code
    */
    int SetExTCPPoint(int point_num);

Calculate external tool coordinate system - six point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Calculate external tool coordinate system - three point method
    * @param [out] tcp_pose External tool coordinate system
    * @return Error code
    */
    int ComputeExTCF(DescPose tcp_pose);

Set external tool coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set external tool coordinate system
    * @param [in] id Coordinate system number, 20-39 correspond to external tool coordinate systems 0-19
    * @param [in] etcp Tool center point pose relative to end flange center
    * @param [in] etool To be determined
    * @return Error code
    */
    int SetExToolCoord(int id, DescPose etcp, DescPose etool);

Set external tool coordinate system list
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set external tool coordinate system list
    * @param [in] id Coordinate system number, 20-39 correspond to external tool coordinate systems 0-19
    * @param [in] etcp Tool center point pose relative to end flange center
    * @param [in] etool To be determined
    * @return Error code
    */
    int SetExToolList(int id, DescPose etcp, DescPose etool);

Robot external tool coordinate system operation code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestExtCoord(Robot robot)
    {
        DescPose p1Desc=new DescPose(-89.606, 779.517, 193.516, 178.000, 0.476, -92.484);
        JointPos p1Joint=new JointPos(-108.145, -50.137, 85.818, -125.599, -87.946, 74.329);

        DescPose p2Desc=new DescPose(-24.656, 850.384, 191.361, 177.079, -2.058, -95.355);
        JointPos p2Joint=new JointPos(-111.024, -41.538, 69.222, -114.913, -87.743, 74.329);

        DescPose p3Desc=new DescPose(-99.813, 766.661, 241.878, -176.817, 1.917, -91.604);
        JointPos p3Joint=new JointPos(-107.266, -56.116, 85.971, -122.560, -92.548, 74.331);

        robot.GetForwardKin(p1Joint,p1Desc);
        robot.GetForwardKin(p2Joint,p2Desc);
        robot.GetForwardKin(p3Joint,p3Desc);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        DescPose[] posTCP = { p1Desc , p2Desc , p3Desc };
        DescPose coordRtn = new DescPose();

        robot.MoveJ(p1Joint, p1Desc, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetExTCPPoint(1);
        robot.MoveJ(p2Joint, p2Desc, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetExTCPPoint(2);
        robot.MoveJ(p3Joint, p3Desc, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetExTCPPoint(3);
        int rtn = robot.ComputeExTCF(coordRtn);

        robot.SetExToolCoord(1, coordRtn, offdese);
        robot.SetExToolList(1, coordRtn, offdese);
        return 0;
    }

Set workpiece reference point - three point method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set workpiece reference point - three point method
    * @param [in] point_num Point number, range [1~3]
    * @return Error code
    */
    int SetWObjCoordPoint(int point_num);

Calculate workpiece coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Calculate workpiece coordinate system
    * @param [in] method Calculation method 0: origin-x axis-z axis 1: origin-x axis-xy plane
    * @param [in] refFrame Reference coordinate system
    * @param [out] wobj_pose Workpiece coordinate system
    * @return Error code
    */
    int ComputeWObjCoord(int method, int refFrame, DescPose wobj_pose);

Set workpiece coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set workpiece coordinate system
    * @param [in] id Coordinate system number, range [1~15]
    * @param [in] coord Workpiece coordinate system pose relative to end flange center
    * @param [in] refFrame Reference coordinate system
    * @return Error code
    */
    int SetWObjCoord(int id, DescPose coord, int refFrame);

Set workpiece coordinate system list
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set workpiece coordinate system list
    * @param [in] id Coordinate system number, range [1~15]
    * @param [in] coord Workpiece coordinate system pose relative to end flange center
    * @param [in] refFrame Reference coordinate system
    * @return Error code
    */
    int SetWObjList(int id, DescPose coord, int refFrame);

Calculate workpiece coordinate system based on point information
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Calculate workpiece coordinate system based on point information
    * @param [in] method Calculation method; 0: origin-x axis-z axis 1: origin-x axis-xy plane
    * @param [in] pos Three TCP position groups
    * @param [in] refFrame Reference coordinate system
    * @param [in] tcp_pose Output workpiece coordinate system
    * @return Error code
    */
    int ComputeWObjCoordWithPoints(int method, DescPose[] pos, int refFrame,DescPose tcp_pose);

Get current workpiece coordinate system
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get current workpiece coordinate system
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] desc_pos Workpiece coordinate system pose
    * @return Error code
    */
    int GetWObjOffset(int flag, DescPose desc_pos);

Workpiece coordinate system operation code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestWobjCoord(Robot robot)
    {
        DescPose p1Desc=new DescPose(-89.606, 779.517, 193.516, 178.000, 0.476, -92.484);
        JointPos p1Joint=new JointPos(-108.145, -50.137, 85.818, -125.599, -87.946, 74.329);

        DescPose p2Desc=new DescPose(-24.656, 850.384, 191.361, 177.079, -2.058, -95.355);
        JointPos p2Joint=new JointPos(-111.024, -41.538, 69.222, -114.913, -87.743, 74.329);

        DescPose p3Desc=new DescPose(-99.813, 766.661, 241.878, -176.817, 1.917, -91.604);
        JointPos p3Joint=new JointPos(-107.266, -56.116, 85.971, -122.560, -92.548, 74.331);

        robot.GetForwardKin(p1Joint,p1Desc);
        robot.GetForwardKin(p2Joint,p2Desc);
        robot.GetForwardKin(p3Joint,p3Desc);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        DescPose[] posTCP =new DescPose[]{p1Desc , p2Desc , p3Desc };
        DescPose coordRtn =new DescPose();
        int rtn = robot.ComputeWObjCoordWithPoints(1, posTCP, 0, coordRtn);

        robot.MoveJ(p1Joint, p1Desc, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetWObjCoordPoint(1);
        robot.MoveJ(p2Joint, p2Desc, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetWObjCoordPoint(2);
        robot.MoveJ(p3Joint, p3Desc, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetWObjCoordPoint(3);
        rtn = robot.ComputeWObjCoord(1, 0, coordRtn);

        robot.SetWObjCoord(1, coordRtn, 0);
        robot.SetWObjList(1, coordRtn, 0);

        DescPose getWobjDesc = new DescPose();
        rtn = robot.GetWObjOffset(0, getWobjDesc);
        return 0;
    }

Set global speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set global speed
    * @param [in] vel Speed percentage, range [0~100]
    * @return Error code
    */
    int SetSpeed(int vel);

Set robot acceleration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set robot acceleration
    * @param [in] acc Robot acceleration percentage
    * @return Error code
    */
    int SetOaccScale(double acc);

Get robot default speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get robot default speed
    * @return List[0]:int Error code; List[1]: double vel Speed, unit mm/s
    */
    List<Number> GetDefaultTransVel();

Set end load weight
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set end load weight
    * @param [in] loadNum Load number
    * @param [in] weight Load weight, unit kg
    * @return Error code
    */
    int SetLoadWeight(int loadNum,double weight);

Set end load center of mass coordinates
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set end load center of mass coordinates
    * @param [in] coord Center of mass coordinates, unit mm
    * @return Error code
    */
    int SetLoadCoord(DescTran coord);

Set End Effector Load Center of Mass Coordinates
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: java
   :linenos:

   /**
    * @brief  Set end effector load center of mass coordinates
    * @param  [in] loadNum Load number
    * @param  [in] coord Center of mass coordinates, unit: mm
    * @return  Error code
    */
   public int SetLoadCoord(int loadNum, DescTran coord)

Get current load weight
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get current load weight
    * @param [in] flag 0-blocking, 1-non-blocking
    * @return List[0]:int Error code; List[1]: double weight Load weight, unit kg
    */
    List<Number> GetTargetPayload(int flag);

Get current load center of mass
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get current load center of mass
    * @param [in] flag 0-blocking, 1-non-blocking
    * @param [out] cog Load center of mass, unit mm
    * @return Error code
    */
    int GetTargetPayloadCog(int flag, DescTran cog);

Set robot installation method
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set robot installation method
    * @param [in] install Installation method, 0-standard, 1-side, 2-inverted
    * @return Error code
    */
    int SetRobotInstallPos(int install);

Set robot installation angle
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set robot installation angle, free installation
    * @param [in] yangle Tilt angle
    * @param [in] zangle Rotation angle
    * @return Error code
    */
    int SetRobotInstallAngle(double yangle, double zangle);

Get robot installation angle
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get robot installation angle
    * @return List[0]:Error code; List[1]:double yangle Tilt angle; List[2]:double zangle Rotation angle
    */
    public List<Number> GetRobotInstallAngle()

Set system variable value
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set system variable value
    * @param [in] id Variable number, range [1~20]
    * @param [in] value Variable value
    * @return Error code
    */
    int SetSysVarValue(int id, double value);

Get system variable value
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get system variable value
    * @param [in] id System variable number, range [1~20]
    * @return List[0]:Error code; List[1]:double value System variable value
    */
    List<Number> GetSysVarValue(int id);

Robot common settings code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestLoadInstall(Robot robot)
    {
        for (int i = 1; i < 100; i++)
        {
            robot.SetSpeed(i);
            robot.SetOaccScale(i);
            robot.Sleep(30);
        }

        List<Number> defaultVel=new ArrayList<>();

        defaultVel=robot.GetDefaultTransVel();
        System.out.println("GetDefaultTransVel is:"+ defaultVel.get(1));

        for (int i = 1; i < 21; i++)
        {
            robot.SetSysVarValue(i, i + 0.5);
            robot.Sleep(100);
        }

        for (int i = 1; i < 21; i++)
        {
            float value = 0;
            defaultVel=robot.GetSysVarValue(i);
            System.out.println("sys value :"+i+", is :"+defaultVel.get(1));
            robot.Sleep(100);
        }

        robot.SetLoadWeight(0, 2.5);

        DescTran loadCoord = new DescTran();
        loadCoord.x = 3.0;
        loadCoord.y = 4.0;
        loadCoord.z = 5.0;
        robot.SetLoadCoord(loadCoord);

        robot.Sleep(1000);

        List<Number> getLoad = new ArrayList<>();

        getLoad=robot.GetTargetPayload(0);

        DescTran getLoadTran =new DescTran();
        robot.GetTargetPayloadCog(0, getLoadTran);
        System.out.println("get load is:"+getLoad.get(1)+", get load cog is: "+getLoadTran.x+","+getLoadTran.y+","+getLoadTran.z);

        robot.SetRobotInstallPos(0);
        robot.SetRobotInstallAngle(15.0, 25.0);

        List<Number> angle=new ArrayList<>();
        angle=robot.GetRobotInstallAngle();
        System.out.println("GetRobotInstallAngle x:"+angle.get(1)+";  y:"+angle.get(2));

        robot.CloseRPC();
        return 0;
    }

Joint friction compensation switch
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Joint friction compensation switch
    * @param [in] state 0-off, 1-on
    * @return Error code
    */
    int FrictionCompensationOnOff(int state);

Set joint friction compensation coefficient - standard installation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set joint friction compensation coefficient - standard installation
    * @param [in] coeff Six joint compensation coefficients, range [0~1]
    * @return Error code
    */
    int SetFrictionValue_level(Object[] coeff);

Set joint friction compensation coefficient - side installation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set joint friction compensation coefficient - side installation
    * @param [in] coeff Six joint compensation coefficients, range [0~1]
    * @return Error code
    */
    int SetFrictionValue_wall(Object[] coeff);

Set joint friction compensation coefficient - inverted installation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set joint friction compensation coefficient - inverted installation
    * @param [in] coeff Six joint compensation coefficients, range [0~1]
    * @return Error code
    */
    int SetFrictionValue_ceiling(Object[] coeff);

Set joint friction compensation coefficient - free installation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set joint friction compensation coefficient - free installation
    * @param [in] coeff Six joint compensation coefficients, range [0~1]
    * @return Error code
    */
    int SetFrictionValue_freedom(Object[] coeff);

Robot set joint friction compensation code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestFriction(Robot robot)
    {

        Object[] lcoeff = { 0.9,0.9,0.9,0.9,0.9,0.9 };
        Object[] wcoeff = { 0.4,0.4,0.4,0.4,0.4,0.4 };
        Object[] ccoeff = { 0.6,0.6,0.6,0.6,0.6,0.6 };
        Object[] fcoeff = { 0.5,0.5,0.5,0.5,0.5,0.5 };

        int rtn = robot.FrictionCompensationOnOff(1);
        System.out.println("FrictionCompensationOnOff rtn is:"+ rtn);

        rtn = robot.SetFrictionValue_level(lcoeff);
        System.out.println("SetFrictionValue_level rtn is:"+ rtn);

        rtn = robot.SetFrictionValue_wall(wcoeff);
        System.out.println("SetFrictionValue_wall rtn is:"+ rtn);

        rtn = robot.SetFrictionValue_ceiling(ccoeff);
        System.out.println("SetFrictionValue_ceiling rtn is:"+ rtn);

        rtn = robot.SetFrictionValue_freedom(fcoeff);
        System.out.println("SetFrictionValue_freedom rtn is:"+ rtn);

        robot.CloseRPC();
        return 0;
    }

Query robot error code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Query robot error code
     * @param [out] maincode Main error code
     * @param [out] subcode Sub error code
     * @return Error code
     */
    int GetRobotErrorCode(int[] maincode, int[] subcode);

Error state clear
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Error state clear
    * @return Error code
    */
    int ResetAllError();

Robot fault state get and clear error code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestGetError(Robot robot)
    {
        int[] maincode={0}, subcode={0};
        robot.GetRobotErrorCode(maincode, subcode);

        robot.ResetAllError();

        robot.Sleep(1000);

        robot.GetRobotErrorCode(maincode, subcode);
        return 0;
    }

Set wide voltage control box temperature and fan speed monitoring parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.6-3.8.3

.. code-block:: Java
    :linenos:

    /**
    * @brief Set wide voltage control box temperature and fan speed monitoring parameters
    * @param [in] enable 0-disable monitoring; 1-enable monitoring
    * @param [in] period Monitoring period(s), range 1-100
    * @return Error code
    */
    int SetWideBoxTempFanMonitorParam(int enable, int period);

Get wide voltage control box temperature and fan speed monitoring parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.6-3.8.3

.. code-block:: Java
    :linenos:

    /**
    * @brief Get wide voltage control box temperature and fan speed monitoring parameters
    * @return List[0]-Error code,List[1]-enable 0-disable monitoring; 1-enable monitoring,List[2]-period Monitoring period(s), range 1-100
    */
    List<Number> GetWideBoxTempFanMonitorParam()

Wide voltage control box temperature and fan current state get code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void TestWideVoltageCtrlBoxtemp(Robot robot)
    {
        robot.SetWideBoxTempFanMonitorParam(1, 2);
        List<Number> list=robot.GetWideBoxTempFanMonitorParam();
        System.out.println("GetWideBoxTempFanMonitorParam enable is:"+list.get(1)+", period is:"+list.get(2));
        ROBOT_STATE_PKG pkg=new ROBOT_STATE_PKG();
        for (int i = 0; i < 100; i++)
        {
            pkg=robot.GetRobotRealTimeState();
            System.out.println("robot ctrl box temp is:"+pkg.wideVoltageCtrlBoxTemp+",fan current is:"+pkg.wideVoltageCtrlBoxFanCurrent);
            robot.Sleep(100);
        }

        int rtn = robot.SetWideBoxTempFanMonitorParam(0, 2);

        list=robot.GetWideBoxTempFanMonitorParam();
        for (int i = 0; i < 100; i++)
        {
            pkg=robot.GetRobotRealTimeState();
            System.out.println("robot ctrl box temp is:"+pkg.wideVoltageCtrlBoxTemp+" ,fan current is:"+pkg.wideVoltageCtrlBoxFanCurrent);
            robot.Sleep(100);
        }

        robot.CloseRPC();
        robot.Sleep(2000);
        return 0;
    }

Set Focus Calibration Point
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.7-3.8.4

.. code-block:: Java
    :linenos:

    /**
    * @brief Set focus calibration point
    * @param [in] pointNum Focus calibration point number (1-8)
    * @param [in] point    Calibration point coordinates
    * @return Error code
    */
    int SetFocusCalibPoint(int pointNum, DescPose point)

Compute Focus Calibration Result
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.7-3.8.4

.. code-block:: Java
    :linenos:

    /**
    * @brief Compute focus calibration result
    * @param [in] pointNum  Number of calibration points
    * @param [in] resultPos Calibration result XYZ
    * @param [out] accuracy  Calibration accuracy error
    * @return Error code
    */
    int ComputeFocusCalib(int pointNum, DescTran resultPos, double[] accuracy)

Start focus following
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.7-3.8.4

.. code-block:: Java
    :linenos:

    /**
    * @brief Start focus following
    * @param [in] kp       Proportional parameter (default: 50.0)
    * @param [in] kpredict Feedforward parameter (default: 19.0)
    * @param [in] aMax     Maximum angular acceleration limit (default: 1440°/s²)
    * @param [in] vMax     Maximum angular velocity limit (default: 180°/s)
    * @param [in] type     Lock X-axis direction (0-Reference input vector; 1-Horizontal; 2-Vertical)
    * @return Error code
    */
    int FocusStart(double kp, double kpredict, double aMax, double vMax, int type)

Stop Focus Following
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.7-3.8.4

.. code-block:: Java
    :linenos:

    /**
    * @brief Stop focus following
    * @return Error code
    */
    int FocusEnd()

Set Focus Position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: Java SDK-v1.0.7-3.8.4

.. code-block:: Java
    :linenos:
    
    /**
    * @brief Set focus position
    * @param pos Focus position XYZ
    * @return Error code
    */
    public int SetFocusPosition(DescTran pos)

Focus Following Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void TestFocus(Robot robot){
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

        robot.GetForwardKin(p1Joint, p1Desc);
        robot.GetForwardKin(p2Joint,  p2Desc);
        robot.GetForwardKin(p3Joint,  p3Desc);
        robot.GetForwardKin(p4Joint,  p4Desc);
        robot.GetForwardKin(p5Joint,  p5Desc);
        robot.GetForwardKin(p6Joint,  p6Desc);

        robot.MoveJ(p1Joint, p1Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(1);
        robot.MoveJ(p2Joint, p2Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(2);
        robot.MoveJ(p3Joint, p3Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(3);
        robot.MoveJ(p4Joint, p4Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(4);

        DescPose coordRtn = new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        int rtn = robot.ComputeTcp4( coordRtn);

        robot.SetToolCoord(1, coordRtn, 0, 0, 1, 0);

        robot.GetForwardKin(p1Joint, p1Desc);
        robot.GetForwardKin(p2Joint, p2Desc);
        robot.GetForwardKin(p3Joint, p3Desc);

        robot.SetFocusCalibPoint(1, p1Desc);
        robot.SetFocusCalibPoint(2, p2Desc);
        robot.SetFocusCalibPoint(3, p3Desc);

        DescTran resultPos = new DescTran(0.0, 0.0, 0.0);
        double[] accuracy = {0.0};
        rtn = robot.ComputeFocusCalib(3,  resultPos,  accuracy);
        rtn = robot.SetFocusPosition(resultPos);

        robot.GetForwardKin(p5Joint,  p5Desc);
        robot.GetForwardKin(p6Joint,  p6Desc);

        robot.MoveL(p5Joint, p5Desc, 1, 0, 10, 100, 100, -1, 0, exaxisPos, 0, 1, offdese,0,100);
        robot.MoveL(p6Joint, p6Desc, 1, 0, 10, 100, 100, -1, 0, exaxisPos, 0, 1, offdese,0,100);

        robot.FocusStart(50, 19, 710, 90, 0);
        robot.MoveL(p5Joint, p5Desc, 1, 0, 10, 100, 100, -1, 0, exaxisPos, 0, 1, offdese,0,100);
        robot.MoveL(p6Joint, p6Desc, 1, 0, 10, 100, 100, -1, 0, exaxisPos, 0, 1, offdese,0,100);
        robot.FocusEnd();
    }

Joint Torque Sensor Sensitivity Calibration Function Enabled
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Joint torque sensor sensitivity calibration function enabled
    * @param status 0-Disabled; 1-Enabled
    * @return Error code
    */
    public int JointSensitivityEnable(int status)

Joint Torque Sensor Sensitivity Data Acquisition
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Joint Torque Sensor Sensitivity Data Collection
    * @return Error code
    */
    public int JointSensitivityCollect()

Get Joint Torque Sensor Sensitivity Calibration Results
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get joint torque sensor sensitivity calibration results
    * @param calibResult j1~j6 joint sensitivity [0-1]
    * @param linearity j1~j6 joint linearity [0-1]
    * @return Error code
    */
    public int JointSensitivityCalibration(double[] calibResult, double[] linearity)

Get Joint Torque Sensor Hysteresis Error
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: Java
    :linenos:

    /**
    * @brief Get joint torque sensor hysteresis error
    * @param hysteresisError j1~j6 joint hysteresis error
    * @return Error code
    */
    public int JointHysteresisError(double[] hysteresisError);
    
Get Joint Torque Sensor Repeatability
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: Java
    :linenos:
    
    /**
    * @brief Get joint torque sensor repeatability
    * @param repeatability j1~j6 joint torque sensor repeatability
    * @return Error code
    */
    public int JointRepeatability(double[] repeatability);
    
Set Joint Force Sensor Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: Java
    :linenos:

    /**
    * @brief Set joint force sensor parameters
    * @param required M J1-J6 mass coefficient []
    * @param required B J1-J6 damping coefficient []
    * @param required K J1-J6 stiffness coefficient []
    * @param default threshold Force control threshold, Nm
    * @param default sensitivity Sensitivity, Nm/V, []
    * @param default setZeroFlag Function enable flag; 0-Off; 1-On; 2-Record zero point at position 1; 3-Record zero point at position 2
    * @return Error code
    */
    public int SetAdmittanceParams(double[] M, double[] B, double[] K, double[] threshold, double[] sensitivity, int setZeroFlag);

Joint Torque Sensor Sensitivity Auto-Calibration Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void TestSensitivityCalib(Robot robot)
    {
        int rtn = robot.JointSensitivityEnable(0);
        rtn = robot.JointSensitivityEnable(1);
        System.out.printf("JointSensitivityEnable rtn is %d\n", rtn);
        JointPos curJPos = new JointPos();
        robot.GetActualJointPosDegree(curJPos);
        ExaxisPos epos = new ExaxisPos(0,0,0,0);
        DescPose offset_pos =new DescPose(0,0,0,0,0,0 );
        JointPos jointPos1 = new JointPos(curJPos.J1, 0, 0, -90, 0.02, curJPos.J6);
        DescPose descPos1 = new DescPose();
        robot.GetForwardKin(jointPos1, descPos1);
        robot.MoveJ(jointPos1, descPos1, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(200);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 1 rtn is %d\n", rtn);
        robot.Sleep(100);
        JointPos jointPos2 =new JointPos( curJPos.J1, -30, 0, -90, 0.02, curJPos.J6 );
        DescPose descPos2 =new DescPose();
        robot.GetForwardKin(jointPos2, descPos2);
        robot.MoveJ(jointPos2, descPos2, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 2 rtn is %d\n", rtn);
        robot.Sleep(100);
        JointPos jointPos3 = new JointPos( curJPos.J1, -60, 0, -90, 0.02, curJPos.J6 );
        DescPose descPos3 =new DescPose();
        robot.GetForwardKin(jointPos3, descPos3);
        robot.MoveJ(jointPos3, descPos3, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 3 rtn is %d\n", rtn);
        robot.Sleep(100);
        JointPos jointPos4 = new JointPos(curJPos.J1, -90, 0, -90, 0.02, curJPos.J6);
        DescPose descPos4 = new DescPose();
        robot.GetForwardKin(jointPos4, descPos4);
        robot.MoveJ(jointPos4, descPos4, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 4 rtn is %d\n", rtn);
        robot.Sleep(100);
        JointPos jointPos5 = new JointPos(curJPos.J1, -120, 0, -90, 0.02, curJPos.J6);
        DescPose descPos5 = new DescPose();
        robot.GetForwardKin(jointPos5, descPos5);
        robot.MoveJ(jointPos5, descPos5, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 5 rtn is %d\n", rtn);
        robot.Sleep(100);
        JointPos jointPos6 = new JointPos(curJPos.J1, -150, 0, -90, 0.02, curJPos.J6);
        DescPose descPos6 = new DescPose();
        robot.GetForwardKin(jointPos6, descPos6);
        robot.MoveJ(jointPos6, descPos6, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 6 rtn is %d\n", rtn);
        robot.Sleep(100);
        JointPos jointPos7 = new JointPos(curJPos.J1, -180, 0, -90, 0.02, curJPos.J6);
        DescPose descPos7 = new DescPose();
        robot.GetForwardKin(jointPos7, descPos7);
        robot.MoveJ(jointPos7, descPos7, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 7 rtn is %d\n", rtn);
        robot.Sleep(100);
        // Reverse stroke
        robot.MoveJ(jointPos6, descPos6, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 8 rtn is %d\n", rtn);
        robot.Sleep(100);
        robot.MoveJ(jointPos5, descPos5, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 9 rtn is %d\n", rtn);
        robot.Sleep(100);
        robot.MoveJ(jointPos4, descPos4, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 10 rtn is %d\n", rtn);
        robot.Sleep(100);
        robot.MoveJ(jointPos3, descPos3, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 11 rtn is %d\n", rtn);
        robot.Sleep(100);
        robot.MoveJ(jointPos2, descPos2, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 12 rtn is %d\n", rtn);
        robot.Sleep(100);
        robot.MoveJ(jointPos1, descPos1, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.Sleep(200);
        rtn = robot.JointSensitivityCollect();
        System.out.printf("JointSensitivityCollect 13 rtn is %d\n", rtn);
        robot.Sleep(100);
        double[] calibResult =new double[6];
        double[] linearity = new double[6];
        rtn = robot.JointSensitivityCalibration(calibResult, linearity);
        System.out.printf("JointSensitivityCalibration rtn is %d\n", rtn);
        rtn = robot.JointSensitivityEnable(0);
        System.out.printf("JointSensitivityEnable rtn is %d\n", rtn);
        System.out.printf("jointSensor Calib result is %f %f %f %f %f %f\njointSensor linearity is %f %f %f %f %f %f\n",
                calibResult[0], calibResult[1], calibResult[2],
                calibResult[3], calibResult[4], calibResult[5],
                linearity[0], linearity[1], linearity[2],
                linearity[3], linearity[4], linearity[5]);
        double[] hysteresisError = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        rtn = robot.JointHysteresisError(hysteresisError);
        System.out.printf("JointHysteresisError result is %f %f %f %f %f %f\n",
                hysteresisError[0], hysteresisError[1], hysteresisError[2],
                hysteresisError[3], hysteresisError[4], hysteresisError[5]);
        double[] repeatability = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        rtn = robot.JointRepeatability(repeatability);
        System.out.printf("JointRepeatability result is %f %f %f %f %f %f\n",
                repeatability[0], repeatability[1], repeatability[2],
                repeatability[3], repeatability[4], repeatability[5]);
        double[] M = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        double[] B = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        double[] K = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double[] threshold = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        int setZeroFlag = 1;
        rtn = robot.SetAdmittanceParams(M, B, K, threshold, calibResult, setZeroFlag);
        System.out.printf("SetAdmittanceParams rtn is %d\n", rtn);
    }

Retrieve the number of error frames for the robot's 8 slave ports
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Retrieve error frame counts for the robot's 8 slave ports
    * @param  inRecvErr Input received error frame count
    * @param  inCRCErr Input CRC error frame count
    * @param  inTransmitErr Input transmitted error frame count
    * @param  inLinkErr Input link error frame count
    * @param  outRecvErr Output received error frame count
    * @param  outCRCErr Output CRC error frame count
    * @param  outTransmitErr Output transmitted error frame count
    * @param  outLinkErr Output link error frame count
    * @return Error code
    */
    public int GetSlavePortErrCounter(int[] inRecvErr, int[] inCRCErr, int[] inTransmitErr, int[] inLinkErr, int[] outRecvErr, int[] outCRCErr, int[] outTransmitErr, int[] outLinkErr)

Reset slave port error frames
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 .. code-block:: Java
    :linenos:

    /**
    * @brief Clears error frames for slave port
    * @param slaveID Slave ID (0~7)
    * @return Error code
    */
    public int SlavePortErrCounterClear(int slaveID)

Example for retrieving slave port error frame codes
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 .. code-block:: Java
    :linenos:

    public static void TestSlavePortErr(Robot robot)
    {
        ROBOT_STATE_PKG pkg =new ROBOT_STATE_PKG();
        int[] inRecvErr =new int[8];
        int[] inCRCErr =new int[8];
        int[] inTransmitErr =new int[8];
        int[] inLinkErr =new int[8];
        int[] outRecvErr =new int[8];
        int[] outCRCErr =new int[8];
        int[] outTransmitErr =new int[8];
        int[] outLinkErr =new int[8];
        robot.GetSlavePortErrCounter(inRecvErr,  inCRCErr, inTransmitErr, inLinkErr,outRecvErr, outCRCErr, outTransmitErr, outLinkErr);
        for (int i = 0; i < 8; i++)
        {
            if (inRecvErr[i] != 0)
            {
                System.out.printf("inRecvErr %d is %d\n", i, inRecvErr[i]);
            }
            if (inCRCErr[i] != 0)
            {
                System.out.printf("inRecvErr %d is %d\n", i, inCRCErr[i]);
            }
            if (inTransmitErr[i] != 0)
            {
                System.out.printf("inRecvErr %d is %d\n", i, inTransmitErr[i]);
            }
            if (inLinkErr[i] != 0)
            {
                System.out.printf("inRecvErr %d is %d\n", i, inLinkErr[i]);
            }
            if (outRecvErr[i] != 0)
            {
                System.out.printf("outRecvErr %d is %d\n", i, outRecvErr[i]);
            }
            if (outCRCErr[i] != 0)
            {
                System.out.printf("outCRCErr %d is %d\n", i, outCRCErr[i]);
            }
            if (outTransmitErr[i] != 0)
            {
                System.out.printf("outTransmitErr %d is %d\n", i, outTransmitErr[i]);
            }
            if (outLinkErr[i] != 0)
            {
                System.out.printf("outLinkErr %d is %d\n", i, outLinkErr[i]);
            }
        }
        System.out.printf("others has no err!\n");
        for (int i = 0; i < 8; i++)
        {
            robot.SlavePortErrCounterClear(i);
        }
        robot.CloseRPC();
    }

Set speed feedforward coefficients for each axis
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Sets the feedforward ratio for each axis
    * @param  radio The feedforward ratio for each axis
    * @return Error code
    */
    public int SetVelFeedForwardRatio(double[] radio)

Retrieve speed feedforward ratios for each axis
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 .. code-block:: Java
    :linenos:

    /**
    * @brief Retrieve speed feedforward ratios for each axis
    * @param  radio Speed feedforward coefficients for each axis
    * @return Error code
    */
    public int GetVelFeedForwardRatio(double[] radio)

Robot Speed Feedforward Coefficient Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 .. code-block:: Java
    :linenos:
    
    public static void TestVelFeedForwardRatio(Robot robot)
    {
        double[] setRadio =new double[] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        robot.SetVelFeedForwardRatio(setRadio);
        double[] getRadio = new double[]{ 0.0 };
        robot.GetVelFeedForwardRatio(getRadio);
        System.out.printf(" %f %f %f %f %f %f\n", getRadio[0], getRadio[1], getRadio[2], getRadio[3], getRadio[4], getRadio[5]);
        robot.CloseRPC();
    }

Photoelectric Sensor TCP Calibration - Compute Tool RPY
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Photoelectric sensor TCP calibration - compute tool RPY
    * @param  Btool Robot Cartesian position
    * @param  Etool Current tool coordinate coefficient value
    * @param  sensor Current sensor coordinate coefficient value (not yet available)
    * @param  radius Circular motion radius mm (not yet available)
    * @param  dz Movement distance along the negative direction of the base coordinate system Z-axis; when dz = 10000, the function directly returns tool RPY
    * @param  TCPRPY Tool RPY value
    * @return Error code
    */
    public int TCPComputeRPY(DescPose Btool, DescPose Etool, DescPose sensor, double radius, double dz, Rpy TCPRPY)

Photoelectric Sensor TCP Calibration - Compute Tool XYZ
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Photoelectric sensor TCP calibration - compute tool XYZ
    * @param  select 0-Compute tool TCP; 1-Compute sensor origin; 2-Compute sensor orientation; 3-Directly return tool TCP; 4-Record current workpiece coordinate system and tool coordinate system
    * @param  originDirection 0-X direction; 1-Y direction; 2-Z direction
    * @param  pos1 Robot Cartesian position 1
    * @param  pos2 Robot Cartesian position 2
    * @param  pos3 Robot Cartesian position 3
    * @param  pos4 Robot Cartesian position 4
    * @param  TCP Tool XYZ value
    * @return Error code
    */
    public int TCPComputeXYZ(int select, double originDirection, DescTran pos1, DescTran pos2, DescTran pos3, DescTran pos4, DescTran TCP)

Photoelectric Sensor TCP Calibration - Start Recording Flange Center Position
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Photoelectric sensor TCP calibration - start recording flange center position
    * @return Error code
    */
    public int TCPRecordFlangePosStart()

Photoelectric Sensor TCP Calibration - Stop Recording Flange Center Position
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Photoelectric sensor TCP calibration - stop recording flange center position
    * @return Error code
    */
    public int TCPRecordFlangePosEnd()

Photoelectric Sensor TCP Calibration - Get Tool Center Point Position
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Photoelectric sensor TCP calibration - get tool center point position
    * @param  TCP Tool center point position (x,y,z)
    * @return Error code
    */
    public int TCPGetRecordFlangePos(DescTran TCP)

Photoelectric Sensor TCP Calibration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Photoelectric sensor TCP calibration
    * @param luaPath Automatic calibration Lua program path: "FR_CalibrateTheToolTcp.lua"
    * @param offset Teach point offset (x,y,z) mm
    * @param TCP Calibrated tool coordinate system (x,y,z,rx,ry,rz)
    * @return Error code
    */
    public int PhotoelectricSensorTCPCalibration(String luaPath, DescTran offset, DescPose TCP)

Photoelectric Sensor TCP Calibration Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void TestPhotoelectricSensorTCPCalib(Robot robot)
    {
        DescTran offset = new DescTran(10.0, 10.0, 3.0 );
        DescPose TCP = new DescPose();
        int rtn = robot.PhotoelectricSensorTCPCalibration("FR_CalibrateTheToolTcp.lua", offset, TCP);
        System.out.printf("PhotoelectricSensorTCPCalibration rtn is %d %f %f %f %f %f %f \n", rtn, TCP.tran.x, TCP.tran.y, TCP.tran.z, TCP.rpy.rx, TCP.rpy.ry, TCP.rpy.rz);
        robot.CloseRPC();
        robot.Sleep(9999999);
    }