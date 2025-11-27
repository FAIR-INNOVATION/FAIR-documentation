Common Settings
==========================

.. toctree:: 
    :maxdepth: 5

Set Tool Reference Point - Six-Point Method
+++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set tool reference point - six-point method
     * @param [in] point_num Point number, range [1~6] 
     * @return Error code
     */
    errno_t SetToolPoint(int point_num);

Calculate Tool Coordinate System
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Calculate tool coordinate system
     * @param [out] tcp_pose Tool coordinate system
     * @return Error code
     */
    errno_t ComputeTool(DescPose *tcp_pose);

Set Tool Reference Point - Four-Point Method
++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set tool reference point - four-point method
     * @param [in] point_num Point number, range [1~4] 
     * @return Error code
     */
    errno_t SetTcp4RefPoint(int point_num);

Calculate Tool Coordinate System
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Calculate tool coordinate system
     * @param [out] tcp_pose Tool coordinate system
     * @return Error code
     */
    errno_t ComputeTcp4(DescPose *tcp_pose);

Calculate Tool Coordinate System from Points
++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
     * @brief Calculate tool coordinate system from point data
     * @param [in] method Calculation method: 0-four-point method; 1-six-point method
     * @param [in] pos Joint position array (4 points for four-point method, 6 points for six-point method)
     * @param [out] coord Resulting tool coordinate system
     * @return Error code
     */
    errno_t ComputeToolCoordWithPoints(int method, JointPos pos[], DescPose& coord);

Set Tool Coordinate System
++++++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
     * @brief Set tool coordinate system
     * @param [in] id Coordinate system ID, range [0~14]
     * @param [in] coord Tool center point pose relative to end flange center
     * @param [in] type 0-tool coordinate system, 1-sensor coordinate system
     * @param [in] install Installation position: 0-robot end, 1-external to robot
     * @param [in] toolID Tool ID
     * @param [in] loadNum Load number
     * @return Error code
     */
    errno_t SetToolCoord(int id, DescPose *coord, int type, int install, int toolID, int loadNum);

Set Tool Coordinate System List
++++++++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
     * @brief Set tool coordinate system list
     * @param [in] id Coordinate system ID, range [0~14]
     * @param [in] coord Tool center point pose relative to end flange center
     * @param [in] type 0-tool coordinate system, 1-sensor coordinate system
     * @param [in] install Installation position: 0-robot end, 1-external to robot
     * @param [in] loadNum Load number
     * @return Error code
     */
    errno_t SetToolList(int id, DescPose *coord, int type, int install, int loadNum);

Get Current Tool Coordinate System
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get current tool coordinate system
     * @param [in] flag 0-blocking, 1-non-blocking
     * @param [out] desc_pos Tool coordinate system pose
     * @return Error code
     */
    errno_t GetTCPOffset(uint8_t flag, DescPose *desc_pos);

Robot Tool Coordinate System Operation Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestTCPCompute(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        DescPose p1Desc(186.331, 487.913, 209.850, 149.030, 0.688, -114.347);
        JointPos p1Joint(-127.876, -75.341, 115.417, -122.741, -59.820, 74.300);
        DescPose p2Desc(69.721, 535.073, 202.882, -144.406, -14.775, -89.012);
        JointPos p2Joint(-101.780, -69.828, 110.917, -125.740, -127.841, 74.300);
        DescPose p3Desc(146.861, 578.426, 205.598, 175.997, -36.178, -93.437);
        JointPos p3Joint(-112.851, -60.191, 86.566, -80.676, -97.463, 74.300);
        DescPose p4Desc(136.284, 509.876, 225.613, 178.987, 1.372, -100.696);
        JointPos p4Joint(-116.397, -76.281, 113.845, -128.611, -88.654, 74.299);
        DescPose p5Desc(138.395, 505.972, 298.016, 179.134, 2.147, -101.110);
        JointPos p5Joint(-116.814, -82.333, 109.162, -118.662, -88.585, 74.302);
        DescPose p6Desc(105.553, 454.325, 232.017, -179.426, 0.444, -99.952);
        JointPos p6Joint(-115.649, -84.367, 122.447, -128.663, -90.432, 74.303);
        ExaxisPos exaxisPos(0, 0, 0, 0);
        DescPose offdese(0, 0, 0, 0, 0, 0);
        JointPos posJ[6] = { p1Joint , p2Joint , p3Joint , p4Joint , p5Joint , p6Joint };
        DescPose coordRtn = {};
        rtn = robot.ComputeToolCoordWithPoints(1, posJ, coordRtn);
        printf("ComputeToolCoordWithPoints    %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
        robot.MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetToolPoint(1);
        robot.MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetToolPoint(2);
        robot.MoveJ(&p3Joint, &p3Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetToolPoint(3);
        robot.MoveJ(&p4Joint, &p4Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetToolPoint(4);
        robot.MoveJ(&p5Joint, &p5Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetToolPoint(5);
        robot.MoveJ(&p6Joint, &p6Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetToolPoint(6);
        rtn = robot.ComputeTool(&coordRtn);
        printf("6 Point ComputeTool        %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
        robot.SetToolList(1, &coordRtn, 0, 0, 0);
        robot.MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetTcp4RefPoint(1);
        robot.MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetTcp4RefPoint(2);
        robot.MoveJ(&p3Joint, &p3Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetTcp4RefPoint(3);
        robot.MoveJ(&p4Joint, &p4Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetTcp4RefPoint(4);
        rtn = robot.ComputeTcp4(&coordRtn);
        printf("4 Point ComputeTool        %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
        robot.SetToolCoord(2, &coordRtn, 0, 0, 1, 0);
        DescPose getCoord = {};
        rtn = robot.GetTCPOffset(0, &getCoord);
        printf("GetTCPOffset    %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
        robot.CloseRPC();
        return 0;
    }

Set External Tool Reference Point - Six-Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set external tool reference point - six-point method
     * @param [in] point_num Point number, range [1~4] 
     * @return Error code
     */
    errno_t SetExTCPPoint(int point_num);

Calculate External Tool Coordinate System
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Calculate external tool coordinate system
     * @param [out] tcp_pose External tool coordinate system
     * @return Error code
     */
    errno_t ComputeExTCF(DescPose *tcp_pose);  

Set External Tool Coordinate System
+++++++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    /**
     * @brief Set external tool coordinate system
     * @param [in] id Coordinate system ID, range [0~14]
     * @param [in] etcp Tool center point pose relative to end flange center
     * @param [in] etool To be determined
     * @return Error code
     */
    errno_t SetExToolCoord(int id, DescPose *etcp, DescPose *etool);

Set External Tool Coordinate System List
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.2.0

.. code-block:: c++
    :linenos:

    /**
     * @brief Set external tool coordinate system list
     * @param [in] id Coordinate system ID, range [0~14]
     * @param [in] etcp Tool center point pose relative to end flange center
     * @param [in] etool To be determined
     * @return Error code
     */
    errno_t SetExToolList(int id, DescPose *etcp, DescPose *etool);

Robot External Tool Coordinate System Operation Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestExtCoord(void)
    {
       ROBOT_STATE_PKG pkg = {};
       FRRobot robot;
       robot.LoggerInit();
       robot.SetLoggerLevel(1);
       int rtn = robot.RPC("192.168.58.2");
       if (rtn != 0)
       {
          return -1;
       }
       robot.SetReConnectParam(true, 30000, 500);
       DescPose p1Desc(-89.606, 779.517, 193.516, 178.000, 0.476, -92.484);
       JointPos p1Joint(-108.145, -50.137, 85.818, -125.599, -87.946, 74.329);
       DescPose p2Desc(-24.656, 850.384, 191.361, 177.079, -2.058, -95.355);
       JointPos p2Joint(-111.024, -41.538, 69.222, -114.913, -87.743, 74.329);
       DescPose p3Desc(-99.813, 766.661, 241.878, -176.817, 1.917, -91.604);
       JointPos p3Joint(-107.266, -56.116, 85.971, -122.560, -92.548, 74.331);
       ExaxisPos exaxisPos(0, 0, 0, 0);
       DescPose offdese(0, 0, 0, 0, 0, 0);
       DescPose posTCP[3] = { p1Desc , p2Desc , p3Desc };
       DescPose coordRtn = {};
       robot.MoveJ(&p1Joint, &p1Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
       robot.SetExTCPPoint(1);
       robot.MoveJ(&p2Joint, &p2Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
       robot.SetExTCPPoint(2);
       robot.MoveJ(&p3Joint, &p3Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
       robot.SetExTCPPoint(3);
       rtn = robot.ComputeExTCF(&coordRtn);
       printf("ComputeExTCF          %d coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
       robot.SetExToolCoord(1, &coordRtn, &offdese);
       robot.SetExToolList(1, &coordRtn, &offdese);
       robot.CloseRPC();
       return 0;
    }

Set Workpiece Reference Point - Three-Point Method
++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set workpiece reference point - three-point method
     * @param [in] point_num Point number, range [1~3] 
     * @return Error code
     */
    errno_t SetWObjCoordPoint(int point_num);

Calculate Workpiece Coordinate System
+++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
     * @brief Calculate workpiece coordinate system
     * @param [in] method Calculation method: 0-origin-x-axis-z-axis, 1-origin-x-axis-xy-plane
     * @param [in] refFrame Reference coordinate system
     * @param [out] wobj_pose Workpiece coordinate system
     * @return Error code
     */
    errno_t ComputeWObjCoord(int method, int refFrame, DescPose *wobj_pose);

Set Workpiece Coordinate System
+++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.5.0

.. code-block:: c++
    :linenos:

    /**
     * @brief Set workpiece coordinate system
     * @param [in] id Coordinate system ID, range [0~14]
     * @param [in] coord Workpiece coordinate system pose relative to end flange center
     * @param [in] refFrame Reference coordinate system
     * @return Error code
     */
    errno_t SetWObjCoord(int id, DescPose *coord, int refFrame);

Set Workpiece Coordinate System List
++++++++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

    /**
     * @brief Set workpiece coordinate system list
     * @param [in] id Coordinate system ID, range [0~14]
     * @param [in] coord Workpiece coordinate system pose relative to end flange center
     * @param [in] refFrame Reference coordinate system
     * @return Error code
     */
    errno_t SetWObjList(int id, DescPose *coord, int refFrame);

Calculate Workpiece Coordinate System from Points
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
     * @brief Calculate workpiece coordinate system from point data
     * @param [in] method Calculation method: 0-origin-x-axis-z-axis, 1-origin-x-axis-xy-plane
     * @param [in] pos Three TCP position array
     * @param [in] refFrame Reference coordinate system
     * @param [out] coord Resulting workpiece coordinate system
     * @return Error code
     */
    errno_t ComputeWObjCoordWithPoints(int method, DescPose pos[], int refFrame, DescPose& coord);

Get Current Workpiece Coordinate System
++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get current workpiece coordinate system
     * @param [in] flag 0-blocking, 1-non-blocking
     * @param [out] desc_pos Workpiece coordinate system pose
     * @return Error code
     */   
    errno_t GetWObjOffset(uint8_t flag, DescPose *desc_pos);

Robot Workpiece Coordinate System Operation Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestWobjCoord(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        DescPose p1Desc(-89.606, 779.517, 193.516, 178.000, 0.476, -92.484);
        JointPos p1Joint(-108.145, -50.137, 85.818, -125.599, -87.946, 74.329);
        DescPose p2Desc(-24.656, 850.384, 191.361, 177.079, -2.058, -95.355);
        JointPos p2Joint(-111.024, -41.538, 69.222, -114.913, -87.743, 74.329);
        DescPose p3Desc(-99.813, 766.661, 241.878, -176.817, 1.917, -91.604);
        JointPos p3Joint(-107.266, -56.116, 85.971, -122.560, -92.548, 74.331);
        ExaxisPos exaxisPos(0, 0, 0, 0);
        DescPose offdese(0, 0, 0, 0, 0, 0);
        DescPose posTCP[3] = { p1Desc , p2Desc , p3Desc };
        DescPose coordRtn = {};
        rtn = robot.ComputeWObjCoordWithPoints(1, posTCP, 0, coordRtn);
        printf("ComputeWObjCoordWithPoints    %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
        robot.MoveJ(&p1Joint, &p1Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetWObjCoordPoint(1);
        robot.MoveJ(&p2Joint, &p2Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetWObjCoordPoint(2);
        robot.MoveJ(&p3Joint, &p3Desc, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.SetWObjCoordPoint(3);
        rtn = robot.ComputeWObjCoord(1, 0, &coordRtn);
        printf("ComputeWObjCoord                   %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
        robot.SetWObjCoord(1, &coordRtn, 0);
        robot.SetWObjList(1, &coordRtn, 0);
        DescPose getWobjDesc = {};
        rtn = robot.GetWObjOffset(0, &getWobjDesc);
        printf("GetWObjOffset                   %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
        robot.CloseRPC();
        return 0;
    }

Set Global Speed
++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set global speed
     * @param [in] vel Speed percentage, range [0~100]
     * @return Error code
     */
    errno_t SetSpeed(int vel);

Set Robot Acceleration
+++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set robot acceleration
     * @param [in] acc Robot acceleration percentage
     * @return Error code
     */
    errno_t SetOaccScale(double acc);

Get Robot Default Speed
+++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get robot default speed
     * @param [out] vel Speed in mm/s
     * @return Error code
     */   
    errno_t GetDefaultTransVel(float *vel);
    
Set End Load Weight
++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.8-3.7.8

.. code-block:: c++
    :linenos:

    /**
     * @brief Set end load weight
     * @param [in] loadNum Load number
     * @param [in] weight Load weight in kg
     * @return Error code
     */
    errno_t SetLoadWeight(int loadNum = 0, float weight);

Set End Effector Payload Center of Gravity Coordinates
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C++SDK-v3.8.6
    
.. code-block:: c++
    :linenos:

    /**
    * @brief Set End Effector Payload Center of Gravity Coordinates
    * @param [in] loadNum Payload number
    * @param [in] coord Center of gravity coordinates, unit mm
    * @return Error code
    */
    errno_t SetLoadCoord(int loadNum, DescTran* coord);

Get Current Load Weight
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get current load weight
     * @param [in] flag 0-blocking, 1-non-blocking
     * @param [out] weight Load weight in kg
     * @return Error code
     */
    errno_t GetTargetPayload(uint8_t flag, float *weight);

Get Current Load Center of Gravity
+++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get current load center of gravity
     * @param [in] flag 0-blocking, 1-non-blocking
     * @param [out] cog Load center of gravity in mm
     * @return Error code
     */   
    errno_t GetTargetPayloadCog(uint8_t flag, DescTran *cog);

Set Robot Installation Method
+++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set robot installation method
     * @param [in] install Installation method: 0-standard, 1-wall-mounted, 2-ceiling-mounted
     * @return Error code
     */
    errno_t SetRobotInstallPos(uint8_t install);   

Set Robot Installation Angle
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set robot installation angle (free installation)
     * @param [in] yangle Tilt angle
     * @param [in] zangle Rotation angle
     * @return Error code
     */
    errno_t SetRobotInstallAngle(double yangle, double zangle);

Get Robot Installation Angle
+++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get robot installation angle
     * @param [out] yangle Tilt angle
     * @param [out] zangle Rotation angle
     * @return Error code
     */
    errno_t GetRobotInstallAngle(float *yangle, float *zangle);

Set System Variable Value
+++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set system variable value
     * @param [in] id Variable number, range [1~20]
     * @param [in] value Variable value
     * @return Error code
     */
    errno_t SetSysVarValue(int id, float value);

Get System Variable Value
++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Get system variable value
     * @param [in] id System variable number, range [1~20]
     * @param [out] value System variable value
     * @return Error code
     */
    errno_t GetSysVarValue(int id, float *value);

Robot Common Settings Example
++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestLoadInstall(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        for (int i = 1; i < 100; i++)
        {
            robot.SetSpeed(i);
            robot.SetOaccScale(i);
            robot.Sleep(30);
        }
        float defaultVel = 0.0;
        robot.GetDefaultTransVel(&defaultVel);
        printf("GetDefaultTransVel is %f\n", defaultVel);
        for (int i = 1; i < 21; i++)
        {
            robot.SetSysVarValue(i, i + 0.5);
            robot.Sleep(100);
        }
        for (int i = 1; i < 21; i++)
        {
            float value = 0;
            robot.GetSysVarValue(i, &value);
            printf("sys value  %d is :%f\n", i, value);
            robot.Sleep(100);
        }
        robot.SetLoadWeight(0, 2.5);
        DescTran loadCoord = {};
        loadCoord.x = 3.0;
        loadCoord.y = 4.0;
        loadCoord.z = 5.0;
        robot.SetLoadCoord(&loadCoord);
        robot.Sleep(1000);
        float getLoad = 0.0;
        robot.GetTargetPayload(0, &getLoad);
        DescTran getLoadTran = {};
        robot.GetTargetPayloadCog(0, &getLoadTran);
        printf("get load is %f; get load cog is %f %f %f\n", getLoad, getLoadTran.x, getLoadTran.y, getLoadTran.z);
        robot.SetRobotInstallPos(0);
        robot.SetRobotInstallAngle(15.0, 25.0);
        float anglex = 0.0;
        float angley = 0.0;
        robot.GetRobotInstallAngle(&anglex, &angley);
        printf("GetRobotInstallAngle x:  %f;  y:  %f\n", anglex, angley);
        robot.CloseRPC();
        return 0;
    }

Joint Friction Compensation Switch
+++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Joint friction compensation switch
     * @param [in] state 0-off, 1-on
     * @return Error code
     */
    errno_t FrictionCompensationOnOff(uint8_t state);

Set Joint Friction Compensation Coefficient - Standard Installation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set joint friction compensation coefficient - standard installation
     * @param [in] coeff Six joint compensation coefficients, range [0~1]
     * @return Error code
     */
    errno_t SetFrictionValue_level(float coeff[6]);

Set Joint Friction Compensation Coefficient - Wall Installation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set joint friction compensation coefficient - wall installation
     * @param [in] coeff Six joint compensation coefficients, range [0~1]
     * @return Error code
     */
    errno_t SetFrictionValue_wall(float coeff[6]);

Set Joint Friction Compensation Coefficient - Ceiling Installation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set joint friction compensation coefficient - ceiling installation
     * @param [in] coeff Six joint compensation coefficients, range [0~1]
     * @return Error code
     */
    errno_t SetFrictionValue_ceiling(float coeff[6]);

Set Joint Friction Compensation Coefficient - Free Installation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Set joint friction compensation coefficient - free installation
     * @param [in] coeff Six joint compensation coefficients, range [0~1]
     * @return Error code
     */
    errno_t SetFrictionValue_freedom(float coeff[6]);

Robot Joint Friction Compensation Example
++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestFriction(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;

        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        float lcoeff[6] = { 0.9,0.9,0.9,0.9,0.9,0.9 };
        float wcoeff[6] = { 0.4,0.4,0.4,0.4,0.4,0.4 };
        float ccoeff[6] = { 0.6,0.6,0.6,0.6,0.6,0.6 };
        float fcoeff[6] = { 0.5,0.5,0.5,0.5,0.5,0.5 };
        rtn = robot.FrictionCompensationOnOff(1);
        printf("FrictionCompensationOnOff rtn is %d\n", rtn);
        rtn = robot.SetFrictionValue_level(lcoeff);
        printf("SetFrictionValue_level rtn is %d\n", rtn);
        rtn = robot.SetFrictionValue_wall(wcoeff);
        printf("SetFrictionValue_wall rtn is %d\n", rtn);
        rtn = robot.SetFrictionValue_ceiling(ccoeff);
        printf("SetFrictionValue_ceiling rtn is %d\n", rtn);
        rtn = robot.SetFrictionValue_freedom(fcoeff);
        printf("SetFrictionValue_freedom rtn is %d\n", rtn);
        robot.CloseRPC();
        return 0;
    }

Query Robot Error Code
+++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Query robot error code
     * @param [out] maincode Main error code
     * @param [out] subcode Sub error code
     * @return Error code
     */ 
    errno_t GetRobotErrorCode(int *maincode, int *subcode);

Error State Clear
+++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief Error state clear
     * @return Error code
     */
    errno_t ResetAllError();

Robot Fault State Acquisition and Error Clear Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    int TestGetError(void)
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return -1;
        }
        robot.SetReConnectParam(true, 30000, 500);
        int maincode, subcode;
        robot.GetRobotErrorCode(&maincode, &subcode);
        printf("robot maincode is %d; subcode is %d\n", maincode, subcode);
        robot.ResetAllError();
        robot.Sleep(1000);
        robot.GetRobotErrorCode(&maincode, &subcode);
        printf("robot maincode is %d; subcode is %d\n", maincode, subcode);
        robot.CloseRPC();
        return 0;
    }

Set wide voltage control box temperature and fan current monitoring parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    *@brief setting wide voltage control box temperature and fan current monitoring parameters
    *@param [ in ] enable 0-do not enable monitoring; 1-enable monitoring
    *@param [ in ] period (s) , range 1-100
    *@return error code
    */
    errno_t SetWideBoxTempFanMonitorParam(int enable, int period);
    
Obtain wide voltage control box temperature and fan current monitoring parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Obtain wide voltage control box temperature and fan current monitoring parameters
    *@param [ in ] enable 0-do not enable monitoring; 1-enable monitoring
    *@param [ in ] period (s) , range 1-100
    *@return error code
    */
    errno_t GetWideBoxTempFanMonitorParam(int &enable, int &period);
    
Wide voltage control box temperature and fan current state acquisition code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

     int TestWideVoltageCtrlBoxtemp(void)
     {
         ROBOT_STATE_PKG pkg = {};
         FRRobot robot;
         robot.LoggerInit();
         robot.SetLoggerLevel(1);
         int rtn = robot.RPC("192.168.58.2");
         printf("robot rpc rtn is %d\n", rtn);
         if (rtn != 0)
         {
             return -1;
         }
         robot.SetReConnectParam(true, 30000, 500);
         robot.SetWideBoxTempFanMonitorParam(1, 2);
         int enable = 0;
         int period = 0;
         robot.GetWideBoxTempFanMonitorParam(enable, period);
         printf("GetWideBoxTempFanMonitorParam enable is %d   period is %d\n", enable, period);
         for (int i = 0; i < 100; i++)
         {
             robot.GetRobotRealTimeState(&pkg);
             printf("robot ctrl box temp is %f,  fan current is %d\n", pkg.wideVoltageCtrlBoxTemp, pkg.wideVoltageCtrlBoxFanCurrent);
             robot.Sleep(100);
         }
         rtn = robot.SetWideBoxTempFanMonitorParam(0, 2);
         printf("SetWideBoxTempFanMonitorParam rtn is %d\n", rtn);
         enable = 0;
         period = 0;
         robot.GetWideBoxTempFanMonitorParam(enable, period);
         printf("GetWideBoxTempFanMonitorParam enable is %d   period is %d\n", enable, period);
         for (int i = 0; i < 100; i++)
         {
             robot.GetRobotRealTimeState(&pkg);
             printf("robot ctrl box temp is %f,  fan current is %d\n", pkg.wideVoltageCtrlBoxTemp, pkg.wideVoltageCtrlBoxFanCurrent);
             robot.Sleep(100);
         }
         robot.CloseRPC();
         robot.Sleep(2000);
         return 0;
     }

Calculate Focus Calibration Results
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Calculate focus calibration results
    * @param [in] pointNum Number of calibration points
    * @param [out] resultPos Calibration result XYZ
    * @param [out] accuracy Calibration accuracy error
    * @return Error code
    */
    errno_t ComputeFocusCalib(int pointNum, DescTran& resultPos, float& accuracy);
         
Set Focus Position
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Set focus position
    * @param [in] pos Focus position XYZ
    * @return Error code
    */
    errno_t SetFocusPosition(DescTran pos);
         
Enable Focus Following
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Enable focus following
    * @param [in] kp Proportional parameter (default: 50.0)
    * @param [in] kpredict Feedforward parameter (default: 19.0)
    * @param [in] aMax Maximum angular acceleration limit (default: 1440°/s²)
    * @param [in] vMax Maximum angular velocity limit (default: 180°/s)
    * @param [in] type X-axis locking mode (0-reference input vector; 1-horizontal; 2-vertical)
    * @return Error code
    */
    errno_t FocusStart(double kp, double kpredict, double aMax, double vMax, int type);
         
Disable Focus Following
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Disable focus following
    * @return Error code
    */
    errno_t FocusEnd();

Robot Focus Following Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestFocus()
    {
      ROBOT_STATE_PKG pkg = {};
      FRRobot robot;
      robot.LoggerInit();
      robot.SetLoggerLevel(1);
      int rtn = robot.RPC("192.168.58.2");
      if (rtn != 0)
      {
        return -1;
      }
      robot.SetReConnectParam(true, 30000, 500);
      DescPose p1Desc(186.331, 487.913, 209.850, 149.030, 0.688, -114.347);
      JointPos p1Joint(-127.876, -75.341, 115.417, -122.741, -59.820, 74.300);
      DescPose p2Desc(69.721, 535.073, 202.882, -144.406, -14.775, -89.012);
      JointPos p2Joint(-101.780, -69.828, 110.917, -125.740, -127.841, 74.300);
      DescPose p3Desc(146.861, 578.426, 205.598, 175.997, -36.178, -93.437);
      JointPos p3Joint(-112.851, -60.191, 86.566, -80.676, -97.463, 74.300);
      DescPose p4Desc(136.284, 509.876, 225.613, 178.987, 1.372, -100.696);
      JointPos p4Joint(-116.397, -76.281, 113.845, -128.611, -88.654, 74.299);
      DescPose p5Desc(138.395, 505.972, 298.016, 179.134, 2.147, -101.110);
      JointPos p5Joint(-116.814, -82.333, 109.162, -118.662, -88.585, 74.302);
      DescPose p6Desc(105.553, 454.325, 232.017, -179.426, 0.444, -99.952);
      JointPos p6Joint(-115.649, -84.367, 122.447, -128.663, -90.432, 74.303);
      ExaxisPos exaxisPos(0, 0, 0, 0);
      DescPose offdese(0, 0, 100, 0, 0, 0);
      robot.MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
      robot.SetTcp4RefPoint(1);
      robot.MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
      robot.SetTcp4RefPoint(2);
      robot.MoveJ(&p3Joint, &p3Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
      robot.SetTcp4RefPoint(3);
      robot.MoveJ(&p4Joint, &p4Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
      robot.SetTcp4RefPoint(4);
      DescPose coordRtn = {};
      rtn = robot.ComputeTcp4(&coordRtn);
      printf("4 Point ComputeTool    %d coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
      robot.SetToolCoord(1, &coordRtn, 0, 0, 1, 0);
      robot.GetForwardKin(&p1Joint, &p1Desc);
      robot.GetForwardKin(&p2Joint, &p2Desc);
      robot.GetForwardKin(&p3Joint, &p3Desc);
      robot.SetFocusCalibPoint(1, p1Desc);
      robot.SetFocusCalibPoint(2, p2Desc);
      robot.SetFocusCalibPoint(3, p3Desc);
      DescTran resultPos = {};
      float accuracy = 0.0;
      rtn = robot.ComputeFocusCalib(3, resultPos, accuracy);
      printf("ComputeFocusCalib coord is %d %f %f %f accuracy is %f\n", rtn, resultPos.x, resultPos.y, resultPos.z, accuracy);
      rtn = robot.SetFocusPosition(resultPos);
      robot.GetForwardKin(&p5Joint, &p5Desc);
      robot.GetForwardKin(&p6Joint, &p6Desc);
      robot.MoveL(&p5Joint, &p5Desc, 1, 0, 10, 100, 100, -1, 0, &exaxisPos, 0, 1, &offdese);
      robot.MoveL(&p6Joint, &p6Desc, 1, 0, 10, 100, 100, -1, 0, &exaxisPos, 0, 1, &offdese);
      robot.FocusStart(50, 19, 710, 90, 0);
      robot.MoveL(&p5Joint, &p5Desc, 1, 0, 10, 100, 100, -1, 0, &exaxisPos, 0, 1, &offdese);
      robot.MoveL(&p6Joint, &p6Desc, 1, 0, 10, 100, 100, -1, 0, &exaxisPos, 0, 1, &offdese);
      robot.FocusEnd();
      robot.CloseRPC();
      return 0;
    }

Open the joint torque sensor sensitivity calibration function
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Open the joint torque sensor sensitivity calibration function
    * @param [ in ] status 0-off; 1-on
    * @return  error code
    */
    errno_t JointSensitivityEnable(int status);

Data acquisition of joint torque sensor sensitivity
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Data acquisition of joint torque sensor sensitivity
    * @return error code
    */
    errno_t JointSensitivityCollect();

Obtain joint torque sensor sensitivity calibration results
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Get joint torque sensor sensitivity calibration results
    * @param [out] calibResult j1~j6 joint sensitivity [0-1]
    * @param [out] linearity j1~j6 joint linearity [0-1]
    * @return Error code
    */
    errno_t JointSensitivityCalibration(double calibResult[6], double linearity[6]);

Get Joint Torque Sensor Hysteresis Error
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Get joint torque sensor hysteresis error
    * @param [out] hysteresisError j1~j6 joint hysteresis error
    * @return Error code
    */
    errno_t JointHysteresisError(double hysteresisError[6]);
    
Get Joint Torque Sensor Repeatability
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:
    
    /**
    * @brief Get joint torque sensor repeatability
    * @param [out] repeatability j1~j6 joint torque sensor repeatability
    * @return Error code
    */
    errno_t JointRepeatability(double repeatability[6]);
    
Set Joint Force Sensor Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
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
    errno_t SetAdmittanceParams(double M[6], double B[6], double K[6], double threshold[6], double sensitivity[6], int setZeroFlag);

An example of joint torque sensor sensitivity auto-calibration code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestSensitivityCalib()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        robot.SetReConnectParam(true, 30000, 500);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return 0;
        }
        rtn = robot.JointSensitivityEnable(0);
        rtn = robot.JointSensitivityEnable(1);
        printf("JointSensitivityEnable rtn is %d\n", rtn);
        JointPos curJPos = {};
        robot.GetActualJointPosDegree(0, &curJPos);
        ExaxisPos epos = { 0,0,0,0 };
        DescPose offset_pos = { 0,0,0,0,0,0 };
        JointPos jointPos1 = { curJPos.jPos[0], 0, 0, -90, 0.02, curJPos.jPos[5] };
        DescPose descPos1 = {};
        robot.GetForwardKin(&jointPos1, &descPos1);
        robot.MoveJ(&jointPos1, &descPos1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(200);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 1 rtn is %d\n", rtn);
        robot.Sleep(100);
        JointPos jointPos2 = { curJPos.jPos[0], -30, 0, -90, 0.02, curJPos.jPos[5] };
        DescPose descPos2 = {};
        robot.GetForwardKin(&jointPos2, &descPos2);
        robot.MoveJ(&jointPos2, &descPos2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 2 rtn is %d\n", rtn);
        robot.Sleep(100);
        JointPos jointPos3 = { curJPos.jPos[0], -60, 0, -90, 0.02, curJPos.jPos[5] };
        DescPose descPos3 = {};
        robot.GetForwardKin(&jointPos3, &descPos3);
        robot.MoveJ(&jointPos3, &descPos3, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 3 rtn is %d\n", rtn);
        robot.Sleep(100);
        JointPos jointPos4 = { curJPos.jPos[0], -90, 0, -90, 0.02, curJPos.jPos[5] };
        DescPose descPos4 = {};
        robot.GetForwardKin(&jointPos4, &descPos4);
        robot.MoveJ(&jointPos4, &descPos4, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 4 rtn is %d\n", rtn);
        robot.Sleep(100);
        JointPos jointPos5 = { curJPos.jPos[0], -120, 0, -90, 0.02, curJPos.jPos[5] };
        DescPose descPos5 = {};
        robot.GetForwardKin(&jointPos5, &descPos5);
        robot.MoveJ(&jointPos5, &descPos5, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 5 rtn is %d\n", rtn);
        robot.Sleep(100);
        JointPos jointPos6 = { curJPos.jPos[0], -150, 0, -90, 0.02, curJPos.jPos[5] };
        DescPose descPos6 = {};
        robot.GetForwardKin(&jointPos6, &descPos6);
        robot.MoveJ(&jointPos6, &descPos6, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 6 rtn is %d\n", rtn);
        robot.Sleep(100);
        JointPos jointPos7 = { curJPos.jPos[0], -180, 0, -90, 0.02, curJPos.jPos[5] };
        DescPose descPos7 = {};
        robot.GetForwardKin(&jointPos7, &descPos7);
        robot.MoveJ(&jointPos7, &descPos7, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 7 rtn is %d\n", rtn);
        robot.Sleep(100);
        //-------------------
        robot.MoveJ(&jointPos6, &descPos6, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 8 rtn is %d\n", rtn);
        robot.Sleep(100);
        robot.MoveJ(&jointPos5, &descPos5, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 9 rtn is %d\n", rtn);
        robot.Sleep(100);
        robot.MoveJ(&jointPos4, &descPos4, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 10 rtn is %d\n", rtn);
        robot.Sleep(100);
        robot.MoveJ(&jointPos3, &descPos3, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 11 rtn is %d\n", rtn);
        robot.Sleep(100);
        robot.MoveJ(&jointPos2, &descPos2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(100);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 12 rtn is %d\n", rtn);
        robot.Sleep(100);
        robot.MoveJ(&jointPos1, &descPos1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        robot.Sleep(200);
        rtn = robot.JointSensitivityCollect();
        printf("JointSensitivityCollect 13 rtn is %d\n", rtn);
        robot.Sleep(100);
        double calibResult[6] = { 0.0 };
        double linearity[6] = { 0.0 };
        rtn = robot.JointSensitivityCalibration(calibResult, linearity);
        printf("JointSensitivityCalibration rtn is %d\n", rtn);
        rtn = robot.JointSensitivityEnable(0);
        printf("JointSensitivityEnable rtn is %d\n", rtn);
        printf("jointSensor Calib result is %f %f %f %f %f %f\njointSensor linearity is %f %f %f %f %f %f\n", 
            calibResult[0], calibResult[1], calibResult[2], 
            calibResult[3], calibResult[4], calibResult[5], 
            linearity[0], linearity[1], linearity[2],
            linearity[3], linearity[4], linearity[5]);
        double hysteresisError[6] = { 0.0 };
        rtn = robot.JointHysteresisError(hysteresisError);
        printf("JointHysteresisError result is %f %f %f %f %f %f\n",
            hysteresisError[0], hysteresisError[1], hysteresisError[2],
            hysteresisError[3], hysteresisError[4], hysteresisError[5]);
        double repeatability[6] = { 0.0 };
        rtn = robot.JointRepeatability(repeatability);
        printf("JointRepeatability result is %f %f %f %f %f %f\n",
            repeatability[0], repeatability[1], repeatability[2],
            repeatability[3], repeatability[4], repeatability[5]);
        double M[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        double B[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        double K[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        double threshold[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        int setZeroFlag = 1;
        rtn = robot.SetAdmittanceParams(M, B, K, threshold, calibResult, setZeroFlag);
        printf("SetAdmittanceParams rtn is %d\n", rtn);
        robot.CloseRPC();
    }
    
Gets 8 slave error frames
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief  gets 8 slave error frames
    * @param [ out ] in recverr
    * @param [ out -RSB- inCRCErr enCRC CRC error frames
    * @param [ out ] in forwarding error frames
    * @param [ out ] inlinker R enter link error frames
    * @param [ out ] out recverr output received error frames
    * @param [ out ] outCRCErr output CRC error frames
    * @param [ out ] outtransmitterr output forwarding error frames
    * @param [ out ] outLinkErr output link error frames
    * @return  error code
    */
    errno_t GetSlavePortErrCounter(int inRecvErr[8], int inCRCErr[8], int inTransmitErr[8], int inLinkErr[8], int outRecvErr[8], int outCRCErr[8], int outTransmitErr[8], int outLinkErr[8]);

Error frame zeroing at slave port
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Error frame zeroing at slave port
    * @param [in] slaveID Slave station number 0 ~ 7
    * @return error code
    */
    errno_t SlavePortErrCounterClear(int slaveID);
    
Gets an example of a slave port error frame code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestSlavePortErr()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return 0;
        }
        robot.SetReConnectParam(true, 30000, 500);
        int inRecvErr[8] = {0.0}; 
        int inCRCErr[8] = { 0.0 };
        int inTransmitErr[8] = { 0.0 };
        int inLinkErr[8] = { 0.0 };
        int outRecvErr[8] = { 0.0 };
        int outCRCErr[8] = { 0.0 };
        int outTransmitErr[8] = { 0.0 };
        int outLinkErr[8] = { 0.0 };
        robot.GetSlavePortErrCounter(inRecvErr,  inCRCErr, inTransmitErr, inLinkErr,
            outRecvErr, outCRCErr, outTransmitErr, outLinkErr);
        for (int i = 0; i < 8; i++)
        {
            if (inRecvErr[i] != 0)
            {
                printf("inRecvErr %d is %d\n", i, inRecvErr[i]);
            }
            if (inCRCErr[i] != 0)
            {
                printf("inRecvErr %d is %d\n", i, inCRCErr[i]);
            }
            if (inTransmitErr[i] != 0)
            {
                printf("inRecvErr %d is %d\n", i, inTransmitErr[i]);
            }
            if (inLinkErr[i] != 0)
            {
                printf("inRecvErr %d is %d\n", i, inLinkErr[i]);
            }
            if (outRecvErr[i] != 0)
            {
                printf("outRecvErr %d is %d\n", i, outRecvErr[i]);
            }
            if (outCRCErr[i] != 0)
            {
                printf("outCRCErr %d is %d\n", i, outCRCErr[i]);
            }
            if (outTransmitErr[i] != 0)
            {
                printf("outTransmitErr %d is %d\n", i, outTransmitErr[i]);
            }
            if (outLinkErr[i] != 0)
            {
                printf("outLinkErr %d is %d\n", i, outLinkErr[i]);
            }
        }
        printf("others has no err!\n");
        for (int i = 0; i < 8; i++)
        {
            robot.SlavePortErrCounterClear(i);
        }
        robot.CloseRPC();
        return 0;
    }

Set feed-forward coefficients for each axis
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Set feed-forward coefficients for each axis
    * @param [in] radio Feed-forward coefficient of velocity of each axis
    * @return error code
    */
    errno_t SetVelFeedForwardRatio(double radio[6]);

Obtain the feed-forward coefficients of each axis
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief Obtain the feed-forward coefficients of each axis
    * @param [out] radio Feed-forward coefficient of velocity of each axis
    * @return error code
    */
    errno_t GetVelFeedForwardRatio(double radio[6]);

Example of robot velocity feed-forward coefficient code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestVelFeedForwardRatio()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(1);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return 0;
        }
        robot.SetReConnectParam(true, 30000, 500);
        double setRadio[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        robot.SetVelFeedForwardRatio(setRadio);
        double getRadio[6] = { 0.0 };
        robot.GetVelFeedForwardRatio(getRadio);
        printf(" %f %f %f %f %f %f\n", getRadio[0], getRadio[1], getRadio[2], getRadio[3], getRadio[4], getRadio[5]);
        robot.CloseRPC();
        return 0;
    }