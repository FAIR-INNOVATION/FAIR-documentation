Robot safety settings
================================ 

.. toctree::  
    :maxdepth: 5  

Set collision level  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set collision level  
    * @param  [in]  mode  0-level, 1-percentage  
    * @param  [in]  level Collision threshold. Level range: [1-10 corresponds to levels 1-10, 100-disabled]. Percentage range: [0~10 corresponds to 0% - 100%]  
    * @param  [in]  config 0-Do not update config file, 1-Update config file  
    * @return  Error code  
    */  
    int SetAnticollision(int mode, Object[] level, int config);  

Set post-collision strategy  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Set post-collision strategy  
    * @param  [in] strategy  0-Report error and stop, 1-Continue running  
    * @param  [in] safeTime  Safe stop time [1000 - 2000]ms  
    * @param  [in] safeDistance  Safe stop distance [1-150]mm  
    * @param  [in] safetyMargin  J1-J6 safety coefficients [1-10]  
    * @return  Error code    
    */  
    int SetCollisionStrategy(int strategy, int safeTime, int safeDistance, int safetyMargin[]);  

Custom collision detection threshold function start  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. versionadded:: Java SDK-v1.0.3-3.8.0  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Start custom collision detection threshold function. Set collision detection thresholds for joints and TCP  
    * @param  [in] flag 1-Joint detection only; 2-TCP detection only; 3-Both joint and TCP detection  
    * @param  [in] jointDetectionThreshould  Joint collision detection thresholds for J1-J6  
    * @param  [in] tcpDetectionThreshould  TCP collision detection thresholds for xyzabc  
    * @param  [in] block 0-Non-blocking; 1-Blocking  
    * @return  Error code  
    */     
    public int CustomCollisionDetectionStart(int flag, double[] jointDetectionThreshould, double[] tcpDetectionThreshould, int block);  

Custom collision detection threshold function end  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. versionadded:: Java SDK-v1.0.3-3.8.0  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  End custom collision detection threshold function  
    * @return  Error code  
    */     
    public int CustomCollisionDetectionEnd();  

Robot collision level setting code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    public static int TestCollision(Robot robot)  
    {  
        int mode = 0;  
        int config = 1;  
        Object[] level1 = new Object[]{ 1.0,2.0,3.0,4.0,5.0,6.0 };  
        Object[] level2 = new Object[]{ 50.0,20.0,30.0,40.0,50.0,60.0 };  

        int rtn = robot.SetAnticollision(mode, level1, config);  
        System.out.println("SetAnticollision mode 0 rtn is: "+ rtn);  
        mode = 1;  
        rtn = robot.SetAnticollision(mode, level2, config);  
        System.out.println("SetAnticollision mode 1 rtn is :"+ rtn);  

        JointPos p1Joint=new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);  
        JointPos p2Joint=new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);  

        DescPose p1Desc=new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);  
        DescPose p2Desc=new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);  

        ExaxisPos exaxisPos=new ExaxisPos(0.0, 0.0, 0.0, 0.0);  
        DescPose offdese=new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);  
        robot.MoveL(p2Joint, p2Desc, 0, 0, 100, 100, 100, 2,0, exaxisPos, 0, 0, offdese,0,10);  
        robot.ResetAllError();  
        int[] safety = new int[]{ 5,5,5,5,5,5 };  
        rtn = robot.SetCollisionStrategy(3, 1000, 150, 250, safety);  
        System.out.println("SetCollisionStrategy rtn is:"+ rtn);  

        double[] jointDetectionThreshould = new double[]{ 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };  
        double[] tcpDetectionThreshould =new double[] { 60,60,60,60,60,60 };  
        rtn = robot.CustomCollisionDetectionStart(3, jointDetectionThreshould, tcpDetectionThreshould, 0);  
        System.out.println("CustomCollisionDetectionStart rtn is :"+ rtn);  

        robot.MoveL(p1Joint, p1Desc, 0, 0, 100, 100, 100, -1,0, exaxisPos, 0, 0, offdese,0,10);  
        robot.MoveL(p2Joint, p2Desc, 0, 0, 100, 100, 100, -1,0, exaxisPos, 0, 0, offdese,0,10);  
        rtn = robot.CustomCollisionDetectionEnd();  
        System.out.println("CustomCollisionDetectionEnd rtn is: "+ rtn);  
        return 0;  
    }  

Set positive limit  
++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Set positive limit  
    * @param  [in] limit Six joint positions in deg  
    * @return  Error code  
    */  
    int SetLimitPositive(Object[] limit);  

Set negative limit  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Set negative limit  
    * @param  [in] limit Six joint positions in deg  
    * @return  Error code  
    */  
    int SetLimitNegative(Object[] limit);  

Get joint soft limit angles  
+++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief  Get joint soft limit angles  
    * @param  [in] flag 0-Blocking, 1-Non-blocking  
    * @param  [out] negative  Negative limit angles in deg  
    * @param  [out] positive  Positive limit angles in deg  
    * @return  Error code  
    */  
    int GetJointSoftLimitDeg(int flag, Object[] negative, Object[] positive);  

Robot limit setting code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    public static int TestLimit(Robot robot)  
    {  
        Object[] plimit =new Object[] { 170.0,80.0,150.0,80.0,170.0,160.0 };  
        robot.SetLimitPositive(plimit);  
        Object[] nlimit =new Object[] { -170.0,-260.0,-150.0,-260.0,-170.0,-160.0 };  
        robot.SetLimitNegative(nlimit);  

        Object[] neg_deg =new Object[] {0, 0 , 0, 0, 0, 0}, pos_deg = new Object[]{0, 0 , 0, 0, 0, 0};  
        robot.GetJointSoftLimitDeg(1,  neg_deg,  pos_deg);  
        System.out.println("neg limit deg:"+ neg_deg[0]+","+ neg_deg[1]+","+ neg_deg[2]+","+ neg_deg[3]+","+ neg_deg[4]+","+ neg_deg[5]);  
        System.out.println("pos limit deg:"+pos_deg[0]+","+ pos_deg[1]+","+ pos_deg[2]+","+ pos_deg[3]+","+ pos_deg[4]+","+pos_deg[5]);  
        return 0;  
    }  

Set robot collision detection method  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. versionchanged:: Java SDK-v1.0.5-3.8.2  

.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set robot collision detection method  
    * @param [in] method Collision detection method: 0-Current mode; 1-Dual encoder; 2-Both current and dual encoder  
    * @param [in] thresholdMode Collision level threshold method: 0-Fixed threshold; 1-Custom collision detection threshold  
    * @return Error code  
    */  
    int SetCollisionDetectionMethod(int method,int thresholdMode)  

Set static collision detection on/off  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Set static collision detection on/off  
    * @param  [in] status 0-Off; 1-On  
    * @return  Error code  
    */  
    public int SetStaticCollisionOnOff(int status)  

Robot collision detection method code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    public static int TestCollisionMethod(Robot robot)  
    {  
        int rtn = robot.SetCollisionDetectionMethod(0);  

        rtn = robot.SetStaticCollisionOnOff(1);  
        System.out.println("SetStaticCollisionOnOff On rtn is:"+ rtn);  
        robot.Sleep(5000);  
        rtn = robot.SetStaticCollisionOnOff(0);  
        System.out.println("SetStaticCollisionOnOff Off rtn is:"+ rtn);  

        robot.CloseRPC();  
        return 0;  
    }  

Joint torque power detection  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    /**  
    * @brief Joint torque power detection  
    * @param  [in] status 0-Off; 1-On  
    * @param  [in] power Set maximum power (W)  
    * @return  Error code  
    */  
    public int SetPowerLimit(int status, double power)  

Joint torque power detection code example  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
.. code-block:: Java  
    :linenos:  

    public static int TestPowerLimit(Robot robot)  
    {  
        robot.DragTeachSwitch(1);  
        robot.SetPowerLimit(1, 200);  
        List<Number> joint_toq=new ArrayList<>();  
        joint_toq=robot.GetJointTorques(1);  

        int count = 100;  
        robot.ServoJTStart(); //   #servoJT start  
        int error = 0;  
        while (count > 0)  
        {  
            count = count - 1;  
            robot.Sleep(1);  
        }  
        error = robot.ServoJTEnd();  
        robot.DragTeachSwitch(0);  

        robot.CloseRPC();  
        return 0;  
    }  

Set Safety Speed Parameters
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set safety speed parameters
    * @param enable 0-off; 1-enabled in manual mode; 2-enabled in all modes (automatic speed limiting not supported)
    * @param maxTCPVel Maximum TCP speed limit; [0-1000] mm/s
    * @param strategy Strategy after overspeed; 0-stop with alarm; 1-automatic speed limiting; 2-stop with alarm and disable
    * @param maxJointVel Maximum speed for 6 joints (°/s), default 45°/s
    * @return Error code
    */
    public int SetVelReducePara(int enable, double maxTCPVel, int strategy, double[] maxJointVel)
        
SDK Code Example for Setting Safety Speed Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestSetVelReducePara(Robot robot) {
        int rtn = 0;

        JointPos j1 = new JointPos(0, -90, 90, 0, 0, 0);
        JointPos j2 = new JointPos(90, -90, 90, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);

        robot.SetSpeed(80);
        rtn = robot.SetVelReducePara(2, 30, 1);
        System.out.printf("SetVelReducePara param error rtn is %d\n", rtn);

        rtn = robot.SetVelReducePara(0, 30, 1);
        System.out.printf("SetVelReducePara disable reduce vel rtn is %d\n", rtn);
        robot.MoveJ(j1, 0, 0, 100, 100, 100.0, epos, -1.0, 0, offset_pos);
        robot.MoveJ(j2, 0, 0, 100, 100, 100.0, epos, -1.0, 0, offset_pos);

        rtn = robot.SetVelReducePara(1, 30, 1);
        System.out.printf("SetVelReducePara reduce vel rtn is %d\n", rtn);
        robot.MoveJ(j1, 0, 0, 100, 100, 100.0, epos, -1.0, 0, offset_pos);
        robot.MoveJ(j2, 0, 0, 100, 100, 100.0, epos, -1.0, 0, offset_pos);

        rtn = robot.SetVelReducePara(2, 30, 2);
        System.out.printf("SetVelReducePara disable robot rtn is %d\n", rtn);
        robot.MoveJ(j1, 0, 0, 100, 100, 100.0, epos, -1.0, 0, offset_pos);
        robot.MoveJ(j2, 0, 0, 100, 100, 100.0, epos, -1.0, 0, offset_pos);

        robot.Sleep(2000);
        robot.ResetAllError();
        robot.RobotEnable(1);
        robot.Sleep(1000);

        rtn = robot.SetVelReducePara(2, 30, 0);
        System.out.printf("SetVelReducePara report error rtn is %d\n", rtn);
        robot.MoveJ(j1, 0, 0, 100, 100, 100.0, epos, -1.0, 0, offset_pos);
        robot.MoveJ(j2, 0, 0, 100, 100, 100.0, epos, -1.0, 0, offset_pos);

        robot.Sleep(1000);
        return 0;
    }

Code Example for Setting Robot Joint Safety Speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static int TestSetJointVelReducePara(Robot robot) {
        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();

        JointPos j1 = new JointPos(10.220, -11.121, -118.086, -46.739, 82.036, 131.503);
        JointPos j2 = new JointPos(89.782, -11.122, -118.086, -46.740, 82.036, 131.504);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        robot.SetSpeed(20);

        double[] maxJointVelA = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
        int rtn = robot.SetVelReducePara(2, 200, 0, maxJointVelA);
        System.out.printf("SetVelReducePara param error rtn is %d\n", rtn);
        robot.MoveJ(j1, 1, 2, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.MoveJ(j2, 1, 2, 100, 100, 100, epos, -1, 0, offset_pos);

        double[] maxJointVelB = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
        rtn = robot.SetVelReducePara(2, 200, 0, maxJointVelB);
        System.out.printf("SetVelReducePara reduce vel rtn is %d\n", rtn);
        robot.MoveJ(j1, 1, 2, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.MoveJ(j2, 1, 2, 100, 100, 100, epos, -1, 0, offset_pos);

        robot.Sleep(2000);
        return 0;
    }    