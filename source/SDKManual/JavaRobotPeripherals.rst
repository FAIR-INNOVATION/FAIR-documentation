Robotics peripherals
=====================================

.. toctree:: 
    :maxdepth: 5

Configuration of jaws
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Configuring the jaws
    * @param [in] config .company clamp jaw vendor, 1-Robotiq, 2-Huiling, 3-Tianji, 4-Dahuan, 5-Kiho
    * @param [in] config .device Device number, Robotiq (0-2F-85 series), Huiling (0-NK series,1-Z-EFG-100), Tianji (0-TEG-110), Dahuan (0-PGI-140), Zhixing (0-CTPM2F20)
    * @param [in] config .softvesion software version number, not used yet, default is 0.
    * @param [in] config .bus The device is hooked to the end bus location, which is not used at the moment and defaults to 0.
    * @return error code
    */
    int SetGripperConfig(DeviceConfig config).

Get Jaw Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get jaw configuration
    * @param [out] config .company Jaw Clamp Vendor, 1-Robotiq, 2-Huiling, 3-Tianji, 4-Dahuan, 5-Kiho
    * @param [out] config .device Device number, Robotiq (0-2F-85 series), Huiling (0-NK series,1-Z-EFG-100), Tianji (0-TEG-110), Dahuan (0-PGI-140), Zhixing (0-CTPM2F20)
    * @param [out] config .softvesion software version number, not used, default is 0
    * @param [out] config .bus device hangs on the end bus location, not used yet, default is 0
    * @return error code
    */
    int GetGripperConfig(DeviceConfig config).

Activate jaws
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Activate jaws
    * @param [in] index Jaw number
    * @param [in] act 0-reset, 1-activate
    * @return error code
    */
    int ActGripper(int index, int act). 

Control jaws
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Control jaws
    * @param [in] index Jaw number
    * @param [in] pos position percentage, range [0~100]
    * @param [in] vel velocity percentage, range [0~100]
    * @param [in] force Percentage of torque, range [0~100].
    * @param [in] max_time Maximum wait time, range [0~30000], unit ms
    * @param [in] block 0-blocking, 1-non-blocking
    * @param [in] type Jaw type, 0-parallel jaws; 1-rotary jaws
    * @param [in] rotNum number of rotations
    * @param [in] rotVel rotationVelocity percentage [0-100]
    * @param [in] rotTorque Percentage of rotational torque [0-100].
    * @return error code
    */
    int MoveGripper(int index, int pos, int vel, int force, int max_time, int block, int type, double rotNum, int rotVel, int rotTorque); 

Getting the jaw movement status
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Obtaining jaw motion status
    * @return List[0]:error code; List[1] : fault 0-no error, 1-with error; List[2]: staus 0-motion not completed, 1-motion completed
    */
    List<Integer> GetGripperMotionDone(); 

code example
++++++++++++++++
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
        int company = 3;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int deviceID = -1;

        DeviceConfig gripperConfig = new DeviceConfig(company, device, softversion, bus);

        robot.SetGripperConfig(gripperConfig);
        robot.Sleep(1000);

        DeviceConfig getConfig = new DeviceConfig();
        robot.GetGripperConfig(getConfig);
        System.out.println("gripper Vendor: " + getConfig.company + " , Type: " + getConfig.device + " , SoftwareVersion: " + getConfig.softwareVersion);

        int index = 1;
        byte act = 0;
        int max_time = 30000;
        byte block = 0;
        int status = -1, fault = -1;
        int rtn = -1;

        rtn = robot.ActGripper(index, act);//activate the jaws
        System.out.println("ActGripper rtn : " + rtn);
        act = 1;
        rtn = robot.ActGripper(index, act);
        System.out.println("ActGripper rtn : " + rtn);

        rtn = robot.MoveGripper(index, 80, 20, 50, max_time,0,0,0,0);//move the gripper jaws
        System.out.println("MoveGripper rtn : " + rtn);
        robot.Sleep(2000);
        robot.MoveGripper(index, 20, 20, 50, max_time, block,0,0,0,0);//move the gripper jaws

        robot.Sleep(4000);
        List<Integer> rtnArray = new ArrayList<Integer>() {};
        rtnArray=robot.GetGripperMotionDone();
        System.out.println("gripper motion done : " + rtnArray.get(2) + ", " + rtnArray.get(1));
    }

Calculate pre-capture point-visual
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Calculate pre-capture point-visuals 
    * @param [in] desc_pos Grab the point Cartesian position.
    * @param [in] zlength z-axis offset
    * @param [in] zangle rotational offset around z-axis
    * @param [out] pre_pos Get point
    * @return error code 
    */ 
    int ComputePrePick(DescPose desc_pos, double zlength, double zangle, DescPose pre_pos);

Calculate retreat point-visual
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Computational Retreat Points -- Visual 
    * @param [in] desc_pos Grab the point Cartesian position.
    * @param [in] zlength z-axis offset 
    * @param [in] zangle rotational offset around z-axis
    * @param [out] post_poss withdrawal point
    * @return error code 
    */ 
    int ComputePostPick(DescPose desc_pos, double zlength, double zangle, DescPose post_pos);

code example
++++++++++++++++
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
        int company = 3;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int deviceID = -1;

        DeviceConfig gripperConfig = new DeviceConfig(company, device, softversion, bus);

        robot.SetGripperConfig(gripperConfig);
        robot.Sleep(1000);

        DescPose desc_pos1, desc_pos2.
        desc_pos1 = new DescPose(-228.943, -584.228, 461.958,179.16, 5.559, 125.643);
        robot.ComputePrePick(desc_pos1, 10, 0, desc_pos2);
        System.out.println("ComputePrePick: " + desc_pos2.toString());

        desc_pos2.tran.x = 0;
        robot.ComputePostPick(desc_pos1, 10, 0, desc_pos2);
        System.out.println("ComputePostPick: " + desc_pos2.toString());
    }