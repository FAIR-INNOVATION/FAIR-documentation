Peripheral
====================

.. toctree:: 
    :maxdepth: 5

Configure the gripper
++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Configure the gripper 
    * @param [in] company The company of gripper，1-Robotiq，2-HITBOT，3-TIANJI，4-DAHUAN，5-ZHIXING 
    * @param [in] device Device number, not used yet. The default value is 0 
    * @param [in] softvesion Software version. The value is not used. The default value is 0 
    * @param [in] bus The device is attached to the terminal bus and is not in use. The default value is 0
    * @return Error code 
    */ 
    int SetGripperConfig(int company, int device, int softvesion, int bus);

Obtain the gripper configuration
+++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Obtain the gripper configuration 
    * @param [out] deviceID the id of gripper
    * @param [out] company the company of gripper，1-Robotiq，2-HITBOT，3-TIANJI，4-DAHUAN，5-ZHIXING 
    * @param [out] device device number, not used yet. The default value is 0  
    * @param [out] softvesion 0 Software version. The value is not used. The default value is 0
    * @return Error code
    */ 
    int GetGripperConfig(ref int deviceID, ref int company, ref int device, ref int softvesion); 

Activate gripper
++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Activate Activate gripper
    * @param  [in] index  gripper gripper
    * @param  [in] act  0- reset, 1- activate
    * @return  Error code
    */
    int ActGripper(int index, byte act); 

Control gripper
++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Control gripper
    * @param  [in] index  gripper number
    * @param  [in] pos  Percentage of position, range[0~100]
    * @param  [in] vel  Percentage of velocity, range[0~100]
    * @param  [in] force  Percentage of torque, range[0~100]
    * @param  [in] max_time  Maximum wait time, range[0~30000], unit: ms
    * @param  [in] block  0- blocking, 1- non-blocking
    * @return  Error code
    */
    int MoveGripper(int index, int pos, int vel, int force, int max_time, byte block);

Obtain the gripper motion state
+++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Obtain the gripper motion state
    * @param  [out] fault  0- no error, 1- error
    * @param  [out] staus  0- motion incomplete, 1- motion complete
    * @return  Error code
    */
    int GetGripperMotionDone(ref int fault, ref int status);

Code example
++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnOperateGripper_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2");

        int company = 4;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int index = 1;
        byte act = 0;
        int max_time = 30000;
        byte block = 0;
        int status = 0, fault = 0;
        int rtn = -1;

        robot.SetGripperConfig(company, device, softversion, bus);
        Thread.Sleep(1000);
        robot.GetGripperConfig(ref company, ref device, ref softversion, ref bus);
        Console.WriteLine($"gripper config : {company}, {device}, {softversion}, {bus}");

        rtn = robot.ActGripper(index, act);
        Console.WriteLine($"ActGripper  {rtn}");
        Thread.Sleep(1000);
        act = 1;
        rtn = robot.ActGripper(index, act);
        Console.WriteLine($"ActGripper  {rtn}");
        Thread.Sleep(4000);

        rtn = robot.MoveGripper(index, 20, 50, 50, max_time, block);
        Console.WriteLine($"MoveGripper  {rtn}");
        Thread.Sleep(2000);
        robot.MoveGripper(index, 10, 50, 0, max_time, block);

        Thread.Sleep(4000);
        robot.GetGripperMotionDone(ref fault, ref status);
        Console.WriteLine($"motion status : {fault}, {status}");
    }

Calculate pre-grab point - Vision
+++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Calculate pre-grab point - Vision 
    * @param [in] desc_pos Grab point cartesian pose 
    * @param [in] zlength z-axis offset 
    * @param [in] zangle Rotational offset about the z axis
    * @param [out] pre_pos pre-grab point cartesian pose
    * @return Error code 
    */ 
    int ComputePrePick(DescPose desc_pos, double zlength, double zangle, ref DescPose pre_pos);

Calculate retreat point - Vision
+++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Calculate retreat point - Vision 
    * @param [in] desc_pos Grab point cartesian pose
    * @param [in] zlength z-axis offset 
    * @param [in] zangle Rotational offset about the z axis
    * @param [out] post_pos retreat point cartesian pose
    * @return Error code 
    */ 
    int ComputePostPick(DescPose desc_pos, double zlength, double zangle, ref DescPose post_pos);
