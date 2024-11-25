Robot Peripherals
=========================

.. toctree:: 
    :maxdepth: 5


Configuring the gripper
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Configure the jaws.
    * @param [in] company Jaw manufacturer, to be determined.
    * @param [in] device Device number, not used yet, default is 0.
    * @param [in] softvesion software version number, not used, default is 0
    * @param [in] bus The device hangs on the end bus position, not used yet, default is 0.
    * @return Error code
    */
    int SetGripperConfig(int company, int device, int softvesion, int bus);

Get the gripper configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the jaw configuration.
    * @param [in] company Jaw manufacturer, to be determined.
    * @param [in] device Device number, not used yet, default is 0.
    * @param [in] softvesion software version number, not used, default is 0
    * @param [in] bus The device hangs on the end bus position, not used yet, default is 0.
    * @return Error code
    */
    int GetGripperConfig(int *company, int *device, int *softvesion, int *bus);

Activate the gripper
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Activate the jaws.
    * @param [in] index Jaw number.
    * @param [in] act 0-reset, 1-activate
    * @return Error code.
    */
    int ActGripper(int index, byte act). 

Controls the gripper.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Controls the jaws.
    * @param [in] index Jaw number.
    * @param [in] pos position percentage, range [0~100].
    * @param [in] vel velocity percentage, range [0~100].
    * @param [in] force % torque, range [0~100] * @param [in] max % position, range [0~100
    * @param [in] max_time Maximum wait time, range [0~30000], in ms
    * @param [in] block 0-blocking, 1-non-blocking
    * @param [in] type Gripper type, 0-parallel gripper; 1-rotary gripper
    * @param [in] rotNum Number of rotations.
    * @param [in] rotVel Percentage of rotational velocity [0-100].
    * @param [in] rotTorque Percentage of rotational torque [0-100].
    * @return Error code
    */
    int MoveGripper(int index, int pos, int vel, int force, int max_time, byte block, int type, double rotNum, int rotVel, int rotTorque);

Get the movement status of the gripper
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the state of the gripper motion.
    * @param [out] fault 0-no error, 1-error
    * @param [out] staus 0-motion not completed, 1-motion completed
    * @return Error code.
    */
    int GetGripperMotionDone(ref int fault, ref int status); 

Get Gripper Motion Done(ref fault, ref status)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the jaw activation status.
    * @param [out] fault 0-no fault, 1-fault
    * @param [out] status bit0~bit15 corresponds to gripper number 0~15, bit=0 is not activated, bit=1 is activated.
    * @return Error code
    */
    int GetGripperActivateStatus(ref int fault, ref int status);

Get gripper position
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the position of the jaws.
    * @param [out] fault 0-no error, 1-with error
    * @param [out] position position percentage, range 0~100
    * @return Error code.
    */
    int GetGripperCurPosition(ref int fault, ref int position);

Get the gripper cur speed
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the jaw speed.
    * @param [out] fault 0-no error, 1-error
    * @param [out] speed Speed percentage, range 0~100%.
    * @return Error code.
    */
    int GetGripperCurSpeed(ref int fault, ref int speed).
     
Get the gripper current
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the jaw current.
    * @param [out] fault 0-no error, 1-error
    * @param [out] current Percentage of current, range 0-100%.
    * @return Error code.
    */
    int GetGripperCurCurrent(ref int fault, ref int current);

Get the gripper voltage
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the jaw voltage.
    * @param [out] fault 0-no error, 1-error
    * @param [out] voltage Voltage in 0.1V.
    * @return Error code.
    */
    int GetGripperVoltage(ref int fault, ref int voltage).

Get the temperature of the gripper
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the jaw temperature.
    * @param [out] fault 0-no error, 1-with error
    * @param [out] temp Temperature in ℃.
    * @return Error code.
    */
    int GetGripperTemp(ref int fault, ref int temp);

Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

        int current = -1;
        int tempture = -1;
        int voltage = -1;
        int position = -1;
        int activestatus = -2;
        int speed = -1;
        rtn = robot.GetGripperCurCurrent(ref fault, ref current);
        Console.WriteLine($"current { current}  rtn { rtn} fault { fault} ");
        rtn = robot.GetGripperCurPosition(ref fault, ref position);
        Console.WriteLine($"position {position}  rtn {rtn} fault {fault} ");
        rtn = robot.GetGripperActivateStatus(ref fault, ref activestatus);
        Console.WriteLine($"activestatus {activestatus}  rtn {rtn} fault {fault} ");
        rtn = robot.GetGripperCurSpeed(ref fault, ref speed);
        Console.WriteLine($"speed {speed}  rtn {rtn} fault {fault} ");
        rtn = robot.GetGripperVoltage(ref fault, ref voltage);
        Console.WriteLine($"voltage {voltage}  rtn {rtn} fault {fault} ");
        rtn = robot.GetGripperTemp(ref fault, ref tempture);
        Console.WriteLine($"voltage {tempture}  rtn {rtn} fault {fault} ");
    }

Calculate pre-capture point-visual
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Calculate pre-capture points - visual 
    * @param [in] desc_pos Grab point Cartesian position. 
    * @param [in] zlength z-axis offset 
    * @param [in] zangle rotational offset around z-axis
    * @param [out] pre_pos pre_capture point
    * @return error code 
    */ 
    int ComputePrePick(DescPose desc_pos, double zlength, double zangle, ref DescPose pre_pos);

Calculate retreat point-visual
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * :: @brief Computational Retreat Point-Vision 
    * @param [in] desc_pos withdrawal point Cartesian position 
    * @param [in] zlength z-axis offset 
    * @param [in] zangle rotational offset around z-axis
    * @param [out] post_pos withdrawal point
    * @return error code 
    */ 
    int ComputePostPick(DescPose desc_pos, double zlength, double zangle, ref DescPose post_pos);