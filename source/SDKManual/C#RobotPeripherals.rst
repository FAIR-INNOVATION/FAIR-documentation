Robot Peripherals
=================

.. toctree:: 
    :maxdepth: 5

Configure Gripper
++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Configure gripper
    * @param  [in] company  Gripper manufacturer, to be determined
    * @param  [in] device  Device number, not currently used, default is 0
    * @param  [in] softvesion  Software version number, not currently used, default is 0
    * @param  [in] bus Device bus position, currently unused, default is 0
    * @return  Error code
    */
    int SetGripperConfig(int company, int device, int softvesion, int bus);

Get gripper configuration
++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get gripper configuration
    * @param  [in] company  Gripper manufacturer, to be determined
    * @param  [in] device  Device number, currently unused, default is 0
    * @param  [in] softvesion  Software version number, currently unused, default is 0
    * @param  [in] bus Device bus position, currently unused, default is 0
    * @return  Error code
    */
    int GetGripperConfig(int *company, int *device, int *softvesion, int *bus);

Activate Gripper
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Activate gripper
    * @param  [in] index  Gripper number
    * @param  [in] act  0-reset, 1-activate
    * @return  Error code
    */
    int ActGripper(int index, byte act);

Control gripper
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Control gripper
    * @param  [in] index  Gripper ID
    * @param  [in] pos  Position percentage, range [0~100]
    * @param  [in] vel  Speed percentage, range [0~100]
    * @param  [in] force  Torque percentage, range [0~100]
    * @param  [in] max_time  Maximum wait time, range [0~30000], unit ms
    * @param  [in] block  0-blocking, 1-non-blocking
    * @param  [in] type Gripper type, 0-parallel gripper; 1-rotating gripper
    * @param  [in] rotNum Number of rotation cycles
    * @param  [in] rotVel Rotation speed percentage [0-100]
    * @param  [in] rotTorque Rotation torque percentage [0-100]
    * @return Error code
    */
    int MoveGripper(int index, int pos, int vel, int force, int max_time, byte block, int type, double rotNum, int rotVel, int rotTorque);

Get gripper motion status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get gripper motion status
    * @param  [out] fault  0-no error, 1-error
    * @param  [out] staus  0-motion not completed, 1-motion completed
    * @return  Error code
    */
    int GetGripperMotionDone(ref int fault, ref int status); 

Get gripper activation status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get gripper activation status
    * @param  [out] fault  0-no error, 1-error
    * @param  [out] status  bit0~bit15 corresponds to gripper numbers 0~15, bit=0 is not activated, bit=1 is activated
    * @return  Error code
    */
    int GetGripperActivateStatus(ref int fault, ref int status);

Get Gripper Position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get gripper position
    * @param  [out] fault  0-no error, 1-error
    * @param  [out] position  Position percentage, range 0~100%
    * @return  Error code
    */
    int GetGripperCurPosition(ref int fault, ref int position);

Get gripper speed
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get gripper speed
    * @param  [out] fault  0-no error, 1-error
    * @param  [out] speed  Speed percentage, range 0~100%
    * @return  Error code
    */
    int GetGripperCurSpeed(ref int fault, ref int speed);
     
Get gripper current
++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get gripper current
    * @param  [out] fault  0-no error, 1-error
    * @param  [out] current  Current percentage, range 0~100%
    * @return Error code
    */
    int GetGripperCurrent(ref int fault, ref int current);

Get gripper voltage
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get gripper voltage
    * @param [out] fault 0-no error, 1-error
    * @param  [out] voltage  Voltage, unit 0.1V
    * @return  Error code
    */
    int GetGripperVoltage(ref int fault, ref int voltage);

Get gripper temperature
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get gripper temperature
    * @param  [out] fault  0-no error, 1-error
    * @param  [out] temp  Temperature, unit °C
    * @return  Error code
    */
    int GetGripperTemp(ref int fault, ref int temp);

Calculate pre-gripping point - vision
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Calculate pre-gripping point - vision 
    * @param [in] desc_pos Gripping point Cartesian pose 
    * @param [in] zlength Z-axis offset 
    * @param [in] zangle Rotation offset around the z-axis
    * @param [out] pre_pos Pre-pick point
    * @return Error code 
    */ 
    int ComputePrePick(DescPose desc_pos, double zlength, double zangle, ref DescPose pre_pos);

Calculate retreat point - vision
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Calculate retreat point - visual 
    * @param [in] desc_pos Retreat point Cartesian pose 
    * @param [in] zlength Z-axis offset 
    * @param [in] zangle Rotation offset around the Z-axis
    * @param [out] post_pos Retreat point
    * @return Error code 
    */ 
    int ComputePostPick(DescPose desc_pos, double zlength, double zangle, ref DescPose post_pos);

Robot Gripper Operation Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button36_Click(object sender, EventArgs e)
    {
        int company = 4;
        int device = 0;
        int softversion = 0;
        int bus = 2;
        int index = 2;
        byte act = 0;
        int max_time = 30000;
        byte block = 0;
        int status=0;
        int fault=0;
        int active_status = 0;
        int current_pos = 0;
        int current = 0;
        int voltage = 0;
        int temp = 0;
        int speed = 0;

        robot.SetGripperConfig(company, device, softversion, bus);
        Thread.Sleep(1000);
        robot.GetGripperConfig(ref company, ref device, ref softversion, ref bus);
        Console.WriteLine("gripper config:{0},{1},{2},{3}\n", company, device, softversion, bus);

        robot.ActGripper(index, act);
        Thread.Sleep(1000);
        act = 1;
        robot.ActGripper(index, act);
        Thread.Sleep(1000);

        robot.MoveGripper(index, 90, 50, 50, max_time, block, 0, 0, 0, 0);
        Thread.Sleep(1000);
        robot.MoveGripper(index, 30, 50, 0, max_time, block, 0, 0, 0, 0);

        robot.GetGripperMotionDone(ref fault, ref status);
        Console.WriteLine("motion status:{0},{1}\n", fault, status);

        robot.GetGripperActivateStatus(ref fault, ref active_status);
        Console.WriteLine("gripper active fault is: {0}, status is: {1}\n", fault, active_status);

        robot.GetGripperCurPosition(ref fault, ref current_pos);
        Console.WriteLine("fault is:{0}, current position is: {1}\n", fault, current_pos);

        robot.GetGripperCurCurrent(ref fault, ref current);
        Console.WriteLine("fault is:{0}, current current is: {1}\n", fault, current);

        robot.GetGripperVoltage(ref fault, ref voltage);
        Console.WriteLine("fault is:{0}, current voltage is: {1} \n", fault, voltage);

        robot.GetGripperTemp(ref fault, ref temp);
        Console.WriteLine("fault is:{0}, current temperature is: {1}\n", fault, temp);

        robot.GetGripperCurSpeed(ref fault, ref speed);
        Console.WriteLine("fault is:{0}, current speed is: {1}\n", fault, speed);

        int retval = 0;
        DescPose prepick_pose = new DescPose();
        DescPose postpick_pose = new DescPose();

        DescPose p1Desc = new DescPose(-419.524f, -13.000f, 351.569f, -178.118f, 0.314f, 3.833f);
        DescPose p2Desc = new DescPose(-321.222f, 185.189f, 335.520f, -179.030f, -1.284f, -29.869f);

        retval = robot.ComputePrePick(p1Desc, 10, 0, ref prepick_pose);
        Console.WriteLine("ComputePrePick retval is: {0}\n", retval);
        Console.WriteLine("xyz is: {0}, {1}, {2}; rpy is: {3}, {4}, {5}\n",
            prepick_pose.tran.x, prepick_pose.tran.y, prepick_pose.tran.z,
            prepick_pose.rpy.rx, prepick_pose.rpy.ry, prepick_pose.rpy.rz);

        retval = robot.ComputePostPick( p2Desc, -10, 0, ref postpick_pose);
        Console.WriteLine("ComputePostPick retval is: {0}\n", retval);
        Console.WriteLine("xyz is: {0}, {1}, {2}; rpy is: {3}, {4}, {5}\n",
            postpick_pose.tran.x, postpick_pose.tran.y, postpick_pose.tran.z,
            postpick_pose.rpy.rx, postpick_pose.rpy.ry, postpick_pose.rpy.rz);

    }

Get the number of rotations of the rotating gripper
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the number of rotations of the rotating gripper
    * @param  [out] fault  0-no error, 1-error
    * @param  [out] num  Number of rotations
    * @return  Error code
    */
    int GetGripperRotNum(ref UInt16 fault, ref double num);

Get the rotation speed percentage of the rotating gripper
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the rotation speed percentage of the rotating gripper
    * @param  [out] fault  0-no error, 1-error
    * @param  [out] speed  Rotation speed percentage
    * @return  Error code
    */
    int GetGripperRotSpeed(ref UInt16 fault, ref int speed);

Get the rotation torque percentage of the rotating gripper
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get the rotational torque percentage of the rotating gripper
    * @param  [out] fault  0-no error, 1-error
    * @param  [out] torque  Rotational torque percentage
    * @return  Error code
    */
    int GetGripperRotTorque(ref UInt16 fault, ref int torque);

Example of retrieving the rotational gripper status code
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    int MoveRotGripper(int pos, double rotPos)
    {
        robot.ResetAllError();
        robot.ActGripper(1, 1);
        Thread.Sleep(1000);
        int rtn = robot.MoveGripper(1, pos, 50, 50, 5000, 1, 1, rotPos, 50, 100);
        Console.WriteLine($"move gripper rtn is {rtn}" );
        UInt16 fault = 0;
        double rotNum = 0.0;
        int rotSpeed = 0;
        int rotTorque = 0;
        robot.GetGripperRotNum(ref fault, ref rotNum);
        robot.GetGripperRotSpeed(ref fault, ref rotSpeed);
        robot.GetGripperRotTorque(ref fault, ref rotTorque);
        Console.WriteLine($"gripper rot num :{ rotNum}, gripper rotSpeed :{rotSpeed}, gripper rotTorque : { rotTorque}");
        return 0;
    }

Drive belt start/stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Conveyor belt start/stop 
    * @param [in] status Status, 1-start, 0-stop
    * @return Error code 
    */ 
    int ConveyorStartEnd(byte status);

Record IO detection points
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Record IO detection point 
    * @return Error code 
    */ 
    int ConveyorPointIORecord(); 

Record point A
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Record Point A 
    * @return Error code 
    */ 
    int ConveyorPointARecord();

Record reference point
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Record reference point 
    * @return Error code 
    */ 
    int ConveyorRefPointRecord(); 

Record Point B
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Record Point B 
    * @return Error code 
    */ 
    int ConveyorPointBRecord();

Conveyor belt workpiece IO detection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Conveyor workpiece IO detection 
    * @param [in] max_t Maximum detection time, unit ms
    * @return Error code 
    */ 
    int ConveyorIODetect(int max_t);

Get Object Current Position
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Get object current position 
    * @param [in] mode 1-track grab, 2-track movement, 3-TPD tracking
    * @return Error code 
    */ 
    int ConveyorGetTrackData(int mode);

Start conveyor tracking
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Start conveyor tracking 
    * @param [in] status Status, 1-start, 0-stop
    * @return Error code 
    */
    int ConveyorTrackStart(byte status);

Conveyor tracking stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Conveyor tracking stop 
    * @return Error code 
    */
    int ConveyorTrackEnd();

Drive Belt Parameter Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Drive belt parameter configuration
    * @param [in] para[0] Encoder channel 1~2
    * @param [in] para[1] Number of pulses per encoder revolution
    * @param [in] para[2] Conveyor belt travel distance per encoder revolution
    * @param [in] para[3] Workpiece coordinate system number Select the workpiece coordinate system number for tracking motion functionality; set to 0 for tracking grasping and TPD tracking
    * @param [in] para[4] Whether to configure vision 0: No configuration 1: Configuration
    * @param [in] para[5] Speed ratio for conveyor belt tracking and grasping options (1-100). Other options default to 1.
    * @param [in] followType Tracking motion type: 0-Tracking motion; 1 - Inspection movement
    * @param [in] startDis Inspection tracking requires setting, tracking start distance, -1: automatically calculated (inspection starts automatically when the workpiece reaches below the robot), unit mm, default value 0
    * @param [in] endDis Inspection tracking requires setting, tracking end distance, unit mm, default value 100
    * @return Error code
    */
    int ConveyorSetParam(int encChannel, int resolution, double lead, int wpAxis, int vision, double speedRadio, int followType, int startDis=0, int endDis=100);

Set conveyor belt pickup point compensation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Set conveyor belt catch point compensation 
    * @param [in] cmp Compensation position double[3]{x, y, z}
    * @return Error code 
    */
    int ConveyorCatchPointComp(double[] cmp);

Conveyor belt tracking linear motion
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Conveyor belt tracking linear motion 
    * @param [in] name Motion point name
    * @param [in] tool Tool coordinate number, range [0~14] 
    * @param [in] wobj Workpiece coordinate number, range [0~14] 
    * @param [in] vel Velocity percentage, range [0~100] 
    * @param [in] acc Acceleration percentage, range [0~100], currently unavailable 
    * @param [in] ovl Velocity scaling factor, range [0~100] 
    * @param [in] blendR [-1.0] - move to position (blocked), [0~1000.0] - smooth radius (unblocked), unit mm  
    * @return Error code 
    */
    int ConveyorTrackMoveL(string name, int tool, int wobj, float vel, float acc, float ovl, float blendR);

Conveyor communication input detection
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Conveyor communication input detection
    * @param [in] timeout Wait timeout in ms
    * @return Error code
    */
    int ConveyorComDetect(int timeout);

Conveyor Communication Input Detection Trigger
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Conveyor Communication Input Detection Trigger
    * @return Error code
    */
    int ConveyorComDetectTrigger();

Conveyor Communication Input Detection Trigger Example Program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button3_Click(object sender, EventArgs e)
    {

        // Disable the button to prevent repeated clicks
        button3.Enabled = false;

        // Execute time-consuming operations in a background thread
        Thread conveyorThread = new Thread(ConveyorTest);
        conveyorThread.IsBackground = true;
        conveyorThread.Start();
    }

    private void button4_Click(object sender, EventArgs e)
    {
        // Get user input
        string input = texBox.Text;
        Console.WriteLine($"please input a number to trigger:{input}");
    
        int rtn = robot.ConveyorComDetectTrigger();
        Console.WriteLine($"ConveyorComDetectTrigger return value: {rtn}");
        
    }

    private void ConveyorTest()
    {
        // Use Invoke to update controls on the UI thread
        this.Invoke((MethodInvoker)delegate {
            Console.WriteLine( "Starting conveyor test...");
        });

        int retval = 0;
        int index = 1;
        int max_time = 30000;
        bool block = false;
        retval = 0;

        /* Conveyor belt grabbing process */
        DescPose startdescPose = new DescPose(139.176f, 4.717f, 9.088f, -179.999f, -0.004f, -179.990f);
        JointPos startjointPos = new JointPos(-34.129f, -88.062f, 97.839f, -99.780f, -90.003f, -34.140f);

        DescPose homePose = new DescPose(139.177f, 4.717f, 69.084f, -180.000f, -0.004f, -179.989f);
        JointPos homejointPos = new JointPos(-34.129f, -88.618f, 84.039f, -85.423f, -90.003f, -34.140f);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

        // Move to a safe position
        retval = robot.MoveL(homejointPos, homePose, 1, 1, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        Console.WriteLine($"MoveL to safe position return value: {retval}");

        // Conveyor detection
        retval = robot.ConveyComDetect(1000 * 10);
        Console.WriteLine($"ConveyorComDetect return value: {retval}");

        // Get tracking data
        retval = robot.ConveyorGetTrackData(2);
        Console.WriteLine($"ConveyorGetTrackData return value: {retval}");

        // Start tracking
        retval = robot.ConveyorTrackStart(2);
        Console.WriteLine($"ConveyorTrackStart return value: {retval}");

        // Move to the starting position
        robot.MoveL(startjointPos, startdescPose, 1, 1, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        robot.MoveL(startjointPos, startdescPose, 1, 1, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);

        // End tracking
        retval = robot.ConveyorTrackEnd();
        Console.WriteLine($"ConveyorTrackEnd return value: {retval}");

        // Return to safe position
        robot.MoveL(homejointPos, homePose, 1, 1, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);

        this.Invoke((MethodInvoker)delegate {
            Console.WriteLine( "Conveyor belt test completed!");
            button3.Enabled = true;
        });
    }

Robot Conveyor Belt Operation Example Program
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void btnConvert_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC( "192.168.58.2");
        DescPose pos1 = new DescPose(0, 0, 0, 0, 0, 0);
        DescPose pos2 = new DescPose(0, 0, 0, 0, 0, 0);

        pos1.tran.x = -351.175;
        pos1.tran.y = 3.389;
        pos1.tran.z = 431.172;
        pos1.rpy.rx = -179.111;
        pos1.rpy.ry = -0.241;
        pos1.rpy.rz = 90.388;

        pos2.tran.x = -333.654;
        pos2.tran.y = -229.003;
        pos2.tran.z = 404.335;
        pos2.rpy.rx = -179.139;
        pos2.rpy.ry = -0.779;
        pos2.rpy.rz = 91.269;
        int rtn = -1;

        double[] cmp = new double[3] { 0, 9.99, 0};
        rtn = robot.ConveyorCatchPointComp(cmp);
        if(rtn != 0)
        {
            return;
        }
        Console.WriteLine($"ConveyorCatchPointComp: rtn  {rtn}");

        rtn = robot.MoveCart(pos1, 0, 0, 100.0f, 180.0f, 100.0f, -1.0f, -1);
        Console.WriteLine($"MoveCart: rtn  {rtn}");

        rtn = robot.ConveyorIODetect(10000);
        Console.WriteLine($"ConveyorIODetect: rtn  {rtn}");

        robot.ConveyorGetTrackData(1);
        rtn = robot. ConveyorTrackStart(1);
        Console.WriteLine($"ConveyorTrackStart: rtn  {rtn}");

        rtn = robot.ConveyorTrackMoveL( "cvrCatchPoint", 0, 0, 100.0f, 0.0f, 100.0f, -1.0f, 0, 0);
        Console.WriteLine($"ConveyorTrackMoveL: rtn  {rtn}");

        rtn = robot.MoveGripper(1, 59, 43, 21, 30000, 0);
        Console.WriteLine($"MoveGripper: rtn  {rtn}");

        rtn = robot.ConveyorTrackMoveL( "cvrRaisePoint", 0, 0, 100.0f, 0.0f, 100.0f, -1.0f, 0, 0);
        Console.WriteLine($"ConveyorTrackMoveL: rtn  {rtn}");

        rtn = robot.ConveyorTrackEnd();
        Console.WriteLine($"ConveyorTrackEnd: rtn  {rtn}");

        rtn = robot.MoveCart(pos2, 0, 0, 100.0f, 180.0f, 100.0f, -1.0f, -1);
        Console.WriteLine($"MoveCart: rtn  {rtn}");

        rtn = robot.MoveGripper(1, 100, 43, 21, 30000, 0);
        Console.WriteLine($"MoveGripper: rtn  {rtn}");
    }

End Sensor Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  End Sensor Configuration
    * @param  [in] idCompany Manufacturer, 18-JUNKONG; 25-HUIDE
    * @param  [in] idDevice Type, 0-JUNKONG/RYR6T.V1.0
    * @param  [in] idSoftware Software version, 0-J1.0/HuiDe1.0 (not yet available)
    * @param  [in] idBus Mounting location, 1-End 1 port; 2-End 2 port... 8-Endpoint 8 port (not yet available)
    * @return Error code
    */
    int AxleSensorConfig(int idCompany, int idDevice, int idSoftware, int idBus);

Get endpoint sensor configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Get terminal sensor configuration
    * @param  [out] idCompany Manufacturer, 18-JUNKONG; 25-HUIDE
    * @param  [out] idDevice Type, 0-JUNKONG/RYR6T.V1.0
    * @return  Error code
    */
    int AxleSensorConfigGet(ref int idCompany, ref int idDevice);

End-of-line sensor activation
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
    
    /**
    * @brief  End-of-line sensor activation
    * @param  [in] actFlag 0-reset; 1-activate
    * @return  Error code
    */
    int AxleSensorActivate(int actFlag);

End Sensor Register Write
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  End Sensor Register Write
    * @param  [in] devAddr  Device Address Number 0-255
    * @param  [in] regHAddr High 8 bits of the register address
    * @param  [in] regLAddr Low 8 bits of the register address
    * @param  [in] regNum  Number of registers 0-255
    * @param  [in] data1 Value to write to the register 1
    * @param  [in] data2 Value to write to the register 2
    * @param  [in] isNoBlock 0-blocking; 1-non-blocking
    * @return  Error code
    */
     int AxleSensorRegWrite(int devAddr, int regHAddr, int regLAddr, int regNum, int data1, int data2, int isNoBlock);

End Sensor Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button2_Click_1(object sender, EventArgs e)
    {
        robot.AxleSensorConfig(18, 0, 0, 1);
        int company = -1;
        int type = -1;
        robot.AxleSensorConfigGet(ref company, ref type);
        Console.WriteLine( "company is " + company +  ", type is " + type);

        int rtn = robot.AxleSensorActivate(1);
        Console.WriteLine( "AxleSensorActivate rtn is " + rtn);

        Thread.Sleep(1000);

        rtn = robot.AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0);
        Console.WriteLine( "AxleSensorRegWrite rtn is " + rtn);   
    }

Obtain robot peripheral protocol
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Get robot peripheral protocol
    * @param [out] protocol Robot peripheral protocol number 4096-Extended Axle Control Card; 4097-ModbusSlave; 4098-ModbusMaster
    * @return Error code 
    */
    int GetExDevProtocol(ref int protocol);

Set robot peripheral protocol
++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:

    /** 
    * @brief Set robot peripheral protocol
    * @param [in] protocol Robot peripheral protocol number 4096-Extended Axis Control Card; 4097-Modbus Slave; 4098-Modbus Master
    * @return Error code 
    */
    int SetExDevProtocol(int protocol);

Example program for setting robot peripheral protocol
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:


    private void btnSetProto_Click(object sender, EventArgs e)
    {
      int protocol = 4096;
      int rtn = robot.SetExDevProtocol(protocol);
      
      Console.WriteLine( "SetExDevProtocol rtn " + rtn);
      rtn = robot.GetExDevProtocol(ref protocol);
      Console.WriteLine( "GetExDevProtocol rtn " + rtn +  " protocol is: " + protocol);
    }

Get end-point communication parameters
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get terminal communication parameters
    * @param param Terminal communication parameters
    * @return  Error code
    */
    int GetAxleCommunicationParam(ref AxleComParam getParam);

Set terminal communication parameters
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set terminal communication parameters
    * @param param  Terminal communication parameters
    * @return  Error code
    */
    int SetAxleCommunicationParam(AxleComParam param);

Set terminal file transfer type
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the terminal file transfer type
    * @param type 1-MCU upgrade file; 2-LUA file
    * @return  Error code
    */
    int SetAxleFileType(int type);

Set enable terminal LUA execution
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Enable LUA execution at the end
    * @param enable 0-Disabled; 1-Enabled
    * @return  Error code
    */
    int SetAxleLuaEnable(int enable);

End LUA file exception error recovery
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief End-of-file LUA file exception error recovery
    * @param status 0-do not recover; 1-recover
    * @return Error code
    */
    int SetRecoverAxleLuaErr(int status);

Get the enable status of the terminal LUA execution
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the enable status of the terminal LUA execution
    * @param [out] status 0-disabled; 1-enabled
    * @return  Error code
    */
    int GetAxleLuaEnableStatus(ref int status);

Set the enable type of the terminal LUA terminal device
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the enable type of the terminal LUA terminal device
    * @param [in] forceSensorEnable Force sensor enable status, 0-disabled; 1-enabled
    * @param [in] gripperEnable Gripper enable status, 0-disabled; 1-enabled
    * @param [in] IOEnable IO device enable status, 0-disabled; 1-enabled
    * @return  Error code
    */
    int SetAxleLuaEnableDeviceType(int forceSensorEnable, int gripperEnable, int IOEnable);

Get the enable type of the end-of-line LUA device
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the enable type of the end LUA device
    * @param [out] forceSensorEnable Force sensor enable status, 0-disabled; 1-enabled
    * @param [out] gripperEnable Gripper enable status, 0-disabled; 1-enabled
    * @param [out] IOEnable IO device enable status, 0-disabled; 1-enabled
    * @return  Error code
    */
    int GetAxleLuaEnableDeviceType(ref int forceSensorEnable, ref int gripperEnable, ref int IOEnable);

Get the currently configured end device
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the currently configured end device
    * @param [out] forceSensorEnable Force sensor enable device number 0-disabled; 1-enabled
    * @param [out] gripperEnable Gripper enable device number, 0-disabled;1-enabled
    * @param [out] IODeviceEnable IO device enable ID, 0-disabled; 1-enabled
    * @return  Error code
    */
    int GetAxleLuaEnableDevice(ref int[] forceSensorEnable, ref int[] gripperEnable, ref int[] IODeviceEnable);

Set enable gripper action control functionality
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set enable claw action control function
    * @param [in] id claw device number
    * @param [in] func func[0]-claw enable; func[1]-claw initialization; 2-position setting;3-Set speed; 4-Set torque; 6-Read gripper status; 7-Read initialization status; 8-Read fault code; 9-Read position; 10-Read speed;11-read torque
    * @return  Error code
    */
    int SetAxleLuaGripperFunc(int id, int[] func);

Get enable claw action control function
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get enable claw action control function
    * @param [in] id Gripper device ID
    * @param [out] func func[0]-Gripper enable; func[1]-Gripper initialization; 2-Position setting; 3-Speed setting; 4-Torque setting; 6-Read gripper status; 7-Read initialization status; 8-Read fault code; 9-Read position; 10-Read speed; 11-Read torque
    * @return  Error code
    */
    int GetAxleLuaGripperFunc(int id, ref int[] func);

Writing robot Ethercat slave file
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Writing robot Ethercat slave file
    * @param [in] type Slave file type, 1-upgrade slave file; 2-upgrade slave configuration file
    * @param [in] slaveID Slave ID
    * @param [in] fileName Upload file name
    * @return  Error code
    */
    int SlaveFileWrite(int type, int slaveID, string fileName);

Upload terminal Lua open protocol file
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Upload end Lua open protocol file
    * @param filePath Local lua file path name  ".../AXLE_LUA_End_DaHuan.lua"
    * @return Error code
    */       
    int AxleLuaUpload(string filePath);

Robot Ethercat slave enters boot mode
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Robot Ethercat slave enters boot mode
    * @return  Error code
    */
    int SetSysServoBootMode();

Robot End-of-Arm LUA File Operation Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     private void button41_Click(object sender, EventArgs e)
    {
        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
        robot.AxleLuaUpload("D://zUP/AXLE_LUA_End_JunDuo_Xinjingcheng.lua");

        AxleComParam param = new AxleComParam(7, 8, 1, 0, 5, 3, 1);
        robot.SetAxleCommunicationParam(param);

        AxleComParam getParam = new AxleComParam();
        robot.GetAxleCommunicationParam(ref getParam);
        Console.WriteLine("GetAxleCommunicationParam param is {0} {1} {2} {3} {4} {5} {6}",
            getParam.baudRate, getParam.dataBit, getParam.stopBit, getParam.verify,
            getParam.timeout, getParam.timeoutTimes, getParam.period);

        robot.SetAxleLuaEnable(1);
        int luaEnableStatus = 0;
        robot.GetAxleLuaEnableStatus(ref luaEnableStatus);
        robot.SetAxleLuaEnableDeviceType(0, 1, 0);

        int forceEnable = 0;
        int gripperEnable = 0;
        int ioEnable = 0;
        robot.GetAxleLuaEnableDeviceType(ref forceEnable, ref gripperEnable, ref ioEnable);
        Console.WriteLine("GetAxleLuaEnableDeviceType param is {0} {1} {2}", forceEnable, gripperEnable, ioEnable);

        int[] func = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
        robot.SetAxleLuaGripperFunc(1, func);
        int[] getFunc = new int[16];
        robot.GetAxleLuaGripperFunc(1, ref getFunc);
        int[] getforceEnable = new int[16];
        int[] getgripperEnable = new int[16];
        int[] getioEnable = new int[16];
        robot.GetAxleLuaEnableDevice(ref getforceEnable, ref getgripperEnable, ref getioEnable);
        Console.WriteLine("\ngetforceEnable status : ");
        foreach (int i in getforceEnable)
        {
            Console.Write(i + ",");
        }
        Console.WriteLine("\ngetgripperEnable status : ");
        foreach (int i in getgripperEnable)
        {
            Console.Write(i + ",");
        }
        Console.WriteLine("\ngetioEnable status : ");
        foreach (int i in getioEnable)
        {
            Console.Write(i + ",");
        }
        Console.WriteLine();
        robot.ActGripper(1, 0);
        Thread.Sleep(2000);
        robot.ActGripper(1, 1);
        Thread.Sleep(2000);
        robot.MoveGripper(1, 90, 10, 100, 50000, 0, 0, 0, 0, 0);
        int pos = 0;
        while (true)
        {
            robot.GetRobotRealTimeState(ref pkg);
            Console.WriteLine("gripper pos is " + pkg.gripper_position);
            Thread.Sleep(100);
        }
    }
    
Get SmartTool button status
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Get SmartTool button status
    * @param [out] state SmartTool handle button status; (bit0: 0 - communication normal; 1 - communication disconnected; bit1 - cancel operation; bit2 - clear program; 
    bit3 - A key; bit4 - B key; bit5 - C key; bit6 - D key; bit7 - E key; bit8 - IO key; bit9 - manual/automatic; bit10 - start)
    * @return Error code
    */
    int GetSmarttoolBtnState(ref int state);

Code example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3  Web-3.8.2
    
.. code-block:: c#
    :linenos:

    private void button11_Click(object sender, EventArgs e)
    {

        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
        int state = 0;
        while (true)
        {
            int rtn = robot.GetSmarttoolBtnState(ref state);
            string binaryString = Convert.ToString(state, 2).PadLeft(32, '0');
            Console.WriteLine($"GetSmarttoolBtnState rtn (binary): {binaryString}");
            Thread.Sleep(100);
        }

    }

Upload Open Protocol Lua File
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Upload Open Protocol Lua File
    * @param  filePath Local open protocol lua file path name
    * @return Error code
    */
    public int OpenLuaUpload(String filePath)


Get Slave Board Parameters
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief  Get Slave Board Parameters
    * @param  type  0-Ethercat, 1-CClink, 3-Ethercat, 4-EIP
    * @param  version  Protocol version
    * @param  connState  0-Disconnected 1-Connected
    * @return  Error code
    */
    public int GetFieldBusConfig(int[] type, int[] version, int[] connState)

Write Slave DO
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief  Write Slave DO
    * @param   DOIndex  DO number
    * @param   wirteNum  Number to write
    * @param   status Value to write, max 8
    * @return  Error code
    */
    public int FieldBusSlaveWriteDO(int DOIndex, int wirteNum, int[] status)

Write Slave AO
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief  Write Slave AO
    * @param  AOIndex  AO number
    * @param  wirteNum  Number to write
    * @param  status Value to write, max 8
    * @return  Error code
    */
    public int FieldBusSlaveWriteAO(int AOIndex, int wirteNum, int[] status)

Read Slave DI
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief  Read Slave DI
    * @param  DOIndex  DI number
    * @param  readNum  Number to read
    * @param  status Read value, max 8
    * @return  Error code
    */
    public int FieldBusSlaveReadDI(int DOIndex, int readNum, int[] status)

Read Slave AI
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief  Read Slave AI
    * @param  AIIndex  AI number
    * @param  readNum  Number to read
    * @param  status Read value, max 8
    * @return  Error code
    */
    public int FieldBusSlaveReadAI(int AIIndex, int readNum, double[] status)

Wait for Extended DI Input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Wait for Extended DI Input
    * @param  DIIndex DI number
    * @param  status 0-Low level; 1-High level
    * @param  waitMs Max waiting time (ms)
    * @return Error code
    */
    public int FieldBusSlaveWaitDI(int DIIndex, int status, int waitMs)

Wait for Extended AI Input
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Wait for Extended AI Input
    * @param  AIIndex AI number
    * @param  waitType 0-Greater than; 1-Less than
    * @param  value AI value
    * @param  waitMs Max waiting time (ms)
    * @return Error code
    */
    public int FieldBusSlaveWaitAI(int AIIndex, int waitType, double value, int waitMs)

Slave Mode Related Interface Command Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void button101_Click(object sender, EventArgs e)
    {
        int rtn = 0;
    
        int type = 0, version = 0, connState = 0;
        int[] ctrl = new int[8];
        int[] ctrlAO = new int[8];
        int[] DI = new int[8];
        double[] AI = new double[8];
        if (rtn != 0)
        {
            return;
        }
        // Upload and load open protocol file
        robot.OpenLuaUpload("E://zup/CtrlDev_field.lua");
        Thread.Sleep(2000);
        robot.SetCtrlOpenLUAName(3, "CtrlDev_field.lua");
        robot.UnloadCtrlOpenLUA(3);
        robot.LoadCtrlOpenLUA(3);
        Thread.Sleep(8000);
    
        // Get protocol type, software version, and connection status with PLC
        robot.GetFieldBusConfig(ref type, ref version, ref connState);
        Console.WriteLine($"type is {type}, version is {version}, connState is {connState}");
    
        // Write DO0 = 1, DO1 = 0, DO2 = 1
        ctrl[0] = 1;
        ctrl[1] = 0;
        ctrl[2] = 1;
        robot.FieldBusSlaveWriteDO(0, 3, ctrl);
    
        // Write AO2 = 0x1000
        ctrlAO[0] = 0x1000;
        robot.FieldBusSlaveWriteAO(2, 1, ctrlAO);

        for (int i = 0; i < 100; i++)
        {
            robot.FieldBusSlaveReadDI(0, 4, ref DI);
            Console.WriteLine($"DI0 is {DI[0]}, DI1 is {DI[1]}, DI2 is {DI[2]}, DI3 is {DI[3]}");
            robot.FieldBusSlaveReadAI(0, 3, ref AI);
            Console.WriteLine($"AI0 is {AI[0]}, AI1 is {AI[1]}, AI2 is {AI[2]}");
            Thread.Sleep(10);
        }
        int ret = robot.FieldBusSlaveWaitDI(0, 1, 100);
        Console.WriteLine($"FieldBusSlaveWaitDI result is {ret}");

        ret = robot.FieldBusSlaveWaitAI(0, 0, 400.00f, 100);
        Console.WriteLine($"FieldBusSlaveWaitAI result is {ret}"); 
    }

Control Array Sucker
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Control Array Sucker
    * @param  slaveID Slave ID
    * @param  len Length
    * @param  ctrlValue Control value 1-Suction at max vacuum; 2-Suction at set vacuum; 3-Stop suction
    * @return Error code
    */
    public int SetSuckerCtrl(int slaveID, int len, int[] ctrlValue)

Get Array Sucker Status
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Get Array Sucker Status
    * @param  slaveID Slave ID
    * @param  state Adsorption state 0-Release object; 1-Workpiece detected and adsorbed successfully; 2-No object adsorbed; 3-Object detached
    * @param  pressValue Current vacuum degree Unit kpa
    * @param  error Sucker current error code
    * @return Error code
    */
    public int GetSuckerState(int slaveID, int[] state, int[] pressValue, int[] error)

Wait for Sucker Status
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Wait for Sucker Status
    * @param  slaveID Slave ID
    * @param  state Adsorption state 0-Release object; 1-Workpiece detected and adsorbed successfully; 2-No object adsorbed; 3-Object detached
    * @param  ms Max waiting time
    * @return Error code
    */
    public int WaitSuckerState(int slaveID, int state, int ms)

Array Sucker Control Command Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private void TestSucker(Robot robot)
    {
    
        int[] ctrl = new int[20];
        int state=0;
        int pressValue=0;
        int error=0;
        int rtn;
    
    
        // Upload and load open protocol file
        robot.OpenLuaUpload(@"C:\SDK\CtrlDev_sucker.lua");
        Thread.Sleep(2000);
        robot.UnloadCtrlOpenLUA(1);
        robot.LoadCtrlOpenLUA(1);
        Thread.Sleep(1000);
    
        // Control sucker in broadcast mode with maximum adsorption capacity
        ctrl[0] = 1;
        robot.SetSuckerCtrl(0, 1, ctrl);
    
        // Monitor states of sucker 1 and sucker 12 in a loop
        for (int i = 0; i < 100; i++)
        {
            robot.GetSuckerState(1, ref state, ref pressValue, ref error);
            Console.WriteLine($"sucker1 state is {state}, pressValue is {pressValue}, error num is {error}");
            robot.GetSuckerState(12, ref state, ref pressValue, ref error);
            Console.WriteLine($"sucker12 state is {state}, pressValue is {pressValue}, error num is {error}");
            Thread.Sleep(100);
        }
        // Wait for sucker 1 to reach adsorbed state, timeout 100ms
        int ret = robot.WaitSuckerState(1, 1, 100);
        Console.WriteLine($"WaitSuckerState result is {ret}");
    
        // Unicast mode to turn off sucker 1 and 12
        ctrl[0] = 3;
        robot.SetSuckerCtrl(1, 1, ctrl);
        robot.SetSuckerCtrl(12, 1, ctrl);
    
        robot.CloseRPC();
    }

Laser peripheral on/off function
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Laser peripheral on/off function
    * @param [in] OnOff 0-off 1-on
    * @param [in] weldId Weld seam ID, default is 0
    * @return Error code
    */
    public int LaserTrackingLaserOnOff(int OnOff, int weldId)
    
Laser tracking start/stop function
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:

    
    /**
    * @brief Laser tracking start/stop function
    * @param [in] OnOff 0-stop 1-start
    * @param [in] coordId Laser peripheral tool coordinate system number
    * @return Error code
    */
    public int LaserTrackingTrackOnOff(int OnOff, int coordId)

Laser positioning - fixed direction
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Laser positioning - fixed direction
    * @param [in] direction 0-x+ 1-x- 2-y+ 3-y- 4-z+ 5-z-
    * @param [in] vel Speed in %
    * @param [in] distance Maximum positioning distance in mm
    * @param [in] timeout Positioning timeout in ms
    * @param [in] posSensorNum Laser calibrated tool coordinate number
    * @return Error code
    */
    public int LaserTrackingSearchStart_xyz(int direction, int vel, int distance, int timeout, int posSensorNum)
    
Laser positioning - arbitrary direction
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Laser positioning - arbitrary direction
    * @param [in] directionPoint XYZ coordinates of the positioning input point
    * @param [in] vel Speed in %
    * @param [in] distance Maximum positioning distance in mm
    * @param [in] timeout Positioning timeout in ms
    * @param [in] posSensorNum Laser calibrated tool coordinate number
    * @return Error code
    */
    public int LaserTrackingSearchStart_point(DescTran directionPoint, int vel, int distance, int timeout, int posSensorNum)
   
Laser positioning stop
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Laser positioning stop
    * @return Error code
    */
    public int LaserTrackingSearchStop()

Laser IP configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Laser IP configuration
    * @param [in] ip IP address of the laser peripheral
    * @param [in] port Port number of the laser peripheral
    * @return Error code
    */
    public int LaserTrackingSensorConfig(string ip, int port)

Laser peripheral sampling period configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Laser peripheral sampling period configuration
    * @param [in] period Laser peripheral sampling period in ms
    * @return Error code
    */
    public int LaserTrackingSensorSamplePeriod(int period)

Laser peripheral driver loading
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Laser peripheral driver loading
    * @param [in] type Laser peripheral driver protocol type 101-Ruiniu 102-Chuangxiang 103-Quanshi 104-Tongzhou 105-Aotai
    * @return Error code
    */
    public int LoadPosSensorDriver(int type)

Laser Peripheral Driver Unloading
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Laser peripheral driver unloading
    * @return Error code
    */
    public int UnLoadPosSensorDriver()

Laser Weld Seam Trajectory Recording
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Laser weld seam trajectory recording
    * @param [in] status 0-stop recording 1-real-time tracking 2-start recording
    * @param [in] delayTime Delay time in ms
    * @return Error code
    */
    public int LaserSensorRecord1(int status, int delayTime)

Laser Weld Seam Trajectory Replay
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Laser weld seam trajectory replay
    * @param [in] delayTime Delay time in ms
    * @param [in] speed Speed in %
    * @return Error code
    */
    public int LaserSensorReplay(int delayTime, double speed)

Laser Tracking Replay
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Laser tracking replay
    * @return Error code
    */
    public int MoveLTR()

Laser Weld Seam Trajectory Recording and Replay
++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Laser Seam Trajectory Recording and Replay
    * @param [in] delayMode Mode 0-Delay Time 1-Delay Distance
    * @param [in] delayTime Delay time in milliseconds (ms)
    * @param [in] delayDisExAxisNum Extended Axis Number
    * @param [in] delayDis Delay distance in millimeters (mm)
    * @param [in] sensitivePara Compensation Sensitivity Coefficient
    * @param [in] trackMode Fixed-point Tracking Type. 0-Extended Axis Asynchronous Motion; 1-Robot
    * @param [in] triggerMode Fixed-point Tracking Trigger Method. 0-Tracking Duration; 1-IO
    * @param [in] runTime Robot Fixed-point Tracking Duration in seconds (s)
    * @param [in] speed Speed in percentage (%)
    * @return Error Code
    */
    public int LaserSensorRecordandReplay(int delayMode, int delayTime, int delayDisExAxisNum,double delayDis, double sensitivePara, int trackMode, int triggerMode,double runTime, double speed)

Move to Laser Record Start Point
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Move to laser record start point
    * @param [in] moveType 0-PTP 1-LIN
    * @param [in] ovl Speed in %
    * @return Error code
    */
    public int MoveToLaserRecordStart(int moveType, double ovl)

Move to Laser Record End Point
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Move to laser record end point
    * @param [in] moveType 0-PTP 1-LIN
    * @param [in] ovl Speed in %
    * @return Error code
    */
    public int MoveToLaserRecordEnd(int moveType, double ovl)

Move to Laser Sensor Positioning Point
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Move to laser sensor positioning point
    * @param [in] moveFlag Motion type: 0-PTP; 1-LIN
    * @param [in] ovl Speed scaling factor, 0-100
    * @param [in] dataFlag Weld seam cache data selection: 0-execute planning data; 1-execute recorded data
    * @param [in] plateType Plate type: 0-corrugated plate; 1-corrugated cardboard; 2-fence plate; 3-oil drum; 4-corrugated shell steel
    * @param [in] trackOffectType Laser sensor offset type: 0-no offset; 1-base coordinate system offset; 2-tool coordinate system offset; 3-laser sensor raw data offset
    * @param [in] offset Offset value
    * @return Error code
    */
    public int MoveToLaserSeamPos(int moveFlag, double ovl, int dataFlag, int plateType, int trackOffectType, DescPose offset)

    
Get laser sensor positioning point coordinate information
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    /**
    * @brief Get laser sensor positioning point coordinate information
    * @param [in] trackOffectType Laser sensor offset type: 0-no offset; 1-base coordinate system offset; 2-tool coordinate system offset; 3-laser sensor raw data offset
    * @param [in] offset Offset value
    * @param [out] jPos Joint position [°]
    * @param [out] descPos Cartesian position [mm]
    * @param [out] tool Tool coordinate system
    * @param [out] user Workpiece coordinate system
    * @param [out] exaxis Extended axis position [mm]
    * @return Error code
    */
    public int GetLaserSeamPos(int trackOffectType, DescPose offset, ref JointPos jPos, ref DescPose descPos, ref int tool, ref int user, ref ExaxisPos exaxis)

Laser Peripheral Sensor Parameter Configuration and Debugging Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    void testLaserConfig()
    {
        int[] ctrl = new int[20];
        int state;
        int pressValue;
        int error;
        robot.LaserTrackingSensorConfig("192.168.58.20", 5020);
        robot.LaserTrackingSensorSamplePeriod(20);
        robot.LoadPosSensorDriver(101);
        robot.LaserTrackingLaserOnOff(0, 0);
        System.Threading.Thread.Sleep(3000);
        robot.LaserTrackingLaserOnOff(1, 0);
    }

Laser Trajectory Scanning and Trajectory Replay Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    void testLaserRecordAndReplay()
    { 
        int[] ctrl = new int[20];
        int state;
        int pressValue;
        int error;
        robot.OpenLuaUpload("D://zUP/CtrlDev_laser_ruiniu-0117.lua");
        System.Threading.Thread.Sleep(2000);
        robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua");
        robot.UnloadCtrlOpenLUA(0);
        robot.LoadCtrlOpenLUA(0);
        System.Threading.Thread.Sleep(8000);
        for (int i=0;i<10;++i)
        {
            JointPos startjointPos = new JointPos(56.205, -117.951, 141.872, -118.149, -94.217, -122.176);
            DescPose startdescPose = new DescPose(-97.552, -282.855, 26.675, 174.182, -1.338, -91.707);
            ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
            DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

            robot.MoveL(startjointPos, startdescPose, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0);
            robot.LaserSensorRecord1(2, 10);

            JointPos endjointPos = new JointPos(68.809, -87.100, 121.120, -127.233, -95.038, -109.555);
            DescPose enddescPose = new DescPose(-103.555, -464.234, 13.076, 174.179, -1.344, -91.709);
            robot.MoveL(endjointPos, enddescPose, 1, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese, 0);

            robot.LaserSensorRecord1(0, 10);
            robot.MoveToLaserRecordStart(1, 30);
            robot.LaserSensorReplay(10, 100);
            robot.MoveLTR();
            robot.LaserSensorRecord1(0, 10);
            Console.WriteLine($"Number of completions : {i+1} ");
        }
                
    }

Laser Positioning and Real-time Tracking Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    public static void testLasertrack()
    {
        int[] ctrl = new int[20];
        int state;
        int pressValue;
        int error;
        robot.OpenLuaUpload("D://zUP/CtrlDev_laser_ruiniu-0117.lua");
        System.Threading.Thread.Sleep(2000);
        robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua");
        robot.UnloadCtrlOpenLUA(0);
        robot.LoadCtrlOpenLUA(0);
        System.Threading.Thread.Sleep(8000);
        for (int i = 0; i < 10; ++i)
        {
            JointPos startjointPos = new JointPos(56.205, -117.951, 141.872, -118.149, -94.217, -122.176);
            DescPose startdescPose = new DescPose(-97.552, -282.855, 26.675, 174.182, -1.338, -91.707);
            ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
            DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
            DescTran directionPoint = new DescTran();

            robot.MoveL(startjointPos, startdescPose, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0);
            robot.LaserTrackingSearchStart_xyz(3, 100, 300, 1000, 3);
            robot.LaserTrackingSearchStop();
            robot.MoveToLaserSeamPos(1, 30, 0, 0, 0, offdese);

            robot.LaserTrackingTrackOnOff(1, 3);

            JointPos endjointPos = new JointPos(68.809, -87.100, 121.120, -127.233, -95.038, -109.555);
            DescPose enddescPose = new DescPose(-103.555, -464.234, 13.076, 174.179, -1.344, -91.709);
            robot.MoveL(endjointPos, enddescPose, 1, 0, 20, 100, 100, -1, exaxisPos, 0, 0, offdese, 0);
            robot.LaserTrackingTrackOnOff(0, 3);
            Console.WriteLine($"Number of completions : {i + 1} ");
        }
    }

Extended Axis and Robot Synchronized Laser Tracking Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.8  Web-3.8.6

.. code-block:: c#
    :linenos:


    public void TestLaserTrackAndExitAxis()
    {   
        ExaxisPos startexaxisPos = new ExaxisPos(0, 0, 0, 0);
        ExaxisPos seamexaxisPos = new ExaxisPos(-10, 0, 0, 0);
        ExaxisPos endexaxisPos = new ExaxisPos(-30, 0, 0, 0);      
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);     
        JointPos startjointPos = new JointPos(58.337, -119.628, 146.037, -116.358, -92.224, -117.654);
        DescPose startdescPose = new DescPose(-53.375, -255.363, 0.919, 178.054, 1.077, -94.026);
        for (int i=0;i<10;++i)
        {
            robot.ExtAxisSyncMoveJ(startjointPos, startdescPose, 1, 0, 100, 100, 100, startexaxisPos, -1, 0, offdese);
            Console.WriteLine("11111");
            int ret = robot.LaserTrackingSearchStart_xyz(3, 100, 300, 1000, 2);
            robot.LaserTrackingSearchStop();
            Console.WriteLine("2222");
            int tool = 0;
            int user = 0;
            JointPos seamjointPos = new JointPos();
            DescPose seamdescPose = new DescPose();
            robot.GetLaserSeamPos(0, offdese, ref seamjointPos, ref seamdescPose, ref tool, ref user, ref startexaxisPos);
            Console.WriteLine($"{seamjointPos.jPos[0]}, {seamjointPos.jPos[1]}, {seamjointPos.jPos[2]}, " +
                            $"{seamjointPos.jPos[3]}, {seamjointPos.jPos[4]}, {seamjointPos.jPos[5]}, " +
                            $"{seamdescPose.tran.x}, {seamdescPose.tran.y}, {seamdescPose.tran.z}, " +
                            $"{seamdescPose.rpy.rx}, {seamdescPose.rpy.ry}, {seamdescPose.rpy.rz}");
            if (ret == 0)
            {
                robot.ExtAxisSyncMoveJ(seamjointPos, seamdescPose, 1, 0, 100, 100, 100, seamexaxisPos, -1, 0, offdese);
                Console.WriteLine("3333");
                robot.LaserTrackingTrackOnOff(1, 2);
                JointPos endjointPos = new JointPos(70.580, -90.918, 126.593, -125.154, -92.162, -105.403);
                DescPose enddescPose = new DescPose(-53.375, -419.020, 0.920, 178.054, 1.076, -94.026);
                robot.ExtAxisSyncMoveL(endjointPos, enddescPose, 1, 0, 20, 100, 100, -1, endexaxisPos, 0, offdese);
                robot.LaserTrackingTrackOnOff(0, 2);
            }
            Console.WriteLine($"Number of completions : {i + 1} ");
        }     
    }

