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

Conveyor Belt In-Place Tracking Parameter Configuration
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Configure conveyor belt in-place tracking parameters
    * @param  [in] trackMode 0-time; 1-distance; 2-time and distance, either condition satisfied
    * @param  [in] trackTime Tracking time, unit s
    * @param  [in] trackDis Tracking distance
    * @return  Error code
    */
    public int SetStationaryTrackPara(int trackMode, double trackTime, int trackDis)
    
Wait for In-Place Idle Motion to Complete
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Wait for in-place idle motion to complete
    * @return Error code
    */
    public int WaitStationaryMotionDone()
        
Conveyor Belt In-Place Tracking Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    public int TestStationaryTrack()
    {
        Console.WriteLine("\n========== Stationary Track Test ==========");

        int rtn;

        JointPos j1 = new JointPos(-35.146, -102.684, 120.805, -100.401, -90.295, 150.105);
        DescPose d1 = new DescPose(-121.814, -348.341, 209.978, -173.152, -3.585, -5.446);

        ExaxisPos ex = new ExaxisPos(0, 0, 0, 0);
        DescPose zeroOff = new DescPose(0, 0, 0, 0, 0, 0);

        int tool = 1;
        int workpiece = 1;

        rtn = robot.ConveyorSetParam(0, 10000, 200, 0, 0, 10);

        robot.MoveJ(j1, d1, tool, workpiece, 100, 100, 100, ex, -1, 0, zeroOff);

        // Step 1: SetDO control signal ON
        Console.WriteLine("--- Step 1: SetDO(6,1) ---");
        rtn = robot.SetDO(6, 1, 0, 0);
        Console.WriteLine("  SetDO(6,1) rtn={0}", rtn);

        // Step 2: Conveyor tracking start
        Console.WriteLine("--- Step 2: ConveyorTrackStart(2) ---");
        rtn = robot.ConveyorTrackStart(2);
        Console.WriteLine("  ConveyorTrackStart(2) rtn={0}", rtn);

        // Step 3: Workpiece IO detect
        Console.WriteLine("--- Step 3: ConveyorIODetect(10000) ---");
        rtn = robot.ConveyorIODetect(10000);
        Console.WriteLine("  ConveyorIODetect(10000) rtn={0}", rtn);

        // Step 4: Get track data
        Console.WriteLine("--- Step 4: ConveyorGetTrackData(2) ---");
        rtn = robot.ConveyorGetTrackData(2);
        Console.WriteLine("  ConveyorGetTrackData(2) rtn={0}", rtn);

        // Step 5: Set stationary track parameters (time mode, 200s, distance 5)
        Console.WriteLine("--- Step 5: SetStationaryTrackPara(0,200,5) ---");
        rtn = robot.SetStationaryTrackPara(0, 5, 5);
        Console.WriteLine("  SetStationaryTrackPara(0,200,5) rtn={0}", rtn);

        // Step 6: Execute stationary motion
        Console.WriteLine("--- Step 6: MoveStationary() ---");
        rtn = robot.MoveStationary();
        Console.WriteLine("  MoveStationary() rtn={0}", rtn);

        // Step 7: Wait for stationary motion done
        Console.WriteLine("--- Step 7: WaitStationaryMotionDone() ---");
        rtn = robot.WaitStationaryMotionDone();
        Console.WriteLine("  WaitStationaryMotionDone() rtn={0}", rtn);

        // Step 8: Conveyor tracking end
        Console.WriteLine("--- Step 8: ConveyorTrackEnd() ---");
        rtn = robot.ConveyorTrackEnd();
        Console.WriteLine("  ConveyorTrackEnd() rtn={0}", rtn);

        // Step 9: SetDO control signal OFF
        Console.WriteLine("--- Step 9: SetDO(6,0) ---");
        rtn = robot.SetDO(6, 0, 0, 0);
        Console.WriteLine("  SetDO(6,0) rtn={0}", rtn);

        Console.WriteLine("\n========== Stationary Track Test Complete ==========");
        return 0;
    }

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
    public int ConveyorComDetect(int timeout)

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
        // Conveyor belt tracking
        DescPose pos1 = new DescPose(-354.549, 63.914, 270.176, -179.679, -0.134, 2.468);
        DescPose pos2 = new DescPose(-351.203, -213.393, 351.054, -179.932, -0.508, 2.472);

        double[] cmp = { 0.0, 0.0, 0.0 };
        int rtn = robot.ConveyorCatchPointComp(cmp); // Set conveyor pick-up point compensation
        if (rtn != 0)
        {
            return;
        }
        Console.WriteLine("ConveyorCatchPointComp: rtn  " + rtn);

        rtn = robot.MoveCart(pos1, 1, 0, (float)30.0, (float)180.0, (float)100.0, (float)-1.0, -1);
        Console.WriteLine("MoveCart: rtn  " + rtn);

        rtn = robot.ConveyorIODetect(10000); // Conveyor workpiece I/O detection
        Console.WriteLine("ConveyorIODetect: rtn   " + rtn);

        robot.ConveyorGetTrackData(1); // Configure conveyor tracking for picking
        rtn = robot.ConveyorTrackStart(1); // Start tracking
        Console.WriteLine("ConveyorTrackStart: rtn  " + rtn);

        rtn = robot.ConveyorTrackMoveL("cvrCatchPoint", 1, 0, (float)100.0, (float)0.0, (float)100.0, (float)-1.0, 0, 0);
        Console.WriteLine("ConveyorTrackMoveL: rtn  " + rtn);

        rtn = robot.MoveGripper(2, 30, 60, 30, 30000, 0, 0, 0, 50, 50);
        Console.WriteLine("ConveyorTrackMoveL: rtn  " + rtn);
            

        rtn = robot.ConveyorTrackMoveL("cvrRaisePoint", 1, 0, (float)100.0, (float)0.0, (float)100.0, (float)-1.0, 0, 0);
        Console.WriteLine("ConveyorTrackMoveL: rtn   " + rtn);

        rtn = robot.ConveyorTrackEnd(); // Stop conveyor tracking
        Console.WriteLine("ConveyorTrackEnd: rtn  " + rtn);

        rtn = robot.MoveCart(pos2, 1, 0, (float)30.0, (float)180.0, (float)100.0, (float)-1.0, -1);
        Console.WriteLine("MoveCart: rtn  " + rtn);

        rtn = robot.MoveGripper(2, 100, 60, 30, 30000, 0,0,0,50,50);
        Console.WriteLine("MoveGripper: rtn  " + rtn);

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

Set the enabled device types for the end-effector LUA
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the enabled device types for the end-effector LUA
    * @param [in] forceSensorEnable Force sensor enable status, 0-disabled; 1-enabled
    * @param [in] gripperEnable Gripper enable status, 0-disabled; 1-enabled
    * @param [in] IOEnable IO device enable status, 0-disabled; 1-enabled
    * @param [in] dexhandEnable Dexterous hand enable status, 0-disabled; 1-enabled
    * @return  Error code
    */
    public int SetAxleLuaEnableDeviceType(int forceSensorEnable, int gripperEnable, int IOEnable, int dexhandEnable)

Get the enabled device types for the end-effector LUA
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the enabled device types for the end-effector LUA
    * @param [out] forceSensorEnable Force sensor enable status, 0-disabled; 1-enabled
    * @param [out] gripperEnable Gripper enable status, 0-disabled; 1-enabled
    * @param [out] IOEnable IO device enable status, 0-disabled; 1-enabled
    * @param [out] dexhandEnable Dexterous hand enable status, 0-disabled; 1-enabled
    * @return  Error code
    */
    public int GetAxleLuaEnableDeviceType(ref int forceSensorEnable, ref int gripperEnable, ref int IOEnable, ref int dexhandEnable)

Get the currently configured end-effector devices
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the currently configured end-effector devices
    * @param [out] forceSensorEnable Force sensor enabled device number, 0-disabled; 1-enabled
    * @param [out] gripperEnable Gripper enabled device number, 0-disabled; 1-enabled
    * @param [out] IODeviceEnable IO device enabled device number, 0-disabled; 1-enabled
    * @param [out] decHandEnable Dexterous hand enabled device number, 0-disabled; 1-enabled
    * @return  Error code
    */
    public int GetAxleLuaEnableDevice(ref int[] forceSensorEnable, ref int[] gripperEnable, ref int[] IODeviceEnable, ref int[] decHandEnable)

Set the enabled gripper action control functions
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Set the enabled gripper action control functions
    * @param [in] id Gripper device number
    * @param [in] func func[0]-gripper enable; func[1]-gripper initialization; func[2]-position setting; func[3]-speed setting; func[4]-torque setting; func[6]-read gripper status;
        func[7]-read initialization status; func[8]-read fault code; func[9]-read position; func[10]-read speed; func[11]-read torque; func[12]-set rotation count for rotary gripper;
        func[13]-set rotation speed for rotary gripper; func[14]-set rotation torque for rotary gripper; func[15]-read rotary gripper status; func[16]-read rotary gripper initialization status;
        func[17]-read rotary gripper rotation count; func[18]-read rotary gripper speed; func[19]-read rotary gripper torque; func[20]-multi-axis synchronous motion setting; func[21]-fault clear command;
        func[22]-single-axis running status; func[23]-all-axis running status;
    * @return  Error code
    */
    public int SetAxleLuaGripperFunc(int id, int[] func)

Get the enabled gripper action control functions
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the enabled gripper action control functions
    * @param [in] id Gripper device number
    * @param [out] func func[0]-gripper enable; func[1]-gripper initialization; func[2]-position setting; func[3]-speed setting; func[4]-torque setting; func[6]-read gripper status;
        func[7]-read initialization status; func[8]-read fault code; func[9]-read position; func[10]-read speed; func[11]-read torque; func[12]-set rotation count for rotary gripper;
        func[13]-set rotation speed for rotary gripper; func[14]-set rotation torque for rotary gripper; func[15]-read rotary gripper status; func[16]-read rotary gripper initialization status;
        func[17]-read rotary gripper rotation count; func[18]-read rotary gripper speed; func[19]-read rotary gripper torque; func[20]-multi-axis synchronous motion setting; func[21]-fault clear command;
        func[22]-single-axis running status; func[23]-all-axis running status;
    * @return  Error code
    */
    public int GetAxleLuaGripperFunc(int id, ref int[] func)

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
        robot.AxleLuaUpload("D://zUP/AXLE_LUA_End_JunDuo_V0.4_20260602.lua");

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
        robot.SetAxleLuaEnableDeviceType(0, 1, 0, 0);

        int forceEnable = 0;
        int gripperEnable = 0;
        int ioEnable = 0;
        int dexhandEnable = 0;
        robot.GetAxleLuaEnableDeviceType(ref forceEnable, ref gripperEnable, ref ioEnable, ref dexhandEnable);
        Console.WriteLine("GetAxleLuaEnableDeviceType param is {0} {1} {2}", forceEnable, gripperEnable, ioEnable);

        int[] func = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        robot.SetAxleLuaGripperFunc(1, func);

        int[] getFunc = new int[32];
        robot.GetAxleLuaGripperFunc(1, ref getFunc);
        int[] getforceEnable = new int[16];
        int[] getgripperEnable = new int[16];
        int[] getioEnable = new int[16];
        int[] dexhandEnable1 = new int[16];
        robot.GetAxleLuaEnableDevice(ref getforceEnable, ref getgripperEnable, ref getioEnable,ref dexhandEnable1);
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
        Thread.Sleep(3000);
        robot.ActGripper(1, 1);
        Thread.Sleep(4000);
        robot.MoveGripper(1, 50, 10, 100, 50000, 0, 0, 0, 0, 0);
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
    * @brief  Write slave station AO
    * @param [in] AOIndex AO number
    * @param [in] writeNum Number of values to write
    * @param [in] status Array of values to write (maximum 8), AO0~AO15 are integer type, AO16~AO31 are floating point type
    * @return Error code
    */
    public int FieldBusSlaveWriteAO(int AOIndex, int writeNum, double[] status)

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
        double[] ctrlAO = new double[8];
        int[] DI = new int[8];
        double[] AI = new double[8];
        if (rtn != 0)
        {
            return;
        }
        // Upload and load open protocol file
        robot.OpenLuaUpload("E://temp/CtrlDev_field.lua");
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

End-Effector Transparent Transmission Function Enable/Disable
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Enable end-effector general transparent transmission function
    * @param [in] enable, 0-disable, 1-enable
    * @return Error code
    */
    public int SetAxleGenComEnable(int mode)

End-Effector Transparent Transmission Function Non-Periodic Data Transmission and Reception
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief End-effector sends non-periodic data and waits for response
    * @param [in] len_snd, length of data to send
    * @param [in] sndBuff[], data to send
    * @param [in] len_rcv, length of data to receive
    * @param [out] rcvBuff[], response data
    * @return Error code
    */
    public int SndRcvAxleGenComCmdData(int len_snd, int[] sndBuff, int len_rcv, ref int[] rcvdata)

Code Example for Non-Periodic Data Communication of DIO Health Care Moxibustion Head Based on End-Effector Transparent Transmission Function
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    void testAxleGenCom()
    {
        int[] led_on = new int[6] { 0xAB, 0xBA, 0x12, 0x01, 0x01, 0x79 };
        int[] led_off = new int[6] { 0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78 };
        int[] version = new int[5]{ 0xAB, 0xBA, 0x11, 0x00, 0x76 };
        int[] state = new int[6] { 0xAB, 0xBA, 0x1B,0x01, 0xAA, 0x2B };
        int[] cycleState = new int[6] { 0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78 };

        int[] rcvdata = new int[16];
        int ret = 0;
        int cnt = 1;

        JointPos p1Joint = new JointPos(88.708, -86.178, 140.989, -141.825, -89.162, -49.879);
        DescPose p1Desc = new DescPose(188.007, -377.850, 260.207, 178.715, 2.823, -131.466);

        JointPos p2Joint = new JointPos(112.131, -75.554, 126.989, -139.027, -88.044, -26.477);
        DescPose p2Desc = new DescPose(368.003, -377.848, 260.211, 178.715, 2.823, -131.465);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

        //Enable end-effector transparent transmission function
        robot.SetAxleGenComEnable(1);
        robot.SetAxleLuaEnable(1);

        while(cnt<=10)
        { 
            //Read version number
            ret = robot.SndRcvAxleGenComCmdData(5, version, 10, ref rcvdata);
            Console.WriteLine($" hard version : {rcvdata[4]},hard code:{rcvdata[5]}, soft version:{rcvdata[6]} {rcvdata[7]}, soft code:{rcvdata[8]}");
            if (ret != 0)
            {
                break;
            }
            Thread.Sleep(1000);
            //Read moxibustion head presence status
            ret = robot.SndRcvAxleGenComCmdData(6, state, 6, ref rcvdata);
            Console.WriteLine($" state : {rcvdata[4]}");
            Thread.Sleep(1000);
            //Turn on moxibustion head laser
            ret = robot.SndRcvAxleGenComCmdData(6, led_on, 6, ref rcvdata);
            Console.WriteLine($"led on rcv data is: {rcvdata[0]},{rcvdata[1]}, {rcvdata[2]}, {rcvdata[3]}, {rcvdata[4]}, {rcvdata[5]}");
            robot.MoveJ(p1Joint, p1Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
            Thread.Sleep(4000);
            //Turn off moxibustion head laser
            ret = robot.SndRcvAxleGenComCmdData(6, led_off, 6, ref rcvdata);
            Console.WriteLine($"led off rcv data is: {rcvdata[0]},{rcvdata[1]}, {rcvdata[2]}, {rcvdata[3]}, {rcvdata[4]}, {rcvdata[5]}");
            robot.MoveJ(p2Joint, p2Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
            Thread.Sleep(1000);
            Console.WriteLine($"***********************complate No. {cnt}  SDK test*****************************");
            cnt++;
        }

    }

Download Open Protocol Lua File
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c#
    :linenos:

    /**
    * @brief Download open protocol Lua file
    * @param [in] fileName Open protocol file name "CtrlDev_XXX.lua"
    * @param [in] savePath Path to save the open protocol file
    * @return Error code
    */
    public int OpenLuaDownload(string fileName, string savePath)
    
Delete Open Protocol Lua File
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Delete open protocol Lua file
    * @param [in] fileName Name of the open protocol Lua file to delete "CtrlDev_XXX.lua"
    * @return Error code
    */
    public int OpenLuaDelete(string fileName)
        
Delete All Open Protocol Lua Files
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Delete all open protocol Lua files
    * @return Error code
    */
    public int AllOpenLuaDelete()

SDK Code Example for Open Protocol Lua File Operations
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:

    public int TestCtrlOpenLuaOperate()
    {
        int rtn;

        // Upload Lua file to robot
        rtn = robot.OpenLuaUpload("D://zUP/openlua/CtrlDev_WELDING_A.lua");
        Console.WriteLine($"OpenLuaUpload rtn is {rtn}");
        rtn = robot.OpenLuaUpload("D://zUP/openlua/CtrlDev_SWDPOLISH.lua");
        Console.WriteLine($"OpenLuaUpload rtn is {rtn}");

        // Download Lua file from robot
        rtn = robot.OpenLuaDownload("CtrlDev_WELDING_A.lua", "D://zDOWN/");
        Console.WriteLine($"OpenLuaDownload rtn is {rtn}");
        rtn = robot.OpenLuaDownload("CtrlDev_SWDPOLISH.lua", "D://zDOWN/");
        Console.WriteLine($"OpenLuaDownload rtn is {rtn}");

        // Set control open protocol Lua name
        rtn = robot.SetCtrlOpenLUAName(0, "CtrlDev_WELDING_A.lua");
        Console.WriteLine($"SetCtrlOpenLUAName rtn is {rtn}");
        rtn = robot.SetCtrlOpenLUAName(1, "CtrlDev_SWDPOLISH.lua");
        Console.WriteLine($"SetCtrlOpenLUAName rtn is {rtn}");

        // Get control open protocol Lua name
        string[] name = new string[4];
        rtn = robot.GetCtrlOpenLUAName(ref name);
        Console.WriteLine($"ctrl open lua names : {name[0]}, {name[1]}, {name[2]}, {name[3]}");

        // Load and unload open protocol Lua
        rtn = robot.LoadCtrlOpenLUA(1);
        Console.WriteLine($"LoadCtrlOpenLUA rtn is {rtn}");
        robot.Sleep(2000);
        rtn = robot.UnloadCtrlOpenLUA(1);
        Console.WriteLine($"UnloadCtrlOpenLUA rtn is {rtn}");

        // Delete specified Lua file and all Lua files
        rtn = robot.OpenLuaDelete("CtrlDev_WELDING_A.lua");
        Console.WriteLine($"OpenLuaDelete rtn is {rtn}");
        rtn = robot.AllOpenLuaDelete();
        Console.WriteLine($"AllOpenLuaDelete rtn is {rtn}");

        return 0;
    }

Control Dexterous Hand Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:    

    /**
    * @brief  Control dexterous hand motion
    * @param  [in] idstart  Starting slave station number
    * @param  [in] slaveNum  Number of slaves
    * @param  [in] pos[16]  Position (-360~360) 
    * @param  [in] speed[16]  Speed percentage, range [0~100]
    * @param  [in] force[16]  Torque percentage, range [0~100]
    * @param  [in] max_time  Maximum wait time, range [0~30000], unit ms
    * @return  Error code
    */
    public int SetDexterousHandsMove(int idstart, int slaveNum, double[] pos, int[] speed, int[] force, int max_time)
    
Control Dexterous Hand Reset and Activation
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:   

    /**
    * @brief  Control dexterous hand reset and activation
    * @param  [in] id  Slave station number
    * @param  [in] act  0-reset 1-activate
    * @return  Error code
    */
    public int SetDexterousHandsAct(int id, int act)

Clear Dexterous Hand Error
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:   

    /**
    * @brief  Clear dexterous hand error
    * @return  Error code
    */
    public int ClearDexterousHandsError()
    
Set Enabled Dexterous Hand Action Control Functions
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:   

    /**
    * @brief Set enabled dexterous hand action control functions
    * @param [in] id Dexterous hand slave station number
    * @param [in] func 0-grip trigger, 1-gripper initialization, 2-position setting, 3-speed setting, 4-torque setting, 6-read gripper status, 7-read initialization status, 8-read fault code, 9-read position, 10-read speed, 11-read torque, 12-rotation count setting, 13-rotation speed setting, 14-rotation torque setting, 15-read rotary gripper status, 16-read rotary initialization status, 17-read rotation count, 18-read rotation speed, 19-read rotation torque, 20-multi-axis synchronous motion setting, 21-fault clear command, 22-single-axis running status, 23-all-axis running status
    * @return  Error code
    */
    public int SetDexterousHandsFunc(int id, int[] func)
    
Get Enabled Dexterous Hand Action Control Functions
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:   

    /**
    * @brief Get enabled dexterous hand action control functions
    * @param [in] id Dexterous hand device number
    * @param [out] func 0-grip trigger, 1-gripper initialization, 2-position setting, 3-speed setting, 4-torque setting, 6-read gripper status, 7-read initialization status, 8-read fault code, 9-read position, 10-read speed, 11-read torque, 12-rotation count setting, 13-rotation speed setting, 14-rotation torque setting, 15-read rotary gripper status, 16-read rotary initialization status, 17-read rotation count, 18-read rotation speed, 19-read rotation torque, 20-multi-axis synchronous motion setting, 21-fault clear command, 22-single-axis running status, 23-all-axis running status
    * @return  Error code
    */
    public int GetDexterousHandsFunc(int id, ref int[] func)

End-Effector Dexterous Hand Configuration and Motion Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
.. code-block:: c#
    :linenos:
    
    private void button105_Click(object sender, EventArgs e)
    {
        int id = 1;               // Slave station number
        int slaveNum = 4;         // Control 4 fingers
        int max_time = 8000;      // Maximum wait time 8 seconds
        int[] speed = new int[16]; // Speed array, all 0 means use default speed
        int[] force = new int[16]; // Torque array

        // Initialize torque array: first 4 fingers set to 50%, the rest 0 (values sent via Move command)
        for (int i = 0; i < 16; i++)
            force[i] = (i < 4) ? 50 : 0;

        // Helper function: set position array (only first 4 fingers are effective)
        double[] pos = new double[16];
        void SetPositions(double v1, double v2, double v3, double v4)
        {
            for (int i = 0; i < 16; i++)
                pos[i] = 0;
            pos[0] = v1;
            pos[1] = v2;
            pos[2] = v3;
            pos[3] = v4;
        }

        JointPos j1 = new JointPos(-91.876, -85.920, 109.279, -86.239, -96.664, -28.563);
        JointPos j2 = new JointPos(-40.954, -85.920, 109.279, -86.239, -96.664, -28.563);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);

        Console.WriteLine("===== Dexterous Hand Full Function Test Started =====");

        // 1. Clear error
        int ret = robot.ClearDexterousHandsError();
        Console.WriteLine($"ClearDexterousHandsError -> {ret}");

        // ========== 2. Set function switches ==========
        int[] setFunc = new int[32];
        setFunc[2] = 1;   // Enable position setting function
        setFunc[4] = 1;   // Enable torque setting function
        setFunc[9] = 1;   // Read position
        setFunc[10] = 1;  // Read torque
        setFunc[11] = 1;  // Read status
        setFunc[22] = 1;  // Single-axis motion status

        ret = robot.SetDexterousHandsFunc(id, setFunc);
        Console.WriteLine($"SetDexterousHandsFunc(enable init + position/torque functions) -> {ret}");

        // ========== 3. Read function status (verify settings took effect) ==========
        int[] getFunc = new int[32];  // GetDexterousHandsFunc returns 32 integers
        ret = robot.GetDexterousHandsFunc(id, ref getFunc);
        Console.WriteLine($"GetDexterousHandsFunc -> {ret}");
        if (ret == 0)
        {
            // Print all 32 values
            Console.WriteLine("All 32 values returned by GetDexterousHandsFunc:");
            for (int i = 0; i < getFunc.Length; i++)
            {
                Console.Write($"  [{i}]={getFunc[i]}");
                if ((i + 1) % 8 == 0)
                    Console.WriteLine();          // New line every 8 items
                else if (i < getFunc.Length - 1)
                    Console.Write(", ");
            }
            if (getFunc.Length % 8 != 0)
                Console.WriteLine();              // Add newline if last line has fewer than 8 items
        }

        // ========== 4. Activate dexterous hand ==========
        ret = robot.SetDexterousHandsAct(id, 1);
        Console.WriteLine($"SetDexterousHandsAct(activate) -> {ret}");
        if (ret != 0)
        {
            Console.WriteLine("Activation failed, test aborted");
            return;
        }

        // ========== 5. Initial move to 20° (send position and torque values via Move command) ==========
        SetPositions(20, 20, 20, 20);
        ret = robot.SetDexterousHandsMove(id, slaveNum, pos, speed, force, max_time);
        Console.WriteLine($"Initial move to 20° -> {ret}");
        robot.Sleep(5000);

        // ========== 6. Reciprocating motion 10 times (10° ↔ 50°) ==========
        Console.WriteLine("Starting 10 reciprocating motions...");
        for (int iteration = 1; iteration <= 10; iteration++)
        {
            robot.MoveJ(j1, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);

            SetPositions(10, 10, 10, 10);
            ret = robot.SetDexterousHandsMove(id, slaveNum, pos, speed, force, max_time);
            Console.WriteLine($"[{iteration}] Move to 10° -> {ret}");
            robot.Sleep(1000);

            robot.MoveJ(j2, 0, 0, 100, 100, 100, epos, -1, 0, offset_pos);

            SetPositions(50, 50, 50, 50);
            ret = robot.SetDexterousHandsMove(id, slaveNum, pos, speed, force, max_time);
            Console.WriteLine($"[{iteration}] Move to 50° -> {ret}");
            robot.Sleep(1000);
        }

        Console.WriteLine("Test completed (function switch set/read + activation + 10 reciprocating motions).");
    }