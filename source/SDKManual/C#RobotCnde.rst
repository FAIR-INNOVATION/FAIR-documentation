CNDE
=================

.. toctree:: 
    :maxdepth: 5

Configure Robot CNDE Data List and Update Period
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Configure the data list and update period for robot real-time status feedback (overwrites previous configuration)
    * @param [in] states List of status enums to subscribe to, the order determines the arrangement order in the data packet
    * @param [in] period Data update period, unit milliseconds, value range [8, 1000]
    * @return Returns 0 on success; returns a negative error code on failure (e.g., ERR_STATE_INVALID, ERR_PARAM_VALUE, etc.)
    */
    public int SetRobotRealtimeStateConfig(List<RobotState> states, int period)

Add a Status Item to the Existing Status Feedback List
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Add a status item to the existing status feedback list
    * @param [in] state Status enum value to add
    * @return Returns 0 on success; returns a negative error code on failure (e.g., ERR_STATE_ALREADY_EXISTS, ERR_STATE_INVALID, etc.)
    */
    public int AddRobotRealtimeState(RobotState state)
    
Delete a Status Item from the Existing Status Feedback List
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Delete a status item from the existing status feedback list (at least one status must remain)
    * @param [in] state Status enum value to delete
    * @return Returns 0 on success; returns a negative error code on failure (e.g., ERR_STATE_INVALID, ERR_NEED_AT_LEAST_ONE_STATE)
    */
    public int DeleteRobotRealtimeState(RobotState state)
        
Modify Only the Update Period of Status Feedback
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

     /**
    * @brief Modify only the update period of status feedback without changing the status list
    * @param [in] period New update period, unit milliseconds, value range [8, 1000]
    * @return Returns 0 on success; returns a negative error code on failure (e.g., ERR_PARAM_VALUE)
    */
    public int SetRobotRealtimeStatePeriod(int period)
        
Get the Currently Configured Status Feedback List and Update Period
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get the currently configured status feedback list and update period
    * @param [out] states Output the currently subscribed status enum list
    * @param [out] period Output the current data update period, unit milliseconds
    * @return Returns 0 on success; returns a negative error code on failure
    */
    public int GetRobotRealtimeStateConfig(out List<RobotState> states, out int period)

CNDE Configuration Related SDK Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private async void TestRobotRealtimeStates()
    {
        // 1. Define the status fields to subscribe to
        List<RobotState> requiredStates = new List<RobotState>
        {
            RobotState.JointCurPos,
            RobotState.ToolCurPos, 
            RobotState.JointDriverTemperature,
            RobotState.RobotTime,
        };

        // 2. Configure status feedback (period 8ms)
        int periodMs = 8;
        int ret = robot.SetRobotRealtimeStateConfig(requiredStates, periodMs);
        if (ret != 0)
        {
            Console.WriteLine($"Status configuration failed, error code: {ret}");
            return;
        }
        Console.WriteLine($"Status configuration successful, {requiredStates.Count} fields, period {periodMs} ms");

        // Verify if the configuration took effect
        List<RobotState> actualStates;
        int actualPeriod;
        robot.GetRobotRealtimeStateConfig(out actualStates, out actualPeriod);
        Console.WriteLine($"Actual active status count: {actualStates.Count}, period: {actualPeriod} ms");
        Thread.Sleep(3000);
        // 3. Establish RPC connection (automatically completes CNDE handshake internally)
        robot.SetReconnectParam(true, 10, 1000);
        ret = robot.RPC("192.168.58.2");  // Modify according to the actual robot IP
        if (ret != 0)
        {
            Console.WriteLine($"RPC connection failed, error code: {ret}");
            return;
        }
        // 4. Loop to read and print status data
        DateTime startTime = DateTime.Now;
        const int durationSeconds = 500;

        while ((DateTime.Now - startTime).TotalSeconds < durationSeconds)
        {
            ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
            ret = robot.GetRobotRealTimeState(ref pkg);
            Console.WriteLine($"GetRobotRealTimeState: {ret}");

            //Joint positions (degrees)
            if (pkg.jt_cur_pos != null && pkg.jt_cur_pos.Length >= 6)
                Console.WriteLine($"Joint positions(°): J1={pkg.jt_cur_pos[0]:F2}, J2={pkg.jt_cur_pos[1]:F2}, J3={pkg.jt_cur_pos[2]:F2}, J4={pkg.jt_cur_pos[3]:F2}, J5={pkg.jt_cur_pos[4]:F2}, J6={pkg.jt_cur_pos[5]:F2}");

            //TCP pose (mm /°)
            if (pkg.tl_cur_pos != null && pkg.tl_cur_pos.Length >= 6)
                Console.WriteLine($"TCP pose(mm/°): X={pkg.tl_cur_pos[0]:F2}, Y={pkg.tl_cur_pos[1]:F2}, Z={pkg.tl_cur_pos[2]:F2}, RX={pkg.tl_cur_pos[3]:F2}, RY={pkg.tl_cur_pos[4]:F2}, RZ={pkg.tl_cur_pos[5]:F2}");
    
            // Joint temperatures
            if (pkg.jointDriverTemperature != null && pkg.jointDriverTemperature.Length >= 6)
                Console.WriteLine($"Joint temperatures(°C): J1={pkg.jointDriverTemperature[0]:F2}, J2={pkg.jointDriverTemperature[1]:F2}, J3={pkg.jointDriverTemperature[2]:F2}, J4={pkg.jointDriverTemperature[3]:F2}, J5={pkg.jointDriverTemperature[4]:F2}, J6={pkg.jointDriverTemperature[5]:F2}");

            // Robot time
            Console.WriteLine($"Robot time: {pkg.robotTime.year}-{pkg.robotTime.mouth:D2}-{pkg.robotTime.day:D2} {pkg.robotTime.hour:D2}:{pkg.robotTime.minute:D2}:{pkg.robotTime.second:D2}.{pkg.robotTime.millisecond:D3}");

            await Task.Delay(100);
        }

        // 5. Disconnect
        robot.CloseRPC();
    }

CNDE Add/Delete Configuration Status and Set Communication Period SDK Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    private async void TestAddDeleteCNDE()
    {
        List<RobotState> finalStates;
        int finalPeriod;
        // Initial configuration: request no status (default configuration)
        List<RobotState> emptyStates = new List<RobotState>();
        int ret = robot.SetRobotRealtimeStateConfig(emptyStates, 20);

        robot.SetRobotRealtimeStatePeriod(10);
        // Delete two statuses
        ret = robot.DeleteRobotRealtimeState(RobotState.JointCurPos);
        Console.WriteLine($"Delete JointCurPos result: {ret}");
        ret = robot.DeleteRobotRealtimeState(RobotState.ToolCurPos);
        Console.WriteLine($"Delete ToolCurPos result: {ret}");
        // Add one status
        ret = robot.AddRobotRealtimeState(RobotState.CollisionLevel);
        Console.WriteLine($"Add CollisionLevel result: {ret}");

        // Get the current configuration list and resend
        List<RobotState> currentStates;
        int currentPeriod;
        robot.GetRobotRealtimeStateConfig(out currentStates, out currentPeriod);
        Console.WriteLine($"Current configuration status count: {currentStates.Count}");
        ret = robot.SetRobotRealtimeStateConfig(currentStates, currentPeriod);
        Console.WriteLine($"Apply new configuration result: {ret}"); Console.WriteLine($"Initial configuration result: {ret}");
        robot.GetRobotRealtimeStateConfig(out finalStates, out finalPeriod);
        Console.WriteLine($"Configuration status count: {finalStates.Count}");
        foreach (var s in finalStates) Console.WriteLine($"  {s}");
        Console.WriteLine($"Period: {finalPeriod} ms");

        Thread.Sleep(1000);
        // Establish RPC connection (automatically connects to CNDE internally)
        robot.SetReconnectParam(true, 100, 1000);
        ret = robot.RPC("192.168.58.2");
        if (ret != 0)
        {
            Console.WriteLine($"RPC connection failed: {ret}");
            return;
        }

        // Loop to print deleted and added statuses, deleted statuses print as 0, added statuses can retrieve real-time values normally
        DateTime lastTime = DateTime.Now;
        int frameCount = 0;
        DateTime startTime = DateTime.Now;
        while ((DateTime.Now - startTime).TotalSeconds < 10)
        {
            ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
            robot.GetRobotRealTimeState(ref pkg);
            DateTime now = DateTime.Now;
            double interval = (now - lastTime).TotalMilliseconds;
            lastTime = now;
            frameCount++;

            if (pkg.jt_cur_pos != null && pkg.jt_cur_pos.Length >= 6)
            {
                Console.WriteLine($"  Joint positions(°): J1={pkg.jt_cur_pos[0]:F2}, J2={pkg.jt_cur_pos[1]:F2}, J3={pkg.jt_cur_pos[2]:F2}, J4={pkg.jt_cur_pos[3]:F2}, J5={pkg.jt_cur_pos[4]:F2}, J6={pkg.jt_cur_pos[5]:F2}");
            }
            if (pkg.tl_cur_pos != null && pkg.tl_cur_pos.Length >= 6)
            {
                Console.WriteLine($"  TCP pose(mm/°): X={pkg.tl_cur_pos[0]:F2}, Y={pkg.tl_cur_pos[1]:F2}, Z={pkg.tl_cur_pos[2]:F2}, RX={pkg.tl_cur_pos[3]:F2}, RY={pkg.tl_cur_pos[4]:F2}, RZ={pkg.tl_cur_pos[5]:F2}");
            }
            // Collision level
            if (pkg.collisionLevel != null && pkg.collisionLevel.Length >= 6)
                Console.WriteLine($"Collision level: J1={pkg.collisionLevel[0]}, J2={pkg.collisionLevel[1]}, J3={pkg.collisionLevel[2]}, J4={pkg.collisionLevel[3]}, J5={pkg.collisionLevel[4]}, J6={pkg.collisionLevel[5]}");

            await Task.Delay(50);
        }
        //Disconnect
        robot.CloseRPC();
        Console.WriteLine("Test completed.");
    }