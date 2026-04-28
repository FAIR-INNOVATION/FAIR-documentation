CNDE
=============

.. toctree:: 
    :maxdepth: 5

Configure Robot Status Feedback
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Configure robot status feedback
    * @param state List of robot status enums
    * @param period Status feedback period, range 8-1000
    * @return Error code, 0-normal, 4-parameter error, 18-status field does not exist, 20-total bytes exceed 4K
    */
    public int SetRobotRealtimeStateConfig(List<RobotState> state, int period)
    
Add a Robot Status to CNDE Status Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Add a robot status to the configuration list
    * @param state Robot status enum
    * @return Error code, 0-normal, 17-status already exists, 18-status field does not exist, 20-exceeds 4K
    */
    public int AddRobotRealtimeState(RobotState state)
    
Delete a Robot Status from CNDE Status Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Delete a robot status from the configuration list
    * @param state Robot status enum
    * @return Error code, 0-normal, 18-status does not exist, 19-at least one status must remain
    */
    public int DeleteRobotRealtimeState(RobotState state)
        
Set CNDE Status Feedback Period
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Set CNDE status feedback period
    * @param period Status feedback period, range 8-1000
    * @return Error code, 0-normal, 4-parameter error
    */
    public int SetRobotRealtimeStatePeriod(int period)
            
Get Current CNDE Status Feedback All State Set and Period
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
     * @brief Get all current state sets and period
     * @return Configuration result structure containing state list and period
    */
    public StateConfigResult GetRobotRealtimeStateConfig()
                
CNDE Status Feedback Usage Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Example of using CNDE real-time status configuration interface
    */
    public static void TestRealtimeStateConfig(Robot robot)
    {
        
        // 1. Create initial state list
        List<RobotState> stateList1 = new ArrayList<>();
        stateList1.add(RobotState.ProgramState);
        stateList1.add(RobotState.RobotState);
        stateList1.add(RobotState.JointCurPos);
        stateList1.add(RobotState.ToolCurPos);
        
        // 2. First call to SetRobotRealtimeStateConfig to configure states and period
        int period1 = 100;  // 100ms period
        int rtn = robot.SetRobotRealtimeStateConfig(stateList1, period1);
        System.out.printf("1. SetRobotRealtimeStateConfig (initial list, period=%d) rtn: %d%n", period1, rtn);
        
        if (rtn == 0) {
            // 3. Add additional status
            rtn = robot.AddRobotRealtimeState(RobotState.RobotTime);
            System.out.printf("2. AddRobotRealtimeState (RobotTime) rtn: %d%n", rtn);
            
            // 4. Call SetRobotRealtimeStateConfig again to reconfigure (different state list)
            List<RobotState> stateList2 = new ArrayList<>();
            stateList2.add(RobotState.ProgramState);
            stateList2.add(RobotState.RobotState);
            stateList2.add(RobotState.MainCode);
            stateList2.add(RobotState.SubCode);
            stateList2.add(RobotState.JointCurPos);
            stateList2.add(RobotState.ToolCurPos);
            stateList2.add(RobotState.ActualJointTorque);
            
            int period2 = 50;  // 50ms period
            rtn = robot.SetRobotRealtimeStateConfig(stateList2, period2);
            System.out.printf("3. SetRobotRealtimeStateConfig (updated list, period=%d) rtn: %d%n", period2, rtn);
            
            // 5. Modify period
            int newPeriod = 80;  // 80ms period
            rtn = robot.SetRobotRealtimeStatePeriod(newPeriod);
            System.out.printf("4. SetRobotRealtimeStatePeriod (period=%d) rtn: %d%n", newPeriod, rtn);
            
            // 6. Get current configuration and print
            Robot.StateConfigResult configResult = robot.GetRobotRealtimeStateConfig();
            System.out.println("5. GetRobotRealtimeStateConfig result:");
            System.out.printf("   - Period: %d ms%n", configResult.period);
            System.out.println("   - Configured States:");
            for (int i = 0; i < configResult.stateList.size(); i++) {
                System.out.printf("     [%d] %s%n", i, configResult.stateList.get(i));
            }
            
            rtn = robot.RPC("192.168.58.2");
            if (rtn == 0) {
                System.out.println("rpc connection success");
            } else {
                System.out.println("rpc connection fail");
                return;
            }
            // Wait for CNDE connection to establish
            System.out.println("Waiting for CNDE connection to establish...");
            while (robot.CNDEGetStateData() == null) {
                robot.Sleep(100);
            }
            System.out.println("CNDE connection established, starting to receive data...");

            // 7. Loop to read real-time status to verify configuration
            System.out.println("6. Reading real-time states...");
            while(true) {
                robot.Sleep(1000);
                // Get status data via CNDE
                ROBOT_STATE_PKG pkg = robot.CNDEGetStateData();
                if (pkg == null) {
                    System.out.println("Status data is null, CNDE connection disconnected, waiting for reconnection");
                    continue;  // Continue loop while disconnected, waiting for reconnection
                }
                System.out.println("\n--- Robot Time ---");
                if (pkg.robotTime != null) {
                    System.out.println("robotTime: " + pkg.robotTime.year + "-" + pkg.robotTime.month + "-" + pkg.robotTime.day +
                            " " + pkg.robotTime.hour + ":" + pkg.robotTime.minute + ":" + pkg.robotTime.second +
                            "." + pkg.robotTime.millisecond);
                }

                System.out.println("   --- Status Information ---");
                System.out.printf("   program_state: %d%n", pkg.program_state);
                System.out.printf("   robot_state: %d%n", pkg.robot_state);
                System.out.printf("   main_code: %d%n", pkg.main_code);
                System.out.printf("   sub_code: %d%n", pkg.sub_code);
                System.out.println("   --- Joint Positions (actual_joint_pos) ---");
                System.out.printf("   jt_cur_pos[0-2]: %.2f, %.2f, %.2f%n",
                    pkg.jt_cur_pos[0], pkg.jt_cur_pos[1], pkg.jt_cur_pos[2]);
                System.out.printf("   jt_cur_pos[3-5]: %.2f, %.2f, %.2f%n",
                    pkg.jt_cur_pos[3], pkg.jt_cur_pos[4], pkg.jt_cur_pos[5]);
                System.out.println("   --- TCP Positions (actual_TCP_pos) ---");
                System.out.printf("   tl_cur_pos[0-2]: %.2f, %.2f, %.2f%n",
                    pkg.tl_cur_pos[0], pkg.tl_cur_pos[1], pkg.tl_cur_pos[2]);
                System.out.printf("   tl_cur_pos[3-5]: %.2f, %.2f, %.2f%n",
                    pkg.tl_cur_pos[3], pkg.tl_cur_pos[4], pkg.tl_cur_pos[5]);
                System.out.println("   --- Joint Torques (actual_joint_torque) ---");
                System.out.printf("   jt_cur_tor[0-2]: %.2f, %.2f, %.2f%n",
                    pkg.jt_cur_tor[0], pkg.jt_cur_tor[1], pkg.jt_cur_tor[2]);
                System.out.printf("   jt_cur_tor[3-5]: %.2f, %.2f, %.2f%n",
                    pkg.jt_cur_tor[3], pkg.jt_cur_tor[4], pkg.jt_cur_tor[5]);
                robot.Sleep(500);
            }
        } else {
            System.out.printf("SetRobotRealtimeStateConfig failed with error: %d%n", rtn);
        }
    }