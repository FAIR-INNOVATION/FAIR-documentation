CNDE
=============

.. toctree:: 
    :maxdepth: 5

Configure Robot CNDE Status Feedback
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotRealtimeStateConfig(states: List[RobotState], period: int = 500) -> int:``"
    "Description", "Set CNDE default configuration (call before RPC connection)"
    "Required Parameters", "
    - ``states``: List of RobotState enums
    - ``period``: Data period (ms), range 8-1000, default 8ms
    "
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure-errcode"

Add Robot Status to CNDE Status Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``AddRobotRealtimeState(states: List[RobotState], ip: str = None) -> int:``"
    "Description", "Add CNDE status list based on existing configuration (supports dynamic maintenance and IP isolation)"
    "Required Parameters", "
    - ``states``: List of RobotState enums to add
    - ``ip``: Optional, specify robot IP (for multi-robot isolated configuration; if not provided, modifies global configuration)
    "
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure-errcode"

Delete Robot Status from CNDE Status Configuration
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``DeleteRobotRealtimeState(states: List[RobotState], ip: str = None) -> int:``"
    "Description", "Delete CNDE status list based on existing configuration (supports dynamic maintenance and IP isolation)"
    "Required Parameters", "
    - ``states``: List of RobotState enums to delete
    - ``ip``: Optional, specify robot IP (for multi-robot isolated configuration; if not provided, modifies global configuration)
    "
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure-errcode"

Set CNDE Status Feedback Period
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``SetRobotRealtimeStatePeriod(period: int, ip: str = None) -> int:``"
    "Description", "Set CNDE status feedback period (supports global or IP-isolated configuration)"
    "Required Parameters", "
    - ``period``: Data period (ms), range 8-1000
    - ``ip``: Optional, specify robot IP (if not provided, modifies global configuration)
    "
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure-errcode"

Get Current CNDE Status Feedback All State Set
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. csv-table:: 
    :stub-columns: 1
    :widths: 10 30

    "Prototype", "``CNDEGetConfig(self) -> tuple:``"
    "Description", "Get all current state sets"
    "Required Parameters", "None"
    "Default Parameters", "None"
    "Return Value", "- Error code Success-0 Failure-errcode Configuration result structure containing state list"

CNDE Status Feedback Usage Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python
    :linenos: 

    from fairino import Robot
    from fairino.Robot import RobotState, SetRobotRealtimeStateConfig, DEFAULT_CNDE_STATES, AddRobotRealtimeState, DeleteRobotRealtimeState, SetRobotRealtimeStatePeriod
    import time

    # ==================== Global Configuration Parameters ====================
    ROBOT_IP = '192.168.58.2'       # Robot IP address
    # ========== Test1: CNDE Configuration and Data Acquisition Test =============
    # Test steps:
    # 1. Set CNDE configuration (JointCurPos, ToolCurPos, 20ms period)
    # 2. Establish RPC connection
    # 3. Print robot joint and TCP pose data
    # 4. Get timestamp and verify period
    # 5. Modify configuration (RobotMode, RbtEnableState, 10ms period)
    # 6. Verify new configuration takes effect

    def test1_cnde_config_and_data():
        """Test1: CNDE configuration and data acquisition test - verify configuration settings and data real-time nature"""
        print_separator("Test1: CNDE configuration and data acquisition test")

        # ===== Step 1: Set CNDE configuration (JointCurPos, ToolCurPos, 20ms) =====
        print("\n[Step 1] Setting CNDE configuration...")
        print("  Configuration fields: JointCurPos, ToolCurPos")
        print("  Feedback period: 20ms")

        custom_states = [
            RobotState.JointCurPos,   # Current joint position
            RobotState.ToolCurPos,    # Current tool (TCP) position
        ]

        rtn = SetRobotRealtimeStateConfig(custom_states, 20)
        if rtn != 0:
            print(f"✗ Configuration setting failed, error code: {rtn}")
            return None
        print("✓ CNDE configuration set successfully")

        # ===== Step 2: Establish RPC connection =====
        print(f"\n[Step 2] Establishing RPC connection ({ROBOT_IP})...")
        robot = Robot.RPC(ROBOT_IP)
        time.sleep(0.5)  # Wait for connection and data reception

        # Verify configuration
        config = robot.CNDEGetConfig()
        if config:
            states, period = config
            print(f"✓ Connection successful, current configuration: {len(states)} fields, period {period}ms")
        else:
            print("✗ Unable to get CNDE configuration")
            return robot

        # ===== Step 3: Print robot joint and TCP pose =====
        print("\n[Step 3] Printing robot joint and TCP pose...")
        print("  (Tip: Drag the robot to observe data changes)")
        print("  Press Ctrl+C to stop data printing")
        print("  (Use Wireshark to capture packets to verify actual data period)\n")

        sample_count = 0
        try:
            while sample_count < 100:  # Collect 100 samples
                pkg = robot.robot_state_pkg

                # Print every 10 frames
                if sample_count % 10 == 0:
                    print(f"--- Sample #{sample_count} ---")
                    print(f"  Joint positions (deg): [{', '.join([f'{x:.3f}' for x in pkg.jt_cur_pos])}]")
                    print(f"  TCP pose (mm/deg): [{', '.join([f'{x:.3f}' for x in pkg.tl_cur_pos])}]")
                    print(f"  Current frame count: {pkg.frame_cnt}")
                    print()

                sample_count += 1
                time.sleep(0.02)  # 20ms

        except KeyboardInterrupt:
            print("\n  User interrupted data printing")

        # Close connection
        robot.CloseRPC()
        time.sleep(1)

        # ===== Step 4: Modify configuration and verify =====
        print("\n[Step 4] Modifying CNDE configuration...")
        print("  New configuration fields: RobotMode, RbtEnableState")
        print("  New feedback period: 10ms")

        new_states = [
            RobotState.RobotMode,
            RobotState.RbtEnableState,
        ]

        # Set new configuration
        rtn = SetRobotRealtimeStateConfig(new_states, 10)
        if rtn != 0:
            print(f"✗ New configuration setting failed, error code: {rtn}")
            return robot
        print("✓ New configuration set successfully")

        # Reconnect
        robot = Robot.RPC(ROBOT_IP)
        time.sleep(0.5)

        # Verify new configuration
        config = robot.CNDEGetConfig()
        if config:
            states, period = config
            print(f"✓ Current configuration: {[s.name for s in states]}")
            print(f"✓ Current period: {period}ms")

            if period == 10:
                print("✓ Configuration modification verification passed (period changed to 10ms)")
            else:
                print(f"⚠ Period did not take effect (expected 10ms, actual {period}ms)")

            # Print new data
            pkg = robot.robot_state_pkg
            print(f"\n[New Configuration Data]")
            print(f"  robot_mode: {pkg.robot_mode}")
            print(f"  rbtEnableState: {pkg.rbtEnableState}")
        else:
            print("✗ Unable to get new configuration")

        print("\n✓ Test1 completed")
        return robot


    if __name__ == "__main__":
        test1_cnde_config_and_data()


    # ======== Test2: Add/Delete State Field Test ====================
    # Function: Test AddRobotRealtimeState() and DeleteRobotRealtimeState()
    # Test steps:
    #   1. Use AddRobotRealtimeState() to add SpeedScaleManual and SpeedScaleAuto
    #   2. Connect to robot, print manual/automatic mode global speed
    #   3. Modify global speed in WebApp and observe SDK data changes
    #   4. Use DeleteRobotRealtimeState() to delete the added fields
    #   5. Reconnect and verify that speed values are 0 (fields no longer updated)


    def test2_add_delete_state():
        """Test2: Add/Delete state field test - verify dynamic addition and deletion of CNDE states"""
        print_separator("Test2: Add/Delete state field test")

        # ===== Step 1: Add SpeedScaleManual and SpeedScaleAuto fields =====
        print("\n[Step 1] Using AddRobotRealtimeState() to add speed scale fields...")
        print("  Adding fields: SpeedScaleManual, SpeedScaleAuto")

        rtn = AddRobotRealtimeState([
            RobotState.SpeedScaleManual,
            RobotState.SpeedScaleAuto,
        ])

        if rtn != 0:
            print(f"✗ Field addition failed, error code: {rtn}")
            return None
        print("✓ Fields added successfully")

        # ===== Step 2: Establish RPC connection and print speed =====
        print(f"\n[Step 2] Establishing RPC connection ({ROBOT_IP})...")
        robot = Robot.RPC(ROBOT_IP)
        time.sleep(0.5)  # Wait for connection and data reception

        # Verify configuration
        config = robot.CNDEGetConfig()
        if config:
            states, period = config
            print(f"✓ Connection successful, current configuration: {len(states)} fields")
            # Check if added fields are included
            has_manual = RobotState.SpeedScaleManual in states
            has_auto = RobotState.SpeedScaleAuto in states
            if has_manual and has_auto:
                print("✓ Configuration verification passed: SpeedScaleManual and SpeedScaleAuto have been added")
            else:
                print(f"⚠ Configuration verification warning: Manual={has_manual}, Auto={has_auto}")
        else:
            print("✗ Unable to get CNDE configuration")

        # Print speed data
        print("\n[Current Speed Data] (Modify global speed in WebApp to observe changes)")
        print("  Tip: Enable robot by dragging and switch between manual/auto modes to observe speed values")
        print("  Press Ctrl+C to stop data printing\n")

        sample_count = 0
        try:
            while sample_count < 100:  # Collect 100 samples (about 10 seconds at 100ms intervals)
                pkg = robot.robot_state_pkg
                print(f"  [{sample_count:3d}] SpeedScaleManual: {pkg.speedScaleManual:.2f}, "
                    f"SpeedScaleAuto: {pkg.speedScaleAuto:.2f}, "
                    f"Mode: {pkg.robot_mode}")
                sample_count += 1
                time.sleep(0.1)  # 100ms interval
        except KeyboardInterrupt:
            print("\n  User interrupted data printing")

        print(f"\n✓ Data collection completed, total {sample_count} samples")

        # ===== Step 3: Disconnect =====
        print("\n[Step 3] Disconnecting current connection...")
        robot.CloseRPC()
        time.sleep(1.0)  # Wait for CNDE to fully close

        # ===== Step 4: Delete added fields =====
        print("\n[Step 4] Using DeleteRobotRealtimeState() to delete speed scale fields...")
        rtn = DeleteRobotRealtimeState([
            RobotState.SpeedScaleManual,
            RobotState.SpeedScaleAuto,
        ])

        if rtn != 0:
            print(f"✗ Field deletion failed, error code: {rtn}")
            return robot
        print("✓ Fields deleted successfully")

        # ===== Step 5: Reconnect and verify field values are 0 =====
        print(f"\n[Step 5] Reconnecting and verifying deleted field values...")

        robot = Robot.RPC(ROBOT_IP)
        time.sleep(0.5)

        # Read speed values
        pkg = robot.robot_state_pkg
        manual_speed = pkg.speedScaleManual
        auto_speed = pkg.speedScaleAuto

        print(f"\n  SpeedScaleManual after deletion: {manual_speed:.2f}")
        print(f"  SpeedScaleAuto after deletion: {auto_speed:.2f}")

        # Verify if they are 0
        if manual_speed == 0 and auto_speed == 0:
            print("\n✓ Test2 verification passed: Speed values are 0 after field deletion")
        else:
            print(f"\n⚠ Test2 warning: Speed values are non-zero after field deletion")

        print("\n✓ Test2 completed")
        return robot

    if __name__ == "__main__":
        test2_add_delete_state()