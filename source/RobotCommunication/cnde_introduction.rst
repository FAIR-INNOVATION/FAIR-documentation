Introduction to CNDE
=======================

Collaborative robot configurable network data exchange protocol (CNDE) is a way for the client to control the robot and obtain the feedback status of the robot through UDP communication.

Table 1-1 shows all the states of the robot that can be obtained by CNDE. The client can arbitrarily select several required states from the table and make the robot perform state feedback according to the set feedback period.

Similarly, the client can also select the required robot control function combinations from Table 1-2 to perform robot control operations. The communication data between the client and the robot CNDE must comply with the specified frame format. The robot CNDE communication ports are 20005 and 20006. Port 20005 is used for TCP communication, and port 20006 is used for UDP communication.

There are four main steps to use the CNDE function of robot:

①Configuration of input and output data content: the client sends an input and output configuration command to the robot, in which the command content is in the form of a series of control or state function names such as "std_DI_box,cfg_DI_box, motion_queue_len", and the robot records and recognizes these names and then feeds back the corresponding function data types such as "UINT8,UINT8,INT32" to the client, which indicates that the configuration is successful.

②Start the CNDE data output of the robot: the client sends a command to start the CNDE data output to the robot, and the robot starts to send the robot state data to the client through UDP in the form of byte array (Little Endian) according to the configured period.

③Analyze the robot state data: the client receives the state data fed back by the robot circularly, and analyzes the data according to the data types fed back by the robot during output configuration and the byte length corresponding to each data type in Table 1-3 to obtain the actual value of each state. The output data of robot CNDE can support up to 4096 bytes, The CNDE output period ranges from 1 ms to 200ms.

④Sending robot control data: the client groups the control data according to the data types fed back by the robot during input configuration and the byte length corresponding to each data type in Table 1-3, and sends it to the robot through UDP communication. After receiving the control data, the robot performs data analysis and robot control operations. The CNDE input of the robot supports 256 recipes, and the client can configure multiple input recipes as needed. When sending the input data to the robot, it is necessary to specify the recipe number corresponding to the current data.

.. centered:: Table 1-1 Robot Output Configuration Function

.. list-table::
   :widths: 20 40 80
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Name**
     - **Data type**
     - **Description**

   * - std_DI_box
     - UINT8
     - Control box standard DI input (bit0 ~ bit7 indicates DI0 ~ DI7)

   * - cfg_DI_box
     - UINT8
     - Control box configurable CI input (bit0 ~ bit7 indicates CI0 ~ CI7)

   * - cfg_DI_tool
     - UINT8
     - Configurable tool DI inputs (bit0 ~ bit2 indicates toolDI0 ~ toolDI1)

   * - std_AI0_box
     - DOUBLE
     - Control box analog input AI0(0 ~ 4095)

   * - std_AI1_box
     - DOUBLE
     - Control box analog input AI1(0 ~ 4095)

   * - std_AI_tool
     - DOUBLE
     - Analog input of end tool tool_AI0(0 ~ 4095)

   * - run_up_time
     - DOUBLE
     - Statistics of Robot Boot Time (s)

   * - target_joint_pos
     - DOUBLE_6
     - Target position of joint 1-6 (°)

   * - target_joint_vel
     - DOUBLE_6
     - Target speed of joints 1-6 (°/s)

   * - target_joint_acc
     - DOUBLE_6
     - Target acceleration of joints 1-6 (°/s2)

   * - target_joint_current
     - DOUBLE_6
     - Joint 1-6 target current (A)

   * - target_joint_torque
     - DOUBLE_6
     - Target torque of joints 1-6 (Nm)

   * - actual_joint_pos
     - DOUBLE_6
     - Current position of joints 1-6 (°)

   * - actual_joint_vel
     - DOUBLE_6
     - Current speed of joints 1-6 (°/s)

   * - actual_joint_current
     - DOUBLE_6
     - Current current of joints 1-6 (A)

   * - actual_joint_torque
     - DOUBLE_6
     - Joint 1-6 target torque (Nm)

   * - actual_TCP_pos
     - DOUBLE_6
     - Current position of tool DKR(mm)

   * - actual_TCP_vel
     - DOUBLE_6
     - Current tool speed DKR(mm/s)

   * - actual_TCP_force
     - DOUBLE_6
     - Tool resultant force DKR(mm/s2)

   * - target_TCP_pos
     - DOUBLE_6
     - Tool target position DKR(mm)

   * - target_TCP_vel
     - DOUBLE_6
     - Tool target speed DKR(mm/s)

   * - std_DO_box
     - UINT8
     - Standard DO output of control box (bit0 ~ bit7 indicates DO0 ~ DO7)

   * - cfg_DO_box
     - UINT8
     - Control box configurable with CO output (bit0 ~ bit7 indicates CO0 ~ CO7)

   * - cfg_DO_tool
     - UINT8
     - Standard tool DO output (bit0 ~ bit1 indicates toolDO0 ~ toolDO1)

   * - std_AO0_box
     - DOUBLE
     - Control box analog AO0 (0.0 ~ 4095.0)

   * - std_AO1_box
     - DOUBLE
     - Control box analog AO1 (0.0 ~ 4095.0)

   * - std_AO_tool
     - DOUBLE
     - Tool analog AO1 (0.0 ~ 4095.0)

   * - robot_mode
     - UINT8
     - Robot mode (0- automatic; 1- Manual)

   * - collision_level
     - UINT8_6
     - Joint 1-6 collision grade (1-10)

   * - speed_scaling_man
     - DOUBLE
     - Manual mode speed percentage (0 ~ 100)

   * - speed_scaling_auto
     - DOUBLE
     - Automatic mode speed percentage (0 ~ 100)

   * - program_state
     - UINT8
     - Robot program running state (1- stop; 2- in motion; 3- pause; 4- Drag)

   * - line_number
     - INT32
     - Current program running line number

   * - payload
     - DOUBLE
     - Load mass (kg)

   * - pay_cog
     - DOUBLE_3
     - Load centroid (x,y,z)(mm)

   * - motion_queue_len
     - INT32
     - Current motion queue length

   * - ft_sensor_data
     - DOUBLE_6
     - Force sensor raw data

   * - main_code
     - INT32
     - Main fault code

   * - sub_code
     - INT32
     - Sub fault code

   * - emergency_stop
     - UINT8
     - Emergency stop status

   * - motion_done
     - INT32
     - Motion completion status

   * - timestamp_us
     - UINT64
     - Robot system time (us)
  
   * - output_BIT_reg_8xX
     - UINT8_X
     - BIT-type robot output registers (8xX indicates the number of registers, if you need 16 BIT-type output registers, the actual name is "output_BIT_reg_8x2", and the robot can support up to 128 bit-type output registers)

   * - output_INT_reg_X
     - INT32_X
     - INT robot output registers (X represents the number of registers. If you need 16 INT output registers, the actual name is "output_INT_reg_16", and the robot can support up to 64 INT output registers)

   * - output_DOUBLE_reg_X
     - DOUBLE_X
     - DOUBLE robot output register (X represents the number of registers, if you need 16 DOUBLE output registers, the actual name is "output_DOUBLE_reg_16", and the robot can support up to 64 DOUBLE output registers)

   * - servoj_time
     - UINT64_5
     - servoj timestamp data, unit ns

   * - actual_joint_temp
     - DOUBLE_6
     - Joint j1-j6 driver temperature, unit °

   * - axle_gen_com_data
     - UINT8_130
     - End-effector general periodic data

   * - abnormal_stop
     - UINT8
     - Abnormal status, 0: no abnormality, 1: abnormality present

   * - cur_lua_file_name
     - UINT8_256
     - Currently running lua file name

   * - prog_total_line
     - UINT8
     - Total number of lines in current file

   * - safety_box_signal
     - UINT8_6
     - Button box key signal 0: pressed 1: released

   * - welding_voltage
     - DOUBLE
     - Actual welding voltage, unit V

   * - welding_current
     - DOUBLE
     - Actual welding current, unit A

   * - welding_track_speed
     - DOUBLE
     - Weld seam tracking speed, unit mm/s

   * - tpd_exception
     - UINT8
     - TPD exception

   * - alarm_reboot_robot
     - UINT8
     - Robot restart warning

   * - modbus_master_connect
     - UINT8
     - | Connection status of 8 Modbus masters  
       | bit0-bit7 represent masters 0-7, 0-not connected, 1-connected

   * - modbus_slave_connect
     - UINT8
     - Modbus slave connection status, 0-not connected, 1-connected

   * - btn_box_stop_signal
     - UINT8
     - Emergency stop button signal

   * - drag_alarm
     - UINT8
     - Drag warning

   * - safety_door_alarm
     - UINT8
     - Safety door warning, 0 no warning, 1 safety door triggered

   * - safety_plane_alarm
     - UINT8
     - Safety wall warning, 0 no warning, 1 entering safety wall

   * - motion_alarm
     - UINT8
     - | Motion warning, 0-no warning,
       | 1-LIN instruction posture change too large,
       | 2-TCP overspeed,
       | 3-Collision occurred,
       | 4-Axis 1 exceeded maximum speed,
       | 5-Axis 2 exceeded maximum speed,
       | 6-Axis 3 exceeded maximum speed,
       | 7-Axis 4 exceeded maximum speed,
       | 8-Axis 5 exceeded maximum speed,
       | 9-Axis 6 exceeded maximum speed,
       | 10-Axis 1 joint command and feedback deviation too large,
       | 11-Axis 2 joint command and feedback deviation too large,
       | 12-Axis 3 joint command and feedback deviation too large,
       | 13-Axis 4 joint command and feedback deviation too large,
       | 14-Axis 5 joint command and feedback deviation too large,
       | 15-Axis 6 joint command and feedback deviation too large,
       | 16-Singular posture,
       | 17-LIN instruction motion speed adaptive,
       | 18-Target speed adjustment timeout or incomplete,
       | 19-Rotary insertion motion failed,
       | 20-Helical insertion motion failed,
       | 21-Linear insertion motion failed,
       | 22-Surface positioning motion failed

   * - interfere_alarm
     - UINT8
     - Interference area warning, 0 no warning, 1 entering interference area

   * - udp_cmd_state
     - INT32
     - UDP connection status

   * - weld_ready_state
     - UINT8
     - Welder ready status, 1: ready, 0: welder error

   * - alarm_check_emerg_stop_btn
     - UINT8
     - Joint communication abnormality warning

   * - ts_tm_cmd_com_error
     - UINT8
     - Torque system command error

   * - ts_tm_state_com_error
     - UINT8
     - Torque system status error

   * - ctrl_box_error
     - INT32
     - Control box error

   * - safety_data_state
     - UINT8
     - Safety data status flag, 0 no abnormality, 1-abnormality

   * - force_sensor_err_state
     - UINT8
     - Force sensor connection timeout fault; bit0-bit1 correspond to force sensor ID1-ID2

   * - ctrl_open_lua_errcode
     - UINT8_4
     - Controller open protocol error code

   * - servo_id
     - UINT8
     - Servo driver ID number

   * - servo_errcode
     - INT32
     - Servo driver fault code

   * - servo_state
     - INT32
     - Servo driver status

   * - servo_actual_pos
     - DOUBLE
     - Servo current position

   * - servo_actual_speed
     - DOUBLE
     - Servo current speed
   
   * - servo_actual_torque
     - DOUBLE
     - Servo current torque
   
   * - gripper_active
     - INT32
     - Gripper activation status
   
   * - gripper_position
     - UINT8
     - Gripper position feedback (percentage)
   
   * - gripper_speed
     - INT32
     - Gripper speed feedback (percentage)
   
   * - gripper_current
     - INT32
     - Gripper current feedback (percentage)
   
   * - gripper_temp
     - INT32
     - Gripper current temperature, unit °
   
   * - gripper_voltage
     - INT32
     - Gripper current voltage, unit V
   
   * - rotating_gripper_num
     - DOUBLE
     - Rotating gripper rotation count feedback
   
   * - rotating_gripper_speed
     - UINT8
     - Rotating gripper rotation speed feedback (percentage)
   
   * - rotating_gripper_tor
     - UINT8
     - Rotating gripper rotation torque feedback (percentage)
   
   * - weld_break_off_state
     - UINT8
     - Welding interruption status: 0-welding not interrupted, 1-welding interrupted
   
   * - weld_arc_state
     - UINT8
     - Welding arc status, 0-arc not interrupted, 1-arc interrupted
   
   * - smarttool_state
     - UINT32
     - End-effector extended IO data (Smart-Tool)
   
   * - tool_coord
     - DOUBLE_6
     - Current tool pose relative to end-effector
   
   * - wobj_coord
     - DOUBLE_6
     - Current workpiece pose relative to base
   
   * - exTool_coord
     - DOUBLE_6
     - Current external tool pose relative to tool
   
   * - exAxis_coord
     - DOUBLE_6
     - Current external axis pose relative to base coordinate system
   
   * - robot_state
     - UINT8
     - | Robot running status,
       | 1. Stopped; 2. Moving; 3. Paused; 4. Dragging
   
   * - actual_flange_pos
     - DOUBLE_6
     - End-effector actual pose
   
   * - target_TCP_cmpvel
     - DOUBLE_2
     - TCP command linear and angular velocity, unit mm/s, °/s
   
   * - actual_TCP_cmpvel
     - DOUBLE_2
     - TCP actual linear and angular velocity, unit mm/s, °/s
   
   * - tool_id
     - INT32
     - Tool number
   
   * - wobj_id
     - INT32
     - Workpiece number
   
   * - ft_sensor_raw_data
     - DOUBLE_6
     - End-effector force/torque - raw data in sensor coordinate system
   
   * - ft_sensor_active
     - UINT8
     - Force/torque sensor activation status
   
   * - gripper_motion_done
     - UINT8
     - Gripper motion completed
   
   * - collision_state
     - UINT8
     - Collision detection status
   
   * - trajectory_pnum
     - INT32
     - Current sequence number of discrete point motion
   
   * - safety_stop0_state
     - UINT8
     - Safety stop SI0 signal status
   
   * - safety_stop1_state
     - UINT8
     - Safety stop SI1 signal status
   
   * - gripper_fault_id
     - UINT8
     - Faulty gripper number
   
   * - gripper_fault
     - INT32
     - Gripper error number
   
   * - ext_DI_state
     - UINT8_16
     - Extended DI
   
   * - ext_DO_state
     - UINT8_16
     - Extended DO
   
   * - ext_AI_state
     - INT32_4
     - Extended AI
   
   * - ext_AO_state
     - INT32_4
     - Extended AO
   
   * - rbt_enable_state
     - INT32
     - Driver enable completion status
   
   * - joint_driver_torque
     - DOUBLE_6
     - Actual torque values of joints j1-j6, unit Nm
   
   * - robot_time
     - INT32_7
     - Robot system time, year, month, day, hour, minute, second, millisecond
   
   * - software_upgrade_state
     - INT32
     - Robot software upgrade status
   
   * - end_lua_err_code
     - INT32
     - End button signal status
   
   * - wide_voltage_ctrl_box_temp
     - DOUBLE
     - Wide voltage control box temperature, unit °
   
   * - wide_voltage_ctrl_box_fan_current
     - INT32
     - Wide voltage control box fan current
   
   * - last_servoJ_target
     - DOUBLE_6
     - Last servoJ target command pose
   
   * - servoJ_cmd_num
     - INT32
     - ServoJ command count
   
   * - strange_pos_flag
     - UINT8
     - Singular posture flag
   
   * - alarm
     - UINT8
     - | Warning, 0-no warning,
       | 1-Shoulder joint configuration change,
       | 2-Elbow joint configuration change,
       | 3-Wrist joint configuration change,
       | 4-RPY initialization failed,
       | 5-WaitDI wait timeout,
       | 6-WaitAI wait timeout,
       | 7-WaitToolDI wait timeout,
       | 8-WaitToolAI wait timeout,
       | 9-Arc start success DI not configured,
       | 10-WaitAuxDI wait timeout,
       | 11-WaitAuxAI wait timeout,
       | 12-Unreachable point in swing trajectory,
       | 13-Conveyor tracking pick point calculation failed,
       | 14-Joint torque sensor data abnormality
   
   * - dr_alarm
     - UINT8
     - | Driver warning, 0-no warning,
       | 1-Axis 1 driver warning, resettable,
       | 2-Axis 2 driver warning, resettable,
       | 3-Axis 3 driver warning, resettable,
       | 4-Axis 4 driver warning, resettable,
       | 5-Axis 5 driver warning, resettable,
       | 6-Axis 6 driver warning, resettable
   
   * - alive_slave_num_error
     - UINT8
     - Active slave count fault, 0-no fault, 1-error, non-resettable
   
   * - slave_com_error
     - UINT8_8
     - | Slave communication fault, 0-no fault,
       | 1-Slave offline,
       | 2-Slave status inconsistent with set value,
       | 3-Slave not configured,
       | 4-Slave configuration error,
       | 5-Slave initialization error,
       | 6-Slave mailbox communication initialization error
   
   * - cmd_point_error
     - UINT8
     - | Command point fault, 0-no fault,
       | 1-Joint command point error,
       | 2-Linear target point error,
       | 3-Arc intermediate point error,
       | 4-Arc target point error,
       | 5-Arc command point spacing too small,
       | 6-Full circle/helix intermediate point 1 error,
       | 7-Full circle/helix intermediate point 2 error,
       | 8-Full circle/helix intermediate point 3 error,
       | 9-Full circle/helix command point spacing too small,
       | 10-TPD command point error,
       | 11-TPD command tool does not match current tool,
       | 12-TPD current command and next command start point deviation too large,
       | 13-Internal/external tool switching error,
       | 14-New helix start point error,
       | 15-New spline curve command point error,
       | 17-PTP joint command out of limit,
       | 18-TPD joint command out of limit,
       | 19-LIN\ARC issued joint command out of limit,
       | 20-Command overspeed in Cartesian space,
       | 21-Torque command out of limit in joint space,
       | 22-JOG joint command out of limit,
       | 23-Axis 1 command speed out of limit in joint space,
       | 24-Axis 2 command speed out of limit in joint space,
       | 25-Axis 3 command speed out of limit in joint space,
       | 26-Axis 4 command speed out of limit in joint space,
       | 27-Axis 5 command speed out of limit in joint space,
       | 28-Axis 6 command speed out of limit in joint space,
       | 29-Joint feedback speed out of limit,
       | 30-Joint command and feedback deviation too large,
       | 31-DMP target point error (including tool mismatch),
       | 32-TCP speed out of limit,
       | 33-Next command joint configuration change,
       | 34-Current command joint configuration change,
       | 35-Joint speed out of limit in LIN command,
       | 36-LIN command adaptive speed exceeds threshold,
       | 37-Unreachable point in trajectory,
       | 38-Unreachable point in trajectory - singular posture,
       | 49-PTP and SPTP commands not allowed between ARCSTART and ARCEND,
       | 50-PTP and SPTP commands not allowed between WEAVESTART and WEAVEEND,
       | 51-Weave welding parameter error,
       | 52-Weave welding command point spacing too small,
       | 53-Unreachable point in swing trajectory - singular posture,
       | 54-Unreachable point in swing trajectory - joint command out of limit,
       | 55-Unreachable point in swing trajectory - planning exception (tool Z coincides with forward direction X angle),
       | 56-Unreachable point in swing trajectory - planning exception (arc path point error),
       | 65-Laser sensor command deviation too large,
       | 66-Laser sensor command interrupted, seam tracking ended early,
       | 81-External axis command speed out of limit,
       | 82-External axis command and feedback deviation too large,
       | 83-Extended peripheral (external axis/IO) communication interrupted,
       | 84-Extended peripheral (external axis/IO) communication packet loss exception,
       | 85-External axis 1 command speed out of limit in joint space,
       | 86-External axis 2 command speed out of limit in joint space,
       | 87-External axis 3 command speed out of limit in joint space,
       | 88-External axis 4 command speed out of limit in joint space,
       | 89-Extended peripheral (external axis/IO) Ethercat communication error,
       | 90-Extended peripheral (external axis/IO) Canopen communication error,
       | 97-Conveyor tracking - start point and reference point posture change too large,
       | 113-Constant force control - X direction exceeds maximum adjustment distance,
       | 114-Constant force control - Y direction exceeds maximum adjustment distance,
       | 115-Constant force control - Z direction exceeds maximum adjustment distance,
       | 116-Constant force control - RX direction exceeds maximum adjustment angle,
       | 117-Constant force control - RY direction exceeds maximum adjustment angle,
       | 118-Constant force control - RZ direction exceeds maximum adjustment angle,
       | 119-External sensor data error,
       | 120-Helical search motion failed,
       | 121-Rotary insertion motion failed,
       | 122-Linear insertion motion failed,
       | 123-Surface positioning motion failed,
       | 124-Abnormal dragging force, entering drag failed,
       | 129-Exceeded maximum torque recording points,
       | 130-Speed switching error,
       | 131-Tool direction out of limit,
       | 132-Momentum out of limit,
       | 133-Power out of limit,
       | 134-STL-Flash diagnostic abnormality,
       | 135-STL-RAM diagnostic abnormality,
       | 136-STL-CPU diagnostic abnormality,
       | 137-Safety board II-ECAT safety data abnormality,
       | 138-Axis 1 ECAT safety data abnormality,
       | 139-Axis 2 ECAT safety data abnormality,
       | 140-Axis 3 ECAT safety data abnormality,
       | 141-Axis 4 ECAT safety data abnormality,
       | 142-Axis 5 ECAT safety data abnormality,
       | 143-Axis 6 ECAT safety data abnormality,
       | 145-Abnormal communication between safety board and controller,
       | 147-Focus following error,
       | 148-Angular velocity out of limit,
       | 149-Joint status word feedback abnormality
   
   * - IO_error
     - UINT8
     - | IO fault, 0-no fault,
       | 1-Channel error, resettable,
       | 2-Value error, resettable,
       | 3-WaitDI wait timeout, resettable,
       | 4-WaitAI wait timeout, resettable,
       | 5-WaitAxleDI wait timeout, resettable,
       | 6-WaitAxleAI wait timeout, resettable,
       | 7-Channel configured function error, resettable,
       | 8-Arc start timeout, resettable,
       | 9-Arc end timeout, resettable,
       | 10-Search timeout, resettable,
       | 11-Conveyor IO detection timeout, resettable,
       | 12-WaitAuxDI wait timeout, resettable,
       | 13-WaitAuxAI wait timeout, resettable,
       | 14-Wire search timeout, resettable
   
   * - gripper_error
     - UINT8
     - Gripper fault, 0-no fault, 1-gripper motion timeout error, resettable
   
   * - file_error
     - UINT8
     - | Configuration file fault, 0-no fault,
       | 1-zbt configuration file version error, initialization error - non-resettable,
       | 2-zbt configuration file load failed, initialization error - non-resettable,
       | 3-user configuration file version error, initialization error - non-resettable,
       | 4-user configuration file load failed, initialization error - non-resettable,
       | 5-exaxis configuration file version error, initialization error - non-resettable,
       | 6-exaxis configuration file load failed, initialization error - non-resettable,
       | 7-Robot model mismatch, need to reset - non-resettable,
       | 8-dhpara configuration file version error, initialization error - non-resettable,
       | 9-dhpara configuration file load failed, initialization error - non-resettable,
       | 10-Robot model not set - non-resettable,
       | 11-load configuration file version error, initialization error - non-resettable,
       | 12-load configuration file load failed, initialization error - non-resettable,
       | 13-speed configuration file version error, initialization error - non-resettable,
       | 14-speed configuration file load failed, initialization error - non-resettable,
       | 15-weavepara configuration file version error, initialization error - non-resettable,
       | 16-weavepara configuration file load failed, initialization error - non-resettable
   
   * - para_error
     - UINT8
     - | Configuration parameter fault, 0-no fault,
       | 1-Tool number out of limit error - resettable,
       | 2-Positioning completion threshold error - resettable,
       | 3-Collision level error - resettable,
       | 4-Load weight error - resettable,
       | 5-Load center of mass X error - resettable,
       | 6-Load center of mass Y error - resettable,
       | 7-Load center of mass Z error - resettable,
       | 8-DI filter time error - resettable,
       | 9-AxleDI filter time error - resettable,
       | 10-AI filter time error - resettable,
       | 11-AxleAI filter time error - resettable,
       | 12-DI high/low level range error - resettable,
       | 13-DO high/low level range error - resettable,
       | 14-Workpiece number out of limit error - resettable,
       | 15-External axis number out of limit error - resettable,
       | 16-Conveyor tracking - encoder channel error - resettable,
       | 17-Conveyor tracking - workpiece axis number error - resettable,
       | 18-FR30L mounting method - non-standard mounting error - resettable
   
   * - exaxis_out_slimit_error
     - UINT8
     - | External axis soft limit fault, 0-no fault,
       | 1-External axis 1 out of soft limit fault, resettable,
       | 2-External axis 2 out of soft limit fault, resettable,
       | 3-External axis 3 out of soft limit fault, resettable,
       | 4-External axis 4 out of soft limit fault, resettable,
       | 5-External axis 5 out of soft limit fault, resettable,
       | 6-External axis 6 out of soft limit fault, resettable
   
   * - dr_com_err
     - UINT8_6
     - Driver communication fault, 0-no fault, 1-fault
   
   * - dr_err
     - UINT8
     - | Driver fault, 0-no fault,
       | 1-Axis 1 driver fault, non-resettable,
       | 2-Axis 2 driver fault, non-resettable,
       | 3-Axis 3 driver fault, non-resettable,
       | 4-Axis 4 driver fault, non-resettable,
       | 5-Axis 5 driver fault, non-resettable,
       | 6-Axis 6 driver fault, non-resettable
   
   * - out_sflimit_err
     - UINT8
     - | Soft limit fault, 0-no fault,
       | 1-Axis 1 out of soft limit fault, resettable,
       | 2-Axis 2 out of soft limit fault, resettable,
       | 3-Axis 3 out of soft limit fault, resettable,
       | 4-Axis 4 out of soft limit fault, resettable,
       | 5-Axis 5 out of soft limit fault, resettable,
       | 6-Axis 6 out of soft limit fault, resettable

.. centered:: Table 1-2 Configuration Functions of Robot Input Control

.. list-table::
   :widths: 20 40 80
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Name**
     - **Data type**
     - **Description**

   * - speed_mask
     - UINT8
     - Global speed setting mask: 0-disable; 1- enable

   * - speed
     - UINT8
     - Set the global speed (0-100)

   * - std_DO_mask
     - UINT8
     - Control box standard DO output control mask (bit0 ~ bit7 indicates DO0 ~ DO7)

   * - std_DO_box
     - UINT8
     - Control box standard DO output (bit0 ~ bit7 indicates DO0 ~ DO7)

   * - cfg_DO_mask
     - UINT8
     - Control box configurable CO output mask (bit0 ~ bit7 indicates CO0 ~ CO7)

   * - cfg_DO_box
     - UINT8
     - Control box Configurable with CO output (bit0 ~ bit7 indicates CO0 ~ CO7)

   * - cfg_DO_tool_mask
     - UINT8
     - Control box standard tool DO output control mask (bit0 ~ bit1 indicates toolDO0 ~ toolDO1)

   * - cfg_DO_tool
     - UINT8
     - Control box standard tool DO output (bit0 ~ bit1 indicates toolDO0 ~ toolDO1)

   * - std_AO_mask
     - UINT8
     - Robot analog output control mask (bit0 ~ bit1 indicates control box AO0 ~ AO1；; Bit2 stands for tool AO0)

   * - std_AO0_box
     - DOUBLE
     - Control box analog AO0 (0.0 ~ 4095.0)

   * - std_AO1_box
     - DOUBLE
     - Control box analog AO1 (0.0 ~ 4095.0)

   * - std_AO0_tool
     - DOUBLE
     - Tool analog AO1 (0.0 ~ 4095.0)

   * - input_BIT_reg_8xX
     - UINT8_X
     - BIT-type robot input registers (8xX indicates the number of registers, if you need 16 BIT-type input registers, the actual name is "input_BIT_reg_8x2", and the robot can support up to 128 bit-type registers)

   * - input_INT_reg_X
     - INT32_X
     - INT robot input registers (X represents the number of registers, if you need 16 INT input registers, the actual name is "input_INT_reg_16", and the robot can support up to 64 INT registers)
  
   * - input_DOUBLE_reg_X
     - DOUBLE_X
     - DOUBLE robot input register (X represents the number of registers, if you need 16 DOUBLE input registers, the actual name is "input_DOUBLE_reg_16", and the robot can support up to 64 DOUBLE registers)

.. centered:: Table 1-3 Correspondence between Data Types and Byte Length

.. list-table::
   :widths: 60 40
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Data type**
     - **Byte length**

   * - UINT8
     - 1

   * - INT32
     - 4

   * - DOUBLE
     - 8

   * - UINT8_X
     - 1*X

   * - INT32_X
     - 4*X

   * - DOUBLE_X
     - 8*X