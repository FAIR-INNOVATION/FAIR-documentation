Status feedback information
==============================================================================

.. toctree::
    :maxdepth: 5

Status Feedback Comparison Table
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. versionadded:: python SDK-v2.0.4
    
.. csv-table:: 
    :header-rows: 1
    :name: Status Feedback Comparison Table
    :widths: 20 30

    "Variable", "Meaning"
    "program_state", "program_running_state, 1-stop; 2-run; 3-pause"
    "robot_state", "robot_motion_state, 1-stop; 2-run; 3-pause; 4-drag"
    "main_code", "main_fault_code"
    "sub_code", sub_code"
    "robot_mode", "robot_mode, 0-automatic mode; 1-manual mode"
    "jt_cur_pos[i]", "Current position of joint, in deg, i:0~5"
    "tl_cur_pos[i]", "Tool current position in deg&mm,i:0~5"
    "flange_cur_pos[i]", "The end flange is in its current position, in degrees and millimeters. i: 0~5"
    "actual_qd[i]", "Robot's current joint velocity in deg/s^2,i:0~5"
    "actual_qdd[i]", "Current joint acceleration of the robot, in mm/s, i:0~5"
    "target_TCP_CmpSpeed[i]", "Robot TCP synthesis command speed in mm/s & deg/s,i:0~1"
    "target_TCP_Speed[i]", "Robot TCP command speed in mm/s & deg/s,i:0~5"
    "actual_TCP_CmpSpeed[i]", "Robot TCP synthesized actual speed in mm/s & deg/s, i:0~1"
    "actual_TCP_Speed[i]", "Robot TCP actual speed in mm/s & deg/s,i:0~5"
    "jt_cur_tor[i]", "Current torque, unit N-m ,i:0~5"
    "tool", "Applied tool coordinate system number"
    "user", "Applied workpiece coordinate system number"
    "cl_dgt_output_h", "Control Box Digital IO Output 15-8"
    "cl_dgt_output_l", "Control Box Digital IO Output 7-0"
    "tl_dgt_output_l", "Tool Digital IO Output 7-0, only bit0-bit1 valid"
    "dgt_input_h", "Control Box Digital IO Input 15-8"
    "cl_dgt_input_l", "Control Box Digital IO Input 7-0"
    "tl_dgt_input_l", "Tool digital IO input 7-0, only bit0-bit1 valid"
    "cl_analog_input[i]", "Control box analog input,i:0~2"
    "tl_anglog_input", "tool_analog_input"
    "ft_sensor_raw_data", "torque sensor raw data, unit N&Nm, i:0~5"
    "ft_sensor_data", "torque sensor data, unit N&Nm, i:0~5"
    "ft_sensor_active", "torque sensor active status, 0-reset, 1-active"
    "EmergencyStop", "Emergency stop sign, 0 - emergency stop not pressed, 1 - emergency stop pressed"
    "motion_done", "motion_in_place signal,1-in place, 0-not in place"
    "gripper_motiondone", "Gripper motion done signal,1-done, 0-not done "
    "mc_queue_len", "motion command queue length"
    "collisionState", "Collision detection, 1-collision, 0-no collision "
    "trajectory_pnum", "trajectory point number"
    "safety_stop0_state", "safety stop signal SI0"
    "safety_stop1_state", "safety stop signal SI1"
    "gripper_fault_id", "error_claw_number"
    "gripper_fault", "gripper_fault"
    "gripper_active", "gripper_jaw_activity_status, 0-unactivated, 1-activated"
    "gripper_position", "Gripper position (percentage)"
    "gripper_speed", "Gripper speed (percentage)"
    "gripper_current", "gripper_current (percentage)"
    "gripper_tmp", "Gripper temperature in °C"
    "gripper_voltage", "gripper_voltage in V"
    "auxState.servoId", "485 extended axis, servo drive ID number, i:0~3"
    "auxState.servoErrCode", "485 Extended Axis, Servo Drive Error Code, i:0~3"
    "auxState.servoState", "485 Extended Axis, Servo Drive State, i:0~3"
    "auxState.servoPos", "485 extended axis, servo current position, i:0~3"
    "auxState.servoVel", "485 extended axis, servo current speed, i:0~3"
    "auxState.servoTorque", "485 Extended axis, servo current torque, i:0~3"
    "extAxisStatus[i].pos", "UDP Extension Axis, Position, i:0~3"
    "extAxisStatus[i].vel", "UDP Extended Axis,velocity,i:0~3"
    "extAxisStatus[i].errorCode", "UDP Extended Axis, Error Code, i:0~3"
    "extAxisStatus[i].ready", "UDP extension axis, servo ready, i:0~3"
    "extAxisStatus[i].inPos", "UDP Extended Axis, servo in place, i:0~3"
    "extAxisStatus[i].alarm", "UDP Extended Axis, Servo Alarm, i:0~3"
    "extAxisStatus[i].flerr", "UDP Extended Axis, Follow Error, i:0~3"
    "extAxisStatus[i].nlimit", "UDP extension axis, to negative limit, i:0~3"
    "extAxisStatus[i].pLimit", "UDP extension axis, to positive limit, i:0~3"
    "extAxisStatus[i].mdbsOffLine", "UDP Extension Axis, Drive 485 Bus Offline"
    "extAxisStatus[i].mdbsTimeout", "UDP Extension Axis, Control Card and Control Box 485 Communication Timeout"
    "extAxisStatus[i].homingStatus", "UDP extension axis, back to zero status"
    "extDIState", "Extended Digital Input State"
    "extDOState", "Extended Digital Output State"
    "extAIState", "Extended Analog Input State"
    "extAOState", "Extended Analog Output State"
    "rbtEnableState", "robotEnableState"
    "jointDriverTorque", "Joint Driver Current Torque"
    "jointDriverTemperature", "Joint Driver Current Temperature"
    "year", "year"
    "mouth", "moon"
    "day", "day"
    "hour", "hours"
    "minute", "minutes"
    "second", "seconds"
    "millisecond", "milliseconds"
    "softwareUpgradeState", "Robot Software Upgrade State"
    "endLuaErrCode", "endLUARunningStatus"
    
code example
---------------------------------------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # Establish a connection with the robot controller and return a robot object if the connection is successful
    robot = Robot.RPC('192.168.58.2')
    print("program_state:", robot.robot_state_pkg.program_state)
    print("robot_state:", robot.robot_state_pkg.robot_state)
    print("main_code:", robot.robot_state_pkg.main_code)
    print("sub_code:", robot.robot_state_pkg.sub_code)
    print("robot_mode:", robot.robot_state_pkg.robot_mode)
    print("jt_cur_pos0:", robot.robot_state_pkg.jt_cur_pos[0])
    print("jt_cur_pos1:", robot.robot_state_pkg.jt_cur_pos[1])
    print("jt_cur_pos2:", robot.robot_state_pkg.jt_cur_pos[2])
    print("jt_cur_pos3:", robot.robot_state_pkg.jt_cur_pos[3])
    print("jt_cur_pos4:", robot.robot_state_pkg.jt_cur_pos[4])
    print("jt_cur_pos5:", robot.robot_state_pkg.jt_cur_pos[5])
    print("tl_cur_pos0:", robot.robot_state_pkg.tl_cur_pos[0])
    print("tl_cur_pos1:", robot.robot_state_pkg.tl_cur_pos[1])
    print("tl_cur_pos2:", robot.robot_state_pkg.tl_cur_pos[2])
    print("tl_cur_pos3:", robot.robot_state_pkg.tl_cur_pos[3])
    print("tl_cur_pos4:", robot.robot_state_pkg.tl_cur_pos[4])
    print("tl_cur_pos5:", robot.robot_state_pkg.tl_cur_pos[5])
    print("flange_cur_pos0:", robot.robot_state_pkg.flange_cur_pos[0])
    print("flange_cur_pos1:", robot.robot_state_pkg.flange_cur_pos[1])
    print("flange_cur_pos2:", robot.robot_state_pkg.flange_cur_pos[2])
    print("flange_cur_pos3:", robot.robot_state_pkg.flange_cur_pos[3])
    print("flange_cur_pos4:", robot.robot_state_pkg.flange_cur_pos[4])
    print("flange_cur_pos5:", robot.robot_state_pkg.flange_cur_pos[5])
    print("actual_qd0:", robot.robot_state_pkg.actual_qd[0])
    print("actual_qd1:", robot.robot_state_pkg.actual_qd[1])
    print("actual_qd2:", robot.robot_state_pkg.actual_qd[2])
    print("actual_qd3:", robot.robot_state_pkg.actual_qd[3])
    print("actual_qd4:", robot.robot_state_pkg.actual_qd[4])
    print("actual_qd5:", robot.robot_state_pkg.actual_qd[5])
    print("actual_qdd0:", robot.robot_state_pkg.actual_qdd[0])
    print("actual_qdd1:", robot.robot_state_pkg.actual_qdd[1])
    print("actual_qdd2:", robot.robot_state_pkg.actual_qdd[2])
    print("actual_qdd3:", robot.robot_state_pkg.actual_qdd[3])
    print("actual_qdd4:", robot.robot_state_pkg.actual_qdd[4])
    print("actual_qdd5:", robot.robot_state_pkg.actual_qdd[5])
    print("target_TCP_CmpSpeed0:", robot.robot_state_pkg.target_TCP_CmpSpeed[0])
    print("target_TCP_CmpSpeed1:", robot.robot_state_pkg.target_TCP_CmpSpeed[1])
    print("target_TCP_Speed0:", robot.robot_state_pkg.target_TCP_Speed[0])
    print("target_TCP_Speed1:", robot.robot_state_pkg.target_TCP_Speed[1])
    print("target_TCP_Speed2:", robot.robot_state_pkg.target_TCP_Speed[2])
    print("target_TCP_Speed3:", robot.robot_state_pkg.target_TCP_Speed[3])
    print("target_TCP_Speed4:", robot.robot_state_pkg.target_TCP_Speed[4])
    print("target_TCP_Speed5:", robot.robot_state_pkg.target_TCP_Speed[5])
    print("actual_TCP_CmpSpeed0:", robot.robot_state_pkg.actual_TCP_CmpSpeed[0])
    print("actual_TCP_CmpSpeed1:", robot.robot_state_pkg.actual_TCP_CmpSpeed[1])
    print("actual_TCP_Speed0:", robot.robot_state_pkg.actual_TCP_Speed[0])
    print("actual_TCP_Speed1:", robot.robot_state_pkg.actual_TCP_Speed[1])
    print("actual_TCP_Speed2:", robot.robot_state_pkg.actual_TCP_Speed[2])
    print("actual_TCP_Speed3:", robot.robot_state_pkg.actual_TCP_Speed[3])
    print("actual_TCP_Speed4:", robot.robot_state_pkg.actual_TCP_Speed[4])
    print("actual_TCP_Speed5:", robot.robot_state_pkg.actual_TCP_Speed[5])
    print("jt_cur_tor0:", robot.robot_state_pkg.jt_cur_tor[0])
    print("jt_cur_tor1:", robot.robot_state_pkg.jt_cur_tor[1])
    print("jt_cur_tor2:", robot.robot_state_pkg.jt_cur_tor[2])
    print("jt_cur_tor3:", robot.robot_state_pkg.jt_cur_tor[3])
    print("jt_cur_tor4:", robot.robot_state_pkg.jt_cur_tor[4])
    print("jt_cur_tor5:", robot.robot_state_pkg.jt_cur_tor[5])
    print("tool:", robot.robot_state_pkg.tool)
    print("user:", robot.robot_state_pkg.user)
    print("cl_dgt_output_h:", robot.robot_state_pkg.cl_dgt_output_h)
    print("cl_dgt_output_l:", robot.robot_state_pkg.cl_dgt_output_l)
    print("tl_dgt_output_l:", robot.robot_state_pkg.tl_dgt_output_l)
    print("cl_dgt_input_h:", robot.robot_state_pkg.cl_dgt_input_h)
    print("cl_dgt_input_l:", robot.robot_state_pkg.cl_dgt_input_l)
    print("tl_dgt_input_l:", robot.robot_state_pkg.tl_dgt_input_l)
    print("cl_analog_input0:", robot.robot_state_pkg.cl_analog_input[0])
    print("cl_analog_input1:", robot.robot_state_pkg.cl_analog_input[1])
    print("tl_anglog_input:", robot.robot_state_pkg.tl_anglog_input)
    print("ft_sensor_raw_data0:", robot.robot_state_pkg.ft_sensor_raw_data[0])
    print("ft_sensor_raw_data1:", robot.robot_state_pkg.ft_sensor_raw_data[1])
    print("ft_sensor_raw_data2:", robot.robot_state_pkg.ft_sensor_raw_data[2])
    print("ft_sensor_raw_data3:", robot.robot_state_pkg.ft_sensor_raw_data[3])
    print("ft_sensor_raw_data4:", robot.robot_state_pkg.ft_sensor_raw_data[4])
    print("ft_sensor_raw_data5:", robot.robot_state_pkg.ft_sensor_raw_data[5])
    print("ft_sensor_data0:", robot.robot_state_pkg.ft_sensor_data[0])
    print("ft_sensor_data1:", robot.robot_state_pkg.ft_sensor_data[1])
    print("ft_sensor_data2:", robot.robot_state_pkg.ft_sensor_data[2])
    print("ft_sensor_data3:", robot.robot_state_pkg.ft_sensor_data[3])
    print("ft_sensor_data4:", robot.robot_state_pkg.ft_sensor_data[4])
    print("ft_sensor_data5:", robot.robot_state_pkg.ft_sensor_data[5])
    print("ft_sensor_active:", robot.robot_state_pkg.ft_sensor_active)
    print("EmergencyStop:", robot.robot_state_pkg.EmergencyStop)
    print("motion_done:", robot.robot_state_pkg.motion_done)
    print("gripper_motiondone:", robot.robot_state_pkg.gripper_motiondone)
    print("mc_queue_len:", robot.robot_state_pkg.mc_queue_len)
    print("collisionState:", robot.robot_state_pkg.collisionState)
    print("trajectory_pnum:", robot.robot_state_pkg.trajectory_pnum)
    print("safety_stop0_state:", robot.robot_state_pkg.safety_stop0_state)
    print("safety_stop1_state:", robot.robot_state_pkg.safety_stop1_state)
    print("gripper_fault_id:", robot.robot_state_pkg.gripper_fault_id)
    print("gripper_fault:", robot.robot_state_pkg.gripper_fault)
    print("gripper_active:", robot.robot_state_pkg.gripper_active)
    print("gripper_position:", robot.robot_state_pkg.gripper_position)
    print("gripper_speed:", robot.robot_state_pkg.gripper_speed)
    print("gripper_current:", robot.robot_state_pkg.gripper_current)
    print("gripper_tmp:", robot.robot_state_pkg.gripper_tmp)
    print("gripper_voltage:", robot.robot_state_pkg.gripper_voltage)
    print("auxState.servoId:", robot.robot_state_pkg.auxState.servoId)
    print("auxState.servoErrCode:", robot.robot_state_pkg.auxState.servoErrCode)
    print("auxState.servoState:", robot.robot_state_pkg.auxState.servoState)
    print("auxState.servoPos:", robot.robot_state_pkg.auxState.servoPos)
    print("auxState.servoVel:", robot.robot_state_pkg.auxState.servoVel)
    print("auxState.servoTorque:", robot.robot_state_pkg.auxState.servoTorque)
    for i in range(4):
        print("extAxisStatus.pos:", i,robot.robot_state_pkg.extAxisStatus[i].pos)
        print("extAxisStatus.vel:", i,robot.robot_state_pkg.extAxisStatus[i].vel)
        print("extAxisStatus.errorCode:", i,robot.robot_state_pkg.extAxisStatus[i].errorCode)
        print("extAxisStatus.ready:", i,robot.robot_state_pkg.extAxisStatus[i].ready)
        print("extAxisStatus.inPos:", i,robot.robot_state_pkg.extAxisStatus[i].inPos)
        print("extAxisStatus.alarm:", i,robot.robot_state_pkg.extAxisStatus[i].alarm)
        print("extAxisStatus.flerr:", i,robot.robot_state_pkg.extAxisStatus[i].flerr)
        print("extAxisStatus.nlimit:", i,robot.robot_state_pkg.extAxisStatus[i].nlimit)
        print("extAxisStatus.pLimit:", i,robot.robot_state_pkg.extAxisStatus[i].pLimit)
        print("extAxisStatus.mdbsOffLine:", i,robot.robot_state_pkg.extAxisStatus[i].mdbsOffLine)
        print("extAxisStatus.mdbsTimeout:", i,robot.robot_state_pkg.extAxisStatus[i].mdbsTimeout)
        print("extAxisStatus.homingStatus:", i,robot.robot_state_pkg.extAxisStatus[i].homingStatus)
    for i in range(8):
        print("extDIState:",i, robot.robot_state_pkg.extDIState[i])
        print("extDOState:", i,robot.robot_state_pkg.extDOState[i])
    for i in range(4):
        print("extAIState:", i,robot.robot_state_pkg.extAIState[i])
        print("extAOState:", robot.robot_state_pkg.extAOState[i])
    print("rbtEnableState:", robot.robot_state_pkg.rbtEnableState)
    print("jointDriverTorque0:", robot.robot_state_pkg.jointDriverTorque[0])
    print("jointDriverTorque1:", robot.robot_state_pkg.jointDriverTorque[1])
    print("jointDriverTorque2:", robot.robot_state_pkg.jointDriverTorque[2])
    print("jointDriverTorque3:", robot.robot_state_pkg.jointDriverTorque[3])
    print("jointDriverTorque4:", robot.robot_state_pkg.jointDriverTorque[4])
    print("jointDriverTorque5:", robot.robot_state_pkg.jointDriverTorque[5])
    print("jointDriverTemperature:", robot.robot_state_pkg.jointDriverTemperature[0])
    print("jointDriverTemperature:", robot.robot_state_pkg.jointDriverTemperature[1])
    print("jointDriverTemperature:", robot.robot_state_pkg.jointDriverTemperature[2])
    print("jointDriverTemperature:", robot.robot_state_pkg.jointDriverTemperature[3])
    print("jointDriverTemperature:", robot.robot_state_pkg.jointDriverTemperature[4])
    print("jointDriverTemperature:", robot.robot_state_pkg.jointDriverTemperature[5])
    print("year:", robot.robot_state_pkg.year)
    print("mouth:", robot.robot_state_pkg.mouth)
    print("day:", robot.robot_state_pkg.day)
    print("hour:", robot.robot_state_pkg.hour)
    print("minute:", robot.robot_state_pkg.minute)
    print("second:", robot.robot_state_pkg.second)
    print("millisecond:", robot.robot_state_pkg.millisecond)
    print("softwareUpgradeState:", robot.robot_state_pkg.softwareUpgradeState)
    print("endLuaErrCode:", robot.robot_state_pkg.endLuaErrCode)