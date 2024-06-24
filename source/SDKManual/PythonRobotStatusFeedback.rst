Status feedback information
================================

.. toctree:: 
    :maxdepth: 5

Status feedback information comparison table
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. versionadded:: python SDK-v2.0.4
    
.. csv-table:: 
    :header-rows: 1
    :name: Status feedback information comparison table
    :widths: 20 30

    "Variable", "Meaning"
    "program_state", "Program running status, 1-stop; 2-run; 3-suspend"
    "robot_state", "Robot motion status, 1-stop; 2-run; 3- suspend; 4-drag"
    "main_code", "Main error code"
    "sub_code", Sub-error code"
    "robot_mode", "Robot mode, 0-automatic mode; 1-manual mode"
    "jt_cur_pos[i]", "Current position of joint, unit deg, i: 0~5"
    "tl_cur_pos[i]", "Current position of tool, unit deg&mm, i: 0~5"
    "flange_cur_pos[i]", "Current position of end flange, unit deg&mm, i: 0~5"
    "actual_qd[i]", "Current joint speed of robot, unit deg/s^2, i: 0~5"
    "actual_qdd[i]", "Current joint acceleration of robot, unit mm/s, i: 0~5"
    "target_TCP_CmpSpeed[i]","Robot TCP synthetic command speed, unit mm/s&deg/s,i:0~1"
    "target_TCP_Speed[i]","Robot TCP command speed, unit mm/s&deg/s,i:0~5"
    "actual_TCP_CmpSpeed[i]","Robot TCP synthetic actual speed, unit mm/s&deg/s,i:0~1"
    "actual_TCP_Speed[i]","Robot TCP actual speed, unit mm/s&deg/s,i:0~5"
    "jt_cur_tor[i]","Current torque, unit N·m,i:0~5"
    "tool","Applied tool coordinate system number"
    "user","Applied workpiece coordinate system number"
    "cl_dgt_output_h","Control box digital IO output 15-8"
    "cl_dgt_output_l","Control box digital IO output 7-0"
    "tl_dgt_output_l","Tool digital IO output 7-0, only bit0-bit1 is valid"
    "dgt_input_h","Control box digital IO input 15-8"
    "cl_dgt_input_l","Control box digital IO input 7-0"
    "tl_dgt_input_l","Tool digital IO input 7-0, only bit0-bit1 is valid"
    "cl_analog_input[i]","Control box analog input, i:0~2"
    "tl_anglog_input","Tool analog input"
    "ft_sensor_raw_data","Torque sensor raw data, unit N&Nm, i:0~5"
    "ft_sensor_data","Torque sensor data, unit N&Nm, i:0~5"
    "ft_sensor_active","Torque sensor activation status, 0-reset, 1-activate"
    "EmergencyStop", "Emergency stop flag, 0-Emergency stop not pressed, 1-Emergency stop pressed"
    "motion_done", "Motion in place signal, 1-in place, 0-not in place"
    "gripper_motiondone", "Gripper movement completion signal, 1-completed, 0-not completed"
    "mc_queue_len", "Motion instruction queue length"
    "collisionState", "Collision detection, 1-collision, 0-no collision"
    "trajectory_pnum", "Trajectory point number"
    "safety_stop0_state", "Safety stop signal SI0"
    "safety_stop1_state", "Safety stop signal SI1"
    "gripper_fault_id", "Error gripper number"
    "gripper_fault", "Gripper fault"
    "gripper_active", "Gripper activation state, 0-not activated, 1-activated"
    "gripper_position", "Gripper position (percentage)"
    "gripper_speed", "Gripper speed (percentage)"
    "gripper_current","gripper current (percentage)"
    "gripper_tmp","gripper temperature, unit ℃"
    "gripper_voltage","gripper voltage, unit V"
    "auxState.servoId","485 extension axis, servo drive ID number, i:0~3"
    "auxState.servoErrCode","485 extension axis, servo drive fault code, i:0~3"
    "auxState.servoState","485 extension axis, servo drive state, i:0~3"
    "auxState.servoPos","485 extension axis, servo current position, i:0~3"
    "auxState.servoVel","485 extension axis, servo current speed, i:0~3"
    "auxState.servoTorque","485 extension axis, servo current torque, i:0~3"
    "extAxisStatus[i].pos","UDP extension axis, position, i:0~3"
    "extAxisStatus[i].vel","UDP extended axis, speed, i:0~3"
    "extAxisStatus[i].errorCode","UDP extended axis, fault code, i:0~3"
    "extAxisStatus[i].ready","UDP extended axis, servo ready, i:0~3"
    "extAxisStatus[i].inPos","UDP extended axis, servo in place, i:0~3"
    "extAxisStatus[i].alarm","UDP extended axis, servo alarm, i:0~3"
    "extAxisStatus[i].flerr","UDP extended axis, following error, i:0~3"
    "extAxisStatus[i].nlimit","UDP extended axis, to negative limit, i:0~3"
    "extAxisStatus[i].pLimit","UDP extended axis, to positive limit, i:0~3"
    "extAxisStatus[i].mdbsOffLine", "UDP extension axis, driver 485 bus offline"
    "extAxisStatus[i].mdbsTimeout", "UDP extension axis, control card and control box 485 communication timeout"
    "extAxisStatus[i].homingStatus", "UDP extension axis, homing status"
    "extDIState", "extended digital input status"
    "extDOState", "extended digital output status"
    "extAIState", "extended analog input status"
    "extAOState", "extended analog output status"
    
Code example
---------------

.. code-block:: python
    :linenos:

    from fairino import Robot
    # A connection is established with the robot controller. A successful connection returns a robot object
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