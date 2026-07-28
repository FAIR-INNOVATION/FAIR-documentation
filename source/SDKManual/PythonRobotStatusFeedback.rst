Data structure description
==============================================================================

.. toctree::
    :maxdepth: 5

Robot Status Feedback Structure Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python
    :linenos:
    
    class ROBOT_AUX_STATE(Structure):
        _pack_ = 1
        _fields_ = [
            ("servoId", c_uint8),         # Servo driver ID number
            ("servoErrCode", c_int),     # Servo driver fault code
            ("servoState", c_int),       # Servo driver status
            ("servoPos", c_double),      # Servo current position
            ("servoVel", c_float),       # Servo current speed
            ("servoTorque", c_float),    # Servo current torque
        ]

    class EXT_AXIS_STATUS(Structure):
        _pack_ = 1
        _fields_ = [
            ("pos", c_double),        # Extension axis position
            ("vel", c_double),        # Extension axis speed
            ("errorCode", c_int),     # Extension axis fault code
            ("ready", c_uint8),        # Servo ready
            ("inPos", c_uint8),        # Servo in position
            ("alarm", c_uint8),        # Servo alarm
            ("flerr", c_uint8),        # Following error
            ("nlimit", c_uint8),       # Negative limit reached
            ("pLimit", c_uint8),       # Positive limit reached
            ("mdbsOffLine", c_uint8),  # Driver 485 bus offline
            ("mdbsTimeout", c_uint8),  # Control card to control box 485 communication timeout
            ("homingStatus", c_uint8), # Extension axis homing status
        ]

    class WELDING_BREAKOFF_STATE(Structure):
        _pack_ = 1
        _fields_ = [
            ("breakOffState", c_uint8),        # Welding interruption status
            ("weldArcState", c_uint8),        # Welding arc interruption status
        ]

    # ==================== Complete Robot State Structure ====================
    class RobotStatePkg(Structure):
        """
        Robot status feedback data packet
        """
        _pack_ = 1
        _fields_ = [
            # Frame header information
            ("frame_head", c_uint16),           # Frame header, preset to 0x5A5A
            ("frame_cnt", c_uint8),             # Frame count, cyclic count 0-255
            ("data_len", c_uint16),             # Length of data content
            ("program_state", c_uint8),         # Program running status, 1-stopped; 2-running; 3-paused
            ("robot_state", c_uint8),             # Robot motion status, 1-stopped; 2-running; 3-paused; 4-dragging
            ("main_code", c_int),               # Main fault code
            ("sub_code", c_int),                # Sub fault code
            ("robot_mode", c_uint8),            # Robot mode, 1-manual mode; 0-automatic mode

            # Joint positions and velocities
            ("jt_cur_pos", c_double * 6),       # Current joint positions of 6 axes, unit deg
            ("tl_cur_pos", c_double * 6),       # Current tool position [x,y,z,rx,ry,rz]
            ("flange_cur_pos", c_double * 6),   # Current end flange position [x,y,z,rx,ry,rz]
            ("actual_qd", c_double * 6),        # Current velocities of 6 joints, unit deg/s
            ("actual_qdd", c_double * 6),       # Current accelerations of 6 joints, unit deg/s^2
            ("target_TCP_CmpSpeed", c_double * 2),  # TCP composite command speed [position mm/s, orientation deg/s]
            ("target_TCP_Speed", c_double * 6), # TCP command speed [x,y,z,rx,ry,rz]
            ("actual_TCP_CmpSpeed", c_double * 2),  # TCP composite actual speed [position mm/s, orientation deg/s]
            ("actual_TCP_Speed", c_double * 6), # TCP actual speed [x,y,z,rx,ry,rz]
            ("jt_cur_tor", c_double * 6),       # Current torques of 6 axes, unit N·m

            # Tool and user coordinate systems
            ("tool", c_int),                    # Applied tool coordinate system number
            ("user", c_int),                    # Applied workpiece coordinate system number

            # Digital I/O
            ("cl_dgt_output_h", c_uint8),       # Control box digital IO output 15-8
            ("cl_dgt_output_l", c_uint8),       # Control box digital IO output 7-0
            ("tl_dgt_output_l", c_uint8),       # Tool digital IO output 7-0, only bit0-bit1 valid
            ("cl_dgt_input_h", c_uint8),        # Control box digital IO input 15-8
            ("cl_dgt_input_l", c_uint8),        # Control box digital IO input 7-0
            ("tl_dgt_input_l", c_uint8),        # Tool digital IO input 7-0, only bit0-bit1 valid

            # Analog I/O
            ("cl_analog_input", c_uint16 * 2),  # Control box analog input [0],[1]
            ("tl_anglog_input", c_uint16),      # Tool analog input

            # Force/torque sensor
            ("ft_sensor_raw_data", c_double * 6),   # Force/torque sensor raw data
            ("ft_sensor_data", c_double * 6),      # Force/torque sensor data
            ("ft_sensor_active", c_uint8),          # Force/torque sensor activation status

            # Status signals
            ("EmergencyStop", c_uint8),         # Emergency stop flag, 0-not pressed, 1-pressed
            ("motion_done", c_int),             # Motion completion signal, 1-completed, 0-not completed
            ("gripper_motiondone", c_uint8),    # Gripper motion complete signal, 0-not complete, 1-complete (no object detected), 2-motion complete (object detected)
            ("mc_queue_len", c_int),            # Motion command queue length
            ("collisionState", c_uint8),        # Collision detection, 1-collision, 0-no collision
            ("trajectory_pnum", c_int),         # Trajectory point number
            ("safety_stop0_state", c_uint8),    # Safety stop signal SI0
            ("safety_stop1_state", c_uint8),    # Safety stop signal SI1

            # Gripper information
            ("gripper_fault_id", c_uint8),      # Faulty gripper number
            ("gripper_fault", c_uint16),        # Gripper fault 0-no fault 1-485 timeout 2-command error 3-workpiece dropped Other-gripper fault code
            ("gripper_active", c_uint16),      # Gripper activation status
            ("gripper_position", c_uint8),      # Gripper position
            ("gripper_speed", c_int8),          # Gripper speed
            ("gripper_current", c_int8),        # Gripper current
            ("gripper_temp", c_int),            # Gripper temperature
            ("gripper_voltage", c_int),         # Gripper voltage

            # Extension axis status
            ("aux_axis_state", ROBOT_AUX_STATE * 25),    # 485 extension axis status (25)
            ("extAxisStatus", EXT_AXIS_STATUS * 4), # UDP extension axis status (4)

            # Extension I/O status
            ("extDIState", c_uint16 * 8),       # Extension DI input
            ("extDOState", c_uint16 * 8),       # Extension DO output
            ("extAIState", c_uint16 * 4),        # Extension AI input
            ("extAOState", c_uint16 * 4),        # Extension AO output

            # Robot and joint status
            ("rbtEnableState", c_int),                  # Robot enable status
            ("jointDriverTorque", c_double * 6),        # Robot joint driver torque
            ("jointDriverTemperature", c_double * 6),   # Robot joint driver temperature

            # Robot time
            #("robotTime", c_int * 7),             # Robot system time [year,month,day,hour,min,sec,ms]
            ("year", ctypes.c_uint16),  # Year
            ("mouth", ctypes.c_uint8),  # Month
            ("day", ctypes.c_uint8),   # Day
            ("hour", ctypes.c_uint8),   # Hour
            ("minute", ctypes.c_uint8), # Minute
            ("second", ctypes.c_uint8), # Second
            ("millisecond", ctypes.c_uint16), # Millisecond

            ("softwareUpgradeState", c_int),      # Robot software upgrade status
            ("endLuaErrCode", c_uint16),          # End Lua running status

            # Analog output
            ("cl_analog_output", c_uint16 * 2), # Control box analog output [0],[1]
            ("tl_analog_output", c_uint16),       # Tool analog output

            # Rotating gripper
            ("gripperRotNum", c_float),         # Current rotation count of rotating gripper
            ("gripperRotSpeed", c_uint8),       # Current rotation speed percentage of rotating gripper
            ("gripperRotTorque", c_uint8),      # Current rotation torque percentage of rotating gripper

            # Welding interruption status - using structure
            ("weldingBreakOffState", WELDING_BREAKOFF_STATE),  # Welding interruption status

            # Target joint torque
            ("jt_tgt_tor", c_double * 6),       # Joint command torque

            ("smartToolState", c_int),          # SmartTool handle button status
            ("wideVoltageCtrlBoxTemp", c_float),        # Wide voltage control box temperature
            ("wideVoltageCtrlBoxFanCurrent", c_uint16), # Wide voltage control box fan current (mA)

            # Coordinate system values
            ("toolCoord", c_double * 6),        # Current tool coordinate system values; x,y,z,rx,ry,rz
            ("wobjCoord", c_double * 6),        # Current workpiece coordinate system values; x,y,z,rx,ry,rz
            ("extoolCoord", c_double * 6),      # Current external tool coordinate system values; x,y,z,rx,ry,rz
            ("exAxisCoord", c_double * 6),      # Current extension axis coordinate system values; x,y,z,rx,ry,rz

            # Load
            ("load", c_double),                 # Load mass
            ("loadCog", c_double * 3),            # Load center of gravity

            # Servo commands
            ("lastServoTarget", c_double * 6),  # Last ServoJ target position in the queue
            ("servoJCmdNum", c_int),            # ServoJ command count

            # Target joint data
            ("targetJointPos", c_double * 6),   # 6 joints command position, unit °
            ("targetJointVel", c_double * 6),   # 6 joints command velocity, unit °/s
            ("targetJointAcc", c_double * 6),   # 6 joints command acceleration, unit °/s2
            ("targetJointCurrent", c_double * 6), # 6 joints command current, unit A
            ("actualJointCurrent", c_double * 6), # 6 joints current current, unit A
            ("actualTCPForce", c_double * 6),   # Robot end-effector torque Nm; x,y,z,rx,ry,rz
            ("targetTCPPos", c_double * 6),     # Robot TCP command position mm; x,y,z,rx,ry,rz

            ("collisionLevel", c_uint8 * 6),    # Robot collision level
            ("speedScaleManual", c_double),     # Manual mode global speed percentage
            ("speedScaleAuto", c_double),       # Automatic mode global speed percentage
            ("luaLineNum", c_int),              # Current Lua program running line number
            ("abnomalStop", c_uint8),           # 0-no abnormality; 1-abnormality present
            ("currentLuaFileName", c_uint8 * 256),  # Name of currently running Lua program
            ("programTotalLine", c_uint8),      # Total lines of Lua program
            ("safetyBoxSingal", c_uint8 * 6),   # Robot button box button status

            # Welding data
            ("weldVoltage", c_double),          # Welding voltage V
            ("weldCurrent", c_double),          # Welding current
            ("weldTrackVel", c_double),         # Seam tracking speed mm/s

            ("tpdException", c_uint8),            # TPD trajectory load count exceeded, 0-not exceeded, 1-exceeded
            ("alarmRebootRobot", c_uint8),      # Warning, 1-release emergency stop button and power cycle the control box, 2-joint communication abnormality, power cycle the control box
            ("modbusMasterConnect", c_uint8),   # bit0-bit7 correspond to ModbusTCP master 0-7 connection status
            ("modbusSlaveConnect", c_uint8),    # ModbusTCP slave connection status
            ("btnBoxStopSignal", c_uint8),      # Button box emergency stop signal
            ("dragAlarm", c_uint8),             # Drag warning
            ("safetyDoorAlarm", c_uint8),       # Safety door warning
            ("safetyPlaneAlarm", c_uint8),      # Entering safety wall warning
            ("motonAlarm", c_uint8),            # Motion warning
            ("interfaceAlarm", c_uint8),        # Entering interference area warning
            ("udpCmdState", c_int),             # Port 20007 UDP communication connection status
            ("weldReadyState", c_uint8),        # Welder ready status
            ("alarmCheckEmergStopBtn", c_uint8),    # 0-normal; 1-communication abnormality, check if emergency stop button is released
            ("tsTmCmdComError", c_uint8),       # 0-normal; 1-torque command communication failure
            ("tsTmStateComError", c_uint8),     # 0-normal; 1-torque status communication failure
            ("ctrlBoxError", c_int),            # Control box error
            ("safetyDataState", c_uint8),       # Safety data status flag
            ("forceSensorErrState", c_uint8),   # Force sensor connection timeout fault
            ("ctrlOpenLuaErrCode", c_uint8 * 4),  # 4 controller peripheral protocol error codes
            ("strangePosFlag", c_uint8),        # Currently in singular posture flag
            ("alarm", c_uint8),                 # Warning
            ("driverAlarm", c_uint8),           # Driver alarm axis number
            ("aliveSlaveNumError", c_uint8),    # Active slave count error
            ("slaveComError", c_uint8 * 8),     # Slave error status
            ("cmdPointError", c_uint8),         # Command point error
            ("IOError", c_uint8),               # IO error
            ("gripperError", c_uint8),          # Gripper error
            ("fileError", c_uint8),             # File error
            ("paraError", c_uint8),             # Parameter error
            ("exaxisOutLimitError", c_uint8),   # External axis soft limit exceeded error
            ("driverComError", c_uint8 * 6),    # Driver communication fault
            ("driverError", c_uint8),           # Driver communication fault axis number
            ("outSoftLimitError", c_uint8),     # Soft limit exceeded fault
            ("axleGenComData", c_uint8 * 130),   # Axle general communication non-periodic data
            ("socketConnTimeout", c_uint8),     # Socket connection timeout
            ("socketReadTimeout", c_uint8),     # Socket read timeout
            ("tsWebStateComErr", c_uint8),      # TS_WEB status communication error
            ("exaxisCoordID", c_uint8),         # External extended axis ID
            ("programRunState", c_uint8),         # LUA program running status 0-program not running; 1-program running (including program paused)
            ("check_sum", c_uint16)          # Checksum
        ]

Controller status feedback data packet
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. versionadded:: python SDK-v2.1.7
    
.. csv-table:: 
    :header-rows: 1
    :name: Controller status feedback data packet
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
    "actual_qd[i]", "Robot's current joint velocity in deg/s,i:0~5"
    "actual_qdd[i]", "Current joint acceleration of the robot, in deg/s^2, i:0~5"
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
    "gripper_motiondone", "Gripper motion complete signal, 0-not complete, 1-complete (no object detected), 2-motion complete (object detected)"
    "mc_queue_len", "motion command queue length"
    "collisionState", "Collision detection, 1-collision, 0-no collision "
    "trajectory_pnum", "trajectory point number"
    "safety_stop0_state", "safety stop signal SI0"
    "safety_stop1_state", "safety stop signal SI1"
    "gripper_fault_id", "error_claw_number"
    "gripper_fault", "Gripper fault 0-no fault 1-485 timeout 2-command error 3-workpiece dropped Other-gripper fault code"
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
    "cl_analog_output[i]","Control box analog output,i:0~1"
    "tl_analog_output","Tool analog output"
    "gripperRotNum","Rotation gripper current rotation number"
    "gripperRotSpeed","Rotation gripper current rotation speed percentage"
    "gripperRotTorque","Rotation gripper current rotation torque percentage"
    "weldingBreakOffState","Welding interruption state"
    "jt_tgt_tor","Joint command torque"
    "smartToolState","The status of the SmartTool handle buttons"
    "wideVoltageCtrlBoxTemp","Wide voltage control box temperature"
    "wideVoltageCtrlBoxFanCurrent","Wide voltage control box fan current(ma)"
    "toolCoord[i]","Tool coordinate system,i:0~5"
    "wobjCoord[i]","Workpiece coordinate system,i:0~5"
    "extoolCoord[i]","External tool coordinate system,i:0~5"
    "exAxisCoord[i]","Extended axis coordinate system,i:0~5"
    "load","Quality of load"
    "loadCog[i]","Center of mass of load,i:0~2"
    "lastServoTarget[i]","The last ServoJ target position in the queue,i:0 to 5"
    "servoJCmdNum","ServoJ instruction count"

Status of the servo controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. versionadded:: python SDK-v2.1.3
    
.. csv-table:: 
    :header-rows: 1
    :name: Status of the servo controller
    :widths: 20 30

    "Variable", "Meaning"
    "servoId","Servo driver ID number"
    "servoErrCode","Servo driver fault code"
    "servoState","Status of the servo driver"
    "servoPos","Current position of the servo"
    "servoVel","Current speed of the servo"
    "servoTorque","Current torque of the servo"

Extended axis status
~~~~~~~~~~~~~~~~~~~~~~~~
.. versionadded:: python SDK-v2.1.3
    
.. csv-table:: 
    :header-rows: 1
    :name: Extended axis status
    :widths: 20 30

    "Variable", "Meaning"
    "pos","Extended axis position"
    "vel","Extended axis speed"
    "errorCode","Extended shaft fault code"
    "ready","The servo is ready"
    "inPos","Servo in place"
    "alarm","Servo alarm"
    "flerr","Following error"
    "nlimit","To the negative limit"
    "pLimit","To the positive limit position"
    "mdbsOffLine","The 485 bus of the driver is disconnected"
    "mdbsTimeout","The 485 communication between the control card and the control box has timed out"
    "homingStatus","The expansion axis returns to the zero state"

Welding interruption state
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. versionadded:: python SDK-v2.1.3
    
.. csv-table:: 
    :header-rows: 1
    :name: Welding interruption state
    :widths: 20 30

    "Variable", "Meaning"
    "breakOffState","Welding interruption state"
    "weldArcState","Welding arc interruption state"

    
code example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
    print("cl_analog_output[0]:", robot.robot_state_pkg.cl_analog_output[0])
    print("cl_analog_output[1]:", robot.robot_state_pkg.cl_analog_output[1])
    print("tl_analog_output:", robot.robot_state_pkg.tl_analog_output)
    print("gripperRotNum:", robot.robot_state_pkg.gripperRotNum)
    print("gripperRotSpeed:", robot.robot_state_pkg.gripperRotSpeed)
    print("gripperRotTorque:", robot.robot_state_pkg.gripperRotTorque)
    print("jt_tgt_tor:", robot.robot_state_pkg.jt_tgt_tor)
    print("smartToolState:", robot.robot_state_pkg.smartToolState)
    print("wideVoltageCtrlBoxTemp:", robot.robot_state_pkg.wideVoltageCtrlBoxTemp)
    print("wideVoltageCtrlBoxFanCurrent:", robot.robot_state_pkg.wideVoltageCtrlBoxFanCurrent)
    print("toolCoord0:", robot.robot_state_pkg.toolCoord[0])
    print("toolCoord1:", robot.robot_state_pkg.toolCoord[1])
    print("toolCoord2:", robot.robot_state_pkg.toolCoord[2])
    print("toolCoord3:", robot.robot_state_pkg.toolCoord[3])
    print("toolCoord4:", robot.robot_state_pkg.toolCoord[4])
    print("toolCoord5:", robot.robot_state_pkg.toolCoord[5])
    print("wobjCoord0:", robot.robot_state_pkg.wobjCoord[0])
    print("wobjCoord1:", robot.robot_state_pkg.wobjCoord[1])
    print("wobjCoord2:", robot.robot_state_pkg.wobjCoord[2])
    print("wobjCoord3:", robot.robot_state_pkg.wobjCoord[3])
    print("wobjCoord4:", robot.robot_state_pkg.wobjCoord[4])
    print("wobjCoord5:", robot.robot_state_pkg.wobjCoord[5])
    print("extoolCoord0:", robot.robot_state_pkg.extoolCoord[0])
    print("extoolCoord1:", robot.robot_state_pkg.extoolCoord[1])
    print("extoolCoord2:", robot.robot_state_pkg.extoolCoord[2])
    print("extoolCoord3:", robot.robot_state_pkg.extoolCoord[3])
    print("extoolCoord4:", robot.robot_state_pkg.extoolCoord[4])
    print("extoolCoord5:", robot.robot_state_pkg.extoolCoord[5])
    print("exAxisCoord0:", robot.robot_state_pkg.exAxisCoord[0])
    print("exAxisCoord1:", robot.robot_state_pkg.exAxisCoord[1])
    print("exAxisCoord2:", robot.robot_state_pkg.exAxisCoord[2])
    print("exAxisCoord3:", robot.robot_state_pkg.exAxisCoord[3])
    print("exAxisCoord4:", robot.robot_state_pkg.exAxisCoord[4])
    print("exAxisCoord5:", robot.robot_state_pkg.exAxisCoord[5])
    print("load:", robot.robot_state_pkg.load)
    print("loadCog0:", robot.robot_state_pkg.loadCog[0])
    print("loadCog1:", robot.robot_state_pkg.loadCog[1])
    print("loadCog2:", robot.robot_state_pkg.loadCog[2])
    print("lastServoTarget0:", robot.robot_state_pkg.lastServoTarget[0])
    print("lastServoTarget1:", robot.robot_state_pkg.lastServoTarget[1])
    print("lastServoTarget2:", robot.robot_state_pkg.lastServoTarget[2])
    print("lastServoTarget3:", robot.robot_state_pkg.lastServoTarget[3])
    print("lastServoTarget4:", robot.robot_state_pkg.lastServoTarget[4])
    print("lastServoTarget5:", robot.robot_state_pkg.lastServoTarget[5])
    print("servoJCmdNum:", robot.robot_state_pkg.servoJCmdNum)

Robot Status Feedback Configuration List Enumeration Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python
    :linenos:

    # ==================== RobotState Configuration List Enumeration ====================
    class RobotState(enum.Enum):
        """CNDE status type enumeration"""
        FrameHead = 0
        FrameCnt = 1
        DataLen = 2
        ProgramState = 3
        RobotState = 4
        MainCode = 5
        SubCode = 6
        RobotMode = 7
        JointCurPos = 8
        ToolCurPos = 9
        FlangeCurPos = 10
        ActualJointVel = 11
        ActualJointAcc = 12
        TargetTCPCmpSpeed = 13
        TargetTCPSpeed = 14
        ActualTCPCmpSpeed = 15
        ActualTCPSpeed = 16
        ActualJointTorque = 17
        Tool = 18
        User = 19
        ClDgtOutputH = 20
        ClDgtOutputL = 21
        TlDgtOutputL = 22
        ClDgtInputH = 23
        ClDgtInputL = 24
        TlDgtInputL = 25
        ClAnalogInput = 26
        TlAnglogInput = 27
        FtSensorRawData = 28
        FtSensorData = 29
        FtSensorActive = 30
        EmergencyStop = 31
        MotionDone = 32
        GripperMotiondone = 33
        McQueueLen = 34
        CollisionState = 35
        TrajectoryPnum = 36
        SafetyStop0State = 37
        SafetyStop1State = 38
        GripperFaultId = 39
        GripperFault = 40
        GripperActive = 41
        GripperPosition = 42
        GripperSpeed = 43
        GripperCurrent = 44
        GripperTemp = 45
        GripperVoltage = 46
        AuxState = 47
        ExtAxisStatus = 48
        ExtDIState = 49
        ExtDOState = 50
        ExtAIState = 51
        ExtAOState = 52
        RbtEnableState = 53
        JointDriverTorque = 54
        JointDriverTemperature = 55
        RobotTime = 56
        SoftwareUpgradeState = 57
        EndLuaErrCode = 58
        ClAnalogOutput = 59
        TlAnalogOutput = 60
        GripperRotNum = 61
        GripperRotSpeed = 62
        GripperRotTorque = 63
        WeldingBreakOffState = 64
        TargetJointTorque = 65
        SmartToolState = 66
        WideVoltageCtrlBoxTemp = 67
        WideVoltageCtrlBoxFanCurrent = 68
        ToolCoord = 69
        WobjCoord = 70
        ExtoolCoord = 71
        ExAxisCoord = 72
        Load = 73
        LoadCog = 74
        LastServoTarget = 75
        ServoJCmdNum = 76
        TargetJointPos = 77
        TargetJointVel = 78
        TargetJointAcc = 79
        TargetJointCurrent = 80
        ActualJointCurrent = 81
        ActualTCPForce = 82
        TargetTCPPos = 83
        CollisionLevel = 84
        SpeedScaleManual = 85
        SpeedScaleAuto = 86
        LuaLineNum = 87
        AbnomalStop = 88
        CurrentLuaFileName = 89
        ProgramTotalLine = 90
        SafetyBoxSingal = 91
        WeldVoltage = 92
        WeldCurrent = 93
        WeldTrackVel = 94
        TpdException = 95
        AlarmRebootRobot = 96
        ModbusMasterConnect = 97
        ModbusSlaveConnect = 98
        BtnBoxStopSignal = 99
        DragAlarm = 100
        SafetyDoorAlarm = 101
        SafetyPlaneAlarm = 102
        MotonAlarm = 103
        InterfaceAlarm = 104
        UdpCmdState = 105
        WeldReadyState = 106
        AlarmCheckEmergStopBtn = 107
        TsTmCmdComError = 108
        TsTmStateComError = 109
        CtrlBoxError = 110
        SafetyDataState = 111
        ForceSensorErrState = 112
        CtrlOpenLuaErrCode = 113
        StrangePosFlag = 114
        Alarm = 115
        DriverAlarm = 116
        AliveSlaveNumError = 117
        SlaveComError = 118
        CmdPointError = 119
        IOError = 120
        GripperError = 121
        FileError = 122
        ParaError = 123
        ExaxisOutLimitError = 124
        DriverComError = 125
        DriverError = 126
        OutSoftLimitError = 127
        AxleGenComData = 128
        SocketConnTimeout = 129
        SocketReadTimeout = 130
        TsWebStateComErr = 131
        exaxisCoordID = 132
        programRunState = 133
        check_sum = 134