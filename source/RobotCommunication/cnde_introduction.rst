Introduction to CNDE
=======================

Collaborative robot configurable network data exchange protocol (CNDE) is a way for the client to control the robot and obtain the feedback status of the robot through UDP communication.

Table 1-1 shows all the states of the robot that can be obtained by CNDE. The client can arbitrarily select several required states from the table and make the robot perform state feedback according to the set feedback period.

Similarly, the client can also select the required combination of robot control functions from Table 1-2 for robot control operation. The communication data between the client and the robot CNDE should be in the specified frame format, and the communication port of the robot CNDE is 20006.

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

   * - output_BIT_reg_8xX
     - UINT8_X
     - BIT-type robot output registers (8xX indicates the number of registers, if you need 16 BIT-type output registers, the actual name is "output_BIT_reg_8x2", and the robot can support up to 128 bit-type output registers)

   * - output_INT_reg_X
     - INT32_X
     - INT robot output registers (X represents the number of registers. If you need 16 INT output registers, the actual name is "output_INT_reg_16", and the robot can support up to 64 INT output registers)

   * - output_DOUBLE_reg_X
     - DOUBLE_X
     - DOUBLE robot output register (X represents the number of registers, if you need 16 DOUBLE output registers, the actual name is "output_DOUBLE_reg_16", and the robot can support up to 64 DOUBLE output registers)

   * - ft_sensor_data
     - DOUBLE_6
     - Force sensor data

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