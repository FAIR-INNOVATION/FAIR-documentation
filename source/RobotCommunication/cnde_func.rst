
CNDE function operation
===========================

Input configuration and input data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The client sends data frames to the robot through CNDE to control the robot's DO, AO output, input registers, etc. Before sending the input data, it is necessary to configure the functional content to be controlled. Table 2-1 shows the format of CNDE input configuration content, including recipe number and a series of input configuration function names (Table 1-2); The corresponding table 3-2 is the content format of input data, including recipe number and input data byte group.

CNDE data input supports up to 8 recipes. When sending input data, the robot will match the recipe number in the received data to the corresponding recipe configuration function name group, and analyze the data to get the input data value of each function name, and then control the robot according to the input data.

.. centered:: Table 3-1 Input Configuration Content Format

.. list-table::
   :widths: 40 20 40
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Name**
     - **Recipe number**
     - **Function name string**

   * - Length (byte)
     - 1
     - --

   * - Content
     - 0 ~ 7
     - A series of input data function names

.. centered:: Table 3-2 Input Data Content Format

.. list-table::
   :widths: 40 20 40
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Name**
     - **Recipe number**
     - **Data byte group**

   * - Length(byte)
     - 1
     - --

   * - Content
     - 0 ~ 7
     - Input data content

When inputting configuration, the robot controller will check each name after receiving the configuration name group. If the configured function names are correct, the robot will feed back the data type names of all configured functions divided by ","; If the configured function name is wrong, the robot will feed back the corresponding error content. An example of the input configuration data frame (hexadecimal) is as follows:

.. image:: cnde/001.png
   :width: 6in
   :align: center

Among them, the total length of the configured input function name group is 54 bytes, plus 1 byte of the input recipe number, which is 55 bytes, which is converted into hexadecimal as 0x0037. In the Little-Endian mode, the data length in the corresponding input data frame is "37 00".

At this time, the robot will feed back a data frame with the type of messages (Message in section 3.3.1 of this article):

.. image:: cnde/002.png
   :width: 6in
   :align: center

The message type "00" indicates that this is a successful feedback message. The client can extract the "Input Data Configuration Type" and compare it with Table 1-3 to get the byte length of the input configuration. In this example, the total data length is 1*5+4*30+8*30 = 365 bytes.

If the configuration name is entered incorrectly:

.. image:: cnde/003.png
   :width: 6in
   :align: center

The corresponding feedback information is:

.. image:: cnde/004.png
   :width: 6in
   :align: center

The input data can be input circularly according to a certain period, or only when necessary. The fastest period that the robot can process during cyclic input is 1ms, but the faster input period will bring some resource expenses to the robot system. It is suggested that you set the data input period reasonably according to the actual situation. 

In addition, when sending a data frame to the robot, the robot will not have feedback information unless the length of the sent data frame or the data is abnormal. An example of an input data frame is as follows, in which the input data recipe number and the input data byte length should be consistent with the input configuration:

.. image:: cnde/005.png
   :width: 6in
   :align: center

Output configuration and output data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The client can customize the content and period of the state feedback through CNDE. Using the state feedback of the robot CNDE requires the following three steps: ①output configuration; ②Start the output; ③Receiving output data.

Output configuration
+++++++++++++++++++++++

The output configuration frame includes the output period and the output function name group (see Table 1-1 for all configurable names). The configurable range of the output period is 1 ~ 200ms, and the maximum number of output data bytes supports 4096byte. The output function name group is a series of output function name strings separated by ",".After the client sends the output configuration frame, the robot will check the configured function names. If the configured function names are all supported by the current robot CNDE, the robot will feed back a series of data type combinations separated by ","; Otherwise, if the verification of the output configuration name fails, the corresponding error message is fed back.

.. centered:: Table 3-3 Output Configuration Contents

.. list-table::
   :widths: 40 20 40
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Nmae**
     - **Output period (ms)**
     - **Function name string**

   * - Length (byte)
     - 2
     - --

   * - 内容
     - 1-200
     - Output function name group

If the output configuration frame is as follows:

.. image:: cnde/006.png
   :width: 6in
   :align: center

Among them, the total length of the configured output function name group is 48 bytes, plus 2 bytes of the output period, totaling 50 bytes, which is converted into hexadecimal as 0x0032. In the Little-Endian mode, the data length in the corresponding input data frame is "32 00".

At this time, the robot will feed back a data frame with the type of message (Message in section 3.3.1 of this article):

.. image:: cnde/007.png
   :width: 6in
   :align: center

The message type "00" indicates that this is a successful feedback message. The client can extract the "output data configuration type" and compare it with Table 1-3 to get the byte length of the output configuration. In this example, the total data length is 1+8*10+4 = 85 bytes.

If the input configuration name is wrong, such as "queue" wrongly written as "quene":

.. image:: cnde/008.png
   :width: 6in
   :align: center

The corresponding feedback information is:

.. image:: cnde/009.png
   :width: 6in
   :align: center

Output start and stop
+++++++++++++++++++++++++

After the CNDE output configuration of the robot is completed, send a start command to start the CNDE output, and the robot will perform state feedback output according to the configured output period and output content, and also send a CNDE stop output command, and the robot will stop the state feedback output. CNDE start and stop commands have no command content, and the corresponding data length is 0.

.. centered:: Table 3-4 Start and Stop Contents of CNDE Output

.. list-table::
   :widths: 40 40
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Name**
     - **Data byte group**

   * - Length (byte)
     - 0

   * - Content
     - without

An example of starting the robot CNDE to output data frames is as follows:

.. image:: cnde/010.png
   :width: 3in
   :align: center

The client receives the output data
+++++++++++++++++++++++++++++++++++++++

After the CNDE data output of the robot is started, the client needs a cycle to receive the data information fed back by the robot, and the cycle receiving frequency of the client is higher than the configured output data frequency, otherwise data packet loss may occur. The contents of robot output data are shown in Table 3-5. The length of the byte group of robot output data is the sum of the byte lengths of all the function data configured for output, and the byte array is a combination of all the state data in the order of configured functions with 1 byte alignment.

.. centered:: Table 3-5 CNDE Output Data Content

.. list-table::
   :widths: 40 40
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Name**
     - **Data byte group**

   * - Length(byte)
     - --

   * - Content
     - Output data byte group

Examples of robot output data frames are as follows:

.. image:: cnde/011.png
   :width: 4in
   :align: center

CNDE auxiliary function
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Messages
++++++++++++++++++

The client and the robot can send messages to each other through CNDE, including message types and message strings (Table 3-6), where the message types are defined in Table 3-7. When the CNDE client sends commands such as input configuration, output configuration, output start and output stop to the robot, the robot replies with a message. 

If the above commands are executed successfully, the feedback message type of the robot is "successful", and the numerical code of the corresponding message type is 0x00；; On the other hand, if the above commands fail to be executed, the message type fed back by the robot is "error", and the corresponding message type value is 0x03. The client can judge the command execution result according to the feedback message type, and if the message type is "error", the error information can be extracted to analyze the cause of the error.

.. centered:: Table 3-6 Content of Message

.. list-table::
   :widths: 40 20 40
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Name**
     - **Message type**
     - **Message string**

   * - Length(byte)
     - 1
     - --

   * - Content
     - 0 ~ 4
     - Message string

.. centered:: Table 3-7 Robot CNDE Message Types

.. list-table::
   :widths: 40 40
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Type**
     - **Numerical value**

   * - SUCCESS
     - 0x00

   * - INFORMATION
     - 0x01

   * - WARINING
     - 0x02

   * - ERROR
     - 0x03

   * - FAULT
     - 0x04

Switch the version number of CNDE protocol of robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

At present, there is only one version of the robot CNDE, and the version number is "FR-CNDE-V0001", so this function is reserved and has not been put into use yet.

Acquire that version information of the robot soft firmware
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

The client sends an command to get the software firmware version information to the robot through CNDE, and the command content is empty. After receiving the request, the robot will feed back a message, including the robot model, robot software version, robot firmware version, robot hardware version and other related information.

End-Effector Transparent Transmission Function Periodic Data Acquisition (CNDE)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

CNDE Configuration Description
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

After the end-effector transparent transmission function is enabled, the "axle_gen_com_data" option and period can be configured in CNDE to obtain the periodic data of the peripheral read by the end-effector. The feedback data frame is defined as follows.

.. centered:: Table 3-8  End-Effector Transparent Transmission Function Periodic Data CNDE Feedback Protocol Definition

.. list-table::
   :widths: 30 30 40
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Byte 1**
     - **Byte 2**
     - **Byte 3-130**

   * - ErrorCode
     - Len
     - Data

   * - 0 - Communication normal
     - Length of periodic data
     - Data frame buffer

   * - 1 - Communication abnormality between end-effector and robot
     - Length cleared when error code is non-zero
     - Buffer cleared when error code is non-zero

   * - 2 - End-effector 485 communication abnormality
     - 
     - 

Taking the DIO Health Care Moxibustion Head peripheral periodic data configuration as an example, the code shows the configuration to obtain the end-effector periodic transparent transmission data with an acquisition period of 50ms.

End-Effector Transparent Transmission CNDE Configuration Code Example:

.. code-block:: 
    :linenos:

    tring outputCfg = "axle_gen_com_data";    //Obtain end-effector transparent transmission periodic data
    byte[] sendBuffer = new byte[] { };
    byte[] cfgBuffer = Encoding.UTF8.GetBytes(outputCfg);
    CNDEPkg pkg  = new CNDEPkg();
    pkg.type = 1;  //Output configuration
    pkg.len = (ushort)(2 + outputCfg.Length);
    pkg.data.Clear();
    UInt16 period = 50;   //50ms update
    byte[] periodBt = new byte[2] {0, 0};
    Int16ToByte(period, ref periodBt);
    pkg.data.AddRange(periodBt);  //Communication period
    pkg.data.AddRange(cfgBuffer); 
    pkg.ToBytes(ref sendBuffer);

CNDE-Based DIO Health Care Moxibustion Head Periodic Data Unpacking Code Example:
  
.. code-block:: 
    :linenos:

    if (pkg.type == 4)
    {
        int size = Marshal.SizeOf(putDate);
        IntPtr structPtr = Marshal.AllocHGlobal(size);
        Marshal.Copy(pkg.data.ToArray(), 0, structPtr, size);
        putDate = (OUTPKG)Marshal.PtrToStructure(structPtr, typeof(OUTPKG));

        int errorcode = putDate.axle_gen_com_data[0];
        int datalen = putDate.axle_gen_com_data[1];
        // Filter out abnormal packets
        if ((errorcode != 0) || (datalen == 0) ||
        (putDate.axle_gen_com_data[2] != 0xAB) || 
        (putDate.axle_gen_com_data[3] != 0xBA))
        {
            Console.WriteLine($"rcv data is error");
            continue;
        }
        // Package according to DIO Health Care Moxibustion Head protocol
        int curTem = putDate.axle_gen_com_data[6];
        int targetTem = putDate.axle_gen_com_data[7];
        int genData1 = putDate.axle_gen_com_data[8] << 8 | putDate.axle_gen_com_data[9];
        int genData2 = putDate.axle_gen_com_data[10] << 8 | putDate.axle_gen_com_data[11];
        int genData3 = putDate.axle_gen_com_data[12] << 8 | putDate.axle_gen_com_data[13];
        int genData4 = putDate.axle_gen_com_data[14] << 8 | putDate.axle_gen_com_data[15];
        int genData5 = putDate.axle_gen_com_data[16] << 8 | putDate.axle_gen_com_data[17];
        int genData6 = putDate.axle_gen_com_data[18] << 8 | putDate.axle_gen_com_data[19];

        Console.WriteLine($"the data is errorcode {errorcode};  datalen  {datalen}  curTem  {curTem}; targetTem  {targetTem}  genData1  {genData1}  genData2  {genData2}  genData3  {genData3}  genData4  {genData4}  genData5  {genData5}  genData6  {genData6}  ");
        udpClient.Client.ReceiveTimeout = 100;
        Marshal.FreeHGlobal(structPtr);
    }

End-Effector Transparent Transmission Function-Based DIO Health Care Moxibustion Head Non-Periodic Data Communication Code Example:
  
.. code-block:: 
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