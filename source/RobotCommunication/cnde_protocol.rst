CNDE data frame protocol format
=====================================

The communication protocol of CNDE for cooperative robots is as follows. The protocol distinguishes data frames with different functions by frame types. See Table 2-2 for the definition of frame types. Different types of frames correspond to different data content. See Table 3-1 ~ Table 3-7 for specific data content definitions.

.. centered:: Table 2-1 Data Frame Format of Robot CNDE

.. list-table::
   :widths: 20 20 20 20 20 20 20
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Name**
     - **Frame header**
     - **Frame count**
     - **Frame type**
     - **Data length**
     - **Content**
     - **Frame end**
   
   * - **Length(byte)**
     - 2
     - 1
     - 1
     - 2
     - --
     - 2
   
   * - **Content**
     - 0x5A5A
     - 0 ~ 255
     - 0 ~ 8
     - Number of bytes of "content"
     - Data frame content
     - 0xA5A5

.. centered:: Table 2-2 Data Frame Types of Robot CNDE

.. list-table::
   :widths: 40 20 40
   :header-rows: 0
   :align: center
   :class: sheet-center

   * - **Type**
     - **Numerical value**
     - **Data frame direction**

   * - Input Configuration Frame (Control Configuration)
     - 0x00
     - Client->Robot

   * - Output Configuration Frame (Status Configuration)
     - 0x01
     - Client->Robot

   * - CNDE output start
     - 0x02
     - Client->Robot

   * - CNDE output stops
     - 0x03
     - Client->Robot

   * - Output data frame (status data)
     - 0x04
     - Robot->Client

   * - Input data frame (control data)
     - 0x05
     - Client->Robot

   * - Message
     - 0x06
     - Client->Robot, Robot->Client

   * - Set the version number of CNDE protocol of robot
     - 0x07
     - Client->Robot

   * - Obtain the robot soft firmware version
     - 0x08
     - Client->Robot, Robot->Client