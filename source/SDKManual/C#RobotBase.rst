Basics
=================

.. toctree:: 
    :maxdepth: 5

Instantiate the robot
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Robot interface class constructor
    */
    Robot();

Establishes communication with the controller
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Establish communication with the robot controller
    * @param  [in] ip  Controller IP address. The default value is 192.168.58.2
    * @return Error code
    */
    int RPC(string ip);

Disconnect the communication with the robot control box
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
    
    /** 
    * @brief Disconnect the communication with the robot control box
    * @return Error code 
    */ 
    int CloseRPC();

Query the SDK version number
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Query the SDK version number
    * @param  [out] version  SDK version
    * @return  Error code
    */  
    int GetSDKVersion(ref string version);

Obtain Controller IP address
++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Obtain Controller IP address
    * @param  [out] ip  Controller IP
    * @return  Error code
    */
    int GetControllerIP(ref string ip);

Control the robot to enter or exit the drag teaching mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Control the robot to enter or exit the drag teaching mode
    * @param  [in] state 0-exit drag mode，1-enter the drag mode
    * @return  Error code
    */
    int DragTeachSwitch(byte state);

Queries whether the robot is in drag mode
++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Check whether the robot is in drag mode
    * @param  [out] state 0-non-drag teaching mode，1-drag the teaching mode
    * @return  Error code
    */
    int IsInDragTeach(ref byte state); 

Control up enable and down enable
+++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief  Enable or disable the function on or off the robot. By default, the function is enabled automatically after the robot is powered on
    * @param  [in] state  0-down-enable，1-upper enable
    * @return  Error code
    */
    int RobotEnable(byte state); 

Control robot hand/automatic mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Control robot hand/automatic mode
    * @param [in] mode 0-automatic mode，1-manual mode
    * @return Error code
    */
    int Mode(int mode);

Code example
+++++++++++++++
.. code-block:: c#
    :linenos:
    
    private void btnStandard_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2"); 

        string ip = "";
        string version = "";
        byte state = 0;

        robot.GetSDKVersion(ref version);
        Console.WriteLine($"SDK version : {version}");
        robot.GetControllerIP(ref ip);
        Console.WriteLine($"controller ip : {ip}");

        robot.Mode(1);
        Thread.Sleep(1000);
        robot.DragTeachSwitch(1);
        int rtn = robot.IsInDragTeach(ref state);
        Console.WriteLine($"drag state : {state}");
        Thread.Sleep(3000);
        robot.DragTeachSwitch(0);
        Thread.Sleep(1000);
        robot.IsInDragTeach(ref state);
        Console.WriteLine($"drag state : {state}");
        Thread.Sleep(3000);
        robot.RobotEnable(0);
        Thread.Sleep(3000);
        robot.RobotEnable(1);

        robot.Mode(0);
        Thread.Sleep(1000);
        robot.Mode(1);
    }
