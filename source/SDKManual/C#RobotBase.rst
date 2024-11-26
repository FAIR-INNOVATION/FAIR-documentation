Robotics Basics
======================

.. toctree:. 
    :maxdepth: 5

Instantiated Robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Robot interface class constructor
    */
    Robot(). 

Establish communication with the controller
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * :: @brief establishes communication with the robot controller
    * @param [in] ip controller IP address, out default is 192.168.58.2
    * @return error code
    */
    int RPC(string ip);

Disconnect communication with the robot
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Disconnect communication with robot controller 
    * @return error code 
    */ 
    int CloseRPC(); 

Query SDK version number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief query SDK version number 
    * @param [out] version SDK version number 
    * @return error code 
    */  
    int GetSDKVersion(ref string version).

Get controller IP
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Get controller IP
    * @param [out] ip Controller IP
    * @return error code
    */
    int GetControllerIP(ref string ip).

Controls the robot into and out of drag-and-drop instructor mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Controls the robot into and out of drag-and-drop instructor mode.
    * @param [in] state 0-exit drag instructor mode, 1-enter drag instructor mode
    * @return error code
    */
    int DragTeachSwitch(byte state).

Queries whether the robot is in drag mode
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Query whether the robot is in drag-and-drop mode.
    * @param [out] state 0-non-drag instructional mode, 1-drag instructional mode
    * @return error code
    */
    int IsInDragTeach(ref byte state). 

Control robot up-enable or down-enable
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Controls robot up-enable or down-enable, defaults to automatic up-enable when the robot is powered on.
    * @param [in] state 0-down enable, 1-up enable
    * @return error code
    */
    int RobotEnable(byte state). 

Control of robot hand-automatic mode switching
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Control of robot hand-automatic mode switching
    * @param [in] mode 0-automatic mode, 1-manual mode
    * @return error code
    */
    int Mode(int mode).

code example
+++++++++++++
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