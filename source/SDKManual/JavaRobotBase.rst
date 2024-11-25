Robotics Basics
=====================================

.. toctree:: 
    :maxdepth: 5

Instantiated Robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Robot interface class constructor
    */
    Robot robot = new Robot(); 

Establish communication with the controller
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Establish communication with the robot controller
    * @param [in] ip controller IP address, out default is 192.168.58.2
    * @return error code
    */
    int RPC(String ip).

Disconnect communication with the robot
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /** 
    * @brief Disconnect communication with robot controller 
    * @return error code 
    */ 
    int CloseRPC(). 

Query SDK version number
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Query SDK version number 
    * @return version number 
    */  
    String GetSDKVersion();

Get controller IP
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Get controller IP
    * @param [out] ip Controller IP
    * @return error code
    */
    int GetControllerIP(String[] ip).

Controlling the robot into and out of drag-and-drop instructor mode
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Controls the robot into and out of drag-and-drop instructor mode.
    * @param [in] state 0-exit drag instructor mode, 1-enter drag instructor mode
    * @return error code
    */
    int DragTeachSwitch(int state).

Control robot up-enable or down-enable
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Controls robot up-enable or down-enable, robot is automatically up-enabled by default after power-up
    * @param [in] state 0-down enable, 1-up enable
    * @return error code
    */
    int RobotEnable(int state). 

Control of robot hand-automatic mode switching
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    /**
    * @brief Control of robot hand-automatic mode switching
    * @param [in] mode 0-automatic mode, 1-manual mode
    * @return error code
    */
    int Mode(int mode).

code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: Java
    :linenos:

    public static void main(String[] args)
    {
        Robot robot = new Robot();
        robot.SetReconnectParam(true,20,500);//Set the number of reconnections, interval
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc connection success");
        }
        else
        {
            System.out.println("rpc connection fail");
            return ;
        }
        String[] ip={""};
        String version = "";
        version=robot.GetSDKVersion();
        System.out.println("SDK version : " + version);
        int rtn = robot.GetControllerIP(ip);
        System.out.println("controller ip : " + ip[0] + " " + rtn);
        robot.Mode(1);//1-Manual mode 0-Automatic mode
        robot.Sleep(1000);
        robot.DragTeachSwitch(1);//enter drag mode
        robot.Sleep(1000);
        ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
        System.out.println("drag state : " + pkg.robot_state);
        robot.Sleep(1000);
        robot.DragTeachSwitch(0);//exit drag mode
        robot.Sleep(1000);
        pkg = robot.GetRobotRealTimeState();
        System.out.println("drag state : " + pkg.robot_state);
        
        if (pkg.robot_state == 4){
           System.out.println("Drag and Drop mode");
        }else {
           System.out.println("Non-dragging mode");
        }
    }