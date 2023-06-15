Overview
++++++++++
frcobot_ros2 is an API interface developed by Fao collaborative robot based on ROS2, aiming to use Fao SDK more conveniently for entry-level users. The configuration of the default parameters through the parameter configuration file can adapt to different customer requirements.

fr_ros2
++++++++++++++
This chapter describes how to configure the APP running environment.

Basic environment installation
----------------------------------
It is recommended to use it on Ubuntu22.04LTS (Jammy). After the system is installed, you can install ROS2. It is recommended to use ros2-humble. For the installation of ROS2, please refer to the tutorial: https://docs.ros.org/en/humble/index.html.

Compile and build
---------------------
1. Create colcon workspace
fr_ros2 consists of two function packages, one is the function package frhal_msgs of the custom data structure, and the other is the program main body fr_ros2 function package. After installing the basic environment, first create a colcon workspace, such as:

.. code-block:: shell
    :linenos:

    cd ~/
    mkdir -p ros2_ws/src

2. Compile feature pack
Copy the code of the installation package to the ros2_ws/src directory, and run the following command in the ros2_ws directory:

.. code-block:: shell
    :linenos:

    colcon build --packages-select frhal_msgs

After waiting for the previous command to finish compiling, enter:

.. code-block::  shell
    :linenos:

    colcon build --packages-select fr_ros2

Quick start
++++++++++++++

Start
-----------------
Open the command line under Ubuntu and enter:

.. code-block::  shell
    :linenos:

    cd ros2_ws
    source install/setup.bash
    ros2 run fr_ros2 ros2_cmd_server

View the robotic arm status feedback
--------------------------------------------------
The status feedback of the robotic arm is released through the topic. Users can observe the status data refresh through the ros2 built-in command, or write a program to obtain the data. The following shows how to observe the status data of the robotic arm through the ros2 command.

Open the command line under Ubuntu and enter:

.. code-block:: shell
    :linenos:

    cd ros2_ws
    source install/setup.bash
    ros2 topic echo /nonrt_state_data

You can see the status data constantly refreshed in the command line window, as shown in the figure below.

.. image:: img/fr_ros2_002.png
    :width: 6in
    :align: center

Issue order
--------------------------
Open the command line under Ubuntu and enter:

.. code-block:: shell
    :linenos:

    cd ros2_ws
    source install/setup.bash
    rqt

After the above command is executed, a rqt GUI interface will be called out, as shown in the figure below.

.. image:: img/fr_ros2_003.png
    :width: 6in
    :align: center

Select plugins->serivce->serivce caller in the GUI interface, call up the following interface, select /FR_ROS_API_service, enter the command string in the interface expression and click call to see the reply message in the dialog box below.

.. image:: img/fr_ros2_004.png
    :width: 6in
    :align: center

.. important:: 

   - Enter a string rule description:

   The program internally screens the input string format. The format of the function input must be in the form of [function name](), and the parameter string in parentheses must be composed of letters, numbers, commas and minus signs. Other characters or spaces will report an error.

   - Command feedback value description:

   Except for the GET command which will feedback a string of strings, the feedback values of the rest of the functions are all int types. Generally, 0 means that an error occurred, and 1 means that it was executed correctly. If there are other values, please refer to the error code corresponding to the error code defined in the xmlrpc SDK.

Modify parameter
--------------------------
Since the simplified SDK is an improvement from the original SDK interface, it can be simplified because some parameters are given default values, and in the actual use process, there will be situations where the default parameters cannot meet the requirements. At this time, you can modify the values ​​of the corresponding default parameters. , and then loaded into the node.

There is a fr_ros2_para.yaml parameter file in the source code file. The parameters in the file are preset default parameters, which are used to simplify the command input parameters. You can modify the parameters according to your specific needs, and then use the command to dynamically modify the parameters: ros2 param load FR_ROS_API_nod ~/ros2_ws/src/fr_ros2/fr_ros2_para.yaml.

API Description
++++++++++++++++++

.. code-block:: c++
    :linenos:

    // Store a joint space point, note that the joint point id is independent of the following Cartesian point id
    int JNTPoint(int id, double j1-j6)      
    // example
    JNTPoint(1,10,11,12,13,14,15)

    // store a point in Cartesian space
    int CARTPoint(int id, double x,y,z,rx,ry,rz)
    // example
    CARTPoint(1,100,110,200,0,0,0)

    // Get the content of the point corresponding to the id serial number, name can be input JNT or CART
    string GET(string name, int id)          
    // example
    GET("JNT",1)

    int DragTeachSwitch(uint8_t state)
    // example
    DragTeachSwitch(0)

    int RobotEnable(uint8_t state)
    // example
    RobotEnable(1)

    int Mode(uint8_t state)
    // example
    Mode(1)

    int SetSpeed(float vel)
    // example
    SetSpeed(10)

    int SetToolCoord(int id, float x,float y, float z,float rx,float ry,float rz)
    // example
    SetToolCoord(1,0,0,0,0,0,0)

    int SetToolList(int id, float x,float y, float z,float rx,float ry,float rz );
    // example
    SetToolList(1,0,0,0,0,0,0)

    int SetExToolCoord(int id, float x,float y, float z,float rx,float ry,float rz);	
    // example
    SetExToolCoord(1,0,0,0,0,0,0)

    int SetExToolList(int id, float x,float y, float z,float rx,float ry,float rz);
    // example
    SetExToolList(1,0,0,0,0,0,0)

    int SetWObjCoord(int id, float x,float y, float z,float rx,float ry,float rz);
    // example
    SetWObjCoord(1,0,0,0,0,0,0)

    int SetWObjList(int id, float x,float y, float z,float rx,float ry,float rz);
    // example
    SetWObjList(1,0,0,0,0,0,0)

    int SetLoadWeight(float weight);
    // example
    SetLoadWeight(3.5)

    int SetLoadCoord(float x,float y,float z);
    // example
    SetLoadCoord(10,20,30)

    int SetRobotInstallPos(uint8_t install);
    // example
    SetRobotInstallPos(0)

    int SetRobotInstallAngle(double yangle,double zangle);
    // example
    SetRobotInstallAngle(90,0)

    // security configuration
    int SetAnticollision(float level1-level6);
    // example
    SetAnticollision(0.5,0.5,0.5,0.5,0.5,0.5)

    int SetCollisionStrategy(int strategy);
    // example
    SetCollisionStrategy(1)

    int SetLimitPositive(float limit1-limit6);
    // example
    SetLimitPositve(100,90,90,90,90,90)

    int SetLimitNegative(float limit1-limit6);
    // example
    SetLimitNegative(-100,-90,-90,-90,-90,-90)

    // clear all errors
    int ResetAllError();  

    int FrictionCompensationOnOff(uint8_t state);
    // example
    FrictionCompensationOnOff(1)

    int SetFrictionValue_level(float coeff1-coeff6);
    // example
    SetFrictionValue_level(1,1,1,1,1,1)

    int SetFrictionValue_wall(float coeff1-coeff6);
    // example
    SetFrictionValue_wall(0.5,0.5,0.5,0.5,0.5,0.5)

    int SetFrictionValue_ceiling(float coeff1-coeff6);
    // example
    SetFrictionValue_ceiling(0.5,0.5,0.5,0.5,0.5,0.5)

    // peripheral control
    int ActGripper(int idex,uint8_t act);
    // example
    ActGripper(1,1)

    int MoveGripper(int index,int pos);
    // example
    MoveGripper(1,10)

    // I/O control
    int SetDO(int id,uint8_t status);
    // example
    SetDO(1,1)

    int SetToolDO(int id,uint8_t status);
    // example
    SetToolDO(0,1)

    int SetAO(int id,uint8_t status);
    // example
    SetAO(1,100)

    int SetToolAO(int id,uint8_t status);
    // example
    SetToolAO(0,100)

    // motion command
    int StartJOG(uint8_t ref, uint8_t dir, float vel);
    // example
    StartJOG(1,1,10)

    int StopJOG(uint8_t ref);
    // example
    StartJOG(1)

    int ImmStopJOG();

    // point_name is to input pre-stored point information. For example, 
    // JNT1 is the point with the serial number of the joint point information 1, 
    // and CART1 is the point with the serial number of the Cartesian point information. 
    // The MoveJ command supports the input of joint points or Cartesian points.
    int MoveJ(string point_name, float vel);    
    // example
    MoveJ("JNT1",10)

    int MoveL(string point_name,float vel);
    // example
    MoveL("CART1",10)

    int MoveC(string point1_name,string point2_name, float vel);
    // example
    MoveC("JNT1","JNT2",10)

    int SplineStart();

    // This instruction only supports the input of joint data such as JNT1, 
    // and an error will be reported when inputting Cartesian points
    int SplinePTP(string point_name, float vel);
    // example
    SplinePTP("JNT2",10)

    int SplineEnd();

    int NewSplineStart(uint8_t ctlpoint);
    // example
    NewSplineStrart(1)

    // This instruction only supports the input of Cartesian data such as CART1, 
    // and an error will be reported when inputting joint data
    int NewSplinePoint(string point_name, float vel, int lastflag);
    // example
    NewSplinePoint("JNT2",20,0)

    int NewSplineEnd();

    // stop robot movement
    int StopMotion();   

    int PointsOffsetEnable(int flag, double x,y,z,rx,ry,rz);
    // example
    PointsOffsetEnable(1,10,10,10,0,0,0)

    int PointsOffsetDisable();
