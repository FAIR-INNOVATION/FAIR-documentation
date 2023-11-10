C#
=======

This manual is the secondary development interface document of C#.

.. important::

    Robot parameter unit description: The robot position unit is millimeter (mm), and the attitude unit is degree (°).

.. important:: 

    1)	In code examples that are not specifically stated, the robot has been powered on and enabled by default;
    2)	All code examples in the documentation default to no interference within the robot's workspace;
    3)	Please use the data of the on-site robot in the actual use test;
    4)	Before using this SDK, you need to find the "xmlrpcnet" package through NuGet and add it to the project reference;


.. toctree:: 
    :numbered: 5
    :maxdepth: 5

    C#DataStructure
    C#RobotBase
    C#RobotMovement
    C#RobotIO
    C#RobotCommonSettings
    C#RobotSecuritySettings
    C#RobotStatusInquiry
    C#RobotTrajectoryRecurrence
    C#RobotWebAPPProgramUse
    C#RobotPeripherals
    C#RobotForceControl
    C#RobotOthers