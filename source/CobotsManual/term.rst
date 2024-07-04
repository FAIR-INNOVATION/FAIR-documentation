Term
========

.. toctree:: 
	:maxdepth: 5


**Shutdown Category**:

- **Class 0 shutdown**:When the power supply of the robot is cut off, the robot stops working immediately. This is an uncontrollable stop. Since each joint will brake at the fastest speed, the robot may deviate from the path set by the program.This protective stop can be used when the safety assessment limit is exceeded, or when an error occurs in the safety assessment part of the control system. For more information, refer to ENISO13850:2008 or IEC60204-1:2006.
- **Class 1 shutdown**:When the robot is powered to stop, the robot will stop. When the robot stops, the power will be cut off. This is a controllable stop, and the robot will follow the programmed path.Cut off the power supply after one second or once the robot stops stably. For more information, refer to ENISO13850:2008 or IEC60204-1:2006.
- **Class 2 shutdown**:Controllable stop when the robot is powered on. The robot stops all movements within one second.The operation of the safety assessment control system can make the robot stay at the stop position. For more information, refer to IEC60204-1:2006.

**Diagnostic coverage rate（DC）**:Used to measure the effectiveness of the diagnosis implemented to achieve the assessed performance level. For more information, see ENISO13849-1:2008..

**Integrator**:The integrator is the mechanism that designs the final installation of the robot. The integrator is responsible for the final risk assessment and must ensure that the final installation complies with local laws and regulations.

**Mean time to dangerous failure（MTTFd）**:The mean time to dangerous failure (MTTFd) refers to the value calculated and detected to reach the assessed performance level. For more information, see ENISO13849-1:2008.

**Risk assessment**:Risk assessment is the whole process of identifying all risks and reducing them to an appropriate level.The risk assessment shall be recorded and archived.For details, please refer to ISO12100.

**Performance Level**:Performance Level (PL) is a separate level, which is used to describe the ability of each safety related part of the control system to perform safety functions under predictable conditions. PL is the second highest reliability category, which means that the security function is quite trustworthy.For more information, refer to EN ISO13849-1:2008.

**Connecting flange**:The structure used to connect external tools, generally called flange.

**Robot end**: The center point of the last axis or connecting flange of the robot.

**Tool center point（TCP）**:The tool center point is the characteristic point of the robot tool, which is the control point of the robot system. It defaults to the center of the last moving axis or connecting flange when leaving the factory. The tool center point of each tool contains transformations and rotations set relative to the center of the tool output flange. The position coordinates X, Y, Z determine the position of the tool center point, and RX, RY, RZ determine the direction of the tool center.When the values are all zero, the center of the tool coincides with the center of the connecting flange.

**Tool position and attitude point（TCF）**:Reflect the attitude of the tool coordinate system relative to the end link coordinate system on the basis of the tool center point TCP.

**Base coordinate system**:The origin of the base coordinate system is generally defined at the center point between the first axis of the robot and the mounting surface. The x-axis is forward, and the y-axis is determined according to the right-hand rule in the axial direction.

**World coordinate system**:A fixed coordinate system established in a work unit or workstation.When there is only one robot, the coordinate system can be considered to coincide with the base coordinate system;When there are multiple robots or external devices, the world coordinate system can provide a unique reference system for these devices. On the premise of satisfying the coordinate system for convenient calibration of other devices, its specific position can be arbitrarily specified.

**Joint coordinate system**: The joint coordinate system is the coordinate system in the robot joint. Under the joint coordinate system, each axis of the robot can achieve independent forward or reverse movement within the limit range. It is applicable to the robot that needs to move in a large range and does not require the TCP posture of the robot. The single axis inching of the robot in manual mode is carried out in the joint coordinate system.

**Tool coordinate system**:The coordinate system used to define the position of the tool center point and the tool attitude. If not defined, the tool coordinate system defaults to the center of the connecting flange.After installing the tool, TCP will change and become the center of the end of the tool.

**External tool coordinate system**: The coordinate system used to define the position and pose of tools fixed outside the robot.

**Extension axis**:Remove the axis on the robot body, and add additional axis for work. The extension axis mainly includes slide rail, turntable and external servo equipment.

**Manual mode**: In this mode, all movements of the robot are manually controlled by the user, and external safety facilities such as safety gratings and safety doors do not work, so as to facilitate close debugging. 

**Automatic mode**:This mode is generally used for the robot to run the teaching program. At this time, the external safety facilities are enabled.

**Repetitive positioning accuracy**:The consistency of the position and attitude measured by the robot in the same condition and the same method for n times.

**Teaching pendant**:A hand-held unit that programs or moves the robot and is connected to the control system.
