Version Update Description
================================

.. toctree:: 
    :maxdepth: 5

.. list-table::
   :widths: 10 10 30
   :header-rows: 0
   :align: center

   * - **Version**
     - **Date**
     - **Update Description**

   * - V3.9.8
     - 2026-07-27
     - | 1. Updated the robot status feedback structure to include the current robot LUA program running status: 0 - program not running; 1 - program running (including program paused);
       | 2. Updated parameter descriptions for SetExToolCoord() and SetExToolList() interfaces for setting external tool coordinate system and tool coordinate system list. The external tool coordinate system numbers have been updated to 20-39. Updated the code examples for external tool coordinate system operations.
       | 3. Added the ability to retrieve tool type, installation position, tool ID, and load number parameters to the GetToolCoordWithID() interface for getting tool coordinate system parameters.
       | 4. Added the ability to retrieve reference coordinate system parameters to the GetWObjCoordWithID() interface for getting workpiece coordinate system parameters.
       | 5. Added the ability to retrieve the robot end-mounted workpiece coordinate system pose parameters to the GetExToolCoordWithID() interface for getting external tool coordinate system parameters.
       | 6. Added the ability to retrieve extended axis number and calibration flag parameters to the GetExAxisCoordWithID() interface for getting extended axis coordinate system parameters.
       | 7. Added robot joint safety speed parameter settings to the SetVelReducePara() interface for setting robot safety speed.
       | 8. Added a welding machine control mode acquisition example to the welding parameter configuration code example.
       | 9. Added code examples for getting extended DI and extended DO function configurations to the extended IO welding signal configuration code example.
       | 10. Added a new code example for setting robot joint safety speed;
       | 11. Added a new interface WaitStationaryMotionDone() for waiting for in-place idle motion to complete;
       | 12. Added a new interface SetStationaryTrackPara() for configuring conveyor belt in-place tracking parameters, along with a conveyor belt in-place tracking code example;
       | 13. Added new interfaces WorkPieceTrsfStart() and WorkPieceTrsfEnd() for starting and ending workpiece coordinate system transformation, along with a workpiece coordinate system transformation code example.
       | 14. Added GetWeldMachineCtrlMode() interface for getting the welding machine control mode.
       | 15. Added GetExtDIConfig() and GetExtDOConfig() interfaces for getting extended DI function and extended DO function configurations.

   * - V3.9.7
     - 2026-06-25
     - | 1. PhotoelectricSensorTCPCalibration() parameters can now adapt to filenames without a path;
       | 2. LoadTrajectoryJ() parameters can now adapt to filenames without a path;
       | 3. LoadTrajectoryLA() parameters can now adapt to filenames without a path;
       | 4. LoadDefaultProgConfig() parameters can now adapt to filenames without a path;
       | 5. ProgramLoad() parameters can now adapt to filenames without a path;
       | 6. Added dexterous hand enable status parameter to SetAxleLuaEnableDeviceType() interface;
       | 7. Added dexterous hand enable status parameter to GetAxleLuaEnableDeviceType() interface;
       | 8. Modified the interfaces for getting currently configured end-effector device enable types and gripper action control;
       | 9. Added dexterous hand enable and function codes;
       | 10. Added SetDexterousHandsMove() interface for controlling dexterous hand motion;
       | 11. Added SetDexterousHandsAct() interface for controlling dexterous hand reset and activation;
       | 12. Added ClearDexterousHandsError() interface for clearing dexterous hand errors;
       | 13. Added SetDexterousHandsFunc() interface for setting enabled dexterous hand action control functions;
       | 14. Added GetDexterousHandsFunc() interface for getting enabled dexterous hand action control functions;
       | 15. Added interfaces for setting and getting the weave end return to cycle zero point parameters;
       | 16. Added SetWeaveOffsetRT() interface for setting real-time weave offset, and SetSpeedInstant() interface for real-time speed setting.

   * - V3.9.6
     - 2026-05-26
     - | 1.Updated robot status feedback structure, added extension axis coordinate system number status;
       | 2.Updated robot status feedback configuration enumeration type, added extension axis coordinate system number configuration enumeration;
       | 3.Added ExtAxisGetParamConfig() interface to get UDP extension axis parameter configuration;
       | 4.Added ServoJV() interface for robot joint space velocity servo mode motion;
       | 5.Added ServoMITStart() interface for robot joint MIT control start;
       | 6.Added ServoMITEnd() interface for robot joint MIT control end;
       | 7.Added ServoMIT() interface for robot joint MIT control;
       | 8.Added SetLaserWeldingParam() interface for robot laser welding parameter configuration;
       | 9.Added SetLaserWeldingStartEnd() interface to set robot laser welding start/stop;
       | 10.Added SetLaserWeldingEnable() interface to set laser welding machine enable/disable;
       | 11.Added ResetLaserWeldingErr() interface to set laser welding machine fault reset;
       | 12.Added GetLaserWeldingRunningState() interface to get laser welding machine running status;
       | 13.Added GetLaserWeldingErrState() interface to get laser welding machine fault status;
       | 14.Added GetLaserWeldingParamTarget() interface to get laser welding configuration parameters;
       | 15.Added GetLaserWeldingParamActual() interface to get the currently active configuration parameters of the laser welding machine;
       | 16.Added SetLaserWeldingEnableExtDoNum() interface to configure the extended IO enable DO port of the laser welding machine;
       | 17.Added SetLaserWeldingStartExtDoNum() interface to configure the extended IO start DO port of the laser welding machine;
       | 18.Added SetLaserWeldingErrResetExtDoNum() interface to configure the extended IO fault reset DO port of the laser welding machine;
       | 19.Added SetLaserWeldingRunningStateExtDiNum() interface to configure the extended IO running status (laser on status) DI port of the laser welding machine;
       | 20.Added SetLaserWeldingErrStateExtDiNum() interface to configure the extended IO fault status DI port of the laser welding machine.
   
   * - V3.9.5
     - 2026-04-24
     - | 1.SetTrajectoryJSpeed() interface adds two modes: speed reduction mode and direct switching;
       | 2.Updated robot status feedback structure type;
       | 3.Added robot status feedback configuration enumeration type;
       | 4.Added robot status feedback configuration result class;
       | 5.Added SetRobotRealtimeStateConfig() interface to configure robot CNDE status feedback;
       | 6.Added AddRobotRealtimeState() interface to add a robot state to CNDE status configuration;
       | 7.Added DeleteRobotRealtimeState() interface to delete a robot state from CNDE status configuration;
       | 8.Added SetRobotRealtimeStatePeriod() interface to set the CNDE status feedback period;
       | 9.Added GetRobotRealtimeStateConfig() interface to get all current CNDE status feedback state sets and period.

   * - V3.9.4
     - 2026-03-25
     - | 1.ServoJTStart() interface adds communication type selection parameter, supporting XMLRPC/UDP communication;
       | 2.ServoJTEnd() interface adds communication type selection parameter, supporting XMLRPC/UDP communication;
       | 3.ServoJT() interface adds communication type selection parameter, supporting XMLRPC/UDP communication;
       | 4.ServoMoveStart() interface adds communication type selection parameter, supporting XMLRPC/UDP communication;
       | 5.ServoMoveEnd() interface adds communication type selection parameter, supporting XMLRPC/UDP communication;
       | 6.ServoJ() interface adds communication type selection parameter, supporting XMLRPC/UDP communication;
       | 7.SetWeldMachineCtrlMode() interface adds control mode selection parameter;
       | 8.ExtDevGetUDPComParam() interface adds ability to obtain UDP communication parameter: whether to automatically reconnect after control box restart;
       | 9.Adds SetAxleGenComEnable() interface to enable end-effector general transparent transmission function;
       | 10.Adds SndRcvAxleGenComCmdData() interface for end-effector to send non-periodic data and wait for response;
       | 11.Adds SetRobotStopOnComDisc() interface to stop robot operation when port communication is disconnected;
       | 12.Adds GetRobotStopOnComDisc() interface to obtain parameters for stopping robot operation when port communication is disconnected;
       | 13.Adds SetDIConfig() interface to set configurable CI port functions of the control box;
       | 14.Adds GetDIConfig() interface to obtain configurable CI port functions of the control box;
       | 15.Adds SetDOConfig() interface to set configurable CO port functions of the control box;
       | 16.Adds GetDOConfig() interface to obtain configurable CO port functions of the control box;
       | 17.Adds SetToolDIConfig() interface to set configurable End-CI port functions of the end-effector;
       | 18.Adds GetToolDIConfig() interface to obtain configurable End-CI port functions of the end-effector;
       | 19.Adds SetDIConfigLevel() interface to set the active state of configurable CI ports of the control box;
       | 20.Adds GetDIConfigLevel() interface to obtain the active state of configurable CI ports of the control box;
       | 21.Adds SetDOConfigLevel() interface to set the active state of configurable CO ports of the control box;
       | 22.Adds GetDOConfigLevel() interface to obtain the active state of configurable CO ports of the control box;
       | 23.Adds SetToolDIConfigLevel() interface to set the active state of configurable CI ports of the end-effector;
       | 24.Adds GetToolDIConfigLevel() interface to obtain the active state of configurable CI ports of the end-effector;
       | 25.Adds SetStandardDILevel() interface to set the active state of standard DI ports of the control box;
       | 26.Adds GetStandardDILevel() interface to obtain the active state of standard DI ports of the control box;
       | 27.Adds SetStandardDOLevel() interface to set the active state of standard DO ports of the control box;
       | 28.Adds GetStandardDOLevel() interface to obtain the active state of standard DO ports of the control box;
       | 29.Adds SetExAxisCmdDoneTimeUDP() interface for setting extension axis positioning completion time;
       | 30.Adds OpenLuaDownload() interface for downloading open protocol Lua files;
       | 31.Adds OpenLuaDelete() interface for deleting open protocol Lua files;
       | 32.Adds AllOpenLuaDelete() interface for deleting open protocol Lua files;
       | 33.Adds SendUDPFrameUDP() interface for sending instruction frames;
       | 34.Adds SetCmdRpyCallback() interface to set callback function for execution results of instructions sent by SDK via UDP;
       | 35.Adds SetVelReducePara() interface for setting safety speed parameters;
       | 36.Adds OriginPointWeaveStart() interface for starting fixed-point swing;
       | 37.Adds OriginPointWeaveEnd() interface for ending fixed-point swing;
       | 38.Adds SetUserLEDColor() interface for setting user-defined robot end-effector LED color;
       | 39.Adds MoveToTPDStart() interface for moving to the starting point of TPD trajectory recording; 

   * - V3.9.3
     - 2026-02-11
     - | 1.ServoCart() interface added extended axis parameters
       | 2.SetOutputResetCtlBoxDO() interface added parameter for whether to reload DO state before reset after pause resume
       | 3.SetOutputResetCtlBoxAO() interface added parameter for whether to reload DO state before reset after pause resume
       | 4.SetOutputResetAxleDO() interface added parameter for whether to reload DO state before reset after pause resume
       | 5.SetOutputResetAxleAO() interface added parameter for whether to reload DO state before reset after pause resume
       | 6.SetOutputResetExtDO() interface added parameter for whether to reload DO state before reset after pause resume
       | 7.SetOutputResetExtAO() interface added parameter for whether to reload DO state before reset after pause resume
       | 8.SetOutputResetSmartToolDO() interface added parameter for whether to reload DO state before reset after pause resume
       | 9.Added GetInverseKinExaxis() inverse kinematics solution interface including extended axis position

   * - V3.9.2
     - 2026-01-26
     - | 1. Added a processing strategy parameter for undetected force/torque to the FT_RotInsertion() interface
       | 2. Added robot fixed-point tracking related parameters to the LaserSensorRecordandReplay() interface
       | 3. Added the MoveStationary() interface
       | 4. Added the TCPComputeRPY() interface
       | 5. Added the TCPComputeXYZ() interface
       | 6. Added the TCPRecordFlangePosStart() interface
       | 7. Added the TCPRecordFlangePosEnd() interface
       | 8. Added the TCPGetRecordFlangePos() interface
       | 9. Added the PhotoelectricSensorTCPCalibration() interface 

   * - V3.9.1
     - 2025-12-25
     - | 1. Added oacc speed scaling factor parameter / physical acceleration parameter to the MoveL() interface;
       | 2. Added oacc speed scaling factor parameter / physical acceleration parameter to the MoveC() interface;
       | 3. Optimized parameter descriptions for physical speed and physical acceleration in the Circle() interface;
       | 4. Added FT_Control() overloaded function with rx, ry activation thresholds and torque adjustment coefficient parameters;
       | 5. Added SerCoderCompenParams() interface;

   * - V3.9.0
     - 2025-11-26
     - | 1. JointSensitivityCalibration() interface adds j1~j6 joint linearity return
       | 2. Added JointHysteresisError() interface
       | 3. Added JointRepeatability() interface
       | 4. Added SetAdmittanceParams() interface
       | 5. Added MoveToIntersectLineStart() interface
       | 6. Added MoveIntersectLine() interface
       
   * - V3.8.7
     - 2025-10-21
     - | 1.FT_Control() adds mass parameter and damping parameter interface
       | 2.Adds JointSensitivityCalibration() interface
       | 3.Adds JointSensitivityCollect() interface
       | 4.Adds MotionQueueClear() interface
       | 5.Adds GetSlavePortErrCounter() interface
       | 6.Adds SlavePortErrCounterClear() interface
       | 7.Adds SetVelFeedForwardRatio() interface
       | 8.Adds GetVelFeedForwardRatio() interface
       | 9.Adds RobotMCULogCollect() interface
       | 10.State structure adds ServoJ command count and last command target position data
       | 11.New spiral parameter structure SpiralParam adds velocity acceleration parameter mode;

   * - V3.8.6
     - 2025-09-19
     - | 1.SetLoadCoord() interface adds load number parameter
       | 2.Adds LaserTrackingLaserOnOff() interface
       | 3.Adds LaserTrackingTrackOnOff() interface
       | 4.Adds LaserTrackingSearchStart_xyz() interface
       | 5.Adds LaserTrackingSearchStart_point() interface
       | 6.Adds LaserTrackingSearchStop() interface
       | 7.Adds LaserTrackingSensorConfig() interface
       | 8.Adds LaserTrackingSensorSamplePeriod() interface
       | 9.Adds LoadPosSensorDriver() interface
       | 10.Adds UnLoadPosSensorDriver() interface
       | 11.Adds LaserSensorRecord1() interface
       | 12.Adds LaserSensorReplay() interface
       | 13.Adds MoveLTR() interface
       | 14.Adds LaserSensorRecordandReplay() interface
       | 15.Adds MoveToLaserRecordStart() interface
       | 16.Adds MoveToLaserRecordEnd() interface
       | 17.Adds MoveToLaserSeamPos() interface
       | 18.Adds GetLaserSeamPos() interface
       | 19.Adds ImpedanceControlStartStop() interface
       | 20.Adds GetToolCoordWithID() interface
       | 21.Adds GetWObjCoordWithID() interface
       | 22.Adds GetExToolCoordWithID() interface
       | 23.Adds GetExAxisCoordWithID() interface
       | 24.Adds GetTargetPayloadWithID() interface
       | 25.Adds GetExAxisCoordWithID() interface
       | 26.Adds GetCurWObjCoord() interface
       | 27.Adds GetCurExToolCoord() interface
       | 28.Adds GetCurExToolCoord() interface
       | 29.Adds KernelUpgrade() interface
       | 30.Adds GetKernelUpgradeResult() interface
       | 31.Adds CustomWeaveSetPara() interface
       | 32.Adds CustomWeaveGetPara() interface
       | 33.State structure adds tool, workpiece, external tool, extended axis coordinate system and load mass, centroid data

   * - V3.8.5
     - 2025-08-20
     - | 1.Adds OpenLuaUpload() interface
       | 2.Adds GetFieldBusConfig() interface
       | 3.Adds FieldBusSlaveWriteDO() interface
       | 4.Adds FieldBusSlaveWriteAO() interface
       | 5.Adds FieldBusSlaveReadDI() interface
       | 6.Adds FieldBusSlaveReadAI() interface
       | 7.Adds FieldBusSlaveWaitDI() interface
       | 8.Adds FieldBusSlaveWaitAI() interface
       | 9.Adds SetSuckerCtrl() interface
       | 10.Adds GetSuckerState() interface
       | 11.Adds WaitSuckerState() interface
       | 12.Adds MoveL() velocity acceleration parameter mode velAccParamMode interface
       | 13.Adds MoveL() overload function 1 interface
       | 14.Adds MoveL() overload function 2 interface
       | 15.Adds MoveC() velocity acceleration parameter mode velAccParamMode interface
       | 16.Adds MoveC() overload function 1 interface
       | 17.Adds Circle() velocity acceleration parameter mode velAccParamMode interface
       | 18.Adds Circle() overload function 1 interface
       | 19.Adds SetExAxisRobotPlan() interface

   * - V3.8.4
     - 2025-07-17
     - | 1.ExtAxisMove() interface adds blend smoothing parameter;
       | 2.Adds SetFocusCalibPoint() interface
       | 3.Adds ComputeFocusCalib() interface;
       | 4.Adds FocusStart() interface;
       | 5.Adds FocusEnd() interface
       | 6.Adds SetFocusPosition() interface;
       | 7.Adds SetEncoderUpgrade() interface;
       | 8.Adds SetJointFirmwareUpgrade() interface
       | 9.Adds SetCtrlFirmwareUpgrade() interface;
       | 10.Adds SetEndFirmwareUpgrade() interface;
       | 11.Adds JointAllParamUpgrade() interface;
       
   * - V3.8.3
     - 2025-06-24
     - | 1.Circle() interface adds acceleration percentage and smoothing radius parameters;
       | 2.EndForceDragControl() interface adds robot collision detection flag parameter during assisted dragging;
       | 3.ServoJ() interface adds command ID parameter;
       | 4.Adds SetSSHScpCmd() interface
       | 5.Adds SetWideBoxTempFanMonitorParam() interface;
       | 6.Adds GetWideBoxTempFanMonitorParam() interface;
       | 7.State structure adds control box temperature and fan current status data;
              
   * - V3.8.2
     - 2025-06-13
     - | 1.WeaveSetPara() interface adds swing direction roll angle (rotation around swing X-axis) parameter
       | 2.WeaveChangeStart() interface adds swing number, welding start speed, welding end speed parameters
       | 3.ExtDevSetUDPComParam() interface adds whether to automatically establish connection after power restart parameter
       | 4.SetCollisionDetectionMethod() interface adds collision level threshold method selection
       | 5.PtpFIRPlanningStart() interface adds unified joint jerk extreme value
       | 6.Adds WeldingSetVoltageGradualChangeStart() interface
       | 7.Adds WeldingSetVoltageGradualChangeEnd() interface
       | 8.Adds WeldingSetCurrentGradualChangeStart() interface
       | 9.Adds WeldingSetCurrentGradualChangeEnd() interface
       | 10.Adds ArcWeldTraceAIChannelCurrent() interface
       | 11.Adds ArcWeldTraceAIChannelVoltage() interface
       | 12.Adds ArcWeldTraceCurrentPara() interface
       | 13.Adds ArcWeldTraceVoltagePara() interface
       | 14.Adds GetSmarttoolBtnState() interface
       | 15.Adds ExtAxisGetCoord() interface
                     
   * - V3.8.1
     - 2025-04-24
     - | 1.ConveyorSetParam() interface adds tracking motion type, tracking start distance, tracking end distance parameters
       | 2.Adds AccSmoothStart() interface
       | 3.Adds AccSmoothEnd() interface
       | 4.Adds RbLogDownload() interface
       | 5.Adds AllDataSourceDownload() interface
       | 6.Adds DataPackageDownload() interface
       | 7.Adds GetRobotSN() interface
       | 8.Adds ShutDownRobotOS() interface
       | 9.Adds ConveyorComDetect() interface
       | 10.Adds ConveyorComDetectTrigger() interface