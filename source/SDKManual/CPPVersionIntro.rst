Version Update Description
==================================

.. toctree:: 
    :maxdepth: 5

.. list-table::
   :widths: 10 10 30
   :header-rows: 0
   :align: center

   * - **Version**
     - **Date**
     - **Update Description**

   * - V3.9.5
     - 2026-04-24
     - | 1.SetTrajectoryJSpeed() interface adds two modes: speed reduction mode and direct switching;
       | 2.FieldBusSlaveWriteAO() interface changes the written value type to double, where AO0~AO15 are integer type and AO16~AO31 are floating-point type;
       | 3.FieldBusSlaveReadAI() interface changes the read value type to double, where AI0~AI15 are integer type and AI16~AI31 are floating-point type;
       | 4.Updated robot status feedback structure type;
       | 5.Added robot status feedback configuration enumeration type;
       | 6.Added SetRobotRealtimeStateConfig() interface to configure robot CNDE status feedback;
       | 7.Added AddRobotRealtimeState() interface to add a robot state to CNDE status configuration;
       | 8.Added DeleteRobotRealtimeState() interface to delete a robot state from CNDE status configuration;
       | 9.Added SetRobotRealtimeStatePeriod() interface to set the CNDE status feedback period;
       | 10.Added GetRobotRealtimeStateConfig() interface to get all current CNDE status feedback state sets and period.

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
       | 11.New spiral parameter structure SpiralParam adds velocity acceleration parameter mode

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
     - | 1.MoveL(), MoveC(), Circle() commands add velAccParamMode parameter
       | 2.Adds MoveJ(), SplinePTP(), ExtAxisSyncMoveJ() automatic forward kinematics calculation interface
       | 3.LoadTrajectoryLA() adds uniform velocity lookahead switch parameter
       | 4.Adds MoveL(), MoveC(), Circle(), NewSplinePoint(), NewSpiral(), ExtAxisSyncMoveL(), ExtAxisSyncMoveC() automatic inverse kinematics calculation interface
       | 5.Adds SetSuckerCtrl(), GetSuckerState(), WaitSuckerState() suction cup control interfaces, SetExAxisRobotPlan() synchronized motion strategy interface
       | 6.Adds OpenLuaUpload(), GetFieldBusConfig(), FieldBusSlaveWriteDO(), FieldBusSlaveWriteAO(), FieldBusSlaveReadDI(), FieldBusSlaveReadAI(), FieldBusSlaveWaitDI(), FieldBusSlaveWaitAI() robot slave mode control commands

   * - V3.8.4
     - 2025-07-17
     - | 1.ExtAxisMove() interface adds blend smoothing parameter
       | 2.Adds SetFocusCalibPoint() interface
       | 3.Adds ComputeFocusCalib() interface
       | 4.Adds SetFocusPosition() interface
       | 5.Adds FocusStart() interface
       | 6.Adds FocusEnd() interface
       | 7.Adds SetJointFirmwareUpgrade() interface
       | 8.Adds SetCtrlFirmwareUpgrade() interface
       | 9.Adds SetEndFirmwareUpgrade() interface
       | 10.Adds JointAllParamUpgrade() interface
       
   * - V3.8.3
     - 2025-06-24
     - | 1.Circle() interface adds acceleration percentage and smoothing radius parameters
       | 2.EndForceDragControl() interface adds robot collision detection flag parameter during assisted dragging
       | 3.ServoJ() interface adds command ID parameter
       | 4.Adds SetWideBoxTempFanMonitorParam() interface
       | 5.Adds GetWideBoxTempFanMonitorParam() interface
       | 6.State structure adds control box temperature and fan current status data
              
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
                     
   * - V3.8.0
     - 2025-02-12
     - | 1.EndForceDragControl() interface adds singularity avoidance parameter
       | 2.ArcWeldTraceControl() interface adds offset parameter
       | 3.Adds WeaveChangeStart() interface
       | 4.Adds WeaveChangeEnd() interface
       | 5.Adds LoadTrajectoryLA() interface
       | 6.Adds MoveTrajectoryLA() interface
       | 7.Adds CustomCollisionDetectionStart() interface
       | 8.Adds CustomCollisionDetectionEnd() interface