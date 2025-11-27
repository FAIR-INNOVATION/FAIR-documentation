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

   * - V3.9.0
     - 2025-11-26
     - | 1. JointSensitivityCalibration() interface adds j1~j6 joint linearity return;
       | 2. Added JointHysteresisError() interface;
       | 3. Added JointRepeatability() interface;
       | 4. Added SetAdmittanceParams() interface;
       | 5. Added MoveToIntersectLineStart() interface;
       | 6. Added MoveIntersectLine() interface;

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