Trajectory recurrence
==============================

.. toctree:: 
    :maxdepth: 5

Set TPD Trajectory Recording Parameters
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set TPD trajectory recording parameters
    * @param  [in] type  Recording data type, 1-Joint position
    * @param  [in] name  Trajectory file name
    * @param  [in] period_ms  Data sampling period, fixed value 2ms or 4ms or 8ms
    * @param  [in] di_choose  DI selection, bit0~bit7 correspond to controller DI0~DI7, bit8~bit9 correspond to end DI0~DI1, 0-Not selected, 1-Selected
    * @param  [in] do_choose  DO selection, bit0~bit7 correspond to controller DO0~DO7, bit8~bit9 correspond to end DO0~DO1, 0-Not selected, 1-Selected
    * @return  Error code
    */
    errno_t  SetTPDParam(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose);

Start TPD Trajectory Recording
++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Start TPD trajectory recording
    * @param  [in] type  Recording data type, 1-Joint position
    * @param  [in] name  Trajectory file name
    * @param  [in] period_ms  Data sampling period, fixed value 2ms or 4ms or 8ms
    * @param  [in] di_choose  DI selection, bit0~bit7 correspond to controller DI0~DI7, bit8~bit9 correspond to end DI0~DI1, 0-Not selected, 1-Selected
    * @param  [in] do_choose  DO selection, bit0~bit7 correspond to controller DO0~DO7, bit8~bit9 correspond to end DO0~DO1, 0-Not selected, 1-Selected
    * @return  Error code
    */
    errno_t  SetTPDStart(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose); 

Stop TPD Trajectory Recording
++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Stop TPD trajectory recording
    * @return  Error code
    */
    errno_t  SetWebTPDStop();

Delete TPD Trajectory Recording
++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Delete TPD trajectory recording
    * @param  [in] name  Trajectory file name
    * @return  Error code
    */   
    errno_t  SetTPDDelete(char name[30]);

TPD Trajectory Preload
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  TPD trajectory preload
    * @param  [in] name  Trajectory file name
    * @return  Error code
    */      
    errno_t  LoadTPD(char name[30]);

TPD Trajectory Playback
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  TPD trajectory playback
    * @param  [in] name  Trajectory file name
    * @param  [in] blend 0-No smoothing, 1-Smoothing
    * @param  [in] ovl  Speed scaling percentage, range [0~100]
    * @return  Error code
    */
    errno_t  MoveTPD(char name[30], uint8_t blend, float ovl);

Get TPD Start Pose
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get TPD start pose
     * @param  [in] name TPD file name, no file extension needed
     * @return  Error code
     */     
    errno_t  GetTPDStartPose(char name[30], DescPose *desc_pose);

Robot TPD Trajectory Recording Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestTPD(void)
    {
      ROBOT_STATE_PKG pkg = {};
      FRRobot robot;
      robot.LoggerInit();
      robot.SetLoggerLevel(1);
      int rtn = robot.RPC("192.168.58.2");
      if (rtn != 0)
      {
        return -1;
      }
      robot.SetReConnectParam(true, 30000, 500);
      int type = 1;
      char name[30] = "tpd2025";
      int period_ms = 4;
      uint16_t di_choose = 0;
      uint16_t do_choose = 0;
      robot.SetTPDParam(type, name, period_ms, di_choose, do_choose);
      robot.Mode(1);
      robot.Sleep(1000);
      robot.DragTeachSwitch(1);
      robot.SetTPDStart(type, name, period_ms, di_choose, do_choose);
      robot.Sleep(10000);
      robot.SetWebTPDStop();
      robot.DragTeachSwitch(0);
      float ovl = 100.0;
      uint8_t blend = 0;
      DescPose start_pose = {};
      rtn = robot.LoadTPD(name);
      printf("LoadTPD rtn is: %d\n", rtn);
      robot.GetTPDStartPose(name, &start_pose);
      printf("start pose, xyz is: %f %f %f. rpy is: %f %f %f \n", start_pose.tran.x, start_pose.tran.y, start_pose.tran.z, start_pose.rpy.rx, start_pose.rpy.ry, start_pose.rpy.rz);
      robot.MoveCart(&start_pose, 0, 0, 100, 100, ovl, -1, -1);
      robot.Sleep(1000);
      rtn = robot.MoveTPD(name, blend, ovl);
      printf("MoveTPD rtn is: %d\n", rtn);
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
      robot.SetTPDDelete(name);
      robot.CloseRPC();
      return 0;
    }

Trajectory Preprocessing
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Trajectory preprocessing
     * @param  [in] name  Trajectory file name
     * @param  [in] ovl Speed scaling percentage, range [0~100]
     * @param  [in] opt 1-Control point, default is 1
     * @return  Error code
     */     
    errno_t  LoadTrajectoryJ(char name[30], float ovl, int opt);

Trajectory Playback
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Trajectory playback
     * @return  Error code
     */     
    errno_t  MoveTrajectoryJ();

Get Trajectory Start Pose
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get trajectory start pose
     * @param  [in] name Trajectory file name
     * @return  Error code
     */     
    errno_t  GetTrajectoryStartPose(char name[30], DescPose *desc_pose);

Get Trajectory Point Number
++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Get trajectory point number
     * @return  Error code
     */     
    errno_t  GetTrajectoryPointNum(int *pnum);

Set Trajectory Running Speed
++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set trajectory running speed
     * @param  [in] ovl Speed percentage
     * @return  Error code
     */     
    errno_t  SetTrajectoryJSpeed(float ovl);

Set Trajectory Running Force and Torque
++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set trajectory running force and torque
     * @param  [in] ft Force and torque in three directions, unit N and Nm
     * @return  Error code
     */     
    errno_t  SetTrajectoryJForceTorque(ForceTorque *ft);

Set Trajectory Running Force in X Direction
++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set trajectory running force in X direction
     * @param  [in] fx Force in X direction, unit N
     * @return  Error code
     */     
    errno_t  SetTrajectoryJForceFx(double fx);

Set Trajectory Running Force in Y Direction
++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set trajectory running force in Y direction
     * @param  [in] fy Force in Y direction, unit N
     * @return  Error code
     */     
    errno_t  SetTrajectoryJForceFy(double fy);

Set Trajectory Running Force in Z Direction
++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set trajectory running force in Z direction
     * @param  [in] fz Force in X direction, unit N
     * @return  Error code
     */     
    errno_t  SetTrajectoryJForceFz(double fz);

Set Trajectory Running Torque Around X Axis
++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set trajectory running torque around X axis
     * @param  [in] tx Torque around X axis, unit Nm
     * @return  Error code
     */     
    errno_t  SetTrajectoryJTorqueTx(double tx);

Set Trajectory Running Torque Around Y Axis
++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set trajectory running torque around Y axis
     * @param  [in] ty Torque around Y axis, unit Nm
     * @return  Error code
     */     
    errno_t  SetTrajectoryJTorqueTy(double ty);

Set Trajectory Running Torque Around Z Axis
++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
     * @brief  Set trajectory running torque around Z axis
     * @param  [in] tz Torque around Z axis, unit Nm
     * @return  Error code
     */     
    errno_t  SetTrajectoryJTorqueTz(double tz);

Upload Trajectory J File
+++++++++++++++++++++++++++++
.. versionadded:: V3.7.7

.. code-block:: c++
    :linenos:

    /**
	 * @brief Upload trajectory J file
	 * @param [in] filePath Full path name of trajectory file to upload   C://test/testJ.txt
	 * @return Error code
	 */
	errno_t TrajectoryJUpLoad(const std::string& filePath);

Delete Trajectory J File
+++++++++++++++++++++++++++++
.. versionadded:: V3.7.7

.. code-block:: c++
    :linenos:

    /**
	 * @brief Delete trajectory J file
	 * @param [in] fileName File name testJ.txt
	 * @return Error code
	 */
	errno_t TrajectoryJDelete(const std::string& fileName);

Robot Trajectory J File Playback Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestTraj(void)
    {
      ROBOT_STATE_PKG pkg = {};
      FRRobot robot;
      robot.LoggerInit();
      robot.SetLoggerLevel(1);
      int rtn = robot.RPC("192.168.58.2");
      if (rtn != 0)
      {
        return -1;
      }
      robot.SetReConnectParam(true, 30000, 500);
      rtn = robot.TrajectoryJUpLoad("D://zUP/traj1.txt");
      printf("Upload TrajectoryJ A %d\n", rtn);
      char traj_file_name[30] = "/fruser/traj/traj1.txt";
      rtn = robot.LoadTrajectoryJ(traj_file_name, 100, 1);
      printf("LoadTrajectoryJ %s, rtn is: %d\n", traj_file_name, rtn);
      DescPose traj_start_pose;
      memset(&traj_start_pose, 0, sizeof(DescPose));
      rtn = robot.GetTrajectoryStartPose(traj_file_name, &traj_start_pose);
      printf("GetTrajectoryStartPose is: %d\n", rtn);
      printf("desc_pos:%f,%f,%f,%f,%f,%f\n", traj_start_pose.tran.x, traj_start_pose.tran.y, traj_start_pose.tran.z, traj_start_pose.rpy.rx, traj_start_pose.rpy.ry, traj_start_pose.rpy.rz);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      robot.SetSpeed(50);
      robot.MoveCart(&traj_start_pose, 0, 0, 100, 100, 100, -1, -1);
      int traj_num = 0;
      rtn = robot.GetTrajectoryPointNum(&traj_num);
      printf("GetTrajectoryStartPose rtn is: %d, traj num is: %d\n", rtn, traj_num);
      rtn = robot.SetTrajectoryJSpeed(50.0);
      printf("SetTrajectoryJSpeed is: %d\n", rtn);
      ForceTorque traj_force;
      memset(&traj_force, 0, sizeof(ForceTorque));
      traj_force.fx = 10;
      rtn = robot.SetTrajectoryJForceTorque(&traj_force);
      printf("SetTrajectoryJForceTorque rtn is: %d\n", rtn);
      rtn = robot.SetTrajectoryJForceFx(10.0);
      printf("SetTrajectoryJForceFx rtn is: %d\n", rtn);
      rtn = robot.SetTrajectoryJForceFy(0.0);
      printf("SetTrajectoryJForceFy rtn is: %d\n", rtn);
      rtn = robot.SetTrajectoryJForceFz(0.0);
      printf("SetTrajectoryJForceFz rtn is: %d\n", rtn);
      rtn = robot.SetTrajectoryJTorqueTx(10.0);
      printf("SetTrajectoryJTorqueTx rtn is: %d\n", rtn);
      rtn = robot.SetTrajectoryJTorqueTy(10.0);
      printf("SetTrajectoryJTorqueTy rtn is: %d\n", rtn);
      rtn = robot.SetTrajectoryJTorqueTz(10.0);
      printf("SetTrajectoryJTorqueTz rtn is: %d\n", rtn);
      rtn = robot.MoveTrajectoryJ();
      printf("MoveTrajectoryJ rtn is: %d\n", rtn);
      robot.CloseRPC();
      return 0;
    }

Trajectory Preprocessing (Trajectory Lookahead)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
	 * @brief  Trajectory preprocessing (Trajectory Lookahead)
	 * @param  [in] name  Trajectory file name
	 * @param  [in] mode Sampling mode, 0-No sampling; 1-Equal data interval sampling; 2-Equal error limit sampling
	 * @param  [in] errorLim Error limit, takes effect when using linear fitting
	 * @param  [in] type Smoothing method, 0-Bezier smoothing
	 * @param  [in] precision Smoothing precision, takes effect when using Bezier smoothing
	 * @param  [in] vamx Set maximum speed, mm/s
	 * @param  [in] amax Set maximum acceleration, mm/s2
	 * @param  [in] jmax Set maximum jerk, mm/s3
	 * @return  Error code
	 */
    errno_t LoadTrajectoryLA(char name[30], int mode, double errorLim, int type, double precision, double vamx, double amax, double jmax);

Trajectory Playback (Trajectory Lookahead)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @brief  Trajectory playback (Trajectory Lookahead)
    * @return  Error code
    */
    errno_t MoveTrajectoryLA();

Trajectory Playback (Trajectory Lookahead) Code Example
++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    int TestLoadTrajLA(void)
    {
      ROBOT_STATE_PKG pkg = {};
      FRRobot robot;
      robot.LoggerInit();
      robot.SetLoggerLevel(1);
      int rtn = robot.RPC("192.168.58.2");
      if (rtn != 0)
      {
        return -1;
      }
      robot.SetReConnectParam(true, 30000, 500);
      rtn = robot.TrajectoryJUpLoad("D://zUP/traj.txt");
      printf("Upload TrajectoryJ A %d\n", rtn);
      char traj_file_name[30] = "/fruser/traj/traj.txt";
      rtn = robot.LoadTrajectoryLA(traj_file_name, 1, 2, 0, 2, 100, 200, 1000);
      printf("LoadTrajectoryLA %s, rtn is: %d\n", traj_file_name, rtn);
      DescPose traj_start_pose;
      memset(&traj_start_pose, 0, sizeof(DescPose));
      rtn = robot.GetTrajectoryStartPose(traj_file_name, &traj_start_pose);
      printf("GetTrajectoryStartPose is: %d\n", rtn);
      printf("desc_pos:%f,%f,%f,%f,%f,%f\n", traj_start_pose.tran.x, traj_start_pose.tran.y, traj_start_pose.tran.z, traj_start_pose.rpy.rx, traj_start_pose.rpy.ry, traj_start_pose.rpy.rz);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      robot.SetSpeed(50);
      robot.MoveCart(&traj_start_pose, 0, 0, 100, 100, 100, -1, -1);
      rtn = robot.MoveTrajectoryLA();
      printf("MoveTrajectoryLA rtn is: %d\n", rtn);
      robot.CloseRPC();
      return 0;
    }