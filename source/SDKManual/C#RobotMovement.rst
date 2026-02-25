Robot Motion
=================

.. toctree:: 
    :maxdepth: 5
 
jog point motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /** 
    * @brief jog pointing 
    * @param [in] refType Type of pointing: 0-joint pointing, 2-pointing in base coordinate system, 4-pointing in tool coordinate system, 8-pointing in artifact coordinate system. 
    * @param [in] nb 1-joint 1 (or x-axis), 2-joint 2 (or y-axis), 3-joint 3 (or z-axis), 4-joint 4 (or rotate around x-axis), 5-joint 5 (or rotate around y-axis), 6-joint 6 (or rotate around z-axis)
    * @param [in] dir 0-negative direction, 1-positive direction 
    * @param [in] vel velocity percentage, [0~100] 
    * @param [in] acc acceleration percentage, [0~100] 
    * @param [in] max_dis Maximum angle of a single tap, in [°] or distance, in [mm] 
    * @return Error code 
    */ 
    int StartJOG(byte refType, byte nb, byte dir, float vel, float acc, float max_dis);

jog nudging deceleration stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief jog pointwise deceleration stops.
    * @param [in] ref 1-joint-point deceleration stop, 3-point deceleration stop in base coordinate system, 5-point deceleration stop in tool coordinate system, 9-point deceleration stop in workpiece coordinate system
    * @return Error code
    */
    int StopJOG(byte stopType).
 
jog pointing stops immediately
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief jog tapping stops immediately.
    * @return Error code
    */
    int ImmStopJOG(); 
 
Sample Robot Tap Control Code
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    private void btnJOG_Click(object sender, EventArgs e)
    {
        Robot robot = new Robot();
        robot.RPC("192.168.58.2"); 

        robot.SetSpeed(35);
        robot.StartJOG(0, 1, 0, 15, 20.0f, 30.0f);  
        robot.StopJOG(1);  
        //robot.ImmStopJOG(); 
        robot.StartJOG(0, 2, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(0, 3, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(0, 4, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(0, 5, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(0, 6, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();

        robot.StartJOG(2, 1, 0, 15, 20.0f, 30.0f);   
        Thread.Sleep(1000);
        robot.StopJOG(3);  
        //robot.ImmStopJOG(); 
        robot.StartJOG(2, 2, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(2, 3, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(2, 4, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(2, 5, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(2, 6, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();

        robot.StartJOG(4, 1, 0, 15, 20.0f, 30.0f);  
        Thread.Sleep(1000);
        robot.StopJOG(5); 
        //robot.ImmStopJOG(); 
        robot.StartJOG(4, 2, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(4, 3, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(4, 4, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(4, 5, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(4, 6, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();

        robot.StartJOG(8, 1, 0, 15, 20.0f, 30.0f);  
        Thread.Sleep(1000);
        robot.StopJOG(9);  
        //robot.ImmStopJOG();  
        robot.StartJOG(8, 2, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(8, 3, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(8, 4, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(8, 5, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
        robot.StartJOG(8, 6, 1, 15, 20.0f, 30.0f);
        Thread.Sleep(1000);
        robot.ImmStopJOG();
    }
 
Joint space motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Joint space motion.
    * @param [in] joint_pos Target joint position in degrees.
    * @param [in] desc_pos Target Cartesian position.
    * @param [in] tool Tool coordinate number, range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100] * @param [in] acc.
    * @param [in] acc Acceleration percentage, range [0~100], not available yet.
    * @param [in] ovl Velocity scaling factor, range [0~100].
    * @param [in] epos Extended axis position in mm.
    * @param [in] blendT [-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in ms
    * @param [in] offset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos Bit position offset
    * @return Error code
    */
    int MoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos epos, float blendT, byte offset_flag, DescPose offset_pos); 

Joint space motion (automatic forward kinematics calculation)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /** 
    * @brief Joint space motion (automatic forward kinematics calculation)
    * @param [in] joint_pos  Target joint position, unit deg
    * @param [in] tool  Tool coordinate number, range [0~14]
    * @param [in] user  Workpiece coordinate number, range [0~14]
    * @param [in] vel  Velocity percentage, range [0~100]
    * @param [in] acc  Acceleration percentage, range [0~100], not open yet
    * @param [in] ovl  Velocity scaling factor, range [0~100]
    * @param [in] epos  Extended axis position, unit mm
    * @param [in] blendT [-1.0]-move to position (blocking), [0~500.0]-smoothing time (non-blocking), unit ms
    * @param [in] offset_flag  0-no offset, 1-offset in base/workpiece coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos  Pose offset
    * @return Error code
    */ 
    int MoveJ(JointPos joint_pos, int tool, int user, double vel, double acc, double ovl, ExaxisPos epos, double blendT, int offset_flag, DescPose offset_pos)

Linear motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Cartesian space linear motion
    * @param [in] joint_pos Target joint position, unit deg
    * @param [in] desc_pos Target Cartesian pose
    * @param [in] tool Tool coordinate number, range [0~14]
    * @param [in] user Workpiece coordinate number, range [0~14]
    * @param [in] vel Speed percentage, range [0~100]
    * @param [in] acc Acceleration percentage, range [0~100], currently not available
    * @param [in] ovl Speed scaling factor [0~100]/Physical speed (mm/s)
    * @param [in] blendR [-1.0]-Motion to position (blocking), [0~1000.0]-Blend radius (non-blocking), unit mm
    * @param [in] blendMode Transition mode; 0-Inscribed transition; 1-Corner transition
    * @param [in] epos Extended axis position, unit mm
    * @param [in] search 0-No wire seam tracking, 1-Wire seam tracking
    * @param [in] offset_flag 0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos Pose offset
    * @param [in] oacc Acceleration scaling factor [0-100]/Physical acceleration (mm/s²)
    * @param [in] velAccParamMode Speed/acceleration parameter mode; 0-Percentage; 1-Physical speed (mm/s) and acceleration (mm/s²)
    * @param [in] overSpeedStrategy Overspeed handling strategy, 1-Standard; 2-Error and stop on overspeed; 3-Adaptive speed reduction, default is 0
    * @param [in] speedPercent Allowable speed reduction threshold percentage [0-100], default 10%
    * @return Error code
    */
    public int MoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int blendMode, ExaxisPos epos, int search, int offset_flag, DescPose offset_pos, float oacc, int velAccParamMode, int overSpeedStrategy = 0, int speedPercent = 10)
 
Cartesian space linear motion (automatic inverse kinematics calculation)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Cartesian space linear motion (automatic inverse kinematics calculation)
    * @param [in] desc_pos  Target cartesian pose
    * @param [in] tool  Tool coordinate number, range [1~15]
    * @param [in] user  Workpiece coordinate number, range [1~15]
    * @param [in] vel  Velocity percentage, range [0~100]
    * @param [in] acc  Acceleration percentage, range [0~100], not open yet
    * @param [in] ovl  Velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-move to position (blocking), [0~1000.0]-smoothing radius (non-blocking), unit mm
    * @param [in] blendMode Transition mode; 0-tangent transition; 1-corner transition
    * @param [in] epos  Extended axis position, unit mm
    * @param [in] search  0-no wire search, 1-wire search
    * @param [in] offset_flag  0-no offset, 1-offset in base/workpiece coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos  Pose offset
    * @param [in] config Inverse kinematics joint space configuration, [-1]-calculate based on current joint position, [0~7]-solve according to specific joint space configuration
    * @param [in] overSpeedStrategy  Overspeed handling strategy, 1-standard; 2-stop on error when overspeed; 3-adaptive speed reduction, default is 0
    * @param [in] speedPercent  Allowed speed reduction threshold percentage [0-100], default 10%
    * @return Error code
    */
    int MoveL(DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendR, int blendMode, ExaxisPos epos, int search, int offset_flag, DescPose offset_pos, int config, int overSpeedStrategy, int speedPercent)

Cartesian Space Linear Motion (Added velAccParamMode parameter for velocity and acceleration modes)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief  Cartesian Space Linear Motion (Added velAccParamMode parameter for velocity and acceleration modes)
    * @param  [in] joint_pos  Target joint position, unit deg
    * @param  [in] desc_pos   Target Cartesian pose
    * @param  [in] tool  Tool coordinate number, range [1~15]
    * @param  [in] user  Workpiece coordinate number, range [1~15]
    * @param  [in] vel  Velocity percentage, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not yet open
    * @param  [in] ovl  Velocity scaling factor, range [0~100]
    * @param  [in] blendR [-1.0]-Motion complete (blocking), [0~1000.0]-Blending radius (non-blocking), unit mm
    * @param  [in] epos  Extended axis position, unit mm
    * @param  [in] search  0-No wire search, 1-Wire search
    * @param  [in] offset_flag  0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param  [in] offset_pos  Pose offset
    * @param  [in] velAccParamMode Velocity and acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) and acceleration (mm/s^2)
    * @param  [in] overSpeedStrategy  Overspeed handling strategy, 1-Standard; 2-Stop with error on overspeed; 3-Adaptive deceleration, default is 0
    * @param  [in] speedPercent  Allowed deceleration threshold percentage [0-100], default 10%
    * @return  Error code
    */
    public int MoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendR, ExaxisPos epos, int search, int offset_flag, DescPose offset_pos, int velAccParamMode, int overSpeedStrategy, int speedPercent)

Cartesian Space Linear Motion (Overload Function 1, Added blendMode)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief  Cartesian Space Linear Motion (Overload Function 1, Added blendMode)
    * @param  [in] joint_pos  Target joint position, unit deg
    * @param  [in] desc_pos   Target Cartesian pose
    * @param  [in] tool  Tool coordinate number, range [1~15]
    * @param  [in] user  Workpiece coordinate number, range [1~15]
    * @param  [in] vel  Velocity percentage, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not yet open
    * @param  [in] ovl  Velocity scaling factor, range [0~100]
    * @param  [in] blendR [-1.0]-Motion complete (blocking), [0~1000.0]-Blending radius (non-blocking), unit mm
    * @param  [in] blendMode Transition mode; 0-Tangent transition; 1-Corner transition
    * @param  [in] epos  Extended axis position, unit mm
    * @param  [in] search  0-No wire search, 1-Wire search
    * @param  [in] offset_flag  0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param  [in] offset_pos  Pose offset
    * @param  [in] velAccParamMode Velocity and acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) and acceleration (mm/s^2)
    * @param  [in] overSpeedStrategy  Overspeed handling strategy, 1-Standard; 2-Stop with error on overspeed; 3-Adaptive deceleration, default is 0
    * @param  [in] speedPercent  Allowed deceleration threshold percentage [0-100], default 10%
    * @return  Error code
    */
    public int MoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendR, int blendMode, ExaxisPos epos, int search, int offset_flag, DescPose offset_pos, int velAccParamMode, int overSpeedStrategy, int speedPercent)

Cartesian Space Linear Motion (Overload Function 2, No Joint Position Input Required)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief  Cartesian Space Linear Motion (Overload Function 2, No Joint Position Input Required)
    * @param  [in] desc_pos   Target Cartesian pose
    * @param  [in] tool  Tool coordinate number, range [1~15]
    * @param  [in] user  Workpiece coordinate number, range [1~15]
    * @param  [in] vel  Velocity percentage, range [0~100]
    * @param  [in] acc  Acceleration percentage, range [0~100], not yet open
    * @param  [in] ovl  Velocity scaling factor, range [0~100]
    * @param  [in] blendR [-1.0]-Motion complete (blocking), [0~1000.0]-Blending radius (non-blocking), unit mm
    * @param  [in] blendMode Transition mode; 0-Tangent transition; 1-Corner transition
    * @param  [in] epos  Extended axis position, unit mm
    * @param  [in] search  0-No wire search, 1-Wire search
    * @param  [in] offset_flag  0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param  [in] offset_pos  Pose offset
    * @param  [in] config Inverse kinematic joint space configuration, [-1]-Reference current joint position for calculation, [0~7]-Solve based on specific joint space configuration
    * @param  [in] velAccParamMode Velocity and acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) and acceleration (mm/s^2)
    * @param  [in] overSpeedStrategy  Overspeed handling strategy, 1-Standard; 2-Stop with error on overspeed; 3-Adaptive deceleration, default is 0
    * @param  [in] speedPercent  Allowed deceleration threshold percentage [0-100], default 10%
    * @return  Error code
    */
    public int MoveL(DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendR, int blendMode, ExaxisPos epos, int search, int offset_flag, DescPose offset_pos, int config, int velAccParamMode, int overSpeedStrategy, int speedPercent)

Circular motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Cartesian circular motion.
    * @param [in] joint_pos_p Pathpoint joint position in deg.
    * @param [in] desc_pos_p Path point Cartesian position.
    * @param [in] ptool tool coordinate number, range [0~14].
    * @param [in] puser workpiece coordinate number, range [0~14]
    * @param [in] pvel speed percentage, range [0~100]
    * @param [in] pacc Acceleration percentage, range [0~100], not available yet.
    * @param [in] epos_p Extended axis position, in mm
    * @param [in] poffset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos_p Positional offset
    * @param [in] joint_pos_t Joint position of target point, in deg.
    * @param [in] desc_pos_t Cartesian position of target point.
    * @param [in] ttool tool coordinate number, range [0~14].
    * @param [in] tuser Workpiece coordinate number, range [0~14].
    * @param [in] tvel Velocity percentage, range [0~100]
    * @param [in] tacc Acceleration percentage, range [0~100], not available yet.
    * @param [in] epos_t Extended axis position in mm.
    * @param [in] toffset_flag 0-no offset, 1-offset in base/work coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos_t Bit position offset   
    * @param [in] ovl velocity scaling factor, range [0~100]    
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking) in mm    
    * @param  [in] oacc Acceleration scaling factor [0-100]/Physical acceleration (mm/s²)
    * @param  [in] velAccParamMode Speed/acceleration parameter mode; 0-Percentage; 1-Physical speed (mm/s) and acceleration (mm/s²)
    * @return  Error code
    */
    public int MoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc,ExaxisPos epos_p, int poffset_flag, DescPose offset_pos_p,JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc,ExaxisPos epos_t, int toffset_flag, DescPose offset_pos_t,float ovl, float blendR, float oacc, int velAccParamMode)

Cartesian space circular motion (automatic inverse kinematics calculation)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Cartesian space circular motion (automatic inverse kinematics calculation)
    * @param [in] desc_pos_p  Path point cartesian pose
    * @param [in] ptool  Tool coordinate number, range [1~15]
    * @param [in] puser  Workpiece coordinate number, range [1~15]
    * @param [in] pvel  Velocity percentage, range [0~100]
    * @param [in] pacc  Acceleration percentage, range [0~100], not open yet
    * @param [in] epos_p  Extended axis position, unit mm
    * @param [in] poffset_flag  0-no offset, 1-offset in base/workpiece coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos_p  Pose offset
    * @param [in] desc_pos_t  Target point cartesian pose
    * @param [in] ttool  Tool coordinate number, range [1~15]
    * @param [in] tuser  Workpiece coordinate number, range [1~15]
    * @param [in] tvel  Velocity percentage, range [0~100]
    * @param [in] tacc  Acceleration percentage, range [0~100], not open yet
    * @param [in] epos_t  Extended axis position, unit mm
    * @param [in] toffset_flag  0-no offset, 1-offset in base/workpiece coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos_t  Pose offset
    * @param [in] ovl  Velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-move to position (blocking), [0~1000.0]-smoothing radius (non-blocking), unit mm
    * @param [in] config Inverse kinematics joint space configuration, [-1]-calculate based on current joint position, [0~7]-solve according to specific joint space configuration
    * @return Error code
    */
    int MoveC(DescPose desc_pos_p, int ptool, int puser, double pvel, double pacc, ExaxisPos epos_p, int poffset_flag, DescPose offset_pos_p, DescPose desc_pos_t, int ttool, int tuser, double tvel, double tacc, ExaxisPos epos_t, int toffset_flag, DescPose offset_pos_t, double ovl, double blendR, int config)

Cartesian Space Arc Motion (Added velAccParamMode parameter for velocity and acceleration modes)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief  Cartesian Space Arc Motion (Added velAccParamMode parameter for velocity and acceleration modes)
    * @param  [in] joint_pos_p  Path point joint position, unit deg
    * @param  [in] desc_pos_p   Path point Cartesian pose
    * @param  [in] ptool  Tool coordinate number, range [1~15]
    * @param  [in] puser  Workpiece coordinate number, range [1~15]
    * @param  [in] pvel  Velocity percentage, range [0~100]
    * @param  [in] pacc  Acceleration percentage, range [0~100], not yet open
    * @param  [in] epos_p  Extended axis position, unit mm
    * @param  [in] poffset_flag  0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param  [in] offset_pos_p  Pose offset
    * @param  [in] joint_pos_t  Target point joint position, unit deg
    * @param  [in] desc_pos_t   Target point Cartesian pose
    * @param  [in] ttool  Tool coordinate number, range [1~15]
    * @param  [in] tuser  Workpiece coordinate number, range [1~15]
    * @param  [in] tvel  Velocity percentage, range [0~100]
    * @param  [in] tacc  Acceleration percentage, range [0~100], not yet open
    * @param  [in] epos_t  Extended axis position, unit mm
    * @param  [in] toffset_flag  0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param  [in] offset_pos_t  Pose offset
    * @param  [in] ovl  Velocity scaling factor, range [0~100]
    * @param  [in] blendR [-1.0]-Motion complete (blocking), [0~1000.0]-Blending radius (non-blocking), unit mm
    * @param  [in] velAccParamMode Velocity and acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) and acceleration (mm/s^2)
    * @return  Error code
    */
    public int MoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, double pvel, double pacc, ExaxisPos epos_p, int poffset_flag, DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, double tvel, double tacc, ExaxisPos epos_t, int toffset_flag, DescPose offset_pos_t, double ovl, double blendR, int velAccParamMode)

Cartesian Space Arc Motion (Overload Function 1, No Joint Position Input Required)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief  Cartesian Space Arc Motion (Overload Function 1, No Joint Position Input Required)
    * @param  [in] desc_pos_p   Path point Cartesian pose
    * @param  [in] ptool  Tool coordinate number, range [1~15]
    * @param  [in] puser  Workpiece coordinate number, range [1~15]
    * @param  [in] pvel  Velocity percentage, range [0~100]
    * @param  [in] pacc  Acceleration percentage, range [0~100], not yet open
    * @param  [in] epos_p  Extended axis position, unit mm
    * @param  [in] poffset_flag  0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param  [in] offset_pos_p  Pose offset
    * @param  [in] desc_pos_t   Target point Cartesian pose
    * @param  [in] ttool  Tool coordinate number, range [1~15]
    * @param  [in] tuser  Workpiece coordinate number, range [1~15]
    * @param  [in] tvel  Velocity percentage, range [0~100]
    * @param  [in] tacc  Acceleration percentage, range [0~100], not yet open
    * @param  [in] epos_t  Extended axis position, unit mm
    * @param  [in] toffset_flag  0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param  [in] offset_pos_t  Pose offset
    * @param  [in] ovl  Velocity scaling factor, range [0~100]
    * @param  [in] blendR [-1.0]-Motion complete (blocking), [0~1000.0]-Blending radius (non-blocking), unit mm
    * @param  [in] config Inverse kinematic joint space configuration, [-1]-Reference current joint position for calculation, [0~7]-Solve based on specific joint space configuration
    * @param  [in] velAccParamMode Velocity and acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) and acceleration (mm/s^2)
    * @return  Error code
    */
    public int MoveC(DescPose desc_pos_p, int ptool, int puser, double pvel, double pacc, ExaxisPos epos_p, int poffset_flag, DescPose offset_pos_p, DescPose desc_pos_t, int ttool, int tuser, double tvel, double tacc, ExaxisPos epos_t, int toffset_flag, DescPose offset_pos_t, double ovl, double blendR, int config, int velAccParamMode)

Point-to-point motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /** 
    * @brief Cartesian space point-to-point motion. 
    * @param [in] desc_pos Cartesian position of target in base coordinate system. 
    * @param [in] tool tool coordinate number, range [0~14] 
    * @param [in] user Workpiece coordinate number, range [0~14]. 
    * @param [in] vel velocity percentage, range [0~100] * @param [in] acc [in] tool coordinate number, range [0~14] 
    * @param [in] acc Acceleration percentage, range [0~100], not available yet. 
    * @param [in] ovl velocity scaling factor, range [0~100] 
    * @param [in] blendT [-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in ms 
    * @param [in] config Joint space configuration, [-1]-Refer to current joint position, [0~7]-Refer to specific joint space configuration, default is -1. 
    * @return Error code 
    */ 
    int MoveCart(DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendT, int config);
 
Whole circle motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.4  Web-3.8.3
    
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Whole circle motion in Cartesian space.
    * @param [in] joint_pos_p path point 1 joint position in deg.
    * @param [in] desc_pos_p path point 1 Cartesian position.
    * @param [in] ptool tool coordinate number, range [0~14].
    * @param [in] puser workpiece coordinate number, range [0~14]
    * @param [in] pvel velocity percentage, range [0~100]
    * @param [in] pacc Acceleration percentage, range [0~100], not available yet.
    * @param [in] epos_p extended axis position in mm
    * @param [in] joint_pos_t Path point 2 joint position in deg.
    * @param [in] desc_pos_t Path point 2 Cartesian position in deg.
    * @param [in] ttool tool coordinate number, range [0~14].
    * @param [in] tuser Workpiece coordinate number, range [0~14]
    * @param [in] tvel Velocity percentage, range [0~100]
    * @param [in] tacc Acceleration percentage, range [0~100], not available yet.
    * @param [in] epos_t Extended axis position in mm.
    * @param [in] ovl  Speed scaling factor [0~100]/Physical speed (mm/s)
    * @param [in] offset_flag  0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param [in] offset_pos  Pose offset
    * @param [in] oacc Acceleration scaling factor [0-100]/Physical acceleration (mm/s²)
    * @param [in] blendR -1: Blocking; 0~1000: Blend radius
    * @param [in] velAccParamMode Speed/acceleration parameter mode; 0-Percentage; 1-Physical speed (mm/s) and acceleration (mm/s²)
    * @return  Error code
    */     
    public int Circle(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc,ExaxisPos epos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser,float tvel, float tacc, ExaxisPos epos_t, float ovl, int offset_flag,DescPose offset_pos, double oacc, double blendR, int velAccParamMode)

Cartesian space full circle motion (automatic inverse kinematics calculation)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Cartesian space full circle motion (automatic inverse kinematics calculation)
    * @param [in] desc_pos_p  Path point 1 cartesian pose
    * @param [in] ptool  Tool coordinate number, range [0~14]
    * @param [in] puser  Workpiece coordinate number, range [0~14]
    * @param [in] pvel  Velocity percentage, range [0~100]
    * @param [in] pacc  Acceleration percentage, range [0~100], not open yet
    * @param [in] epos_p  Extended axis position, unit mm
    * @param [in] desc_pos_t  Path point 2 cartesian pose
    * @param [in] ttool  Tool coordinate number, range [0~14]
    * @param [in] tuser  Workpiece coordinate number, range [0~14]
    * @param [in] tvel  Velocity percentage, range [0~100]
    * @param [in] tacc  Acceleration percentage, range [0~100], not open yet
    * @param [in] epos_t  Extended axis position, unit mm
    * @param [in] ovl  Velocity scaling factor, range [0~100]
    * @param [in] offset_flag  0-no offset, 1-offset in base/workpiece coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos  Pose offset
    * @param [in] oacc Acceleration percentage
    * @param [in] blendR -1: blocking; 0~1000: smoothing radius
    * @param [in] config Inverse kinematics joint space configuration, [-1]-calculate based on current joint position, [0~7]-solve according to specific joint space configuration
    * @return Error code
    */
    int Circle(DescPose desc_pos_p, int ptool, int puser, double pvel, double pacc, ExaxisPos epos_p, DescPose desc_pos_t, int ttool, int tuser, double tvel, double tacc, ExaxisPos epos_t, double ovl, int offset_flag, DescPose offset_pos, double oacc, double blendR,int config)

Cartesian Space Full Circle Motion (Added velAccParamMode parameter for velocity and acceleration modes)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    *@brief  Cartesian Space Full Circle Motion (Added velAccParamMode parameter for velocity and acceleration modes)
    *@param  [in] joint_pos_p  Path point 1 joint position, unit deg
    *@param  [in] desc_pos_p   Path point 1 Cartesian pose
    *@param  [in] ptool  Tool coordinate number, range [1~15]
    *@param  [in] puser  Workpiece coordinate number, range [1~15]
    *@param  [in] pvel  Velocity percentage, range [0~100]
    *@param  [in] pacc  Acceleration percentage, range [0~100], not yet open
    *@param  [in] epos_p  Extended axis position, unit mm
    *@param  [in] joint_pos_t  Path point 2 joint position, unit deg
    *@param  [in] desc_pos_t   Path point 2 Cartesian pose
    *@param  [in] ttool  Tool coordinate number, range [1~15]
    *@param  [in] tuser  Workpiece coordinate number, range [1~15]
    *@param  [in] tvel  Velocity percentage, range [0~100]
    *@param  [in] tacc  Acceleration percentage, range [0~100], not yet open
    *@param  [in] epos_t  Extended axis position, unit mm
    *@param  [in] ovl  Velocity scaling factor, range [0~100]
    *@param  [in] offset_flag  0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
    *@param  [in] offset_pos  Pose offset
    *@param  [in] oacc Acceleration percentage
    *@param  [in] blendR -1: Blocking; 0~1000: Blending radius
    *@param  [in] velAccParamMode Velocity and acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) and acceleration (mm/s^2)
    *@return  Error code
    */
    public int Circle(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, double pvel, double pacc, ExaxisPos epos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, double tvel, double tacc, ExaxisPos epos_t, double ovl, int offset_flag, DescPose offset_pos, double oacc, double blendR, int velAccParamMode)

Cartesian Space Full Circle Motion (Overload Function 1, No Joint Position Input Required)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief  Cartesian Space Full Circle Motion (Overload Function 1, No Joint Position Input Required)
    * @param  [in] desc_pos_p   Path point 1 Cartesian pose
    * @param  [in] ptool  Tool coordinate number, range [0~14]
    * @param  [in] puser  Workpiece coordinate number, range [0~14]
    * @param  [in] pvel  Velocity percentage, range [0~100]
    * @param  [in] pacc  Acceleration percentage, range [0~100], not yet open
    * @param  [in] epos_p  Extended axis position, unit mm
    * @param  [in] desc_pos_t   Path point 2 Cartesian pose
    * @param  [in] ttool  Tool coordinate number, range [0~14]
    * @param  [in] tuser  Workpiece coordinate number, range [0~14]
    * @param  [in] tvel  Velocity percentage, range [0~100]
    * @param  [in] tacc  Acceleration percentage, range [0~100], not yet open
    * @param  [in] epos_t  Extended axis position, unit mm
    * @param  [in] ovl  Velocity scaling factor, range [0~100]
    * @param  [in] offset_flag  0-No offset, 1-Offset in base/workpiece coordinate system, 2-Offset in tool coordinate system
    * @param  [in] offset_pos  Pose offset
    * @param  [in] oacc Acceleration percentage
    * @param  [in] blendR -1: Blocking; 0~1000: Blending radius
    * @param  [in] config Inverse kinematic joint space configuration, [-1]-Reference current joint position for calculation, [0~7]-Solve based on specific joint space configuration
    * @param  [in] velAccParamMode Velocity and acceleration parameter mode; 0-Percentage; 1-Physical velocity (mm/s) and acceleration (mm/s^2)
    * @return  Error code
    */
    public int Circle(DescPose desc_pos_p, int ptool, int puser, double pvel, double pacc, ExaxisPos epos_p, DescPose desc_pos_t, int ttool, int tuser, double tvel, double tacc, ExaxisPos epos_t, double ovl, int offset_flag, DescPose offset_pos, double oacc, double blendR, int config, int velAccParamMode)

Sample Code for Whole Circle Motion in Cartesian Space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.4  Web-3.8.3
    
.. code-block:: c#
    :linenos:
 
    private void btnMovetest_Click(object sender, EventArgs e)
    {
        int rtn = 0;
        DescPose middescPoseCir1 = new DescPose(-435.414, -342.926, 309.205, -171.382, -4.513, 171.520);
        JointPos midjointPosCir1 = new JointPos(26.804, -79.866, 106.642, -125.433, -85.562, -54.721);
        DescPose enddescPoseCir1 = new DescPose(-524.862, -217.402, 308.459, -171.425, -4.810, 156.088);
        JointPos endjointPosCir1 = new JointPos(11.399, -78.055, 104.603, -125.421, -85.770, -54.721);

        DescPose middescPoseCir2 = new DescPose(-482.691, -587.899, 318.594, -171.001, -4.999, -172.996);
        JointPos midjointPosCir2 = new JointPos(42.314, -53.600, 67.296, -112.969, -85.533, -54.721);
        DescPose enddescPoseCir2 = new DescPose(-403.942, -489.061, 317.038, -163.189, -10.425, -175.627);
        JointPos endjointPosCir2 = new JointPos(39.959, -70.616, 96.679, -134.243, -82.276, -54.721);

        DescPose middescPoseMoveC = new DescPose(-435.414, -342.926, 309.205, -171.382, -4.513, 171.520);
        JointPos midjointPosMoveC = new JointPos(26.804, -79.866, 106.642, -125.433, -85.562, -54.721);
        DescPose enddescPoseMoveC = new DescPose(-524.862, -217.402, 308.459, -171.425, -4.810, 156.088);
        JointPos endjointPosmoveC = new JointPos(11.399, -78.055, 104.603, -125.421, -85.770, -54.721);

        DescPose middescPoseCir3 = new DescPose(-435.414, -342.926, 309.205, -171.382, -4.513, 171.520);
        JointPos midjointPosCir3 = new JointPos(26.804, -79.866, 106.642, -125.433, -85.562, -54.721);
        DescPose enddescPoseCir3 = new DescPose(-569.505, -405.378, 357.596, -172.862, -10.939, 171.108);
        JointPos endjointPosCir3 = new JointPos(27.138, -63.750, 78.586, -117.861, -90.588, -54.721);

        DescPose middescPoseCir4 = new DescPose(-482.691, -587.899, 318.594, -171.001, -4.999, -172.996);
        JointPos midjointPosCir4 = new JointPos(42.314, -53.600, 67.296, -112.969, -85.533, -54.721);
        DescPose enddescPoseCir4 = new DescPose(-569.505, -405.378, 357.596, -172.862, -10.939, 171.108);
        JointPos endjointPosCir4 = new JointPos(27.138, -63.750, 78.586, -117.861, -90.588, -54.721);

        DescPose startdescPose = new DescPose(-569.505, -405.378, 357.596, -172.862, -10.939, 171.108);
        JointPos startjointPos = new JointPos(27.138, -63.750, 78.586, -117.861, -90.588, -54.721);

        DescPose linedescPose = new DescPose(-403.942, -489.061, 317.038, -163.189, -10.425, -175.627);
        JointPos linejointPos = new JointPos(39.959, -70.616, 96.679, -134.243, -82.276, -54.721);


        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);


        robot.MoveJ(startjointPos, startdescPose, 3, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        rtn = robot.Circle(midjointPosCir1, middescPoseCir1, 3, 0, 100, 100, exaxisPos, endjointPosCir1, enddescPoseCir1, 3, 0, 100, 100, exaxisPos, 100, -1, offdese, 100, 20);
        Console.WriteLine("Circle1" + rtn);
        rtn = robot.Circle(midjointPosCir2, middescPoseCir2, 3, 0, 100, 100, exaxisPos, endjointPosCir2, enddescPoseCir2, 3, 0, 100, 100, exaxisPos, 100, -1, offdese, 100, 20);
        Console.WriteLine("Circle2" + rtn);

        robot.MoveC(midjointPosMoveC, middescPoseMoveC, 3, 0, 100, 100, exaxisPos, 0, offdese, endjointPosmoveC, enddescPoseMoveC, 3, 0, 100, 100, exaxisPos, 0, offdese, 100, 20);
        rtn = robot.Circle(midjointPosCir3, middescPoseCir3, 3, 0, 100, 100, exaxisPos, endjointPosCir3, enddescPoseCir3, 3, 0, 100, 100, exaxisPos, 100, -1, offdese, 100, 20);
        Console.WriteLine("Circle3" + rtn);
        rtn = robot.MoveL(linejointPos, linedescPose, 3, 0, 100, 100, 100, -1, 0, exaxisPos, 0, 0, offdese);
        Console.WriteLine("MoveL " + rtn);
        rtn = robot.Circle(midjointPosCir4, middescPoseCir4, 3, 0, 100, 100, exaxisPos, endjointPosCir4, enddescPoseCir4, 3, 0, 100, 100, exaxisPos, 100, -1, offdese, 100, 20);
        Console.WriteLine("Circle4" + rtn);
    }
 
Sample code for basic robot motion instructions
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    public void TestMove()
        int rtn;
        JointPos j1 = new JointPos(-11.904f, -99.669f, 117.473f, -108.616f, -91.726f, 74.256f);
        JointPos j2 = new JointPos(-45.615f, -106.172f, 124.296f, -107.151f, -91.282f, 74.255f);
        JointPos j3 = new JointPos(-29.777f, -84.536f, 109.275f, -114.075f, -86.655f, 74.257f);
        JointPos j4 = new JointPos(-31.154f, -95.317f, 94.276f, -88.079f, -89.740f, 74.256f);
        DescPose desc_pos1 = new DescPose(-419.524f, -13.000f, 351.569f, -178.118f, 0.314f, 3.833f);
        DescPose desc_pos2 = new DescPose(-321.222f, 185.189f, 335.520f, -179.030f, -1.284f, -29.869f);
        DescPose desc_pos3 = new DescPose(-487.434f, 154.362f, 308.576f, 176.600f, 0.268f, -14.061f);
        DescPose desc_pos4 = new DescPose(-443.165f, 147.881f, 480.951f, 179.511f, -0.775f, -15.409f);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float oacc = 100.0f;
        float blendT = 0.0f;
        float blendR = 0.0f;
        byte flag = 0;
        byte search = 0;
        int blendMode = 0;
        int velAccMode = 0;
        robot.SetSpeed(20);
        rtn = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Console.WriteLine($"movej errcode:{rtn}");
        rtn = robot.MoveL(j2, desc_pos2, tool, user, vel, acc, ovl, blendR, blendMode, epos, search, flag, offset_pos, oacc, velAccMode,0,10);
        Console.WriteLine($"movel errcode:{rtn}");
        rtn = robot.MoveC(j3, desc_pos3, tool, user, vel, acc, epos, flag, offset_pos,j4, desc_pos4, tool, user, vel, acc, epos, flag, offset_pos, ovl, blendR, oacc, velAccMode);
        Console.WriteLine($"movec errcode:{rtn}");
        rtn = robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Console.WriteLine($"movej errcode:{rtn}");
        rtn = robot.Circle(j3, desc_pos3, tool, user, vel, acc, epos,j1, desc_pos1, tool, user, vel, acc, epos,ovl, flag, offset_pos, oacc, -1, velAccMode);
        Console.WriteLine($"circle errcode:{rtn}");
        rtn = robot.MoveCart(desc_pos4, tool, user, vel, acc, ovl, blendT, -1);
        Console.WriteLine($"MoveCart errcode:{rtn}");
        rtn = robot.MoveJ(j1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Console.WriteLine($"movej errcode:{rtn}");
        rtn = robot.MoveL(desc_pos2, tool, user, vel, acc, ovl, blendR, blendMode, epos, search, flag, offset_pos, -1, velAccMode);
        Console.WriteLine($"movel errcode:{rtn}");
        rtn = robot.MoveC(desc_pos3, tool, user, vel, acc, epos, flag, offset_pos,desc_pos4, tool, user, vel, acc, epos, flag, offset_pos,ovl, blendR, -1, velAccMode);
        Console.WriteLine($"movec errcode:{rtn}");
        rtn = robot.MoveJ(j2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Console.WriteLine($"movej errcode:{rtn}");
        rtn = robot.Circle(desc_pos3, tool, user, vel, acc, epos, desc_pos1, tool, user, vel, acc, epos,ovl, flag, offset_pos, oacc, blendR, -1, velAccMode);
        Console.WriteLine($"circle errcode:{rtn}");
    }
 
Spiral motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Cartesian spiral motion. 
    * @param [in] joint_pos target joint position in deg 
    * @param [in] desc_pos target cartesian position in deg 
    * @param [in] tool Tool coordinate number, range [0~14]. 
    * @param [in] user Workpiece coordinate number, range [0~14]. 
    * @param [in] vel velocity percentage, range [0~100] * @param [in] acc [in] tool coordinate number, range [0~14] 
    * @param [in] acc Acceleration percentage, range [0~100], not available yet. 
    * @param [in] epos extended axis position in mm 
    * @param [in] ovl Velocity scaling factor, range [0~100]. 
    * @param [in] offset_flag 0-no offset, 1- offset in base/work coordinate system, 2- offset in tool coordinate system 
    * @param [in] offset_pos Bit position offset 
    * @param [in] spiral_param spiral_param 
    * @return Error code 
    */
    int NewSpiral(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, ExaxisPos epos, float ovl, byte offset_flag, DescPose offset_pos, SpiralParam spiral_param); 

Cartesian space spiral motion (automatic inverse kinematics calculation)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Cartesian space spiral motion (automatic inverse kinematics calculation)
    * @param [in] desc_pos  Target cartesian pose
    * @param [in] tool  Tool coordinate number, range [0~14]
    * @param [in] user  Workpiece coordinate number, range [0~14]
    * @param [in] vel  Velocity percentage, range [0~100]
    * @param [in] acc  Acceleration percentage, range [0~100], not open yet
    * @param [in] epos  Extended axis position, unit mm
    * @param [in] ovl  Velocity scaling factor, range [0~100]
    * @param [in] offset_flag  0-no offset, 1-offset in base/workpiece coordinate system, 2-offset in tool coordinate system
    * @param [in] offset_pos  Pose offset
    * @param [in] spiral_param  Spiral parameters
    * @param [in] config Inverse kinematics joint space configuration, [-1]-calculate based on current joint position, [0~7]-solve according to specific joint space configuration
    * @return Error code
    */
    int NewSpiral(DescPose desc_pos, int tool, int user, double vel, double acc, ExaxisPos epos, double ovl, int offset_flag, DescPose offset_pos, SpiralParam spiral_param,int config)

Sample code for spiral motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    private void btnDescSpiral_Click(object sender, EventArgs e)
    {
        int rtn;
        JointPos j = new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        DescPose desc_pos = new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose offset_pos1 = new DescPose(50, 0, 0, -30, 0, 0);
        DescPose offset_pos2 = new DescPose(50, 0, 0, -5, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
        SpiralParam sp;
        sp.circle_num = 5;
        sp.circle_angle = 5.0f;
        sp.rad_init = 50.0f;
        sp.rad_add = 10.0f;
        sp.rotaxis_add = 10.0f;
        sp.rot_direction = 0;

        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = 0.0f;
        byte flag = 2;

        robot.SetSpeed(20);

        rtn = robot.MoveJ(j, desc_pos, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos1);
        Console.WriteLine($"MoveJ errcode:{rtn}");

        rtn = robot.NewSpiral(j, desc_pos, tool, user, vel, acc, epos, ovl, flag, offset_pos2, sp);
        Console.WriteLine($"NewSpiral errcode:{rtn}");
    }
 
Starting servo motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /** 
    * @brief Servo motion start, used with ServoJ, ServoCart instructions.
    * @return Error code. 
    */ 
    int ServoMoveStart();
 
ServoMoveStart
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /** 
    * @brief End of servo motion, used with ServoJ, ServoCart instructions.
    * @return Error code. 
    */ 
    int ServoMoveEnd().
 
Joint space servo mode movement
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.4  Web-3.8.3
    
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Joint space servo mode motion
    * @param [in] joint_pos Target joint position in degrees.
    * @param [in] acc Acceleration percentage, range [0~100], not open, default is 0.
    * @param [in] vel Speed percentage, range[0~100], not open, default is 0.
    * @param [in] cmdT Command send cycle, unit s, recommended range [0.001~0.0016].
    * @param [in] filterT filter time, unit s, not open, default is 0
    * @param [in] gain proportional amplifier for target position, not open, default is 0
    * @param [in] id servoJ command ID, default is 0
    * @return Error code
    */
    int ServoJ(JointPos joint_pos, float acc, float vel, float cmdT, float filterT, float gain,int id=0);
 
Joint space servo mode motion code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.4  Web-3.8.3
    
.. code-block:: c#
    :linenos:
 
    private void btnJointServoMove_Click(object sender, EventArgs e)
    {
        JointPos j = new JointPos(0, 0, 0, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);

        float vel = 0.0f;
        float acc = 0.0f;
        float cmdT = 0.008f;
        float filterT = 0.0f;
        float gain = 0.0f;
        byte flag = 0;
        int count = 500;
        float dt = 0.1f;
        int cmdID = 0;
        int ret = robot.GetActualJointPosDegree(flag, ref j);
        if (ret == 0)
        {
            robot.ServoMoveStart();

            try
            {
                while (count > 0)
                {

                    robot.ServoJ(j, epos, acc, vel, cmdT, filterT, gain, cmdID);


                    j.jPos[0] += dt;
                    count--;


                    robot.WaitMs((int)(cmdT * 1000));
                }
            }
            finally
            {

                robot.ServoMoveEnd();
            }
        }
        else
        {
            Console.WriteLine($"GetActualJointPosDegree error code: {ret}");

        }
    }
 
Start of joint torque control
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Joint torque control start
    * @return Error code
    */
    int ServoJTStart().
 
Joint torque control
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Joint Torque Control
    * @param [in] torque Torque for joints j1~j6, unit: Nm
    * @param [in] interval Command cycle, unit: s, range: [0.001~0.008]
    * @param [in] checkFlag Detection strategy 
    *                       0-No restrictions; 
    *                       1-Power limit; 
    *                       2-Velocity limit; 
    *                       3-Both power and velocity limits
    * @param [in] jPowerLimit Maximum joint power limit (W)
    * @param [in] jVelLimit Maximum joint velocity (°/s)
    * @return Error code
    */
    public int ServoJT(double[] torque, double interval, int checkFlag, double[] jPowerLimit, double[] jVelLimit)
 
End of joint torque control
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief End of joint torque control.
    * @return Error code
    */
    int ServoJTEnd();
 
Joint Torque Control Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
    private void button27_Click(object sender, EventArgs e)
    {
        robot.DragTeachSwitch(1);
        robot.SetPowerLimit(1, 200);
        double[] torques = { 0, 0, 0, 0, 0, 0 };
        robot.GetJointTorques(1, torques);

        int count = 100;
        robot.ServoJTStart();
        int error = 0;
        while (count > 0)
        {
            error = robot.ServoJT(torques, 0.001f);
            count--;
            Thread.Sleep(1);
        }
        error = robot.ServoJTEnd();
        robot.DragTeachSwitch(0);
    }

Joint Torque Control Code Example with Overspeed Protection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    public int ServoJTWithSafety()
    {
        robot.ResetAllError();
        Thread.Sleep(500);

        double[] torques = new double[6] { 0, 0, 0, 0, 0, 0 };
        robot.GetJointTorques(1, torques);

        robot.ServoJTStart();
        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
        robot.DragTeachSwitch(1);

        int checkFlag = 0;
        double[] jPowerLimit = new double[6] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        //double[] jPowerLimit = new double[6] { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };
        // double[] jVelLimit = new double[6] { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };
        double[] jVelLimit = new double[6] {50, 50, 50, 50, 50, 50 };
        int count = 80000;
        int errorNum = 0;
        int error = 0;
        while (count > 0)
        {
            
            torques[2] = torques[2] + 0.01; 
            error = robot.ServoJT(torques, 0.008, checkFlag, jPowerLimit, jVelLimit); 

            Console.WriteLine($"ServoJT rtn is {error}");
            count = count - 1;
            Thread.Sleep(1);
                
            robot.GetRobotRealTimeState(ref pkg);
            Console.WriteLine($"maincode {pkg.main_code}, subcode {pkg.sub_code}");
            if (error != 0)
            {
                errorNum++;
                if (errorNum > 5)
                {
                    break;
                }

            }
        }
        robot.DragTeachSwitch(0);
        error = robot.ServoJTEnd();

        return 0;
    } 

Servo mode motion in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
Cartesian Space Servo Mode Motion
++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Cartesian space servo mode motion
    * @param [in] mode 0-Absolute motion (base coordinate system), 1-Incremental motion (base coordinate system), 2-Incremental motion (tool coordinate system)
    * @param [in] desc_pos Target Cartesian pose or pose increment
    * @param [in] exaxis Extended axis position
    * @param [in] pos_gain Pose increment proportionality coefficient, only effective in incremental motion, range [0~1]
    * @param [in] acc Acceleration percentage, range [0~100], temporarily not available, default 0
    * @param [in] vel Velocity percentage, range [0~100], temporarily not available, default 0
    * @param [in] cmdT Command transmission period, unit s, recommended range [0.001~0.016]
    * @param [in] filterT Filter time, unit s, temporarily not available, default 0
    * @param [in] gain Proportional amplifier for target position, temporarily not available, default 0
    * @return Error code
    */
    public int ServoCart(int mode, DescPose desc_pose, ExaxisPos exaxis, double[] pos_gain, double acc, double vel, double cmdT, double filterT, double gain);
 
Example of servo mode motion code in Cartesian space
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
    public void TestServoCart()
    {
        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();

        int rtn;
        DescPose desc_pos_dt = new DescPose(83.00800f, 50.525000f, 29.246f, 179.629f, -7.138f, -166.975f);
        ExaxisPos exaxis = new ExaxisPos(100.0f, 0.0f, 0.0f, 0.0f);
        double[] pos_gain = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        int mode = 0;
        float vel = 0.0f;
        float acc = 0.0f;
        float cmdT = 0.001f;
        float filterT = 0.0f;
        float gain = 0.0f;
        byte flag = 0;
        int count = 5000;

        robot.SetSpeed(20);

        while (count > 0)
        {
            rtn = robot.ServoCart(mode, desc_pos_dt, exaxis, pos_gain, acc, vel, cmdT, filterT, gain);
            Console.WriteLine($"ServoCart rtn is {rtn}");
            count -= 1;
            desc_pos_dt.tran.x += 0.01f;
            exaxis.ePos[0] += 0.01f;
        }
    }
 
Spline motion starts
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Spline start
    * @return Error code.
    */
    int SplineStart();
 
Spline motion PTP
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Joint space spline motion.
    * @param [in] joint_pos Target joint position in degrees.
    * @param [in] desc_pos Target Cartesian position.
    * @param [in] tool Tool coordinate number, range [0~14].
    * @param [in] user Workpiece coordinate number, range [0~14].
    * @param [in] vel velocity percentage, range [0~100] * @param [in] acc.
    * @param [in] acc Acceleration percentage, range [0~100], not available yet.
    * @param [in] ovl velocity scaling factor, range [0~100].   
    * @return Error code
    */
    int SplinePTP(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl);

Joint space spline motion (automatic forward kinematics calculation)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief Joint space spline motion (automatic forward kinematics calculation)
    * @param [in] joint_pos  Target joint position, unit deg
    * @param [in] tool  Tool coordinate number, range [0~14]
    * @param [in] user  Workpiece coordinate number, range [0~14]
    * @param [in] vel  Velocity percentage, range [0~100]
    * @param [in] acc  Acceleration percentage, range [0~100], not open yet
    * @param [in] ovl  Velocity scaling factor, range [0~100]
    * @return Error code
    */
    int SplinePTP(JointPos joint_pos, int tool, int user, double vel, double acc, double ovl)

End of spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief End of spline motion.
    * @return Error code.
    */
    int SplineEnd(); 
 
Sample code for spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
    private void btnSplineMove_Click(object sender, EventArgs e)
    {
        JointPos j1 = new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2 = new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        JointPos j3 = new JointPos(-61.954, -84.409, 108.153, -116.316, -91.283, 74.260);
        JointPos j4 = new JointPos(-89.575, -80.276, 102.713, -116.302, -91.284, 74.267);
        DescPose desc_pos1 = new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2 = new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose desc_pos3 = new DescPose(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
        DescPose desc_pos4 = new DescPose(-104.066, 544.321, 327.023, -177.715, 3.371, -73.818);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);

        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = -1.0f;
        byte flag = 0;

        robot.SetSpeed(20);

        int err = -1;
        err = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Console.WriteLine($"movej errcode:  {err}");

        robot.SplineStart();
        robot.SplinePTP(j1, desc_pos1, tool, user, vel, acc, ovl);
        robot.SplinePTP(j2, desc_pos2, tool, user, vel, acc, ovl);
        robot.SplinePTP(j3, desc_pos3, tool, user, vel, acc, ovl);
        robot.SplinePTP(j4, desc_pos4, tool, user, vel, acc, ovl);
        robot.SplineEnd();
    }
 
Starting new spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionchanged:: C#SDK-v1.0.6

.. code-block:: c#
    :linenos:
 
    /** 
    * @brief New spline motion start 
    * @param [in] type 0-circular transition, 1-given point as path point
    * @param [in] averageTime global average articulation time (ms) (10 ~ ), default 2000
    * @return ErrorCode 
    */ 
    int NewSplineStart(int type, int averageTime=2000);
    
New spline instruction point
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /** 
    * @brief Add spline command point. 
    * @param [in] joint_pos Target joint position, in deg. 
    * @param [in] desc_pos Target Cartesian position. 
    * @param [in] tool Tool coordinate number, range [0~14]. 
    * @param [in] user Tool coordinate number, range [0~14]. 
    * @param [in] vel Speed percentage, range [0~100]. 
    * @param [in] acc Acceleration Percentage, range [0~100], not available yet. 
    * @param [in] ovl velocity scaling factor, range [0~100] 
    * @param [in] blendR [-1.0]-motion in place (blocking), [0~1000.0]-smoothing radius (non-blocking), unit mm
    * @param [in] lastFlag whether it is the last point, 0-no, 1-yes
    * @return error code 
    */ 
    int NewSplinePoint(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int lastFlag);;

New spline command point (automatic inverse kinematics calculation)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.7  Web-3.8.5

.. code-block:: c#
    :linenos:

    /**
    * @brief New spline command point (automatic inverse kinematics calculation)
    * @param [in] desc_pos  Target cartesian pose
    * @param [in] tool  Tool coordinate number, range [0~14]
    * @param [in] user  Workpiece coordinate number, range [0~14]
    * @param [in] vel  Velocity percentage, range [0~100]
    * @param [in] acc  Acceleration percentage, range [0~100], not open yet
    * @param [in] ovl  Velocity scaling factor, range [0~100]
    * @param [in] blendR [-1.0]-move to position (blocking), [0~1000.0]-smoothing radius (non-blocking), unit mm
    * @param [in] lastFlag Whether it is the last point, 0-no, 1-yes
    * @param [in] config Inverse kinematics joint space configuration, [-1]-calculate based on current joint position, [0~7]-solve according to specific joint space configuration
    * @return Error code
    */
    int NewSplinePoint(DescPose desc_pos, int tool, int user, double vel, double acc, double ovl, double blendR, int lastFlag,int config)

End of new spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /** 
    * @brief Start of new spline motion 
    * @return Error code. 
    */ 
    int NewSplineEnd();
    
Sample code for new spline motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    private void btnNewSpline_Click(object sender, EventArgs e)
    {
        JointPos j1 = new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2 = new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        JointPos j3 = new JointPos(-61.954, -84.409, 108.153, -116.316, -91.283, 74.260);
        JointPos j4 = new JointPos(-89.575, -80.276, 102.713, -116.302, -91.284, 74.267);
        JointPos j5 = new JointPos(-95.228, -54.621, 73.691, -112.245, -91.280, 74.268);
        DescPose desc_pos1 = new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2 = new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose desc_pos3 = new DescPose(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
        DescPose desc_pos4 = new DescPose(-104.066, 544.321, 327.023, -177.715, 3.371, -73.818);
        DescPose desc_pos5 = new DescPose(-33.421, 732.572, 275.103, -177.907, 2.709, -79.482);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);

        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = -1.0f;
        byte flag = 0;

        robot.SetSpeed(20);

        int err = -1;
        err = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Console.WriteLine($"movej errcode:  {err}");

        robot.NewSplineStart(1, 2000);
        robot.NewSplinePoint(j1, desc_pos1, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(j2, desc_pos2, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(j3, desc_pos3, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(j4, desc_pos4, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplinePoint(j5, desc_pos5, tool, user, vel, acc, ovl, -1, 0);
        robot.NewSplineEnd();
    }
 
Terminate the motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Terminate the campaign.
    * @return Error code
    */
    int StopMotion();
 
Pause the motion.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
    
    /** 
    * @brief Pause motion 
    * @return Error code 
    */  
    int PauseMotion();
 
Resumes motion.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /** 
    * @brief Resume movement 
    * @return Error code 
    */ 
    int ResumeMotion();
 
Motion Pause, Resume, Stop Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    private void btnMotionPause_Click(object sender, EventArgs e)
    {
        int rtn;
        JointPos j1 = new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j5 = new JointPos(-95.228, -54.621, 73.691, -112.245, -91.280, 74.268);
        DescPose desc_pos1 = new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos5 = new DescPose(-33.421, 732.572, 275.103, -177.907, 2.709, -79.482);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);

        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = -1.0f;
        byte flag = 0;

        robot.SetSpeed(20);

        rtn = robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        rtn = robot.MoveJ(j5, desc_pos5, tool, user, vel, acc, ovl, epos, 1, flag, offset_pos);
        Thread.Sleep(1000);
        robot.PauseMotion();

        Thread.Sleep(1000);
        robot.ResumeMotion();

        Thread.Sleep(1000);
        robot.StopMotion();

        Thread.Sleep(1000);

    }
 
Start the overall point offset
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Start of overall point offset.
    * @param [in] flag 0 - offset in base/workpiece coordinate system, 2 - offset in tool coordinate system
    * @param [in] offset_pos Positional Offset
    * @return Error code
    */
    int PointsOffsetEnable(int flag, DescPose offset_pos); 
 
Points overall offset end
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief End of point offset.
    * @return Error code
    */
    int PointsOffsetDisable(); 
 
Points Offset Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    private void btnPointOffect_Click(object sender, EventArgs e)
    {
        JointPos j1, j2;
        DescPose desc_pos1, desc_pos2, offset_pos, offset_pos1;
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);

        j1 = new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        desc_pos1 = new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);

        j2 = new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);

        desc_pos2 = new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

        offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        offset_pos1 = new DescPose(50.0, 50.0, 50.0, 5.0, 5.0, 5.0);

        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = -1.0f;
        byte flag = 0;
        int type = 0;

        robot.SetSpeed(20);

        robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Thread.Sleep(1000);
        robot.PointsOffsetEnable(type, offset_pos1);
        Thread.Sleep(1000);
        robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        Thread.Sleep(1000);
        robot.PointsOffsetDisable();
    }
 
Control box AO flytap start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Control box AO flyswatter start.
    * @param [in] AONum control box AO number
    * @param [in] maxTCPSpeed maxTCPSpeed value [1-5000mm/s], default 1000
    * @param [in] maxAOPercent Percentage of AO corresponding to the maximum TCP speed value, default 100%
    * @param [in] zeroZoneCmp AO percentage for dead zone compensation value, shaped, default 20%, range [0-100]
    * @return Error code.
    */
    int MoveAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);
 
Control box AO flytap stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Control box AO flytap stops
    * @return Error code.
    */
    int MoveAOStop();
    
End AO flyswatter start
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief End AO flytap start
    * @param [in] AONum The end AO number.
    * @param [in] maxTCPSpeed maxTCPSpeed value [1-5000mm/s], default 1000
    * @param [in] maxAOPercent Percentage of AOs corresponding to the maximum TCP speed value, default 100%
    * @param [in] zeroZoneCmp AO percentage for dead zone compensation value, shaped, default 20%, range [0-100]
    * @return Error code.
    */
    int MoveToolAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp);
    
End AO flytap stop
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.7
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief End AO flyswatter stops
    * @return Error code
    */
    int MoveToolAOStop();
 
AO flyswatter code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
    private void btnMoveAO_Click(object sender, EventArgs e)
    {
        JointPos j1 = new JointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
        JointPos j2 = new JointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
        DescPose desc_pos1 = new DescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
        DescPose desc_pos2 = new DescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);

        int tool = 0;
        int user = 0;
        float vel = 100.0f;
        float acc = 100.0f;
        float ovl = 100.0f;
        float blendT = 0.0f;
        float blendR = 0.0f;
        byte flag = 0;
        byte search = 0;

        robot.SetSpeed(5);

        robot.MoveAOStart(0,100,100,20);
        robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveAOStop();

        robot.MoveToolAOStart(0, 100, 100, 20);
        robot.MoveJ(j1, desc_pos1, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveJ(j2, desc_pos2, tool, user, vel, acc, ovl, epos, blendT, flag, offset_pos);
        robot.MoveToolAOStop();
    }
 
Start Ptp motion FIR filtering
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3 Web-3.8.2
    
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Start Ptp motion FIR filtering.
    * @param [in] maxAcc Maximum Acceleration Extreme (deg/s2)
    * @param [in] maxJek Uniform Joint Acuity Extreme (deg/s3)
    * @return Error code.
    */
    int PtpFIRPlanningStart(double maxAcc, double maxJek=1000);
 
Turn off Ptp motion FIR filtering
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Turn off Ptp motion FIR filtering.
    * @return Error code
    */
    int PtpFIRPlanningEnd();
 
Start LIN, ARC motion FIR filtering.
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Start LIN, ARC motion FIR filtering.
    * @param [in] maxAccLin Linear Acceleration Extreme (mm/s2)
    * @param [in] maxAccDeg angular acceleration extreme (deg/s2)
    * @param [in] maxJerkLin Linear plus Acceleration Extreme (mm/s3)
    * @param [in] maxJerkDeg Angle plus acceleration extreme (deg/s3)
    * @return Error code
    */
    int LinArcFIRPlanningStart(double maxAccLin, double maxAccDeg, double maxJerkLin, double maxJerkDeg);
 
Turn off LIN, ARC motion FIR filtering
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Turn off LIN, ARC motion FIR filtering.
    * @return Error code
    */
    int LinArcFIRPlanningEnd();
 
FIR filtering code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.3 Web-3.8.2
    
.. code-block:: c#
    :linenos:
 
    private void button69_Click(object sender, EventArgs e)
    {
        int rtn;
        JointPos startjointPos = new JointPos(-11.904f, -99.669f, 117.473f, -108.616f, -91.726f, 74.256f);
        JointPos midjointPos = new JointPos(-45.615f, -106.172f, 124.296f, -107.151f, -91.282f, 74.255f);
        JointPos endjointPos = new JointPos(-29.777f, -84.536f, 109.275f, -114.075f, -86.655f, 74.257f);

        DescPose startdescPose = new DescPose(-419.524f, -13.000f, 351.569f, -178.118f, 0.314f, 3.833f);
        DescPose middescPose = new DescPose(-321.222f, 185.189f, 335.520f, -179.030f, -1.284f, -29.869f);
        DescPose enddescPose = new DescPose(-487.434f, 154.362f, 308.576f, 176.600f, 0.268f, -14.061f);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

        rtn = robot.PtpFIRPlanningStart(1000,1000);
        Console.WriteLine("PtpFIRPlanningStart rtn is " + rtn);
        robot.MoveJ( startjointPos,  startdescPose, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.MoveJ( endjointPos,  enddescPose, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.PtpFIRPlanningEnd();
        Console.WriteLine("PtpFIRPlanningEnd rtn is " + rtn);

        robot.LinArcFIRPlanningStart(1000, 1000, 1000, 1000);
        Console.WriteLine("LinArcFIRPlanningStart rtn is " + rtn);
        robot.MoveL( startjointPos,  startdescPose, 0, 0, 100, 100, 100, -1,  exaxisPos, 0, 0,  offdese, 1, 1);
        robot.MoveC( midjointPos,  middescPose, 0, 0, 100, 100,  exaxisPos, 0,  offdese,  endjointPos,  enddescPose, 0, 0, 100, 100,  exaxisPos, 0,  offdese, 100, -1);
        robot.LinArcFIRPlanningEnd();
        Console.WriteLine("LinArcFIRPlanningEnd rtn is " + rtn);
    }
 
Acceleration smoothing on
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Acceleration smoothing on
    * @param [in] saveFlag Whether to power down and save.
    * @return ErrorCode
    */
    int AccSmoothStart(bool saveFlag);
 
Acceleration SmoothStart(bool saveFlag); int AccSmoothStart
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Accelerate smooth shutdown
    * @param [in] saveFlag whether to power down and save
    * @return ErrorCode
    */
    int AccSmoothEnd(bool saveFlag);
 
Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    private void button1_Click(object sender, EventArgs e)
    {

        int rtn;
        JointPos startjointPos = new JointPos(-11.904f, -99.669f, 117.473f, -108.616f, -91.726f, 74.256f);
        JointPos endjointPos = new JointPos(-45.615f, -106.172f, 124.296f, -107.151f, -91.282f, 74.255f);

        DescPose startdescPose = new DescPose(-419.524f, -13.000f, 351.569f, -178.118f, 0.314f, 3.833f);
        DescPose enddescPose = new DescPose(-321.222f, 185.189f, 335.520f, -179.030f, -1.284f, -29.869f);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
        rtn = robot.AccSmoothStart(false);
        Console.WriteLine("AccSmoothStart rtn is " + rtn);
        robot.MoveJ( startjointPos,  startdescPose, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.MoveJ( endjointPos,  enddescPose, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        rtn = robot.AccSmoothEnd(false);
        Console.WriteLine("AccSmoothEnd rtn is " + rtn);
    }
 
Specify the attitude speed to turn on
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Specify that Attitude Speed is on
    * @param [in] ratio Attitude Speed Percentage [0-300]
    * @return Error code
    */
    int AngularSpeedStart(int ratio).
 
Specifies that the attitude speed is off
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
    
    /**
    * @brief Specify attitude speed off
    * @return Error code
    */
    int AngularSpeedEnd();
 
Robot Specify Angular Speed Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:
 
    private void button71_Click(object sender, EventArgs e)
    {
        int rtn;
        JointPos startjointPos = new JointPos(-11.904f, -99.669f, 117.473f, -108.616f, -91.726f, 74.256f);
        JointPos endjointPos = new JointPos(-45.615f, -106.172f, 124.296f, -107.151f, -91.282f, 74.255f);

        DescPose startdescPose = new DescPose(-419.524f, -13.000f, 351.569f, -178.118f, 0.314f, 3.833f);
        DescPose enddescPose = new DescPose(-321.222f, 185.189f, 335.520f, -179.030f, -1.284f, -29.869f);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
        rtn = robot.AngularSpeedStart(50);
        Console.WriteLine("AngularSpeedStart rtn is " + rtn);
        robot.MoveJ( startjointPos,  startdescPose, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.MoveJ( endjointPos,  enddescPose, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        rtn = robot.AngularSpeedEnd();
        Console.WriteLine("AngularSpeedEnd rtn is " + rtn);
    }
 
Starting singular position protection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.9
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Starting singular bitmap protection.
    * @param [in] protectMode Singular protection mode, 0: articulation mode; 1 - Cartesian mode
    * @param [in] minShoulderPos shoulder singularity adjustment range (mm), default 100
    * @param [in] minElbowPos Elbow singularity adjustment range (mm), default 50
    * @param [in] minWristPos Wrist singularity range (°), default 10
    * @return Error code
    */
    int SingularAvoidStart(int protectMode, double minShoulderPos, double minElbowPos, double minWristPos);
 
Stop singular attitude protection
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.9
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Stop odd-bit pose protection.
    * @return Error code
    */
    int SingularAvoidEnd();
 
Code example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-v1.0.9
 
.. code-block:: c#
    :linenos:
 
    private void btnTestSingularAvoidEArc_Click(object sender, EventArgs e)
    {
        int rtn;
        JointPos startjointPos = new JointPos(-11.904f, -99.669f, 117.473f, -108.616f, -91.726f, 74.256f);
        JointPos endjointPos = new JointPos(-45.615f, -106.172f, 124.296f, -107.151f, -91.282f, 74.255f);

        DescPose startdescPose = new DescPose(-419.524f, -13.000f, 351.569f, -178.118f, 0.314f, 3.833f);
        DescPose enddescPose = new DescPose(-321.222f, 185.189f, 335.520f, -179.030f, -1.284f, -29.869f);

        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);

        rtn = robot.SingularAvoidStart(2, 10, 5, 5);
        Console.WriteLine("SingularAvoidStart rtn is " + rtn);
        robot.MoveJ( startjointPos,  startdescPose, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        robot.MoveJ( endjointPos,  enddescPose, 0, 0, 100, 100, 100,  exaxisPos, -1, 0,  offdese);
        rtn = robot.SingularAvoidEnd();
        Console.WriteLine("SingularAvoidEnd rtn is " + rtn);
    }
 
Safety Stop Trigger
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
.. code-block:: c#
    :linenos:
 
    /**
    * @brief Safe stop trigger signal
    * @return Error code
    */
    int GetSafetyCode();
 
Clear the motion command queue
++++++++++++++++++++++++++++++++++++++++++++++++++
.. versionadded:: C#SDK-V1.1.9  Web-3.8.7
    
.. code-block:: c#
    :linenos:

    /**
    * @brief Clear the motion command queue
    * @return Error code
    */
    public int MotionQueueClear();
 
Move to Intersecting Line Start Point
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Move to intersecting line start point
    * @param [in] mainPoint Cartesian poses of 6 taught points on the main pipe
    * @param [in] mainExaxisPos Extended axis positions for 6 taught points on the main pipe
    * @param [in] piecePoint Cartesian poses of 6 taught points on the branch pipe
    * @param [in] pieceExaxisPos Extended axis positions for 6 taught points on the branch pipe
    * @param [in] extAxisFlag Whether to enable extended axis; 0-Disable; 1-Enable
    * @param [in] exaxisPos Start point extended axis position
    * @param [in] tool Tool coordinate system number
    * @param [in] wobj Workpiece coordinate system number
    * @param [in] vel Velocity percentage
    * @param [in] acc Acceleration percentage
    * @param [in] ovl Velocity scaling factor
    * @param [in] oacc Acceleration scaling factor
    * @param [in] moveType Motion type; 0-PTP; 1-LIN
    * @param [in] moveDirection Motion direction; 0-Clockwise; 1-Counterclockwise
    * @param [in] offset Offset value
    * @return Error code
    */
    public int MoveToIntersectLineStart(DescPose[] mainPoint, ExaxisPos[] mainExaxisPos, DescPose[] piecePoint, ExaxisPos[] pieceExaxisPos, int extAxisFlag, ExaxisPos exaxisPos, int tool, int wobj, double vel, double acc, double ovl, double oacc, int moveType, int moveDirection, DescPose offset);
            
Intersecting Line Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Intersecting line motion
    * @param [in] mainPoint Cartesian poses of 6 taught points on the main pipe
    * @param [in] mainExaxisPos Extended axis positions for 6 taught points on the main pipe
    * @param [in] piecePoint Cartesian poses of 6 taught points on the branch pipe
    * @param [in] pieceExaxisPos Extended axis positions for 6 taught points on the branch pipe
    * @param [in] extAxisFlag Whether to enable extended axis; 0-Disable; 1-Enable
    * @param [in] exaxisPos Start point extended axis positions
    * @param [in] tool Tool coordinate system number
    * @param [in] wobj Workpiece coordinate system number
    * @param [in] vel Velocity percentage
    * @param [in] acc Acceleration percentage
    * @param [in] ovl Velocity scaling factor
    * @param [in] oacc Acceleration scaling factor
    * @param [in] moveDirection Motion direction; 0-Clockwise; 1-Counterclockwise
    * @param [in] offset Offset value
    * @return Error code
    */
    public int MoveIntersectLine(DescPose[] mainPoint, ExaxisPos[] mainExaxisPos, DescPose[] piecePoint, ExaxisPos[] pieceExaxisPos, int extAxisFlag, ExaxisPos[] exaxisPos, int tool, int wobj, double vel, double acc, double ovl, double oacc, int moveDirection, DescPose offset);
                
Robot Intersecting Line Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    void TestIntersectLineMove()
    {
        ROBOT_STATE_PKG pkg = {};
        FRRobot robot;
        robot.LoggerInit();
        robot.SetLoggerLevel(3);
        int rtn = robot.RPC("192.168.58.2");
        if (rtn != 0)
        {
            return ;
        }
        robot.SetReConnectParam(true, 30000, 500);
        DescPose mainPoint[6] = {};
        DescPose piecePoint[6] = {};
        ExaxisPos mainExaxisPos[6] = {};
        ExaxisPos pieceExaxisPos[6] = {};
        int extAxisFlag = 1;
        ExaxisPos exaxisPos[4] = {};
        DescPose offset = { 0.0, 2.0 ,30.0, -2.0, 0.0, 0.0 };
        mainPoint[0] = {490.004, -383.194, 402.735, -9.332, -1.528, 69.594};
        mainPoint[1] = {444.950, -407.117, 389.011, -5.546, -2.196, 65.279};
        mainPoint[2] = {445.168, -463.605, 355.759, -1.544, -10.886, 57.104};
        mainPoint[3] = {507.529, -485.385, 343.013, -0.786, -4.834, 61.799};
        mainPoint[4] = {554.390, -442.647, 367.701, -4.761, -10.181, 64.925};
        mainPoint[5] = {532.552, -394.003, 396.467, -13.732, -13.592, 67.411};
        mainExaxisPos[0] = { -29.996, 0.000, 0.000, 0.000 };
        mainExaxisPos[1] = { -29.996, 0.000, 0.000, 0.000 };
        mainExaxisPos[2] = { -29.996, 0.000, 0.000, 0.000 };
        mainExaxisPos[3] = { -29.996, 0.000, 0.000, 0.000 };
        mainExaxisPos[4] = { -29.996, 0.000, 0.000, 0.000 };
        mainExaxisPos[5] = { -29.996, 0.000, 0.000, 0.000 };
        piecePoint[0] = { 505.571, -192.408, 316.759, 38.098, 37.051, 139.447 };
        piecePoint[1] = {533.837, -201.558, 332.340, 34.644, 42.339, 137.748};
        piecePoint[2] = {530.386, -225.085, 373.808, 35.431, 45.111, 137.560};
        piecePoint[3] = {485.646, -229.195, 383.778, 33.870, 45.173, 137.064};
        piecePoint[4] = {460.551, -212.161, 354.256, 28.856, 45.602, 135.930};
        piecePoint[5] = {474.217, -197.124, 324.611, 42.469, 41.133, 148.167};
        pieceExaxisPos[0] = { -29.996, -0.000, 0.000, 0.000 };
        pieceExaxisPos[1] = { -29.996, -0.000, 0.000, 0.000 };
        pieceExaxisPos[2] = { -29.996, -0.000, 0.000, 0.000 };
        pieceExaxisPos[3] = { -29.996, -0.000, 0.000, 0.000 };
        pieceExaxisPos[4] = { -29.996, -0.000, 0.000, 0.000 };
        pieceExaxisPos[5] = { -29.996, -0.000, 0.000, 0.000 };
        exaxisPos[0] = {-29.996, -0.000, 0.000, 0.000};
        exaxisPos[1] = {-44.994, 90.000, 0.000, 0.000};
        exaxisPos[2] = {-59.992, 0.002, 0.000, 0.000};
        exaxisPos[3] = {-44.994, -89.997, 0.000, 0.000};
        int tool = 2;
        int wobj = 0;
        double vel = 100.0;
        double acc = 100.0;
        double ovl = 12.0;
        double oacc = 12.0; 
        int moveType = 1;
        int moveDirection = 1;
        rtn = robot.MoveToIntersectLineStart(mainPoint, mainExaxisPos, piecePoint, pieceExaxisPos, extAxisFlag, exaxisPos[0], tool, wobj, vel, acc, ovl, oacc, moveType, moveDirection, offset);
        printf("MoveToIntersectLineStart rtn is %d\n", rtn);
        rtn = robot.MoveIntersectLine(mainPoint, mainExaxisPos, piecePoint, pieceExaxisPos, extAxisFlag, exaxisPos, tool, wobj, vel, acc, 5.0, 5.0, moveDirection, offset);
        printf("MoveIntersectLine rtn is %d\n", rtn);
        robot.CloseRPC();
        return ;
    }

Stationary Air Motion
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Stationary Air Motion
    * @return Error code
    */
    public int MoveStationary()

Stationary Air Motion Code Example
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    public void LaserSensorRecordandReplay()
    {
        int rtn = robot.LaserSensorRecordandReplay(0, 10, 1, 0, 0.1, 1, 1, 10, 100);
        Console.WriteLine($"LaserSensorRecordandReplay rtn is {rtn}");
        rtn = robot.MoveStationary();
        Console.WriteLine($"MoveStationary rtn is {rtn}");
        rtn = robot.LaserSensorRecord1(0, 10);
        Console.WriteLine($"LaserSensorRecord1 rtn is {rtn}"); 
    }