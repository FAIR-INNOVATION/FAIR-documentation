Data structure specification
==============================

.. toctree:: 
    :maxdepth: 5

Data structure specification
++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Joint position data type
    */  
    struct JointPos
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] jPos;   /* Six joint positions, unit: deg */
    }

Cartesian spatial location data type
++++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Cartesian spatial location data type
    */
    struct DescTran
    {
        public double x;    /* X-axis coordinate, unit: mm  */
        public double y;    /* Y-axis coordinate, unit: mm  */
        public double z;    /* Z-axis coordinate, unit: mm  */
    }

Euler Angle attitude data type
++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief Euler Angle attitude data type
    */
    struct Rpy
    {
        public double rx;   /* Rotation Angle about fixed axis X, unit: deg  */
        public double ry;   /* Rotation Angle about fixed axis y, unit: deg  */
        public double rz;   /* Rotation Angle about fixed axis Z, unit: deg  */
    }

Cartesian space pose data type
++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    *@brief Cartesian space pose type
    */
    struct DescPose
    {
        public DescTran tran;     /* Cartesian position  */
        public Rpy rpy;			/* Cartesian space attitude  */
    }

Extension axis position data type
++++++++++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Extension axis position data type
    */ 
    struct ExaxisPos
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public double[] ePos;   /* Position of four expansion shafts, unit: mm */
    }

Torque sensor data type
+++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /**
    * @brief The force component and torque component of the force sensor
    */
    struct ForceTorque
    {
        public double fx;  /* Component of force along the x axis, unit: N  */
        public double fy;  /* Component of force along the y axis, unit: N  */
        public double fz;  /* Component of force along the z axis, unit: N  */
        public double tx;  /* Component of torque about the X-axis, unit: Nm */
        public double ty;  /* Component of torque about the Y-axis, unit: Nm */
        public double tz;  /* Component of torque about the Z-axis, unit: Nm */
    }

Spiral parameter data type
+++++++++++++++++++++++++++++
.. code-block:: c#
    :linenos:

    /** 
    * @brief Spiral parameter data type
    */ 
    struct SpiralParam
    {
        public int circle_num;      /* Coil number  */
        public float circle_angle;  /* Spiral Angle  */
        public float rad_init;      /* Initial radius of spiral, unit: mm  */
        public float rad_add;       /* Radius increment  */
        public float rotaxis_add;   /* Increment in the direction of the axis of rotation  */
        public uint rot_direction;  /* Rotation direction, 0- clockwise, 1- counterclockwise  */
    }
