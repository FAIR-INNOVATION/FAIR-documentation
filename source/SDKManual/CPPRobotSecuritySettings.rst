Security settings
========================

.. toctree:: 
    :maxdepth: 5

Set collision level
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief Set collision level
    * @param  [in]  mode  0- grade, 1- percentage
    * @param  [in]  level Collision threshold, grade range [], percentage range [0~1]
    * @param  [in]  config 0- Do not update the configuration file. 1- Update the configuration file
    * @return  Error code
    */
    errno_t  SetAnticollision(int mode, float level[6], int config);

Set the post-collision policy
++++++++++++++++++++++++++++++++
.. versionchanged:: C++SDK-v2.1.5.0
    
.. code-block:: c++
    :linenos:

	/**
    * @brief  Set the post-collision policy
    * @param  [in] strategy  0- Error stop, 1- Continue running
	* @param  [in] safeTime  Safe stop time[1000 - 2000]ms
	* @param  [in] safeDistance  Safe stopping distance[1-150]mm
    * @param  [in] safetyMargin  j1-j6 Safety factor[1-10]
    * @return  Error code   
	 */
	errno_t  SetCollisionStrategy(int strategy, int strategy, int safeTime, int safeDistance, int safetyMargin[]);

Set the positive limit
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the positive limit
    * @param  [in] limit Six joint positions, unit: deg
    * @return  Error code
    */
    errno_t  SetLimitPositive(float limit[6]);

Set the negative limit
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the negative limit
    * @param  [in] limit Six joint positions, unit: deg
    * @return  Error code
    */
    errno_t  SetLimitNegative(float limit[6]);   

Error status clearing
++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Error status clearing
    * @return  Error code
    */
    errno_t  ResetAllError();

Joint friction compensation switch
+++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Joint friction compensation switch
    * @param  [in]  state  0- off, 1- on
    * @return  Error code
    */
    errno_t  FrictionCompensationOnOff(uint8_t state);

Set joint friction compensation coefficient - formal
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set joint friction compensation coefficient - formal
    * @param  [in]  coeff Six joint compensation coefficients, range [0~1]
    * @return  Error code
    */
    errno_t  SetFrictionValue_level(float coeff[6]);

Set joint friction compensation coefficient - side mount
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set joint friction compensation coefficient - side mount
    * @param  [in]  coeff Six joint compensation coefficients, range [0~1]
    * @return  Error code
    */
    errno_t  SetFrictionValue_wall(float coeff[6]);

Set joint friction compensation coefficient - reverse mount
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set joint friction compensation coefficient - inverted
    * @param  [in]  coeff Six joint compensation coefficients, range [0~1]
    * @return  Error code
    */
    errno_t  SetFrictionValue_ceiling(float coeff[6]);

Set joint friction compensation coefficient - free mount
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set joint friction compensation coefficient - free mount
    * @param  [in]  coeff Six joint compensation coefficients, range [0~1]
    * @return  Error code
    */
    errno_t  SetFrictionValue_freedom(float coeff[6]);

Code example
++++++++++++++
.. code-block:: c++
    :linenos:

    #include <cstdlib>
    #include <iostream>
    #include <stdio.h>
    #include <cstring>
    #include <unistd.h>
    #include "FRRobot.h"
    #include "RobotTypes.h"

    using namespace std;

    int main(void)
    {
        FRRobot robot;                 //Instantiate the robot object
        robot.RPC("192.168.58.2");     //Establish a communication connection with the robot controller

        int mode = 0;
        int config = 1;
        float level1[6] = {1.0,2.0,3.0,4.0,5.0,6.0};
        float level2[6] = {50.0,20.0,30.0,40.0,50.0,60.0};

        robot.SetAnticollision(mode, level1, config);
        mode = 1;
        robot.SetAnticollision(mode, level2, config);
        robot.SetCollisionStrategy(1);

        float plimit[6] = {170.0,80.0,150.0,80.0,170.0,160.0};
        robot.SetLimitPositive(plimit);
        float nlimit[6] = {-170.0,-260.0,-150.0,-260.0,-170.0,-160.0};
        robot.SetLimitNegative(nlimit);

        robot.ResetAllError();

        float lcoeff[6] = {0.9,0.9,0.9,0.9,0.9,0.9};
        float wcoeff[6] = {0.4,0.4,0.4,0.4,0.4,0.4};
        float ccoeff[6] = {0.6,0.6,0.6,0.6,0.6,0.6};
        float fcoeff[6] = {0.5,0.5,0.5,0.5,0.5,0.5};
        robot.FrictionCompensationOnOff(1);
        robot.SetFrictionValue_level(lcoeff);
        robot.SetFrictionValue_wall(wcoeff);
        robot.SetFrictionValue_ceiling(ccoeff);
        robot.SetFrictionValue_freedom(fcoeff);

        return 0;
    }
