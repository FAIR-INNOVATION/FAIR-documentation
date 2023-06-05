WebAPP program use
================================

.. toctree:: 
    :maxdepth: 5

Set the default job program to be automatically loaded upon startup
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Set the default job program to be automatically loaded upon startup
    * @param  [in] flag  0- boot does not automatically load the default program, 1- boot automatically load the default program
    * @param  [in] program_name Job program name and path, for example, /fruser/movej.lua, where /fruser/ is a fixed path
    * @return  Error code
    */
    errno_t  LoadDefaultProgConfig(uint8_t flag, char program_name[64]);

Load the specified job program
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Load the specified job program
    * @param  [in] program_name Job program name and path, for example, /fruser/movej.lua, where /fruser/ is a fixed path
    * @return  Error code
    */
    errno_t  ProgramLoad(char program_name[64]);

Get the loaded job program name
+++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the loaded job program name
    * @param  [out] program_name Job program name and path, for example, /fruser/movej.lua, where /fruser/ is a fixed path
    * @return  Error code
    */
    errno_t  GetLoadedProgram(char program_name[64]);  

Get the line number of the current robot job program
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the line number of the current robot job program
    * @param  [out] line  line number
    * @return  Error code
    */   
    errno_t  GetCurrentLine(int *line);

Run the currently loaded job program
+++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Run the currently loaded job program
    * @return  Error code
    */
    errno_t  ProgramRun();

Pause the current running job program
++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Pause the current running job program
    * @return  Error code
    */ 
    errno_t  ProgramPause();

Resume the currently suspended job program
+++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Resume the currently suspended job program
    * @return  Error code
    */ 
    errno_t  ProgramResume();  

Terminates the currently running job program
++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Terminates the currently running job program
    * @return  Error code
    */ 
    errno_t  ProgramStop();    

Get the robot job program execution state
+++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: c++
    :linenos:

    /**
    * @brief  Get the robot job program execution state
    * @param  [out]  state 1- program stop or no program running, 2- program running, 3- program pause
    * @return  Error code
    */
    errno_t  GetProgramState(uint8_t *state);

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

        char program_name[64] = "/fruser/ptps.lua";
        char loaded_name[64] = "";
        uint8_t state;
        int line;

        robot.Mode(0);
        robot.ProgramLoad(program_name);
        robot.ProgramRun();
        sleep(5);
        robot.ProgramPause();
        robot.GetProgramState(&state);
        printf("program state:%u\n", state);
        robot.GetCurrentLine(&line);
        printf("current line:%d\n", line);
        robot.GetLoadedProgram(loaded_name);
        printf("program name:%s\n", loaded_name);
        sleep(5);
        robot.ProgramResume();
        sleep(5);
        robot.ProgramStop();
        sleep(2);

        return 0;
    }
