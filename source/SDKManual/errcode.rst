Error Code Comparison Table
===================================

.. csv-table:: 
    :header-rows: 1
    :name: Interface return value error code comparison table
    :widths: 10 20 30

    "Errcode","Describe","Processing method"
    "-3","Interface exception","Contact the after-sales engineer to view the controller log"
    "-2","The communication with the controller is abnormal","Check the connection to the robot control box"
    "-1","Other errors","Contact the after-sales engineer to view the controller log"
    "0","Successful call","/"
    "3","The number of interface parameters is inconsistent","Check the number of interface parameters"
    "4","Interface parameter value exception","Check parameter value type or range"
    "8","Failed to open track file","Check if the TPD track file exists or the track name is correct"
    "14","Interface execution failed","Check whether the web interface reports a fault or status feedback reports a fault"
    "18","The robot program is running, please stop it first","Stop the program before performing other operations"
    "25","Data exception, calculation failed","Re-calibration or identification"
    "28","Inverse kinematics calculation results are abnormal","Check if the pose is reasonable"
    "29","ServoJ joint overrun","Check whether the joint data is within a reasonable range"
    "30","Non-resettable fault, please power off and restart the control box","Please power off and restart the control box"
    "34","Wrong workpiece number","Please check that the workpiece number is reasonable"
    "36","Filename too long","Please shorten the filename length"
    "38","Singular pose, calculation failed","Please change pose"
    "64","Not added to the instruction queue","Contact the after-sales engineer to view the controller log"
    "66","The middle point 1 of the full circle/helix command is wrong","Check whether the middle point 1 data is correct"
    "67","The middle point 2 of the full circle/helix command is wrong","Check whether the middle point 2 data is correct"
    "68","The middle point 3 of the full circle/helix command is wrong","Check whether the middle point 3 data is correct"
    "69","The middle point of the arc command is wrong","Check if the intermediate point data is correct"
    "70","Arc instruction target point error","Check if the target point data is correct"
    "73","Gripper movement error","Check whether the communication status of the gripper is normal"
    "74","Line instruction point error","Check whether the point data is correct"
    "75","Channel error","Check if IO number is in range"
    "76","Wait timeout","Check whether the IO signal is input or the wiring is correct"
    "82","TPD instruction point error","Re-record the teaching track"
    "83","TPD instruction tool does not match current tool","Change the tool coordinate system used when teaching to TPD"
    "94","Spline cue point error","Check whether the point data is correct"
    "108","Wrong starting point for helix command","Check whether the starting point data is correct"
    "112","The given pose cannot be reached","Check if the target pose is reasonable"
