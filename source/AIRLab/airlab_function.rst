Function Description
==============================================

The mixed palletizing function of this software is based on deep reinforcement learning. You can use either the pre-installed AI models in the software or your custom AI models to perform mixed palletizing of boxes of different sizes.

.. toctree:: 
   :maxdepth: 6

Scope of Application
--------------------------

Operation Types
+++++++++++++++++

The mixed palletizing function provided by this software is an online palletizing system. Unlike traditional offline palletizing, an online palletizing system can process and stack goods immediately upon arrival without the need for prior planning and preprocessing.

Box Types
+++++++++++++++++

Currently, only regular-shaped rectangular boxes are supported.

Pallet Types
+++++++++++++++++
Currently, only square pallets are supported for stacking, meaning the length and width of the pallet are equal. If your pallet is rectangular, please consider the shorter side as the pallet's side length.

Palletizing Parameter Description
-----------------------------------------------

Preview Number of Boxes (Preview Number k)
+++++++++++++++++++++++++++++++++++++++++++++++++

This parameter determines the number of known boxes during each inference of the AI model. Generally, a larger value for this parameter leads to better palletizing results. If you choose to use conveyor belt transportation for the boxes, the distance of conveyor belt movement between the camera and the box picking point should also increase with the preview number k. To achieve better stacking results, we recommend setting the lookahead count k > 2. For the corresponding conveyor belt length estimation method, please refer to the AIRLab Mixed Palletizing Software Case Manual.

Number of Moves After Obtaining Box Position Information
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

This parameter determines the number of conveyor belt movements after the software obtains information about a box. Once the movements are completed, the position where the box is located becomes the position for the robot to pick up the box. It is necessary to set this parameter reasonably based on the conveyor belt's single movement distance, conveyor belt length, and camera position.

AI Models
++++++++++++++++++

The following models are currently pre-installed in the software:

.. csv-table:: 
    :header: "Model Names","Pallet Grid Numbers","Recommended Pallet Sizes(mm)","Applicable Range"
    :widths: 10 20 20 50

    "G25B214S40","25*25","≤1000","Boxes with side lengths ranging from 2 to 14 grids"
    "G20B411S20","20*20","≤500","Boxes with side lengths ranging from 4 to 11 grids"
    "G20T812S20","20*20","≤500","Standard express cartons #8~12"
    "G20T812S20v0","20*20","≤500","Standard express cartons #8~12"
    "G20T812S20v1","20*20","≤500","Standard express cartons #8~12"
    "G20T812S20v2","20*20","≤500","Standard express cartons #8~12"
    "G20T812S20v3","20*20","≤500","Standard express cartons #8~12"
    "G20T812S20v4","20*20","≤500","Standard express cartons #8~12"

.. important::
    where each grid size equals the pallet size divided by the number of pallet grids. For example, for model G25B214S40, the size of each grid is at most 1000/25 = 40 mm, suitable for box sizes ranging from 80 to 560 mm.
    
    You can choose the model that best suits your needs. If the pre-installed models cannot meet your requirements, please feel free to contact us.

Palletizing Height
+++++++++++++++++++++++++++++++++++++

This parameter determines the maximum height that the palletizing can reach, currently defaulting to no more than the length and width of the pallet.

Other Parameters
+++++++++++++++++++++++++++++++++++++

**Safety Height**: The height offset added to the palletizing height, serving as a safety transition point height before the robot places the box.

**Default Speed, Safety Speed**: Used to control the running speed of the robot, ranging from 0 to 100.

**End Detection DI, Gripper DO**: IO port numbers.

