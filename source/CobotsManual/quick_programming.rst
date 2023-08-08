Robot Quick Programming
=========================

Introduction to Simple Motion Instructions
---------------------------------------------

**PTP command**: Click the “PTP” icon to enter the PTP command editing interface.

You can choose the point to be reached, and the smooth transition time setting can realize that the movement from this point to the next point is continuous. Whether to set the offset, you can choose to offset based on the base coordinate system and based on the tool coordinates, and pop up x, y, z, rx, ry, rz offset settings, PTP specific path is the optimal path automatically planned by the motion controller, click “Add” and “Apply” to save this command.

.. image:: teaching_pendant_software/ptp.png
   :width: 6in
   :align: center

.. centered:: Figure 5.1-1 PTP command interface

**Lin command**: Click the “Lin” icon to enter the Lin command editing interface.

The function of this command is similar to the “PTP” command, but the path of the point reached by this command is a straight line.

.. image:: teaching_pendant_software/line.png
   :width: 6in
   :align: center

.. centered:: Figure 5.1-2 Lin command interface

Operate on program files
--------------------------

Modify the program tree using the toolbar at the bottom of the program tree.

.. note:: 
   .. image:: teaching_pendant_software/070.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **Open**
   
   effect: Open user program file

.. note:: 
   .. image:: teaching_pendant_software/071.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **New build**
   
   effect: Select a template to create a new program file
   
.. note:: 
   .. image:: teaching_pendant_software/072.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **Import**
   
   effect: Import the file into the user program folder

.. note:: 
   .. image:: teaching_pendant_software/073.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **Export**
   
   effect: Export user program files to a local point.

.. note:: 
   .. image:: teaching_pendant_software/074.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **Save**
   
   effect: Save file edits

.. note:: 
   .. image:: teaching_pendant_software/075.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **Save as**
   
   effect: Rename the file and store it in the user program or template program folder.

.. note:: 
   .. image:: teaching_pendant_software/076.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **Copy**
   
   effect: Duplicates a node and allows it to be used for other operations (eg: paste it elsewhere in the program tree).

.. note:: 
   .. image:: teaching_pendant_software/077.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **Paste**
   
   effect: Allows you to paste previously cut or copied nodes.

.. note:: 
   .. image:: teaching_pendant_software/078.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **To cut**
   
   effect: Cuts a node and allows it to be used for other operations (eg: paste it elsewhere in the program tree).

.. note:: 
   .. image:: teaching_pendant_software/079.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **Delete**
   
   effect: Deletes a node from the program tree.

.. note:: 
   .. image:: teaching_pendant_software/080.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **Move up**
   
   effect: Move the node up.

.. note:: 
   .. image:: teaching_pendant_software/081.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **Move down**
   
   effect: Move the node down.

.. note:: 
   .. image:: teaching_pendant_software/082.png
      :width: 1.5in
      :height: 0.75in
      :align: left

   name: **Toggle edit mode**
   
   effect: The program tree mode and lua editing mode switch each other.


Write and run a program
-------------------------

The left side is mainly for adding program commands. Click the icon above each keyword to enter the detailed interface. There are two main operations for adding program commands to the file. One way is to open the relevant command and click the Apply button to add the command to the program. , another way is to click the “Add” button first, at this time the command is not saved in the program file, and you need to click “Apply” again to save the command in the file. The second method often occurs when multiple commands of the same type are issued. We add an add button and display the content of the added command to this type of command. Click the Add button to add a command, and the added command displays all the added commands. , click “Apply” to save the added command to the opened file on the right.

Click the start button to run the program; click the stop button to stop the program; click the pause/resume button to pause/resume the program; when the program is running, the currently executing program node is highlighted in gray.

In manual mode, click the first icon on the right side of the node to make the robot execute the instruction alone, and the second icon is to edit the content of the node.

.. image:: teaching_pendant_software/069.png
   :width: 6in
   :align: center

.. centered:: Figure 5.3-1 Program tree interface
