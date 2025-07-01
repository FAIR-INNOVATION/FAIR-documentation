Status
===============

.. toctree:: 
   :maxdepth: 6

System log
----------------------

When you enter the "Status Information" -> "System Log" interface for the first time, all types of log data for the day are displayed by default.

The log data is classified into levels, currently divided into: all, error warnings, basic settings, security settings, peripheral settings, main unit operations, teaching programs, tool applications, system settings, and file import and export.

There is a search input box in the upper right corner of the data table. Users can enter the filter content according to their search needs. The interface is as follows:

.. image:: status/001.png
   :width: 6in
   :align: center

.. centered:: Figure 13.1-1 System log interface

Status Query
-------------------

Function Usage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Turn on the control box and connect the Ethernet cable to the PC;
2. Open a browser on the PC and visit the target URL: 192.168.58.2. Log in with the account "admin" and password "123" to access the page;
3. Click the "Status Information" -> "Status Query" option in the left menu bar to enter the status query interface, as shown below;

.. image:: status/002.png
   :width: 6in
   :align: center

.. centered:: Figure 13.2‑1 Status Query

.. note:: 
   .. image:: status/006.png
      :width: 0.75in
      :height: 0.75in
      :align: left

   Name: **Query Button**
   
   Function: Click to send an instruction to query chart/trajectory data, indicating an unqueried state.

.. note:: 
   .. image:: status/007.png
      :width: 0.75in
      :height: 0.75in
      :align: left

   Name: **Right Arrow Button**
   
   Function: Click to add the selected item on the left to the sub-items on the right.

.. note:: 
   .. image:: status/008.png
      :width: 0.75in
      :height: 0.75in
      :align: left

   Name: **Delete Button**
   
   Function: Click to delete the selected sub-item on the right.

.. note:: 
   .. image:: status/009.png
      :width: 0.75in
      :height: 0.75in
      :align: left

   Name: **Clear Button**
   
   Function: Click to clear all sub-items on the right.

4. Select "Chart Display," fill in the waveform time, and choose the desired query parameters in the "Parameter Configuration" section on the left. Click the "Right Arrow" button to add the parameters to the list on the right;

.. note:: The waveform time can be customized (10-30s), and a maximum of 6 parameters can be selected.

5. Click the "Query" button to start the query. Based on the parameter configuration, real-time data will be displayed as a line chart, as shown below;

.. image:: status/003.png
   :width: 6in
   :align: center

.. centered:: Figure 13.2‑2 Chart Display

Chart Export
~~~~~~~~~~~~~~~~~~~~~~~~

1. Click the chart title to open a dialog box where you can directly modify the title, as shown below:

.. image:: status/004.png
   :width: 6in
   :align: center

.. centered:: Figure 13.2‑3 Rename Chart Title

2. After clicking the "Stop Query" button to successfully stop the query, a download button will appear. Click it, and the browser will download the chart file with the chart title as the filename, as shown below:

.. image:: status/005.png
   :width: 6in
   :align: center

.. centered:: Figure 13.2‑4 Chart Export

Data View Display
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. After stopping the query, click the "Show Data View" button in the upper-right corner of the chart, as shown below:

.. image:: status/010.png
   :width: 6in
   :align: center

.. centered:: Figure 13.2‑5 Data View Button

2. The data in the view is shown below, and the content supports copying.

.. image:: status/011.png
   :width: 6in
   :align: center

.. centered:: Figure 13.2‑6 Data View Display