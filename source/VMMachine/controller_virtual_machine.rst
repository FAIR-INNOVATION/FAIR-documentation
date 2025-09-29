Virtual Machine
==========================================

Overview
-----------
This manual aims to introduce how to use the FAIRINO SimMachine virtual machine.

Operating Instructions
----------------------

Installing VMware Workstation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

VMware Workstation demo version: 17.6.3 (skip this step if already installed).

Directly search for the VMware official website in your browser or click the URL \ `<https://www.vmware.com>`__\, then download the installation package and install it using the default path.

.. image:: controller_virtual_machine/001.png
   :width: 6in
   :align: center

.. centered:: Figure 6.2-1 VMWare Interface

Opening the Virtual Machine Image
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Download the virtual machine image FAIRINO_SimMachine.zip and extract it.
   
2. Open VMware, click File->Open. As shown in Figure 2-2 below:

.. image:: controller_virtual_machine/002.png
   :width: 6in
   :align: center

.. centered:: Figure 6.2-2 Open Image

3. Locate the extracted folder and select the file with the .vmx extension. As shown in Figure 2-3 below:
   
.. image:: controller_virtual_machine/003.png
   :width: 6in
   :align: center

.. centered:: Figure 6.2-3 Select File

4. Click "Power on this virtual machine" to start the virtual machine. As shown in Figure 2-4 below:
   
.. image:: controller_virtual_machine/004.png
   :width: 6in
   :align: center

.. centered:: Figure 6.2-4 Start Virtual Machine

5. Find "fr_get_vm_net" in the extracted folder and double-click to open it. As shown in Figure 2-5 below. The output content is the virtual machine IP. As shown in Figure 2-6 below.
      
.. note:: If the acquisition fails, please go to the virtual machine and obtain it by executing the "ifconfig" command.

.. image:: controller_virtual_machine/005.png
   :width: 6in
   :align: center

.. centered:: Figure 6.2-5 fr_get_vm_net.bat
      
.. image:: controller_virtual_machine/006.png
   :width: 4in
   :align: center

.. centered:: Figure 6.2-6 Virtual Machine IP

Accessing WebApp from Windows
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. After obtaining the virtual machine IP, directly access the virtual machine IP in the Windows browser to enter WebApp. For example, enter: 192.168.182.222, as shown in Figure 2-7:
         
.. image:: controller_virtual_machine/007.png
   :width: 6in
   :align: center

.. centered:: Figure 6.2-7 Access WebApp via Virtual Machine IP