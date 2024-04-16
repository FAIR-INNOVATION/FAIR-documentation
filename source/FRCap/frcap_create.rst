Create wizard
=========================

.. toctree:: 
   :maxdepth: 6

"Creation Wizard" is a tool in FRCap-Tools, through which you can easily and quickly initialize an FRCap project by entering a small number of parameters.

Parameter configuration
--------------------------------

Creating FRCap mainly requires two types of parameters, basic information of FRCap and configurations at all levels. They will be explained separately below.

Basic information
+++++++++++++++++++++

The basic information includes "plug-in name", "plug-in author" and "plug-in description".

Plugin name:

- Required fields;
- There are no restrictions on input characters and character length, and no spaces are allowed;
- It is recommended that the name should not exceed 7 CKJ characters (Chinese, Japanese, Korean, etc.) or 10 all uppercase Latin letters or 14 all lowercase Latin letters (English, French, etc.).
- Recommended examples:

  - Palletizer;
  - Polishing process software;
  - Device Config;
  - HELLO FRCAP.

Plugin author:

- Required fields;
- There are no restrictions on input characters and character length. For example, you can enter your personal name, company name, etc.;
- Recommended examples:

  - Zhang San
  - Franklin Peter
  - FAIR Innovation (Suzhou) Robot Systems Co., Ltd.

Plugin description:

- Optional fields;
- There are no restrictions on input characters or character length, just briefly describe your plug-in.

Advanced configuration
-------------------------

Plug-in type:

- Required fields;
- The type options are "Configuration" and "Application" respectively.
- "Configuration" is recommended for FRCap that implements relatively simple configuration and control operations such as setting parameters and button operations. After importing, use it in "Auxiliary Application" -> "FRCap" in WebApp.
- "Application" recommends FRCap that implements complex process scenarios, such as palletizing scenarios, welding processes and other industry applications. After importing, use it directly under "Auxiliary Applications" in WebApp.

Plug-in icon:

- Optional fields;
- You can upload the company logo or any icon you want to use for the icon. Please pay attention to the copyright. Our company is not responsible for any copyright issues caused by any reason;
- If you do not upload an icon, the company's "FAIRINO" Logo icon will be used by default in the exported FRCap project. You can replace and modify it in the public folder under the project folder. This icon is for initialization purposes only. Please do not use the "FAIRINO" Logo directly in any commercial scenarios.


Download
-------------
After all the above parameters are configured and the FRCap is successfully created, you will be redirected to the download page. You need to confirm that the name is correct before you can download the created FRCap project to your local computer for subsequent development work and construction.

The downloaded plug-in is in ".tar.gz" compressed format.

On Windows systems we recommend using 7-Zip software to decompress.

On Linux systems, you can use the following command in the terminal to decompress:

.. code-block:: c++
   :linenos:

    tar -zxvf frcap_{FRCapName}.tar.gz