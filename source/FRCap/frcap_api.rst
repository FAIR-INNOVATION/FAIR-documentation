API description
=========================

.. toctree:: 
   :maxdepth: 6

act command 
-------------

All the following act instructions use POST, and the URL is /action/act.

Save teaching points
++++++++++++++++++++++++++

Command name: save_point. 

Command parameters: 

.. code-block:: c++
    :linenos:

    /** 
    * @param  string name The name of the teaching point recorded by string name
    * @param  string speed speed
    * @param  string elbow_speed elbow speed
    * @param  string acc acceleration
    * @param  string elbow_acc elbow acceleration
    * @param  string toolnum Tool number
    * @param  string workpiecenum Workpiece number
    */ 

Instruction example:

.. code-block:: c++
    :linenos:

    {
        cmd: "save_point",
        data:{
            name: "point1",
            speed: "100",
            elbow_speed: "100",
            acc: "100",
            elbow_acc: "100",
            toolnum: "1",
            workpiecenum: "1"
        }
    }

Command feedback:

.. code-block:: c++
    :linenos:

    /** 
    * @return status:200 "success"
    * @return status:404 "fail"
    */ 

sta command
-------------

All the following sta instructions use POST, and the URL is /action/ sta .

Get robot status data
++++++++++++++++++++++++++++++++++++++++

Command name: basic. 

Command parameters: none.

Instruction example:

.. code-block:: c++
    :linenos:

    {
        cmd: "basic",
    }

Command feedback:

.. code-block:: c++
    :linenos:

    /** 
    * @return status:200 
    * @param  object joints joint position
    * @param  object tcp Descartes pose
    * @param  array exAxisPos External axis position
    * @return status:404 "fail"
    */
    {
        joints: {
            j1: "90",
            j2: "90",
            j3: "90",
            j4: "90",
            j5: "90",
            j6: "90",
        },
        tcp: {
            x: "100",
            x: "100",
            z: "100",
            rx: "90",
            ry: "90",
            rz: "90",
        },
        exAxisPos: [0,0,0,0]
    }

get command
-------------

All the following get instructions use POST, and the URL is /action/get.

Get teaching points 
++++++++++++++++++++++++++

Command name: get_points().

Command parameters: none.

Instruction example:

.. code-block:: c++
    :linenos:

    {
        cmd: "get_points"
    }

Command feedback:

.. code-block:: c++
    :linenos:

    /** 
    * @return status:200 "success"
    * @param  ${point_name}: object teaching point related information
    * @return status:404 "fail"
    */ 

Instruction feedback case:

.. code-block:: c++
    :linenos:

    {
        "localpoint1": {
            "name":"localpoint1",
            "elbow_speed":"1",
            "elbow_acc":"1",
            "x": "1",
            "y": "1",
            "z": "1",
            "rx": "1",
            "ry": "1",
            "rz": "1",
            "j1": "1",
            "j2": "1",
            "j3": "1",
            "j4": "1",
            "j5": "1",
            "j6": "1",
            "toolnum": "1",
            "workpiecenum": "1",
            "speed": "1",
            "acc": "1",
            "E1": "1",
            "E2: "1",
            "E3": "1",
            "E4": "1"
        }
    }


Get system configuration 
++++++++++++++++++++++++++

Command name: get_syscfg().

Command parameters: none.

Instruction example:

.. code-block:: c++
    :linenos:

    {
        cmd: "get_syscfg"
    }

Command feedback:

.. code-block:: c++
    :linenos:

    /** 
    * @return status:200 "success"
    * @param  string log_count Maximum number of log days recorded
    * @param  string language Currently using language pack
    * @param  string lifespan overtime time
    * * @return status:404 "fail"
    */ 

Instruction feedback case:

.. code-block:: c++
    :linenos:

    {
        log_count:"10",
        language:"zh",
        lifespan:"1800"
    }

set command
-------------

All the following set instructions use POST, and the URL is /action/set.

Issue system variable instructions
++++++++++++++++++++++++++++++++++++

Command name: 511.

Command parameters: 

.. code-block:: c++
    :linenos:

    /** 
    * @param int index system variable serial number: 1-20
    * @param int value system variable value
    */ 

Instruction example:

.. code-block:: c++
    :linenos:

    {
        cmd: 511,
        data:{
            content:"SetSysVarValue(2,1)"
        }
    }

Command feedback:

.. code-block:: c++
    :linenos:

    /** 
    * @return status:200 1: represents success, 0: represents 
    * @return status:404 "fail"
    */

Instruction feedback case:

.. code-block:: c++
    :linenos:

    1

Get system variable instructions 
++++++++++++++++++++++++++++++++++++++

Command name: 512.

Command parameters: 

.. code-block:: c++
    :linenos:

    /** 
    * @param int index system variable serial number: 1-20
    * /

Instruction example:

.. code-block:: c++
    :linenos:

    {
        cmd: 512,
        data:{
            content:"GetSysVarValue(2)"
        }
    }

Command feedback:

.. code-block:: c++
    :linenos:

    /** 
    * @return status:200
    * @param int value system variable value 
    * @return status:404 "fail"
    * /

Instruction feedback case:

.. code-block:: c++
    :linenos:

    1

better-sqlite3 directive 
----------------------------------

Query the first row of records in the database
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Command parameters: 

.. code-block:: c++
    :linenos:

    /**
    * @param string db_name Database name (including absolute path)
    * @param string sql sql statement
    * @return string result The first row of records queried
    */

Instruction content:

.. code-block:: c++
    :linenos:

    queryget(string db_name, string sql);

Query all records in the database 
++++++++++++++++++++++++++++++++++++++++++

Command parameters: 

.. code-block:: c++
    :linenos:

    /**
    * @param string db_name Database name (including absolute path)
    * @param string sql sql statement
    * @return string result All records queried
    */

Instruction content:

.. code-block:: c++
    :linenos:

    queryall(string db_name, string sql);

Execute database statements
++++++++++++++++++++++++++++++++++++++++++

.. code-block:: c++
    :linenos:

    /**
    * @param string db_name Database name (including absolute path)
    * @param string sql sql statement
    * @param object obj sql Parameters required for sql statement execution
    * @return \
    */

Command parameters: 

.. code-block:: c++
    :linenos:

    exec(string db_name, string sql, object obj);

Instruction content:

socket command
-----------------------

socket send
++++++++++++++++++++++

Command parameters: 

.. code-block:: c++
    :linenos:

    /**
    * @param string send_content socket Socket communication command sends content
    * @return \
    */

Instruction content:

.. code-block:: c++
    :linenos:

    socket_cmd.send(string send_content);//8065
    socket_file.send(string send_content);//8067

socket recv
+++++++++++++++++++++

Command parameters: 

.. code-block:: c++
    :linenos:

    /**
    * @return string recv_content socket Socket communication command reply content
    */

Instruction content:

.. code-block:: c++
    :linenos:

    socket_cmd.recv();//8065
    socket_file.recv();//8067
