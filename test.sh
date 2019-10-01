#!/bin/bash

package=("ujson" "configparser" "mock" "pytest" "pydispatcher" "websocket-client" "numpy" "transforms3d" "pyros_setup", "jsonpickle")
import=("import ujson" "import configparser" "import mock" "import pytest" "from pydispatch import dispatcher" "import websocket" "import numpy" "import transforms3d" "import rospy" "import jsonpickle")

len=${#package[@]}
for ((i=0;i<${len};i++));
do
    python -c "${import[${i}]}"
    if [ $? == 1 ];
    then
        echo "installing ${package[${i}]}..."
        pip install ${package[${i}]}
    fi
done

#python -m pytest --junitxml=out_report.xml