#!/bin/bash 

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sleep 5

xbot2-core --simtime --config $SCRIPT_DIR/../config/xbot2_config.yaml --verbose