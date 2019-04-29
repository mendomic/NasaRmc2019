#!/bin/bash
# Must run 'source ./set_detailed_logging.sh' to get the 
export ROSCONSOLE_FORMAT='[${node}] [${file}] [line:${line}] [${function}] [${severity}] [${time}]: ${message}'
CURRENT_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
export ROSCONSOLE_CONFIG_FILE=$CURRENT_SCRIPT_DIR/detailed_logging_rosconsole.config

