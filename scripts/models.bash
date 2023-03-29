SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
export GAZEBO_MODEL_PATH=$(realpath $SCRIPT_DIR/../models):$GAZEBO_MODEL_PATH