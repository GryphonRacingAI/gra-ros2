#!/bin/bash

cd /local/date/$USER/colcon_ws

echo " pwd: $(pwd)"

# Name of the virtual environment folder
VENV_DIR="virtual_env"

# Path to requirements.txt
REQ_FILE="src/gra-ros2/perception/src/ultralytics_ros/requirements.txt"

# Check if Python3 is installed
if ! command -v python3 &> /dev/null; then
    echo "Python3 is not installed. Please install it first."
    exit 1
fi

# Create virtual environment
echo "Creating virtual environment in ./$VENV_DIR ..."
python3 -m venv "$VENV_DIR"

# Activate virtual environment
source "$VENV_DIR/bin/activate"

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install requirements (for perception)
if [ -f "$REQ_FILE" ]; then
    echo "Installing dependencies from $REQ_FILE ..."
    pip install -r "$REQ_FILE"
else
    echo "$REQ_FILE not found! Please make sure it exists."
    deactivate
    exit 1
fi

pip install "fsd-path-planning @ git+https://git@github.com/papalotis/ft-fsd-path-planning.git"

echo "Setup complete. To activate the virtual environment, run:"
echo "source $VENV_DIR/bin/activate"
