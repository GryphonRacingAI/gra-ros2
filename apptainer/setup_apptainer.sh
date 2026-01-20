#!bin/bash

# University Red Hat apptainer container setup in: '/local/data/$USER'

# Paste below into a shell

SETUP_DIR="/local/data/$USER"

echo "Setting up $SETUP_DIR for user $USER"

mkdir -p "$SETUP_DIR"
cp ros_jazzy.def "$SETUP_DIR"
cd "$SETUP_DIR"

echo "Replacing '/local/data/<username>/' with '$SETUP_DIR/'"

sed -i "s|/local/data/<username>|$SETUP_DIR|g" "$SETUP_DIR/ros_jazzy.def"

# Download ros_gz_bridge.py 
echo "Downloading ros_gz_bridge.py..."
wget -O ros_gz_bridge.py "https://raw.githubusercontent.com/gazebosim/ros_gz/0abd2b217d92ae7f65c1fa9f2a6464072217a038/ros_gz_bridge/ros_gz_bridge/actions/ros_gz_bridge.py"

# Download cuda keyring
echo "Downloading CUDA keyring..."
wget -O cuda-keyring_1.1-1_all.deb "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb"

# Manual download for ZED SDK 
echo "Downloading ZED SDK from official Stereolabs URL (this may take 10-30 minutes for ~2.4GB)..."
wget --continue -O ZED_SDK_Ubuntu24_cuda12.8_tensorrt10.9_v5.0.5.zstd.run https://download.stereolabs.com/zedsdk/5.0/cu12/ubuntu24

# zed_ros2_wrapper setup
echo "Setting up colcon workspace and cloning zed_ros2_wrapper..."
mkdir -p colcon_ws/src
cd colcon_ws/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git

cd "$SETUP_DIR"
echo "Setup complete! Files are ready in $SETUP_DIR/"
apptainer build --disable-cache --nv ros_jazzy.sif ros_jazzy.def

# Create another env that the user can source
echo "Bootstrap: localimage
From: $SETUP_DIR/ros_jazzy.sif

%post
    # User-customizable post section
    # Add your custom installations, package builds, or commands here.

%environment
    # Inherit and ensure base environment is sourced
    export GZ_SIM_RESOURCE_PATH=/uolstore/home/users/$USER/colcon_ws/install/simulation/share/
    source /opt/ros/jazzy/setup.bash
    
    # CUDA paths (if needed for your additions)
    export PATH=/usr/local/cuda-12.8/bin${PATH:+:${PATH}}
    export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

%runscript
    # Optional: Customize the default runscript if needed" > $SETUP_DIR/custom.def

sed -i "s|<username>|g" "$USER"

# Cleanup to not waste disk space
rm -r $SETUP_DIR/ros_gz_bridge.py $SETUP_DIR/cuda-keyring_1.1-1_all.deb $SETUP_DIR/ZED_SDK_Ubuntu24_cuda12.8_tensorrt10.9_v5.0.5.zstd.run $SETUP_DIR/colcon_ws

