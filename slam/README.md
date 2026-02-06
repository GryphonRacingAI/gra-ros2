# Pre-requisites
### Python virtual environment setup

```bash
cd ~/colcon_ws
python3 -m venv ros_venv
source ros_venv/bin/activate
```

**Note:** If the above commands didn't work, you may need to install python3-venv first:

```bash
sudo apt install python3-venv
```

### SLAM Dependency installations

`slam` package relies on our custom fork of the `fastslam64` package.

After setting up your Python virtual environment as described [above](#python-virtual-environment-setup), run the commands below once to install all dependencies.

```bash
pip install git+https://github.com/GryphonRacingAI/fastslam64.git
```

**Note:** If the above commands didn't work, due to issues with `pycuda` installation run the command below and try again:
```bash
sudo apt install nvidia-cuda-toolkit
```


Now you should be able to run the `fast_slam_node.py` as mentioned in [the main README](../README.md#launch-slam) 

### Altering the `fastslam64` package
Recommended approach to altering the `fastslam64` package is as follows:

> Clone in your workspace

```bash
cd ~/colcon_ws
git clone https://github.com/GryphonRacingAI/fastslam64.git
```

> If you want to test your changes to fastslam64

```bash
source ~/ros_venv/bin/activate
cd ~/colcon_ws/fastslam64
pip install -e .
```

**NOTE: if your changes are not merged to main, uninstall fastslam64 to avoid conflicts with different versions**

```bash
pip uninstall fastslam64  # Remove the your editted version
pip install git+https://github.com/GryphonRacingAI/fastslam64.git
```

> Once changes are merged to main branch
```bash
source ~/ros_venv/bin/activate
pip install --upgrade git+https://github.com/GryphonRacingAI/fastslam64.git
```

