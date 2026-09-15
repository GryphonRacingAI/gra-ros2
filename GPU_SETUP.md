# NVIDIA GPU setup

The default path-planning pipeline needs a working NVIDIA GPU because
`tmux/startup.sh` starts the YOLO perception node with `device:=cuda:0`. YOLO
publishes `/cone_array`, which `track_pathfinder` consumes. There is no CPU
fallback in this launch configuration.

The `path_planning` package itself does not load a Torch model. Its GPU
dependency is indirect when it is run as part of the default YOLO-to-path
pipeline.

These instructions target the repository's supported native environment:
Ubuntu 24.04 on a machine with an NVIDIA GPU. Do not copy a driver version from
another contributor's machine. Let Ubuntu select a driver compatible with the
GPU and current kernel.

## 1. Confirm that the machine has an NVIDIA GPU

```bash
lspci -nnk | grep -A3 -i 'vga\|3d\|display'
```

An NVIDIA device should appear in the output. A virtual machine, container, or
remote compute environment must also expose the physical GPU to the guest. If
the machine has no supported NVIDIA GPU, use the project's
[Apptainer environment](https://github.com/GryphonRacingAI/apptainer) on a
GPU-equipped university machine.

## 2. Install Ubuntu's recommended NVIDIA driver

Remove neither an existing driver nor a working CUDA installation just to
follow this guide. First inspect the current state:

```bash
nvidia-smi
```

If that already lists the GPU without an error, continue to the Python setup.
Otherwise install the driver selected for this machine:

```bash
sudo apt update
sudo apt install ubuntu-drivers-common
ubuntu-drivers devices
sudo ubuntu-drivers install
sudo reboot
```

After the reboot, verify the kernel driver and GPU:

```bash
nvidia-smi
lsmod | grep '^nvidia'
```

`nvidia-smi` must list the GPU and a driver version. The "CUDA Version" shown
there is the maximum CUDA version supported by the installed driver; it does
not prove that the repository's Python environment can use CUDA.

For alternative installation modes, including servers that should use the
`-server` driver branch, follow Ubuntu's
[official NVIDIA driver documentation](https://documentation.ubuntu.com/server/how-to/graphics/install-nvidia-drivers/).

### Secure Boot

Check Secure Boot with:

```bash
mokutil --sb-state
```

With Secure Boot enabled, Ubuntu may ask you to create a Machine Owner Key
(MOK) password during driver installation and enroll that key on the next
boot. Complete that firmware-screen enrollment; otherwise the NVIDIA kernel
modules can be installed but blocked from loading. Follow Ubuntu's signed
driver/MOK flow rather than disabling Secure Boot unless the machine owner has
chosen to do so.

## 3. Install and verify Torch in the project environment

The launcher probes `~/colcon_ws/ros_venv/bin/python3` when it exists, so test
that exact environment rather than the system Python:

```bash
cd ~/colcon_ws
python3 -m venv ros_venv
source ros_venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r src/perception/ultralytics_ros/requirements.txt
```

The perception requirements install Ultralytics and Torch. A separate
system-wide CUDA Toolkit is normally unnecessary because the official PyTorch
packages include the required CUDA runtime libraries. If a CPU-only Torch
build was installed, select the command appropriate for Linux, Pip, Python,
and the supported CUDA version on the
[official PyTorch installer](https://pytorch.org/get-started/locally/), then
run it inside `ros_venv`.

Verify both CUDA discovery and a real allocation:

```bash
~/colcon_ws/ros_venv/bin/python3 -c "import torch; print('torch', torch.__version__); print('built for CUDA', torch.version.cuda); assert torch.cuda.is_available(); torch.zeros(1, device='cuda'); print('cuda ok:', torch.cuda.get_device_name(0))"
```

The command must finish with `cuda ok: <GPU name>`. Merely importing Torch is
not sufficient.

## 4. Run the repository preflight

From the workspace root:

```bash
cd ~/colcon_ws
./src/tmux/reload_nvidia_uvm.sh
```

Despite its name, this helper is diagnostic and does not run `sudo` or reload
anything by itself. When it reports that CUDA is usable, start the stack:

```bash
./src/tmux/startup.sh
```

With the default `CONES=yolo`, `startup.sh` repeats the Torch allocation test
before starting tmux. On failure it stops early and writes details to
`~/colcon_ws/logs/latest/gpu.log` (or the active `$LOG_DIR/gpu.log`).

Do not use `GPU_SKIP=1` as a fix. It only skips the preflight; YOLO still starts
with `device:=cuda:0` and will fail until CUDA works.

## Troubleshooting

| Symptom | Likely cause | Action |
|---|---|---|
| `nvidia-smi: command not found` | NVIDIA userspace tools/driver are not installed | Run the Ubuntu driver installation in step 2, then reboot. |
| `NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver` | Kernel module is missing, blocked, or does not match the running kernel | Reboot first. Check `mokutil --sb-state`, `lsmod`, `uname -r`, and `dkms status`; reinstall the Ubuntu-recommended driver if needed. |
| `nvidia-smi` shows no GPU | The hardware is absent or not passed through | Check the host, BIOS/PCI configuration, VM GPU passthrough, or container GPU configuration. |
| `torch import failed` | Torch is not installed in `ros_venv` | Activate `ros_venv` and install the perception requirements. |
| `torch.version.cuda` is `None` | A CPU-only Torch build is installed | Reinstall Torch in `ros_venv` using the official PyTorch selector. |
| `CUDA driver version is insufficient for CUDA runtime version` | The driver is too old for the installed Torch CUDA runtime | Update to Ubuntu's recommended driver, or install a compatible Torch build. |
| `nvidia-smi` works but `torch.cuda.is_available()` is false with `CUDA unknown error` | `nvidia_uvm` may be wedged | Stop GPU-using processes, then follow the recovery below. |
| `Failed to initialize NVML: Driver/library version mismatch` | Driver packages changed without the running kernel module changing | Reboot, then run both verification checks again. |

### Recover a wedged `nvidia_uvm`

First run the repository helper; it records the diagnosis and prints the same
recovery commands:

```bash
cd ~/colcon_ws
./src/tmux/reload_nvidia_uvm.sh
```

If `nvidia-smi` works and the helper identifies the UVM failure, close programs
using the GPU and run:

```bash
sudo rmmod nvidia_uvm
sudo modprobe nvidia_uvm
~/colcon_ws/ros_venv/bin/python3 -c "import torch; assert torch.cuda.is_available(); torch.zeros(1, device='cuda'); print('cuda ok', torch.cuda.get_device_name(0))"
```

If `rmmod` reports that the module is in use, stop the tmux stack and other GPU
applications before retrying. Rebooting is the simpler recovery when it is not
safe to stop those processes individually.

## Information to include when asking for help

Do not post only "CUDA does not work." Include the output of:

```bash
lspci -nnk | grep -A3 -i 'vga\|3d\|display'
nvidia-smi
uname -r
mokutil --sb-state
dkms status
~/colcon_ws/ros_venv/bin/python3 -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"
```

Also attach `~/colcon_ws/logs/latest/gpu.log` if `startup.sh` created it. Review
logs for machine names or other sensitive details before sharing them publicly.
