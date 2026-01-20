**Note:** this is **ONLY** for UoL students with ACCESS to the Bragg Compute Cluster.


## ROS Jazzy Apptainer Setup 
Follow each bullet point in steps below.
>1. Open the terminal app in uni cluster (press Windows Key <kbd>⊞</kbd> and search for `terminal`). 

If you are using your own laptop go to [this section](#accessing-university-apptainer-containers-remotely-using-ssh-with-gui) to setup VPN and remotely access Bragg Computers.

1. Run the bragg setup script to get code in your shared bragg computer

Download the file `setup.sh` inside the apptainer folder and open up a terminal to follow the instructions below
```
chmod u+x setup.sh
bash setup.sh
```

When you see the REPL below you are inside your Apptainer container.
```
Apptainer>
```

>2. Inside your container build all the ros2 packages by following the commands below 
```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-skip bringup
```

NOTE: make sure you don't have any active python environments in background (mainly `conda`) or else `colcon` won't work

`Apptainer>`

>3. Sourcing environment and getting the `Apptainer>` repl in a new terminal session.

The `setup.sh` script created the following files: `~/.fsairc` and the alias: `fsai`

```bash
fsai
```

You are now inside the apptainer container: **ros_jazzy.sif** located in `/local/data/$USER/`.

When inside run the command to source ros2 and colcon_ws
```bash
source ~/.fsairc
```


You can now try running `ros2 launch simulation` command from the bottom of main [README](https://github.com/GryphonRacingAI/gra-ros2/blob/dev/README.md)

## Creating your Custom Apptainer Container with extra dependencies
- `cd /local/data/$USER`

That `ros_jazzy.sif` script takes around 10 minutes to complete, to add your own dependencies without waiting for 15 minutes or breaking things accidentally the `custom.def` file exists.

Familiarize yourself with containers if you haven't used containers before, if you have 
[apptainer docs](https://apptainer.org/documentation/) is self-explanatory.

>Add your custom dependencies or files in `/local/data/$USER`
- add any local files you can't extract from web eg things like installer (e.g. `ros_jazzy.def` used ZED_SDK installer, cuda key ring, .deb files etc.
- creating your own bash script e.g: `setup_username_custom.sh` is a good idea to keep track of files you added for your `custom.def`
- make sure to add to the `%files%` 
- usually done by `apt-get -y <ros-pkg-name>` in the `%runscript%` section of `custom.def`

>Build your custom container and enter it with: 
```bash
apptainer build custom.sif custom.def
apptainer shell --nv custom.sif
```
Update our `fsai` alias & create your `.fsarc` files for ease.

## Accessing University Apptainer containers remotely using SSH with GUI
Over weekends or out of uni hours you can access your containers using the method below.
>Get uol vpn: [Ivanti Secure Access Client Download](https://library.ucdavis.edu/vpn/)
- Add UOL VPN server url `<server-url>`
- Go to `feng-linux.leeds.ac.uk` on your browser
- `ssh -Y <username>@uol-pc-<id>` make sure to note the `<id>` of local pc where you installed your containers

**Note:** If above doesn't work pc must be remotely powered on by following the steps below:
```bashrc
module add wol
wol -h <ip> <mac>
```

> Continue setting things up by following the instructions [above](#ros-jazzy-apptainer-setup)

Email or message on Teams the Technical Director: `sc23pg@leeds.ac.uk` for the values of `<server-url>` as I am not sure I can share this in a public repo.

You can find your ip and mac using: `ip addr show` and hostname using `hostname`.

Turning on the computer may take a few minutes, then you can `ssh` into it.

## Setting up IPG Carmaker
Checkout out [Carmaker setup](./carmaker/README.md)

## Troubleshooting
**Disk Quota Limit Exceeded**:

You shouldn't get this if you have used `/local/data/$USER` if you do let Technical Director know.
- run `quota` to check your disk quota
- run `du -sh * | sort -h` at `~` to see where disk space is being used
- carefully use `rm` to free up space, be sure to backup or important files (eg git commits)

Note: the `ros_jazzy.sif` file should only take up: 12 GB of space.
- Read more about [Disk Quotas](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/7/html/storage_administration_guide/ch-disk-quotas) or contact admin if you still run into issues or need more space.

- This is why we are using the shared disk space in `/local/data` not the NFS shared across university networks, which has ~15 GB quota per student.

