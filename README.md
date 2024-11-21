# Onboard Computer Package

This package contains all the necessary components for an Odroid used as an onboard companion with Pixhawk controllers running PX4 firmware.

## Requirements

- This package is meant to be used with a Pixhawk running PX4 >1.15, running an XRCE server
- The use of this packages requires a working ROS2 humble installation, this can be done from the official guides, or, in case of an Odroid, following  section 1 of [this guide](https://github.com/polimi-saslab/4DDS-Project/blob/main/Documentation/gbeam_on_odroid.md#1-setting-up-the-odroid-xu4).
- Setup of a swap memory on the odroid is preferrable, to avoid memory issues during compilation, 8GB is suggested.

## Setup

1. Setup the Odroid with ROS2:
    1. Flash Ubuntu 22.04 image from official website
    2. Enable ssh (`touch ssh` in `boot/` partition of the newly flashed drive)
    3. Optional: Setup a telegram bot to communicate the IP for ssh access <!-- TODO add guide -->
    4. Optional: setup NTP server to synchronize time with the ground station <!-- TODO add guide -->
    5. Setup persistent names of the usb devices:
        1. Copy the file `99-usb-serial.rules` to `/etc/udev/rules.d/`
        2. Reload the udev rules with `sudo udevadm control --reload-rules && sudo udevadm trigger`
        3. Optional: add new devices to the rule, the information can be found with the command `udevadm info --name=/dev/ttyUSBx --attribute-walk` for the exemplaty device `/dev/ttyUSBx`

2. Install and build this repository, **make sure the ROS2 installation is sourced**
    1. Clone the repository in your workspace source folder (adapt to your workspace if you already have one):
       ```bash
       mkdir -p ~/onboard_ws/src/
       cd ~/onboard_ws/src
       git clone https://github.com/yourusername/onboard-computer.git
       ```
    2. Navigate to the workspace, install dependencies, and build:
       ```bash
       cd /home/leonardo/onboard_comp_ws
       rosdep install --from-paths src --ignore-src -r -y
       catkin_make
       ```
3. Setup the XRCE connection with Pixhawk
    1. Setup the Pixhawk and enable XRCE on the telemetry port in use, with baud rate `921600`.
    2. Install the MicroXRCE agent on the Odroid following [this guide](https://docs.px4.io/main/en/middleware/uxrce_dds.html#install-standalone-from-source). <!-- TODO add connection guide -->

## Usage

1. Source the environment in all terminals
    ```bash
        cd ~/onboard_ws/src
        source install/setup.bash
    ```
2. Launch everything:
    ```bash
        ros2 launch px4_custom_fn obc_launch.py
    ```
3. Optional: launch rviz to visualize.
    ```bash
        rviz2 -d ~/onboard_ws/src/px4_custom_fn/px4_custom_fn/rviz/obc.rviz
    ```

## Support

For any issues or questions, please open an issue on the [GitHub repository](https://github.com/polimi-saslab/onboard-computer/issues).
