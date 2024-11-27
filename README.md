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
        1. Install `curl` with `sudo apt install curl`
        2. Create the telegram bot:
            - Open Telegram and search for `@BotFather`
            - Create a new bot by typing `/newbot` and following the instructions
        3. Start the bot by opening a new chat with the bot and sending `/start`
        4. Add the bot to a group chat, then diable access to group chat in the bot settings:
            - writhe to `@BotFather` and send `/setjoingroups`
            - select the bot and disable the option
        5. Get the group chat id:
            - Add @getidsbot to the group chat
            - Get the chat id from the message it sends
            - Remove the bot from the group
        6. Create the script in `/opt/send_ip.sh`:
            ```bash
            #!/bin/bash

            # Telegram bot credentials
            BOT_TOKEN="YOUR_BOT_TOKEN"
            CHAT_ID="YOUR_CHAT_ID"
            
            # Get device name
            DEVICE_NAME=$(hostname)

            # Get current date and time
            CURRENT_DATE=$(date)

            # Get the IP address
            IP_ADDRESS=$(hostname -I | awk '{print $1}')

            # If no IP address is found, set a default message
            if [ -z "$IP_ADDRESS" ]; then
            IP_ADDRESS="No IP address found"
            fi

            # Telegram API URL
            API_URL="https://api.telegram.org/bot$BOT_TOKEN/sendMessage"

            # Compose the message
            MESSAGE="Device: $DEVICE_NAME
            Date: $CURRENT_DATE
            IP: $IP_ADDRESS"

            # Send the message to the Telegram bot
            curl -s -X POST $API_URL -d chat_id=$CHAT_ID -d text="$MESSAGE"
            ```
        7. Make the script executable with `sudo chmod +x /opt/send_ip.sh`
        8. Create a Systemd Service in `/etc/systemd/system/send_ip.service`:
            ```bash
            [Unit]
            Description=Send IP address to Telegram bot
            After=network-online.target
            Wants=network-online.target

            [Service]
            ExecStart=/opt/send_ip.sh
            Restart=on-failure  
            RestartSec=60
            User=root

            [Install]
            WantedBy=multi-user.target
            ```
        9. Enable the service with `sudo systemctl enable send_ip.service` and start it with `sudo systemctl start send_ip.service`
    4. Optional: setup NTP server to synchronize time with the ground station <!-- TODO add guide -->
    5. Setup persistent names of the usb devices:
        1. Copy the file `99-usb-serial.rules` to `/etc/udev/rules.d/`
        2. Reload the udev rules with `sudo udevadm control --reload-rules && sudo udevadm trigger`
        3. Optional: add new devices to the rule, the information can be found with the command `udevadm info --name=/dev/ttyUSBx --attribute-walk` for the exemplaty device `/dev/ttyUSBx`

2. Install and build this repository, **make sure the ROS2 installation is sourced**
    1. Clone the repository in your workspace source folder (adapt to your workspace if you already have one):
       ```bash
       mkdir -p ~/onboard_comp_ws/src/
       cd ~/onboard_comp_ws/src
       git clone https://github.com/polimi-saslab/onboard-computer
       cd onboard-computer
       git submodule update --init --recursive
       ```
    2. Navigate to the workspace, install dependencies, and build:
       ```bash
       cd ~/onboard_comp_ws
       rosdep install --from-paths src --ignore-src -r -y
       colcon build --symlink-install
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
