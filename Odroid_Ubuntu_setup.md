# Setop of Ubuntu 22.04 on the Odroid XU4

1. Flash Ubuntu 22.04 image from official website
2. Enable ssh (`touch ssh` in `boot/` partition of the newly flashed drive)
3. Optional: Setup a telegram bot to communicate the IP for ssh access
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
5. Setup git:
    1. Install git with `sudo apt install git`
    2. Configure git with your name and email:
        ```bash
        git config --global user.name "Your Name"
        git config --global user.email "your@email.com"
    3. Configure credential helper to store credentials:
        ```bash
        git config --global credential.helper store
        ```
    4. Generate a new token from github.com: Settings -> Developer settings -> Personal access tokens -> Generate new token
    5. Use the token as password when first using git to clone, push, or pull from a repository