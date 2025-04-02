# Setop of Ubuntu 22.04 on the Odroid XU4

1. Flash Ubuntu 22.04 image from official website
2. Enable ssh (`touch ssh` in `boot/` partition of the newly flashed drive)
3. Optional: setup hostname and hostname resolver
    1. Set hostname with `sudo hostnamectl set-hostname <hostname>`
    2. Install and setup avahi-daemon:
        ```bash
        sudo apt update
        sudo apt install avahi-daemon
        sudo systemctl enable avahi-daemon
        sudo systemctl start avahi-daemon
        ```
    You can now access the Odroid with `<hostname>.local` from other devices on the same network
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