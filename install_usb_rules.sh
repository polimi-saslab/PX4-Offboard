sudo cp 99-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger