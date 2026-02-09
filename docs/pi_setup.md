# Pi Setup

## Pi OS 32 

Only for simple testing, no ML, Pi 1 B+1

    # enable i2c
    sudo raspi-config

    # mount a flash usb for working space
    # the os might do this on plugging in
    mkdir ~/usbdrive
    mount /dev/sda1 ~/usbdrive
    # add the usb drive to fstab as optional,
    vi /etc/fstab
    # install required os dependencies
    sudo apt install python3-dev python3-setuptools
    curl -LsSf https://astral.sh/uv/install.sh | sh    
    # enable overlay file system (no persisted os changes past here)
    sudo raspi-config
    # reboot

After reboot pull and setup the autopilot

    cd ~/usbdrive
    git clone https://github.com/ieb/autopilot.git
    uv sync --extra rpi32
    # or just for imu testing
    uv sync --extra imu


uv sync will take time since the packages have to be compiled.

## Pi OS 64

TODO, probably the same as the above, but with

* uv sync --extra rpi