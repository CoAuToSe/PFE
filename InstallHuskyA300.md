# Installation of the Husky A300 from scratch

Based on [Clearpath Robotics documentation](https://docs.clearpathrobotics.com/docs/ros/installation/robot/) for installation and other sources provided by the support of Clearpath Robotics.

## Requirements

A DHCP server: an internet connection without any login.

An ethernet cable to connect the computer inside the Husky A300 to the DHCP server.

A Wi-Fi connection that you can log in to using `netplan` (if you have a Wi-Fi router, after usage of the internet cable, you can connect it as an internet source for the router and it will provide a Wi-Fi connection if the router was "correctly initialized").

## Installation

### Flashing the OS

Download: [Clearpath ISO image](https://packages.clearpathrobotics.com/stable/images/2.0.0/) (a preconfigured Ubuntu 24.04 server ISO)

Use [Rufus](https://rufus.ie/) or any preferred software to flash the ISO file on a USB stick.

Open the top plate of the Husky A300.

Put the flashed USB stick into one of the available USB ports of the onboard computer.

While making sure where they belong (as you will need to reconnect them later), disconnect all ethernet ports from the onboard computer.

Connect the DHCP server via the ethernet cable to the onboard computer.

Connect a screen and a keyboard (United States keyboard layout) to the external ports of the Husky A300 (you may use the ports from the onboard computer but at this step external ports should still work).

Start the Husky A300 using the external button.

Change the boot option to boot from the USB stick. To do so you will have to press a specific key (usually around F1-F4 or F9-F12, you may spam them during start to ensure either boot choice or BIOS access to change the boot order).

[Extra possible step here.](#extra-possible-step)

Your Husky A300 will shut down its computer, meaning that the lights will still be on but the screen will be shut down.

Restart the Husky A300 using the external power button.

#### Extra possible step

During initialization you may be required to log in: `robot` password: `clearpath` (beware if your keyboard is not in QWERTY) but do not try anything else as the computer is supposed to still install things but may require the login.

### Software Installation

Log in using: `robot` and `clearpath` (beware if your keyboard is not in QWERTY).

If everything went well, then when using `ls` you should see the scrip `clearpath_computer_installer.sh` which means that the step [Flashing the OS](#flashing-the-os) did not have any issue and had internet connection.

Make sure that you still have internet using `ping google.com`. Use it on a computer that has internet to see usual printing.

At this point you may want to connect to the Husky A300 by SSH. For that go to [Wi-Fi setup](#wi-fi-setup) and then [SSH setup](#ssh-connection).

Run the command below:

```bash
bash -e clearpath_computer_installer.sh
```

[Extra possible steps here if there is any error message.](#ros2-jazzy-did-not-install-correctly) It might say that it was not able to ping Clearpath Robotics GitLab, but I ignored this error and everything went well.

Make sure that the Clearpath services are correctly running by using:

```bash
sudo systemctl status clearpath-platform
```

If not, see this [extra possible step](#service-did-not-launch-correctly). (It might be a necessary step.)

At this point, if everything went well, then you should proceed to the [Wi-Fi setup](#wi-fi-setup) if not already done.

#### Extra possible steps

##### ROS2 Jazzy did not install correctly

During installation I had an issue with the installation of ROS2 saying that `/tmp/ros2-apt-source.deb` did not exist.
To fix this issue I had to do the installation of ROS2 Jazzy by myself.

At this step there are two solutions: you either do the installation yourself following the steps from [ROS 2 Documentation: Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) or you change the installation script `clearpath_computer_installer.sh`, rewriting:

```sh
step_setup_osrf_packge_server() {
  log_info "Setup Open Robotics package server to install ROS 2 $ROS_DISTRO_MANUAL"

  if [ -e /etc/apt/sources.list.d/ros2.list ]; then
    # Remove old ROS 2 installation if present
    # See https://discourse.ros.org/t/ros-signing-key-migration-guide/43937
    log_info "Detected old ROS 2 installation"
    sudo rm /etc/apt/sources.list.d/ros2.list
    sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
  fi

  # Check if ROS 2 sources are already installed
  if dpkg -s ros2-apt-source &> /dev/null; then
    log_warn "ROS 2 sources exist, skipping"
  else
    sudo apt -y -qq install software-properties-common
    sudo add-apt-repository universe -y
    sudo apt -y -qq update && sudo apt -y -qq upgrade && sudo apt -y -qq install curl -y

    # See https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
    # We've had issues with api.github.com having rate-limiting issues, so use git ls-remote instead or curl
    export ROS_APT_SOURCE_VERSION=$(git ls-remote --tags https://github.com/ros-infrastructure/ros-apt-source.git | \
      awk -F/ '{print $NF}' | \
      sort -V | \
      tail -n 1
    )

    if [ -z "${ROS_APT_SOURCE_VERSION}" ]; then
      # Just in case the above fails (e.g. more rate-limiting), keep a known fall-back version
      # but log this event so we can keep tabs on it if/how often this occurs.
      log_warn "Failed to fetch latest apt-source version from GitHub. Falling back to 1.1.0"
      export ROS_APT_SOURCE_VERSION="1.1.0"
    fi

    # URL should resolve to something like https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.noble_all.deb
    GH_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_VERSION}_all.deb"
    wget -L -o /tmp/ros2-apt-source.deb "${GH_URL}"
    ret=$?
    if [ "$ret" != "0" ] ; then
      log_error "Failed to download OSRF apt sources from ${GH_URL}: code ${ret}. Exiting"
      exit 0
    fi
    sudo apt install /tmp/ros2-apt-source.deb
    sudo rm /tmp/ros2-apt-source.deb

    # Check if sources were added
    if ! dpkg -s ros2-apt-source &> /dev/null; then
      log_error "Unable to add ROS 2 package server, exiting"
      exit 0
    fi
  fi

  log_done "Setup ROS 2 package server"
}
```

Into:

```sh
step_setup_osrf_packge_server() {
  log_info "Setup Open Robotics package server to install ROS 2 $ROS_DISTRO_MANUAL"

  if [ -e /etc/apt/sources.list.d/ros2.list ]; then
    # Remove old ROS 2 installation if present
    # See https://discourse.ros.org/t/ros-signing-key-migration-guide/43937
    log_info "Detected old ROS 2 installation"
    sudo rm /etc/apt/sources.list.d/ros2.list
    sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
  fi

  # Check if ROS 2 sources are already installed
  if dpkg -s ros2-apt-source &> /dev/null; then
    log_warn "ROS 2 sources exist, skipping"
  else
    sudo apt -y -qq install software-properties-common
    sudo add-apt-repository universe -y
    sudo apt -y -qq update && sudo apt -y -qq upgrade && sudo apt -y -qq install curl -y

    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

    if [ -z "${ROS_APT_SOURCE_VERSION}" ]; then
      # Just in case the above fails (e.g. more rate-limiting), keep a known fall-back version
      # but log this event so we can keep tabs on it if/how often this occurs.
      log_warn "Failed to fetch latest apt-source version from GitHub. Falling back to 1.1.0"
      export ROS_APT_SOURCE_VERSION="1.1.0"
    fi

    # URL should resolve to something like https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.noble_all.deb
    GH_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_VERSION}_all.deb"
    curl -L -o /tmp/ros2-apt-source.deb "${GH_URL}"
    ret=$?
    if [ "$ret" != "0" ] ; then
      log_error "Failed to download OSRF apt sources from ${GH_URL}: code ${ret}. Exiting"
      exit 0
    fi
    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo rm /tmp/ros2-apt-source.deb

    # Check if sources were added
    if ! dpkg -s ros2-apt-source &> /dev/null; then
      log_error "Unable to add ROS 2 package server, exiting"
      exit 0
    fi
  fi

  log_done "Setup ROS 2 package server"
}
```

###### Service did not launch correctly

If by running `sudo systemctl status clearpath-platform` you can see that the service is not active then run:

```bash
ros2 run clearpath_robot install
sudo systemctl daemon-reload && sudo systemctl start clearpath-robot
```

### Wi-Fi setup

#### With Internet

Use the software provided by Clearpath Robotics to set up your Wi-Fi:

```bash
sudo apt install python3-clearpath-computer-setup
sudo clearpath-computer-setup
```

Then follow the steps from [Clearpath Robotics networking documentation](https://docs.clearpathrobotics.com/docs/ros/networking/computer_setup).

If anything goes wrong you might resort to following the steps [without internet](#without-internet).

#### Without Internet

You may want to follow the steps from [this part](https://docs.clearpathrobotics.com/docs/ros/installation/robot/#internet-connection) as it is supposedly the same but I will try to be clearer.

You will have to create a file `/etc/netplan/60-wireless.yaml` that describes the network that you will connect to.

> `/etc/netplan/50...` is the file configuring the network inside the Husky A300 so do not remove it.

First run:

```bash
ip a
```

It will display every connection port available by the computer. You will be looking for something looking like `wlp2s0`, `wlp3s0` or `wlan0`, this will be your `${WIRELESS_INTERFACE}`.

`${SSID_GOES_HERE}` will be the name of your network.

`${PASSWORD_GOES_HERE}` will be your password.

```yaml
network:
  wifis:
    # Replace ${WIRELESS_INTERFACE} with the name of the wireless network device, e.g. wlan0 or wlp3s0
    # Fill in the SSID and PASSWORD fields as appropriate.  The password may be included as plain-text
    # or as a password hash.  To generate the hashed password, run
    #   echo -n 'WIFI_PASSWORD' | iconv -t UTF-16LE | openssl md4 -binary | xxd -p
    # If you have multiple wireless cards you may include a block for each device.
    # For more options, see https://netplan.io/reference/
    ${WIRELESS_INTERFACE}:
      optional: true
      access-points:
        ${SSID_GOES_HERE}:
          password: ${PASSWORD_GOES_HERE}
      dhcp4: true
      dhcp4-overrides:
        send-hostname: true
```

For example:

```yaml
network:
  wifis:
    wlp2s0:
      optional: true
      access-points:
        myphone:
          password: mypassword
      dhcp4: true
      dhcp4-overrides:
        send-hostname: true
```

Now create the file using your preferred editor:

```bash
sudo nano /etc/netplan/60-wireless.yaml
```

```bash
sudo vim /etc/netplan/60-wireless.yaml
```

After writing the file run:

```bash
sudo netplan apply
```

You now need to check if the Wi-Fi connection was made:

```bash
ip a
```

If you can see an IP address `192.168.X.X` (we will refer to the value as `${IP_HUSKY_A300}`) that is not `192.168.131.1` you can go to the [firmware update step](#firmware-update). Otherwise, do the following steps:

```bash
sudo ip link set dev ${WIRELESS_INTERFACE} up
```

You may be required to restart to apply these changes:

```bash
sudo reboot
```

Check `ip a` again after the reboot, if everything went well now the Husky A300 should be connected to your Wifi network.

### SSH connection

You will need to be connected to the same network as the Husky A300, then run:

```bash
ssh robot@${IP_HUSKY_A300}
```

For example:

```bash
ssh robot@192.162.50.134
```

If you can successfully connect via SSH, at this point it is advised to disconnect the internet connection inside the Husky A300. Before closing the Husky you must reconnect the two ethernet connections you disconnected during the step [Flashing OS](#flashing-the-os).

### Firmware Update

#### Requirements for firmware update

Step [Wi-Fi setup](#wi-fi-setup) done.
Step [SSH setup](#ssh-connection) done.

Reconnect the internal ethernet cables that you disconnected at the start during the step [Flashing OS](#flashing-the-os).

Disconnect any external cable.

Place the Husky A300 in a position where all the wheels can turn freely (example: by turning it upside down, but beware of the antennas and their connectors as you might damage them with the weight of the Husky A300).

#### Updating the Firmware

After making sure that every step of the [requirements](#requirements-for-firmware-update) is done, run:

```bash
sudo apt-get update
sudo apt-get install ros-jazzy-clearpath-firmware
```

```bash
source /opt/ros/jazzy/setup.bash
ros2 run clearpath_firmware flash
```

After finishing the flashing, the Husky will turn off.

### Remote Controller

#### Pairing the Controller

You may want to follow this [tutorial](https://docs.clearpathrobotics.com/docs/ros/installation/controller). Here is a copy:

##### ds4drv-pair

> If your robot comes with a PS4 controller, it will be paired already. Simply turn on the robot and turn on the controller.

To re-pair the PS4 controller or pair a new PS4 controller:

Install the python3-ds4drv package if it is not installed already. In terminal, run:

```bash
sudo apt-get install python3-ds4drv
```

Put the controller in pairing mode. Press and hold the PS and Share buttons on your controller until the LED begins rapidly flashing white.
Run the ds4drv-pair script to pair the controller to the computer. In terminal, run:

```bash
sudo ds4drv-pair
```

This script will scan for nearby Bluetooth devices, and pair automatically to the controller.

Alternatively, if ds4drv-pair fails to detect the controller, you can pair the controller using [bluetoothctl](#bluetoothctl).

##### bluetoothctl

Install the bluez package if it is not installed already. In terminal, run:

```bash
sudo apt-get install bluez
```

Run the bluetoothctl command. In terminal, run:

```bash
sudo bluetoothctl
```

Use bluetoothctl to scan for nearby devices. In the bluetooth control application, run:

```bluetoothctl
agent on
scan on
```

Put the controller in pairing mode. Press and hold the PS and Share buttons on your controller until the LED begins rapidly flashing white.
The bluetooth scan will display the MAC addresses of nearby devices. Determine which MAC address corresponds to the controller and copy it. In the bluetooth control application, run:

```bluetoothctl
scan off
pair ${MAC Address}
trust ${MAC Address}
connect ${MAC Address}
```

The controller should now be paired.

> Make sure to set the correct controller type in your robot.yaml file. See Joystick Controller setup in the Clearpath Configuration.
