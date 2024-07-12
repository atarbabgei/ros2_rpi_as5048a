
# ROS2 RPi AS5048A Encoder Package

This package provides a ROS 2 node for interfacing with the AS5048A encoder via SPI on a Raspberry Pi. The node publishes encoder data to various ROS topics.

**Tested on:**
- Raspberry Pi 4B, Raspberry Pi CM4
- Ubuntu 22.04, ROS2 Humble

## Features

- Reads data from the AS5048A encoder over SPI.
- Publishes raw angle and cumulative angle to ROS topics.

## Dependencies

This package depends on several ROS 2 packages and the `spidev` Python package.

### ROS 2 Dependencies

- `rclpy`
- `std_msgs`

### Python Dependencies

- `spidev`

## Installation

### ROS 2 and Python Dependencies

1. **Ensure ROS 2 is installed** on your Raspberry Pi. You can follow the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).

2. **Install ROS 2 Python dependencies**:
    ```bash
    sudo apt update
    sudo apt install ros-humble-rclpy ros-humble-std-msgs 
    ```

3. **Install `spidev`** for Python:
    ```bash
    sudo apt install python3-pip
    sudo pip3 install spidev
    ```

### Package Setup

1. **Clone the repository** into your ROS 2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/atarbabgei/ros2_rpi_as5048a.git  
    ```

2. **Build the package**:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

3. **Source the setup file** to overlay the new package into your environment:
    ```bash
    source install/setup.bash
    ```

## Hardware Connection

### IMPORTANT: Power Supply Mode

**Ensure that the AS5048A is running in 3.3V mode.** Connecting the AS5048A to 5V mode may damage the Raspberry Pi. Verify the jumper settings or power connections on the AS5048A to make sure it operates at 3.3V.

### Connect SPI0 between AS5048A and Raspberry Pi

| AS5048A Pin | Raspberry Pi Pin         |
|-------------|--------------------------|
| VCC         | Pin 1 or Pin 17 (3.3V)   |
| GND         | Pin 6 (GND) or any other GND pin |
| MOSI        | GPIO 10 (SPI0 MOSI)      |
| MISO        | GPIO 9 (SPI0 MISO)       |
| SCK         | GPIO 11 (SPI0 SCLK)      |
| CS          | GPIO 8 (SPI0 CE0)        |

### Enable SPI on your Raspberry Pi

If using Ubuntu 22.04 OS, make sure SPI is enabled in `config.txt`:
```
dtparam=spi=on
```

Or if using Raspberry Pi OS:
```bash
sudo raspi-config
```
Navigate to `Interfacing Options` > `SPI` and enable it.

## Usage

**Run the node using the launch file**:
```bash
ros2 launch ros2_rpi_as5048a encoder.launch.py
```

## Node Details

### Parameters

- `encoder_resolution` (float): The resolution of the AS5048A encoder.

### Published Topics

- `/encoder_absolute_angle` (`std_msgs/Float32`): The absolute angle in radians (between 0 to $2\pi$).
- `/encoder_cumulative_angle` (`std_msgs/Float32`): The cumulative angle in radians.
- `/encoder_angular_velocity` (`std_msgs/Float32`): The angular velocity in rads/s.

## License

MIT