#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import spidev
import math

class AS5048AEncoder(Node):
    def __init__(self):
        super().__init__('as5048a_encoder')

        # Declare parameters and their default values
        self.declare_parameter('encoder_resolution', 16383.0)

        # Get parameter values
        self.encoder_resolution = self.get_parameter('encoder_resolution').get_parameter_value().double_value

        self.raw_angle_publisher_ = self.create_publisher(Float32, 'raw_angle', 10)
        self.cumulative_angle_publisher_ = self.create_publisher(Float32, 'cumulative_angle', 10)
        
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1000000
        self.spi.mode = 0b01

        self.previous_angle = None
        self.cumulative_angle = 0.0

        # Check if the device is connected
        if not self.is_device_connected():
            self.get_logger().error("Error: AS5048A device not connected. Shutting down.")
            rclpy.shutdown()
        else:
            self.get_logger().info("AS5048A device successfully connected.")
            
        # Reset cumulative angle at startup and publish initial 0 value
        self.reset_cumulative_angle()
        self.publish_initial_cumulative_angle()

    def is_device_connected(self):
        try:
            command = [0x00, 0x00]  # Send a NOP command
            response = self.spi.xfer2(command)
            if len(response) == 2 and (response[0] != 0x00 or response[1] != 0x00):
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f"SPI communication failed: {e}")
            return False

    def check_connection(self):
        command = [0xFF, 0xFF]
        response = self.spi.xfer2(command)
        if len(response) == 2:
            raw_angle = ((response[0] << 8) | response[1]) & 0x3FFF
            if raw_angle != 0xFFFF:  # 0xFFFF indicates an error or no response
                return True
        return False

    def reset_cumulative_angle(self):
        # Read the initial angle to set as the base for cumulative calculation
        command = [0xFF, 0xFF]
        response = self.spi.xfer2(command)
        if len(response) == 2:
            raw_angle = ((response[0] << 8) | response[1]) & 0x3FFF
            initial_angle_degrees = (raw_angle / self.encoder_resolution) * 360.0
            self.previous_angle = initial_angle_degrees * (math.pi / 180.0)  # Convert to radians
            self.cumulative_angle = 0.0

    def publish_initial_cumulative_angle(self):
        # Publish an initial cumulative angle of 0
        cumulative_angle_msg = Float32()
        cumulative_angle_msg.data = 0.0
        self.cumulative_angle_publisher_.publish(cumulative_angle_msg)

    def timer_callback(self):
        cumulative_angle_radians, raw_angle_radians = self.read_angle()
        if cumulative_angle_radians is not None:
            # Prepare and publish raw angle message
            raw_angle_msg = Float32()
            raw_angle_msg.data = raw_angle_radians
            self.raw_angle_publisher_.publish(raw_angle_msg)

            # Prepare and publish cumulative angle message
            cumulative_angle_msg = Float32()
            cumulative_angle_msg.data = cumulative_angle_radians
            self.cumulative_angle_publisher_.publish(cumulative_angle_msg)
            
            # logs
            #self.get_logger().info(f'Publishing: {cumulative_angle_radians} radians, {raw_angle_radians} radians')

    def read_angle(self):
        command = [0xFF, 0xFF]
        response = self.spi.xfer2(command)
        if len(response) != 2:
            self.get_logger().error("Error: Incomplete response received")
            return None, None

        raw_angle = ((response[0] << 8) | response[1]) & 0x3FFF
        angle_degrees = (raw_angle / self.encoder_resolution) * 360.0
        angle_radians = angle_degrees * (math.pi / 180.0)

        if self.previous_angle is not None:
            delta_angle = angle_radians - self.previous_angle
            if delta_angle < -math.pi:
                delta_angle += 2 * math.pi
            elif delta_angle > math.pi:
                delta_angle -= 2 * math.pi
            self.cumulative_angle += delta_angle

        self.previous_angle = angle_radians
        return self.cumulative_angle, angle_radians

    def destroy_node(self):
        self.spi.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    encoder = AS5048AEncoder()

    try:
        rclpy.spin(encoder)
    except KeyboardInterrupt:
        encoder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
