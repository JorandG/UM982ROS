#!/usr/bin/env python

import rospy
import sys
import math
import re
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler
import serial
import time


class UM982Serial:
    def __init__(self, port, baud):
    	#print("init...")
        try:
            print("init...")
            self.serial = serial.Serial(port, baud, timeout=1)
            if not self.serial.is_open:
                self.serial.open()
            rospy.loginfo(f'Serial port {port} opened successfully at baud rate {baud}!')
        except serial.SerialException as e:
            print("init...")
            rospy.logerr(f"Failed to open serial port {port}: {e}")
            raise e

    def read_line(self):
        """Read a line of data from the serial port."""
        try:
            return self.serial.readline().decode('ascii', errors='ignore').strip()
        except Exception as e:
            rospy.logwarn(f"Error reading from serial port: {e}")
            return None

    def stop(self):
        if self.serial.is_open:
            self.serial.close()
            rospy.loginfo("Serial connection closed.")


class UM982DriverROS1:
    
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('um982_serial_driver', anonymous=True)
        
        # Get parameters from the parameter server
        port = rospy.get_param('~port', '/dev/ttyUM982')
        baud = rospy.get_param('~baud', 115200)
        
        # Set up the serial connection
        try:
            self.um982serial = UM982Serial(port, baud)
        except serial.SerialException:
            rospy.logerr("Exiting due to serial connection failure.")
            sys.exit(0)

        # Set up publishers
        self.fix_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        self.utm_pub = rospy.Publisher('/gps/utmpos', Odometry, queue_size=10)
        self.heading_pub = rospy.Publisher('/gps/heading', Float64, queue_size=10)

        # Create a timer to read and publish data at 20 Hz
        self.pub_timer = rospy.Timer(rospy.Duration(1/20.0), self.pub_task)

        # Initialize variables for position and heading
        self.heading = None
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.position_std_dev = [0.0, 0.0, 0.0]

    def parse_gga(self, data):
        """Parse the GGA sentence for latitude, longitude, and altitude."""
        match = re.match(r'\$GNGGA,\d+\.\d+,(?P<lat>\d+\.\d+),(?P<lat_dir>[NS]),(?P<lon>\d+\.\d+),(?P<lon_dir>[EW]),\d+,\d+,\d+\.\d+,(?P<alt>\d+\.\d+),M', data)
        if not match:
            return None

        lat = float(match.group('lat'))
        lon = float(match.group('lon'))
        alt = float(match.group('alt'))

        # Convert latitude and longitude to decimal degrees
        lat_deg = int(lat / 100) + (lat % 100) / 60
        lon_deg = int(lon / 100) + (lon % 100) / 60

        if match.group('lat_dir') == 'S':
            lat_deg = -lat_deg
        if match.group('lon_dir') == 'W':
            lon_deg = -lon_deg

        self.latitude = lat_deg
        self.longitude = lon_deg
        self.altitude = alt

    def parse_ths(self, data):
        """Parse the THS sentence for heading."""
        match = re.match(r'\$GNTHS,(?P<heading>\d+\.\d+),A\*', data)
        if match:
            self.heading = float(match.group('heading'))

    def pub_task(self, event):
        # Read data from the serial port
        line = self.um982serial.read_line()
        if line is None:
            return

        # Check for GGA or THS sentence
        if line.startswith('$GNGGA'):
            self.parse_gga(line)
        elif line.startswith('$GNTHS'):
            self.parse_ths(line)

        # If GPS position data is available, publish it
        if self.latitude is not None and self.longitude is not None and self.altitude is not None:
            fix_msg = NavSatFix()
            fix_msg.header.stamp = rospy.Time.now()
            fix_msg.header.frame_id = 'gps'
            fix_msg.latitude = self.latitude
            fix_msg.longitude = self.longitude
            fix_msg.altitude = self.altitude
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            self.fix_pub.publish(fix_msg)

            # Publish the UTM data as Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = 'earth'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position.x = self.latitude  # Replace with UTM conversion if needed
            odom_msg.pose.pose.position.y = self.longitude
            odom_msg.pose.pose.position.z = self.altitude

            # Set orientation from heading
            if self.heading is not None:
                quaternion = quaternion_from_euler(0, 0, math.radians(self.heading))
                odom_msg.pose.pose.orientation.x = quaternion[0]
                odom_msg.pose.pose.orientation.y = quaternion[1]
                odom_msg.pose.pose.orientation.z = quaternion[2]
                odom_msg.pose.pose.orientation.w = quaternion[3]

            self.utm_pub.publish(odom_msg)

        # Publish the heading if available
        if self.heading is not None:
            heading_msg = Float64()
            heading_msg.data = self.heading
            self.heading_pub.publish(heading_msg)
            rospy.loginfo(f"Published Heading: {self.heading}")

    def stop(self):
        self.um982serial.stop()
        self.pub_timer.shutdown()


def main():
    driver = UM982DriverROS1()
    
    # Handle shutdown signal to stop the driver gracefully
    rospy.on_shutdown(driver.stop)
    
    # Spin to keep the node running
    rospy.spin()

if __name__ == "__main__":
    main()
