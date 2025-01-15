# **UM982 ROS Driver**

This ROS driver reads data from a UM982 GPS device via a serial port and publishes it as ROS messages. It parses NMEA sentences (GGA and THS) to extract GPS position and heading information.

---

## **Features**
- Reads GPS position (latitude, longitude, altitude) from `$GNGGA` sentences.
- Reads heading from `$GNTHS` sentences.
- Publishes data as:
  - `NavSatFix` messages (`/gps/fix` topic).
  - `Odometry` messages (`/gps/utmpos` topic).
  - `Float64` messages (`/gps/heading` topic).
- Configurable serial port and baud rate.

---

## **Setup Instructions**

### **1. Clone the Repository**
Clone this repository into your ROS workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/JorandG/UM982ROS
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### **2. Add the Python Code**
Ensure the provided Python script (`um982ros.py`) is located in the correct directory within your ROS workspace. For example:
```bash
~/catkin_ws/src/UM982ROS/um982ros.py
```
Make the script executable:
```bash
chmod +x ~/catkin_ws/src/UM982ROS/um982ros.py
```

### **3. Configure Parameters**
The driver parameters can be set via the command line:
- **Port**: Serial port to which the UM982 is connected (default: `/dev/ttyUM982`).
- **Baud Rate**: Baud rate for the serial connection (default: `115200`).

Run the driver directly:
```bash
rosrun um982_driver um982_driver.py _port:=/dev/ttyUSB0 _baud:=115200
```

### **4. Serial Port Permissions**
Ensure your user has permission to access the serial port:
```bash
sudo usermod -a -G dialout $USER
```
Log out and back in for the changes to take effect.

---

## **Topics**

### **Published Topics**
- `/gps/fix` (`sensor_msgs/NavSatFix`):
  GPS position (latitude, longitude, altitude).
- `/gps/utmpos` (`nav_msgs/Odometry`):
  UTM coordinates with heading.
- `/gps/heading` (`std_msgs/Float64`):
  Heading angle (in degrees).

---

## **How It Works**

### **1. NMEA Sentence Parsing**
- **`$GNGGA`**:
  - Extracts latitude, longitude, and altitude.
  - Converts coordinates to decimal degrees.
- **`$GNTHS`**:
  - Extracts heading angle.

### **2. ROS Message Publishing**
- Position and heading data are converted to ROS message formats and published on their respective topics.

### **3. Serial Communication**
- Reads data from the UM982 device via the serial port.

---

## **Using RTK with RTKLIB**

To achieve Real-Time Kinematic (RTK) corrections for improved GPS accuracy, use the `str2str` tool from the RTKLIB suite. Follow these steps:

### **1. Install RTKLIB**
Install RTKLIB on your Linux system:
```bash
sudo apt install rtklib
```

### **2. RTKLIB Command Explanation**
Use the following `str2str` command to set up RTK corrections:
```bash
str2str -in ntrip://<username>:<password>@<ntrip_server>:<port>/<mountpoint> -out serial://<serial_port>:<baud_rate>:8:n:1
```

#### **Command Parameters**
- **`<username>`**: Your NTRIP service username.
- **`<password>`**: Your NTRIP service password.
- **`<ntrip_server>`**: The URL of the NTRIP caster (e.g., `www.euref-ip.net`).
- **`<port>`**: The port used by the NTRIP caster (e.g., `2101`).
- **`<mountpoint>`**: The specific data stream mount point (e.g., `M0SE00ITA0`).
- **`<serial_port>`**: The serial port connected to the UM982 (e.g., `/dev/ttyUSB0`).
- **`<baud_rate>`**: The baud rate of the serial connection (e.g., `115200`).

### **3. Example Command**
Replace placeholders with actual values, except credentials:
```bash
str2str -in ntrip://username:password@www.euref-ip.net:2101/M0SE00ITA0 -out serial://ttyUSB0:115200:8:n:1
```

### **4. Notes**
- Ensure the serial port name matches your device (e.g., `/dev/ttyUSB0`).
- Verify the mount point and network credentials with your NTRIP provider.

---

## **Troubleshooting**

### **1. No Data Published**
- Verify the serial port and baud rate:
  ```bash
  ls /dev/tty*
  ```
- Check permissions for the serial port.

### **2. GPS Not Locked**
- The GPS may take some time to lock onto satellite data, especially after a reset or in poor signal conditions.

### **3. Incorrect Data**
- Verify NMEA sentences output by the UM982.
- Check for potential interference or hardware issues.
