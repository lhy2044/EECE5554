import serial
import re
import rospy
from vn_driver.msg import Vectornav

# Open serial port
ser = serial.Serial('/dev/pts/3', 115200)

def parse_vnymr(vnymr_string):
    """
    Parse the $VNYMR string to extract IMU data.
    """
    pattern = r"\$VNYMR,([-+]?[0-9]*\.?[0-9]+),([-+]?[0-9]*\.?[0-9]+),([-+]?[0-9]*\.?[0-9]+)," \
              r"([-+]?[0-9]*\.?[0-9]+),([-+]?[0-9]*\.?[0-9]+),([-+]?[0-9]*\.?[0-9]+)," \
              r"([-+]?[0-9]*\.?[0-9]+),([-+]?[0-9]*\.?[0-9]+),([-+]?[0-9]*\.?[0-9]+)," \
              r"([-+]?[0-9]*\.?[0-9]+),([-+]?[0-9]*\.?[0-9]+),([-+]?[0-9]*\.?[0-9]+)\*.*"
    match = re.match(pattern, vnymr_string)
    if match:
        return [float(x) for x in match.groups()]
    else:
        return None

def main():
    # Read and parse data
    rospy.init_node('vn_driver', anonymous=True)
    pub = rospy.Publisher('vn', Vectornav, queue_size=10)
    port = rospy.get_param('~port', 'default_port_name')
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        while True:
            line = ser.readline().decode('ascii').strip()
            if line.startswith('$VNYMR'):
                    rospy.init_node('vn_driver', anonymous=True)
                    msg = Vectornav()
                    msg.header.frame_id = "VNYMR"
                    msg.header.stamp = rospy.Time.now()
                    msg.imu = line
                    print(msg)
                
if __name__ == "__main__":
    main()
