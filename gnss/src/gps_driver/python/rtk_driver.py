import utm
import time
import serial
import rospy
from gps_driver.msg import Customrtk

def isGNGGAinString(inputString):
    if '$GNGGA' in inputString:
        print('GNGGA success!')
        return True
    else:
        print('GNGGA not found in string')
        return False

def degMinstoDegDec(LatOrLong):
    string = str(LatOrLong)
    if '.' in string:
        dotIndex = string.index('.')
        degreesLength = dotIndex - 2
        deg = float(string[:degreesLength])
        mins = float(string[degreesLength:])
        degDec = deg + (mins/60)
        return degDec
    else:
        return 0.0

def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if LatOrLongDir == "W" or LatOrLongDir == "S":
        LatOrLong = -1*LatOrLong
    return LatOrLong

def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
    return [UTMVals[0], UTMVals[1], UTMVals[2], UTMVals[3]]

def UTCtoUTCEpoch(UTC):
    local_time = time.localtime()
    TimeSinceEpoch = time.mktime((local_time.tm_year, local_time.tm_mon, local_time.tm_mday, 0, 0, 0, local_time.tm_wday, local_time.tm_yday, local_time.tm_isdst))
    hour = int(str(UTC)[0:2]) 
    minute = int(str(UTC)[2:4])
    sec = float(str(UTC)[4:])
    total_sec = hour*3600 + minute*60 + sec
    CurrentTime = TimeSinceEpoch + total_sec
    return [int(CurrentTime), int((CurrentTime - int(CurrentTime))*10**9)]

def ReadFromSerial(serialPortAddr):
    serialPort = serial.Serial(serialPortAddr)
    gnggaRead = serialPort.readline().decode('utf-8')
    serialPort.close()
    return gnggaRead

def main():
    pub = rospy.Publisher('gps_driver', Customrtk, queue_size=10)
    rospy.init_node('gps_driver', anonymous=True)
    port = rospy.get_param('~port', 'default_port_name')
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        hello = converter(port)
        rospy.loginfo(hello)
        pub.publish(hello)
        rate.sleep()

def converter(port):
    while True:
        serialPortAddr = port
        gnggaRead = ReadFromSerial(serialPortAddr)
        
        if isGNGGAinString(gnggaRead):
            gnggaSplit = [x.strip() for x in gnggaRead.split(',')]

            UTC = float(gnggaSplit[1])
            Latitude = float(gnggaSplit[2])
            LatitudeDir = gnggaSplit[3]
            Longitude = float(gnggaSplit[4])
            LongitudeDir = gnggaSplit[5]
            FixQuality = int(gnggaSplit[6])
            HDOP = float(gnggaSplit[8])
            Altitude = float(gnggaSplit[9])

            Latitude = degMinstoDegDec(Latitude)
            Longitude = degMinstoDegDec(Longitude)

            LatitudeSigned = LatLongSignConvetion(Latitude, LatitudeDir)
            LongitudeSigned = LatLongSignConvetion(Longitude, LongitudeDir)
            
            UTM = convertToUTM(LatitudeSigned, LongitudeSigned)

            CurrentTime = UTCtoUTCEpoch(UTC)
            
            msg = Customrtk()
            msg.header.frame_id = "GNGGA"
            msg.header.stamp.secs = CurrentTime[0]
            msg.header.stamp.nsecs = CurrentTime[1]
            msg.latitude = Latitude
            msg.longitude = Longitude
            msg.altitude = Altitude
            msg.utm_easting = UTM[0]
            msg.utm_northing = UTM[1]
            msg.zone = UTM[2]
            msg.letter = UTM[3]
            msg.hdop = HDOP
            msg.fix_quality = FixQuality
            msg.gngga_read = gnggaRead
            
            return msg
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
