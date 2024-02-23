import utm
import time
import serial
import rospy
from gps_driver.msg import Customgps

def isGPGGAinString(inputString):
    if '$GPGGA' in inputString:
        print('Great success!')
        return True
    else:
        print('GPGGA not found in string')
        return False
        
def degMinstoDegDec(LatOrLong):
    deg = float(str(LatOrLong)[:-7])
    mins = str(LatOrLong)[-7:]
    degDec = float(mins)/60
    return (deg+degDec)

def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if LatOrLongDir == "W" or LatOrLongDir == "S":
        LatOrLong = -1*LatOrLong
    return LatOrLong

def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
    UTMEasting = UTMVals[0]
    UTMNorthing = UTMVals[1]
    UTMZone = UTMVals[2]
    UTMLetter = UTMVals[3]
    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

def UTCtoUTCEpoch(UTC):
    local_time = time.localtime()
    TimeSinceEpoch = time.mktime((local_time.tm_year, local_time.tm_mon, local_time.tm_mday, 0, 0, 0, local_time.tm_wday, local_time.tm_yday, local_time.tm_isdst)) #Replace with a 1-line method to get time since epoch in UTC
    hour = int(str(UTC)[0:2]) 
    minute = int(str(UTC)[2:4])
    sec = float(str(UTC)[4:])
    total_sec = hour*3600 + minute*60 + sec
    CurrentTime = TimeSinceEpoch + total_sec
    CurrentTimeSec = int(CurrentTime)
    CurrentTimeNsec = int((CurrentTime - CurrentTimeSec)*10**9)
    return [CurrentTimeSec, CurrentTimeNsec]

def ReadFromSerial(serialPortAddr):
    serialPort = serial.Serial(serialPortAddr)
    gpggaRead = serialPort.readline().decode('utf-8')
    serialPort.close()
    return gpggaRead

def main():
    rospy.init_node('gps_driver', anonymous=True)
    pub = rospy.Publisher('gps', Customgps, queue_size=10)
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
        gpggaRead = ReadFromSerial(serialPortAddr)
        
        if isGPGGAinString(gpggaRead):
            gpggaSplit = [x.strip() for x in gpggaRead.split(',')]

            UTC = float(gpggaSplit[1])
            Latitude = float(gpggaSplit[2])
            LatitudeDir = gpggaSplit[3]
            Longitude = float(gpggaSplit[4])
            LongitudeDir = gpggaSplit[5]
            HDOP = float(gpggaSplit[8])

            Latitude = degMinstoDegDec(Latitude)
            Longitude = degMinstoDegDec(Longitude)

            LatitudeSigned = LatLongSignConvetion(Latitude, LatitudeDir)
            LongitudeSigned = LatLongSignConvetion(Longitude, LongitudeDir)

            UTM = convertToUTM(LatitudeSigned, LongitudeSigned)

            CurrentTime = UTCtoUTCEpoch(UTC)
            
            msg = Customgps()
            msg.header.frame_id = "GPGGA"
            msg.header.stamp.secs = CurrentTime[0]
            msg.header.stamp.nsecs = CurrentTime[1]
            msg.latitude = Latitude
            msg.longitude = Longitude
            msg.altitude = 0
            msg.utm_easting = UTM[0]
            msg.utm_northing = UTM[1]
            msg.zone = UTM[2]
            msg.letter = UTM[3]
            msg.hdop = HDOP
            msg.gpgga_read = gpggaRead
            
            return msg

        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

