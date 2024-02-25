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
    deg = int(LatOrLong//100)
    mins = LatOrLong%100
    degDec = deg+(mins/60)
    return (degDec)

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
    utc_time_struct = time.gmtime()
    
    hour = int(str(UTC)[0:2])
    minute = int(str(UTC)[2:4])
    sec = float(str(UTC)[4:])
    gpgga_seconds_since_midnight = hour * 3600 + minute * 60 + sec
    
    start_of_day_utc = time.mktime((utc_time_struct.tm_year, utc_time_struct.tm_mon, utc_time_struct.tm_mday, 0, 0, 0, 0, 0, 0))
    
    timestamp = start_of_day_utc + gpgga_seconds_since_midnight
    
    CurrentTimeSec = int(timestamp)
    CurrentTimeNsec = int((timestamp - CurrentTimeSec) * 10**9)
    
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
        gps = converter(port)
        pub.publish(gps)
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
            msg.latitude = LatitudeSigned
            msg.longitude = LongitudeSigned
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

