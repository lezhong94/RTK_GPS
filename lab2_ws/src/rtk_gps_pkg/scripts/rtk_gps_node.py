#!/usr/bin/env python  
import rospy
from sensor_msgs.msg import NavSatFix # Import sensor_msg (ROS build-in msg class)
import numpy as np
import serial   # Import serial module


class SerialReader(object):
    def __init__(self, portName, baudRate):
        self.port = serial.Serial(portName, baudRate, timeout=1)
        pub = rospy.Publisher('rtk_fix', NavSatFix, queue_size=10)
        rospy.init_node('rtk_gps_node', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            line = self.port.readline()
            vals = np.array(line.split(','))
                
            if vals[0] == '$GNGGA':
                vals[vals=='']='0'
                    
                    # self.timestamp = flost(vals[1])
                self.raw_lat = float(vals[2])
                self.lat_indicator = vals[3]
                self.raw_lon = float(vals[4])
                self.lon_indicator = vals[5]
                self.alt = float(vals[9])
                self.alt_unit = vals[10]
                self.gpsdata = NavSatFix()

                if self.lat_indicator == 'N':
                    lat = (int((float(self.raw_lat)/100))+((float(self.raw_lat)%100)/60))
                elif self.lat_indicator == 'S':
                    lat = -1*(int((float(self.raw_lat)/100))+((float(self.raw_lat)%100)/60))
                if self.lon_indicator == 'E':
                    lon = (int((float(self.raw_lon)/100))+((float(self.raw_lon)%100)/60))
                elif self.lon_indicator == 'W':
                    lon = -1*(int((float(self.raw_lon)/100))+((float(self.raw_lon)%100)/60))
                alt = self.alt

                self.gpsdata = NavSatFix(latitude = lat, longitude = lon, altitude = alt)
                    
                    # publish msg to topic 'rtk_fix'
                # pub = rospy.Publisher('rtk_fix', NavSatFix, queue_size=10)
                # rospy.init_node('rtk_gps_node', anonymous=True)
                # rate = rospy.Rate(10)
                

                rospy.loginfo(self.gpsdata)
                pub.publish(self.gpsdata)
                rate.sleep()
        

if __name__ == '__main__':
    # Read data from serial port
    rospy.init_node('rtk_gps_node', anonymous=True)
    portName = rospy.get_param('~port_name', 'default_value')
    baudRate = rospy.get_param('~baud_rate', 'default_value')
    
    SerialReader(portName, baudRate)
    
pass