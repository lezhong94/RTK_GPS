#!/usr/bin/env python  
import roslib
import rospy
import utm
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

global pub
global rate

def callback(data):
    global lat
    global log
    global alt 

    # rospy.loginfo('%f,%f,%f\n',data.latitude,data.longitude,data.altitude)
    
    lat = data.latitude
    log = data.longitude 
    alt = data.altitude
    utm_point = Point()

    # utm converter
    utmconvert  = utm.from_latlon(lat, log)
    utm_easting = float(utmconvert[0])
    utm_northing = float(utmconvert[1])

    utm_point = Point(x = utm_easting, y = utm_northing, z = alt)

    rospy.loginfo(utm_point)
    pub.publish(utm_point)
    

def utmconverter():
    rospy.init_node('gps2utm_node', anonymous=True) 
    rospy.Subscriber('rtk_fix', NavSatFix, callback)
    rate = rospy.Rate(10)
    rospy.spin()
    
    
if __name__ == "__main__":
    try:
        pub = rospy.Publisher('utm_fix', Point, queue_size=10)
        utmconverter() 
        
    except rospy.ROSInterruptException:
        pass