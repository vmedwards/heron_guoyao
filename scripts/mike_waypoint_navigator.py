#!/usr/bin/env python
import rospy
import math
import matplotlib.pyplot as plt
import sys
import time

from geometry_msgs.msg import Vector3Stamped
from heron_msgs.msg import Helm
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

pos_cur=NavSatFix()

yaw_cur = 0.00
yaw_des_old = 0.0

latitude = 39.94339439909326
longitude = -75.19981109992537

square_leg_length = 0.00015

course = ([latitude, longitude],
                  [latitude + square_leg_length, longitude],
                  [latitude + square_leg_length, longitude + square_leg_length],
                  [latitude, longitude + square_leg_length],
                  [latitude, longitude])


i = 0
kp=2
wypt_dist_thresh = 2.0  # distance threshold to stop or switch to next way point
cmd_dt = 0.1            # time interval for cmd_helm publishing
base_thrust = 1.0       # the base thrust level at which the Heron will run

# conversions from lon/lat to meters
met_lat = 111033.4717 
met_lon = 85467.2528

def lawn_mower(course, num_flats = 4, flag="v"):
    if len(course) < 4:
        print("Need at least 4 points to genereate lawn mower pattern")
        return

    if num_flats <= 0:
        print("Invalid number of flats")
        return
    
    new_course = []

    p1 = course[0]
    p2 = course[1]
    p4 = course[3]

    height = (p4[1] - p1[1])
    width = (p2[0] - p1[0])
    
    print("Height of {} and Width of {}".format(height, width))

    # This is the equation to find how many points you need based on the given amount of flats
    points = 2 + (2 * num_flats)
                
    if flag == "v":
        # Vertical lawn Mower Pattern
        
        # The "flat_len" is the length of the short segment
        flat_len = width / (num_flats)

        # These coefficeints help us mimic the hard coded values 
        lat_coefficent = 0
        lon_coefficent = 0

        
        for i in range(points):
           
            # Calculate waypoint for current iteration

            lat = latitude + (lat_coefficent * flat_len)
            lon = longitude + (lon_coefficent * height)
            
            new_course.append([lat, lon])
            
            # Update Coefficients   
            
            # After every odd index, the coefficent in front of flat_len should increase by 1    
            if i % 2 == 1:
                lat_coefficent += 1
            
            # After every even index, the lon_coefficient should alternate between 0 and 1 (so we alternate between including the height and not) 
            if i % 2 == 0:
                lon_coefficent = (lon_coefficent + 1) % 2
    elif flag == "h":
        
        # Horizontal Lawn Mower Paterrn (Similar to vertical)
        
        flat_len = height / (num_flats)

        lat_coefficent = 0
        lon_coefficent = 0

        for i in range(points):
           
            # Calculate waypoint for current iteration

            lat = latitude + (lat_coefficent * width)
            lon = longitude + (lon_coefficent * flat_len)
            
            new_course.append([lat, lon])
            
            # Update Coefficients   
            if i % 2 == 1:
                lon_coefficent += 1
            
            if i % 2 == 0:
                lat_coefficent = (lat_coefficent + 1) % 2
    else:
        print("Invalid Flag")
        return new_course
        
    # Plot the course 
    x_cords = []
    y_cords = []
    for point in (course):
        x_cords.append(point[0] * met_lat)
        y_cords.append(point[1] * met_lon)
        
        plt.plot(x_cords, y_cords, color='red', linestyle='dashed', linewidth = 3)
        
    x_cords = []
    y_cords = []
        
    for point in (new_course):
        x_cords.append(point[0] * met_lat)
        y_cords.append(point[1] * met_lon)
            
    plt.plot(x_cords, y_cords, color='green', linestyle='solid', linewidth = 3,
             marker='o', markerfacecolor='blue', markersize=12)
        
    plt.title("Lawn Mower Pattern")
    plt.show()
        
    return new_course

course_desired = lawn_mower(course, 4)
                

def nav_comp(navsat_msg):
    global pos_cur
    pos_cur = navsat_msg
    return

def yaw_callback(data_msg):
    global yaw_cur
    yaw_cur = data_msg.vector.z
    if(yaw_cur < 0):
    	yaw_cur = yaw_cur + 2 * math.pi
    return


def control_publisher(event):
    global yaw_cur, i, kp, pos_cur, yaw_des_old, course_desired, wypt_dist_thresh, cmd_dt, base_thrust, met_lat, met_lon
    pub_msg = Helm()
    helm_pub = rospy.Publisher('/cmd_helm', Helm, queue_size=100)
    lat_pub = rospy.Publisher('/waypoint_lat', Float32, queue_size=100)
    long_pub = rospy.Publisher('/waypoint_long', Float32, queue_size=100)

    pos_des_lat = course_desired[i][0]
    pos_des_lon = course_desired[i][1]

    delta_lat = (pos_des_lat-pos_cur.latitude)*met_lat
    delta_lon = (pos_des_lon - pos_cur.longitude)*met_lon
    dist_err = math.sqrt( delta_lat**2 + delta_lon**2 )
    print ("dist error", dist_err)
    print (i)
    lat_pub.publish(float(course_desired[i][0]))
    long_pub.publish(float(course_desired[i][1]))
    if (dist_err < wypt_dist_thresh) :
        # go to next way point if within distance threshold
        if( i < len(course_desired)-1 ):
            i = i+1
            pos_des_lat = course_desired[i][0]
            pos_des_lon = course_desired[i][1]
            delta_lat = (pos_des_lat-pos_cur.latitude)*met_lat
            delta_lon = (pos_des_lon - pos_cur.longitude)*met_lon
            dist_err = math.sqrt( delta_lat**2 + delta_lon**2 )

        # stop if at last waypoint
        else:
            pub_msg.thrust = 0.0
            pub_msg.yaw_rate = 0.0
            helm_pub.publish(pub_msg)
            return
    
    yaw_des = math.atan2( delta_lat, delta_lon)
    if( yaw_des < 0):
        yaw_des = 2*math.pi + yaw_des

    # compute rate of change of yaw_des
    yaw_des_rate = (yaw_des-yaw_des_old)/cmd_dt;
    yaw_des_old = yaw_des;

    # compute yaw_error and change it to be within [-pi pi]
    yaw_error = yaw_des - yaw_cur
    if ( yaw_error > math.pi ):
        yaw_error = yaw_error-2*math.pi
    elif ( yaw_error < -math.pi ):
        yaw_error = 2*math.pi + yaw_error

    # controller for yaw_rate
    #pub_msg.yaw_rate = kp*yaw_error + yaw_des_rate
    pub_msg.yaw_rate = kp*yaw_error
    
    # if far away, thrusht is at base thrust level
    if( dist_err > 10 ):
        pub_msg.thrust = base_thrust    
    # gradually slow down as wel approach the waypoint
    else:
        pub_msg.thrust = base_thrust*dist_err/10.0

    # adjust thrust according to yaw_error, i.e., adjust yaw first before thrusting forwayd
    pub_msg.thrust = pub_msg.thrust*math.exp(-10*math.fabs(yaw_error) )
    
    # publish message on topic
    helm_pub.publish(pub_msg)
    
    # print("yaw desired = ", yaw_des)
    # print(" yaw current = ", yaw_cur)
    # print("yaw error= ", yaw_error)
    # print("yaw rate desired= ", pub_msg.yaw_rate)
    return
    
def course_publisher():
    global cmd_dt

    rospy.init_node('wapt_control_publisher', anonymous=True)
    rospy.Subscriber("/navsat/fix", NavSatFix,nav_comp)
    
    # /filtered_yaw is the topic on which the data from the complementary filter will arrive
    rospy.Subscriber("/imu/rpy", Vector3Stamped, yaw_callback)
    
    # publishes data on cmd_helm every cmd_dt seconds
    rospy.Timer(rospy.Duration(cmd_dt), control_publisher)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    course_publisher()
