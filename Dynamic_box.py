#!/usr/bin/env python2

import numpy as np
import rospy
import re 
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
i = 1
j = 0
camino = 0
len1 = 0
cont = 0
conty = 0
dist = 0
roll = pitch = yaw = 0.0

#This code manage the movement of the dynamic obstacle that simulates a car on the main road
# The Pure pursuit code is the same than our vehicule's
def PurePursuit(x_def, y_def, distancia, phi_error, self):
    "calculate de steering angle for the car"
    global cont, conty, len1, j
    if distancia > 0.3 and j == 0:
      i = 0
      px = math.cos(phi_error)*x_def + math.sin(phi_error)*y_def
      py = -math.sin(phi_error)*x_def + math.cos(phi_error)*y_def
      giro = 1.5*math.atan2(py,px)
      print("giro necesario: ", giro)
      self.drive_msg.linear.x = 0.3
      self.drive_msg.angular.z = giro
      
    else:
      print("punto alcanzado")
      print("siguiente punto")
      i = 1
      len = len1 - 1
      if conty == len:
        cont = 0
        conty = 0
      else:
        cont += 2
      j = 0
    return i  
# This function takes the target points form the txt
def extraer_punto(filename):
    with open(filename, 'r') as f:
      Puntos = f.read()
    Puntos_1 = [float(s) for s in re.findall(r'-?\d+\.?\d*', Puntos)]
    return Puntos_1

class Dynamic_box:


  def __init__(self):
    DRIVE_TOPIC = "/dynamic_box/cmd_vel"
    ODOMETRY_TOPIC = "/dynamic_box/odom"
    

    ##################################################
    # TODO >>>
    self.drive_msg = Twist()


    ##################################################

    ##################################################
    # <<< TODO
    ##################################################

    # Initialize a publisher for drive messages
    self.drive_pub = rospy.Publisher(
        DRIVE_TOPIC,
        Twist,
        queue_size=1)

    # Subscribe to the Odometry data
    rospy.Subscriber(
        ODOMETRY_TOPIC,
        Odometry,
        self.position)
    


  
  def position(self, msg):
    ##################################################
    # TODO >>>
    # Take car position from Odometry topic
    # Take target position from txt
    # Make the car move to target position
    ##################################################
    global i, Target_x, Target_y, cont, yaw, roll, pitch, conty, camino

    Pose_car_x = msg.pose.pose.position.x
    Pose_car_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    
    if i == 1:

      Target_x = camino[cont] 
      conty = cont + 1
      Target_y = camino[conty] 

    print("Punto objetivo en x: ", Target_x)
    print("Punto objetivo en y: ", Target_y)
    x_def = Target_x - Pose_car_x  
    y_def = Target_y - Pose_car_y

    dist = math.sqrt(pow(x_def,2) + pow(y_def,2))
    print("distancia al punto objetivo: ", dist)

    i = PurePursuit(x_def, y_def, dist, yaw, self)
    self.drive_pub.publish(self.drive_msg)



    ##################################################
    # <<< TODO
    ##################################################


if __name__ == "__main__":
  # First, we initialize the node
  rospy.init_node("Dynamic_box")
  # In this case, we only take the main road target points
  Puntos_obj = extraer_punto('TargetPoints.txt')
  len1 = len(Puntos_obj)
  camino = Puntos_obj
  mover = Dynamic_box()
  rospy.spin()
