#!/usr/bin/env python2
import numpy as np
import rospy
import re 
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
i = 1
j = 0
camino = camino1 = camino2 = 0
len1 = len2 = 0
cont = conty = 0
dist = 0
roll = pitch = yaw = 0.0
lidarC = lidarT = alt = volver = parada = giro = curva = 0
#This is the master code. Is the only one that can move the car according to the information received from the topics
# This function calculates the Pure Pursuit movement 
def PurePursuit(x_def, y_def, distancia, phi_error, self):
    "calculate de steering angle for the car"
    global cont, conty, len1, len2, j, parada, giro
    if distancia > 0.3 and j == 0:
      i = 0
      px = math.cos(phi_error)*x_def + math.sin(phi_error)*y_def
      py = -math.sin(phi_error)*x_def + math.cos(phi_error)*y_def
      giro = 1.7*math.atan2(py,px)
      #print("giro necesario: ", giro)
      # The car will not move in case we have an emergency stop
      if parada == 0:
        self.drive_msg.drive.speed = 0.7
        self.drive_msg.drive.steering_angle = giro          
      else:
        self.drive_msg.drive.speed = 0          
    else:
      # print("punto alcanzado")
      # print("siguiente punto")
      i = 1
      len = len1 - 1
      if conty == len:
        cont = 0
        conty = 0
      else:
        cont += 2
      j = 0
    return i  

# Here we get the points for the road of the txt files
def extraer_punto(filename):
    with open(filename, 'r') as f:
      Puntos = f.read()
    Puntos_1 = [float(s) for s in re.findall(r'-?\d+\.?\d*', Puntos)]
    return Puntos_1

# Here we decide if we must continue or not on the main road
def adelantar(lidarC):
    global camino, camino2, camino1, j, alt
    if camino1 == 1:
        if lidarC == 1:
          # print("Tomando ruta alternativa")
          j = 1
          camino = Puntos_alt
          camino2 = 1
          camino1 = 0
          alt = 0

# This function decides if we must do an emergency stop 
def parada_emergencia(alt, msg):
    global lidarC, lidarT, giro, parada, camino2, cont
    #print("Punto alternativo: ", alt)
    if giro > 0.1 and alt > 1 and alt < 5 and camino2 == 1:
      if np.min(msg[540:1079]) < 0.3:
        parada = 1
        print("PARADA DE EMERGENCIA")
      else:
        parada = 0
    elif giro < 0.1 and alt > 1 and alt < 5 and camino2 == 1:
      if np.min(msg[400:1079]) < 0.5:
        parada = 1
        print("PARADA DE EMERGENCIA")
      else:
        parada = 0
    elif camino2 == 1 and np.min(msg[515:575]) < 0.3 and alt >= 1:
      parada = 1    
    elif alt == 1 and np.min(msg[545:1079]) < 0.3 and giro > 0.1 and giro < 0.7:
      parada = 1
    elif camino2 == 0 and np.min (msg[1:545]) < 0.3 and cont == 0 and giro > 0.1 and giro < 0.7:
      parada = 1
    else:
      parada = 0

class Safety:


  def __init__(self):
    DRIVE_TOPIC = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"
    ODOMETRY_TOPIC = "/vesc/odom"
    LIDAR_TOPIC = "/scan"
    

    ##################################################
    # TODO >>>
    self.drive_msg = AckermannDriveStamped()


    ##################################################

    ##################################################
    # <<< TODO
    ##################################################

    # Initialize a publisher for drive messages
    self.drive_pub = rospy.Publisher(
        DRIVE_TOPIC,
        AckermannDriveStamped,
        queue_size=1)

    # Subscribe to the Odometry data
    rospy.Subscriber(
        ODOMETRY_TOPIC,
        Odometry,
        self.position)
    # Subcribe to obstacle topic
    rospy.Subscriber("obstacle", 
        Float32, 
        self.obstacle)

    # Subcribe to adelantamiento topic
    rospy.Subscriber("adelantamiento", 
        Float32, 
        self.adelantamiento)
    
    # Subcribe to verificacion topic
    rospy.Subscriber("verificacion", 
        Float32, 
        self.verificacion)
    # Subscribe to the laser scan data 
    rospy.Subscriber(
        LIDAR_TOPIC,
        LaserScan,
        self.callback)

  def callback(self, msg):
    ##################################################
    # TODO >>>
    # Check the LIDAR for the emergency stop
    # 
    ##################################################
    global giro, lidarC

    Lectura = np.asarray(msg.ranges)
    Lectura[Lectura < 0.01] = 20

    if giro > 0.1:
      curva = 1
    else:
      curva = 0
    if np.min(Lectura[400:1079]) < 1 and curva == 1:
      lidarC = 1
      adelantar(lidarC)
    parada_emergencia(alt, Lectura)
    print("Parada: ", parada)
    ##################################################
    # <<< TODO
    ##################################################

  def position(self, msg):
    ##################################################
    # TODO >>>
    # Take Robot position from Odometry topic
    # Take target position from txt
    # Make the robot move to target position
    ##################################################
    global i, Target_x, Target_y, cont, yaw, roll, pitch, conty, camino, camino2, camino1, alt, dist
    
    Pose_car_x = msg.pose.pose.position.x
    Pose_car_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    #print("Posicion coche en x: ", Pose_car_x)
    #print("Posicion coche en y: ", Pose_car_y)
    #print("Orientacion del coche en z: ", yaw)
    
    if i == 1:
      Target_x = camino[cont] 
      conty = cont + 1
      Target_y = camino[conty]
      if camino2 == 1:
        alt += 1  
    # print("Punto objetivo en x: ", Target_x)
    # print("Punto objetivo en y: ", Target_y)
    x_def = Target_x - Pose_car_x  
    y_def = Target_y - Pose_car_y
    # Minimun distance to the target point
    dist = math.sqrt(pow(x_def,2) + pow(y_def,2))
    # print("distancia al punto objetivo: ", dist)

    i = PurePursuit(x_def, y_def, dist, yaw, self)
    self.drive_pub.publish(self.drive_msg)



    ##################################################
    # <<< TODO
    ##################################################

  def obstacle(self, data):
    ##################################################
    # TODO >>>
    # This take de information form the Obstacle topic 
    # And then let the function adelantar decides what to do
    # 
    ##################################################
    global camino, lidarC, j, camino2, alt, curva
    if curva == 0:
      lidarC = data.data
    #print(camino)
    adelantar(lidarC)
      

    ##################################################
    # <<< TODO
    ##################################################
  
  def adelantamiento(self, info):
    ##################################################
    # TODO >>>
    # This thakes the information from the adelantamiento topic
    # And then calculate if we can get back to the main road
    #
    ##################################################
    global camino, lidarC, lidarT, j, camino1, camino2, alt, volver, conty
    lidarT = info.data
    #print(camino)
    if camino2 == 1:
      if conty < 20:
        if lidarC == 0 and lidarT == 0 and alt == 2 and volver == 0:
           # print("Desvio al carril principal")
           camino = Puntos_obj
           j = 1
           alt = 0
           volver = 0
           camino2 = 0
           camino1 = 1
      else:
        if lidarC == 0 and lidarT == 0 and alt == 4 and volver == 0:
           # print("Desvio al carril principal")
           camino = Puntos_obj
           j = 1
           alt = 0
           volver = 0
           camino2 = 0
           camino1 = 1

      

    ##################################################
    # <<< TODO
    ##################################################
  
  def verificacion(self, info):
    ##################################################
    # TODO >>>
    # This takes the information from the verificacion topic
    # And then change the value of volver variable
    # With this, the adelantamiento function can decide if we get back to the main road or not
    ##################################################
    global camino2, alt, volver, lidarC
    # print("verificacion: ", info.data)
    if alt >= 2 and camino2 == 1:
      if info.data == 1:
        volver = 1
        alt = 0
      else:
        volver = 0

      

    ##################################################
    # <<< TODO
    ##################################################

if __name__ == "__main__":
  #initialize the node
  rospy.init_node("Safety")
  #Take the target points of the main road
  Puntos_obj = extraer_punto('TargetPoints.txt')
  len1 = len(Puntos_obj)
  camino = Puntos_obj
  camino1 = 1
  #Take the target points of the alternative road
  Puntos_alt = extraer_punto('AlternativeRoad.txt')
  len2 = len(Puntos_alt)
  print(Puntos_obj)
  mover = Safety()
  rospy.spin()
