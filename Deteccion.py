import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
lidar = 0
control = 0
mindist = 0
continua = 0
movil = 0
j = 0
# This code Take all the information of the sensors and publish variables in differents topics for the master code
# This function checks all the possible obstacles
def adelantamiento(lidar, control, msg):

  if np.min(msg[520:560]) < 1:
    lidar = 1
    print("obstaculo frontal")
  else:
    lidar = 0
  if np.min(msg[596:1079]) < 0.5:
    control = 1
  else:
    control = 0

  return lidar, control

# This function checks if we have a dynamic obstacle and if it is faster than our car
def obstaculo_movil(mindist, msg, j):

  if np.min(msg[530:550]) < 2 and mindist == 0:
    mindist = np.min(msg[530:550])
    print("Posible obstaculo ALERTA")
  elif np.min(msg[530:550]) > 1 and mindist != 0:
    mindist = 0
    j = 1
    print("OBSTACULO MOVIL A MAYOR VELOCIDAD, NO INTENTAR ADELANTAR")
  else:
    j = 0
  
  return j

# This function checks if we must change or not to the main road (We publish this information in a topic)
def verificar(msg):
  if np.min(msg[1:535]) < 0.8:
    continua = 1
  else:
    continua = 0
  return continua


class Master:

  def __init__(self):
    LIDAR_TOPIC = "/scan"
    DRIVE_TOPIC = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"
    

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
    # Subscribe to the laser scan data
    rospy.Subscriber(
        LIDAR_TOPIC,
        LaserScan,
        self.callback)
  
  def callback(self, msg):
    ##################################################
    # TODO >>>
    # Check all the possible obstacle and publish the information
    # 
    ##################################################
    
    Lectura = np.asarray(msg.ranges)
    Lectura[Lectura < 0.01] = 20
    
    print("distancia frontal", np.min(Lectura[530:550]))
    print("distancia lateral", np.min(Lectura[600:1079]))
    # rango = msg.ranges
    # print(len(rango))
    global lidar, control, continua

    [lidar, control] = adelantamiento(lidar, control, Lectura)
    movil = obstaculo_movil(mindist, Lectura, j)
    if movil == 1:
      lidar = 0
      print("OBSTACULO MOVIL A MAYOR VELOCIDAD, NO INTENTAR ADELANTAR")
    continua = verificar(Lectura)
    #rospy.loginfo(lidar)
    print("LidarC: ", lidar)
    print("LidarT: ", control)
    print("Continuar en carril: ", continua)  
    pub.publish(lidar)
    pub2.publish(control) 
    pub3.publish(continua)

    ##################################################
    # <<< TODO
    ##################################################

if __name__ == "__main__":
  # Here we indicate that we are going to publish in three differents topics
  pub = rospy.Publisher('obstacle', Float32, queue_size=10)
  pub2 = rospy.Publisher('adelantamiento', Float32, queue_size=10)
  pub3 = rospy.Publisher('verificacion', Float32, queue_size=10)
  # We initialize the node
  rospy.init_node('Master', anonymous=True)
  rate = rospy.Rate(10)
  mover = Master()
  rospy.spin()