#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy
from geometry_msgs.msg import Point
from duckietown_msgs.msg import Twist2DStamped

#Pato 31mm

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.subpunto = rospy.Subscriber('/duckiebot/camera_node/punto', Point, self.seguridad)
		self.submove = rospy.Subscriber('/duckiebot/possible_cmd', Twist2DStamped, self.callback)
		self.pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=0)
		self.cero = Twist2DStamped()
		self.a = True
	def callback(self, msg):
		if self.a == False:
			msg.v = 0
			msg.omega = 0
			print 'callback'
		self.pub.publish(msg)	
	def seguridad(self, punto):
		if punto.z >= 800 or punto.z == 0:
			self.a = True
			print 'seguridad True'
		else: 
			self.a = False
			print 'seguridad False'

def main():
	rospy.init_node('test2') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
