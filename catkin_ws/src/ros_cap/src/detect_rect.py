#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy
from geometry_msgs.msg import Point

#Pato 31mm

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber('/duckiebot/camera_node/image/rect', Image, self.callback)
		self.pub = rospy.Publisher('/duckiebot/camera_node/imagen_normal', Image, queue_size=0)
		self.pub_punto = rospy.Publisher('/duckiebot/camera_node/punto', Point, queue_size=0)
		self.bridge = CvBridge()
		self.point = Point()

	def callback(self,msg):
		image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		self.procesar_img(image)

	def procesar_img(self, img):
		#Cabiar espacio de color
		img_out = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		# Filtrar rango util
		mask = cv2.inRange(img_out, np.array([20,110,140]), np.array([50,255,255]))

		# Aplicar mascara
		img_out = cv2.bitwise_and(img_out, img_out, mask=mask)

		# Definir blobs			
		_ , contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		# Aplicar transformaciones morfologicas
		kernel = np.ones((5,5), np.uint8)
		img_out = cv2.erode(img_out, kernel, iterations = 1)
		img_out = cv2.dilate(img_out, kernel, iterations = 1)

		# Dibujar rectangulos de cada blob
		self.point.x = 0
		self.point.y = 0
		self.point.z = 0
		for cnt in contours:
			x,y,w,h = cv2.boundingRect(cnt)
			if w*h >= 900:
				cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
				disth = 165*31/h
				distw = 163*35/w
				xc = (x+w/2 - 165) * 31/h 
				yc = (y+h/2 - 125) * 31/h
				zc = disth
				self.point.x = xc
				self.point.y = yc
				self.point.z = zc
				print [xc, yc, zc]
		self.pub_punto.publish(self.point)
		

		#x=163.9807777549233, y=165.964452557702
		#centro 156.7907618046331, 125.71306459330566

		# Publicar imagen final
		msg = self.bridge.cv2_to_imgmsg(img, "bgr8")

		self.pub.publish(msg)

def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
