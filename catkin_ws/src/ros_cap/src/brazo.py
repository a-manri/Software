#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32, Float32, UInt16MultiArray # importar mensajes de ROS tipo String, Int32 y UInt16MultiArray
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from duckietown_msgs.msg import Twist2DStamped

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber('/duckiebot/joy', Joy, self.callback)
		self.pub = rospy.Publisher('/servo', UInt16MultiArray, queue_size=0)			
		self.uint = UInt16MultiArray()
	
	def callback(self,msg):
		back = msg.buttons[6]; #posicion original
		y = msg.buttons[3]; #garra soltar
		x = msg.buttons[2]; #garra apretar
		dUp = msg.buttons[13]; #muneca arriba
		dDwn = msg.buttons[14]; #muneca abajo
		Rsv = msg.axes[4]; #pitch
		Rsh = msg.axes[3]; #yaw
		d = 0.15 #delta error
		if y == 1 or x == 1:
			rospy.loginfo("Garra")
			self.uint.data[0] = self.uint.data[0] + 10*x - 10*y
		elif dUp == 1 or dDwn == 1:
			rospy.loginfo("Muneca")
			self.uint.data[1] = self.uint.data[1] + 10*dUp -10*dDwn
		elif d < Rsv or Rsv < -d:
			rospy.loginfo("Pitch")
			self.uint.data[2] = self.uint.data[2] + 5*Rsv
			self.uint.data[3] = 180 - self.uint.data[2]
		elif d < Rsh or Rsh < -d:
			rospy.loginfo("Yaw")
			self.uint.data[4] = self.uint.data[4] + 5*Rsh
		elif back == 1:
			self.uint = '{data: [36, 72, 108, 144, 180]}' ###RECORDAR EDITAR POSICION INICIAL
		else:
			rospy.loginfo("Quieto")

		self.pub.publish(self.uint)


def main():
	rospy.init_node('template') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers

if __name__ =='__main__':
	main()
