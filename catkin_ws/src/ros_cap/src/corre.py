#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32, Float32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from duckietown_msgs.msg import Twist2DStamped

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber('/duckiebot/joy', Joy, self.callback)
		self.pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=0)			
		self.twist = Twist2DStamped()
	
	def callback(self,msg):
		if msg.buttons[1]==1:
			rospy.loginfo("Emergencia")
			self.twist.v = 0
			self.twist.omega = 0
		else:
			if msg.axes[5] <= 0.2 or msg.axes[0] <= -0.3 or 0.3 <= msg.axes[0]:
				rospy.loginfo("Avanza/Vira")
				self.twist.v = msg.axes[5]*0.5-0.5
				self.twist.omega = msg.axes[0]*12
			elif msg.axes[2] <= 0.35 or msg.axes[0] <= -0.3 or 0.3 <= msg.axes[0]:
				rospy.loginfo("Avanza/Vira")
				self.twist.v = -msg.axes[2]*0.5+0.5
				self.twist.omega = msg.axes[0]*12	
			else:
				rospy.loginfo("Deteni'o")
				self.twist.v = 0
				self.twist.omega = 0
		self.pub.publish(self.twist)

def main():
	rospy.init_node('template') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers

if __name__ =='__main__':
	main()
