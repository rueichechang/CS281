#!/usr/bin/env python

import rospy,math # module for ROS APIs
from geometry_msgs.msg import Twist # message type for velocity command.
from sensor_msgs.msg import LaserScan # message type for laser measurement.
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Constants.
FREQUENCY = 10 #Hz.
VELOCITY = 1 #m/s
DURATION = 5 #s how long the message should be published.

KP = 2
roll = pitch = yaw = 0.0

ROTATE_POSITION = [0,0,0]
PIPELINE = ["R90", "T2", "R-22.5", "T1.53073372946", "R-90","T2.82842712475", "R-157.5", "T1.53073372946", "R90", "T2"]
CURRENT_POSITION = [0,0,0]

class Test:
	def __init__(self):
		rospy.init_node("goforward_node")

		# self.subscriber = rospy.Subscriber("base_scan", LaserScan, self.laserscan_callback)
		self.subscriber = rospy.Subscriber("odom", Odometry, self.odometry)

		self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1) 
		rospy.sleep(2)

	def laserscan_callback(self, laserscan_msg):
		print laserscan_msg

	def distance(self, point, origin = [0,0,0]):
		(origin_x, origin_y, origin_z) = origin
		if point:
			x = point[0] - origin_x
			y = point[1] - origin_y
			z = point[2] - origin_z
			return (x*x + y*y + z*z) ** 0.5
		else: return 0
		
	def odometry(self, msg):
		global roll, pitch, yaw, CURRENT_POSITION
		position = msg.pose.pose.position
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		CURRENT_POSITION = [position.x, position.y, position.z]
		#print CURRENT_POSITION

	def main(self):
		rate = rospy.Rate(FREQUENCY)
		start_time = rospy.get_rostime()
		vel_msg = Twist()

		while not rospy.is_shutdown() and len(PIPELINE) > 0:
			ACTION = PIPELINE[0][0]
			DEGREE = PIPELINE[0][1:]
			if DEGREE[0] == '-':
				DEGREE = -1 * float(DEGREE[1:])
			else: DEGREE = float(DEGREE)
	
			if ACTION == "R":
				target_rad = DEGREE * math.pi / 180
				if round(target_rad - yaw , 2) == 0: 
					PIPELINE.pop(0)
					ROTATE_POSITION = CURRENT_POSITION
				vel_msg.linear.x = 0
				vel_msg.angular.z = KP * (target_rad - yaw)
				self.publisher.publish(vel_msg)

			elif ACTION == "T":
				distance = self.distance(CURRENT_POSITION, ROTATE_POSITION)
				if round(distance,1) == round(DEGREE,1):
					PIPELINE.pop(0)
				else:
					vel_msg.linear.x = VELOCITY
					self.publisher.publish(vel_msg)

			rate.sleep()

if __name__ == "__main__":
	t = Test()
	t.main()

