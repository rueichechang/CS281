#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist # message type for velocity command.
from sensor_msgs.msg import LaserScan, CameraInfo, Image # message type for laser measurement.
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy,math,cv2 # module for ROS APIs
import numpy as np

# Constants.
FREQUENCY = 10 #Hz.
VELOCITY = 0 #m/s
ANGULARZ = 0
DURATION = 5 #s how long the message should be published.

KP = 2
roll = pitch = yaw = 0.0

ROTATE_POSITION = [0,0,0]
PIPELINE = ["R90", "T2", "R-22.5", "T1.53073372946", "R-90","T2.82842712475", "R-157.5", "T1.53073372946", "R90", "T2"]
CURRENT_POSITION = [0,0,0]

light_red = (0, 0, 255)
dark_red = (0, 0, 200)
AREA_STANDARD = 14000
V_CONSTANT = 0.00001
W_CONSTANT = 0.001

image_width = 640
image_height = 480

class Follower:
	def __init__(self):
		rospy.init_node("follower")

		#self.laserscan_sub = rospy.Subscriber("scan", LaserScan, self.laserscan_callback)
		#self.cameraInfo_sub = rospy.Subscriber("camera/rgb/camera_info", CameraInfo, self.cameraInfo_callback)
		self.camera_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_callback)
		self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

		self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1) 
		self.bridge = CvBridge()
		
		rospy.sleep(2)
		
	def odom_callback (self, msg):
		return
		
	def laserscan_callback(self, laserscan_msg):
		return
		
	def cameraInfo_callback(self, cameraInfo_msg):
		print (cameraInfo_msg)
		
	def camera_callback(self, image):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		(rows,cols,channels) = cv_image.shape
		blue,green,red = cv2.split(cv_image)
		_,output = cv2.threshold(red, 250, 255, cv2.THRESH_BINARY)
		print(output.shape)
		_,cnts, _ = cv2.findContours(output.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		#cv2.drawContours(cv_image, cnts, -1, (0, 255, 255), 5)
	
		area,center = self.get_max_cnt_area(cnts)
		#cv2.circle(cv_image, center, 10, (255,255,0), 10)
		if area != 0 and center != 0:
			global VELOCITY, ANGULARZ
			VELOCITY, ANGULARZ = self.computeVW(area,center)
			
		cv2.imshow("reference", cv_image)
		cv2.imshow("output", output)

		cv2.waitKey(3)
	
	def computeVW (self, area, center):
		V = (area - AREA_STANDARD) * (-1)* V_CONSTANT
		W = (center[0] - image_width/2) * W_CONSTANT
		return V, W
		
	def get_max_cnt_area(self, cnts):
		try:
			area = 0
			cnt = []
			for i in range(len(cnts)):
				temp = cv2.contourArea(cnts[i])
				if temp > area: 
					area = temp
					cnt = cnts[i]
			cnt = np.array(cnt)
			center = cnt.mean(axis=0)[0]
			center = (int(center[0]),int(center[1]))
		except:
			return 0,0
		return area, center
		
	def main(self):
		rate = rospy.Rate(FREQUENCY)
		start_time = rospy.get_rostime()
		vel_msg = Twist()

		while not rospy.is_shutdown():
			print(VELOCITY,ANGULARZ)
			vel_msg.linear.x = VELOCITY
			vel_msg.angular.z = ANGULARZ
			self.publisher.publish(vel_msg)
			rate.sleep()

if __name__ == "__main__":
	follower = Follower()
	follower.main()

