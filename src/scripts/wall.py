#!/usr/bin/env python
#from __future__ import print_function, division
import rospy,math,time, cv2
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan,CameraInfo, Image 
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# CV Initialization

# Constants:
# -----------------------------------------------------------------
FREQUENCY = 20 #Hz.
VELOCITY = 0.3 #m/s
SLEEP = 2
TOLERANCE = 0.2

KP = 5 # propotional gain
KD = 10 # derivative gain

DISTANCE_REF = 1.0 # the reference that the robot need to keep from the wall
DISTANCE_TO_AVOID = DISTANCE_REF * 0.8 # the reference that robot need to stop and turn out
DISTANCE_SLOW_DOWN = DISTANCE_TO_AVOID + 0.2 # the reference that robot need to slow down itself
INSPECT_ANGLE = math.pi / 90
TURNOUT_ANGLE = math.pi/4
TURNIN_ANGLE = math.pi/4
TURN_RATE = 0.8
TURNIN_RATIO = 1.4

# -----------------------------------------------------------------

class State:
	INITIALIZATION 		= 0 # state before checking which wall to follow
	FOLLOWING 			= 1 # following the wall to PID controlled without following circumstances
	STOP_AND_TURNOUT 	= 2 # stop and turnout if the wall is front of the robot
	TURN_IN 			= 3 # turn in if there is a road branch
	SLOW_DOWN 			= 4 # slow down if robot is approaching the wall
	ROTATE_OUT 			= 5 # rotate out if the distance between the robot and the wall is too close
	STRAIGHT 			= 6 # go for the direction parrallel to the wall 			
	
class Wall:
	RIGHT 	= 0 # following right wall
	LEFT 	= 1 # following left wall
	NOT_YET = 2 # not yet decided the wall to follow
	
class Follower:
	def __init__(self):
		rospy.init_node("following_wall")
		#self.laserscan_sub = rospy.Subscriber("scan", LaserScan, self.laserscan_callback)
		#self.cameraInfo_sub = rospy.Subscriber("camera/rgb/camera_info", CameraInfo, self.cameraInfo_callback)
		self.camera_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_callback)
		self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
		self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1) 

		self.bridge = CvBridge()
		self.state = State.INITIALIZATION
		self.scanMsgs = None
		self.wallFollowing = Wall.NOT_YET
		self.errors = [0]

		rospy.sleep(SLEEP)

	def laserscan_callback(self, laserscan_msg):
		print("lasercan good")
		print laserscan_msg
		
	def cameraInfo_callback(self, cameraInfo_msg):
		print cameraInfo_msg
		
	def camera_callback(self, image):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
		except CvBridgeError as e:
			print(e)
		
		(rows,cols,channels) = cv_image.shape
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)
		
	def odom_callback(self, msg):
		return
		
	# a mapping function to retrieve data of specific angle from the lasercan.msg.ranges
	def oneDmapping(self, value, min_old, max_old, min_new, max_new):
		return min_new + (value - min_old) * (max_new - min_new) / (max_old - min_old)
	# get the range given start and end radian, and tackle the condition of the right wall
	def range_transform(self,min_radian, max_raidan):
		if self.scanMsgs is not None:
			scan = self.scanMsgs

			if self.wallFollowing == Wall.RIGHT:
				min_radian, max_raidan = -min_radian, -max_raidan

			if min_radian > max_raidan:
				min_radian, max_raidan = max_raidan, min_radian
				
			if min_radian < scan.angle_min: min_radian = scan.angle_min 	
			if max_raidan > scan.angle_max: max_raidan = scan.angle_max 
				
			start = int(self.oneDmapping(min_radian, scan.angle_min, scan.angle_max, 0, len(scan.ranges) - 1))
			end = int(self.oneDmapping(max_raidan, scan.angle_min, scan.angle_max, 0, len(scan.ranges) - 1))
			return start, end
		else:
			return 0,0
	# to decide the closer wall to follow in the initial
	def update_wall_to_follow(self):
		if self.wallFollowing is Wall.NOT_YET:
			distance_to_left = self.min_distance_to_wall(0,math.pi)
			distance_to_right = self.min_distance_to_wall(0,-math.pi)

			self.wallFollowing = Wall.RIGHT if distance_to_left > distance_to_right else Wall.LEFT
			self.state = State.FOLLOWING
			print(distance_to_left, distance_to_right, self.wallFollowing)
		else: 
			return 0
	
	# to get the minimal distance to the wall with a specific range of sector, given min_radian, max_raidan
	def min_distance_to_wall(self, min_radian, max_raidan):
		if self.scanMsgs is not None:
			scan = self.scanMsgs
			start, end = self.range_transform(min_radian, max_raidan)
			return min(scan.ranges[start:end])
		else: return 0
		
	# to get average distance of a given sector
	def get_avg_distance_in_range(self, min_radian, max_raidan):
		if self.scanMsgs is not None:
			scan = self.scanMsgs
			start, end = self.range_transform(min_radian, max_raidan)
			temp = scan.ranges[start:end]
			return sum(temp) / len(temp)
		else: return 0
		
	def computeState(self):
		scan = self.scanMsgs
		self.update_wall_to_follow()
		
		avg_distance = self.get_avg_distance_in_range(-math.pi, math.pi)
		min_distance = self.min_distance_to_wall(-math.pi, math.pi)
		front_distance = self.min_distance_to_wall(-math.pi / 60, math.pi / 60)
		side_front_distance = self.min_distance_to_wall(math.pi / 4, math.pi / 2)
		side_back_distance = self.min_distance_to_wall(math.pi / 2 , math.pi * 3 / 4)
			
		if front_distance <= DISTANCE_TO_AVOID:
			self.state = State.STOP_AND_TURNOUT
		
		elif side_front_distance >= TURNIN_RATIO * side_back_distance:
			self.state = State.TURN_IN
		
		elif side_front_distance <= DISTANCE_TO_AVOID:
			self.state = State.ROTATE_OUT
			
		elif front_distance <= DISTANCE_SLOW_DOWN or side_back_distance <= DISTANCE_TO_AVOID:
			self.state = State.SLOW_DOWN
			
		elif DISTANCE_REF - TOLERANCE < avg_distance < DISTANCE_REF + TOLERANCE:
			self.state = State.STRAIGHT
		
		else:
			self.state = State.FOLLOWING
		
		return self.state

	def computeVW(self):
		state = self.computeState()

		(V , W) = (0.0, 0.0)
		if self.state == State.INITIALIZATION: return (V,W)

		distToWall = self.min_distance_to_wall(math.pi / 2 - INSPECT_ANGLE, math.pi / 2 + INSPECT_ANGLE)
		error = distToWall - DISTANCE_REF
		delta_error = error - self.errors[-1]
		self.errors.append(error)

		W = KP * error + KD * delta_error
		V = VELOCITY
		
		OPPOSITE = -1
		if self.wallFollowing == Wall.RIGHT:
			W *= OPPOSITE
		
		if state == State.STOP_AND_TURNOUT:
			W = - TURNOUT_ANGLE * OPPOSITE
			V = 0.0
		
		elif state == State.SLOW_DOWN:
			V /= 4 
			W = 0
		
		elif state == State.TURN_IN:
			W += OPPOSITE * TURNIN_ANGLE * 2
		
		elif state == State.ROTATE_OUT:
			W *= TURN_RATE
			
		elif state == State.STRAIGHT:
			W = 0
			
		return (V,W)

	def main(self):
		rate = rospy.Rate(FREQUENCY)
		start_time = rospy.get_rostime()
		vel_msg = Twist()

		while not rospy.is_shutdown():
			if self.scanMsgs is not None:
				(vel_msg.linear.x, vel_msg.angular.z) = self.computeVW()
				self.publisher.publish(vel_msg)
				rate.sleep()

if __name__ == "__main__":
	newRobot = Follower()
	newRobot.main()


