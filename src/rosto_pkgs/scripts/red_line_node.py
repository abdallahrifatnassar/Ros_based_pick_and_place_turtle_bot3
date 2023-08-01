
#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovariance, Vector3
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
#Visual obstacle message
Vobs = Float32MultiArray()
from math import cos, sin, atan2
from numpy import array, where, ones, round_, copy, int8
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point32
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

#Pose vector "initially empty"
Pose = []


class image_line_det(object):

	def pose_callback(self,msg):
		global Pose
		
		#updates the pose
		Pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, 2*atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)]

	def __init__(self):
		
		#initializing publishers and subscribers
		self.bridge_object = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
		self.pub_img_proj = rospy.Publisher('/image_out', Image) #our output
		rospy.Subscriber('/odom',Odometry,self.pose_callback)
		self.pointcloud_publisher = rospy.Publisher("/my_pointcloud_topic", PointCloud2)
		self.pub_img_warp_proj = rospy.Publisher('/image_out_warp', Image) #our output
		rospy.sleep(0.5)

	
	def camera_callback(self,data):

		try:
			# We select bgr8 because its the OpneCV encoding by default
			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
			cv_image_orig=cv_image
			orig_image_size = cv_image.shape[::-1][1:]
			width = orig_image_size[0]    
			height = orig_image_size[1]    
			padding = int(0.3* width) # padding from side of the image in pixels

			desired_roi_points = np.float32([[padding, 0], # Top-left corner
			[padding, orig_image_size[1]], # Bottom-left corner         
			[orig_image_size[0]-padding, orig_image_size[1]], # Bottom-right corner
			[orig_image_size[0]-padding, 0] # Top-right corner
			])
			roi_points= np.float32([(0,320), # Top-left corner 
			(0, 480), # Bottom-left corner                 
			(640,480), # Bottom-right corner     
			(640,320) # Top-right corner   
			 ])
			print(cv_image.shape[::-1][1:])
			print("height:" , height)
		except CvBridgeError as e:
			print(e)

		# Convert from RGB to HSV
		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		# Define the red Colour in HSV ########ours
		#RGB
		#[[[255,52,6]]]
		#BGR
		#[[[6,52,255]]]
		#HSV
		#[[[6,249,255]]]
		# lower_red = np.array([0,100,100])
		lower_red = np.array([0,50,50])
		upper_red = np.array([15,255,255])

		# Threshold the HSV image to get only red colors
		mask = cv2.inRange(hsv, lower_red, upper_red)

		# Bitwise-AND mask and original image
		res = cv2.bitwise_and(cv_image,cv_image, mask= mask)
		cv2.imshow("Input image", cv_image)
		cv2.imshow("Image with red lines only", res)
		print(type(res), np.shape(res))

		im = np.array(res.data,dtype=np.uint8)
		self.pub_img_proj.publish(self.bridge_object.cv2_to_imgmsg(res, encoding="bgr8")) ### publishing red lines only


		# Calculate the transformation matrix
		transformation_matrix = cv2.getPerspectiveTransform(roi_points,desired_roi_points )
		# Perform the transform using the transformation matrix
		warped_frame = cv2.warpPerspective(res, transformation_matrix, orig_image_size,flags=(cv2.INTER_LINEAR))
		cv2.imshow("Bird's eye view of the red lines", warped_frame)
		self.pub_img_warp_proj.publish(self.bridge_object.cv2_to_imgmsg(warped_frame, encoding="bgr8")) ### publishing warped (bird's eye view) of the red lines 
		cv2.waitKey(1)



		#Getting the indices of red pixels in the wareped image
		Red = warped_frame[:,:,2]
		print(np.unique(Red))
		obstacles = np.transpose(np.array(np.where( (Red>0) & (Red<=255))))
		
		#Getting coordinates of red line (obstacles) relative to robot frame in meters
		obstacles = obstacles.astype(float)
		obstacles[:,0] = 0.54 - (obstacles[:,0]*0.0004)
		# obs[:,1] = -(obs[:,1]-500)*0.00032
		obstacles[:,1] = -(obstacles[:,1]-200)*0.00032

		
		#Getting coordinates of red line (obstacles) in meters relative to gazebo origin using a transformation matrix
		if Pose != []:
			#Applying rotation
			rot_matrix = np.array([[cos(Pose[2]),-sin(Pose[2])],[sin(Pose[2]),cos(Pose[2])]])
			obstacles[:] = obstacles.dot(rot_matrix.T)
			
		 	#Applying translation
			obstacles[:,0] = obstacles[:,0] + Pose[0]
			obstacles[:,1] = obstacles[:,1] + Pose[1]
			print ("shape of obs array before z",np.shape(obstacles))

			#Adding z=0 to formulate pointcloud message (x,y,z)
			temp_array = np.zeros((np.shape(obstacles)[0],1), dtype=float)
			point_cloud_1channel=np.append(obstacles, temp_array, axis=1)

			print ("shape of obs array after z",np.shape(obstacles))
			print ("shape of obs array after z",np.shape(point_cloud_1channel))

			print(obstacles)
			print(point_cloud_1channel)
			print("done")


			#filling pointcloud header
			header = std_msgs.msg.Header()
			header.stamp = rospy.Time.now()
			header.frame_id = 'map'

			#filling pointcloud2 points
			list_temp_points=[]
			if len(point_cloud_1channel):
				for i in range(np.shape(obstacles)[0]):
					# list_point=point_cloud_1channel[i].tolist()

					list_temp_points.append(point_cloud_1channel[i])
					

			scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, list_temp_points)
			self.pointcloud_publisher.publish(scaled_polygon_pcl)### publishing pointcloud2 points of the red line obstacles


	def clean_up(self):
		# self.moverobot_object.clean_class()
		cv2.destroyAllWindows()


def main():
	rospy.init_node('red_line_node', anonymous=True)
	image_line_det1 = image_line_det()
	rate = rospy.Rate(5)
	ctrl_c = False

	def shutdownhook():
	# works better than the rospy.is_shut_down()
		image_line_det1.clean_up()
		rospy.loginfo("shutdown time!")
		ctrl_c = True

	rospy.on_shutdown(shutdownhook)

	while not ctrl_c:
		rate.sleep()

if __name__ == '__main__':
	main()
	

