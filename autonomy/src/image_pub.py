import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class image_pub():
    def __init__(self):
        rospy.init_node('image_pub', anonymous=True)
        self.bridge = CvBridge()
        self.img = cv2.imread('~/frame0000.jpg')
        self.img_pub = rospy.Publisher("/usb_cam/image_raw",Image,queue_size=5)
        ros_img = self.bridge.cv2_to_imgmsg(self.img, "bgr8")
        for i in range(0,5):
            self.img_pub.publish(ros_img)
            rospy.sleep(0.1)
            
if __name__ == '__main__':
    image_pub()
    



