#!/usr/bin/env python
from cv_bridge import CvBridge
import cv2
import rospy
from sensor_msgs.msg import Image
from image_service.srv import ImageService, ImageServiceResponse

class ImageServer:
    def __init__(self):
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.image_service_server()

    def get_image(self, req):
        print("Sending image")
        cap = cv2.VideoCapture(0)
        _, frame = cap.read()
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding='passthrough')
        return ImageServiceResponse(image_message)

    def image_service_server(self):
        rospy.init_node('image_service_server')
        s = rospy.Service('get_image', ImageService, self.get_image)
        print('Ready to send image')
        rospy.spin()

if __name__ == "__main__":
    img_service = ImageServer()
