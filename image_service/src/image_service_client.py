#!/usr/bin/env python
from cv_bridge import CvBridge
import cv2
import rospy
from image_service.srv import ImageService

class Image_client:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.wait_for_service('get_image')
        self.proxy = rospy.ServiceProxy('get_image', ImageService)

    def image_service_client(self):
        try:
            image_message = self.proxy().img
            cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
            return cv_image
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

if __name__ == '__main__':
    #rospy.init_node('image_service_client')
    image_client = Image_client()
    cv2.namedWindow('Camera image', cv2.WINDOW_AUTOSIZE)
    try:
        while True:
            print("Press e to request image")
            while True:
                if cv2.waitKey(1) & 0xFF == ord('e'):
                    break
            img = image_client.image_service_client()
            cv2.imshow('Camera image', img)
    except KeyboardInterrupt:
        print("Program closed")
    cv2.destroyAllWindows()
