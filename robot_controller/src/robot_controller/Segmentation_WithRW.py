#!/usr/bin/env python
from scipy.spatial.transform import Rotation as R
import cv2
import numpy as np
import rospy
from sklearn import preprocessing
import math
import scipy

from image_service.srv import ImageService
from robot_controller import utilities as utils

from tf2_msgs.msg import TFMessage


class Segmentation():
    def __init__(self):
        self.color_value = 51/2 #[38,45,48] # From side view

        # For getting image of pi camera through ROS.
        #rospy.init_node("image_service_client")
        rospy.wait_for_service("get_image")
        self.proxy = rospy.ServiceProxy("get_image", ImageService)

        #Set up a ros subscriper on the transform node
        self.transform = np.eye(4)
        rospy.Subscriber("tf", TFMessage, self.update_transform)

        #Constant transform from tcp to camera
        self.camera_transform = np.eye(4)
        self.camera_transform[0:3,0:3] = R.as_matrix(R.from_euler('xyz', [0,180,0], degrees=True).__mul__(R.from_euler('xyz', [0,0,-90], degrees=True)))
        #self.camera_transform[0,3] = 0.094
        #self.camera_transform[1,3] = -0.024
        #self.camera_transform[2, 3] = 0.041485

        # Constant tranform from base of robot to table
        transM = [0.13054, -0.24411, -0.03447, 0., 0., -1.17658]
        r = R.from_euler("xyz", transM[3:]).as_matrix()
        T = np.asarray([[1, 0, 0, transM[0]], [0, 1, 0, transM[1]], [0, 0, 1, transM[2]], [0, 0, 0, 1]])
        T[0:3, 0:3] = r
        self.base_to_table_transform = T

    def update_transform(self, data):
        translation = data.transforms[0].transform.translation
        quat = data.transforms[0].transform.rotation
        self.transform[0:3, 3] = [translation.x, translation.y, translation.z]
        self.transform[0:3, 0:3] = R.as_matrix(R.from_quat([quat.x, quat.y, quat.z, quat.w]))
        #print("Transform:", self.transform)

    def get_image(self):
        try:
            image_message = self.proxy().img
            return cv2.cvtColor(np.frombuffer(image_message.data, dtype=np.uint8).reshape(image_message.height, image_message.width, -1), cv2.COLOR_BGR2HSV)
        except rospy.ServiceException as e:
            raise ValueError(e)

    def segment_slab(self, labels, distances, cluster , centers):
        label_ = []
        for label in labels:
            if label == cluster:
                label_.append(label)
            else:
                label_.append(distances[-1])

        centers[cluster] = np.asarray([255, 255, 255] + centers[cluster][3:].tolist())
        centers[distances[-1]] = np.asarray([0, 0, 0] + centers[distances[-1]][3:].tolist())
        return np.array(label_).flatten(), centers    

    def segment_and_threshold_image(self, image):
        original_image_shape = image.shape

        # Creates image data, with 5x(image.width*image.height), as [[R, G, B, x, y], ...]
        lst = []
        for row in range(image.shape[0]):
            for column in range(image.shape[1]):
                lst.append(image[row][column].tolist()+[row,column])
        lst = np.float32(np.asarray(lst))
        #lst[:,0:3] = lst[:,0:3]
        lst_norm = preprocessing.normalize(lst,axis = 0)

        #Implement silhouette       
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.2)
        k = 10 #12 #8
        _, labels, (centers) = cv2.kmeans(lst_norm, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        labels = labels.flatten()

        not_norm_centers = np.asarray([np.average(lst[labels==i,:], axis = 0).tolist() for i in range(len(centers))])

        # convert back to 8 bit values
        centers = np.uint8(centers)
        distances = utils.find_smallest_distances(not_norm_centers, self.color_value)
        labels_, not_norm_centers_ = self.segment_slab(labels, distances, distances[0], not_norm_centers)

        segmented_image = np.asarray(not_norm_centers_[:,0:3][labels_].reshape(original_image_shape), dtype=np.uint8)
        grayscale_image = cv2.cvtColor(cv2.cvtColor(segmented_image,cv2.COLOR_HSV2RGB), cv2.COLOR_RGB2GRAY)
        ret, binary_image = cv2.threshold(grayscale_image, 128, 255, cv2.THRESH_BINARY) #Binaries it before returning

        return binary_image
    
    def find_contours(self, image):
        contours, hierarchy = cv2.findContours(image, 1, 2)
        maxSizeCnt = 0
        temp = 0
        # Finding the contour with the largest area.
        for i in contours:
            if(cv2.contourArea(i)>maxSizeCnt):
                maxSizeCnt = cv2.contourArea(i)
                temp = i
        return np.int0(cv2.boxPoints(cv2.minAreaRect(temp))), temp

    def find_middle_side_points(self,box):
        points = []
        for i in range(len(box)):
            if(i==len(box)-1):
                points.append((box[i]+box[0])/2)
            else:
                points.append((box[i]+box[i+1])/2)
        return points

    def find_center(self,cnt):
           M = cv2.moments(cnt)
           cX = int(M["m10"]/M["m00"])
           cY = int(M["m01"]/M["m00"])
           return [cX,cY]

    def find_points_for_robot(self, points, middlePoint):
        robotPoints = []
        i = 0
        for sidePoint in points:
            if(i==1 or i==3):
                directionVectorX = (sidePoint[0]-middlePoint[0])*4.5+middlePoint[0]
                directionVectorY = ((middlePoint[1]-sidePoint[1])/(middlePoint[0]-sidePoint[0]))*(directionVectorX-middlePoint[0])+middlePoint[1]
                robotPoints.append([directionVectorX,directionVectorY])
            else:
                directionVectorX = (sidePoint[0]-middlePoint[0])*3+middlePoint[0]
                directionVectorY = ((middlePoint[1]-sidePoint[1])/(middlePoint[0]-sidePoint[0]))*(directionVectorX-middlePoint[0])+middlePoint[1]
                robotPoints.append([directionVectorX,directionVectorY])
        return robotPoints

    def pixel_to_realworld(self, image, point):
        focal_length = 5.262945337109372e+02#3.04*0.00112
        u0 = image.shape[0]/2
        v0 = image.shape[1]/2
        cam_to_table_transform = self.camera_transform@self.transform@self.base_to_table_transform
        z = -cam_to_table_transform[2,3] - 0.013
        X = ((point[0]-u0)/focal_length)*z
        Y = ((point[1]-v0)/focal_length)*z
        return [X,Y,z]

    def findSizeOfSlab(self,box,image):
        colors = [(255,0,0),(0,0,255),(0,0,0),(0,255,0)]
        names = ["red","blue","black","green"]
        realWorldPoses = []
        for point in box:
            realWorldPoses.append(self.pixel_to_realworld(image, point))

        #print(realWorldPoses)
        for i in range(len(realWorldPoses)):
            if(i==len(realWorldPoses)-1):
                print("Length of side marked green", math.sqrt((realWorldPoses[i][0]-realWorldPoses[0][0])**2+(realWorldPoses[i][1]-realWorldPoses[0][1])**2))
                cv2.line(image,(int(box[i][0]),int(box[i][1])),(int(box[0][0]),int(box[0][1])),color=(0,255,0),thickness=4)
            else:
                print("Length of side marked "+names[i], math.sqrt((realWorldPoses[i][0]-realWorldPoses[i+1][0])**2+(realWorldPoses[i][1]-realWorldPoses[i+1][1])**2))
                cv2.line(image,(int(box[i][0]),int(box[i][1])),(int(box[i+1][0]),int(box[i+1][1])),color=colors[i],thickness=4)

    def transform_in_relation_camera_to_table(self, points):
        transformed_points = []
        for point in points:
            point = point + [1]
            new_point = self.transform@self.camera_transform@point
            transformed_points.append(list(new_point[0:-1]))
        return transformed_points

    def run(self):
        image = self.get_image()
        image_no_hsv = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
        utils.show_image("Original Image", image_no_hsv)

        segmented_image = self.segment_and_threshold_image(image)
        segmented_image = cv2.dilate(segmented_image, None, iterations=1)
        segmented_image = cv2.erode(segmented_image, None, iterations=1)
        utils.show_image("Segmented Image", segmented_image)

        box, cnt = self.find_contours(segmented_image)
        middlePoint = self.find_center(cnt)
        sidePoints = self.find_middle_side_points(box)
        self.findSizeOfSlab(box, image_no_hsv)
        robotSidePoints = self.find_points_for_robot(sidePoints, middlePoint)


        cv2.circle(image_no_hsv, (int(middlePoint[0]), int(middlePoint[1])), radius=3, color=(0,0,255), thickness=-1)
        for sp, rsp in zip(sidePoints, robotSidePoints):
            cv2.circle(image_no_hsv, (int(sp[0]), int(sp[1])), radius=3, color=(0,255,0), thickness=-1)
            cv2.circle(image_no_hsv, (int(rsp[0]), int(rsp[1])), radius=3, color=(255,0,0), thickness=-1)

        utils.show_image("Box", image_no_hsv)

        real_world_box_points = []
        for box_point in box:
            real_world_box_points.append(self.pixel_to_realworld(image, box_point)) #In relation to camera.

        print("Pixel positions: ", box)
        print("Relative to camera: ", real_world_box_points)
        print("Relative to base: ")
        real_world_box_points_relation_table = self.transform_in_relation_camera_to_table(real_world_box_points)
        print("-"*33)
        for i, pnt in enumerate(real_world_box_points_relation_table):
            print("Box point ", i, pnt)
        print("-"*33)

        cv2.waitKey(0)
        return np.asarray(real_world_box_points_relation_table)


if __name__ == "__main__":
    segmentation = Segmentation()
    segmentation.run()
