from red_msgs.srv import *
from sensor_msgs.msg import Image as Sensor_Image
import rospy
import cv2
import numpy as np
import rospkg
import os
from PIL import Image
from base64 import decodestring
from scipy.spatial.transform import Rotation as R
from utils.markerDetector import detect_markers, calculate_camera_pose
from utils.voxelsegmentation import segmentation
from cv_bridge import CvBridge

class ImageSaver:

    def __init__(self) -> None:

        self.get_state_server = rospy.Service(
            '/marker_detector/save_image', ImageData, self.callback_save_image)
        self.get_ar_image = rospy.Service(
            '/marker_detector/save_ar_image', ImageData, self.callback_save_ar_image)
        self.process_ar_image = rospy.Service(
            '/marker_detector/process_ar_image', ArucoImage, self.callback_process_ar_image
        )

        self.camera_subscriber = rospy.Subscriber(
            "/camera/color/image_raw", Sensor_Image, self.callback_new_image_from_camera)
        
        


        self.images = []
        self.header = ["data","pose"]
        self.first_camera_pose = {}
        self.current_image = {}
        self.current_image_index = 1

    def callback_new_image_from_camera(self, msg):
            self.current_image["data"] = msg.data
            self.current_image["encoding"] = msg.encoding
            self.current_image["height"] = msg.height
            self.current_image["width"] = msg.width

    def callback_save_ar_image(self, req):
        rospack = rospkg.RosPack()
        rospy.loginfo("service called")
        image_directory_path = rospack.get_path('voxel_carving')+"/../ar_images/"
        #image_directory_path = rospack.get_path('voxel_carving')+"/images/"
        directory = image_directory_path+req.file
        if not os.path.exists(directory):
            os.makedirs(directory)
        path_of_image = directory+f"/image_{self.current_image_index}"
        image = Image.frombytes("RGB", (1280, 720), self.current_image["data"])
        image.save(path_of_image, "PNG")

        image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        _, markerCorners, markerIds = detect_markers(image.copy())
        _, cameraPose, worldPoints = calculate_camera_pose(markerCorners,markerIds, image.copy())

        with open(directory+"/poses.txt", mode='a') as file:
            file.write(str(list(cameraPose.flatten()))+"\n")

        self.current_image_index+=1
        return ImageDataResponse()        
        
    def callback_save_image(self,req):
        # obtain first camera pose
        print(type(req.data))
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('voxel_carving')+"/../data/"+req.file+".txt"
        image_directory_path = rospack.get_path('voxel_carving')+"/images/"
        

        if not os.path.exists(file_path) or os.path.getsize(file_path) == 0:
            # File is not existent or empty
            with open(file_path, mode='w') as file:
                print("file empty")
                row_count=1
                #csv_writer = csv.writer(csv_file)
                self.first_camera_pose[str(req.file)] = np.reshape(req.transform, (4, 4), order='F')
                #csv_writer.writerow(self.header)

        else:
            # File is not empty
            with open(file_path, mode='r') as file:
                print("file not empty")
                row_count=1
                for line in file.readlines():
                    row_count+=1


        with open(file_path, mode='a') as file:
            # calculate camera pose relativo to first camera pose
            rotation = R.from_euler("xyz",[180,0,0],degrees=True).as_matrix()
            end_effector_to_camera = np.array([[1, 0, 0, 0.01, 0, 1, 0, 0.02, 0, 0, 1, 0.039978, 0, 0, 0, 1]]).reshape(4,4)
            end_effector_to_camera[:3,:3] = rotation
            end_effector_pose = np.reshape(req.transform, (4 , 4), order='F')
            camera_pose = end_effector_pose @ end_effector_to_camera
            #camera_pose = end_effector_pose 
            #rotation = R.from_euler("xyz",[180,0,0],degrees=True).as_matrix()
            #camera_pose[:3,:3] = camera_pose[:3,:3] @ rotation

            
            # save image
            directory = image_directory_path+req.file
            if not os.path.exists(directory):
                os.makedirs(directory)

            file.write(str(list(camera_pose.flatten()))+"\n")
              
        return ImageDataResponse()

    def callback_process_ar_image(self,req):

        rospy.loginfo("process ar image service called")
        image = Image.frombytes("RGB", (1280, 720), self.current_image["data"])
        image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        _, markerCorners, markerIds = detect_markers(image.copy())
        _, cameraPose, worldPoints = calculate_camera_pose(markerCorners,markerIds, image.copy())
        
        #segmented_image = segmentation(image, cameraPose, )
        
        response = ArucoImageResponse()
        response.image = np.array(image).tobytes()
        response.segmented_image = np.array(segmented_image).tobytes()
        response.transform = cameraPose.flatten()
        return response

if __name__ == "__main__":
    rospy.init_node('image_saver')
    rospy.loginfo("image saver spinning")
    s = ImageSaver()
    rospy.spin()
    